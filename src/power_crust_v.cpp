#include "power_crust.h"
#include <thread>
#include "utils.h"

PowerCrust::point_radius_v PowerCrust::get_poles_to_all_voronoi_cells(std::vector<Point>& filtered_points, Box& box)
{
	auto start = hrclock::now();

	std::set<Point> box_points;
	add_box_points(box_points, filtered_points, box);

#ifdef TBB_ENABLED
	delaunay_triangulation_s T(filtered_points.begin(), filtered_points.end(), &locking_ds);
	delaunay_triangulation_s::Lock_data_structure locking_ds(CGAL::Bbox_3(box.get_xmin(), box.get_ymin(), box.get_zmin(), box.get_xmax(), box.get_ymax(), box.get_zmax()), 50);
#else
	delaunay_triangulation_s T(filtered_points.begin(), filtered_points.end());
#endif

	point_radius_s estimated_normal_endpoints[4];

	//TODO: do it better way - parrarelizm
	std::vector<delaunay_finite_vertices_iterator_s> fvi_v;
	for (delaunay_finite_vertices_iterator_s vit = T.finite_vertices_begin(); vit != T.finite_vertices_end(); vit++)
		fvi_v.push_back(vit);

	std::thread make_cell_thread1(&PowerCrust::make_voronoi_cells_threadsafe, this, T, box_points, &estimated_normal_endpoints[0], std::vector<delaunay_finite_vertices_iterator_s>(fvi_v.begin(), fvi_v.end()));
	make_cell_thread1.join();
	//--------------------------

	point_radius_v merged_normal_endpoints = point_radius_v(estimated_normal_endpoints[0].begin(), estimated_normal_endpoints[0].end());
	
	printf("Power Crust: DONE\n");
	printf("\tPoles choosed in: %u seconds\n", (uint32_t)boost::chrono::duration_cast<milliseconds>(hrclock::now() - start).count());

	return merged_normal_endpoints;
}

void PowerCrust::make_voronoi_cells_threadsafe(delaunay_triangulation_s& T, const std::set<Point>& box_points, point_radius_s* wp, std::vector<delaunay_finite_vertices_iterator_s> fvi_v)
{
	for (auto& vit : fvi_v)
	{
		if (box_points.count(vit->point()) == 1)	//its a bounding box point, dont need their cells
			continue;

		//-----------------------------------------------
		//Getting Incident cells
		std::vector<delaunay_cell_handle_s> inc_cells;
		T.incident_cells_threadsafe(vit, std::back_inserter(inc_cells));

		if (inc_cells.empty())
			continue;

		//-----------------------------------------------
		//the current voronoi cell's surface point
		Point surface_point = vit->point();
		std::vector<Point> cell_vertices;

		for (auto &cell : inc_cells)
		{
			set_dual_threadsafe(T, cell, &cell->info());
			cell_vertices.push_back(cell->info().dual);
		}

		auto estimated_normals_to_surf_p = determine_poles(surface_point, cell_vertices);
		
		//TODO if the result is bad - then we need average of weight
		std::pair<point_radius_s::iterator, bool> pole_l = wp->insert(std::pair<weighted_point, point_inf>(estimated_normals_to_surf_p.first, point_inf()));
		auto tmp = &const_cast<std::vector<Point>&>(pole_l.first->second.surface_points);
		tmp->push_back(surface_point);

		pole_l = wp->insert(std::pair<weighted_point, point_inf>(estimated_normals_to_surf_p.second, point_inf()));
		tmp = &const_cast<std::vector<Point>&>(pole_l.first->second.surface_points);
		tmp->push_back(surface_point);
	}
}

void PowerCrust::set_dual_threadsafe(delaunay_triangulation_s& T, delaunay_cell_handle_s cell, cell_inf* info)
{
	info->mux->lock();
	if (info->index == -1)
	{
		info->dual = T.dual(cell);
		info->index = 0;
	}
	info->mux->unlock();
}

std::pair<weighted_point, weighted_point> PowerCrust::determine_poles(const Point& surface_point, const std::vector<Point>& cell_vertices)
{
	std::pair<weighted_point, weighted_point> estimated_normals_to_surf_p;

	//-----------------
	std::vector<Point>::const_iterator farthest = cell_vertices.cbegin();
	std::vector<Point>::const_iterator it = cell_vertices.cbegin()++;
	float distance = CGAL::squared_distance(surface_point, *farthest);

	for (; it != cell_vertices.cend(); ++it)
	{
		float tmp_distance = CGAL::squared_distance(surface_point, *it);
		if (tmp_distance > distance)
		{
			distance = tmp_distance;
			farthest = it;
		}
	}

	estimated_normals_to_surf_p.first = weighted_point(*farthest, distance);	//regular tringulation needs square of radius

	auto sp1 = *farthest - surface_point;
	farthest = cell_vertices.cend();
	float scalar_dot = 1.0f;

	//choose the most negative product
	it = cell_vertices.cbegin();
	for (; it != cell_vertices.cend(); ++it)
	{
		auto sp2 = *it - surface_point;
		float tmp_scalar_dot = CGAL::scalar_product(sp1, sp2);
		if (scalar_dot > tmp_scalar_dot)
		{
			farthest = it;
			scalar_dot = tmp_scalar_dot;
		}
	}

	if (scalar_dot >= 0)
	{
		if (scalar_dot > 0) printf("\tERROR: COULDNT FIND second pole because the sc. dot is %.4f\n", scalar_dot);
		else printf("\tERROR: COULDNT FIND second pole because scalar dot is 0\n");
	}
	else
	{
		distance = CGAL::squared_distance(surface_point, *farthest);
		estimated_normals_to_surf_p.second = weighted_point(*farthest, distance);
	}

	return estimated_normals_to_surf_p;
}