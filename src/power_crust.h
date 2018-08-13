#pragma once
#include "io_types.h"
#include "reconstructor_utils.h"
#include "power_crust_types.h"
#include "polar_ball.h"
#include "power_diagram.h"

#include "cgal_inc.h"

class PowerCrust
{
	struct inc_ordered_wp
	{
		bool operator ()(const std::pair<weighted_point, point_inf>& lhs, const std::pair<weighted_point, point_inf>& rhs)
		{
			return lhs.first.point() < rhs.first.point();
		}
	};

	typedef std::vector<std::pair<weighted_point, point_inf>> point_radius_v;				//for triangulation - inf: related surf points of pole
	typedef std::set<std::pair<weighted_point, point_inf>, inc_ordered_wp> point_radius_s;	//for threads - finding in logn

	struct power_faces_off_data
	{
		std::map<edge*, std::set<uint32_t>> edge_cell_ids_m;	//to determine cell connections , túl sok él???
		std::vector<point_inf*> inf_ptrs;						//to determine neighbour cells

		std::vector<regular_finite_vertices_iterator> box_v_its;

		std::map<PolarBall*, std::set<PolarBall*>> neigh_graph_m;
		std::map<Point, std::vector<PolarBall*>> surf_polarballs_m;
	};

public:
	PowerCrust()
	{
	}

	export_data calc(std::vector<Point>& filtered_points, bool& oc, bool& th, Box& box = Box(-1.0f,1.0f))
	{
		point_radius_v w_p = get_poles_to_all_voronoi_cells(filtered_points, box * 2.0f);

		/*printf("The weighted points for pd:\n");
		for (auto& it : w_p)
			printf("\t%d - %d\n", it.first.point(),it.first.weight());
		*/
		
		export_data res;

		if (oc)
		{
			std::cout << "original\n";

			PowerDiagram pd;

			pd.set_box(box);
			pd.calc_diagram(w_p);
			pd.label_poles();
			pd.calc_power_crust();

			res = pd.get_triangled_mesh_without_texture();
			pd.clear();
		}
		else if(th)
		{
			std::cout << "third one\n";
			face_simplex power_crust_simplex = get_power_crust_faces(w_p, box);
			res = get_triangled_mesh_from_face_simplex(power_crust_simplex.faces);
		}
		else
		{
			std::cout << "first try\n";
			face_simplex power_crust_simplex = get_power_crust_faces(w_p, box);
			res = get_triangled_mesh_from_face_simplex(power_crust_simplex.faces);
		}
		return res;
	}

protected:

	void add_box_points(std::set<Point>& box_points, std::vector<Point>& points, const Box& box)
	{
		Point p1 = Point(box.get_xmin(), box.get_ymin(), box.get_zmin());
		Point p2 = Point(box.get_xmin(), box.get_ymin(), box.get_zmax());
		Point p3 = Point(box.get_xmin(), box.get_ymax(), box.get_zmin());
		Point p4 = Point(box.get_xmin(), box.get_ymax(), box.get_zmax());
		Point p5 = Point(box.get_xmax(), box.get_ymin(), box.get_zmin());
		Point p6 = Point(box.get_xmax(), box.get_ymin(), box.get_zmax());
		Point p7 = Point(box.get_xmax(), box.get_ymax(), box.get_zmin());
		Point p8 = Point(box.get_xmax(), box.get_ymax(), box.get_zmax());

		box_points.insert(p1); points.push_back(p1);
		box_points.insert(p2); points.push_back(p2);
		box_points.insert(p3); points.push_back(p3);
		box_points.insert(p4); points.push_back(p4);
		box_points.insert(p5); points.push_back(p5);
		box_points.insert(p6); points.push_back(p6);
		box_points.insert(p7); points.push_back(p7);
		box_points.insert(p8); points.push_back(p8);
	}

	void add_box_points(const Box& box, std::set<Point>& box_points, std::vector<std::pair<weighted_point, point_inf>>& points)
	{
		Point p1 = Point(box.get_xmin(), box.get_ymin(), box.get_zmin());
		Point p2 = Point(box.get_xmin(), box.get_ymin(), box.get_zmax());
		Point p3 = Point(box.get_xmin(), box.get_ymax(), box.get_zmin());
		Point p4 = Point(box.get_xmin(), box.get_ymax(), box.get_zmax());
		Point p5 = Point(box.get_xmax(), box.get_ymin(), box.get_zmin());
		Point p6 = Point(box.get_xmax(), box.get_ymin(), box.get_zmax());
		Point p7 = Point(box.get_xmax(), box.get_ymax(), box.get_zmin());
		Point p8 = Point(box.get_xmax(), box.get_ymax(), box.get_zmax());

		//TODO: 6 és 8 miatt ír warningot!

		box_points.insert(p1); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p1, 0.000001f), std::vector<Point>()));
		box_points.insert(p2); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p2, 0.000001f), std::vector<Point>()));
		box_points.insert(p3); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p3, 0.000001f), std::vector<Point>()));
		box_points.insert(p4); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p4, 0.000001f), std::vector<Point>()));
		box_points.insert(p5); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p5, 0.000001f), std::vector<Point>()));
		box_points.insert(p6); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p6, 0.000001f), std::vector<Point>()));
		box_points.insert(p7); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p7, 0.000001f), std::vector<Point>()));
		box_points.insert(p8); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p8, 0.000001f), std::vector<Point>()));
	}

	Box update_box(const std::vector<std::pair<weighted_point, point_inf>>& weighted_points)
	{
		Box res = Box(-1.0f, 1.0f);

		float minX = FLT_MAX, maxX = FLT_MIN;
		float minY = FLT_MAX, maxY = FLT_MIN;
		float minZ = FLT_MAX, maxZ = FLT_MIN;
		float x, y, z;

		for (auto it : weighted_points)
		{
			x = it.first.point().x();
			y = it.first.point().y();
			z = it.first.point().z();

			minX = minX > x ? x : minX;
			maxX = maxX < x ? x : maxX;
			minY = minY > -y ? -y : minY;
			maxY = maxY < -y ? -y : maxY;
			minZ = minZ > z ? z : minZ;
			maxZ = maxZ < z ? z : maxZ;
		}

		if (!res.is_inside(minX, minY, minZ) || !res.is_inside(maxX, maxY, maxZ))
		{
			res = Box(minX < res.get_xmin() ? minX : res.get_xmin()
				, maxX > res.get_xmax() ? maxX : res.get_xmax()
				, minY < res.get_ymin() ? minY : res.get_ymin()
				, maxY > res.get_ymax() ? maxY : res.get_ymax()
				, minZ < res.get_zmin() ? minZ : res.get_zmin()
				, maxZ > res.get_zmax() ? maxZ : res.get_zmax());

			res = Box(res.get_xmin() * 1.2f,
				res.get_xmax() * 1.2f,
				res.get_ymin() * 1.2f,
				res.get_ymax() * 1.2f,
				res.get_zmin() * 1.2f,
				res.get_zmax() * 1.2f);

		}

		return res;
	}
	/*Voronoi*/

	point_radius_v get_poles_to_all_voronoi_cells(std::vector<Point>& filtered_points, Box& box);

	void make_voronoi_cells_threadsafe(delaunay_triangulation_s& T, const std::set<Point>& box_points, point_radius_s* wp, std::vector<delaunay_finite_vertices_iterator_s> fvi_v);

	void set_dual_threadsafe(delaunay_triangulation_s& T, delaunay_cell_handle_s cell, cell_inf* info);

	std::pair<weighted_point, weighted_point> determine_poles(const Point& surface_point, const std::vector<Point>& cell_vertices);

	/*Power*/

	face_simplex get_power_crust_faces(point_radius_v& w_p, const Box& box);

	void make_power_cells(regular_triangulation& R, const std::set<Point>& box_points, regular_finite_vertices_iterator vit, regular_finite_vertices_iterator end, power_faces_off_data* pod);

	void set_cell_info(regular_triangulation& R, regular_cell_handle cell, cell_inf_power* info);

	/* ****************************
	Power Crust calculation functions
	**************************** */
	std::map<PolarBall*, std::set<PolarBall*>> get_neighbour_graph(const std::vector<point_inf*>& p_inf_v, std::map<Point, std::vector<PolarBall*>>& surf_p_balls);

	void label_polar_balls(std::map<PolarBall*, std::set<PolarBall*>>& , std::map<Point, std::vector<PolarBall*>>& surf_p_balls, const Box& bounded);

	void get_power_faces(std::map<PolarBall*, std::set<PolarBall*>>& neigh_graph, face_simplex& fs,const std::vector<point_inf*> inf_ptrs);

	void correct_unoriented_power_crust_faces(std::vector<face*>& unoriented_face_ptrs);

	export_data get_triangled_mesh_from_face_simplex(std::vector<face*>& power_crust_simplex);

private:
	std::vector<PolarBall*> pbs;
};