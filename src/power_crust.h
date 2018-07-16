#pragma once

#include "export.h"	//export_data!
#include "power_diagram.h"
#include "diagram_types.h"

#include "cgal_inc.h"
#include <boost/thread.hpp>

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

public:
	PowerCrust()
	{
	}

	export_data calc(std::vector<Point>& filtered_points,const Box& box = Box(-1.0f,1.0f))
	{
		point_radius_v w_p = get_poles_to_all_voronoi_cells(filtered_points, box * 2.0f);

		/*printf("The weighted points for pd:\n");
		for (auto& it : w_p)
			printf("\t%d - %d\n", it.first.point(),it.first.weight());
		*/

		PowerDiagram pd;

		pd.set_box(box);
		pd.calc_diagram(w_p);
		pd.label_poles();
		pd.calc_power_crust();

		export_data res = pd.get_triangled_mesh_without_texture();
		pd.clear();
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

	point_radius_v get_poles_to_all_voronoi_cells(std::vector<Point>& filtered_points,const Box& box);

	void make_voronoi_cells_threadsafe(delaunay_triangulation_s& T, const std::set<Point>& box_points, point_radius_s* wp, std::vector<delaunay_finite_vertices_iterator_s> fvi_v);

	void set_dual_threadsafe(delaunay_triangulation_s& T, delaunay_cell_handle_s cell, cell_inf* info);

	std::pair<weighted_point, weighted_point> determine_poles(const Point& surface_point, const std::vector<Point>& cell_vertices);

private:

};