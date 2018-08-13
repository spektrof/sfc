#pragma once

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>

#include <vector>
#include <map>
#include <boost/thread/mutex.hpp>
#include "power_crust_types.h"

struct cell_inf
{
	int index;
	Point dual;

	boost::mutex* mux;

	cell_inf() : index(-1) { mux = new boost::mutex(); }
	~cell_inf() { }
};

typedef CGAL::Triangulation_cell_base_with_info_3<cell_inf, Kernel, CGAL::Triangulation_cell_base_3<Kernel>> delaunay_triangulation_cell_base_with_info_s;

typedef CGAL::Triangulation_data_structure_3<
	CGAL::Triangulation_vertex_base_3<Kernel>,
	delaunay_triangulation_cell_base_with_info_s,
	Concurrency_tag>                          delaunay_triangulation_data_structure_s;
typedef CGAL::Delaunay_triangulation_3<Kernel, delaunay_triangulation_data_structure_s> delaunay_triangulation_s;

typedef delaunay_triangulation_s::Finite_vertices_iterator delaunay_finite_vertices_iterator_s;
typedef delaunay_triangulation_s::Finite_cells_iterator delaunay_finite_cells_iterator_s;
typedef delaunay_triangulation_s::Cell_handle delaunay_cell_handle_s;
typedef delaunay_triangulation_s::Vertex_handle delaunay_vertex_handle_s;

//-----------------------------------
//-----------------------------------
//-----------------------------------
//-----------------------------------

struct cell_inf_power
{
	int index;
	Point* dual;
	std::map<Point*, edge> edge;	//mutable???

	boost::mutex* mux;

	cell_inf_power() : index(-1), dual(nullptr) { mux = new boost::mutex();  edge.clear(); }
	~cell_inf_power() {}
};

struct point_inf
{
	std::vector<Point> surface_points;
	std::map<int, std::set<edge*>> neighbour_cell_edges;

	boost::mutex* mux;

	point_inf(const std::vector<Point>& sp = std::vector<Point>()) : surface_points(sp) { mux = new boost::mutex(); neighbour_cell_edges.clear(); }
	~point_inf() { }
};

#include <CGAL/Regular_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>

typedef Kernel::Weighted_point_3                            weighted_point;

typedef CGAL::Regular_triangulation_vertex_base_3<Kernel>        regular_triangulation_vertex_base;
typedef CGAL::Triangulation_vertex_base_with_info_3<point_inf, Kernel, regular_triangulation_vertex_base> regular_triangulation_vertex_base_with_info;
typedef CGAL::Regular_triangulation_cell_base_3<Kernel>          regular_triangulation_cell_base;
typedef CGAL::Triangulation_cell_base_with_info_3<cell_inf_power, Kernel, regular_triangulation_cell_base>  regular_triangulation_cell_base_with_info;

typedef CGAL::Triangulation_data_structure_3<
	regular_triangulation_vertex_base_with_info,
	regular_triangulation_cell_base_with_info,
	Concurrency_tag
> regular_triangulation_data_structure;

typedef CGAL::Regular_triangulation_3<Kernel, regular_triangulation_data_structure> regular_triangulation;

typedef regular_triangulation::Cell_handle regular_cell_handle;
typedef regular_triangulation::Finite_vertices_iterator  regular_finite_vertices_iterator;
