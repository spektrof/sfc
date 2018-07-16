#pragma once

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>

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