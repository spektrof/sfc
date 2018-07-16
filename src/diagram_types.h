#pragma once

#include "cgal_types.h"
#include <map>
#include <set>
#include <vector>

#include <memory>
#include <boost/thread/mutex.hpp>

struct edge_tt;
typedef std::pair<unsigned int, Point*> edge_point;
typedef std::pair<edge_point, edge_point> edge_t;

struct cell_inf_power
{
	int index;
	Point* dual;
	std::map<Point*, edge_tt*> edge;

	boost::mutex* mux;

	cell_inf_power() : index(-1), dual(NULL) { mux = new boost::mutex();  edge.clear(); }
	~cell_inf_power() {}
};

struct point_inf
{
	std::vector<Point> surface_points;
	std::map<int, std::set<edge_tt*>> neighbour_cell_edges;

	boost::mutex* mux;

	point_inf(const std::vector<Point>& sp = std::vector<Point>()) : surface_points(sp) { mux = new boost::mutex(); neighbour_cell_edges.clear(); }
	~point_inf() { }
};

// ************************
//	Tuple structs for storing the data

//		I separated the voronoi (_t) and power diagram data types  (_tt)
//		for storing the less data what I need	
// ***********************
struct vertex_t
{
	typedef std::shared_ptr<vertex_t> v_ptr;
	Point point;

	vertex_t(const Point& p = Point(0, 0, 0)) : point(p) {}
};

struct face_t;

struct edge_tt
{
	edge_t edge;
	std::map<face_t*, unsigned int> related_faces;

	// Constructors and sorter functor
	edge_tt() { related_faces.clear(); }

	edge_tt(edge_t e) : edge(e) {	related_faces.clear();	}

	~edge_tt() {  related_faces.clear();  }

	bool operator < (const edge_tt& e) const
	{
		return edge < e.edge;
	}

	// Methods

	void add_face(face_t* f, const unsigned int& o)
	{
		related_faces.insert(std::pair<face_t*, unsigned int>(f, o));
	}

	void swap_orientation(face_t* face)
	{
		if (face == nullptr)	return;

		auto location = related_faces.find(face);

		if (location != related_faces.end())
		{
			location->second = 1 - location->second;
			if (location->second > 1 || location->second < 0)
				printf("Bad orientation in swapping!\n");
		}
		else
			printf("Didnt find that face!");
	}
};

struct edge_pp
{
	typedef std::shared_ptr<edge_pp> ee_ptr;

	edge_tt edge;
	edge_pp(const edge_tt e) : edge(e) {}
};

struct face_t
{
	std::vector<edge_tt*> edges;
	std::vector<unsigned int> related_cells;		// cell ids
	bool power_crust_face;							

	// Con(De)structors
	face_t() : power_crust_face(false) { edges.clear();  related_cells.clear(); }
	face_t(const std::set<edge_tt*>& in_edges, const std::vector<unsigned int>& rel_cells) : related_cells(rel_cells), power_crust_face(false)
	{ 
		edges.clear();
		for (auto& it : in_edges)
			add_edge(it, 0);
	}

	~face_t() {	edges.clear(); }

	//Methods
	void add_edge(edge_tt* e, const int& o)
	{
		e->add_face(this, o);
		edges.push_back(e);
	}

	void add_edges(const std::set<edge_tt*>& in_edges, const int& o)
	{
		edges.clear();
		for (auto& it : in_edges)
			add_edge(it, o);
	}

	void add_related_cell(const unsigned int& cell)
	{
		related_cells.push_back(cell);
	}

	void set_related_cell(const std::vector<unsigned int>& cell_ids)
	{
		related_cells = cell_ids;
	}

	void set_power_crust_flag(bool val)
	{
		power_crust_face = val;
	}

	bool is_power_crust_face() const
	{
		return power_crust_face;
	}

	int get_neighbour_cell(const unsigned int& mine_index) const
	{
		if (related_cells.empty()) return -1;
		if (related_cells[0] != mine_index) return related_cells[0];
		if (related_cells.size() > 2) return -3;
		if (related_cells.size() == 1) return -2;
		return related_cells[1];
	}

	std::set<Point*> get_face_points()
	{
		if (edges.empty()) return std::set<Point*>();
	
		std::set<Point*> face_points;

		for (auto& edge : edges)
		{
			Point* p1 = edge->edge.first.second;
			Point* p2 = edge->edge.second.second;

			face_points.insert(p1);
			face_points.insert(p2);
		}

		return face_points;
	}

	//-------------------------------------

	void correct_edge_orientations();

	void swap_edge_orientations()
	{
		std::vector<edge_tt*> reversed;
		for (auto& edge : edges)
		{
			edge->swap_orientation(this);
			reversed.insert(reversed.begin(), edge);
		}
		std::vector<edge_tt*>(reversed).swap(edges);
	}

	void correct_boundary_orientation();
};

struct face_tt
{
	typedef std::shared_ptr<face_tt> f_ptr;

	face_t face;
	face_tt() {}
	face_tt(const std::set<edge_tt*>& in_edges, const std::vector<unsigned int>& rel_c) : face(in_edges, rel_c) {}
};

struct Pole
{
	Point* center;
	float radius;
	Pole(Point* c = NULL, const float& r = 0.0f) : center(c), radius(r) {}

	bool is_null() const { return center == NULL; }
};
