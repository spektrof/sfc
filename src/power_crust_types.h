#pragma once
#include "base_types.h"

#include <map>
#include <set>
#include <vector>

struct face;

struct edge
{
	typedef std::pair<Point, Point> edge_t;

	edge_t data;
	std::map<face*, uint32_t> related_faces;

	// Constructors and sorter functor
	edge() { related_faces.clear(); }

	edge(const Point& s, const Point& e) : data(edge_t(s, e)) { related_faces.clear(); }
	edge(const std::pair<Point, Point>& e) : data(edge_t(e.first, e.second)) { related_faces.clear(); }

	~edge() { related_faces.clear(); }

	bool operator < (const edge& e) const
	{
		return data < e.data;
	}

	// Methods

	void add_face(face* f, const uint32_t& o)
	{
		related_faces.insert(std::pair<face*, uint32_t>(f, o));
	}

	void swap_orientation(face* face)
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
			printf("Didnt find that face!\n");
	}
};

struct face
{
	std::vector<edge*> edges;
	std::vector<uint32_t> related_cells;		// cell ids
	bool power_crust_face;

	// Con(De)structors
	face() : power_crust_face(false) { edges.clear();  related_cells.clear(); }
	face(const std::set<edge*>& in_edges, const std::vector<uint32_t>& rel_cells = std::vector<uint32_t>()) : related_cells(rel_cells), power_crust_face(false)
	{
		edges.clear();
		for (auto& it : in_edges)
			add_edge(it, 0);
	}

	~face() { edges.clear(); }

	//Methods
	void add_edge(edge* e, const int& o)
	{
		e->add_face(this, o);
		edges.push_back(e);
	}

	void add_edges(const std::set<edge*>& in_edges, const int& o)
	{
		edges.clear();
		for (auto& it : in_edges)
			add_edge(it, o);
	}

	void add_edges(const std::vector<edge*>& in_edges, const int& o)
	{
		edges.clear();
		for (auto& it : in_edges)
			add_edge(it, o);
	}

	void add_related_cell(const uint32_t& cell)
	{
		related_cells.push_back(cell);
	}

	void set_related_cell(const std::vector<uint32_t>& cell_ids)
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

	int get_neighbour_cell(const uint32_t& mine_index) const
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
			Point* p1 = &edge->data.first;
			Point* p2 = &edge->data.second;

			face_points.insert(p1);
			face_points.insert(p2);
		}

		return face_points;
	}

	//-------------------------------------

	void correct_edge_orientations();

	void swap_edge_orientations()
	{
		std::vector<edge*> reversed;
		for (auto& e : edges)
		{
			e->swap_orientation(this);
			reversed.insert(reversed.begin(), e);
		}
		std::vector<edge*>(reversed).swap(edges);
	}

	void correct_boundary_orientation();
};

struct face_simplex
{
	std::vector<edge*> edges;
	std::vector<face*> faces;
};