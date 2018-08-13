#pragma once
#include "base_types.h"
#include "diagram_types.h"
#include "power_crust_types.h"
#include "reconstructor_utils.h"
#include "priority_queue.h"
#include <boost/thread/mutex.hpp>

#include "io_types.h"
#include "cgal_inc.h"

//typedef Kernel::Weighted_point_3                            weighted_point;

typedef CGAL::Regular_triangulation_vertex_base_3<Kernel>        regular_triangulation_vertex_base2;
typedef CGAL::Triangulation_vertex_base_with_info_3<point_inf2, Kernel, regular_triangulation_vertex_base2> regular_triangulation_vertex_base_with_info2;
typedef CGAL::Regular_triangulation_cell_base_3<Kernel>          regular_triangulation_cell_base2;
typedef CGAL::Triangulation_cell_base_with_info_3<cell_inf_power2, Kernel, regular_triangulation_cell_base2>  regular_triangulation_cell_base_with_info2;

typedef CGAL::Triangulation_data_structure_3<
	regular_triangulation_vertex_base_with_info2,
	regular_triangulation_cell_base_with_info2,
	Concurrency_tag
> regular_triangulation_data_structure2;

typedef CGAL::Regular_triangulation_3<Kernel, regular_triangulation_data_structure2> regular_triangulation2;

typedef regular_triangulation2::Cell_handle regular_cell_handle2;
typedef regular_triangulation2::Finite_vertices_iterator  regular_finite_vertices_iterator2;

struct Pole2
{
	Point* center;
	float radius;
	bool is_null() const { return center == NULL; }
	Pole2(Point* c = NULL, const float& r = 0.0f) : center(c), radius(r) {}
};

class PolarBall2
{
	enum flag
	{
		INNER,
		OUTER,
		UNKNOWN
	};

public:
	PolarBall2() {}
	PolarBall2(Pole2& _pole, const std::vector<Point>& sur_p) : pole(_pole), surf_points(sur_p), label(UNKNOWN), in(0.0f), out(0.0f) {}
	~PolarBall2() {}

	void set_pole(Point* _p, const float& r) { pole.center = _p; pole.radius = r; }

	Pole2 get_pole() const { return pole; }
	Point get_point() const { return *pole.center; }
	float get_radius() const { return pole.radius; }
	std::vector<Point> get_surf_points() const { return surf_points; }
	bool is_empty_pole() const { return pole.is_null(); }
	bool is_inner_pole() const { return label == INNER; }
	bool is_outer_pole() const { return label == OUTER; }
	bool is_unkown() const { return label == UNKNOWN; }

	float get_in_value() const { return in; }
	float get_out_value() const { return out; }
	void  set_in_value(const float& _in) { in = _in; }
	void  set_out_value(const float& _out) { out = _out; }

	void set_flag(const unsigned& ind) { if (ind > 2) return;  label = flag(ind); }
	unsigned int get_flag() const { return label; }

	unsigned int get_opposite_flag() const { return label == 2 ? 2 : 1 - label; }
	float get_value_by_flag(const unsigned int& flag) const
	{
		if (flag == 2) return -1.0f;

		return flag == 0 ? get_in_value() : get_out_value();
	}

	void set_value_by_flag(const unsigned int& flag, const float& new_val)
	{
		if (flag == 2) return;

		if (flag == 0)
			set_in_value(new_val);
		else
			set_out_value(new_val);
	}

	float get_alpha_weight(PolarBall2* rhs);					//intersect angle of shallowly intersect ball
	float get_beta_weight(PolarBall2* rhs, Point& surfpoint);	//angle related to a common surfpoint

	int get_heap_index() const { return heap_index; }
	void set_heap_index(const int& h_index) { heap_index = h_index; }

private:
	Pole2 pole;
	std::vector<Point> surf_points;
	flag label;

	float in, out;	//should be between 0.0f and 1.0f
	int heap_index;
};

class PowerCell
{

public:
	PowerCell(Point* pol_p, const float& r, const std::vector<Point>& sur_p, const unsigned int& c_i) : polar_ball(PolarBall2(Pole2(pol_p, r), sur_p)), my_index(c_i) {
		cell_faces.clear();
	}
	~PowerCell() {
		cell_faces.clear();
	}

	Pole2 get_pole_point() const { return polar_ball.get_pole(); }

	bool has_inner_pole() const { return polar_ball.is_inner_pole(); }
	PolarBall2* get_polarball_ptr() { return &polar_ball; }
	std::vector<Point> get_related_surf_points() const { return polar_ball.get_surf_points(); }

	void add_face(face_t * face)
	{
		cell_faces.push_back(face);
	}

	std::vector<unsigned int> get_neighbours()
	{
		if (cell_faces.empty())	return std::vector<unsigned int>();

		std::vector<unsigned int> my_neighbours;

		for (auto& face : cell_faces)
		{
			int neigh_candidate = face->get_neighbour_cell(my_index);
			if (neigh_candidate == -1 || neigh_candidate == -3) printf("\tERR: BAD getting neighbours\n");
			if (neigh_candidate < 0) continue;

			my_neighbours.push_back(neigh_candidate);
		}

		return my_neighbours;
	}

	int get_neighbour(const int& face_id)
	{
		if (face_id < 0) return -1;		//important because of drawings

		int correct_face_id = face_id % cell_faces.size();

		return cell_faces[correct_face_id]->get_neighbour_cell(my_index);
	}

	face_t* get_common_face(const unsigned int& neighbour)
	{
		for (auto& face : cell_faces)
		{
			int neigh_candidate = face->get_neighbour_cell(my_index);
			if (neigh_candidate == -1 || neigh_candidate == -3) printf("\tERR: BAD neighbours\n");
			if (neigh_candidate < 0) continue;

			if (neigh_candidate == neighbour) return face;
		}

		return nullptr;
	}

	std::vector<face_t*> faces() const
	{
		return cell_faces;
	}

	unsigned int get_my_index() const
	{
		return my_index;
	}

private:
	PolarBall2 polar_ball;

	std::vector<face_t*> cell_faces;
	unsigned int my_index;		//TODO: this is redundant
};

class PowerDiagram
{
	typedef priority_queue<PolarBall2> priority_ball_queue;

public:
	PowerDiagram() : bounded(Box(-1.0f, 1.0f))
	{
		mux = new boost::mutex();

		cells.clear();
		power_faces.clear();
		power_edges.clear();
		power_vertices.clear();
		pole_map.clear();
	}
	~PowerDiagram()
	{
		for (auto& it : cells)
			delete it;

		cells.clear();
		power_edges.clear();
		power_faces.clear();
		power_vertices.clear();
		pole_map.clear();
	}

	typedef std::vector<PowerCell*>::iterator Power_cell_iterator;
	typedef std::vector<PowerCell*>::const_iterator const_Power_cell_iterator;

	Power_cell_iterator cells_begin() { return cells.begin(); }

	Power_cell_iterator cells_end() { return cells.end(); }

	PowerCell* getCell(const int& ind) { return cells[ind]; }

	void addCell(PowerCell* vc) { cells.push_back(vc); }

	void clear() {
		for (auto& it : cells)
			delete it;
		cells.clear();
		pole_map.clear();

		power_faces.clear();;
		power_edges.clear();;
		power_vertices.clear();;
	}

	size_t size() const { return cells.size(); }
	void set_box(const Box& b) { bounded = b; }

	export_data get_triangled_mesh_without_texture();

	//------------------------------------------
	//surf pontok a polejaikkal
	void calc_diagram(const std::vector<std::pair<Pole2, Point> >& weighted_points);
	void calc_diagram(std::vector<std::pair<weighted_point, point_inf>>& w_p);
	void label_poles();
	void calc_power_crust();

protected:
	typedef std::pair<Point*, unsigned int> point_identifier;
	typedef std::pair<edge_tt*, std::set<unsigned int>> edge_identifier;
	typedef std::pair<int, int> neighbour;

	void add_box_points(const Box& box, std::set<Point>& box_points, std::vector<std::pair<weighted_point, point_inf2>>& points);

	Box update_box(const std::vector<std::pair<weighted_point, point_inf2>>& weighted_points);
	Box update_box(const std::vector<std::pair<Pole2, Point> >& weighted_points);

	void set_cell_info(regular_triangulation2& T, regular_cell_handle2 cell, cell_inf_power2* info);
	void set_cell_info_threadsafe(regular_triangulation2& T, regular_cell_handle2 cell, cell_inf_power2* info);

	void make_power_cells(regular_triangulation2&, const std::set<Point>&, regular_finite_vertices_iterator2, regular_finite_vertices_iterator2, std::vector<regular_finite_vertices_iterator2>* , std::vector<point_inf2*>*, std::map<edge_tt, edge_identifier>*);
	void make_power_cells_threadsafe(regular_triangulation2&, const std::set<Point>&, std::vector<regular_finite_vertices_iterator2> fvi_v, std::vector<regular_finite_vertices_iterator2>*, std::vector<point_inf2*>*, std::map<edge_tt, edge_identifier>*, unsigned int* act_cell_id);

	void correct_unoriented_power_crust_faces(std::vector<face_t*>& unoriented_face_ptrs);

private:
	typedef typename vertex_t::v_ptr v_ptr;
	typedef typename face_tt::f_ptr f_ptr;
	typedef typename edge_pp::ee_ptr e_ptr;

	std::vector<PowerCell*> cells;

	boost::mutex* mux;

	std::vector<v_ptr> power_vertices;
	std::vector<e_ptr> power_edges;
	std::vector<f_ptr> power_faces;

	std::map<Point, unsigned int> pole_map;				//Pole verticies with index

	//-----------------------

	Box bounded;
};