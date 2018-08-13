#pragma once
#include <vector>
#include "base_types.h"

class PolarBall
{
	enum flag
	{
		INNER,
		OUTER,
		UNKNOWN
	};

public:
	PolarBall() {}
	PolarBall(Point p, const float& rad, const std::vector<Point>& sur_p, const int& id) : center(p), radius(rad), surf_points(sur_p), pb_id(id), label(UNKNOWN), in(0.0f), out(0.0f) {}
	~PolarBall() {}

	void set_pole(Point _p, const float& r) { center = _p; radius = r; }

	Point get_point() const { return center; }
	float get_radius() const { return radius; }
	std::vector<Point> get_surf_points() const { return surf_points; }
	bool is_inner_pole() const { return label == INNER; }
	bool is_outer_pole() const { return label == OUTER; }
	bool is_unkown() const { return label == UNKNOWN; }
	bool is_empty_pole() const { return /*center == NULL*/false; }

	int get_pb_index() const { return pb_id; }

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

	float get_alpha_weight(PolarBall* rhs);					//intersect angle of shallowly intersect ball
	float get_beta_weight(PolarBall* rhs, Point& surfpoint);	//angle related to a common surfpoint

	int get_heap_index() const { return heap_index; }
	void set_heap_index(const int& h_index) { heap_index = h_index; }

private:
	Point center;
	float radius;

	std::vector<Point> surf_points;
	flag label;

	int pb_id;

	float in, out;	//should be between 0.0f and 1.0f
	int heap_index;
};