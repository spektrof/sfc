/*
	All necessery functions for using k-d tree are here
*/

#pragma once

#include "texture_types.h"

// *********************

inline std::ostream& operator << (std::ostream& os, const tree_2d_element& e)
{
	os << "(" << e.first.first << ", " << e.first.second <<")";
	return os;
}

inline std::ostream& operator << (std::ostream& out, const tree_3d_element& e)
{
	out << e.first.x() << " - " << e.second.index;
	return out;
}

namespace utils_for_kdtree
{
	// functors for tree_3d_element
	struct XYZOrder {
		bool operator()(const tree_3d_element& c, const tree_3d_element& _c) {
			return c.first.x() < _c.first.x() || (c.first.x() == _c.first.x() && c.first.y() < _c.first.y()) || (c.first.x() == _c.first.x() && c.first.y() == _c.first.y() && c.first.z() < _c.first.z());
		}
	};
	struct YZXOrder {
		bool operator()(const tree_3d_element& c, const tree_3d_element& _c) {
			return c.first.y() < _c.first.y() || (c.first.y() == _c.first.y() && c.first.z() < _c.first.z()) || (c.first.y() == _c.first.y() && c.first.z() == _c.first.z() && c.first.x() < _c.first.x());
		}
	};
	struct ZXYOrder {
		bool operator()(const tree_3d_element& c, const tree_3d_element& _c) {
			return c.first.z() < _c.first.z() || (c.first.z() == _c.first.z() && c.first.x() < _c.first.x()) || (c.first.z() == _c.first.z() && c.first.x() == _c.first.x() && c.first.y() < _c.first.y());
		}
	};
	
	inline void sort_part_vector_based_on_axis(const int& axis, tree_3d_element* start, const int& length)
	{
		if (axis > 2 || axis < 0) return;

		if (axis == 2)
			std::nth_element(start, start + length / 2, start + length, ZXYOrder());
		else if (axis == 1)
			std::nth_element(start, start + length / 2, start + length, YZXOrder());
		else
			std::nth_element(start, start + length / 2, start + length, XYZOrder());
	}

	// functors for tree_2d_element
	struct Elem2D_XYOrder {
		bool operator()(const tree_2d_element& c, const tree_2d_element& _c) {
			return c.first.first < _c.first.first || (c.first.first == _c.first.first && c.first.second < _c.first.second);
		}
	};
	struct Elem2D_YXOrder {
		bool operator()(const tree_2d_element& c, const tree_2d_element& _c) {
			return c.first.second < _c.first.second || (c.first.second == _c.first.second && c.first.first < _c.first.first);
		}
	};

	inline void sort_part_vector_based_on_axis(const int& axis, tree_2d_element* start, const int& length)
	{
		if (axis > 1 || axis < 0) return;

		if (axis == 0)
			std::nth_element(start, start + length / 2, start + length, Elem2D_XYOrder());
		else
			std::nth_element(start, start + length / 2, start + length, Elem2D_YXOrder());

	}

	//-----------------------------------

	inline float get_coord_based_on_axis(const uv& element, const int& axis)
	{
		if (axis > 1 || axis < 0) return 0.0f;

		return axis == 0 ? element.first : element.second;
	}

	inline float get_coord_based_on_axis(const Point& element, const int& axis)
	{
		if (axis > 2 || axis < 0) return 0.0f;

		return axis == 0 ? element.x() : axis == 1 ? element.y() : element.z();
	}

	//----------------------------------------

	inline float sum_of_square(const Point& lhs, const Point& rhs)
	{
		float x = lhs.x() - rhs.x();
		float y = lhs.y() - rhs.y();
		float z = lhs.z() - rhs.z();

		return x * x + y * y + z * z;
	}

	inline float sum_of_square(const uv& lhs, const uv& rhs)
	{
		float x = lhs.first - rhs.first;
		float y = lhs.second - rhs.second;

		return sqrt(x * x + y * y);		//TODO: this is related to epsilon
	}

	//-----------------------------------

	inline float distance_based_on_axis(const Point& lhs, const Point& rhs, const int& axis)
	{
		if (axis > 2 || axis < 0) return 0.0f;

		if (axis == 0)	return lhs.x() - rhs.x();
		if (axis == 1)	return lhs.y() - rhs.y();
		
		return lhs.z() - rhs.z();
	}

	inline float distance_based_on_axis(const uv& lhs, const uv& rhs, const int& axis)
	{
		if (axis > 1 || axis < 0) return 0.0f;

		if (axis == 0)	return lhs.first - rhs.first;

		return lhs.second - rhs.second;
	}

	inline float uv_length(const uv& p)
	{
		return sqrt(p.first * p.first + p.second * p.second);
	}
}