#pragma once
#include "io_types.h"

typedef std::vector<texture_data> uv_map;

typedef std::pair<Point, uv_map> point_uv_tuple;
typedef std::vector<point_uv_tuple> point_uv_map;

struct info_2d
{
	unsigned int index;
	unsigned int g_obj_id;
	info_2d(const unsigned int& i = 0, const unsigned int& gid = 0) : index(i), g_obj_id(gid) {}
};

struct info_3d
{
	uv_map uvs;
	unsigned int index;
	info_3d(const uv_map& uv = uv_map(), const unsigned int& i = 0) : uvs(uv), index(i) {}
};

typedef std::pair<uv, info_2d> tree_2d_element;
typedef std::pair<Point, info_3d> tree_3d_element;