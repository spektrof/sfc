#pragma once
#include <vector>
#include <set>
#include <stdio.h>
#include "base_types.h"
#include "base_utils.h"

#include <CGAL/property_map.h>

struct config_data
{
	std::vector<const char*> devices;
	std::vector<std::vector<const char*>> filter_details;
	int number_of_outlier_filter;
};

struct texture_data
{
	uv texture_coordinate;
	uint16_t object_id;
	int16_t device_id;

	texture_data() : device_id(-1) {}
	texture_data(const int16_t& di, const uint16_t oi, const uv& t) : texture_coordinate(t), object_id(oi), device_id(di) {}
};

typedef std::pair<Point, std::vector<texture_data>> tuple;
typedef std::vector<tuple> pointcloud_data;
typedef CGAL::First_of_pair_property_map<tuple>  filter_pointmap;

struct export_data
{
	std::vector<Point> vertex_v;
	std::vector<std::pair<uint32_t, uint32_t>> face_v;
	std::vector<uv> uv_v;
	std::vector<uint32_t> device_related_size;

	export_data() {}
	export_data(const std::vector<Point>& v, const std::vector<std::pair<uint32_t, uint32_t>>& f, const std::vector<uv>& u = std::vector<uv>()) : vertex_v(v), face_v(f), uv_v(u) {}
};

// Related basic functions

inline std::ostream& operator << (std::ostream& out, const uv& texture)
{
	out << texture.first << " " << texture.second;
	return out;
}