#pragma once
#include "base_types.h"

namespace tex_utils
{
	typedef unsigned int index;
	typedef std::pair<index, Point> index_point;
	typedef std::pair<index_point, uv> p_texture_t;

	bool is_inside_triangle(const Point& s, const Point& a, const Point& b, const Point& c)
	{
		float as_x = s.x() - a.x();
		float as_y = s.y() - a.y();

		bool s_ab = (b.x() - a.x())*as_y - (b.y() - a.y())*as_x >= 0;

		if ((c.x() - a.x())*as_y - (c.y() - a.y())*as_x >= 0 == s_ab) return false;

		if ((c.x() - b.x())*(s.y() - b.y()) - (c.y() - b.y())*(s.x() - b.x()) >= 0 != s_ab) return false;

		return true;
	}

	float sign(const Point& p1, const Point& p2, const Point& p3)
	{
		return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y());
	}

	bool is_inside_triangle2(const Point& pt, const Point& v1, const Point& v2, const Point& v3)
	{
		bool b1, b2, b3;

		b1 = sign(pt, v1, v2) < 0.0f;
		b2 = sign(pt, v2, v3) < 0.0f;
		b3 = sign(pt, v3, v1) < 0.0f;

		return ((b1 == b2) && (b2 == b3));
	}

	bool calc_new_coord_system(const Point& p1, const Point& p2, const Point& p3, Vector& unit_x, Vector& unit_y, Vector& unit_z)
	{
	//	qDebug() << p1 << " - " << p2 << " - " << p3;
		if (p1 == p2 || p1 == p3 || p2 == p3) return false;

		auto origin = p1;
		auto localz = CGAL::cross_product(p2 - origin, p3 - origin);

		//calculate local normalvector
		unit_z = localz / sqrt(localz.squared_length());

		//calculate local x vector in plane
		auto localx = p2 - origin;
		unit_x = localx / sqrt(localx.squared_length());

		//calculate local y
		auto localy = CGAL::cross_product(localz, localx);
		unit_y = localy / sqrt(localy.squared_length());
		return true;
	}

	std::vector<Point> calc_projected_points(const Point& p1, const Point& p2, const Point& p3, Vector& unit_x, Vector& unit_y, Vector& unit_z, Point& p)
	{
		std::vector<Point> res;

		res.resize(3);
		res[0] = Point(0, 0, 0);
		auto tmp = CGAL::scalar_product(p2 - p1, unit_x) * unit_x + CGAL::scalar_product(p2 - p1, unit_y) * unit_y + CGAL::scalar_product(p2 - p1, unit_z) * unit_z;
		res[1] = Point(tmp.x(), tmp.y(), tmp.z());
		tmp = CGAL::scalar_product(p3 - p1, unit_x) * unit_x + CGAL::scalar_product(p3 - p1, unit_y) * unit_y + CGAL::scalar_product(p3 - p1, unit_z) * unit_z;
		res[2] = Point(tmp.x(), tmp.y(), tmp.z());
		tmp = CGAL::scalar_product(p - p1, unit_x) * unit_x + CGAL::scalar_product(p - p1, unit_y) * unit_y + 0 * unit_z;
		p = Point(tmp.x(), tmp.y(), tmp.z());
		
	//	qDebug() << "The projected points: \n\t" << res;
	//	qDebug() << "\tThe common projected point: " << p;
	//	qDebug() << CGAL::scalar_product(p - res[1], unit_z) / sqrt((p - res[1]).squared_length());
	//	qDebug() << CGAL::scalar_product(p - res[0], unit_z) / sqrt((p - res[0]).squared_length());
	//	qDebug() << CGAL::scalar_product(p - res[2], unit_z) / sqrt((p - res[2]).squared_length());
	//	if (is_inside_triangle2(p, res[0], res[2], res[1]))
	//		qDebug() << "what?";
		return res;
	}

	void inverse_projection_to_original_cs(std::vector<Point>& pois, Vector& unit_x, Vector& unit_y, Vector& unit_z, Point& origin)
	{
		for (auto& poi : pois)
		{
			auto x = poi.x();
			auto y = poi.y();
			auto z = poi.z();

			poi = Point(CGAL::scalar_product(Vector(unit_x.x(), unit_y.x(), unit_z.x()), Vector(x, y, z)) + origin.x(),
				CGAL::scalar_product(Vector(unit_x.y(), unit_y.y(), unit_z.y()), Vector(x, y, z)) + origin.y(),
				CGAL::scalar_product(Vector(unit_x.z(), unit_y.z(), unit_z.z()), Vector(x, y, z)) + origin.z());
		}
	}

	std::pair<Vector, uv> calc_distance_vector_from_line_to_p(const std::vector<Point>& proj_pois, const std::vector<index>& clos_ind, const std::vector<uv>& clos_uvs, const Point& p, const std::vector<uv>& P_uvs)
	{
		uv p_uv;
		if (pair_length((clos_uvs[0] + clos_uvs[1]) / 2.0f - P_uvs[0]) < 0.05f)
			p_uv = P_uvs[0];
		else
			p_uv = P_uvs[1];

	//	qDebug() << "P uv is " << p_uv;

		index ccw0 = clos_ind[0];
		index ccw1 = clos_ind[1];

		auto AB = proj_pois[ccw1] - proj_pois[ccw0];
		auto AP = p - proj_pois[ccw0];

		AB /= sqrt(AB.squared_length());
		float proj = CGAL::scalar_product(AB, AP);

		auto X = proj_pois[ccw0] + proj * AB;

		uv AB_uv = clos_uvs[1] - clos_uvs[0];
		uv AP_uv = p_uv - clos_uvs[0];
		auto l = pair_length(AB_uv);
		AB_uv = l == 0 ? uv(0.0f,0.0f) : AB_uv / pair_length(AB_uv);

		auto x_uv = clos_uvs[0] + AB_uv * (AB_uv.first * AP_uv.first + AB_uv.second * AP_uv.second);

		return std::pair<Vector, uv>(Vector(p - X), p_uv - x_uv);
	}
}