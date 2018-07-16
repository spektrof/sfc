#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

#ifdef TBB_ENABLED
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

inline std::ostream& operator << (std::ostream& out, const Point& p)
{
	out << p.x() << " " << p.y() << " " << p.z();
	return out;
}

inline std::pair<float, float> operator * (const std::pair<float, float>& lhs, const float& rhs)
{
	return std::pair<float, float>(lhs.first * rhs, lhs.second * rhs);
}

inline std::pair<float, float> operator / (const std::pair<float, float>& lhs,const float& rhs)
{
	return std::pair<float, float>(lhs.first / rhs, lhs.second / rhs);
}

inline std::pair<float, float> operator + (const std::pair<float, float>& lhs, const std::pair<float, float>& rhs)
{
	return std::pair<float, float>(lhs.first + rhs.first, lhs.second + rhs.second);
}

inline std::pair<float, float> operator / (const std::pair<float, float>& lhs, const std::pair<float, float>& rhs)
{
	return std::pair<float, float>(lhs.first / rhs.first, lhs.second / rhs.second);
}

inline std::pair<float, float> operator * (const std::pair<float, float>& lhs, const std::pair<float, float>& rhs)
{
	return std::pair<float, float>(lhs.first * rhs.first, lhs.second * rhs.second);
}

inline Point operator +(const Point& lhs, const Point& rhs)
{
	return Point(lhs.x() + rhs.x(), lhs.y() + rhs.y(), lhs.z() + rhs.z());
}

inline Point operator / (const Point& lhs, const float& rhs)
{
	return Point(lhs.x() / rhs, lhs.y() / rhs, lhs.z() / rhs);
}

inline Point operator *(const Point& lhs, const Point& rhs)
{
	return Point(lhs.x() * rhs.x(), lhs.y() * rhs.y(), lhs.z() * rhs.z());
}

inline Point operator *(const Point& lhs, const Vector& rhs)
{
	return Point(lhs.x() * rhs.x(), lhs.y() * rhs.y(), lhs.z() * rhs.z());
}

inline Point operator *(const Point& lhs, const float& rhs)
{
	return Point(lhs.x() * rhs, lhs.y() * rhs, lhs.z() * rhs);
}

inline std::pair<float, float> operator - (const std::pair<float, float>& lhs, const std::pair<float, float>& rhs)
{
	return std::pair<float, float>(lhs.first - rhs.first, lhs.second - rhs.second);
}

inline float pair_length(const std::pair<float, float> p)
{
	return sqrt(p.first * p.first + p.second * p.second);
}

inline float scalar_p_uv(const std::pair<float, float>& lhs, const std::pair<float, float>& rhs)
{
	return lhs.first * rhs.first + lhs.second * rhs.second;
}