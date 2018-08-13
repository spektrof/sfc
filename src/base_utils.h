#pragma once
#include "base_types.h"

inline std::ostream& operator << (std::ostream& out, const Point& p)
{
	out << p.x() << " " << p.y() << " " << p.z();
	return out;
}

inline uv operator * (const uv& lhs, const float& rhs)
{
	return uv(lhs.first * rhs, lhs.second * rhs);
}

inline uv operator / (const uv& lhs, const float& rhs)
{
	return uv(lhs.first / rhs, lhs.second / rhs);
}

inline uv operator + (const uv& lhs, const uv& rhs)
{
	return uv(lhs.first + rhs.first, lhs.second + rhs.second);
}

inline uv operator / (const uv& lhs, const uv& rhs)
{
	return uv(lhs.first / rhs.first, lhs.second / rhs.second);
}

inline uv operator * (const uv& lhs, const uv& rhs)
{
	return uv(lhs.first * rhs.first, lhs.second * rhs.second);
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

inline uv operator - (const uv& lhs, const uv& rhs)
{
	return uv(lhs.first - rhs.first, lhs.second - rhs.second);
}

inline float pair_length(const uv p)
{
	return sqrt(p.first * p.first + p.second * p.second);
}

inline float scalar_product(const uv& lhs, const uv& rhs)
{
	return lhs.first * rhs.first + lhs.second * rhs.second;
}