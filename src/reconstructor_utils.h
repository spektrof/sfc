#pragma once

struct Section
{
	float min, max;
	Section(const float& mi = 0.0f, const float& ma = 20.0f) : min(mi), max(ma) {}

	bool is_between(const float& val) const { return val >= min && val <= max; }
	bool is_lower(const float& val) const { return val < min; }
	bool is_higher(const float& val) const { return val > max; }
};

struct Box
{
	Section x, y, z;

	Box(const Section& s) : x(s), y(s), z(s) {}
	Box(const float& min = 0.0f, const float& max = 20.0f) : x(Section(min, max)), y(Section(min, max)), z(Section(min, max)) {}
	Box(const Section& _x, const Section& _y, const Section& _z) : x(_x), y(_y), z(_z) {}
	Box(const float& xmi, const float& xma, const float& ymi, const float& yma, const float& zmi, const float& zma) : x(Section(xmi, xma)), y(Section(ymi, yma)), z(Section(zmi, zma)) {}

	bool is_inside(const float& cx, const float& cy, const float& cz) const { return x.is_between(cx) && y.is_between(cy) && z.is_between(cz); }

	float get_xmin() const { return x.min; }
	void set_xmin(const float& v) { x.min = v; }
	float get_xmax() const { return x.max; }
	void set_xmax(const float& v) { x.max = v;; }
	float get_ymin() const { return y.min; }
	void set_ymin(const float& v) { y.min = v;; }
	float get_ymax() const { return y.max; }
	void set_ymax(const float& v) { y.max = v;; }
	float get_zmin() const { return z.min; }
	void set_zmin(const float& v) { z.min = v;; }
	float get_zmax() const { return z.max; }
	void set_zmax(const float& v) { z.max = v;; }

	float get_xlength() const { return x.max - x.min; }
	float get_ylength() const { return y.max - y.min; }
	float get_zlength() const { return z.max - z.min; }

	float get_volume() const { return get_xlength() * get_ylength() * get_zlength(); }

	Box operator *(const float& scale)
	{
		return Box(this->get_xmin() * scale, this->get_xmax() * scale, this->get_ymin() * scale, this->get_ymax() * scale, this->get_zmin() * scale, this->get_zmax() * scale);
	}

};