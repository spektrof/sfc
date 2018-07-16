#pragma once

#include "cgal_types.h" //simplify...

#include <boost\filesystem\fstream.hpp>

typedef std::pair<float, float> uv;

struct export_data
{
	std::vector<Point> vertex_v;
	std::vector<uint32_t> face_v;
	std::vector<uv> uv_v;

	export_data() {}
	export_data(const std::vector<Point>& v, const std::vector<uint32_t>& f, const std::vector<uv>& u = std::vector<uv>()) : vertex_v(v), face_v(f), uv_v(u) {}
};

inline std::ostream& operator << (std::ostream& out, const uv& texture)
{
	out << texture.first << " " << texture.second;
	return out;
}

namespace exporter
{
	static void reindexing_to_export(export_data& data) //rly, why?
	{
		/*
		unsigned int p_ix = 0;
		std::map<Point, unsigned int> pois;

		for (auto& poi : points)
		{
			auto tmp = pois.insert(std::pair<Point, unsigned int>(poi, p_ix));
			if (tmp.second == true)
			{
				file << "v " << std::setprecision(10) << poi.x() << " " << poi.y() << " " << poi.z() << "\n";
				p_ix++;
			}
		}

		for (size_t i = 0; i < indicies.size(); i += 3)
			file << "f " << pois[points[indicies[i]]] << "/" << pois[points[indicies[i]]] << "/" << pois[points[indicies[i]]] << " "
			<< pois[points[indicies[i + 1]]] << "/" << pois[points[indicies[i + 1]]] << "/" << pois[points[indicies[i + 1]]] << " "
			<< pois[points[indicies[i + 2]]] << "/" << pois[points[indicies[i + 2]]] << "/" << pois[points[indicies[i + 2]]] << " " << "\n";*/
	}

	static void export_to_obj_without_texture(const std::string& file_name, const export_data& data)
	{
		std::string exp_file = "result/" + file_name + ".obj";
		printf("Export to %s", exp_file.c_str());
		boost::filesystem::ofstream file(exp_file);

		file << "#" << exp_file << " with PowerCrust\n\n";
		for (auto it : data.vertex_v)
			file << "v " << std::setprecision(20) << it << "\n";

		file << "#" << data.vertex_v.size() << " vertices\n";
		for (auto it = data.face_v.begin(); it != data.face_v.end(); it+=3)
			file << "f " << *it + 1 << " " << *(it+1) + 1 << " " << *(it + 2) + 1 << " " << "\n";
	
		file << "#" << data.face_v.size() / 3 << " faces\n";
		file.close();
	}

	static void export_to_obj(const std::string& file_name, export_data& data)
	{
		reindexing_to_export(data);

		if (data.uv_v.empty())
		{
			export_to_obj_without_texture(file_name, data);
			return;
		}

		std::string exp_file = "result/" + file_name + ".obj";

		boost::filesystem::ofstream file(exp_file);

		file << "#" << exp_file << " with PowerCrust\n\n";
		for (auto it : data.vertex_v)
			file << "v " << std::setprecision(20) << it << "\n";

		file << "#" << data.vertex_v.size() << " vertices\n";

		for (auto it : data.uv_v)
			file << "vt " << it << "\n";

		file << "#" << data.vertex_v.size() << " vertices\n";

		for (auto it = data.face_v.begin(); it != data.face_v.end(); it += 3)
			file << "f " << *it + 1 << " " << *(it + 1) + 1 << " " << *(it + 2) + 1 << " " << "\n";

		file << "#" << data.face_v.size() / 3 << " faces\n";
		file.close();
	}
}