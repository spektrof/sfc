#pragma once
#include "io_types.h"
#include <boost\filesystem\fstream.hpp>

namespace exporter
{
	void reindexing_because_multiple_existance_argh(export_data& data)
	{
		std::vector<uint32_t> correct_point_index;
		std::vector<uint32_t> correct_uv_index;
		std::map<Point, uint32_t> point_m;
		std::map<uv, uint32_t> uv_m;
		std::vector<Point> point_new;
		std::vector<uv> uv_new;
		
		for (auto& it : data.vertex_v)
		{
			auto res = point_m.insert(std::pair<Point, uint32_t>(it, (uint32_t)point_m.size()));
			correct_point_index.push_back(res.first->second);
			if (res.second) point_new.push_back(it);
		}

		data.vertex_v.swap(point_new);

		for (auto& it = data.face_v.begin(); it != data.face_v.end(); it++)
			it->first = correct_point_index[it->first];

		for (auto& it : data.uv_v)
		{
			auto res = uv_m.insert(std::pair<uv, uint32_t>(it, (uint32_t)uv_m.size()));
			correct_uv_index.push_back(res.first->second);
			if (res.second) uv_new.push_back(it);
		}
		data.uv_v.swap(uv_new);

		for (auto& it = data.face_v.begin(); it != data.face_v.end(); it++)
			it->second = correct_uv_index[it->second];
	}

	void export_to_obj_without_texture(char* file_name, const export_data& data)
	{
		std::string exp_file = "result/" + std::string(file_name) + ".obj";
		printf("Export to %s", exp_file.c_str());
		boost::filesystem::ofstream file(exp_file);

		file << "#" << exp_file << " with PowerCrust\n\n";
		for (auto it : data.vertex_v)
			file << "v " << std::setprecision(20) << it << "\n";

		file << "#" << data.vertex_v.size() << " vertices\n";
		for (auto it = data.face_v.begin(); it != data.face_v.end(); it+=3)
			file << "f " << it->first + 1 << " " << (it+1)->first + 1 << " " << (it + 2)->first + 1 << " " << "\n";
	
		file << "#" << data.face_v.size() / 3 << " faces\n";
		file.close();
	}

	void export_to_obj(char* file_name, export_data& data)
	{
		if (data.uv_v.empty())
		{
			export_to_obj_without_texture(file_name, data);
			return;
		}

		reindexing_because_multiple_existance_argh(data);

		std::string exp_file = "result/" + std::string(file_name) + ".obj";
		printf("Export with texture to %s\n", exp_file.c_str());

		boost::filesystem::ofstream file(exp_file);

		file << "#" << exp_file << " with PowerCrust\n\n";
		for (auto it : data.vertex_v)
			file << "v " << std::setprecision(20) << it << "\n";

		file << "#" << data.vertex_v.size() << " vertices\n";

		for (auto it : data.uv_v)
			file << "vt " << it << "\n";

		file << "#" << data.uv_v.size() << " texture coordinates\n";

		for (auto& it : data.device_related_size)
			std::cout << it << "\n";

		data.device_related_size.insert(data.device_related_size.begin(), 0);

		uint32_t start = 0;
		uint32_t end = 0;

		for (size_t obj_id = 0; obj_id < data.device_related_size.size() - 1; ++obj_id)
		{
			file << "gobject " << obj_id << "\n";

			start += data.device_related_size[obj_id];
			end += data.device_related_size[obj_id + 1];

			for (auto it = data.face_v.begin() + start; it != data.face_v.begin() + end; it += 3)
			{
				auto first_p = it->first + 1;
				auto second_p = (it + 1)->first + 1;
				auto third_p =  (it + 2)->first + 1;

				auto first_i = it->second + 1;
				auto second_i = (it + 1)->second + 1;
				auto third_i = (it + 2)->second + 1;

				file << "f ";
				file << first_p << "/" << first_i << "/" << first_p << " ";
				file << second_p << "/" << second_i << "/" << second_p << " ";
				file << third_p << "/" << third_i << "/" << third_p << "\n";
			}

			file << "#" << (end - start) / 3 << " faces\n";
		}
		
		file.close();
	}
}