#pragma once
#include "basic_parser.h"
#include "io_types.h"

#include <boost\filesystem\fstream.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

/*Note: Real number precision is important at reading*/

namespace parser	//only read what we need, nothing more
{
	struct tuple_lesser_operator
	{
		bool operator () (const tuple& lhs, const tuple& rhs) const
		{
			return lhs.first < rhs.first;
		}
	};

	struct input_property
	{
		uint32_t number_of_points;
		uint16_t object_id;		//only for obj
		int16_t device_id;
		std::vector<uv> uv_v;

		input_property() : number_of_points(0) {}
	};

	struct input_elements_obj		//only for obj - output test
	{
		std::set<tuple, tuple_lesser_operator> vertex_uvs_s;
		Point sum_of_cloud_points;

		// to display the original object
		std::vector<Point> vertex_v;
		std::vector<uint32_t> face_v;
		std::vector<uv> uv_v;

		//-----
		std::pair<std::set<tuple, tuple_lesser_operator>::iterator, bool> insert(const Point& p)
		{
			return vertex_uvs_s.insert(tuple(p, std::vector<texture_data>()));
		}
	};

	struct input_elements		//for ply
	{
		std::set<tuple, tuple_lesser_operator> vertex_uvs_s;
		Point sum_of_cloud_points;

		//TODO: calibration datas

		//-----
		std::pair<std::set<tuple, tuple_lesser_operator>::iterator, bool> insert(const Point& p)
		{
			return vertex_uvs_s.insert(tuple(p, std::vector<texture_data>()));
		}
	};

	/**/

	void parse_json(config_data& config)
	{
		boost::property_tree::ptree root;

		try
		{
			boost::property_tree::read_json("config.json", root);
		}
		catch (boost::property_tree::json_parser_error& e)
		{
			std::cerr << "Error during reading config file: " << e.what() << "\n";
			return;
		}

		if (!root.empty() && root.count("devices") > 0)
		{
			auto& devices = root.get_child("devices");
			if (!devices.empty() && devices.find("ip") != devices.not_found())
			{
				for (auto &ip : devices.get_child("ip"))
				{
					auto act = ip.second.data().c_str();
					char* tmp = (char*)malloc((strlen(act) + 1) * sizeof(char));
					strcpy(tmp, act);
					tmp[strlen(act)] = '\0';
					config.devices.push_back(tmp);
				}
			}
		}

		/*printf("Ip address of devices\n");
		for (auto& it : config.devices)
			printf("\t %s\n", it);*/

		if (!root.empty() && root.count("filters") > 0)
		{
			auto& filters = root.get_child("filters");
			if (!filters.empty() && filters.find("outlier") != filters.not_found())
			{
				for (auto &outlier : filters.get_child("outlier"))
				{
					std::vector<const char*> filter;
					filter.push_back("outlier");

					for (auto it : outlier.second)
					{
						auto act = it.first.c_str();
						char* tmp = (char*)malloc((strlen(act) + 1) * sizeof(char));
						strcpy(tmp, act);
						tmp[strlen(act)] = '\0';
						filter.push_back(tmp);

						act = it.second.data().c_str();
						tmp = (char*)malloc((strlen(act) + 1) * sizeof(char));
						strcpy(tmp, act);
						tmp[strlen(act)] = '\0';
						filter.push_back(tmp);
					}
					config.filter_details.push_back(filter);
					filter.clear();
				}
			}

			if (!filters.empty() && filters.find("simplifier") != filters.not_found())
			{
				for (auto &simplifier : filters.get_child("simplifier"))
				{
					std::vector<const char*> filter;
					filter.push_back("simplifier");

					for (auto it : simplifier.second)
					{
						auto act = it.first.c_str();
						char* tmp = (char*)malloc((strlen(act) + 1) * sizeof(char));
						strcpy(tmp, act);
						tmp[strlen(act)] = '\0';
						filter.push_back(tmp);

						act = it.second.data().c_str();
						tmp = (char*)malloc((strlen(act) + 1) * sizeof(char));
						strcpy(tmp, act);
						tmp[strlen(act)] = '\0';
						filter.push_back(tmp);
					}
					config.filter_details.push_back(filter);
					filter.clear();
				}
			}

			if (!filters.empty() && filters.find("smoother") != filters.not_found())
			{
				for (auto &smoother : filters.get_child("smoother"))
				{
					std::vector<const char*> filter;
					filter.push_back("smoother");

					for (auto it : smoother.second)
					{
						auto act = it.first.c_str();
						char* tmp = (char*)malloc((strlen(act) + 1) * sizeof(char));
						strcpy(tmp, act);
						tmp[strlen(act)] = '\0';
						filter.push_back(tmp);

						act = it.second.data().c_str();
						tmp = (char*)malloc((strlen(act) + 1) * sizeof(char));
						strcpy(tmp, act);
						tmp[strlen(act)] = '\0';
						filter.push_back(tmp);
					}
					config.filter_details.push_back(filter);
					filter.clear();
				}
			}
		}
	}

	namespace ply
	{
		void load_calibration()
		{

		}

		void read_header_element(std::istream& in, input_property& i_property)
		{
			std::string type;
			in >> type;
			if (type == "vertex")
				in >> i_property.number_of_points;
		}

		bool parse_header(std::istream* in, input_property& i_property)
		{
			static const uintptr_t header_mask = 0b111;
			uintptr_t act = 0;

			std::string line;
			while (std::getline(*in, line))
			{
				std::istringstream ls(line);
				std::string token;
				ls >> token;
				if (token == "ply" || token == "PLY" || token == "")
				{
					act = token == "ply" ? (act << 1) | 0x1 : act;
					continue;
				}
				else if (token == "format")   continue;
				else if (token == "element")
				{
					act = (act << 1) | 0x1;
					read_header_element(ls, i_property);
					continue;
				}
				else if (token == "property") continue;
				else if (token == "end_header") { act = (act << 1) | 0x1; break; }
				else return false;
			}

			return !(act ^ header_mask);
		}

		void parse_non_binary_context(std::istream* in, std::istream* tex, input_elements& input, input_property& i_property)
		{
			float x, y, z;
			std::string line;

			int i = 0;

			while (i < i_property.number_of_points && std::getline(*in, line))
			{
				std::istringstream ls(line);
				ls >> x >> y >> z;
				++i;

				auto res = input.insert(Point(x, y, z));
				std::vector<texture_data>* non_const = const_cast<std::vector<texture_data>*>(&res.first->second);

				if (res.second == true)
					input.sum_of_cloud_points = input.sum_of_cloud_points + Point(x, y, z);

				if (!tex->fail())
				{
					float u, v;
					(*tex) >> u >> v;
					non_const->push_back(texture_data(i_property.device_id, 0, uv(u, v)));
				}
				else
					non_const->push_back(texture_data(-1, 0, uv(0.0f, 0.0f)));	//-1 device id = invalid texture uv
			}
		}
	
		void parse_binary_context(std::istream* in, std::istream* tex, input_elements& input, input_property& i_property)
		{
			float xyz[3];
			unsigned char rgb[3];

			int i = 0;

			while (i < i_property.number_of_points && in->read((char*)(&xyz), sizeof(xyz)))
			{
				in->read((char *)&rgb, sizeof(rgb));

				++i;

				auto res = input.insert(Point(xyz[0], xyz[1], xyz[2]));
				std::vector<texture_data>* non_const = const_cast<std::vector<texture_data>*>(&res.first->second);

				if (res.second == true)
					input.sum_of_cloud_points = input.sum_of_cloud_points + Point(xyz[0], xyz[1], xyz[2]);

				if (!tex->fail())
				{
					float u, v;
					(*tex) >> u >> v;
					non_const->push_back(texture_data(i_property.device_id, 0, uv(u, v)));
				}
				else
					non_const->push_back(texture_data(-1, 0, uv(0.0f, 0.0f)));	//-1 device id = invalid texture uv
			}
		}
	}
	
	void load_ply(const std::string& file_name, input_elements& pre_loaded_input, bool binary_read, bool no_tex)
	{
		static uint16_t device_id = 0;

		std::string ply = file_name + ".ply";
		std::string tex = file_name + ".tex";

		boost::filesystem::ifstream ply_file(ply, std::ios_base::binary);

		if (ply_file.fail())
		{
			printf("Couldnt open %s!\n", ply.c_str());
			return;
		}

		input_property i_property;
		i_property.device_id = device_id++;

		if (!ply::parse_header(&ply_file, i_property))
		{ 
			ply_file.close();
			printf("Failed to parse header in %s\n!", ply.c_str());
			return;
		}

		boost::filesystem::ifstream tex_file(tex);
		if (!no_tex)
		{
			if (tex_file.fail())
				printf("Couldnt open %s! We wont use it!\n", ply.c_str());
		}
		else
			tex_file.close();

		/*To binary read: - we have to know the lengths - this is fine for realsense but not for kinect datas*/
		/*float buffer[3];
		unsigned char buffer2[3];

		while (ply_file.read((char*)(&buffer), sizeof(buffer)))
		{
			ply_file.read((char *)&buffer2, sizeof(buffer2));
			std::cout << buffer[0] << " " << buffer[1] << " " << buffer[2] << " " << (int)buffer2[0] << " " << (int)buffer2[1] << " " << (int)buffer2[2] << "\n";
		}*/

		if (binary_read)
			ply::parse_binary_context(&ply_file, &tex_file, pre_loaded_input, i_property);
		else
			ply::parse_non_binary_context(&ply_file, &tex_file, pre_loaded_input, i_property);

		ply_file.close();
		tex_file.close();
	}

	namespace obj
	{
		void read_vertex_element(std::istream* in, input_elements_obj& input)
		{
			double x, y, z;			//better precision
			(*in) >> x >> y >> z;

			input.vertex_v.push_back(Point(x,y,z));
			input.sum_of_cloud_points = input.sum_of_cloud_points + Point(x, y, z);
		}

		void read_texture_coordinate_element(std::istream* in, input_property& i_property)
		{
			float u, v;
			(*in) >> u >> v;
			i_property.uv_v.push_back(uv(u, v));
		}

		void read_face_element(std::istream* in, input_elements_obj& input, input_property& i_property)
	{
		std::string point_texture_ind[3];

		(*in) >> point_texture_ind[0] >> point_texture_ind[1] >> point_texture_ind[2];

		for (int i = 0; i < 3; ++i)
		{
			auto act = point_texture_ind[i];
			auto first_separator = act.find_first_of("/");
			auto last_separator = act.find_last_of("/");

			uint32_t index_of_point = atoi(act.substr(0, first_separator).c_str());
			auto res = input.insert(input.vertex_v[index_of_point - 1]);
			std::vector<texture_data>* non_const = const_cast<std::vector<texture_data>*>(&res.first->second);

			if (first_separator != std::string::npos)
			{
				uint32_t index_of_texture = atoi(act.substr(first_separator + 1,
												   last_separator == std::string::npos ? act.length() - (first_separator + 1) : 
																					   last_separator - (first_separator + 1) ).c_str());
				auto texture = texture_data(i_property.device_id, i_property.object_id, i_property.uv_v[index_of_texture - 1]);
				if (non_const->end() == std::find_if(non_const->begin(), non_const->end(),
													 [texture](const texture_data& td) 
														{ return td.texture_coordinate == texture.texture_coordinate &&
																 td.device_id == texture.device_id &&
																 td.object_id == texture.object_id; }))
				{
					non_const->push_back(texture);
				}

				input.uv_v.push_back(i_property.uv_v[index_of_texture - 1]);
			}

			input.face_v.push_back(index_of_point - 1);
		}
	}
	}

	input_elements_obj load_obj(const std::string& file_name)
	{
		static uint16_t device_id = 0;

		std::string obj = file_name + ".obj";
	
		boost::filesystem::ifstream obj_file(obj);

		if (obj_file.fail())
		{
			printf("Couldnt open %s!\n", obj.c_str());
			return input_elements_obj();
		}

		input_property i_property;
		input_elements_obj input;

		i_property.device_id = device_id++;

		std::string line;

		while (std::getline(obj_file, line))
		{
			std::istringstream ls(line);
			std::string token;
			ls >> token;

			if (token == "g")
			{
				i_property.object_id++;
				continue;
			}

			if (token == "v")
			{
				obj::read_vertex_element(&ls, input);
				continue;
			}

			if (token == "vt")
			{
				obj::read_texture_coordinate_element(&ls, i_property);
				continue;
			}

			if (token == "f")
			{
				obj::read_face_element(&ls, input, i_property);
				continue;
			}
		}

		obj_file.close();

		return input;
	}

	//-------------------------

	pointcloud_data input_rescale(input_elements& ie)
	{
		Point center = ie.sum_of_cloud_points / ie.vertex_uvs_s.size();

		float max_x = 0.0f;
		float max_y = 0.0f;
		float max_z = 0.0f;

		for (auto it : ie.vertex_uvs_s)
		{
			auto dist = it.first - center;
			if (max_x < abs(dist.x())) max_x = abs(dist.x());
			if (max_y < abs(dist.y())) max_y = abs(dist.y());
			if (max_z < abs(dist.z())) max_z = abs(dist.z());
		}

		float max = max_x > max_y ? max_x : max_y;
		max = max > max_z ? max : max_z;

		Point scale(max == 0.0f ? 1.0f : 1.0f / max, max == 0.0f ? 1.0f : 1.0f / max, max == 0.0f ? 1.0f : 1.0f / max);

		pointcloud_data pcd;

		for (auto& it : ie.vertex_uvs_s)
		{
			auto new_point = scale * (it.first - center);
			pcd.push_back(tuple(new_point, it.second));
		}

		return pcd;
	}
}