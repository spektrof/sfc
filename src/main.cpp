/*Usage:
	exe
	options:
	pc : using Power Crust algorithm
	todo: pole omitting
	-d <file_index> : using devices - identified by IP addresses from config.json
	-obj <file_name> : using data from one .obj file
	-t      : use texturing
		TODO: epsilons
	-o file_name : output is a .obj file
	-b : binary files
	-notex : there isnt any tex file -> just do reconstruction
*/

#include <stdio.h>
#include "parser.h"
#include "export.h"
#include "filtering.h"
#include "power_crust.h"
#include "texturer.h"

extern pointcloud_data run_filtering(config_data& config, pointcloud_data& pointcloud);

struct options
{};

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		printf("Too few arguments, pls todo");
		return 0;
	}

	//get_options(argc, argv);

	config_data config;
	parser::parse_json(config);

	char* in_file_name = nullptr;
	parser::input_elements input;
	export_data output;
	
	bool binary_data = parser::get_option_arg(argc, argv, "-b");
	bool use_texture = parser::get_option_arg(argc, argv, "-notex");
	bool oc = parser::get_option_arg(argc, argv, "-orig");
	bool th = parser::get_option_arg(argc, argv, "-th");

	//Parse input file
	if (parser::get_char_arg(argc, argv, "-d", &in_file_name))
	{
		for (auto it : config.devices)
		{
			char act_device[100];
			strcpy(act_device, in_file_name);
			strcat(act_device, it);
			parser::load_ply(std::string(act_device), input, binary_data, use_texture);
		}
	}
	else if (parser::get_char_arg(argc, argv, "-obj", &in_file_name))
	{
		parser::input_elements_obj ieo = parser::load_obj(std::string(in_file_name));	//we could show it with opengl - we will take this out
		input.sum_of_cloud_points = ieo.sum_of_cloud_points;
		input.vertex_uvs_s = ieo.vertex_uvs_s;
	}

	if (input.vertex_uvs_s.empty())
	{
		printf("Couldnt read any point, aborted process!\n");
		return 0;
	}

	pointcloud_data noise_free_pointcloud = parser::input_rescale(input);	//it will be noisefree after filtering!
	
	//Filtering to reduce the noise
	pointcloud_data filtered_pointcloud = run_filtering(config, noise_free_pointcloud);		//TODO: we just need the points (surface points)
	
	printf("%u point remained after filtering from %u points.\n", (uint32_t)filtered_pointcloud.size(), (uint32_t)noise_free_pointcloud.size());

	//Use one reconstructor algorithm
	if (!strcmp(argv[1], "pc"))
	{
		std::vector<Point> surface_points;
		for (auto it : filtered_pointcloud) surface_points.push_back(it.first);

		PowerCrust pc;
		output = pc.calc(surface_points, oc, th);
	}
	else
	{
		printf("Invalid algorithm, process interrupted!\n");
		return 0;
	}

	//Surface texturing if needed
	if (parser::get_option_arg(argc, argv, "-t"))
	{
		Texturer texture_calculator;
		texture_calculator.set_required_data(std::vector<Point>(), config.devices.size()); /*TODO!*/

		texture_calculator.add_tex_or_color_info(output, noise_free_pointcloud);
	}

	printf("Exporting\n");
	//Export to obj
	char* out_file_name = nullptr;
	parser::get_char_arg(argc, argv, "-o", &out_file_name);

	exporter::export_to_obj(out_file_name == nullptr ? "output" : out_file_name, output);
	
	//TODO: delete ptrs, i.e. in config char*
	std::cin.get();
	return 0;
}