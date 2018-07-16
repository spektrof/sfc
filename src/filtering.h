#pragma once

#include <vector>
#include "cgal_types.h"	//conc tag and pointmap
#include "filters.h"
#include "parser.h"

#include <CGAL/compute_average_spacing.h>

typedef std::vector < Filter<tuple, filter_pointmap>* > filter_container;

void setup_filters(config_data& config, filter_container& filters)
{
	config.number_of_outlier_filter = 0;

	for (auto it : config.filter_details)
	{
		const char* type = nullptr;
		if (parser::get_char_arg(it, "type", &type))
		{
			if (!strcmp(type, "base"))
			{
				int k_neighbour;
				float outlier_limit;

				if (parser::get_int_arg(it, "k_neighbour", k_neighbour) &&
					parser::get_float_arg(it, "outlier_limit", outlier_limit))
				{
					filters.push_back(new OutlierRemoval<tuple, filter_pointmap>(k_neighbour, outlier_limit));
					config.number_of_outlier_filter++;
				}
			}
			else if (!strcmp(type, "grid"))
			{
				float cell_size;

				if (parser::get_float_arg(it, "cell_size", cell_size))
				{
					filters.push_back(new GridSimplification<tuple, filter_pointmap>(cell_size));
				}
			}
			else if (!strcmp(type, "hier"))
			{
				float max_cluster_size;
				float max_surface_variation;
				if (parser::get_float_arg(it, "max_cluster_size", max_cluster_size) &&
					parser::get_float_arg(it, "max_surface_variation", max_surface_variation))
				{
					filters.push_back(new HierarchySimplification<tuple, filter_pointmap>(max_cluster_size, max_surface_variation));
				}
			}
			else if (!strcmp(type, "jet"))
			{
			#ifdef EIGEN_ENABLED
				int k_neighbour;

				if (parser::get_int_arg(it.size(), it, "k_neighbour", k_neighbour))
				{
					filters.push_back(new JetSmoothing<tuple, filter_pointmap>(k_neighbour));
				}
			#else
				printf("Couldnt add the jetsmoother, please recompile with EIGEN_ENABLED flag and Eigen library!\n");
			#endif
			}
		}
	}
}

void delete_filters(filter_container& filters)
{
	for (auto& it : filters)
		delete it;

	filters.clear();
}

double compute_average_spacing(pointcloud_data& points)
{
	const unsigned int nb_neighbors = 6; // 1 ring
	return CGAL::compute_average_spacing<Concurrency_tag>(points.begin(), points.end(), filter_pointmap(), nb_neighbors);
}

pointcloud_data run_filtering(config_data& config, pointcloud_data& pointcloud)
{
	filter_container filters;

	setup_filters(config, filters);

	for (int it = 0; it < config.number_of_outlier_filter; ++it)
	{
		double average_spacing = compute_average_spacing(pointcloud);
		filters[it]->set_third_property(average_spacing);
		filters[it]->filter_process(pointcloud);
	}

	pointcloud_data filtered_data = pointcloud;

	for (int it = config.number_of_outlier_filter; it < filters.size(); ++it)
		filters[it]->filter_process(filtered_data);

	delete_filters(filters);

	return filtered_data;
}