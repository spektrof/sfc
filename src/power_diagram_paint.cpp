#include "power_diagram.h"
#include <queue>

export_data PowerDiagram::get_triangled_mesh_without_texture()
{
	export_data res;
	
	if (power_faces.empty()) return res;

	std::map<Point*, unsigned int> point_map;	// to get indicies of triangles
	int point_index = 0;

	face_t* start_face;

	for (auto& face : power_faces)
	{
		if (face->face.is_power_crust_face())
		{
			start_face = &face->face;
			break;
		}
	}

	std::queue<face_t*> face_list;
	std::set<face_t*> done_faces;

	done_faces.insert(start_face);

	face_list.push(start_face);

	while (!face_list.empty())
	{
		face_t* act_face = face_list.front();
		face_list.pop();

		std::set<Point*> face_points = act_face->get_face_points();

		for (auto& point : face_points)
		{
			auto pos = point_map.insert(std::pair<Point*, unsigned int>(point, point_index));
			if (pos.second == true)
			{
				res.vertex_v.push_back(*point);
				point_index++;
			}
		}

		auto face_edges = act_face->edges;

		for (auto& face_edge : face_edges)
		{
			face_t* next_boundary_face = nullptr;
			int orientation_on_actual_face = -1;
			int next_face_orientation = -1;
			for (auto& related_face : face_edge->related_faces)
			{
				if (!related_face.first->is_power_crust_face()) continue;
				if (related_face.first == act_face)
				{
					orientation_on_actual_face = related_face.second;
					continue;
				}

				if (next_boundary_face == nullptr)
				{
					next_boundary_face = related_face.first;
					next_face_orientation = related_face.second;
				}
				else
					printf("CORRECT ORIENTATION ON BOUND IS BAAD\n");

			}

			if (next_boundary_face != nullptr)
			{
				if (done_faces.find(next_boundary_face) != done_faces.end())
				{
					if (next_face_orientation == orientation_on_actual_face)
					{
						printf("ERR: Bad orientation!\n");
					}
					continue;
				}

				if (next_face_orientation < 0 || orientation_on_actual_face < 0)
				{
					printf("ERR: orientation didnt set properly\n");
					continue;
				}

				face_list.push(next_boundary_face);
				done_faces.insert(next_boundary_face);
			}
		}

		// partition the face into triangles
		auto edge_orientation = face_edges[0]->related_faces[act_face];

		Point* edge_point = edge_orientation == 0 ? face_edges[0]->edge.first.second : face_edges[0]->edge.second.second;
		auto first_triangle_index = point_map[edge_point];

		face_edges = std::vector<edge_tt*>(face_edges.begin() + 1, face_edges.end() - 1);	// we dont need the first and last edge

		if (face_edges.size() == 0) printf("ERR: Face contains only 2 edge\n");

		for (auto& edge : face_edges)
		{
			res.face_v.push_back(std::pair<uint32_t, uint32_t>(first_triangle_index, 0));

			edge_orientation = edge->related_faces[act_face];
			edge_point = edge_orientation == 0 ? edge->edge.first.second : edge->edge.second.second;
			auto pos = point_map[edge_point];
			res.face_v.push_back(std::pair<uint32_t, uint32_t>(pos, 0));

			edge_point = edge_orientation == 0 ? edge->edge.second.second : edge->edge.first.second;
			pos = point_map[edge_point];
			res.face_v.push_back(std::pair<uint32_t, uint32_t>(pos, 0));
		}
	}
	return res;
}