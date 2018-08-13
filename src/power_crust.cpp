#include "power_crust.h"
#include <queue>

export_data PowerCrust::get_triangled_mesh_from_face_simplex(std::vector<face*>& power_faces)
{
	export_data res;

	if (power_faces.empty()) return res;

	std::map<Point*, unsigned int> point_map;	// to get indicies of triangles
	int point_index = 0;

	face* start_face;

	for (auto& face : power_faces)
	{
		if (face->is_power_crust_face())	//unnecess
		{
			start_face = face;
			break;
		}
	}

	std::queue<face*> face_list;
	std::set<face*> done_faces;

	done_faces.insert(start_face);

	face_list.push(start_face);

	while (!face_list.empty())
	{
		face* act_face = face_list.front();
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
			face* next_boundary_face = nullptr;
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

		Point* edge_point = edge_orientation == 0 ? &face_edges[0]->data.first : &face_edges[0]->data.second;
		auto first_triangle_index = point_map[edge_point];

		face_edges = std::vector<edge*>(face_edges.begin() + 1, face_edges.end() - 1);	// we dont need the first and last edge

		if (face_edges.size() == 0) printf("ERR: Face contains only 2 edge\n");

		for (auto& edge : face_edges)
		{
			res.face_v.push_back(std::pair<uint32_t,uint32_t>(first_triangle_index, 0));

			edge_orientation = edge->related_faces[act_face];
			edge_point = edge_orientation == 0 ? &edge->data.first : &edge->data.second;
			auto pos = point_map[edge_point];
			res.face_v.push_back(std::pair<uint32_t, uint32_t>(pos, 0));

			edge_point = edge_orientation == 0 ? &edge->data.second : &edge->data.first;
			pos = point_map[edge_point];
			res.face_v.push_back(std::pair<uint32_t, uint32_t>(pos, 0));
		}
	}
	return res;
}