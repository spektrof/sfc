#include "power_crust_types.h"
#include <queue>

void face::correct_edge_orientations()
{
	if (edges.empty()) return;

	std::map<face*, unsigned int>::iterator location;

	location = edges[0]->related_faces.find(this);

	std::vector<edge*> edges_o;
	edges_o.push_back(edges[0]);
	std::vector<edge*> edges_cpy(edges.begin() + 1, edges.end());

	for (size_t i = 0; i < edges.size() - 1; ++i)
	{
		Point actual_last_point = location->second == 0 ? edges_o[i]->data.second : edges_o[i]->data.first;

		unsigned int new_orientation;
		auto next = std::find_if(edges_cpy.begin(), edges_cpy.end(), [&actual_last_point, &new_orientation](const edge* e)->bool
		{
			if (e->data.first == actual_last_point)
			{
				new_orientation = 0;
				actual_last_point = e->data.second;
				return true;
			}
			else if (e->data.second == actual_last_point)
			{
				new_orientation = 1;
				actual_last_point = e->data.first;
				return true;
			}
			return false;
		});

		if (next == edges_cpy.end())
		{
			std::cout << "FAIL not founded pair\n";
			return;
		}

		location = (*next)->related_faces.find(this);
		location->second = new_orientation;

		edges_o.push_back(*next);
		edges_cpy.erase(next);
	}

	std::vector<edge*>(edges_o).swap(edges);

	//-------------------- TEST

	/*for (auto& it : edges)
	{
	if (it->related_faces[this] == 0)
	qDebug() << it->edge.first.first << " -> " << it->edge.second.first;
	else
	qDebug() << it->edge.second.first << " -> " << it->edge.first.first;
	}*/
}

void face::correct_boundary_orientation()
{
	//qDebug() << "Correct Boundary orientation";
	std::queue<face*> face_list;
	std::set<face*> progress_faces;
	std::set<face*> done_faces;

	progress_faces.insert(this);
	done_faces.insert(this);

	face_list.push(this);

	while (!face_list.empty())
	{
		face* act_face = face_list.front();
		face_list.pop();

		auto act_face_edges = act_face->edges;

		for (auto& face_edge : act_face_edges)
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
					std::cout << "ERR: CORRECT ORIENTATION ON BOUND IS BAAD\n";

			}

			if (next_boundary_face != nullptr)
			{
				if (done_faces.find(next_boundary_face) != done_faces.end())
				{
					if (next_face_orientation == orientation_on_actual_face)
					{
						std::cout << "ERR: Bad orientation!\n";
					}
					continue;
				}

				if (next_face_orientation < 0 || orientation_on_actual_face < 0)
				{
					std::cout << "ERR: orientation didnt set properly\n";
					continue;
				}

				if (next_face_orientation == orientation_on_actual_face)
					next_boundary_face->swap_edge_orientations();

				face_list.push(next_boundary_face);
				done_faces.insert(next_boundary_face);
			}
		}

	}
	std::cout << "Correct Boundary orientation END\n";
}