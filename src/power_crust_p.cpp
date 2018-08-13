#include "power_crust.h"
#include "priority_queue.h"
#include "utils.h"

face_simplex PowerCrust::get_power_crust_faces(point_radius_v& w_p, const Box& box)
{
	int ct = 0;
	for (auto& it : w_p)
	{
		ct += it.second.surface_points.size();
	}
	printf("valami %d\n", ct);

	auto start = hrclock::now();
	auto process = hrclock::now();

	std::set<Point> box_points;
	Box pole_bound = update_box(w_p);		//we need to calculate the actual bound as the pole could exceed the original box size

	add_box_points(pole_bound, box_points, w_p);	//azért, hogy a külsõ cellák duálisai ne száljanak el -> mybe check again

#ifdef TBB_ENABLED
	regular_triangulation::Lock_data_structure locks(CGAL::Bbox_3(pole_bound.get_xmin(), pole_bound.get_ymin(), pole_bound.get_zmin(), pole_bound.get_xmax(), pole_bound.get_ymax(), pole_bound.get_zmax()), 50);
	regular_triangulation R(w_p.begin(), w_p.end(), &locks);
#else
	regular_triangulation R(w_p.begin(), w_p.end());
#endif
	assert(R.is_valid());

	w_p.clear();

	/*************************************
	*	Calculate 	*
	**************************************/

	face_simplex fs;
	power_faces_off_data pod;

	make_power_cells(R, box_points, R.finite_vertices_begin(), R.finite_vertices_end(), &pod);

	//to get those face which belongs to only 1 cell!
	int box_ind = 0;
	for (auto& it : pod.box_v_its)
	{
		box_ind--;
		//get incident cells
		std::vector<regular_cell_handle> inc_cells;
		R.incident_cells(it, std::back_inserter(inc_cells));

		//-----------------------------------------------
		//Precalculations
		auto inc_c_size = inc_cells.size();

		std::set<regular_cell_handle> inc_cell_map;	//for finding cell in logM

		for (auto& it : inc_cells)
			inc_cell_map.insert(it);

		int act_pc_index = box_ind;	//wtf was here

		for (size_t cit = 0; cit < inc_c_size; ++cit)
		{
			regular_cell_handle act_cell = inc_cells[cit];
			int act_index = act_cell->info().index;

			if (act_index == -1) continue;

			Point* act_dual = act_cell->info().dual;

			//-----------------------------------------------
			//calculate edges
			for (int i = 0; i < 4; ++i)
			{
				regular_cell_handle tmp_cell = act_cell->neighbor(i);
				auto other_cell_candidate = inc_cell_map.find(tmp_cell);

				if (other_cell_candidate != inc_cell_map.end())	//both cell_handle are related to the actual point
				{
					int tmp_index = tmp_cell->info().index;

					if (tmp_index == -1) continue;

					Point* tmp_dual = tmp_cell->info().dual;

					//calculate the edge
					edge e = act_index < tmp_index ? edge(*act_dual, *tmp_dual) : edge(*tmp_dual, *act_dual);

					auto edg_p = pod.edge_cell_ids_m.find(&e);
					if (edg_p == pod.edge_cell_ids_m.end()) continue;

					//create relations between the cells based on the edges
					for (auto& related_power_cell : edg_p->second)
					{
						auto res = pod.inf_ptrs[related_power_cell]->neighbour_cell_edges.insert(std::pair<int, std::set<edge*>>(act_pc_index, std::set<edge*>()));
						res.first->second.insert(edg_p->first);
					}
				}
			}
		}
	}

	printf("Power Diagram base is created with %u edge, %u faces\n", (uint32_t)pod.edge_cell_ids_m.size(), (uint32_t)fs.faces.size());

	printf("Power diagram is calculated under ended in %u\n", (uint32_t)boost::chrono::duration_cast<milliseconds>(hrclock::now() - process).count());

	//-------------------------------------
	pod.neigh_graph_m = get_neighbour_graph(pod.inf_ptrs, pod.surf_polarballs_m);
	printf("Power Diagram cells's neighbours are calculated %zu\n", pod.neigh_graph_m.size());

	label_polar_balls(pod.neigh_graph_m, pod.surf_polarballs_m, box);

	get_power_faces(pod.neigh_graph_m, fs, pod.inf_ptrs);

	return fs;
}

void PowerCrust::make_power_cells(regular_triangulation& R, const std::set<Point>& box_points, regular_finite_vertices_iterator vit, regular_finite_vertices_iterator end, power_faces_off_data* pod)
{
	for (; vit != end; vit++)
	{
		//skip box points
		if (box_points.count(vit->point().point()) == 1)
		{
			pod->box_v_its.push_back(vit);
			continue;
		}

		//-----------------------------------------------
		//get incident cells
		std::vector<regular_cell_handle> inc_cells;
		R.incident_cells(vit, std::back_inserter(inc_cells));

		//check for empty
		if (inc_cells.empty())
			continue;

		//-----------------------------------------------
		//Precalculations
		auto inc_c_size = inc_cells.size();

		std::set<regular_cell_handle> inc_cell_map;	//for finding cell in logM

		for (auto& it : inc_cells)
			inc_cell_map.insert(it);

		unsigned int act_pc_index = pbs.size();

		PolarBall* pb = new PolarBall(vit->point().point(), sqrt(vit->point().weight()), vit->info().surface_points, act_pc_index);
		pbs.push_back(pb);
		for (auto& sp : vit->info().surface_points)
		{
			auto loc = pod->surf_polarballs_m.insert(std::pair<Point, std::vector<PolarBall*>>(sp, std::vector<PolarBall*>()));
			loc.first->second.push_back(pb);
		}

		//-----------------------------------------------

		std::set<edge*> related_edges_of_actual_inc_cells;		//for neighbours calculation

		pod->inf_ptrs.push_back(&vit->info());

		//-----------------------------------------------

		for (size_t cit = 0; cit < inc_c_size; ++cit)
		{
			regular_cell_handle act_cell = inc_cells[cit];

			auto act_info = &act_cell->info();
			set_cell_info(R, act_cell, act_info);
			unsigned int act_index = act_info->index;
			Point* act_vert_ptr = act_info->dual;

			//-----------------------------------------------
			//calculate edges

			for (int i = 0; i < 4; ++i)
			{
				regular_cell_handle tmp_cell = act_cell->neighbor(i);
				auto other_cell_candidate = inc_cell_map.find(tmp_cell);

				if (other_cell_candidate != inc_cell_map.end())	//both cell_handle are related to the actual point
				{
					//calculate the dual if its not ready yet
					auto tmp_info = &tmp_cell->info();
					set_cell_info(R, tmp_cell, tmp_info);
					unsigned int tmp_index = tmp_info->index;
					Point* tmp_vert_ptr = tmp_info->dual;

					//calculate the edge
					std::pair<std::map<Point*, edge>::iterator, bool> edge_it_ref;
					edge e = act_index < tmp_index ? edge(*act_vert_ptr, *tmp_vert_ptr) : edge(*tmp_vert_ptr, *act_vert_ptr);	//csak akkor készítsük el ha kell!
					if (act_index < tmp_index) 
						edge_it_ref = act_info->edge.insert(std::pair<Point*, edge>(tmp_vert_ptr, e));
					else
						edge_it_ref = tmp_info->edge.insert(std::pair<Point*, edge>(act_vert_ptr, e));

					//put the edge into the edge cell id map
					auto ee_res = pod->edge_cell_ids_m.insert(std::pair<edge*, std::set<uint32_t>>(&edge_it_ref.first->second, std::set<uint32_t>()));
					auto edge_element_from_map = *ee_res.first;

					//save the edges which belong to the actual cell
					related_edges_of_actual_inc_cells.insert(edge_element_from_map.first);

					//create relations between the cells
					for (auto& related_power_cell : edge_element_from_map.second)
					{
						if (related_power_cell < act_pc_index)
						{
							auto res = pod->inf_ptrs[related_power_cell]->neighbour_cell_edges.insert(std::pair<int, std::set<edge*>>(act_pc_index, std::set<edge*>()));
							res.first->second.insert(&edge_it_ref.first->second);
						}
						else
						{
							auto res = pod->inf_ptrs[act_pc_index]->neighbour_cell_edges.insert(std::pair<int, std::set<edge*>>(related_power_cell, std::set<edge*>()));
							res.first->second.insert(&edge_it_ref.first->second);
						}
					}
				}
			}
		}

		//add the new cell index to the edges list
		for (auto& it : related_edges_of_actual_inc_cells)
		{
			pod->edge_cell_ids_m[it].insert(act_pc_index);
		}
	}
}

void PowerCrust::set_cell_info(regular_triangulation& R, regular_cell_handle cell, cell_inf_power* info)
{
	static int ind = 0;
	if (info->index == -1)
	{
		info->index = ind;
	//	std::cout << info->index << "\n";
		ind++;
		info->dual = new Point(R.dual(cell));
	}
}

std::map<PolarBall*, std::set<PolarBall*>> PowerCrust::get_neighbour_graph(const std::vector<point_inf*>& p_inf_v, std::map<Point, std::vector<PolarBall*>>& surf_p_balls)
{
	std::map<PolarBall*, std::set<PolarBall*>> neigh_graph;
	uint32_t i = 0;
	
	for (auto& inf : p_inf_v)
	{
		PolarBall* cell_ptr = pbs[i];
		++i;

		std::set<PolarBall*> actual_neighs_ptr;

		//adding neighbours by common face
		auto actual_neighs = inf->neighbour_cell_edges;
		for (auto& neigh : actual_neighs)
		{
			if (neigh.first == (i - 1)) printf("ajaja\n");
			if (neigh.second.size() < 3 /*&& is_coplanar*/) continue;

			actual_neighs_ptr.insert(pbs[neigh.first]);
		}

		//adding neighbours by common surf point
		std::vector<Point> related_surf_points = inf->surface_points;
		int test = surf_p_balls.size();
		for (auto surf_point : related_surf_points)
		{
			auto pole_pairs = surf_p_balls[surf_point];	//fix size ( 2 )

			if (pole_pairs[0] != cell_ptr && pole_pairs.size() == 2 && pole_pairs[1] != cell_ptr)
				printf("\t\tERR: BAD NEIGHBOUR CALCULATION\n");

			//surf point base edge
			if (pole_pairs[0] != cell_ptr)
				actual_neighs_ptr.insert(pole_pairs[0]);
			else if (pole_pairs.size() == 2)
				actual_neighs_ptr.insert(pole_pairs[1]);
		}
		if (surf_p_balls.size() != test)
			printf("BAAAD\n");

		neigh_graph.insert(std::pair<PolarBall*, std::set<PolarBall*>>(cell_ptr, actual_neighs_ptr));
	}

	return neigh_graph;
}

void PowerCrust::label_polar_balls(std::map<PolarBall*, std::set<PolarBall*>>& neigh_graph, std::map<Point, std::vector<PolarBall*>>& neigh_by_surf, const Box& bounded)
{
	auto start = hrclock::now();

	typedef priority_queue<PolarBall> priority_ball_queue;
	priority_ball_queue pri_ball_q;

	for (auto &it : neigh_graph)
	{
		it.first->set_in_value(0);
		it.first->set_out_value(0);
		pri_ball_q.push(0.0f, it.first);
	}
	
	/*************************************
	*	Third step:
	Setting outer poles	based on the bound box*
	**************************************/
	printf("\tSetting outlier poles\n");

	priority_ball_queue tmp_q = pri_ball_q;

	int out_p = 0;
	while (!tmp_q.empty())
	{
		PolarBall* tmp = tmp_q.pop();

		Point pole_point = tmp->get_point();
		if (!bounded.is_inside(pole_point.x(), pole_point.y(), pole_point.z()))
		{
			//auto element_in_priq = pri_ball_q.find(tmp);
			if (pri_ball_q.count(tmp) > 1)
			{
				printf("\t\tERR: Invalid Priority queue\n");
			}

			tmp->set_out_value(1.0f);
			pri_ball_q.update(tmp->get_heap_index());
			out_p++;
		}
	}

	/*************************************
	*	Forth step:
	Labeling	*
	**************************************/
	printf("\tStart labeling\n");

	float prev_p = 1.1f;
	while (!pri_ball_q.empty())
	{
		/*************************************
		*	Pop high priority element	: */

		prev_p = pri_ball_q.top_prior();

		PolarBall* p = pri_ball_q.pop();
		p->set_heap_index(-1);

		if (prev_p < pri_ball_q.highest_prior())
		{
			printf("\tPriority queue FAIL %.4f - %.4f \n", prev_p, pri_ball_q.highest_prior());
		}

		if (pri_ball_q.count(p) > 0)
			printf("\tMore than one occur \n");


		float tmp_p;
		float in_val = p->get_in_value();
		float out_val = p->get_out_value();

		/*************************************
		*	Set its flag	: */

		if (in_val > out_val)
		{
			p->set_flag(0);
			tmp_p = in_val;
		//	printf("\t\t\tIts an Inner Pole\n");
		}
		else
		{
			p->set_flag(1);
			tmp_p = out_val;
		//	printf("\t\t\tIts an Outer Pole\n");
		}

		/*************************************
		*	Calculate new weights with Beta weight	: */

		std::vector<Point> tmp_surf_points = p->get_surf_points();
		//qDebug() << "\t\tNumber of related Surf p: " << tmp_surf_points.size();

		for (std::vector<Point>::iterator related_surf_p = tmp_surf_points.begin();
			related_surf_p != tmp_surf_points.end();		++related_surf_p)
		{
			std::vector<PolarBall*> tmp_related_polarballs = neigh_by_surf[*related_surf_p];

			if (tmp_related_polarballs.size() != 2)
			{
				printf("\tERR: Wrong related polarball ptrs\n");
				if (tmp_related_polarballs[0] != p) printf("\tERR: STRANGE - pole count thing to surface point\n");
				continue;
			}
			PolarBall* opposite_pole = tmp_related_polarballs[0] != p ? tmp_related_polarballs[0] : tmp_related_polarballs[1];
			// We got the opposite pole based on the current surface_point

			if (opposite_pole->get_heap_index() == -1) continue;
			//	qDebug() << "\t\tFINDING: " << opposite_pole;
			/*auto opp_position = pri_ball_q.find(opposite_pole);
			if (opp_position != pri_ball_q.end()) { }
			else
			{
			continue;
			}*/

			float w_pq = p->get_beta_weight(opposite_pole, *related_surf_p);
			unsigned int opp_flag = p->get_opposite_flag();
			float new_val = max_prior(tmp_p * w_pq, opposite_pole->get_value_by_flag(opp_flag));


			opposite_pole->set_value_by_flag(opp_flag, new_val);

			pri_ball_q.update(opposite_pole->get_heap_index());
		}

		/*************************************
		*	Calculate new weights with Alpha weight	: */

		std::set<PolarBall*> related_neighbours = neigh_graph[p];
		//qDebug() << "Neighbours ";
		for (std::set<PolarBall*>::iterator it = related_neighbours.begin(); it != related_neighbours.end(); ++it)
		{
			if ((*it)->get_heap_index() == -1) continue;

			float w_pq = p->get_alpha_weight(*it);
			unsigned int p_flag = p->get_flag();
			float new_val = max_prior(tmp_p * w_pq, (*it)->get_value_by_flag(p_flag));

			(*it)->set_value_by_flag(p_flag, new_val);

			pri_ball_q.update((*it)->get_heap_index());

		}
	}

	/*************************************
	*	Last step:
	Labeling	done	*
	**************************************/

	printf("Labeling ended in: %lld\n", boost::chrono::duration_cast<milliseconds>(hrclock::now() - start).count());
}

void PowerCrust::get_power_faces(std::map<PolarBall*, std::set<PolarBall*>>& neigh_graph, face_simplex& fs, const std::vector<point_inf*> inf_ptrs)
{
	printf("Power Crust calculation\n");

	std::map<edge, edge*> edges;

	for (auto& node : neigh_graph)
	{
		auto act_pb_index = node.first->get_pb_index();

		for (auto& neigh : node.second)
		{
			if (act_pb_index > neigh->get_pb_index()) continue;

			int border_face_check = node.first->is_inner_pole() + neigh->is_inner_pole();

			if (border_face_check == 1)		//this will be a boundary face, correct its orientation
			{
				auto lower_index = act_pb_index < neigh->get_pb_index() ? act_pb_index : neigh->get_pb_index();
				auto upper_index = act_pb_index == lower_index ? neigh->get_pb_index() : act_pb_index;

				auto e_s = inf_ptrs[lower_index]->neighbour_cell_edges.find(upper_index);
				if (e_s == inf_ptrs[lower_index]->neighbour_cell_edges.end()) continue;

				std::vector<edge*> face_edges;

				for (auto& e : e_s->second)
				{
					edge* e_ptr = new edge(e->data);
					auto res = edges.insert(std::pair<edge, edge*>(*e, e_ptr));
					if (res.second)
						fs.edges.push_back(e_ptr);
					face_edges.push_back(res.first->second);
				}
				if (face_edges.empty())
				{
					std::cout << "empty\n";
				}
				
				fs.faces.push_back(new face());
				fs.faces.back()->add_edges(face_edges, 0);
				fs.faces.back()->correct_edge_orientations();
				fs.faces.back()->set_power_crust_flag(true);
				if (fs.faces.back()->edges.empty()) std::cout << "lmao\n";

			}
		}
		
	}
	for (auto& edge : edges)
	{
		for (auto face_ : edge.second->related_faces)
		{
			if (face_.first->edges.empty())
				std::cout << "lmao\n";
		}
	}
	printf("We have %u power crust face!\n", (uint32_t)fs.faces.size());

	correct_unoriented_power_crust_faces(fs.faces);
}

void PowerCrust::correct_unoriented_power_crust_faces(std::vector<face*>& unoriented_face_ptrs)
{
	printf("\tCorrect face orientations\n");

	face* starter_correct_face = nullptr;
	float distance_from_correct_face = FLT_MAX;
	Point outer_point = Point(3.0f, 3.0f, 3.0f);
	Vector surface_normal;
	Vector outer_to_face;

	for (auto& power_face : unoriented_face_ptrs)
	{
		// we already oriented the edges one way
		auto edge1 = power_face->edges[0];
		auto edge2 = power_face->edges[1];

		// fill the first 3 point depends on the orienation
		Point* face_first_point;
		Point* face_second_point;
		Point* face_third_point;

		if (edge1->related_faces[power_face] == 0)
		{
			face_first_point = &edge1->data.first;
			face_second_point = &edge1->data.second;
		}
		else
		{
			face_first_point = &edge1->data.second;
			face_second_point = &edge1->data.first;
		}

		if (edge2->related_faces[power_face] == 0)
			face_third_point = &edge2->data.second;
		else
			face_third_point = &edge2->data.first;

		// calculate the actual normal
		Vector act_normal = CGAL::cross_product(*face_second_point - *face_first_point, *face_third_point - *face_first_point);
		Vector act_outer_to_face_v = *face_first_point - outer_point;
		float length_of_act_outer_to_face_v = act_outer_to_face_v.squared_length();

		if (length_of_act_outer_to_face_v < distance_from_correct_face)
		{
			distance_from_correct_face = length_of_act_outer_to_face_v;
			outer_to_face = act_outer_to_face_v;
			surface_normal = act_normal;
			starter_correct_face = power_face;
		}
	}

	// start the correction from the closest face to bound_point
	if (starter_correct_face != nullptr)
	{
		//	qDebug() << CGAL::scalar_product(outer_to_face, surface_normal);
		if (CGAL::scalar_product(outer_to_face, surface_normal) > 0)	// if thats obtuse angle then swap actual face orientation
			starter_correct_face->swap_edge_orientations();
		std::cout << "start\n";
		starter_correct_face->correct_boundary_orientation();			// we can go around the whole boundary because of the power_crust_flag
	}
	else
		printf("ERR: during correct unoriented normals\n");
}
