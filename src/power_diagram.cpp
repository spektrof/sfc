#include "power_diagram.h"
#include "utils.h"

/* ****************************
Diagram calculation main functions
**************************** */

void PowerDiagram::calc_diagram(const std::vector<std::pair<Pole2, Point> >& weighted_points)
{
	/*************************************
	*	Clear	*
	**************************************/
	for (auto& it : cells)
		delete it;
	cells.clear();
	power_vertices.clear();
	power_edges.clear();
	power_faces.clear();
	pole_map.clear();

	/*************************************
	*	Box update & prepare for Weighted Points	*
	**************************************/

	printf("\tInput: %u poles\n", (uint32_t)weighted_points.size());

	std::map<Point, std::pair<float, std::vector<Point>>> pole_point_longest_radius_pairs;

	auto start = hrclock::now();
	auto process = hrclock::now();

	//atlag
	std::map<Point, std::vector<float>> tmp_rads;		// Point and the related rads
	for (auto& it : weighted_points)
	{
		std::pair<std::map<Point, std::vector<float>>::iterator, bool> rads;
		rads = tmp_rads.insert(std::pair<Point, std::vector<float> >(*it.first.center, std::vector<float>()));
		rads.first->second.push_back(it.first.radius);

		// Point ,  average of radius, suface points
		std::pair<std::map<Point, std::pair<float, std::vector<Point>>>::iterator, bool> pair_it;
		pair_it = pole_point_longest_radius_pairs.insert(std::pair<Point, std::pair<float, std::vector<Point>> >(*it.first.center, std::pair<float, std::vector<Point>>(it.first.radius, std::vector<Point>())));
		if (pair_it.second == false)
		{
			float sum = 0.0f;
			for (auto s : rads.first->second) sum += s;

			pair_it.first->second.first = sum / (float)rads.first->second.size();
		}

		pair_it.first->second.second.push_back(it.second);
	}
	tmp_rads.clear();
	
	printf("\t\texact: %u  different poles!\n", (uint32_t)pole_point_longest_radius_pairs.size());
	
	/*************************************
	*	Checks	*
	**************************************/

	//Checking surf points for poles (added more than once)
	int ct = 0;
	for (auto it : pole_point_longest_radius_pairs)
	{
		std::set<Point> tmp;
		for (auto po_it : it.second.second)
			tmp.insert(po_it);

		ct += it.second.second.size();
		if (tmp.size() != it.second.second.size())
			printf("\tERR: Surf point added more than once!\n");
	}

	if (ct != weighted_points.size())
		printf("\tERR: We lost some surf point pole pair\n");

	/*************************************
	*	Make weighted points and do Triangulation	*
	**************************************/
	
	std::vector<std::pair<weighted_point, point_inf2>> w_p;
	for (auto it : pole_point_longest_radius_pairs)
		w_p.push_back(std::pair<weighted_point, point_inf2>(weighted_point(it.first, it.second.first* it.second.first), point_inf2(it.second.second)));	//d*d - r -> az alap sugár négyzete kell


	pole_point_longest_radius_pairs.clear();

	//qDebug() << "\t\tBox stuff";

	std::set<Point> box_points;
	Box pole_bound = update_box(weighted_points);		//we need to calculate the actual bound as the pole could exceed the original box size
	add_box_points(pole_bound, box_points, w_p);

	process = hrclock::now();

#ifdef TBB_ENABLED
	regular_triangulation::Lock_data_structure locks(CGAL::Bbox_3(pole_bound.get_xmin(), pole_bound.get_ymin(), pole_bound.get_zmin(), pole_bound.get_xmax(), pole_bound.get_ymax(), pole_bound.get_zmax()), 50);
	regular_triangulation R(w_p.begin(), w_p.end(), &locks);
#else
	regular_triangulation2 R(w_p.begin(), w_p.end());
#endif
	w_p.clear();

	assert(R.is_valid());

	/*************************************
	*	Calculate Power Cellas	*
	**************************************/
	
	process = hrclock::now();

	std::map<edge_tt, edge_identifier> edge_cell_ids_map;
	std::vector<point_inf2*> inf_ptrs;

	std::vector<regular_finite_vertices_iterator2> box_v_its;
	
	make_power_cells(R, box_points, R.finite_vertices_begin(), R.finite_vertices_end(), &box_v_its, &inf_ptrs, &edge_cell_ids_map);

	//to get those face which belongs to only 1 cell!
	int box_ind = 0;
	for (auto& it : box_v_its)
	{
		box_ind--;
		//get incident cells
		std::vector<regular_cell_handle2> inc_cells;
		R.incident_cells(it, std::back_inserter(inc_cells));

		//-----------------------------------------------
		//Precalculations
		auto inc_c_size = inc_cells.size();

		std::set<regular_cell_handle2> inc_cell_map;	//for finding cell in logM

		for (auto& it : inc_cells)
			inc_cell_map.insert(it);

		unsigned int act_pc_index = box_ind;

		for (size_t cit = 0; cit < inc_c_size; ++cit)
		{
			regular_cell_handle2 act_cell = inc_cells[cit];
			int act_index = act_cell->info().index;

			if (act_index == -1) continue;

			Point* act_dual = act_cell->info().dual;

			//-----------------------------------------------
			//calculate edges
			for (int i = 0; i < 4; ++i)
			{
				regular_cell_handle2 tmp_cell = act_cell->neighbor(i);
				auto other_cell_candidate = inc_cell_map.find(tmp_cell);

				if (other_cell_candidate != inc_cell_map.end())	//both cell_handle are related to the actual point
				{
					int tmp_index = tmp_cell->info().index;

					if (tmp_index == -1) continue;

					Point* tmp_dual = tmp_cell->info().dual;

					//calculate the edge
					edge_tt edge;

					if (act_index < tmp_index)
						edge = edge_tt(edge_t(edge_point(act_index, act_dual), edge_point(tmp_index, tmp_dual)));
					else
						edge = edge_tt(edge_t(edge_point(tmp_index, tmp_dual), edge_point(act_index, act_dual)));

					auto edg_p = edge_cell_ids_map.find(edge);
					if (edg_p == edge_cell_ids_map.end()) continue;

					//create relations between the cells based on the edges
	
					for (auto& related_power_cell : edg_p->second.second)
					{
						auto res = inf_ptrs[related_power_cell]->neighbour_cell_edges.insert(std::pair<unsigned int, std::set<edge_tt*>>(act_pc_index, std::set<edge_tt*>()));
						res.first->second.insert(edg_p->second.first);
					}

				}
			}

		}
	}
	
	//qDebug() << "bef fac: " << boost::chrono::duration_cast<milliseconds>(hrclock::now() - process).count();

	int tt = 0;
	
	for (size_t i = 0; i < inf_ptrs.size(); ++i)
	{
		auto n_c = inf_ptrs[i]->neighbour_cell_edges;

		for (auto& face_candidate : n_c)
		{
			if (face_candidate.second.size() < 3) continue;

			std::vector<unsigned int> related_cells;
			if (i >= 0) related_cells.push_back(i);
			if (face_candidate.first >= 0) related_cells.push_back(face_candidate.first);

			face_tt face;
			power_faces.push_back(std::make_shared<face_tt>(face));
			power_faces.back()->face.add_edges(face_candidate.second, 0);
			power_faces.back()->face.set_related_cell(related_cells);

			//qDebug() << "Neigh test";
			for (auto& ind : related_cells)
			{
				cells[ind]->add_face(const_cast<face_t*>(&(power_faces.back()->face)));
				//	qDebug() << "\t" << ind << " my neigh is : " << power_faces.back()->face.get_neighbour_cell(ind);
			}
		}
	}


	printf("Power Diagram base is created with %u cells, %u poi, ud edge, ud faces\n", (uint32_t)cells.size(), (uint32_t)power_vertices.size(), (uint32_t)power_edges.size(), (uint32_t)power_faces.size());
	
	printf("Power diagram is calculated under ended in %u\n" , (uint32_t)boost::chrono::duration_cast<milliseconds>(hrclock::now() - process).count());

	//checking önmagának szomszédja e

	for (auto& it : cells)
	{
		auto ne = it->get_neighbours();

		std::set<unsigned int> tmp = std::set<unsigned int>(ne.begin(), ne.end());
		if (tmp.find(it->get_my_index()) != tmp.end())
		{
			printf(" Own neighbour\n");
		}

	}

	printf("Power Diagram cells's neighbours are calculated\n");
}

void PowerDiagram::calc_diagram(std::vector<std::pair<weighted_point, point_inf>>& w_p2)
{
	std::vector<std::pair<weighted_point, point_inf2>> w_p;
	for (auto& it : w_p2)
		w_p.push_back(std::pair<weighted_point, point_inf2>(it.first, point_inf2(it.second.surface_points)));

	int ct = 0;
	for (auto& it : w_p)
	{
		ct += it.second.surface_points.size();
	}
	printf("valami %d\n", ct);

	/*************************************
	*	Clear	*
	**************************************/
	for (auto& it : cells)
		delete it;
	cells.clear();
	power_vertices.clear();
	power_edges.clear();
	power_faces.clear();
	pole_map.clear();

	/*************************************
	*	Box update & prepare for Weighted Points	*
	**************************************/

	std::map<Point, std::pair<float, std::vector<Point>>> pole_point_longest_radius_pairs;

	auto start = hrclock::now();
	auto process = hrclock::now();

	std::set<Point> box_points;
	Box pole_bound = update_box(w_p);		//we need to calculate the actual bound as the pole could exceed the original box size

	add_box_points(pole_bound, box_points, w_p);

	process = hrclock::now();

#ifdef TBB_ENABLED
	regular_triangulation::Lock_data_structure locks(CGAL::Bbox_3(pole_bound.get_xmin(), pole_bound.get_ymin(), pole_bound.get_zmin(), pole_bound.get_xmax(), pole_bound.get_ymax(), pole_bound.get_zmax()), 50);
	regular_triangulation R(w_p.begin(), w_p.end(), &locks);
#else
	regular_triangulation2 R(w_p.begin(), w_p.end());
#endif

	w_p.clear();

	assert(R.is_valid());

	/*************************************
	*	Calculate Power Cellas	*
	**************************************/
	process = hrclock::now();

	std::map<edge_tt, edge_identifier> edge_cell_ids_map;
	std::vector<point_inf2*> inf_ptrs;

	std::vector<regular_finite_vertices_iterator2> box_v_its;

	make_power_cells(R, box_points, R.finite_vertices_begin(), R.finite_vertices_end(), &box_v_its, &inf_ptrs, &edge_cell_ids_map);

	//to get those face which belongs to only 1 cell!
	int box_ind = 0;
	for (auto& it : box_v_its)
	{
		box_ind--;
		//get incident cells
		std::vector<regular_cell_handle2> inc_cells;
		R.incident_cells(it, std::back_inserter(inc_cells));

		//-----------------------------------------------
		//Precalculations
		auto inc_c_size = inc_cells.size();

		std::set<regular_cell_handle2> inc_cell_map;	//for finding cell in logM

		for (auto& it : inc_cells)
			inc_cell_map.insert(it);

		unsigned int act_pc_index = box_ind;

		for (size_t cit = 0; cit < inc_c_size; ++cit)
		{
			regular_cell_handle2 act_cell = inc_cells[cit];
			int act_index = act_cell->info().index;

			if (act_index == -1) continue;

			Point* act_dual = act_cell->info().dual;

			//-----------------------------------------------
			//calculate edges
			for (int i = 0; i < 4; ++i)
			{
				regular_cell_handle2 tmp_cell = act_cell->neighbor(i);
				auto other_cell_candidate = inc_cell_map.find(tmp_cell);

				if (other_cell_candidate != inc_cell_map.end())	//both cell_handle are related to the actual point
				{
					int tmp_index = tmp_cell->info().index;

					if (tmp_index == -1) continue;

					Point* tmp_dual = tmp_cell->info().dual;

					//calculate the edge
					edge_tt edge;

					if (act_index < tmp_index)
						edge = edge_tt(edge_t(edge_point(act_index, act_dual), edge_point(tmp_index, tmp_dual)));
					else
						edge = edge_tt(edge_t(edge_point(tmp_index, tmp_dual), edge_point(act_index, act_dual)));

					auto edg_p = edge_cell_ids_map.find(edge);
					if (edg_p == edge_cell_ids_map.end()) continue;

					//create relations between the cells based on the edges

					for (auto& related_power_cell : edg_p->second.second)
					{
						auto res = inf_ptrs[related_power_cell]->neighbour_cell_edges.insert(std::pair<unsigned int, std::set<edge_tt*>>(act_pc_index, std::set<edge_tt*>()));
						res.first->second.insert(edg_p->second.first);
					}

				}
			}

		}
	}

	//qDebug() << "bef fac: " << boost::chrono::duration_cast<milliseconds>(hrclock::now() - process).count();

	int tt = 0;

	for (size_t i = 0; i < inf_ptrs.size(); ++i)
	{
		auto n_c = inf_ptrs[i]->neighbour_cell_edges;

		for (auto& face_candidate : n_c)
		{
			if (face_candidate.second.size() < 3) continue;

			std::vector<unsigned int> related_cells;
			if (i >= 0) related_cells.push_back(i);
			if (face_candidate.first >= 0) related_cells.push_back(face_candidate.first);

			face_tt face;
			power_faces.push_back(std::make_shared<face_tt>(face));
			power_faces.back()->face.add_edges(face_candidate.second, 0);
			power_faces.back()->face.set_related_cell(related_cells);

			//qDebug() << "Neigh test";
			for (auto& ind : related_cells)
			{
				cells[ind]->add_face(const_cast<face_t*>(&(power_faces.back()->face)));
				//	qDebug() << "\t" << ind << " my neigh is : " << power_faces.back()->face.get_neighbour_cell(ind);
			}
		}
	}

	printf("Power Diagram base is created with %u cells, %u poi, %u edge, %u faces\n", (uint32_t)cells.size(), (uint32_t)power_vertices.size(), (uint32_t)power_edges.size(), (uint32_t)power_faces.size());

	printf("Power diagram is calculated under ended in %u\n", (uint32_t)boost::chrono::duration_cast<milliseconds>(hrclock::now() - process).count());


	//checking önmagának szomszédja e

	for (auto& it : cells)
	{
		auto ne = it->get_neighbours();

		std::set<unsigned int> tmp = std::set<unsigned int>(ne.begin(), ne.end());
		if (tmp.find(it->get_my_index()) != tmp.end())
		{
			printf(" Own neighbour\n");
		}

	}

	printf("Power Diagram cells's neighbours are calculated\n");

}
void PowerDiagram::make_power_cells(regular_triangulation2& R, const std::set<Point>& box_points, regular_finite_vertices_iterator2 vit, regular_finite_vertices_iterator2 end, std::vector<regular_finite_vertices_iterator2>* box_v_its, std::vector<point_inf2*>* inf_ptrs, std::map<edge_tt, edge_identifier>* edge_cell_ids_map)
{
	for (; vit != end; vit++)
	{
		//skip box points
		if (box_points.count(vit->point().point()) == 1)
		{
			box_v_its->push_back(vit);
			continue;
		}

		//-----------------------------------------------
		//get incident cells
		std::vector<regular_cell_handle2> inc_cells;
		R.incident_cells(vit, std::back_inserter(inc_cells));

		//check for empty
		if (inc_cells.empty())
			continue;

		//-----------------------------------------------
		//Precalculations
		auto inc_c_size = inc_cells.size();

		std::set<regular_cell_handle2> inc_cell_map;	//for finding cell in logM

		for (auto& it : inc_cells)
			inc_cell_map.insert(it);

		//-----------------------------------------------
		//store pole point for reference

		std::pair<std::map<Point, unsigned int>::iterator, bool> pol_res;
		pol_res = pole_map.insert(std::pair<Point, unsigned int >(vit->point().point(), (unsigned int)pole_map.size()));
		if (pol_res.second == false)
			printf("ERR: BAAAD, double POLE\n");

		//-----------------------------------------------
		//creating powercell								pole ref				get radius			related surf points
		PowerCell* pc = new PowerCell(const_cast<Point*>(&(pol_res.first->first)), sqrt(vit->point().weight()), vit->info().surface_points, cells.size());		//vissza konverzió
		unsigned int act_pc_index = cells.size();

		std::set<edge_tt*> related_edges_of_actual_inc_cells;		//for neighbours calculation

		inf_ptrs->push_back(&vit->info());

		//-----------------------------------------------

		for (size_t cit = 0; cit < inc_c_size; ++cit)
		{
			regular_cell_handle2 act_cell = inc_cells[cit];

			auto act_info = &act_cell->info();
			set_cell_info(R, act_cell, act_info);
			unsigned int act_index = act_info->index;
			Point* act_vert_ptr = act_info->dual;

			//-----------------------------------------------
			//calculate edges

			for (int i = 0; i < 4; ++i)
			{
				regular_cell_handle2 tmp_cell = act_cell->neighbor(i);
				auto other_cell_candidate = inc_cell_map.find(tmp_cell);

				if (other_cell_candidate != inc_cell_map.end())	//both cell_handle are related to the actual point
				{
					//calculate the dual if its not ready yet
					auto tmp_info = &tmp_cell->info();
					set_cell_info(R, tmp_cell, tmp_info);
					unsigned int tmp_index = tmp_info->index;
					Point* tmp_vert_ptr = tmp_info->dual;

					//calculate the edge
					std::pair<std::map<Point*, edge_tt*>::iterator, bool> edge_it_ref;
					if (act_index < tmp_index)
					{
						edge_it_ref = act_info->edge.insert(std::pair<Point*, edge_tt*>(tmp_vert_ptr, nullptr));
						if (edge_it_ref.second == true)
						{
							edge_tt edge = edge_tt(edge_t(edge_point(act_index, act_vert_ptr), edge_point(tmp_index, tmp_vert_ptr)));

							typename edge_pp::ee_ptr node = std::make_shared<edge_pp>(edge);
							power_edges.push_back(node);
							edge_it_ref.first->second = &power_edges.back()->edge;
						}
					}
					else
					{
						edge_it_ref = tmp_info->edge.insert(std::pair<Point*, edge_tt*>(act_vert_ptr, nullptr));
						if (edge_it_ref.second == true)
						{
							edge_tt edge = edge_tt(edge_t(edge_point(tmp_index, tmp_vert_ptr), edge_point(act_index, act_vert_ptr)));

							typename edge_pp::ee_ptr node = std::make_shared<edge_pp>(edge);
							power_edges.push_back(node);
							edge_it_ref.first->second = &power_edges.back()->edge;
						}
					}

					//put the edge into the edge cell id map
					auto ee_res = edge_cell_ids_map->insert(std::pair<edge_tt, edge_identifier>(*edge_it_ref.first->second, edge_identifier()));
					if (ee_res.second == true)
						ee_res.first->second.first = edge_it_ref.first->second;

					auto edge_element_from_map = *ee_res.first;

					//save the edges which belong to the actual cell
					related_edges_of_actual_inc_cells.insert(edge_element_from_map.second.first);

					//create relations between the cells
					for (auto& related_power_cell : edge_element_from_map.second.second)
					{
						if (related_power_cell < act_pc_index)
						{
							auto res = (*inf_ptrs)[related_power_cell]->neighbour_cell_edges.insert(std::pair<unsigned int, std::set<edge_tt*>>(act_pc_index, std::set<edge_tt*>()));
							res.first->second.insert(edge_it_ref.first->second);
						}
						else
						{
							auto res = (*inf_ptrs)[act_pc_index]->neighbour_cell_edges.insert(std::pair<unsigned int, std::set<edge_tt*>>(related_power_cell, std::set<edge_tt*>()));
							res.first->second.insert(edge_it_ref.first->second);
						}
					}
				}
			}

		}

		//add the new cell index to the edges list
		for (auto& it : related_edges_of_actual_inc_cells)
		{
			(*edge_cell_ids_map)[*it].second.insert(act_pc_index);
		}

		//---------------------------

		addCell(pc);
	}
}

void PowerDiagram::make_power_cells_threadsafe(regular_triangulation2& R, const std::set<Point>& box_points, std::vector<regular_finite_vertices_iterator2> fvi_v, std::vector<regular_finite_vertices_iterator2>* box_v_its, std::vector<point_inf2*>* inf_ptrs, std::map<edge_tt, edge_identifier>* edge_cell_ids_map, unsigned int* act_cell_id)
{
	for (auto& vit : fvi_v)
	{
		//skip box points
		if (box_points.count(vit->point().point()) == 1)
		{
			box_v_its->push_back(vit);
			continue;
		}

		//-----------------------------------------------
		//get incident cells
		std::vector<regular_cell_handle2> inc_cells;
		R.incident_cells_threadsafe(vit, std::back_inserter(inc_cells));

		//check for empty
		if (inc_cells.empty())
			continue;

		//-----------------------------------------------
		//Precalculations
		auto inc_c_size = inc_cells.size();

		std::set<regular_cell_handle2> inc_cell_map;	//for finding cell in logM

		for (auto& it : inc_cells)
			inc_cell_map.insert(it);

		//-----------------------------------------------
		//store pole point for reference

		this->mux->lock();
		auto pol_res = &pole_map.insert(std::pair<Point, unsigned int >(vit->point().point(), (unsigned int)pole_map.size()));
		if (pol_res->second == false)
			printf("ERR: BAAAD, double POLE\n");

		unsigned int act_pc_index = *act_cell_id;
		*act_cell_id = *act_cell_id + 1;
		this->mux->unlock();

		//-----------------------------------------------
		//creating powercell								pole ref				get radius			related surf points
		PowerCell* pc = new PowerCell(const_cast<Point*>(&(pol_res->first->first)), sqrt(vit->point().weight()), vit->info().surface_points, act_pc_index);		//vissza konverzió

		std::set<edge_tt*> related_edges_of_actual_inc_cells;		//for neighbours calculation
	
		//-----------------------------------------------

		for (size_t cit = 0; cit < inc_c_size; ++cit)
		{
			regular_cell_handle2 act_cell = inc_cells[cit];

			auto act_info = &act_cell->info();
			set_cell_info_threadsafe(R, act_cell, act_info);

			unsigned int act_index = act_info->index;
			Point* act_vert_ptr = act_info->dual;

			//-----------------------------------------------
			//calculate edges
			for (int i = 0; i < 4; ++i)
			{
				regular_cell_handle2 tmp_cell = act_cell->neighbor(i);
				auto other_cell_candidate = inc_cell_map.find(tmp_cell);

				if (other_cell_candidate != inc_cell_map.end())	//both cell_handle are related to the actual point
				{
					//calculate the dual if its not ready yet
					auto tmp_info = &tmp_cell->info();
					set_cell_info_threadsafe(R, tmp_cell, tmp_info);

					unsigned int tmp_index = tmp_info->index;
					Point* tmp_vert_ptr = tmp_info->dual;

					//calculate the edge
					std::pair<std::map<Point*, edge_tt*>::iterator, bool>* edge_it_ref;
					if (act_index < tmp_index)
					{
						act_info->mux->lock();

						edge_it_ref = &act_info->edge.insert(std::pair<Point*, edge_tt*>(tmp_vert_ptr, nullptr));
						if (edge_it_ref->second == true)
						{
							edge_tt edge = edge_tt(edge_t(edge_point(act_index, act_vert_ptr), edge_point(tmp_index, tmp_vert_ptr)));

							typename edge_pp::ee_ptr node = std::make_shared<edge_pp>(edge);

							this->mux->lock();
							power_edges.push_back(node);
							edge_it_ref->first->second = &power_edges.back()->edge;
							this->mux->unlock();
						}

						act_info->mux->unlock();
					}
					else
					{
						tmp_info->mux->lock();

						edge_it_ref = &tmp_info->edge.insert(std::pair<Point*, edge_tt*>(act_vert_ptr, nullptr));
						if (edge_it_ref->second == true)
						{
							edge_tt edge = edge_tt(edge_t(edge_point(tmp_index, tmp_vert_ptr), edge_point(act_index, act_vert_ptr)));

							typename edge_pp::ee_ptr node = std::make_shared<edge_pp>(edge);

							this->mux->lock();
							power_edges.push_back(node);
							edge_it_ref->first->second = &power_edges.back()->edge;
							this->mux->unlock();
						}

						tmp_info->mux->unlock();
					}

					//put the edge into the edge cell id map
					this->mux->lock();

					auto ee_res = &edge_cell_ids_map->insert(std::pair<edge_tt, edge_identifier>(*edge_it_ref->first->second, edge_identifier()));
					if (ee_res->second == true)
						ee_res->first->second.first = edge_it_ref->first->second;

					auto edge_element_from_map = *ee_res->first;
					this->mux->unlock();

					//save the edges which belong to the actual cell
					related_edges_of_actual_inc_cells.insert(edge_element_from_map.second.first);

					//create relations between the cells
					for (auto& related_power_cell : edge_element_from_map.second.second)
					{
						if (related_power_cell == act_pc_index) continue;

						if (related_power_cell < act_pc_index)
						{
							(*inf_ptrs)[related_power_cell]->mux->lock();

							auto res = (*inf_ptrs)[related_power_cell]->neighbour_cell_edges.insert(std::pair<unsigned int, std::set<edge_tt*>>(act_pc_index, std::set<edge_tt*>()));
							res.first->second.insert(edge_it_ref->first->second);

							(*inf_ptrs)[related_power_cell]->mux->unlock();
						}
						else
						{
							(*inf_ptrs)[act_pc_index]->mux->lock();

							auto res = (*inf_ptrs)[act_pc_index]->neighbour_cell_edges.insert(std::pair<unsigned int, std::set<edge_tt*>>(related_power_cell, std::set<edge_tt*>()));
							res.first->second.insert(edge_it_ref->first->second);

							(*inf_ptrs)[act_pc_index]->mux->unlock();
						}
					}

				}
			}

		}
		
		this->mux->lock();

		//add the new cell index to the edges list
		for (auto& it : related_edges_of_actual_inc_cells)
			(*edge_cell_ids_map)[*it].second.insert(act_pc_index);
		this->mux->unlock();

		this->mux->lock();

		//---------------------------
		//addCell(pc);
		cells[act_pc_index] = pc;
		this->mux->unlock();
	}
}

void PowerDiagram::set_cell_info(regular_triangulation2& R, regular_cell_handle2 cell, cell_inf_power2* info)
{
	if (info->index == -1)
	{
		info->index = power_vertices.size();
		Point dual = R.dual(cell);
		typename vertex_t::v_ptr node = std::make_shared<vertex_t>(dual);
		power_vertices.push_back(node);
		info->dual = &power_vertices.back()->point;
	}
}

void PowerDiagram::set_cell_info_threadsafe(regular_triangulation2& R, regular_cell_handle2 cell, cell_inf_power2* info)
{
	info->mux->lock();
	if (info->index == -1)
	{
		Point dual = R.dual(cell);
		typename vertex_t::v_ptr node = std::make_shared<vertex_t>(dual);
		this->mux->lock();
		info->index = power_vertices.size();
		power_vertices.push_back(node);
		info->dual = &power_vertices.back()->point;
		this->mux->unlock();
	}
	info->mux->unlock();
}

/* ****************************
Labeling functions
**************************** */

float PolarBall2::get_alpha_weight(PolarBall2* rhs)
{
	float R = this->pole.radius;
	float r = rhs->get_radius();
	float d = (this->get_point() - rhs->get_point()).squared_length();

	/**************************
	Law of cosinus for calculate intersection deepness
	*************************************************/

	float lhs_cosB = (r * r + R * R - d) / (2 * r*R);
	float B_angle = acosf(lhs_cosB);

	float A_angle = 3.14159265359f - B_angle;
	float result_attempt = -1 * cosf(A_angle);

	if (result_attempt < 0)		//only 0.0f - 1.0f range is valid, here the intersection was more deeply -> they have more the same label
	{
		//		qDebug() << "\t\tERR: ALPHA val:";
		/*		qDebug() << "\t\t\tR: " << R << ", r: " <<r <<", d: "<< d;
		qDebug() << "\t\t\tB_angle: " << B_angle;
		qDebug() << "\t\t\tA_angle: " << A_angle;
		qDebug() << "\t\t\tlhs_cosB " << lhs_cosB;
		qDebug() << "\t\t\tgetalphaWeight return: " << -1 * cosf(A_angle);*/
		return 0.0f;
	}

	return result_attempt;
}

float PolarBall2::get_beta_weight(PolarBall2* rhs, Point& surfpoint)
{
	/**************************
	Check for common surf point
	**********************/

	std::vector<Point> lhs_surface_points = get_surf_points();
	std::vector<Point> rhs_surface_points = rhs->get_surf_points();

	std::set<Point> lhs_surface_points_s(lhs_surface_points.begin(), lhs_surface_points.end());
	std::set<Point> rhs_surface_points_s(rhs_surface_points.begin(), rhs_surface_points.end());

	if (lhs_surface_points_s.find(surfpoint) == lhs_surface_points_s.end() ||
		rhs_surface_points_s.find(surfpoint) == rhs_surface_points_s.end())
	{
		printf("\t\tERR: There is no surf point for beta calculation!\n");
		return 0.0f;
	}

	/**************************
	Cos of Angle between the 2 vector : from common points towards the center of pole
	*************************/
	Vector vec_one = this->get_point() - surfpoint;
	Vector vec_two = rhs->get_point() - surfpoint;

	float lhs = CGAL::scalar_product(vec_one, vec_two);
	float cos_val = lhs / sqrt(vec_one.squared_length() * vec_two.squared_length());

	float result_attempt = -1 * cos_val;

	if (result_attempt < 0)		// this should never happen
	{
		//printf("\t\tERR: BETA val:\n";
		/*	qDebug() << "\t\t\tvec1 " << vec_one.x() << ", " << vec_one.y() << ", " << vec_one.z();
		qDebug() << "\t\t\tvec2 " << vec_two.x() << ", " << vec_two.y() << ", " << vec_two.z();
		qDebug() << "\t\t\tscalar dot " << lhs;
		qDebug() << "\t\t\tBetaWeight return: " << -1 * cos_val;*/
		return 0.0f;
	}

	return result_attempt;
}

void PowerDiagram::label_poles()
{
	printf("\nLabeling method\n");

	auto start = hrclock::now();

	/*************************************
	*	Clear	*
	**************************************/
	priority_ball_queue pri_ball_q;
	std::map<PolarBall2*, std::set<PolarBall2*> > neigh_graph;
	neigh_graph.clear();

	bool error_during_init = false;
	/*************************************
	*	0. step:
	get Pole pair by surface point	*
	**************************************/
	std::map<Point, std::vector<PolarBall2*>> neigh_by_surf;
	neigh_by_surf.clear();

	int ct = 0;
	for (auto& it : cells)
	{
		PolarBall2* actual_polarball = it->get_polarball_ptr();
		std::vector<Point> related_surf_points = actual_polarball->get_surf_points();
		for (auto surf_point : related_surf_points)
		{
			std::pair<std::map<Point, std::vector<PolarBall2*>>::iterator, bool> res;
			res = neigh_by_surf.insert(std::pair<Point, std::vector<PolarBall2*>>(surf_point, std::vector<PolarBall2*>()));
			res.first->second.push_back(actual_polarball);
		}
		ct += related_surf_points.size();
	}
	printf("\t\tAll surf_points: %d \n", ct);
	/*************************************
	*	Checks:
	kül. polarball oknak a surf pointjai különbözõek!!!
	Nem 2 polarball tartozik 1 surf pointhoz *
	**************************************/

	for (auto& it : cells)
	{
		PolarBall2* actual_polarball = it->get_polarball_ptr();
		std::vector<Point> related_surf_points = actual_polarball->get_surf_points();
		std::set<Point> tmp_rel_surf(related_surf_points.begin(), related_surf_points.end());

		if (related_surf_points.size() != tmp_rel_surf.size())
		{
			error_during_init = true;
			printf("\t\tERR: One/More surf point added more than once to pole\n");
		}
	}

	for (auto it : neigh_by_surf)
	{
		//qDebug() << it.first.x() << ", " << it.first.y() << ", " << it.first.z() << " - " << it.second.size() << " " << it.second;
		if (it.second.size() != 2)
		{
			//	error_during_init = true;
			printf("\t\tERR: We lost a pole! - there is no pair %u \n", (uint32_t)it.second.size());
		}
	}

	/*************************************
	*	First step:
	Creathe the neighbour graph	*
	**************************************/

	for (auto& cell : cells)
	{
		PolarBall2* cell_ptr = cell->get_polarball_ptr();
		std::set<PolarBall2*> actual_neighs_ptr;

		//adding neighbours by common face
		auto actual_neighs = cell->get_neighbours();
		for (auto& neigh : actual_neighs)
			actual_neighs_ptr.insert(cells[neigh]->get_polarball_ptr());

		//adding neighbours by common surf point
		std::vector<Point> related_surf_points = cell->get_related_surf_points();
		for (auto surf_point : related_surf_points)
		{
			auto pole_pairs = neigh_by_surf[surf_point];	//fix size ( 2 )

			if (pole_pairs[0] != cell_ptr && pole_pairs.size() == 2 && pole_pairs[1] != cell_ptr)
			{
				error_during_init = true;
				printf("\t\tERR: BAD NEIGHBOUR CALCULATION\n");
			}

			//surf point base edge
			if (pole_pairs[0] != cell_ptr)
				actual_neighs_ptr.insert(pole_pairs[0]);
			else if (pole_pairs.size() == 2)
				actual_neighs_ptr.insert(pole_pairs[1]);
		}

		neigh_graph.insert(std::pair<PolarBall2*, std::set<PolarBall2*>>(cell_ptr, actual_neighs_ptr));
	}

	printf("\t\tNeighbour graph created\n");
	/*************************************
	*	Second step:
	init the priority queue	*
	**************************************/

	for (auto &it : neigh_graph)
	{
		it.first->set_in_value(0);
		it.first->set_out_value(0);
		pri_ball_q.push(0.0f, it.first);
	}
	//qDebug() << "\t\tPriority queue initalized with " << pri_ball_q.size() << " element";

	/*************************************
	*	Third step:
	Setting outer poles	based on the bound box*
	**************************************/
	printf("\tSetting outlier poles\n");

	priority_ball_queue tmp_q = pri_ball_q;

	int out_p = 0;
	while (!tmp_q.empty())
	{
		PolarBall2* tmp = tmp_q.pop();

		Point pole_point = tmp->get_point();
		if (!bounded.is_inside(pole_point.x(), pole_point.y(), pole_point.z()))
		{
			//auto element_in_priq = pri_ball_q.find(tmp);
			if (pri_ball_q.count(tmp) > 1)
			{
				error_during_init = true;
				printf("\t\tERR: Invalid Priority queue\n");
			}
			//	qDebug() << tmp->get_heap_index();

			tmp->set_out_value(1.0f);

			pri_ball_q.update(tmp->get_heap_index());
			out_p++;
		}

	}

	if (error_during_init)
	{
		printf("\tERROR: Resolve the errors before go for labeling\n");
		return;
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

		PolarBall2* p = pri_ball_q.pop();
		p->set_heap_index(-1);

		if (prev_p < pri_ball_q.highest_prior())
		{
			printf("\tPriority queue FAIL %.4f - %.4f \n", prev_p , pri_ball_q.highest_prior());
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
			//	qDebug() << "\t\t\tIts an Inner Pole";
		}
		else
		{
			p->set_flag(1);
			tmp_p = out_val;
			//	qDebug() << "\t\t\tIts an Outer Pole";
		}

		/*************************************
		*	Calculate new weights with Beta weight	: */

		std::vector<Point> tmp_surf_points = p->get_surf_points();
		//qDebug() << "\t\tNumber of related Surf p: " << tmp_surf_points.size();

		for (std::vector<Point>::iterator related_surf_p = tmp_surf_points.begin();
			related_surf_p != tmp_surf_points.end();		++related_surf_p)
		{
			std::vector<PolarBall2*> tmp_related_polarballs = neigh_by_surf[*related_surf_p];

			if (tmp_related_polarballs.size() != 2)
			{
				printf("\tERR: Wrong related polarball ptrs\n");
				if (tmp_related_polarballs[0] != p) printf("\tERR: STRANGE - pole count thing to surface point\n");
				continue;
			}
			PolarBall2* opposite_pole = tmp_related_polarballs[0] != p ? tmp_related_polarballs[0] : tmp_related_polarballs[1];
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

		std::set<PolarBall2*> related_neighbours = neigh_graph[p];
		//qDebug() << "Neighbours ";
		for (std::set<PolarBall2*>::iterator it = related_neighbours.begin(); it != related_neighbours.end(); ++it)
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
	neigh_by_surf.clear();
	neigh_graph.clear();

	//qDebug() << "Labeling finished";
	printf("Labeling ended in: %zu\n" , boost::chrono::duration_cast<milliseconds>(hrclock::now() - start).count());
}

/* ****************************
Power Crust calculation functions
**************************** */

void PowerDiagram::calc_power_crust()
{
	printf("Power Crust calculation\n");

	std::vector<face_t*> unoriented_power_crust_faces;

	for (auto& face : power_faces)
	{
		auto related_cells = face->face.related_cells;
		if (related_cells.size() > 2) printf("ERR: BAD FACE\n");
		if (related_cells.size() != 2)	continue;

		unsigned int cell_one = related_cells[0];
		unsigned int cell_two = related_cells[1];

		int border_face_check = (cells[cell_one]->has_inner_pole() + cells[cell_two]->has_inner_pole());

		if (border_face_check == 1)		//this will be a boundary face, correct its orientation
		{
			face->face.set_power_crust_flag(true);
			face->face.correct_edge_orientations();
			unoriented_power_crust_faces.push_back(&face->face);
		}
	}

	printf("We have %u power crust face!\n", (uint32_t)unoriented_power_crust_faces.size());

	correct_unoriented_power_crust_faces(unoriented_power_crust_faces);
}

void PowerDiagram::correct_unoriented_power_crust_faces(std::vector<face_t*>& unoriented_face_ptrs)
{
	printf("\tCorrect face orientations\n");

	face_t* starter_correct_face = nullptr;
	float distance_from_correct_face = FLT_MAX;
	Point outer_point = Point(3.0f*bounded.get_xmax(), 3.0f*bounded.get_ymax(), 3.0f*bounded.get_zmax());
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
			face_first_point = edge1->edge.first.second;
			face_second_point = edge1->edge.second.second;
		}
		else
		{
			face_first_point = edge1->edge.second.second;
			face_second_point = edge1->edge.first.second;
		}

		if (edge2->related_faces[power_face] == 0)
			face_third_point = edge2->edge.second.second;
		else
			face_third_point = edge2->edge.first.second;

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

		starter_correct_face->correct_boundary_orientation();			// we can go around the whole boundary because of the power_crust_flag
	}
	else
		printf("ERR: during correct unoriented normals\n");
}

/* ****************************
Others
**************************** */

void PowerDiagram::add_box_points(const Box& box, std::set<Point>& box_points, std::vector<std::pair<weighted_point, point_inf2>>& points)
{
	Point p1 = Point(box.get_xmin(), box.get_ymin(), box.get_zmin());
	Point p2 = Point(box.get_xmin(), box.get_ymin(), box.get_zmax());
	Point p3 = Point(box.get_xmin(), box.get_ymax(), box.get_zmin());
	Point p4 = Point(box.get_xmin(), box.get_ymax(), box.get_zmax());
	Point p5 = Point(box.get_xmax(), box.get_ymin(), box.get_zmin());
	Point p6 = Point(box.get_xmax(), box.get_ymin(), box.get_zmax());
	Point p7 = Point(box.get_xmax(), box.get_ymax(), box.get_zmin());
	Point p8 = Point(box.get_xmax(), box.get_ymax(), box.get_zmax());

	//TODO: 6 és 8 miatt ír warningot!

	box_points.insert(p1); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p1, 0.000001f), std::vector<Point>()));
	box_points.insert(p2); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p2, 0.000001f), std::vector<Point>()));
	box_points.insert(p3); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p3, 0.000001f), std::vector<Point>()));
	box_points.insert(p4); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p4, 0.000001f), std::vector<Point>()));
	box_points.insert(p5); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p5, 0.000001f), std::vector<Point>()));
	box_points.insert(p6); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p6, 0.000001f), std::vector<Point>()));
	box_points.insert(p7); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p7, 0.000001f), std::vector<Point>()));
	box_points.insert(p8); points.push_back(std::pair<weighted_point, std::vector<Point>>(weighted_point(p8, 0.000001f), std::vector<Point>()));
}

Box PowerDiagram::update_box(const std::vector<std::pair<Pole2, Point> >& weighted_points)
{
	Box res = Box(-1.0f, 1.0f);

	float minX = FLT_MAX, maxX = FLT_MIN;
	float minY = FLT_MAX, maxY = FLT_MIN;
	float minZ = FLT_MAX, maxZ = FLT_MIN;
	float x, y, z;

	for (auto it : weighted_points)
	{
		x = it.first.center->x();
		y = it.first.center->y();
		z = it.first.center->z();

		minX = minX > x ? x : minX;
		maxX = maxX < x ? x : maxX;
		minY = minY > -y ? -y : minY;
		maxY = maxY < -y ? -y : maxY;
		minZ = minZ > z ? z : minZ;
		maxZ = maxZ < z ? z : maxZ;
	}

	if (!res.is_inside(minX, minY, minZ) || !res.is_inside(maxX, maxY, maxZ))
	{
		res = Box(minX < res.get_xmin() ? minX : res.get_xmin()
			, maxX > res.get_xmax() ? maxX : res.get_xmax()
			, minY < res.get_ymin() ? minY : res.get_ymin()
			, maxY > res.get_ymax() ? maxY : res.get_ymax()
			, minZ < res.get_zmin() ? minZ : res.get_zmin()
			, maxZ > res.get_zmax() ? maxZ : res.get_zmax());

		res = Box(res.get_xmin() * 1.2f,
			res.get_xmax() * 1.2f,
			res.get_ymin() * 1.2f,
			res.get_ymax() * 1.2f,
			res.get_zmin() * 1.2f,
			res.get_zmax() * 1.2f);

	}

	return res;
}

Box PowerDiagram::update_box(const std::vector<std::pair<weighted_point, point_inf2>>& weighted_points)
{
	Box res = Box(-1.0f, 1.0f);

	float minX = FLT_MAX, maxX = FLT_MIN;
	float minY = FLT_MAX, maxY = FLT_MIN;
	float minZ = FLT_MAX, maxZ = FLT_MIN;
	float x, y, z;

	for (auto it : weighted_points)
	{
		x = it.first.point().x();
		y = it.first.point().y();
		z = it.first.point().z();

		minX = minX > x ? x : minX;
		maxX = maxX < x ? x : maxX;
		minY = minY > -y ? -y : minY;
		maxY = maxY < -y ? -y : maxY;
		minZ = minZ > z ? z : minZ;
		maxZ = maxZ < z ? z : maxZ;
	}

	if (!res.is_inside(minX, minY, minZ) || !res.is_inside(maxX, maxY, maxZ))
	{
		res = Box(minX < res.get_xmin() ? minX : res.get_xmin()
			, maxX > res.get_xmax() ? maxX : res.get_xmax()
			, minY < res.get_ymin() ? minY : res.get_ymin()
			, maxY > res.get_ymax() ? maxY : res.get_ymax()
			, minZ < res.get_zmin() ? minZ : res.get_zmin()
			, maxZ > res.get_zmax() ? maxZ : res.get_zmax());

		res = Box(res.get_xmin() * 1.2f,
			res.get_xmax() * 1.2f,
			res.get_ymin() * 1.2f,
			res.get_ymax() * 1.2f,
			res.get_zmin() * 1.2f,
			res.get_zmax() * 1.2f);

	}

	return res;
}