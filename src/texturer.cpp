#include "texturer.h"
#include "texture_utils.h"
#include "utils.h"
#include "base_utils.h"

#define UV_CALC_EPSILON 0.1f
#define TEX_CALC_EPSILON2 0.4f
#define N 1
#define NEAREST_RANGE 0.08
#define REAL_TRIANG_MAX_DIST 0.5

void Texturer::add_tex_or_color_info(export_data& untex_paint, pointcloud_data& pd)
{
	Box box(-1.0f, 1.0f);

	point_uv_map poi_all;
	for (auto& it : pd)
		poi_all.push_back(tuple(it.first,it.second));

	printf("\nAdding Texture data to our result %zu all point to %zu new mesh point\n", poi_all.size(), untex_paint.vertex_v.size());

	if (number_of_camera == 0)
	{
		printf("There is 0 camera - cannot do texturing\n");
		adding_empty_uv_data(untex_paint);
		return;
	}

	if (!poi.empty())										//prevent calculation if we already did it
	{
		printf("There is %zu point - cannot do texturing\n", poi.size());
		add_tex_to_untex_paint(untex_paint);
		return;
	}

	uv_trees.clear();
	for (int i = 0; i < number_of_camera; ++i)
		uv_trees.emplace_back(new tree_2d());

	std::cout << "number of cam: " << number_of_camera << "\n";

	std::vector<std::vector<tree_2d_element>> elements_for_2d;		//we need these data on stack because of kdtree pointers!!!
	elements_for_3d_tree.clear();
	init_uv_tree(poi_all, elements_for_2d, elements_for_3d_tree);

	if (!init_tree(elements_for_3d_tree, box))
	{
		printf("Initialization of Kd-tree failed\n");
		return;
	}
	printf("\tSuccess kdtree\n");
	
	//Point t_point(0.5f, 0.2f, 0.5f);
	//Point t_point(-0.5f, 0.2f, 0.1f);
	//Point t_point1(0.4f, 0.5f, -0.2f);
	//Point t_point3(0.4f, 0.4f, -0.45f);
	//Point t_point(0.4f, 0.5f, -0.45f);
	//Point t_point1(0.4f, 0.3f, -0.2f);
	//Point t_point3(0.4f, 0.5f, -0.5f);

	//Point t_point1(0.52f, 0.41f, -0.31f);
	Point t_point(0.6f, 0.5f, -0.3f);
	//Point t_point2(0.3f, 0.2f, -0.2f);

	//auto a_r = calc_good_uv(t_point, elements_for_3d_tree);
	//qDebug() << "uvs for " << t_point << " is " << a_r;

	//for (auto& it : untex_paint.points)
	//	auto a_r = calc_good_uv(it, elements_for_3d_tree);

	//----------------------------------
	//		Calculate uv s for every triangle point
	//---------------------------------

	pi_camuv_pair_ss.clear();
	uv_maps.clear();
	poi.clear();
	
	poi = untex_paint.vertex_v;

	uv_maps.resize(poi.size());
	for (auto& it : uv_maps)
		it.resize(number_of_camera);		

	pi_camuv_pair_ss.resize(number_of_camera);

	std::vector<uv_map> uv_cache;							//prevent multiple calculation
	uv_cache.resize(poi.size());

	std::vector<index_point> one_triangle_points;			//the actual triangle points, we only do sthng when we have 3 point
	one_triangle_points.clear();

	auto start = hrclock::now();


	for (auto& index : untex_paint.face_v)
	{
		one_triangle_points.push_back(std::pair<unsigned int, Point>(index.first, poi[index.first]));

		if (one_triangle_points.size() == 3)
		{
			calc_triangle_details(one_triangle_points, uv_cache);
			while (one_triangle_points.size() != 0)
			{
			//	for (auto& it : one_triangle_points) qDebug() << it.first;

				std::vector<index_point> tmp(one_triangle_points.begin(), one_triangle_points.begin() + 3);
				calc_triangle_details(tmp, uv_cache);
				one_triangle_points = std::vector<index_point>(one_triangle_points.begin() + 3, one_triangle_points.end());
			
			//	for (auto& it : one_triangle_points) qDebug() << it.first;

				for (auto& it : tmp)
					one_triangle_points.push_back(it);

			//	qDebug() << "Remaining: " << one_triangle_points.size();
			//	one_triangle_points.clear();
			}
		}
	//	qDebug() << "ok";
	}

	printf("texturing ended in : %zu\n", boost::chrono::duration_cast<milliseconds>((hrclock::now() - start  )).count());
	//----------------------------------
	//		Fill up our result
	//---------------------------------
	printf("\tFill up the result\n");

	add_tex_to_untex_paint(untex_paint);

	//----------------------------------
	//		Release our trees
	//---------------------------------
	for (auto& it : uv_trees)
	{
		it->release();
		delete it;
	}

	uv_trees.clear();

	tree.release();
	tmp_tree.release();
}

void Texturer::init_uv_tree(const point_uv_map& points, std::vector<std::vector<tree_2d_element>>& elements_for_2d, std::vector<tree_3d_element>& elements_for_3d)
{
	elements_for_2d.clear();
	elements_for_2d.resize(number_of_camera);

	int ind = 0;
	for (auto& poi_uvs = points.begin(); poi_uvs != points.end(); ++poi_uvs)
	{
		for (auto& uv_t : poi_uvs->second)
		{
			elements_for_2d[uv_t.device_id].push_back(tree_2d_element(uv_t.texture_coordinate, info_2d(ind, uv_t.object_id)));
		}

		elements_for_3d.push_back(tree_3d_element(poi_uvs->first, info_3d(poi_uvs->second, ind)));
		ind++;
	}

	std::cout << elements_for_3d.size() << " points in tree\n";
 	for(auto& it : uv_trees)
		it->release();

	//qDebug() << "Uv count for each camera:";
	for (int i = 0; i < number_of_camera; ++i)
	{
		//qDebug() << "\t" << elements_for_2d[i].size();
		bool ok = uv_trees[i]->build_process(elements_for_2d[i], Box(0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f));
		if (ok != true)  printf("ERR: Build Tree\n");
		uv_trees[i]->set_kn(3);
	}
	
}

bool Texturer::init_tree(std::vector<tree_3d_element>& elements_for_3d, const Box& box)
{
	tree.release();
	bool build_p = tree.build_process(elements_for_3d, box);
	tree.set_kn(3 * number_of_camera);		//we need at least 3 point from 1 camera - Pigeonhole principle
	return build_p;
}

void Texturer::adding_empty_uv_data(export_data& untex_paint)
{
	for (auto& p : untex_paint.vertex_v)
		untex_paint.uv_v.push_back(uv(0.0f, 0.0f));

	untex_paint.device_related_size.clear();
	untex_paint.device_related_size.push_back(untex_paint.face_v.size());
}

void Texturer::add_tex_to_untex_paint(export_data& untex_paint)
{
	untex_paint.device_related_size.clear();

	untex_paint.vertex_v.clear();
	untex_paint.face_v.clear();
	untex_paint.uv_v.clear();

//	qDebug() << "result:";
	/*for (auto& it : pi_camuv_pair_ss)
		qDebug() << it;
*/

	std::vector<std::map<std::pair<index, uv>, index>> new_indicies_map;
	std::map<uv, index> new_uv_m;
	new_indicies_map.resize(poi.size());

	for (size_t i = 0; i < pi_camuv_pair_ss.size(); ++i)
	{
		unsigned int camera = i;
		auto& cpu_v = pi_camuv_pair_ss[i];

		for (auto& pu_t : cpu_v)
		{
			auto& act_uv_map = uv_maps[pu_t.first][camera];

			uv act = act_uv_map.begin()->first;

			auto it = act_uv_map.begin();
			for (; it->second != pu_t.second; ++it) {}
			act = it->first;

			auto& act_m = new_indicies_map[pu_t.first];
			auto& in = act_m.insert(std::pair<std::pair<index, uv>, index>(std::pair<index, uv>(camera, act), (index)untex_paint.vertex_v.size()));
			if (in.second == true)
			{
				untex_paint.vertex_v.push_back(poi[pu_t.first]);
			}
			auto& in_uv = new_uv_m.insert(std::pair<uv, index>(act, (uint32_t)new_uv_m.size()));
			if (in_uv.second == true)
				untex_paint.uv_v.push_back(act);
			
			untex_paint.face_v.push_back(std::pair<uint32_t, uint32_t>(in.first->second, in_uv.first->second));
		}

		untex_paint.device_related_size.push_back(cpu_v.size());
	}
	
	for (size_t i = 0; i < untex_paint.device_related_size.size(); ++i)
		printf("\tTexture from Camera %zu - index count: %u\n" , i, untex_paint.device_related_size[i]);
}

void Texturer::update_var_sizes(std::vector< p_texture_t >& uvs, std::vector<uv_map>& uv_cache)
{
	std::map<index, Point> new_indps;
	for (auto& it : uvs)
		if (it.first.first >= poi.size()) new_indps.insert(std::pair<index, Point>(it.first.first, it.first.second));

	if (new_indps.size() == 0) return;

	poi.resize(poi.size() + new_indps.size());

	for (auto& it : new_indps)
	{
		poi[it.first] = it.second;
		uv_cache.push_back(uv_map());
		uv_maps.push_back(std::vector< std::map<uv, index> >());
		uv_maps.back().resize(number_of_camera);
	}
}

void Texturer::fill_uv_and_index_list(const texture_tuple& texture_uv_result)
{
	index cam_index = texture_uv_result.first;
	//qDebug() << cam_index << " " << poi.size();

	for (auto& uv_pair : texture_uv_result.second)
	{
		unsigned int poi_index = uv_pair.first.first;
		
		if (poi[poi_index] != uv_pair.first.second) printf("Baaaad\n");

		auto& act_map = uv_maps[poi_index][cam_index];
	//	qDebug() << "ind " << poi_index << " s " << act_map.size() << " " << uv_pair.second;
		auto in = act_map.insert(std::pair<uv, index>(uv_pair.second, (index) act_map.size()));
		pi_camuv_pair_ss[cam_index].push_back(pi_uv_p(poi_index, in.first->second));
	}
}

/*****************
	calculations
*****************/

void Texturer::calc_triangle_details(std::vector<index_point>& one_triangle_points, std::vector<uv_map>& uv_cache)
{
	//calculate the possible uvs for each triangle point
	std::vector<uv_map> triangle_uvs = calculate_uv_coords_for_triangle(one_triangle_points, uv_cache);
	//print_possible_uv(triangle_uvs, one_triangle_points);

	//calculate the most proper uv from the list
	std::pair<texture_tuple, process_state> res_uvs = calc_camera_based_uv(triangle_uvs, one_triangle_points);

	switch (res_uvs.second)
	{
		case OK:
		{
			fill_uv_and_index_list(res_uvs.first);
			one_triangle_points.clear();
			break;
		}
		case PARTITION:
		{
			partition_by_side(one_triangle_points, uv_cache);
			break;
		}
		case CLIPPED:
		{
		//	qDebug() << "CLIPPED";
			update_var_sizes(res_uvs.first.second, uv_cache);

			auto texture_uv_result = res_uvs.first;

			index cam_index = texture_uv_result.first;
	
			std::vector<std::pair<index, pi_uv_p>> tmp_s;
			int ind = 0;
			for (auto& uv_pair : texture_uv_result.second)
			{
				if (ind == 9) break;
				ind++;

				unsigned int poi_index = uv_pair.first.first;

				if (poi[poi_index] != uv_pair.first.second)  printf("Baaaad\n");

				auto& act_map = uv_maps[poi_index][cam_index];
				//	qDebug() << "ind " << poi_index << " s " << act_map.size() << " " << uv_pair.second;
				auto& in = act_map.insert(std::pair<uv, index>(uv_pair.second, (index)act_map.size()));
	
				pi_camuv_pair_ss[cam_index].push_back(pi_uv_p(poi_index, in.first->second));
				tmp_s.push_back(std::pair<index, pi_uv_p>(cam_index, pi_uv_p(poi_index, in.first->second)));
			}

			one_triangle_points.clear();

			for (; ind < texture_uv_result.second.size(); ++ind)
			{
				auto uv_pair = texture_uv_result.second[ind];
			
				one_triangle_points.push_back(uv_pair.first);
			}
		
			break;
		}
		default:
			one_triangle_points.clear();
	}
}

/*****************
calculation new uv coordinates
*****************/

//gives back the best uv coordinates for each point of triangle and its correctness flag
std::vector<uv_map> Texturer::calculate_uv_coords_for_triangle(std::vector<index_point>& triangle, std::vector<uv_map>& uv_cache, bool use_cache)
{
	std::vector<uv_map> point_uvs;

	//get the possible uv coordinates from each camera - use cache if its available
	for (auto& poi : triangle)
	{
		uv_map* cache;
		if (use_cache)
		{
			cache = &uv_cache[poi.first];

			if (cache->size() != 0)
			{
				point_uvs.push_back(*cache);
				continue;
			}
		}
		//qDebug() << poi.first << " poi i " << poi.second;
		kNearest_queue_3d closest_points;
		tree.knearest_p(&poi.second, tree.get_root_ptr(), closest_points);
		uv_map modified_uvs = calc_new_uv_coords_from_knearest(closest_points, poi.second);
		//uv_map modified_uvs = calc_good_uv(poi.second, elements_for_3d_tree);
		
		point_uvs.push_back(modified_uvs);
		if (use_cache) *cache = modified_uvs;
	}

	return point_uvs;
}

uv_map Texturer::calc_new_uv_coords_from_knearest(kNearest_queue_3d& closest_points, Point& triangle_point)		//seems OK
{
	std::vector< std::set<std::pair<float, p_texture_t>> > separated_uvs;
	separated_uvs.resize(number_of_camera);

	/*-----------------------------------
		Getting all the closest points and load their uvs into sep. uvs
	---------------------- */

	while (!closest_points.empty())
	{
		auto pri_element = closest_points.top();
		closest_points.pop();

		float distance = pri_element.distance;
		Point related_surf_point = pri_element.neighb->first;
		auto related_uvs_from_cameras = pri_element.neighb->second.uvs;
		auto related_ind_from_cameras = pri_element.neighb->second.index;		//this index from all surf point

		//getting all the related uvs
		for (auto& _uv : related_uvs_from_cameras)
		{
			unsigned int camera = _uv.device_id;
			auto tmp_ip = index_point(related_ind_from_cameras, related_surf_point);
			separated_uvs[camera].insert(std::pair<float, p_texture_t>(distance, p_texture_t(tmp_ip, _uv.texture_coordinate)));
		}
	}

	//print_incoming_uv_for_p(separated_uvs, triangle_point);
	
	/*-----------------------------------
			Calculate possible uvs
	---------------------- */

	uv_map uvs_for_point;

	
	for (int i = 0; i < number_of_camera; ++i)
	{
		auto uv_set = separated_uvs[i];		//ordered, smallest at begin

		if (uv_set.size() < 3) continue;	//we dont have enough points from i camera

		std::set<unsigned int> related_indicies;
		std::map<unsigned int, Point> related_points;
		index closest_i = uv_set.begin()->second.first.first;

		for (auto& it : uv_set)		//getting the 3 closest point to the act uv
		{
			related_points.insert(std::pair<unsigned int, Point>(it.second.first.first, it.second.first.second));
			related_indicies.insert(it.second.first.first);
			if (related_indicies.size() == 3) break;
		}
		
		// the uv will most resemble to the closest point's uvs for every camera , 
		//if can belong more than 1 uv for a point on 1 camera image then this will be a list of uvs
		//							otherwise this will contain only 1 possibility by cameras	

		for (auto& _uv : uv_set)
		{
			if (_uv.second.first.first != closest_i) break;

			uv act_uv = _uv.second.second;

			kNearest_queue_2d kq;
			uv_trees[i]->knearest_p_conditional(&act_uv, uv_trees[i]->get_root_ptr(), kq, related_indicies);

			std::vector<p_texture_t> found_uvs;

			while (!kq.empty())
			{
				auto element = kq.top();
				kq.pop();
			//	qDebug() << "\t" << element.distance << " - " << element.neighb->second.index << " " << element.neighb->first;
				if (element.distance <= UV_CALC_EPSILON) found_uvs.push_back(p_texture_t(index_point(element.neighb->second.index, related_points[element.neighb->second.index]), element.neighb->first));
			}

			//qDebug() << "Closests: " << found_uvs;

			switch ((int)found_uvs.size())
			{
				case 1:		//the closest is the most safer - pick that uv
				{
					uvs_for_point.push_back(texture_data(i, 0, found_uvs[0].second));
				//	qDebug() << " --- " << triangle_point <<  "New UV 1: " << found_uvs[0].second;
					break;
				}
				case 2:		//we have 2 close uv -> interpolate
				{
				//	uvs_for_point.push_back(UV(i, found_uvs[1].second));
					uv new_uv = calc_new_uv_from_closest_two(found_uvs, triangle_point);
					uvs_for_point.push_back(texture_data(i, 0, new_uv));
				//	qDebug() << " --- " << triangle_point << "New UV 2: " << new_uv;
					break;
				}
				case 3:		//we have 3 clear close uv -> "interpolate"
				{
				//	uvs_for_point.push_back(UV(i, found_uvs[2].second));

					uv new_uv = calc_new_uv_from_closest_three(found_uvs, triangle_point);
					uvs_for_point.push_back(texture_data(i, 0, new_uv));

				//	std::pair<triple<uv>, uv> asd(triple<uv>(found_uvs[0].second, found_uvs[1].second, found_uvs[2].second), new_uv);
				//	n_uvs.push_back(asd);
				//	qDebug() << " --- " << triangle_point << "New UV 3: " << new_uv;
					break;
				}
				default:
					printf("BIG FAIL - kd tree fail\n");
			}
		}
	}

	return uvs_for_point;
}

uv Texturer::calc_new_uv_from_closest_three(std::vector<p_texture_t>& closest_three, Point triangle_point)
{
	Vector da = closest_three[0].first.second - triangle_point;
	Vector db = closest_three[1].first.second - triangle_point;
	Vector dc = closest_three[2].first.second - triangle_point;

	float d_da = sqrt(da.squared_length());
	float d_db = sqrt(db.squared_length());
	float d_dc = sqrt(dc.squared_length());

	if (d_da == 0) return closest_three[0].second;
	if (d_db == 0) return closest_three[1].second;
	if (d_dc == 0) return closest_three[2].second;

	Point closest_distances_from_point(1.0f / d_da, 1.0f / d_db, 1.0f / d_dc);
	//closest_distances_from_point.normalize();
	float sum_sq = sqrt(closest_distances_from_point.x()*closest_distances_from_point.x() + 
						closest_distances_from_point.y()*closest_distances_from_point.y() + 
						closest_distances_from_point.z()*closest_distances_from_point.z());

	closest_distances_from_point = Point(closest_distances_from_point.x() / sum_sq, closest_distances_from_point.y() / sum_sq, closest_distances_from_point.z() / sum_sq);

	std::pair<float, float> c_a = closest_three[0].second;
	std::pair<float, float> c_b = closest_three[1].second;
	std::pair<float, float> c_c = closest_three[2].second;

	float nd_a = closest_distances_from_point.x();
	float nd_b = closest_distances_from_point.y();
	float nd_c = closest_distances_from_point.z();

	float sum = nd_a + nd_b + nd_c;
	std::pair<float, float> new_uv = (c_a * nd_a + c_b * nd_b + c_c * nd_c) / std::pair<float, float>(sum, sum);
	return new_uv;
}

uv Texturer::calc_new_uv_from_closest_two(std::vector<p_texture_t>& closest_two, const Point& triangle_point)
{
	float diff1 = (closest_two[0].first.second - triangle_point).squared_length();
	float diff2 = (closest_two[1].first.second - triangle_point).squared_length();

	float a = diff1 / (diff2 + diff1);
	float b = diff2 / (diff2 + diff1);

	std::pair<float, float> new_uv = closest_two[0].second * b + closest_two[1].second * a;

	return new_uv;
}

/*****************
calculation new uv coordinates
*****************/

std::pair<Texturer::texture_tuple, Texturer::process_state> Texturer::calc_camera_based_uv(const std::vector<uv_map>& triangle_uvs, const std::vector<index_point>& one_triangle_points)
{
	std::vector< std::vector< p_texture_t > > uvs_by_texture;		//how many uv s we have from each camera
	uvs_by_texture.resize(number_of_camera);

	std::vector<index_point> point_indicies;		//related indicies for prediction
	std::map<index, std::pair<Point, unsigned int> > p_pos_map;
	/*-----------------------------------
	Separate the uvs by camera
	---------------------- */

	for (size_t i = 0; i < triangle_uvs.size(); ++i)
	{
		auto point_uvs = triangle_uvs[i];
		index point_index = one_triangle_points[i].first;
		auto point = one_triangle_points[i].second;
		p_pos_map.insert(std::pair<index, std::pair<Point, unsigned int>>(point_index, std::pair<Point, unsigned int>(point, (unsigned int)i)));

		point_indicies.push_back(index_point(point_index, point));

		for (auto& _uv : point_uvs)																		
			uvs_by_texture[_uv.device_id].push_back(p_texture_t(index_point(point_index, point), _uv.texture_coordinate));
	}

	/*for (int i = 0; i < number_of_camera; ++i)
	{
		qDebug() << "\t" << i << " cam : " << uvs_by_texture[i].size();
		for (auto& it : uvs_by_texture[i])
			qDebug() << it;
	}*/

	/*-----------------------------------
	Getting most proper uv - prediction etc, TODO
	---------------------- */
	//qDebug() << "Pos map\n\t " << p_pos_map;

	std::map<process_state, std::vector<texture_tuple>> possible_outs;

	for (int i = 0; i < number_of_camera; ++i)
	{
		auto uvs = uvs_by_texture[i];

		if (uvs.size() < 3) continue;

						// GOOD					//BAD
		std::pair<std::vector<p_texture_t>,std::vector<p_texture_t>> res_uvs = calc_texture_uvs_from_uv_list(uvs);
		std::vector<p_texture_t> res;

		res.resize(3);
		for (auto& it : res_uvs.first)
			res[p_pos_map[it.first.first].second] = p_texture_t(index_point(it.first.first, p_pos_map[it.first.first].first), it.second);

		for (auto& it : res_uvs.second)
			res[p_pos_map[it.first.first].second] = p_texture_t(index_point(it.first.first, p_pos_map[it.first.first].first), it.second);

		switch ((int)res_uvs.first.size())
		{
			case 1:
			{
				//qDebug() << " partition";
				auto out = &possible_outs.insert(std::pair<process_state, std::vector<texture_tuple>>(PARTITION, std::vector<texture_tuple>()));
				out->first->second.push_back(texture_tuple(i, res));

				break;
			}
			case 2:
			{
				index ta = p_pos_map[res_uvs.first[0].first.first].second;
				index tb = p_pos_map[res_uvs.first[1].first.first].second;
				index a = ta < tb ? ta : tb;
				index b = ta > tb ? ta : tb;

				if (calc_clip(res, a, b))		//return bool for test
				{

					auto out = &possible_outs.insert(std::pair<process_state, std::vector<texture_tuple>>(CLIPPED, std::vector<texture_tuple>()));
					out->first->second.push_back(texture_tuple(i, res));
						
					break;
				}

				auto out = &possible_outs.insert(std::pair<process_state, std::vector<texture_tuple>>(UNDEFINED, std::vector<texture_tuple>()));
				out->first->second.push_back(texture_tuple(i, res)); 

				break;
				}
			case 3:
			{
				auto out = &possible_outs.insert(std::pair<process_state, std::vector<texture_tuple>>(OK, std::vector<texture_tuple>()));
				out->first->second.push_back(texture_tuple(i, res));
				
				break;
			}
		}
		
	}
	
	if (possible_outs.empty())
	{
		std::vector<p_texture_t> to_partition;
		for (auto& it : point_indicies) to_partition.push_back(std::pair<index_point, uv>(it, uv()));
		return std::pair<texture_tuple, process_state>(texture_tuple(-1, to_partition), PARTITION);
	}

	auto pos_out = possible_outs.begin();

	switch (pos_out->first)
	{
		case UNDEFINED:
			printf("Undefined tex\n");
	}

	return std::pair<texture_tuple, process_state>(pos_out->second[0], pos_out->first);
}

std::pair<std::vector<Texturer::p_texture_t>, std::vector<Texturer::p_texture_t>> Texturer::calc_texture_uvs_from_uv_list(std::vector< p_texture_t > uvs)
{
	std::vector<tree_2d_element> tmp_tree_elements;
	std::set<index> related_indicies;

	//qDebug() << "UVS FROM CHOOSE";

	for (auto& it : uvs)
	{
		tmp_tree_elements.push_back(tree_2d_element(it.second, it.first.first));
		related_indicies.insert(it.first.first);
	//	if(uvs.size() ==4) qDebug() << "\t" << it.first.first << " : " << it.first.second <<  " - " << it.second;
	}
	
	tmp_tree.release();
	tmp_tree.build_process(tmp_tree_elements, Box(0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f));

	//--------------------------------	Tree builded

	kNearest_queue_2d kq;

	int counter = 0;
	Point act;

	std::pair<float, std::vector<p_texture_t>> good;
	std::pair<float, std::vector<p_texture_t>> bad;
	good.first = bad.first = FLT_MAX;

	for (auto& _uv : uvs)
	{
	//	if (counter != 0 && uvs.size() == 3) break;
		if (counter == 0 || act != _uv.first.second)
		{
			counter++;
			if (counter == 3) break;
			act = _uv.first.second;
		}

		std::vector<p_texture_t> found_uvs_g;
		std::vector<p_texture_t> found_uvs_b;
		float sum_g = 0.0f;
		float sum_b = 0.0f;

		tmp_tree.knearest_p_conditional(&_uv.second, tmp_tree.get_root_ptr(), kq, related_indicies);

		while (!kq.empty())
		{
			auto element = kq.top();
			kq.pop();
			//if (uvs.size() > 3) qDebug() << "\n\t" << element.distance << " - " << element.neighb->second.index << " " << element.neighb->first;
			if (element.distance <= TEX_CALC_EPSILON2)
			{
				sum_g += element.distance;
				found_uvs_g.push_back(p_texture_t(index_point(element.neighb->second.index, Point()), element.neighb->first));
			}
			else
			{
				sum_b += element.distance;
				found_uvs_b.push_back(p_texture_t(index_point(element.neighb->second.index, Point()), element.neighb->first));
			}
		}

		/*if (uvs.size() >= 3)
		{
			qDebug() << "\tClosests: ";
			for (auto& it : found_uvs_g)
				qDebug() << "\t\t" << it.first.first << " - " << it.second;
			qDebug() << "\t\tBad: ";
			for (auto& it : found_uvs_b)
				qDebug() << "\t\t" <<it.first.first << " - " << it.second;
		}
		*/
		//if (uvs.size() > 3) qDebug() << sum_g << " < " << good.first;

		if (good.second.size() < found_uvs_g.size() || (good.first > sum_g && good.second.size() == found_uvs_g.size()) || (good.first == sum_g && bad.first > sum_b))
		{
		//	qDebug() << "swap";
			good.first = sum_g;
			bad.first = sum_b;
			good.second = found_uvs_g;
			bad.second = found_uvs_b;
		}
	}

	/*if (uvs.size() > 3)
	{
		qDebug() << "Out: ";
		for (auto& it : good.second)
			qDebug() << it.first.first << " - " << it.second;
		for (auto& it : bad.second)
			qDebug() << it.first.first << " - " << it.second;
		qDebug() << "\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/";
	} */

	return std::pair <std::vector<p_texture_t>, std::vector<p_texture_t>>(good.second, bad.second);
}

/*****************
	Partition
*****************/

void Texturer::partition_by_side(std::vector<index_point>& one_triangle_points, std::vector<uv_map>& uv_cache)
{
	Point p1 = one_triangle_points[0].second;
	Point p2 = one_triangle_points[1].second;
	Point p3 = one_triangle_points[2].second;
	unsigned int i1 = one_triangle_points[0].first;
	unsigned int i2 = one_triangle_points[1].first;
	unsigned int i3 = one_triangle_points[2].first;

	Point p12 = (p1 + p2) / 2.0f;
	Point p23 = (p2 + p3) / 2.0f;
	Point p13 = (p1 + p3) / 2.0f;

	//TODO: find it in the point list... -> we should store it
	unsigned int i12 = poi.size();
	poi.push_back(p12);
	unsigned int i23 = poi.size();
	poi.push_back(p23);
	unsigned int i13 = poi.size();
	poi.push_back(p13);

	//qDebug() << res_points.size();
	/*	qDebug() << qSetRealNumberPrecision(15) << "\to1 " << i1 << " " << p1;
	qDebug() << qSetRealNumberPrecision(15) << "\to2 " << i2 << " " << p2;
	qDebug() << qSetRealNumberPrecision(15) << "\to3 " << i3 << " " << p3;

	qDebug() << qSetRealNumberPrecision(15) << "\tpart trip1 " << res_points.size() - 3 << " " << p12;
	qDebug() << qSetRealNumberPrecision(15) << "\tpart trip2 " << res_points.size() - 2 << " " << p23;
	qDebug() << qSetRealNumberPrecision(15) << "\tpart trip3 " << res_points.size() - 1 << " " << p13;
	*/
	//add the extra fields
	uv_cache.push_back(uv_map());
	uv_cache.push_back(uv_map());
	uv_cache.push_back(uv_map());

	uv_maps.push_back(std::vector< std::map<uv, index> >());
	uv_maps.back().resize(number_of_camera);
	uv_maps.push_back(std::vector< std::map<uv, index> >());
	uv_maps.back().resize(number_of_camera);
	uv_maps.push_back(std::vector< std::map<uv, index> >());
	uv_maps.back().resize(number_of_camera);

	one_triangle_points.clear();
	one_triangle_points.push_back(std::pair<unsigned int, Point>(i1, p1));
	one_triangle_points.push_back(std::pair<unsigned int, Point>(i12, p12));
	one_triangle_points.push_back(std::pair<unsigned int, Point>(i13, p13));

	one_triangle_points.push_back(std::pair<unsigned int, Point>(i12, p12));
	one_triangle_points.push_back(std::pair<unsigned int, Point>(i2, p2));
	one_triangle_points.push_back(std::pair<unsigned int, Point>(i23, p23));
	
	one_triangle_points.push_back(std::pair<unsigned int, Point>(i12, p12));
	one_triangle_points.push_back(std::pair<unsigned int, Point>(i23, p23));
	one_triangle_points.push_back(std::pair<unsigned int, Point>(i13, p13));

	one_triangle_points.push_back(std::pair<unsigned int, Point>(i23, p23));
	one_triangle_points.push_back(std::pair<unsigned int, Point>(i3, p3));
	one_triangle_points.push_back(std::pair<unsigned int, Point>(i13, p13));
}

/*****************
	Clip
*****************/

bool Texturer::calc_clip(std::vector< p_texture_t >& uvs, index& g1, index& g2)
{
	//qDebug() << g1 << " - " << g2;
	Point p1 = g1 < g2 ? uvs[g1].first.second : uvs[g2].first.second;
	Point p2 = g1 > g2 ? uvs[g1].first.second : uvs[g2].first.second;
	Point p3 = uvs[3 - g1 - g2].first.second;
	index i1 = g1 < g2 ? uvs[g1].first.first : uvs[g2].first.first;
	index i2 = g1 > g2 ? uvs[g1].first.first : uvs[g2].first.first;
	index i3 = uvs[3 - g1 - g2].first.first;

//	qDebug() << p1 << " - " << p2 << " , " << p3;

	uv u1 = g1 < g2 ? uvs[g1].second : uvs[g2].second;
	uv u2 = g1 > g2 ? uvs[g1].second : uvs[g2].second;
	uv u3 = uvs[3 - g1 - g2].second;
	
//	qDebug() << u1 << " - " << u2 << " , " << u3;

	Point part1[2]; // 0 from , 1 to
	Point part2[2];
	Point part3[2];
	uv u_1[2];		
	uv u_2[2];
	uv u_3[2];
	int pos1[2];
	int pos2[2];
	int pos3[2];


	int ok = 0;

	calc_partition_point(p1, p3, u1, u3, part1, u_1, pos1);
	calc_partition_point(p2, p3, u2, u3, part2, u_2, pos2);
	//calc_partition_point(p1, p2, u1, u2, part3, u_3, pos3);		//Assume that the base edge is good enough -> epsilon is well defined

	index act_i = poi.size();
	uvs.clear();

	auto part_on_first_edge = pos1[0] + pos1[1];
	auto part_on_second_edge = pos2[0] + pos2[1];

	if (part_on_first_edge >= N && part_on_second_edge >= N)	//mindketto atert egymason -> legyen a felezopontjuk az uj pont
	{
	//	qDebug() << "Type 1";

		Point _p0 = (part1[0] + part1[1]) / 2.0f;
		Point _p2 = (part2[0] + part2[1]) / 2.0f;

		uvs.resize(9);

		uvs[g1] = p_texture_t(index_point(act_i, _p0), u_1[1]);
		uvs[g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[1]);
		uvs[3 - g1 - g2] = p_texture_t(index_point(i3, p3), u3);
		uvs[3 + g1] = p_texture_t(index_point(i1, p1), u1);
		uvs[3 + g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[0]);
		uvs[3 + 3 - g1 - g2] = p_texture_t(index_point(act_i, _p0), u_1[0]);
		uvs[6 + g1] = p_texture_t(index_point(i1, p1), u1);
		uvs[6 + g2] = p_texture_t(index_point(i2, p2), u2);
		uvs[6 + 3 - g1 - g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[0]);
	}
	else if (part_on_first_edge >= N)
	{
	//	qDebug() << "Type 2";

		Point _p0 = (part1[0] + part1[1]) / 2.0f;
		Point _p2 = part2[0];
		Point _p3 = part2[1];
	
		uvs.resize(12);

		uvs[g1] = p_texture_t(index_point(act_i, _p0), u_1[1]);
		uvs[g2] = p_texture_t(index_point(act_i + 2, _p3), u_2[1]);
		uvs[3 - g1 - g2] = p_texture_t(index_point(i3, p3), u3);
		uvs[3 + g1] = p_texture_t(index_point(i1, p1), u1);
		uvs[3 + g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[0]);
		uvs[3 + 3 - g1 - g2] = p_texture_t(index_point(act_i, _p0), u_1[0]);
		uvs[6 + g1] = p_texture_t(index_point(i1, p1), u1);
		uvs[6 + g2] = p_texture_t(index_point(i2, p2), u2);
		uvs[6 + 3 - g1 - g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[0]);

		uvs[9 + g1] = p_texture_t(index_point(act_i, _p0), u_1[1]);
		uvs[9 + g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[0]);
		uvs[9 + 3 - g1 - g2] = p_texture_t(index_point(act_i + 2, _p3), u_2[1]);
	}
	else if (part_on_second_edge >= N)
	{
	//	qDebug() << "Type 3";

		Point _p0 = part1[0];
		Point _p1 = part1[1];
		Point _p2 = (part2[0] + part2[1]) / 2.0f;

		uvs.resize(12);

		uvs[g1] = p_texture_t(index_point(act_i + 2, _p1), u_1[1]);
		uvs[g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[1]);
		uvs[3 - g1 - g2] = p_texture_t(index_point(i3, p3), u3);
		uvs[3 + g1] = p_texture_t(index_point(i1, p1), u1);
		uvs[3 + g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[0]);
		uvs[3 + 3 - g1 - g2] = p_texture_t(index_point(act_i, _p0), u_1[0]);
		uvs[6 + g1] = p_texture_t(index_point(i1, p1), u1);
		uvs[6 + g2] = p_texture_t(index_point(i2, p2), u2);
		uvs[6 + 3 - g1 - g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[0]);

		uvs[9 + g1] = p_texture_t(index_point(act_i, _p0), u_1[0]);
		uvs[9 + g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[0]);
		uvs[9 + 3 - g1 - g2] = p_texture_t(index_point(act_i + 2, _p1), u_1[1]);
	}
	else
	{
	//	qDebug() << "Type 4";

		Point _p0 = part1[0];
		Point _p1 = part1[1];
		Point _p2 = part2[0];
		Point _p3 = part2[1];

		uvs.resize(15);

		uvs[g1] = p_texture_t(index_point(act_i + 2, _p1), u_1[1]);
		uvs[g2] = p_texture_t(index_point(act_i + 3, _p3), u_2[1]);
		uvs[3 - g1 - g2] = p_texture_t(index_point(i3, p3), u3);
		uvs[3 + g1] = p_texture_t(index_point(i1, p1), u1);
		uvs[3 + g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[0]);
		uvs[3 + 3 - g1 - g2] = p_texture_t(index_point(act_i, _p0), u_1[0]);
		uvs[6 + g1] = p_texture_t(index_point(i1, p1), u1);
		uvs[6 + g2] = p_texture_t(index_point(i2, p2), u2);
		uvs[6 + 3 - g1 - g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[0]);

		uvs[9 + g1] = p_texture_t(index_point(act_i, _p0), u_1[0]);
		uvs[9 + g2] = p_texture_t(index_point(act_i + 3, _p3), u_2[1]);
		uvs[9 + 3 - g1 - g2] = p_texture_t(index_point(act_i + 2, _p1), u_1[1]);

		uvs[12 + g1] = p_texture_t(index_point(act_i, _p0), u_1[0]);
		uvs[12 + g2] = p_texture_t(index_point(act_i + 1, _p2), u_2[0]);
		uvs[12 + 3 - g1 - g2] = p_texture_t(index_point(act_i + 3, _p3), u_2[1]);
	}

	return true;
}

void Texturer::calc_partition_point(const Point& p1, const Point& p2, const uv& uv1, const uv& uv2, Point r_p[2], uv r_uv[2], int pos[2])
{
	static int i = 0;

	Vector v = p2 - p1;
	float len = sqrt(v.squared_length());
	v = v / len;
	float t = len / N;

	std::vector<uv_map> test;

	r_p[0] = p1;
	r_uv[0] = uv1;
	pos[0] = 0;

	int step = 1;
	//from p1 to p2
	while (true)		//find the last uv which is closer than epsilon -> this means that all the triangle points will take uv from 1 place
	{
		if (step > N) break;
		auto pv = p1 + step * t * v;
		r_p[0] = pv;
		pos[0] = step;		//	first which is different

		kNearest_queue_3d closest_points;
		tree.knearest_p(&pv, tree.get_root_ptr(), closest_points);
		uv_map modified_uvs = calc_new_uv_coords_from_knearest(closest_points, pv);
	
		test.push_back(modified_uvs);
	
		uv rel_uv;
		auto loc = std::find_if(modified_uvs.begin(), modified_uvs.end(), [uv1, &rel_uv](const texture_data&_uv) { rel_uv = _uv.texture_coordinate; return pair_length(uv1 - rel_uv) < TEX_CALC_EPSILON2; });
		if (loc != modified_uvs.end())
		{
			r_uv[0] = rel_uv;
		}
		else
		{
			break;
		}

		step++;
	}

	if (step > N)	//ha az elõzõ végig ért -> fordítva is feltehetjük, hogy ok
	{
		r_p[1] = p1;
		r_uv[1] = uv1;
		pos[1] = N;

		/*if (i < 3)
		{
			qDebug() << "---------Same - OK------\n";
			qDebug() << p1 << " , " << uv1 << " - " << p2 << " , " << uv2;
			for (auto& it : test)
			{
				qDebug() << "\t--";
				for (auto&it2 : it)
					qDebug() << "\t\t" << it2.second;
			}
			qDebug() << "pos : " << r_p[0] << " - " << r_p[1];
			qDebug() << "Calc res: " << r_uv[0] << " - " << r_uv[1];
		}
		qDebug() << "Same - OK";*/
		++i;
		return;
	}

	//qDebug() << "\t" << p1 << " , " << uv1 << " - " << p2 << " , " << uv2;

	r_p[1] = p2;
	r_uv[1] = uv2;
	pos[1] = 0;

	step = N - 1;
	//from p2 to p1
	while (true)		//find the last uv which is closer than epsilon -> this means that all the triangle points will take uv from 1 place
	{
		if (step < 0) break;

		auto pv = p1 + step * t * v;
		r_p[1] = pv;					//	first which is different
		pos[1] = N - step;

		kNearest_queue_3d closest_points;
		tree.knearest_p(&pv, tree.get_root_ptr(), closest_points);
		uv_map modified_uvs = calc_new_uv_coords_from_knearest(closest_points, pv);
		
		test.push_back(modified_uvs);

		uv rel_uv;
		auto loc = std::find_if(modified_uvs.begin(), modified_uvs.end(), [uv2, &rel_uv](const texture_data&_uv) { rel_uv = _uv.texture_coordinate; return pair_length(uv2 - rel_uv) < TEX_CALC_EPSILON2; });
		if (loc != modified_uvs.end())
		{
			r_uv[1] = rel_uv;
		}
		else
		{
			break;
		}

		step--;
	}

	if (pair_length(uv1 - uv2) < TEX_CALC_EPSILON2)
	{
		/*qDebug() << "------------------------\n";
		qDebug() << p1 << " , " << uv1 << " - " << p2 << " , " << uv2;
		for (auto& it : test)
		{
			qDebug() << "\t--";
			for (auto&it2 : it)
				qDebug() << "\t\t" << it2.second;
		}
		qDebug() << "pos : " << r_p[0] << " - " << r_p[1];
		qDebug() << "Calc res: " << r_uv[0] << " - " << r_uv[1];

		qDebug() << pos[0] << " - " << pos[1];*/
	}

	if (i < 3)
	{
	/*	qDebug() << "------------------------\n";
		qDebug() << p1 << " , " << uv1 << " - " << p2 << " , " << uv2;
		for (auto& it : test)
		{
			qDebug() << "\t--";
			for (auto&it2 : it)
				qDebug() << "\t\t" << it2.second;
		}
		qDebug() << "pos : " << r_p[0] << " - " << r_p[1];
		qDebug() << "Calc res: " << r_uv[0] << " - " << r_uv[1];

		qDebug() << pos[0] << " - " << pos[1];*/
	}
	++i;

}

uv_map Texturer::calc_good_uv(Point& t_point, const std::vector<tree_3d_element>& poi_a)
{
	uv_map new_uvs;	//this will return

	kNearest_queue_3d kq;
	tree.set_kn(1);
	tree.knearest_p(&t_point, tree.get_root_ptr(), kq);
	tree.set_kn(3);

	auto ele = kq.top();
	kq.pop();

	int cam = 0;

	for (auto& it : ele.neighb->second.uvs)
	{
		std::map<index, std::vector<std::pair<float, uv>>> clo;		// index - distance uv pairs from the closest point's uv

		//find the uvs in R range (on 0-1 x 0-1 map)
		uv_trees[cam]->all_nearest_in_range_from_one_obj(&it.texture_coordinate, uv_trees[cam]->get_root_ptr(), NEAREST_RANGE, clo, it.object_id);

		//qDebug() << "uv: " << it.second << "\t found in range: " << clo.size();
		add_range_datas(clo, it.texture_coordinate);

		std::set<std::pair<float, std::pair<Point, uv>>, testt> uv_ordered;	//ordered by distance the uvs
		std::vector<p_texture_t> tee;
		std::vector<uv> tt2;
		std::map<Point, index> pmap;

		for (auto& it : clo)
		{
			Point p = poi_a[it.first].first;
			float dist = (p - t_point).squared_length();
			pmap.insert(std::pair<Point, index>(p, it.first));

			for (auto& itv : it.second)
			{
				uv_ordered.insert(std::pair<float, std::pair<Point, uv >>(dist, std::pair<Point, uv >(p, itv.second)));
				/*if (tee.size() < 3)
				{
					tee.push_back(p_texture_t(index_point(it.first, p), itv.second));
					tt2.push_back(itv.second);
				}*/
			}
		}

	/*	for (auto& itv : uv_ordered)
		{
			tee.push_back(p_texture_t(index_point(pmap[itv.second.first], itv.second.first), itv.second.second));
			tt2.push_back(itv.second.second);
			if (tt2.size() == 3) break;
		}

		if (tee.size() == 3)
		{
			uv new_uv = calc_new_uv_from_closest_three(tee, t_point);
			new_uvs.push_back(UV(0, 0, new_uv));
			tt2.push_back(new_uv);
			test_ch.push_back(tt2);
			continue;
		}*/

		//calculate the uv triple
		/*qDebug() << "_----------------\ndd";
		for (auto& it : uv_ordered)
		{
			qDebug() << it.first << " - " << it.second;
		}*/
		std::vector<std::pair<float, std::pair<Point, uv>>> res = get_closest_proper_triangle2(t_point, std::vector<std::pair<float, std::pair<Point, uv>>>(uv_ordered.begin(), uv_ordered.end()));
		
		std::vector<uv> tt;
		if (res[1].first == -1.0f)
		{
			printf("coudlnt find!\n");
			continue;
		}
		std::vector<p_texture_t> aa;
		for (auto& it : res)
		{
			aa.push_back(p_texture_t(index_point(0, it.second.first), it.second.second));
			tt.push_back(it.second.second);
		}
		uv new_uv = calc_new_uv_from_closest_three(aa, t_point);
		new_uvs.push_back(texture_data(0, 0, new_uv));

		//std::vector<std::pair<float, std::pair<Point, uv>>> res = get_closest_proper_triangle(t_point, std::vector<std::pair<float, std::pair<Point, uv>>>(uv_ordered.begin(), uv_ordered.end()));

	/*	if (res[2].first != FLT_MAX)	//Did we find any good triangle? -> increase some default vals if not
		{
			std::vector<uv> tt;

			std::vector<p_texture_t> aa;
			tt.push_back(res[0].second.second);
			aa.push_back(p_texture_t(index_point(0, res[0].second.first), res[0].second.second));
			tt.push_back(res[1].second.second);
			aa.push_back(p_texture_t(index_point(0, res[1].second.first), res[1].second.second));
			tt.push_back(res[2].second.second);
			aa.push_back(p_texture_t(index_point(0, res[2].second.first), res[2].second.second));
			uv new_uv = calc_new_uv_from_closest_three(aa, t_point);
			tt.push_back(new_uv);
			test_ch.push_back(tt);

			new_uvs.push_back(UV(0, new_uv));
		}
		else
		{
			qDebug() << " didnt find any good triangle";
			std::vector<uv> tt;
			tt.push_back(res[0].second.second);
			test_ch.push_back(tt);

			new_uvs.push_back(UV(0, res[0].second.second));

		}
		*/
	}

	return new_uvs; 
}

//????
std::vector<std::pair<float, std::pair<Point, uv>>> Texturer::get_closest_proper_triangle2(const Point& point, const std::vector<std::pair<float, std::pair<Point, uv>>>& uv_ordered)
{
	std::vector<std::pair<float, std::pair<Point, uv>>> res;	//the result, uv distance, related point, uv

	res.resize(3);
	res[0] = uv_ordered[0];
	res[1] = std::pair<float, std::pair<Point, uv>>(-1.0f, std::pair<Point, uv>());
	res[2] = std::pair<float, std::pair<Point, uv>>(-1.0f, std::pair<Point, uv>());
	float best_angle = 0.0f;
	bool already_have = false;
	//if (uv_ordered.size() < 3) qDebug() << "\t too few";

	for (int i = 0 + 1; i < uv_ordered.size() - 1; ++i)
		for (int j = i + 1; j < uv_ordered.size(); ++j)
		{
			auto first_uv = uv_ordered[0].second.second;
			auto candidate_uv1 = uv_ordered[i].second.second;
			auto candidate_uv2 = uv_ordered[j].second.second;

			if (already_have && !is_proper_triangle(first_uv, candidate_uv1, candidate_uv2))
			{
			//	qDebug() << "its NOT ok";
				continue;
			}

			auto first_point = uv_ordered[0].second.first;
			auto candidate_point1 = uv_ordered[i].second.first;
			auto candidate_point2 = uv_ordered[j].second.first;

			if (first_point == candidate_point1 || candidate_point1 == candidate_point2 || candidate_point2 == first_point) continue;

			if ( already_have &&
				((first_point - candidate_point1).squared_length() >  REAL_TRIANG_MAX_DIST ||
				(first_point - candidate_point2).squared_length() >  REAL_TRIANG_MAX_DIST ||
				(candidate_point1 - candidate_point2).squared_length() > REAL_TRIANG_MAX_DIST))
				continue;

			Point surfp_cpy = point;
			Vector unit_x, unit_y, unit_z;
			//calculate the local coordinate system -> false if the points are collinear TODO!
			auto ok = tex_utils::calc_new_coord_system(first_point, candidate_point1, candidate_point2, unit_x, unit_y, unit_z);
			if (!ok)
				continue;

			//calculate the projected points on the local coordinate system
			auto proj_pois = tex_utils::calc_projected_points(first_point, candidate_point1, candidate_point2, unit_x, unit_y, unit_z, surfp_cpy);

			auto normal = (point - uv_ordered[0].second.first) - Vector(surfp_cpy.x(), surfp_cpy.y(), surfp_cpy.z());
	
			Point center = (first_point + candidate_point1 + candidate_point2) / 3.0f;
			auto surf_to_center = Vector(center.x(), center.y(), center.z()) - ((point - uv_ordered[0].second.first));
			auto angle = CGAL::scalar_product(normal, surf_to_center) / sqrt(normal.squared_length()) / sqrt(surf_to_center.squared_length());
			//qDebug() << "\t\t a" << angle;

			if (best_angle == 0.0f || best_angle < angle)
			{
				best_angle = angle;
				res[1] = uv_ordered[i];
				res[2] = uv_ordered[j];
				already_have = true;
			}
		}
	//qDebug() << best_angle << " is the best";
	return res;
}

bool Texturer::is_proper_triangle(uv& first_uv, uv& candidate_uv1, uv& candidate_uv2)
{
	auto s_a = candidate_uv1 - first_uv;
	auto s_b = candidate_uv2 - first_uv;
	
	auto sp_a = scalar_product(s_a, s_b) / pair_length(s_a) / pair_length(s_b);
	if (sp_a < 0.0f) return false;

	s_a = first_uv - candidate_uv1;
	s_b = candidate_uv2 - candidate_uv1;

	auto sp_b = scalar_product(s_a, s_b) / pair_length(s_a) / pair_length(s_b);
	if (sp_b < 0.0f) return false;

	s_a = first_uv - candidate_uv2;
	s_b = candidate_uv1 - candidate_uv2;

	auto sp_c = scalar_product(s_a, s_b) / pair_length(s_a) / pair_length(s_b);
	if (sp_c < 0.0f) return false;

	//qDebug() << "its ok";
	return true;
}

std::vector<std::pair<float, std::pair<Point, uv>>> Texturer::get_closest_proper_triangle(const Point& point, const std::vector<std::pair<float, std::pair<Point, uv>>>& uv_ordered)
{
	std::vector<std::pair<float, std::pair<Point, uv>>> res;	//the result, uv distance, related point, uv

	//qDebug() << "size" << uv_ordered.size();
	/*for (auto& it : uv_ordered)
		qDebug() << it.first << " " << it.second;
*/
	res.resize(3);
	res[0] = uv_ordered[0];
	res[1] = std::pair<float, std::pair<Point, uv>>(FLT_MAX,  std::pair<Point, uv>());
	res[2] = std::pair<float, std::pair<Point, uv>>(FLT_MAX,  std::pair<Point, uv>());
	float best = 0.0f;
	static int aa = 0;
	//we try to find that triangle where we can project our point and can get the uv
	//we choose that one which uv's is the closest to each other
	//for (int k = 0; k < uv_ordered.size() - 2; ++k)
		for (int i = 0 + 1; i < uv_ordered.size() - 1; ++i)
		for (int j = i+1; j < uv_ordered.size(); ++j)
		{
			auto first_point = uv_ordered[0].second.first;
			auto candidate_point1 = uv_ordered[i].second.first;
			auto candidate_point2 = uv_ordered[j].second.first;
			/*if (aa == 0)
				qDebug() << (first_point - point).squared_length() << " - " << (point - candidate_point2).squared_length() << "- " << (candidate_point1 - point).squared_length();
			++aa;*/

			//if the original surf points are far from each other then dont calculate anything
			if ((first_point - candidate_point1).squared_length() >  REAL_TRIANG_MAX_DIST ||
				(first_point - candidate_point2).squared_length() >  REAL_TRIANG_MAX_DIST ||
				(candidate_point1 - candidate_point2).squared_length() > REAL_TRIANG_MAX_DIST)
				continue;

			auto candidate_dist0 = uv_ordered[0].first;
			auto candidate_dist1 = uv_ordered[i].first;
			auto candidate_dist2 = uv_ordered[j].first;

			//if the currently best solution farthest uv is closer than the actual then dont calculate anything
			if (candidate_dist0 >= res[2].first) break;
			if (candidate_dist1 >= res[2].first) break;
			if (candidate_dist2 >= res[2].first) continue;

			Point surfp_cpy = point;
			Vector unit_x, unit_y, unit_z;
			//calculate the local coordinate system -> false if the points are collinear TODO!
			auto ok = tex_utils::calc_new_coord_system(first_point, candidate_point1, candidate_point2, unit_x, unit_y, unit_z);
			if (!ok)
				continue;

			//calculate the projected points on the local coordinate system
			auto proj_pois = tex_utils::calc_projected_points(first_point, candidate_point1, candidate_point2, unit_x, unit_y, unit_z, surfp_cpy);
	
			if (tex_utils::is_inside_triangle2(surfp_cpy, proj_pois[0],proj_pois[1], proj_pois[2]))
			{
				auto act_d = uv_ordered[i].first + uv_ordered[j].first;
		
				if (best == 0.0f || best > act_d)
				{
					best = act_d;
				//	res[0] = uv_ordered[0];
					res[1] = uv_ordered[i];
					res[2] = uv_ordered[j];

				}
			}
		}
	return res;
}

void Texturer::add_range_datas(const std::map<index, std::vector<std::pair<float, uv>>>& clo, const uv& _uv)
{
	//std::vector<uv> tt;

	for (auto& cc : clo)
	{
		if (cc.second.size() > 1) printf("Good we thought this\n");
		for (auto& cc_2 : cc.second)
		{
			if (pair_length(cc_2.second - _uv) <= NEAREST_RANGE)
			{
				//	qDebug() << "c - " << cc_2.second << " - " << cc.first;
			//	tt.push_back(cc_2.second);
				break;
			}
			else
				printf("BAAD\n");
		}
	}
	//tt.push_back(_uv);
	//test_ch.push_back(tt);
}