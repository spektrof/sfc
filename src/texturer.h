#pragma once
#include <vector>
#include <map>
#include "texture_types.h"
#include "reconstructor_utils.h"
#include "kdtree.h"

class Texturer
{
	typedef kdTree<tree_3d_element, Point> tree_3d;
	typedef kdTree<tree_2d_element, uv, 2> tree_2d;
	typedef tree_3d::kNearest_queue kNearest_queue_3d;
	typedef tree_2d::kNearest_queue kNearest_queue_2d;
	//----------------------------
	enum process_state
	{
		OK,
		CLIPPED,
		PARTITION,
		UNDEFINED
	};

	struct testt {
		bool operator()(const std::pair<float, std::pair<Point, uv>>& c, const std::pair<float, std::pair<Point, uv>>& _c) {
			return c.first < _c.first || (c.first == _c.first && c.second.first < _c.second.first) || (c.first == _c.first && c.second.first == _c.second.first && c.second.second < _c.second.second);
		}
	};

	typedef unsigned int index;
	typedef std::pair<index, Point> index_point;
	typedef std::pair<index, index> pi_uv_p;
				// poi ind  camera separated uv
	typedef std::vector<pi_uv_p> point_data;

				  //point index	  uv
	typedef std::pair<index_point, uv> p_texture_t;
	//					camera						
	typedef std::pair<int16_t, std::vector<p_texture_t>> texture_tuple;

public:
	Texturer() : number_of_camera(0), tree(tree_3d(1,2)), tmp_tree(tree_2d(1,1))
	{
		uv_trees.clear();
		calibration_translates.clear();
	}
	~Texturer()
	{
	}

	void clear()
	{
	//	uv_tree.release();
	//	tree.release();
		calibration_translates.clear();
		number_of_camera = 0;
		poi.clear();
		pi_camuv_pair_ss.clear();
		uv_maps.clear();
	}

	void set_required_data(const std::vector<Point>& ct, const unsigned int& n_of_tex)
	{
		calibration_translates = ct;
		number_of_camera = ct.size() == 0 ? n_of_tex : ct.size();

	/*	for (auto& it : calibration_translates)
			qDebug() << "\t\t" << it;*/
	}

	void add_tex_or_color_info(export_data& untex_paint, pointcloud_data& poi_all);
	
	std::vector<Point> get_points_without_redundance() const { return poi; }
	std::vector < std::vector< std::map<uv, index> >> get_uv_map() const { return uv_maps; }
	std::vector<std::vector<std::pair<index, index>>> get_cam_based_indicies() const { return pi_camuv_pair_ss; }

protected:
	void init_uv_tree(const point_uv_map&, std::vector<std::vector<tree_2d_element>>&, std::vector<tree_3d_element>&);
	bool init_tree(std::vector<tree_3d_element>&, const Box&);

	void adding_empty_uv_data(export_data&);
	void add_tex_to_untex_paint(export_data&);

	void fill_uv_and_index_list(const texture_tuple&);
	//-----------------------------

	void calc_triangle_details(std::vector<index_point>&, std::vector<uv_map>&);
	std::vector<uv_map> calculate_uv_coords_for_triangle(std::vector<index_point>&, std::vector<uv_map>&, bool use_cache = true);

	//--------------------

	uv_map calc_new_uv_coords_from_knearest(kNearest_queue_3d& closest_points, Point& triangle_point);
	uv calc_new_uv_from_closest_three(std::vector<p_texture_t>& closest_three, Point triangle_point);
	uv calc_new_uv_from_closest_two(std::vector<p_texture_t>& closest_two, const Point& triangle_point);

	//--------------------

	std::pair<texture_tuple, process_state> calc_camera_based_uv(const std::vector<uv_map>&, const std::vector<index_point>&);
	std::pair<std::vector<p_texture_t>, std::vector<p_texture_t>> calc_texture_uvs_from_uv_list(std::vector< p_texture_t > uvs);

	//--------------------
	void partition_by_side(std::vector<index_point>& one_triangle_points, std::vector<uv_map>& uv_cache);
	
	bool calc_clip(std::vector< p_texture_t >& uvs, index& g1, index& g2);
	void calc_partition_point(const Point& p1, const Point& p2, const uv& uv1, const uv& uv2, Point r_p[2], uv r_uv[2], int pos[2]);
	void update_var_sizes(std::vector< p_texture_t >& uvs, std::vector<uv_map>&);

	uv_map calc_good_uv(Point&, const std::vector<tree_3d_element>&);
	std::vector<std::pair<float, std::pair<Point, uv>>> get_closest_proper_triangle(const Point& point, const std::vector<std::pair<float, std::pair<Point, uv>>>& uv_ordered);
	std::vector<std::pair<float, std::pair<Point, uv>>> get_closest_proper_triangle2(const Point& point, const std::vector<std::pair<float, std::pair<Point, uv>>>& uv_ordered);
	void add_range_datas(const std::map<index, std::vector<std::pair<float, uv>>>& clo, const uv&);

	bool is_proper_triangle(uv& first_uv, uv& candidate_uv1, uv& candidate_uv2);

private:

	std::vector<tree_2d*> uv_trees;		//camera separated uv tree
	tree_3d tree;						//kd tree from all the point
	tree_2d tmp_tree;					//tmp tree for the calculation

	std::vector<Point> calibration_translates;
	unsigned int number_of_camera;
	std::vector<tree_3d_element> elements_for_3d_tree;

	std::vector<Point> poi;
	std::vector<point_data> pi_camuv_pair_ss;
	std::vector < std::vector< std::map<uv, index> >> uv_maps;

	void print_possible_uv(const std::vector<uv_map>& triangle_uvs,const std::vector<index_point>& one_triangle_points)
	{
	/*	qDebug() << "Possible uvs for each triangle points are:";
		int counter = 0;
		for (size_t i = 0; i < triangle_uvs.size(); ++i)
		{
			qDebug() << "\t" << one_triangle_points[i].first << " " << one_triangle_points[i].second;
			counter += triangle_uvs[i].size();
			for (auto& uv : triangle_uvs[i])
				qDebug() << "\t\t" << uv;
		}
		qDebug() << " count: " << counter;*/
	}

	void print_incoming_uv_for_p(const std::vector< std::set<std::pair<float, p_texture_t>> >& separated_uvs, const Point& triangle_point)
	{
	/*	qDebug() << "Triangle point: " << triangle_point;
		for (int i = 0; i < number_of_camera; ++i)
		{
			qDebug() << i << " camera: ";
			for (auto& it : separated_uvs[i])
				qDebug() << "\t " << it;
		}
		qDebug() << "----------------------\n";*/
	}

};