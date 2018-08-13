#pragma once

#include "kdtreeutils.h"
#include "taskscheduler.h"

#include "utils.h"

#include <set>
#include <queue>
#include <algorithm>
#include <cmath> //for log2

template <typename P, typename P_QUERY, int DIM = 3>
class kdTree
{
	//TODO: later - task struct, task* megy a taskmanagerbe - hátrány nekünk kell megszüntetni, elõny: eltünik a start és a length

	struct nearest_node
	{
		P* neighb;
		float distance;

		nearest_node(P* n = NULL, const float& d = FLT_MAX) : neighb(n), distance(d) {}
	};

	struct HighestOnTop
	{
		bool operator()(const nearest_node &a, const nearest_node &b) const { return a.distance < b.distance; }
	};

public:
	
	//we dont provide nearest node in public, use knearest to find only the closest point -> nearest_p only works inside the class

	typedef std::priority_queue<nearest_node, std::vector<nearest_node>, HighestOnTop > kNearest_queue;	//k nearest neighbour result

private:
	struct node
	{
		node* left, *right;
		node* parent;

		int depth, length;
		Box box;

		P* point;	//pointer to node val
		P* start;	//pointer to starter point in the vector

		nearest_node nn;
		kNearest_queue knn;
	
		node(node* p, P* s, const int& d, const int& lg, Box b, P* po = NULL, node* l = NULL, node* r = NULL, const nearest_node& _nn = nearest_node())
			: parent(p), start(s), depth(d), length(lg), box(b), point(po), left(l), right(r), nn(_nn) {}
	};

	typedef typename node* kdnode_ptr;
	typedef std::pair<float, P*> dist_uv;

public:
	kdTree(const int& ma = 1, const int& thr_c = 2) :
		root(NULL),
		stopBuildDepth(STOPLEVEL),
		kn(1),
		sort_time(duration_f(0.0f)), calc_time(milliseconds(0)),
		mesh_accuracy(ma)
	{
		t_schedular = new TaskSchedular<kdTree, node>();
		set_thread_capacity(thr_c);
	}

	~kdTree() {
		release();
		delete t_schedular;
	}

	bool build_process(std::vector<P>& input, const Box& box)
	{
		time_p t1 = hrclock::now();

		build(input, box);
		if (!root)
			return false;
		//----------------------------------------

		//add_neighbours();
		//add_kneighbours(input);

		//----------------------------------------
		calc_time = boost::chrono::duration_cast<milliseconds>(hrclock::now() - t1);

		return true;
	}

	void release() 
	{
		if (root) release_node(root);
		root = nullptr;
	}

	//------------------------------
	inline void set_thread_capacity(const unsigned int& tc);

	inline void get_cal_times(float& join, float& sort, float& all) const;
	
	kdnode_ptr get_root_ptr() const { return root; }

	//--------------------------------------------------------------

	// For a random point in the box
	inline void nearest_p(P_QUERY *query, kdnode_ptr currentNode, nearest_node& best);
	inline void knearest_p(P_QUERY *query, node* currentNode, kNearest_queue& best);

	//Precondition: The node->point need to be a pair, the second should be an info which contains index

		// closest uv from given points uvs
	void knearest_p_conditional(P_QUERY *query, node* currentNode, kNearest_queue& best, const std::set<unsigned int>& condition_index)
	{
		std::map<unsigned int, dist_uv> tmp;
		for (auto& it : condition_index)
			tmp.insert(std::pair<unsigned int, dist_uv>(it, dist_uv(FLT_MAX, nullptr)));

		knearest_p_conditional_f(query, currentNode, best, condition_index, tmp);

		for (auto& it : tmp)
			best.push(nearest_node(it.second.second, it.second.first));
	}

		// closest uv from uv set
	void nearest_p_conditional(P_QUERY *query, node* currentNode, kNearest_queue& best, const std::set<std::pair<float ,float>>& condition_uvs, std::vector<std::pair<float, float>>& res)
	{
		std::pair<std::vector<std::pair<float, float>>, dist_uv> tmp (std::vector<std::pair<float, float>>(), dist_uv(FLT_MAX, nullptr));

		nearest_p_conditional_f(query, currentNode, best, condition_uvs, tmp);

		res = tmp.first;
		best.push(nearest_node(tmp.second.second, tmp.second.first));
	}

	inline void all_nearest_in_range_from_one_obj(P_QUERY *query, node* currentNode, const float& max_dist, std::map<unsigned int, std::vector<std::pair<float, P_QUERY>>>& in_range, const unsigned int& gid);

	// add the neighbour info to all node
	void add_neighbours(const std::vector<P>& data)
	{
		if (data.size() > CLOUDRECURSIVEPOINTCAP && t_schedular->getCloudThreadCapacity() >= 2)
		{
			if (root->depth != stopBuildDepth) nearest_all(root, true, stopBuildDepth);
			else t_schedular->addTask(root);

			while (!t_schedular->isEmptyTask())
				t_schedular->addSubscribeShit(this, &kdTree::nearest_all);

			t_schedular->joinAll();		//wait our threads
		}
		else
			nearest_all(root);
	}

	void add_kneighbours(const std::vector<P>& data)
	{
		if (data.size() > CLOUDRECURSIVEPOINTCAP && t_schedular->getCloudThreadCapacity() >= 2)
		{
			if (root->depth != stopBuildDepth) knearest_all(root, true, stopBuildDepth);
			else t_schedular->addTask(root);

			while (!t_schedular->isEmptyTask())
				t_schedular->addSubscribeShit(this, &kdTree::knearest_all);

			t_schedular->joinAll();		//wait our threads
		}
		else
			knearest_all(root);
	}

	//------------------------------
	inline void in_order_print(node* r, const int& level);

	void set_kn(const unsigned int& k)	{	kn = k;	}

protected:
	//Tree buildings
	void build(std::vector<P>& data, const Box& box)
	{
		if (data.empty()) return;

		calc_time = milliseconds(0);
		sort_time = duration_f(0.0f);

		root = new node(NULL, &data[0], 0, data.size(), box, NULL, NULL, NULL);

		if (data.size() > CLOUDRECURSIVEPOINTCAP && t_schedular->getCloudThreadCapacity() >= 2)		//only do multithread on more points
			build_with_automech();
		else
			build_recursice(root);		//with few points the simple recursion is faster than syncing with main thread

		if (PRINT) in_order_print(root, 0);
	}

	inline void build_with_automech();
	inline void build_recursice(node* r, bool stop = false, const int& level = 0);

	//----------------
// Neighbours calculators
	// for nodes
	void nearest_all(node* query, bool stop = false, const int& stopDepth = 0)
	{
		nearest_recursive(query);

		if (stop && stopDepth == query->depth + 1)
		{
			if (query->left)   t_schedular->addTask(query->left);
			if (query->right)  t_schedular->addTask(query->right);
			return;
		}
		else
		{
			if (query->left) nearest_all(query->left);
			if (query->right) nearest_all(query->right);
		}
	}

	void knearest_all(node* query, bool stop = false, const int& stopDepth = 0)
	{
		knearest_recursive(query);

		if (stop && stopDepth == query->depth + 1)
		{
			if (query->left)   t_schedular->addTask(query->left);
			if (query->right)  t_schedular->addTask(query->right);
			return;
		}
		else
		{
			if (query->left) knearest_all(query->left);
			if (query->right) knearest_all(query->right);
		}
	}

	void nearest_recursive(node* query) {
		if (!root) {
			return;
		}
		nearest(query->point, root, query->nn);
	}

	void knearest_recursive(node* query) {
		if (!root) {
			return;
		}
		knearest(query->point, root, query->knn);
	}

// For a node in our tree
	inline void nearest(P *query, node* currentNode, nearest_node& best);

	inline void knearest(P *query, node* currentNode, kNearest_queue& best);

// Conditional for points from box
	inline void knearest_p_conditional_f(P_QUERY *query, node* currentNode, kNearest_queue& best, const std::set<unsigned int>& condtion_index, std::map<unsigned int, dist_uv>& act);
	inline void nearest_p_conditional_f(P_QUERY *query, node* currentNode, kNearest_queue& best, const std::set<std::pair<float, float>>& condition_uvs, std::pair<std::vector<std::pair<float, float>>, dist_uv>&);

	//----------------
	//Others
	inline void release_node(node*);

	inline void get_active_points(std::vector<P>& active_points, node* r = NULL);

private:
	kdnode_ptr root;

	unsigned int stopBuildDepth;
	TaskSchedular<kdTree, node>* t_schedular;
											
	int kn;			// k nearest
	int mesh_accuracy;		//for later usage, for filtering

	duration_f sort_time;
	milliseconds calc_time;
//----------
};

//--------------------------
//---		-----		----
//million point under 0,4 sec with 4 thread
template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM> ::build_with_automech()
{
	if (root->depth != stopBuildDepth) build_recursice(root, true, stopBuildDepth);
	else t_schedular->addTask(root);

	while (!t_schedular->isEmptyTask())		// inside the build_recursive it adds more task until it reach the limit of task threads
		t_schedular->addSubscribeWithoutAutomechanism(this, &kdTree::build_recursice);

	t_schedular->joinAll();		//wait our threads
}

template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::build_recursice(node* r, bool stop, const int& stopDepth)
{
	if (r->length == 1 )
	{
		r->point = r->start + (r->length >> 1);
		return;
	}

	int axis = r->depth % DIM;
	int med = r->length >> 1;
	//---------------------
	time_p t1 = hrclock::now();

	utils_for_kdtree::sort_part_vector_based_on_axis(axis, r->start, r->length);

	time_p t2 = hrclock::now();
	sort_time += (boost::chrono::duration_cast<duration_f>(t2 - t1));
	//---------------------

	r->point = r->start + med;

	//---------------------
	Box Lbox = r->box;
	Box Rbox = r->box;

	if (!axis)
	{
		Lbox.set_xmax(utils_for_kdtree::get_coord_based_on_axis(r->point->first, axis));
		Rbox.set_xmin(utils_for_kdtree::get_coord_based_on_axis(r->point->first, axis));
	}
	else if (axis == 1)
	{
		Lbox.set_ymax(utils_for_kdtree::get_coord_based_on_axis(r->point->first, axis));
		Rbox.set_ymin(utils_for_kdtree::get_coord_based_on_axis(r->point->first, axis));
	}
	else
	{
		Lbox.set_zmax(utils_for_kdtree::get_coord_based_on_axis(r->point->first, axis));
		Rbox.set_zmin(utils_for_kdtree::get_coord_based_on_axis(r->point->first, axis));
	}

	r->left = r->point - r->start == 0 ? NULL : new node(r, r->start, r->depth + 1, r->point - r->start, Lbox);
	r->right = r->length - r->left->length - 1 == 0 ? NULL : new node(r, r->point + 1, r->depth + 1, r->length - r->left->length - 1, Rbox);

	r->length = 1;

	if (stop && stopDepth == r->depth + 1)
	{
		if (r->left)  t_schedular->addTask(r->left);
		if (r->right) t_schedular->addTask(r->right);
		return;
	}
	else
	{
		if (r->left)  build_recursice(r->left, stop, stopDepth);
		if (r->right) build_recursice(r->right, stop, stopDepth);
	}
}

template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::release_node(node* r)
{
	node* _l = r->left;
	node* _r = r->right;
	delete r;

	if (_l) release_node(_l);
	if (_r) release_node(_r);
}

//---		-----		----
//--------------------------
//---		-----		----
// Neighbours
template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::nearest(P *query, node* currentNode, nearest_node& best) 
{
	if (!currentNode) return;

	int axis = currentNode->depth % DIM;

	float d = utils_for_kdtree::sum_of_square(currentNode->point->first, query->first);

	if (d == 0.0f)	//megtaláltuk a pontunkat, mindkét gyerek irányba menni kell mert a pont rajta van a vágósíkon
	{
		nearest(query, currentNode->left, best);
		nearest(query, currentNode->right, best);
		return;
	}

	//adott vágósíktól vett távolság
	float dx = utils_for_kdtree::distance_based_on_axis(currentNode->point->first, query->first, axis);

	if (d < best.distance) {
		best.neighb = currentNode->point;
		best.distance = d;
	}

	node* _near = dx <= 0 ? currentNode->right : currentNode->left;
	node* _far = dx <= 0 ? currentNode->left : currentNode->right;
	nearest(query, _near, best);
	if (dx * dx >= best.distance) return;	//pitagoras
	nearest(query, _far, best);
}

template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::nearest_p(P_QUERY *query, kdnode_ptr currentNode, nearest_node& best)
{
	if (best.distance == 0) return;
	if (!currentNode) return;

	int axis = currentNode->depth % DIM;

	float d = utils_for_kdtree::sum_of_square(currentNode->point->first, *query);

	//adott vágósíktól vett távolság
	float dx = utils_for_kdtree::distance_based_on_axis(currentNode->point->first, *query, axis);

	if (d < best.distance) {
		best.neighb = currentNode->point;
		best.distance = d;
	}

	node* _near = dx <= 0 ? currentNode->right : currentNode->left;
	node* _far = dx <= 0 ? currentNode->left : currentNode->right;
	nearest_p(query, _near, best);
	if (dx * dx >= best.distance) return;	//pitagoras
	nearest_p(query, _far, best);
}

template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::knearest(P *query, node* currentNode, kNearest_queue& best) {
	if (!currentNode) return;

	int axis = currentNode->depth % DIM;

	float d = utils_for_kdtree::sum_of_square(currentNode->point->first, query->first);

	if (d == 0.0f)	//megtaláltuk a pontunkat, mindkét gyerek irányba menni kell mert a pont rajta van a vágósíkon
	{
		knearest(query, currentNode->left, best);
		knearest(query, currentNode->right, best);
		return;
	}

	//adott vágósíktól vett távolság
	float dx = utils_for_kdtree::distance_based_on_axis(currentNode->point->first, query->first, axis);

	if (best.size() < kn || d <= best.top().distance) {
		best.push(nearest_node(currentNode->point, d));
		
		if (best.size() > kn) 
		{
			best.pop();
		}
	}

	node* _near = dx <= 0 ? currentNode->right : currentNode->left;
	node* _far = dx <= 0 ? currentNode->left : currentNode->right;
	knearest(query, _near, best);
	if (dx * dx >= best.top().distance) return;	//pitagoras
	knearest(query, _far, best);

}

template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::knearest_p(P_QUERY *query, node* currentNode, kNearest_queue& best) {
	if (!currentNode) return;

	int axis = currentNode->depth % DIM;

	float d = utils_for_kdtree::sum_of_square(currentNode->point->first, *query);

	//adott vágósíktól vett távolság
	float dx = utils_for_kdtree::distance_based_on_axis(currentNode->point->first, *query, axis);

	if (best.size() < kn || d <= best.top().distance) {
		best.push(nearest_node(currentNode->point, d));
		
		if (best.size() > kn) 
		{
			best.pop();
		}
	}

	node* _near = dx <= 0 ? currentNode->right : currentNode->left;
	node* _far = dx <= 0 ? currentNode->left : currentNode->right;
	knearest_p(query, _near, best);
	if (dx * dx >= best.top().distance) return;	//pitagoras
	knearest_p(query, _far, best);
}

template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::knearest_p_conditional_f(P_QUERY *query, node* currentNode, kNearest_queue& best, const std::set<unsigned int>& condtion_index, std::map<unsigned int, dist_uv>& act)
{
	if (!currentNode) return;

	unsigned int index = currentNode->point->second.index;

	if (condtion_index.find(index) == condtion_index.end())
	{
		node* _l = currentNode->left;
		node* _r = currentNode->right;
		knearest_p_conditional_f(query, _l, best, condtion_index, act);
		knearest_p_conditional_f(query, _r, best, condtion_index, act);
		return;
	}

	int axis = currentNode->depth % DIM;

	float d = utils_for_kdtree::sum_of_square(currentNode->point->first, *query);

	//adott vágósíktól vett távolság
	float dx = utils_for_kdtree::distance_based_on_axis(currentNode->point->first, *query, axis);

	if (d <= act[index].first) {
		act[index].first = d;
		act[index].second = currentNode->point;
	}

	node* _near = dx <= 0 ? currentNode->right : currentNode->left;
	node* _far = dx <= 0 ? currentNode->left : currentNode->right;
	knearest_p_conditional_f(query, _near, best, condtion_index, act);

	float max_v = FLT_MIN;	//calc the last's distance (the farthest)
	for (auto& it : act)
	{
		if (max_v < it.second.first)
			max_v = it.second.first;
	}

	if (dx * dx >= max_v) return;	//pitagoras
	knearest_p_conditional_f(query, _far, best, condtion_index, act);
}

template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::nearest_p_conditional_f(P_QUERY *query, node* currentNode, kNearest_queue& best, const std::set<std::pair<float, float>>& condition_uvs, std::pair<std::vector<std::pair<float, float>>, dist_uv>& act)
{
	if (!currentNode) return;

	int counter = 0;
	std::vector<std::pair<float, float>> tmp;
	for (auto& _uv : currentNode->point->second.uvs)
	{
		auto _uv_ = _uv._uv;
		auto _fuv = std::find_if(condition_uvs.begin(), condition_uvs.end(), [_uv_](const std::pair<float, float>& u) { return utils_for_kdtree::uv_length(u - _uv_) < UV_EPSILON; });
		if (_fuv == condition_uvs.end()) continue;

		counter++;
		tmp.push_back(_uv._uv);
	}

	int axis = currentNode->depth % DIM;

	float d = utils_for_kdtree::sum_of_square(currentNode->point->first, *query);

	//adott vágósíktól vett távolság
	float dx = utils_for_kdtree::distance_based_on_axis(currentNode->point->first, *query, axis);

	if (d <= act.second.first && counter >= condition_uvs.size()) {
		act.second.first = d;
		act.second.second = currentNode->point;
		act.first = tmp;
	}

	node* _near = dx <= 0 ? currentNode->right : currentNode->left;
	node* _far = dx <= 0 ? currentNode->left : currentNode->right;
	nearest_p_conditional_f(query, _near, best, condition_uvs, act);

	if (dx * dx >= act.second.first) return;	//pitagoras
	nearest_p_conditional_f(query, _far, best, condition_uvs, act);
}

template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::all_nearest_in_range_from_one_obj(P_QUERY *query, node* currentNode, const float& max_dist, std::map<unsigned int, std::vector<std::pair<float, P_QUERY>>>& in_range, const unsigned int& gid)
{
	if (!currentNode) return;

	int axis = currentNode->depth % DIM;

	float d = utils_for_kdtree::sum_of_square(currentNode->point->first, *query);
	//qDebug() << d << " " << currentNode->point->second.index;
	//adott vágósíktól vett távolság
	float dx = utils_for_kdtree::distance_based_on_axis(currentNode->point->first, *query, axis);

	if (currentNode->point->second.g_obj_id == gid && d <= max_dist )
	{
		in_range[currentNode->point->second.index].push_back(std::pair<float, P_QUERY>(d,currentNode->point->first));
	}

	node* _near = dx <= 0 ? currentNode->right : currentNode->left;
	node* _far = dx <= 0 ? currentNode->left : currentNode->right;
	all_nearest_in_range_from_one_obj(query, _near, max_dist, in_range, gid);

	//float n_dx = axis == 1 ? dx * dx : dx;

	if (dx * dx >= max_dist) return;	//pitagoras
	all_nearest_in_range_from_one_obj(query, _far, max_dist, in_range, gid);
}

//---		-----		----
//--------------------------
template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::get_active_points(std::vector<P>& active_points, node* r)
{
	if (!r) r = root;
	if (!r) return;

	active_points.emplace_back(*(r->point));

	if (r->left) get_active_points(active_points, r->left);
	if (r->right) get_active_points(active_points, r->right);
}

template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::set_thread_capacity(const unsigned int& tc)
{
	t_schedular->setCloudThreadCapacity(tc);
	stopBuildDepth = (unsigned int)log2(tc);
}

template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::get_cal_times(float& join, float& sort, float& all) const
{
	join = t_schedular->getJoinTime();
	sort = sort_time.count();
	all = calc_time.count();
}

template <typename P, typename P_QUERY, int DIM = 3>
inline void kdTree<P, P_QUERY, DIM>::in_order_print(node* r, const int& level)
{
	if (r == NULL) return;
	if (r->point == NULL)
	{
		printf("Empty Node\n");
		return;
	}

	in_order_print(r->left, level + 1);
	//printf("%zu : %d", level, *(r->point));
	in_order_print(r->right, level + 1);
}
