#pragma once
#include <vector>
#include <algorithm>

#define max_prior(a,b) (((a)>(b))?(a):(b))

template <typename obj_type, typename prio_type = float>
class priority_queue
{
	typedef std::pair<prio_type, obj_type*> tuple;

public:
	priority_queue() { elements.clear(); }

	void push(const float& priority, obj_type* obj)
	{
		int pos = (int)elements.size();
		obj->set_heap_index(pos);
		elements.push_back(tuple(priority, obj));

		move_up(pos);
	}

	obj_type* pop()
	{
		if (elements.empty()) return nullptr;

		obj_type* best_priored_elem = elements[0].second;
	
		tuple last = elements.back();

		elements.pop_back();

		if (elements.empty()) return best_priored_elem;

		int pos = 0;
		elements[0] = last;
		move_down(pos);

		return best_priored_elem;
	}

	void update(const int& pos)
	{
		int update_pos = pos;
		auto update_element = &elements[update_pos];

		float in_val = update_element->second->get_in_value();
		float out_val = update_element->second->get_out_value();

		float new_prio;
		if (in_val > 0 && out_val > 0)
			new_prio = abs(in_val - out_val) - 1.0f;
		else
			new_prio = max_prior(in_val, out_val);

		update_element->first = new_prio;

		unsigned int parent = get_parent(update_pos);

		if (update_pos == 0 || elements[parent].first > new_prio)
			move_down(update_pos);
		else
			move_up(update_pos);
	}

	prio_type highest_prior() const
	{
		if (empty()) return -2.0f;
		prio_type pri = -2.0f;

		for (auto& it : elements)
			if (pri < it.first) pri = it.first;

		return pri;
	}

	prio_type top_prior() const
	{
		return elements[0].first;
	}

	bool empty() const
	{
		return elements.empty();
	}

	size_t size() const { return elements.size(); }

	typename std::vector<tuple>::iterator find(obj_type* obj)	// O(N) ( as we find in unordered pointer list ), use heap_index in your datatype to get it constant time
	{
		auto& res = std::find_if(elements.begin(), elements.end(), [obj](const tuple& t)->bool { return t.second == obj; });
		return res;
	}

	void print(std::ostream& out)
	{
		for (auto& it : elements)
			out << "\t" << it.first << " " << it.second->a << " index " << it.second->heap_index << "\n";
	}

	int count(obj_type* obj)
	{
		int c = 0;
		for (auto& it : elements)
		{
			if (it.second == obj)
				c++;
		}
		return c;
	}

protected:
	void swap(const int& p1, const int& p2)
	{
		auto p1_ptr = &elements[p1];		//to prevent multiple []
		auto p2_ptr = &elements[p2];

		p1_ptr->second->set_heap_index(p2);
		p2_ptr->second->set_heap_index(p1);

		tuple tmp = *p1_ptr;
		*p1_ptr = *p2_ptr;
		*p2_ptr = tmp;
	}

	unsigned int get_parent(const int& p)
	{
		if (p == 0) return 0;

		return (p - 1) >> 1;
	}

	unsigned int get_left_child(const int& p)
	{
		int _p = (p + 1) << 1;
		_p--;
		return _p;
	}

	unsigned int get_right_child(const int& p)
	{
		int _p = (p + 1) << 1;
		return _p;
	}

	void move_up(int& pos)
	{
		if (pos == 0) return;

		int parent = get_parent(pos);
		while (pos != 0 && elements[parent].first < elements[pos].first)
		{
			swap(parent, pos);
			pos = parent;
			parent = get_parent(pos);
		}
	}

	void move_down(int& pos)
	{
		auto act_priority = elements[pos].first;

		unsigned int e_size = (int)elements.size();
		unsigned int left_child = get_left_child(pos);
		unsigned int right_child = get_right_child(pos);

		unsigned int highest_pos = 0;
		auto highest_pri = act_priority;

		if (e_size > left_child && elements[left_child].first > highest_pri)
		{
			highest_pos = 1;
			highest_pri = elements[left_child].first;
		}

		if (e_size > right_child && elements[right_child].first > highest_pri)
		{
			highest_pos = 2;
			//	highest_pri = elements[right_child].first;	//unnecessary

			swap(right_child, pos);
			pos = right_child;
			move_down(pos);
			return;
		}

		if (highest_pos == 0) return;

		//pos is 1 here
		swap(left_child, pos);
		pos = left_child;
		move_down(pos);
	}

private:

	std::vector<tuple> elements;
};