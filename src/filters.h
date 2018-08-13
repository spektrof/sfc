#pragma once
#include "base_types.h"	
#include <vector>

#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/hierarchy_simplify_point_set.h>
//#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include <CGAL/jet_smooth_point_set.h>

template< typename T, typename PointMap>
class Filter
{
public:
	Filter() {}
	virtual ~Filter() = 0;

	virtual void filter_process(std::vector<T>& points) = 0;
	virtual void set_first_property(const double&) {}
	virtual void set_second_property(const double&) {}
	virtual void set_first_property(const Vector&) {}
	virtual void set_second_property(const Vector&) {}
	virtual void set_third_property(const double&) {}

};
	
template< typename T, typename PointMap>
Filter<T, PointMap>::~Filter() {
}	//provided body for pure virtual destr

template< typename T, typename PointMap>
class Simplifier : public Filter<T, PointMap>
{
public:
	Simplifier() {}
	virtual ~Simplifier() = 0;
	virtual void filter_process(std::vector<T>& points) = 0;
	virtual void set_first_property(const double&) {}
	virtual void set_second_property(const double&) {}
	virtual void set_third_property(const double&) {}
	virtual void set_first_property(const Vector&) {}
	virtual void set_second_property(const Vector&) {}
	
};
template< typename T, typename PointMap>
Simplifier<T, PointMap>::~Simplifier() {
}	//provided body for pure virtual destr

template< typename T, typename PointMap>
class Smoother : public Filter<T, PointMap>
{
public:
	Smoother() {}
	virtual ~Smoother() = 0;
	virtual void filter_process(std::vector<T>& points) = 0;
	virtual void set_first_property(const double&) {}
	virtual void set_second_property(const double&) {}
};
template< typename T, typename PointMap>
Smoother<T, PointMap>::~Smoother() { }	//provided body for pure virtual destr

//----------------------------------
//----------	------	-----------

/******************************
*	Outlier removal componenet   *
******************************/

template< typename T, typename PointMap>
class OutlierComponentRemoval : public Simplifier<T, PointMap>		//remove the points from a side determined by plane
{
public:
	OutlierComponentRemoval(const Vector& _p = Vector(0, 0, 0), const Vector& _n = Vector(1, 0, 0)) : p(_p), n(_n) {}
	~OutlierComponentRemoval() {}

	void filter_process(std::vector<T>& points) override
	{
		float md = p.x()*n.x() + p.y()*n.y() + p.z()*n.z();		//calculate d in plane equation
		std::vector<size_t> del_pos;

		for (size_t it = 0; it < points.size(); ++it)
		{
			auto poi = points[it].first;
			float s_d = poi.x()*n.x() + poi.y()*n.y() + poi.z()*n.z() - md;
			if (s_d < 0) del_pos.push_back(it);
		}
	
		if (del_pos.size() == 0) return;

		std::vector<T>(points).swap(points);
	}

	void set_first_property(const Vector& first) override { p = first; }
	void set_second_property(const Vector& second)override { n = second; }

private:
	Vector p;
	Vector n;
};

template< typename T, typename PointMap>		//kn neighbours, outlier_limit, average spacing
class OutlierRemoval : public Simplifier<T, PointMap>
{
public:
	OutlierRemoval(const int& kn = 6, const double& o_lim = 2.0f, const double& avg_sp = 1.0f) : nb_neighbors(kn), outlier_limit(o_lim), average_spacing(avg_sp) {}
	~OutlierRemoval() {}

	void filter_process(std::vector<T>& points) override
	{
		//FIRST: I dont know the ratio
		std::vector<T>::iterator first_to_remove
			= CGAL::remove_outliers(points.begin(), points.end(),
				PointMap(),
				nb_neighbors,
				100.,                  // No limit on the number of outliers to remove
				outlier_limit * average_spacing); // Point with distance above 2*average_spacing are considered outliers

		points.erase(first_to_remove, points.end());
		
		// SECOND OPTION //
		// I know the ratio of outliers present in the point set

		/*const double removed_percentage = 5.0; // percentage of points to remove

		points.erase(CGAL::remove_outliers(points.begin(), points.end(),
			CGAL::Identity_property_map<Point>(),
			nb_neighbors,
			removed_percentage, // Minimum percentage to remove
			0.), // No distance threshold (can be omitted)
			points.end());
			*/

		std::vector<T>(points).swap(points);
	}

	void set_first_property(const double& first) override { nb_neighbors = first; }
	void set_second_property(const double& second) override { outlier_limit = second; }
	void set_third_property(const double& third) override { average_spacing = third; }

private:
	int nb_neighbors;
	double outlier_limit;
	double average_spacing;
};

//----------------------------------
//----------	------	-----------

/******************************
*	Simplifier componenet     *
******************************/

template <typename T, typename PointMap>		//cell size
class GridSimplification : public Simplifier<T, PointMap>
{
public:
	GridSimplification(const double& cs = 0.001f) :cell_size(cs) {}
	~GridSimplification() {}

	void filter_process(std::vector<T>& points)	override
	{
		points.erase(CGAL::grid_simplify_point_set(points.begin(), points.end(), PointMap(), cell_size),
			         points.end());
		
		std::vector<T>(points).swap(points);
	}

	void set_first_property(const double& first) { cell_size = first; }

private:
	double cell_size;
};

template <typename T, typename PointMap>	//max cluster size, max surface variation
class HierarchySimplification : public Simplifier<T, PointMap>		//quite good
{
public:
	HierarchySimplification(const double& mcs = 100.0f, const double& msv = 0.01f) : max_cluster_size(mcs), max_surface_variation(msv){}
	~HierarchySimplification() {}

	void filter_process(std::vector<T>& points) override
	{
		points.erase(CGAL::hierarchy_simplify_point_set(points.begin(), points.end(), PointMap(), 
			max_cluster_size, 
			max_surface_variation), 
			points.end());

		std::vector<T>(points).swap(points);
	}

	void set_first_property(const double& mcs) { max_cluster_size = mcs; }
	void set_second_property(const double& msv) { max_surface_variation = msv; }

private:
	double max_cluster_size;
	double max_surface_variation;
};

/*template <typename T, typename PointMap>	//2 double
class WLOPSimplification : public Simplifier<T, PointMap>			//wow
{
public:
	WLOPSimplification(const double& rp = 2.0f, const double& nr = 0.5f) : retain_percentage(rp), neighbor_radius(nr){}
	~WLOPSimplification() {}

	//we lost the uvs... -> only can use for reconstruction but not for texturing, mybe check it again
	void filter_process(std::vector<T>& points) override
	{
		std::vector<Point> output;

		CGAL::wlop_simplify_and_regularize_point_set
			<Concurrency_tag>
			(points.begin(),
				points.end(),
				std::back_inserter(output),
				PointMap(),
				retain_percentage,
				neighbor_radius,
				35U, false
				);

		//std::vector<T>(output).swap(points);
	}

	void set_first_property(const double& rp) { retain_percentage = rp; }
	void set_second_property(const double& nr) { neighbor_radius = nr; }

private:
	double retain_percentage;
	double neighbor_radius;
};
*/
//----------------------------------
//----------	------	-----------

/******************************
*	Smoother componenet      *
******************************/

#ifdef EIGEN_ENABLED
template <typename T, typename PointMap>		//neighbours
class JetSmoothing : public Smoother<T, PointMap>		//Need Eigen, and eigen flag
{
public:
	JetSmoothing(const double& kn = 24) : nb_neighbors(kn){}
	~JetSmoothing() {}

	void filter_process(std::vector<T>& points)
	{
		CGAL::jet_smooth_point_set<Concurrency_tag>(points.begin(), points.end(), PointMap(), nb_neighbors);
	}

	void set_first_property(const double& first) { nb_neighbors = first; }

private:
	double nb_neighbors; // default is 24 for real-life point sets
};
#endif