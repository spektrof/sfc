#include "polar_ball.h"
#include <set>

float PolarBall::get_alpha_weight(PolarBall* rhs)
{
	float R = this->radius;
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

float PolarBall::get_beta_weight(PolarBall* rhs, Point& surfpoint)
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
