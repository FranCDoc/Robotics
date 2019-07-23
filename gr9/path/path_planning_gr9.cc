#include "path_planning_gr9.h"
#include "init_pos_gr9.h"
#include "odometry_gr9.h"
#include "speed_regulation_gr9.h"
#include "user_realtime.h"
#include "set_output.h"
#include "limit_angle_gr9.h"
#include "opp_pos_gr9.h"
#include "set_output.h"

NAMESPACE_INIT(ctrlGr9);

/*! \brief update the path-planning algorithm
 *
 * \param[in,out] cvs controller main structure
 */
void path_planning_update(CtrlStruct *cvs, double x_target, double y_target)
{


	//------------------- Part I :  FORCES  COMPUTATION ------------------------//

	cvs->path->F_att_x = force_saturator(Fatt(cvs->rob_pos->x, cvs->path->k_att, x_target), cvs->path->F_bound, -cvs->path->F_bound);
	cvs->path->F_att_y = force_saturator(Fatt(cvs->rob_pos->y, cvs->path->k_att, y_target), cvs->path->F_bound, -cvs->path->F_bound);
	cvs->path->F_rep_x = force_saturator(Frep(cvs, cvs->path->k_rep, cvs->path->rho_0, 1), cvs->path->F_bound, -cvs->path->F_bound);
	cvs->path->F_rep_y = force_saturator(Frep(cvs, cvs->path->k_rep, cvs->path->rho_0, 0), cvs->path->F_bound, -cvs->path->F_bound);

	cvs->path->F_x_tot = cvs->path->F_att_x + cvs->path->F_rep_x;
	cvs->path->F_y_tot = cvs->path->F_att_y + cvs->path->F_rep_y;


	//------------------- Part II :  DESIRED ANGULAR SPEEDS OF WHEELS COMPUTATION ------------------------//


	cvs->path->rho = sqrt(cvs->path->F_x_tot*cvs->path->F_x_tot + cvs->path->F_y_tot * cvs->path->F_y_tot); // Force_tot norm
	cvs->path->alpha_tot = atan2(cvs->path->F_y_tot, cvs->path->F_x_tot) - cvs->rob_pos->theta; // angle between the robot's x_axis and the force direction (positive counterclockwise)
	cvs->path->alpha = limit_angle(cvs->path->alpha_tot);
	cvs->path->beta = -cvs->rob_pos->theta - cvs->path->alpha; // angle between the force direction and x_frame axis (defined positive clockwise)


	// by multipliying k_rho by cos(alpha), you have a smaller linear speed if alpha increases, so you rotate more if alpha is big 
	cvs->path->y_dot = cvs->path->k_rho * cos(limit_alpha(cvs->path->alpha)) * cvs->path->rho; // multiplying by cos(alpha) such that cos(alpha) is always positive by limiting alpha [-pi/2:pi/2]
	cvs->path->theta_dot = cvs->path->k_alpha * cvs->path->alpha + cvs->path->k_beta * cvs->path->beta;


	//------------------- Part III :  COMMAND SENDING ------------------------//


	/// In the following, we stop the robot and make him rotate on itself to align the robot with the force direction (align theta rob to alpha)
	/// This is done only if alpha is too big
	if (cvs->path->alpha > M_PI / 8)
	{
		cvs->path_planning_l_speed = -5.0;
		cvs->path_planning_r_speed = 5.0;
	}
	else if (cvs->path->alpha < -M_PI / 8)
	{
		cvs->path_planning_l_speed = 5.0;
		cvs->path_planning_r_speed = -5.0;
	}
	else // the robot is more or less aligned with the force, sending of the desired wheel speeds
	{
		cvs->path_planning_l_speed = 1.5*(cvs->path->y_dot - cvs->path->theta_dot);
		cvs->path_planning_r_speed = 1.5*(cvs->path->y_dot + cvs->path->theta_dot);
	}

}



/////////////////////// Needed Functions ////////////////////////////////////////

double force_saturator(double force, double up_limit, double down_limit) {
	//  force saturator : bound the force such as down_limit<= force <= up_limit 

	if (force > up_limit) {
		return up_limit;
	}
	else if (force < down_limit) {
		return down_limit;
	}
	else {
		return force;
	}
}


double Fatt(double x, double k_att, double x_target)
{
	// Compute the attractive force along x or y direction

	double F = -k_att * (x - x_target);

	return F;
}


double min_dist(RobotPosition *rob_pos, double x2, double y2)
{
	// Compute the euclidian distance between a point (x2,y2) and the current robot position

	double x1 = rob_pos->x;
	double y1 = rob_pos->y;

	double dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
	return dist;
}

double limit_alpha(double alpha)
{
	// Limit alpha between -pi/2:pi/2 to have positive cosine
	double limit = M_PI / 2;
	if (alpha > limit)
	{
		alpha = alpha - M_PI;
	}
	else if (alpha <= -limit)
	{
		alpha = alpha + M_PI;
	}

	return alpha;
}

double Frep(CtrlStruct *cvs, double k_rep, double rho_0, int direction)
{
	// Compute the repulsive force along the x or y direction : if direction == 1, compute for x, y otherwise.
	// This function takes into account : 
	// 1) The walls
	// 2) The fixed rectangles
	// 3) The opponent 

	double Fx = 0.0;
	double Fy = 0.0;
	double x = cvs->rob_pos->x;
	double y = cvs->rob_pos->y;


	if (direction == 1)
	{
		//------------------- Part I :  WALLS (only the 2 walls that contribute to a x repulsive force are taken into account) ------------------------//

		if (fabs(cvs->rob_pos->x - 1.0) <= rho_0) // wall 1
		{
			double dist_min = fabs(cvs->rob_pos->x - 1.0);
			Fx = Fx + k_rep * (1.0 / dist_min - 1.0 / rho_0) * (1.0 / (dist_min*dist_min)) * (x - (1.0)) / dist_min;
		}
		else if (fabs(cvs->rob_pos->x - (-1.0)) <= rho_0) // wall 2 
		{
			double dist_min = fabs(cvs->rob_pos->x - (-1.0));
			Fx = Fx + k_rep * (1.0 / dist_min - 1.0 / rho_0) * (1.0 / (dist_min*dist_min)) * (x - (-1.0)) / dist_min;
		}

		//------------------- Part II :  Fixed rectangles ------------------------//

		/// Note : The rectangles informations (x,y,width,height) are stored in vector fixed_rect in serie : x_r1, y_r1, width_r1, height_r1, x_r2, y_r2,...

		double fixed_rect[16] = { 0.8, 0.0, 0.44, 2.10, 0.563, -0.75, 0.032, 0.6, 0.563, 0.75, 0.032, 0.6, 0.6, 0.0, 0.44, 0.04 };

		int j = 0;

		for (j; j < 16; j = j + 4)
		{
			double xcenter = fixed_rect[j];
			double ycenter = fixed_rect[j + 1];
			double width = fixed_rect[j + 2];
			double height = fixed_rect[j + 3];

			// First, we need to find the point on the rectangle boundary that is the closest to the robot . Then compute the distance between them in min_dist_rectgl.

			double min_dist_rectgl;
			double point_x;
			double point_y;

			if (x < (xcenter - width / 2.0))
			{
				point_x = xcenter - width / 2.0;
			}
			else if (x > (xcenter + width / 2.0))
			{
				point_x = xcenter + width / 2.0;
			}
			else
			{
				point_x = x;
			}
			if (y < ycenter - height / 2.0)
			{
				point_y = ycenter - height / 2.0;
			}
			else if (y > ycenter + height / 2.0)
			{
				point_y = ycenter + height / 2.0;
			}
			else
			{
				point_y = y;
			}
			min_dist_rectgl = min_dist(cvs->rob_pos, point_x, point_y);

			// We now can add the contribution of the rectangle to the repulsive force iif the distance computed is smaller than rho_0

			if (min_dist_rectgl <= rho_0)
			{
				Fx = Fx + k_rep * (1.0 / min_dist_rectgl - 1.0 / rho_0) * (1.0 / (min_dist_rectgl*min_dist_rectgl)) * (x - point_x) / min_dist_rectgl;
			}
			else
			{
				// nothing to do 
			}
		}


		//------------------- Part III :  Opponent ------------------------//

		double opponent_x = cvs->opp_pos->x;
		double opponent_y = cvs->opp_pos->y;
		double security_dist = 0.015;
		double opp_dist = min_dist(cvs->rob_pos, opponent_x, opponent_y);
		double opp_dist_margin = opp_dist - security_dist;
		if (opp_dist_margin < 0.0)
		{
			// it means the robots are in contact
			opp_dist_margin = 0.0;
		}
		else
		{
			// nothing to do
		}
		if (opp_dist_margin <= cvs->path->rho_0_opp)
		{
			Fx = Fx + k_rep * (1.0 / opp_dist_margin - 1.0 / cvs->path->rho_0_opp) * (1.0 / (opp_dist_margin*opp_dist_margin)) * (x - opponent_x) / opp_dist_margin;
		}


		return Fx;
	}
	else
	{
		//------------------- Part I :  WALLS (only the 2 walls that contribute to a y repulsive force are taken into account) ------------------------//


		if (fabs(cvs->rob_pos->y - 1.5) <= rho_0) // wall 1
		{
			double dist_min = fabs(cvs->rob_pos->y - 1.5);
			Fy = Fy + k_rep * (1.0 / dist_min - 1.0 / rho_0) * (1.0 / (dist_min*dist_min)) * (y - (1.5)) / dist_min;
		}
		else if (fabs(cvs->rob_pos->y - (-1.5)) <= rho_0) // wall 2 
		{
			double dist_min = fabs(cvs->rob_pos->y - (-1.5));
			Fy = Fy + k_rep * (1.0 / dist_min - 1.0 / rho_0) * (1.0 / (dist_min*dist_min)) * (y - (-1.5)) / dist_min;
		}


		//------------------- Part II :  Fixed rectangles ------------------------//

		/// Note : The rectangles informations (x,y,width,height) are stored in vector fixed_rect in serie : x_r1, y_r1, width_r1, height_r1, x_r2, y_r2,...

		double fixed_rect[16] = { 0.8, 0.0, 0.44, 2.10, 0.563, -0.75, 0.032, 0.6, 0.563, 0.75, 0.032, 0.6, 0.6, 0.0, 0.44, 0.04 };

		int j = 0;

		for (j; j < 16; j = j + 4)
		{
			double xcenter = fixed_rect[j];
			double ycenter = fixed_rect[j + 1];
			double width = fixed_rect[j + 2];
			double height = fixed_rect[j + 3];

			// First, we need to find the point on the rectangle boundary that is the closest to the robot . Then compute the distance between them in min_dist_rectgl.

			double min_dist_rectgl;
			double point_x;
			double point_y;

			if (y < ycenter - height / 2.0)
			{
				point_y = ycenter - height / 2.0;
			}
			else if (y > ycenter + height / 2.0)
			{
				point_y = ycenter + height / 2.0;
			}
			else
			{
				point_y = y;
			}
			if (x < (xcenter - width / 2.0))
			{
				point_x = xcenter - width / 2.0;
			}
			else if (x > (xcenter + width / 2.0))
			{
				point_x = xcenter + width / 2.0;
			}
			else
			{
				point_x = x;
			}
			min_dist_rectgl = min_dist(cvs->rob_pos, point_x, point_y);

			// We now can add the contribution of the rectangle to the repulsive force iif the distance computed is smaller than rho_0

			if (min_dist_rectgl <= rho_0)
			{
				Fy = Fy + k_rep * (1.0 / min_dist_rectgl - 1.0 / rho_0) * (1.0 / (min_dist_rectgl*min_dist_rectgl)) * (y - point_y) / min_dist_rectgl;
			}
			else
			{
				//nothing to do
			}

		}

		//------------------- Part III :  Opponent ------------------------//

		double opponent_x = cvs->opp_pos->x;
		double opponent_y = cvs->opp_pos->y;
		double security_dist = 0.015;
		double opp_dist = min_dist(cvs->rob_pos, opponent_x, opponent_y);
		double opp_dist_margin = opp_dist - security_dist;
		if (opp_dist_margin < 0.0)
		{
			// it means the robots are in contact
			opp_dist_margin = 0.0;
		}
		else
		{
			// nothing to do
		}
		if (opp_dist_margin <= cvs->path->rho_0_opp)
		{
			Fy = Fy + k_rep * (1.0 / opp_dist_margin - 1.0 / cvs->path->rho_0_opp) * (1.0 / (opp_dist_margin*opp_dist_margin)) * (y - opponent_y) / opp_dist_margin;
		}


		return Fy;

	}
}


NAMESPACE_CLOSE();
