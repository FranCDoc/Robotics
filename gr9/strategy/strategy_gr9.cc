#include "strategy_gr9.h"
#include "path_planning_gr9.h"
#include "speed_regulation_gr9.h"
#include "odometry_gr9.h"
#include "init_pos_gr9.h"
#include "calibration_gr9.h"

NAMESPACE_INIT(ctrlGr9);


double target_pos[7][3] = { {-0.8, -0.8, 1.0}, {-0.8,  0.8, 1.0}, {0.4, -1.3, 1.0}, {0.4, 1.3, 1.0}, {0.45, -0.15, 2.0}, {0.45, 0.15, 2.0}, {-0.2, 0.0, 3.0} };

/*! \brief startegy during the game
 *
 * \param[in,out] cvs controller main structure
 */
void main_strategy(CtrlStruct *cvs)
{

	// ------------------------------- Initialization of variables --------------------------- //

	Strategy *strat;
	strat = cvs->strat;

	if (cvs->team_id == TEAM_A) {
		cvs->x_out = 0.3;
		cvs->y_out = 1.25; // to avoid the fixed rectangle delimiting the base entry
		cvs->x_base = 0.8;
		cvs->y_base = 1.25;
	}
	else
	{
		cvs->x_out = 0.3;
		cvs->y_out = -1.25; // to avoid the fixed rectangle delimiting the base entry
		cvs->x_base = 0.8;
		cvs->y_base = -1.25;
	}

	// ------------------------------------ Strategy --------------------------------------- //

	switch (strat->state)
	{
	case Target_Drive: 
		

		// -------------------- Part I : Target selection and driving towards ------------------------------ // 


		strat->index = target_choice_maker(target_pos, cvs); // the index of the target to aim in the target_pos vector
		strat->x_target = target_pos[strat->index][0];
		strat->y_target = target_pos[strat->index][1];

		if (strat->x_target == 100.0)
		{
			// There is no more target to grab : 
			strat->state = No_more_Target;
		}
		else
		{
			if (fabs(strat->y_target) == 0.15)
			{
				cvs->path->rho_0 = 0.15;
			}
			else if (strat->x_target == -0.8)
			{
				cvs->path->rho_0 = 0.17;
			}
			else
			{
				cvs->path->rho_0 = 0.22;
			}
		}

		path_planning_update(cvs, strat->x_target, strat->y_target); // update the path-planning 
		speed_regulation(cvs, cvs->path_planning_r_speed, cvs->path_planning_l_speed); // send commands to the wheels


		// -------------------- Part II : Arrival at the target location ------------------------------ // 


			// The robot is at the target position, but does not detect one : it must have been taken by the opponent
			// ==> update the previous target information such that target_choice_maker will not chose that target again

			if (cvs->inputs->target_detected > 0)
			{
				// The robot does detect the target that lays under, go to strat 2
				strat->t_state = cvs->inputs->t;
				strat->state = Target_Grab;
			}
			else if ((strat->x_target - 0.02 < cvs->rob_pos->x) && (cvs->rob_pos->x < strat->x_target + 0.02) && (strat->y_target - 0.02 < cvs->rob_pos->y) && (cvs->rob_pos->y < strat->y_target + 0.02) && cvs->inputs->target_detected == 0)
			{
				target_pos[strat->index][0] = 100.0;
				target_pos[strat->index][1] = 100.0;
				target_pos[strat->index][2] = -1.0;
			}
		//}

		break;


	case Target_Grab: 

		// -------------------- Part I : Grabing the target ------------------------------ // 


		if (cvs->inputs->target_detected == 0) // if the robot was on the target but, due to inertia, is no longer on it
		{
			speed_regulation(cvs, 0.0, 0.0); // to reduce inertia
			strat->state = Target_Drive; // to readjust the position with the target
		}
		else
		{
			// Grab the target 
			speed_regulation(cvs, 0.0, 0.0);
			if (cvs->inputs->t >= (strat->t_state+1.5) )
			{
				strat->counter  = strat->counter + 1; //count the number of target the robot has on itself
				target_pos[strat->index][0] = 100.0;
				target_pos[strat->index][1] = 100.0;
				target_pos[strat->index][2] = -1.0;

				// target has now been grabbed, can go to strat 3
				strat->state = Base_or_Next;
			}
		}

		break;

	case Base_or_Next:

		cvs->path->rho_0 = 0.22;


		// -------------------- Part I : Checking the number of target the robot holds ------------------------------ //

		if (strat->counter < 2)
		{

			// Since the robot can hold up to 2 targets simultaneously, let's find another one 

			strat->state = Target_Drive;
		}
		else
		{
			// -------------------- Part II : Go base (first aime for to way out point)  ------------------------------ //
			
			path_planning_update(cvs, cvs->x_out, cvs->y_out);
			speed_regulation(cvs, cvs->path_planning_r_speed, cvs->path_planning_l_speed); // send commands to the wheels
			
			if ( (cvs->x_out - 0.05 < cvs->rob_pos->x) && (cvs->rob_pos->x  < cvs->x_out+0.05) && ( cvs->y_out-0.05 < cvs->rob_pos->y ) && ( cvs->rob_pos->y < cvs->y_out+0.05 ) )
			{

				// The robot is on the way out point, go in base
				strat->state = Go_in_Base;
			}
		}
		break;

	case Go_in_Base:

		cvs->path->rho_0 = 0.17;

		path_planning_update(cvs, cvs->x_base, cvs->y_base);
		speed_regulation(cvs, cvs->path_planning_r_speed, cvs->path_planning_l_speed); // send commands to the wheels

		if ((cvs->x_base*(1 - strat->tol) < cvs->rob_pos->x) && (cvs->rob_pos->x < cvs->x_base*(1 + strat->tol)) && (fabs(cvs->y_base)*(1 - strat->tol) < fabs(cvs->rob_pos->y)) && (fabs(cvs->rob_pos->y) < fabs(cvs->y_base)*(1 + strat->tol)))
		{
			// The robot is in the base, release of the targets 
			cvs->outputs->flag_release = 1;
			strat->t_out = cvs->inputs->t;
			strat->state = Go_out_Base;
		}
		break;


	case Go_out_Base:

		cvs->path->rho_0 = 0.2;

		// -------------------- Goal : Get out of the base and update the flag_release state ------------------------------ // 
		// -----------------------------------Then : Go after a new target ------------------------------------------------ // 
		

		if ( (cvs->x_out - 0.05 < cvs->rob_pos->x) && (cvs->rob_pos->x < cvs->x_out + 0.05) && (cvs->y_out - 0.05 < cvs->rob_pos->y) && (cvs->rob_pos->y < cvs->y_out + 0.05) )
		{
			// Robot is out, go after new target
			cvs->outputs->flag_release = 0;
			strat->state = Target_Drive;
			strat->counter = 0;
		}
		else
		{
			// Getting out
			path_planning_update(cvs, cvs->x_out, cvs->y_out); //to go out of base easy
			speed_regulation(cvs, cvs->path_planning_r_speed, cvs->path_planning_l_speed);
		}
		break;


	case No_more_Target:

		path_planning_update(cvs,cvs->calib->X_calib, cvs->calib->Y_calib);
		speed_regulation(cvs, cvs->path_planning_r_speed, cvs->path_planning_l_speed); // send commands to the wheels

		if ((cvs->calib->X_calib*(1 - strat->tol) < cvs->rob_pos->x) && (cvs->rob_pos->x < cvs->calib->X_calib*(1 + strat->tol)) && (fabs(cvs->calib->Y_calib)*(1 - strat->tol) < fabs(cvs->rob_pos->y)) && (fabs(cvs->rob_pos->y) < fabs(cvs->calib->Y_calib)*(1 + strat->tol)))
		{
			// The robot is back on its starting position : stop it
			speed_regulation(cvs, 0.0, 0.0);
		}

		break;


	default:
		printf("Strategy error: unknown state: %d !\n", strat->state);
		exit(EXIT_FAILURE);
	}
}



///////////////////////// Needed Functions ///////////////////////////////////


int target_choice_maker(double target_vect[7][3], CtrlStruct *cvs)
{
	// Return the index of the target in target_vect that is the closest to the robot. 
	// If two targets are at the same distance, return the index of the one giving more points

	int j = 0;
	int size = 7;

	int index = 0;
	double dist1 = 100.0;
	for (j; j < size; j++)
	{
		double x_rob = cvs->rob_pos->x;
		double y_rob = cvs->rob_pos->y;
		double x2 = target_vect[j][0];
		double y2 = target_vect[j][1];

		double dist_to_compare = sqrt(pow((x2 - x_rob), 2) + pow((y2 - y_rob), 2));
		if (dist_to_compare < dist1)
		{
			dist1 = dist_to_compare;
			index = j;
		}
		else if (dist_to_compare == dist1)
		{ // then check if the value of the one to compare is greater, if it is then assign its index
			if (target_vect[j][2] > target_vect[index][2])
			{
				index = j;
			}
			else
			{
				//
			}
		}
		else //dist to compare is not closer
		{
			//
		}
	}

	return index;
}



NAMESPACE_CLOSE();