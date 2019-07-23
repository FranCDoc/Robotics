#include "calibration_gr9.h"
#include "speed_regulation_gr9.h"
#include "odometry_gr9.h"
#include "path_planning_gr9.h"
#include "init_pos_gr9.h"
#include "math.h"
#include "triangulation_gr9.h"
#include "kalman_gr9.h"

NAMESPACE_INIT(ctrlGr9);

// calibration states
enum {CALIB_START, CALIB_STATE_1, CALIB_STATE_2, CALIB_STATE_3, CALIB_STATE_4, CALIB_STATE_5, CALIB_STATE_6, CALIB_FINISH};

// to put in structure
double init_pos[6][2] = { {0.05, -1.25}, {-0.25,-1.25}, {-0.55, -1.25}, {0.05, 1.25}, {-0.25,1.25}, {-0.55, 1.25} };

// double counter_y = 0.0;
// double counter_x = 0.0;
// double angle_rotation = 0.0;

/*! \brief calibration of the robot to get its actual position
 * 
 * \param[in,out] cvs controller main structure
 */
void calibration(CtrlStruct *cvs)
{
	// variables declaration
	double t;

	CtrlIn *inputs; ///< controller inputs
	RobotCalibration *calib; ///< calibration structure
	RobotPosition *rob_pos;  ///< robot position (to calibrate)

	// variables initialization
	inputs  = cvs->inputs;
	calib   = cvs->calib;
	rob_pos = cvs->rob_pos;

	double wr = 10.0; // right wheel speed when going forward or backward
	double wl = 10.0; // left wheel speed when going forward or backward
	double wRotr = 3.0; // right wheel speed when rotating
	double wRotl = -3.0; // left wheel speed when rotating

	t = inputs->t;
	double dt = cvs->dt;

	bool leftSwitch = cvs->inputs->u_switch[L_ID];
	bool rightSwitch = cvs->inputs->u_switch[R_ID];

	//------------------ calibration -----------------//
	switch (calib->flag)
	{
		case CALIB_START: // start calibration
			speed_regulation(cvs, 0.0, 0.0); // not moving

			calib->flag = CALIB_STATE_1; // directly go to state CALIB_STATE_1
			calib->t_flag = t; // save current time
			break;

		case CALIB_STATE_1: // first calibration state :going bacward to calibrate y
			
			if ( (!leftSwitch) && (!rightSwitch) ) // don't touch the wall, so have to continue
			{
				speed_regulation(cvs, -wr, -wl);
			}
			else if ( (leftSwitch) && (!rightSwitch) ) // touch the wall with left switch, so rotate to touch the wall with right one
			{
				speed_regulation(cvs, -wr, 0.0);
			}
			else if ( (!leftSwitch) && (rightSwitch) ) // touch the wall with right switch, so rotate to touch the wall with left one
			{
				speed_regulation(cvs, 0.0, -wl);
			}
			if ( (leftSwitch) && (rightSwitch) ) // touch the wall with both switches
			{
				// ------------- calibration of y depend on sign of y, so on the team -------------//
				if (cvs->team_id == TEAM_A)
				{
					calib->Y_calib = 1.5 - cvs->rob_pos->y; // 1.5 - delta(t)*w*R
					calib->flag = CALIB_STATE_2;
					calib->t_flag = t;
				}
				else
				{
					calib->Y_calib =  - 1.5 + cvs->rob_pos->y; // 1.5 - delta(t)*w*R
					calib->flag = CALIB_STATE_2;
					calib->t_flag = t;
				}
				
			}
			break;

		//-------------- going back to initial position ------------------//
		case CALIB_STATE_2: 
			if (cvs->rob_pos->y > 0.1) // go to initial position
			{
				speed_regulation(cvs, wr, wl);
			}
			else //stop at y<=0.1 because after it will continue a bit with inertia
			{
				calib->flag = CALIB_STATE_3;
				calib->t_flag = t;
			}
			break;

		//---------------- second calibration state : rotation to calibrate in x -------------------//
		case CALIB_STATE_3:
			
			if (cvs->team_id == TEAM_A) // the direction of rotation depend on the team such that it is a rotation of -90° that is performed
			{
				if ((cvs->rob_pos->theta) < -0.2) // not yet a rotation of -90°
				{
					speed_regulation(cvs, wRotr, wRotl);
				}
				else // rotation is enough
				{
					calib->flag = CALIB_STATE_4;
					calib->t_flag = t;
					calib->calib_step = cvs->rob_pos->x; // calib step stock the initial position after the first rotation
				}
								}
			else
			{
				if ((cvs->rob_pos->theta) > -M_PI+0.2) // not yet a rotation of -90°
				{
					speed_regulation(cvs, -wRotr, -wRotl);
				}
				else // rotation is enough
				{
					calib->flag = CALIB_STATE_4;
					calib->t_flag = t;
					calib->calib_step = cvs->rob_pos->x; // calib step stock the initial position after the first rotation
				}
			}
			break;

		case CALIB_STATE_4: // second calibration state : going backward to calibrate x

			if ( (!leftSwitch) && (!rightSwitch) ) // don't touch the wall
			{
				speed_regulation(cvs, -wr, -wl);
			}
			else if ( (leftSwitch) && (!rightSwitch) ) // touch the wall with left switch
			{
				speed_regulation(cvs, -wr, 0.0);
			}
			else if ( (!leftSwitch) && (rightSwitch) ) // touch the wall with right switch
			{
				speed_regulation(cvs, 0.0, -wl);
			}
			else if ( (leftSwitch) && (rightSwitch) ) // touch the wall with both switches
			{
				if (fabs(cvs->rob_pos->x) < 1) // depending on the sign of the travelled distance, the calibration equation for x change
				{
					calib->X_calib = -(1.0 - fabs(cvs->rob_pos->x));


					calib->flag = CALIB_STATE_5;
					calib->t_flag = t;
				}
				else
				{
					calib->X_calib = fabs(cvs->rob_pos->x) - 1.0 ;

					calib->flag = CALIB_STATE_5;
					calib->t_flag = t;
				}


			}
			break;

		//-------------------- go back to initial position in x ------------------//
		case CALIB_STATE_5:
			if (cvs->team_id == TEAM_A) { // the moment to stop depend on the team of the robot
				if (cvs->rob_pos->x < calib->calib_step - 0.05) // stop 0.05m before becaus einertia will continue the motion
				{
					speed_regulation(cvs, wr, wl);
				}
				else
				{
					calib->flag = CALIB_STATE_6;
					calib->t_flag = t;
				}
			}
			else {
				if (cvs->rob_pos->x > calib->calib_step + 0.05)
				{
					speed_regulation(cvs, wr, wl);
				}
				else
				{
					calib->flag = CALIB_STATE_6;
					calib->t_flag = t;
				}
			}

			break;

		//------------------------- rotate to reach the initial orientation -----------------------//
		case CALIB_STATE_6: 
			if((cvs->team_id == TEAM_A) && (cvs->rob_pos->theta >= -M_PI/2) ) // initial position not yet reached
			{
				speed_regulation(cvs, -wRotr, -wRotl);
			}
			else if ((cvs->team_id == TEAM_B) && (fabs(cvs->rob_pos->theta) >= M_PI/2)) // initial position not yet reached
			{
				speed_regulation(cvs, wRotr, wRotl);
			}
			else // initial position is reached
			{
				calib->flag = CALIB_FINISH;
				cvs->main_state = WAIT_INIT_STATE;
				
				// choose the closest possible starting position from the starting position obtained by calibration
				int init_index = init_choice_maker(init_pos, cvs->calib->X_calib, cvs->calib->Y_calib); 
				cvs->calib->x_base_update = init_pos[init_index][0];
				cvs->calib->y_base_update = init_pos[init_index][1];

				// initiate the variables for odometry, triangularisation, Kalman and rob_pos
				cvs->rob_pos->x = cvs->calib->x_base_update;
				cvs->rob_pos->y = cvs->calib->y_base_update;
				cvs->myTriangulation->x = cvs->calib->x_base_update;
				cvs->myTriangulation->y = cvs->calib->y_base_update;
				cvs->myOdometry->x = cvs->calib->x_base_update;
				cvs->myOdometry->y = cvs->calib->y_base_update;
				cvs->kalman->x = cvs->calib->x_base_update;
				cvs->kalman->y = cvs->calib->y_base_update;

				// initite the angle depending on the team
				if (cvs->team_id == TEAM_A)
				{
					cvs->myOdometry->theta = -M_PI / 2;
					cvs->rob_pos->theta = -M_PI / 2;
					cvs->myTriangulation->theta = -M_PI / 2;
				}
				else
				{
					cvs->myOdometry->theta = M_PI / 2;
					cvs->rob_pos->theta = M_PI / 2;
					cvs->myTriangulation->theta = M_PI / 2;
				}
			}
			break;


		case CALIB_FINISH: // wait before the match is starting
			speed_regulation(cvs, 0.0, 0.0);
			break;

		default:
			printf("Error: unknown calibration state : %d !\n", calib->flag);
			exit(EXIT_FAILURE);
	}
}



////////////////////////////////////////////////// Needed Functions //////////////////////////////////////////



int init_choice_maker(double target_vect[6][2], double x, double y)
{
	// return the index of the target in target_vect that is the closest to the robot position (x,y).

	int j = 0;
	int size = 6;

	int index = 0;
	double dist1 = 100.0;
	for (j; j < size; j++)
	{
		double x_rob = x;
		double y_rob = y;
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
