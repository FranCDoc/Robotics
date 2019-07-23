/*!
 * \file ctrl_main_gr9.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "ctrl_main_gr9.h"
#include "namespace_ctrl.h"
#include "init_pos_gr9.h"
#include "odometry_gr9.h"
#include "opp_pos_gr9.h"
#include "triangulation_gr9.h"
#include "speed_regulation_gr9.h"
#include "calibration_gr9.h"
#include "strategy_gr9.h"
#include "simu_game_gr9.h"
#include "set_output.h"
#include "calibration_gr9.h"
#include "path_planning_gr9.h"
#include "kalman_gr9.h"
#include "kalman_gr9.h"
#include "matrix_opperations_gr9.h"


NAMESPACE_INIT(ctrlGr9);

/*! \brief initialize controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_init(CtrlStruct *cvs)
{
	// variables declaration
	double t;
	CtrlIn *inputs;

	// variables initialization
	inputs = cvs->inputs;
	t = inputs->t;

	// robot ID
#ifdef SIMU_GAME // simulation
	cvs->robot_id = inputs->robot_id;
#else // real robot (hardcoded)
	cvs->robot_id = ROBOT_B;
#endif

	// robot team
	switch (inputs->robot_id)
	{
	case ROBOT_B: cvs->team_id = TEAM_A; break;
	case ROBOT_R: cvs->team_id = TEAM_A; break;
	case ROBOT_Y: cvs->team_id = TEAM_B; break;
	case ROBOT_W: cvs->team_id = TEAM_B; break;

	default:
		printf("Team detection error: unknown robot ID: %d !\n", inputs->robot_id);
		exit(EXIT_FAILURE);
	}

	// robot position
	set_init_position(cvs->robot_id, cvs->rob_pos);
	cvs->rob_pos->last_t = t;

	// speed regulation
	cvs->sp_reg->last_t = t;
}

/*! \brief controller loop (called every time-step)
 *
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs)
{
	// variables declaration
	double t;
	CtrlIn  *inputs;
	CtrlOut *outputs;

	// variables initialization
	inputs = cvs->inputs;
	outputs = cvs->outputs;

	t = inputs->t;


	// ------------------------------------------------------- Update Odometry ---------------------------------------------- //


	update_odometry(cvs);


	// ------------------------------------------------------- Update Triangulation ---------------------------------------------- //


	if (fabs(cvs->inputs->r_wheel_speed) < 0.5 && fabs(cvs->inputs->l_wheel_speed) < 0.5) // triangulation only work when robot don't moove too much
	{
		update_triangulation(cvs);
	}
	else // this case, triangulation is not used. Values are set such that Kalman will reject them
	{
		cvs->myTriangulation->x = 100.0;
		cvs->myTriangulation->y = 100.0;
		cvs->myTriangulation->theta = 10*M_PI;
	}
	kalman_filter(cvs);


	// ------------------------------------------------------- Update Robot Position ---------------------------------------------- //


	cvs->rob_pos->x = cvs->kalman->x;
	cvs->rob_pos->y = cvs->kalman->y;
	cvs->rob_pos->theta = cvs->myOdometry->theta; // no theta with triangulation, so no theta with kalman


	// ------------------------------------------------------- Update opponents detection ---------------------------------------------- //


	opponents_tower(cvs);

	// tower control
	outputs->tower_command = 10.0;

	double tol = 0.1;


	// -------------------------------------------------------- Different States ----------------------------------------------- //


	switch (cvs->main_state)
	{

	case CALIB_STATE:
		// --------------------------------------------------- Calibration of the robot ------------------------------------------ //
		calibration(cvs);

		break;

	case WAIT_INIT_STATE:
		// --------------------------------------------------- Waiting for the start of the game ------------------------------------------ //

		speed_regulation(cvs, 0.0, 0.0);

		if (t > 0.0)
		{
			cvs->main_state = RUN_STATE;
		}
		break;


	case RUN_STATE:
		// --------------------------------------------------- Game time : robot plays until time is up ------------------------------------------ //
		main_strategy(cvs);

		if (t > 87.0) 
		{
			cvs->main_state = STOP_END_STATE;
		}
		break;

	case STOP_END_STATE:
		// --------------------------------------------------- Time is up, stopping the robot ------------------------------------------ //
		speed_regulation(cvs, 0.0, 0.0);
		break;

	default:
		exit(EXIT_FAILURE);
	}
}

/*! \brief last controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{

}


NAMESPACE_CLOSE();