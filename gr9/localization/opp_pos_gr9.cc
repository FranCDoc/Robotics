#include "opp_pos_gr9.h"
#include <math.h>
#include "path_planning_gr9.h"
#include "init_pos_gr9.h"
#include "odometry_gr9.h"
#include "speed_regulation_gr9.h"
#include "user_realtime.h"
#include "set_output.h"
#include "limit_angle_gr9.h"
#include "kalman_gr9.h"

NAMESPACE_INIT(ctrlGr9);

/*! \brief compute the opponents position using the tower
 * 
 * \param[in,out] cvs controller main structure
 */
void opponents_tower(CtrlStruct *cvs)
{
	// ----------------------------------------------- Variables initialisation --------------------------------------- //


	int nb_opp; ///< number of opponents
	double R_opp = cvs->opp_pos->R_opp; ///< radius of the opponent beacon tower
	CtrlIn *inputs; ///< inputs
	RobotPosition *rob_pos; ///< robot own position
	OpponentsPosition *opp_pos; ///< opponents position
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;


	// ----------------------------------------------- Update of the opponent position --------------------------------------- //


	nb_opp = opp_pos->nb_opp;

	if (!nb_opp)
	{
		// no opponent
		return;
	}
	else
	{
		// angle initialisation
		double theta1 = cvs->inputs->last_rising[cvs->inputs->rising_index];
		double theta2 = cvs->inputs->last_falling[cvs->inputs->falling_index];
		double theta = (theta1 + theta2) / 2;
		double beta = (theta2 - theta1) / 2;

		// distance to oppponent
		double d = R_opp / sin(beta);
		double dx = d * cos(theta + cvs->rob_pos->theta);
		double dy = d * sin(theta + cvs->rob_pos->theta);

		// update opponent position
		cvs->opp_pos->x = cvs->rob_pos->x + dx;
		cvs->opp_pos->y = cvs->rob_pos->y + dy;

	}
}

NAMESPACE_CLOSE();
