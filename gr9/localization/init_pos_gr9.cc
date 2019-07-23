#include "init_pos_gr9.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr9);

/*! \brief set the initial robot position (guess if noise added)
 * 
 * \param[in] robot_id robot ID
 * \param[out] rob_pos robot position structure
 */
void set_init_position(int robot_id, RobotPosition *rob_pos)
{
	switch (robot_id)
	{
		case ROBOT_B: // blue robot
			rob_pos->x = 0.0;
			rob_pos->y = 0.0;
			rob_pos->theta = 0.0;
			break;

		case ROBOT_R: // red robot
			rob_pos->x = 0.0;
			rob_pos->y = 0.0;
			rob_pos->theta = 0.0;
			break;

		case ROBOT_Y: // yellow robot
			rob_pos->x = 0.0;
			rob_pos->y = 0.0;
			rob_pos->theta = 0.0;
			break;

		case ROBOT_W: // white robot
			rob_pos->x = 0.0;
			rob_pos->y = 0.0;
			rob_pos->theta = 0.0;
			break;
	
		default:
			printf("Initial position error: unknown robot ID: %d !\n", robot_id);
			exit(EXIT_FAILURE);
	}		
}

NAMESPACE_CLOSE();
