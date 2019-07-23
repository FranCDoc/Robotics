/*! 
 * \author Nicolas Van der Noot
 * \file init_pos_ex.h
 * \brief initial position of the robot approximate guess
 */

#ifndef _INIT_POS_GR9_H_
#define _INIT_POS_GR9_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr9.h"

NAMESPACE_INIT(ctrlGr9);

/// robot position (to update with odometry)
typedef struct RobotPosition
{
    double x; ///< x position [m]
	double y; ///< y position [m]
	double theta; ///< robot orientation [rad]
	double last_t; ///< last time odometry was updated
    
} RobotPosition;

// function prototype
void set_init_position(int robot_id, RobotPosition *rob_pos);

NAMESPACE_CLOSE();

#endif
