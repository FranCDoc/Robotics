/*! 
 * \file CtrlStruct_gr2.h
 * \brief Controller main structure
 */

#ifndef _CTRL_STRUCT_GR9_H_
#define _CTRL_STRUCT_GR9_H_

#include "ctrl_io.h"
#include "namespace_ctrl.h"
#include "user_realtime.h"
#include "set_output.h"
#include <stdlib.h>
#include <stdio.h>

NAMESPACE_INIT(ctrlGr9);

/// main states
enum { CALIB_STATE, WAIT_INIT_STATE, RUN_STATE, BASE_BACK, STOP_END_STATE, NB_MAIN_STATES };

/// robot IDs
enum { ROBOT_B, ROBOT_R, ROBOT_Y, ROBOT_W, NB_ROBOTS };

/// teams
enum { TEAM_A, TEAM_B, NB_TEAMS };


/// forward declaration
typedef struct RobotPosition RobotPosition;
typedef struct SpeedRegulation SpeedRegulation;
typedef struct RobotCalibration RobotCalibration;
typedef struct OpponentsPosition OpponentsPosition;
typedef struct PathPlanning PathPlanning;
typedef struct Strategy Strategy;
typedef struct Triangulation Triangulation;
typedef struct Odometry Odometry;
typedef struct Kalman Kalman;


/// Main controller structure
typedef struct CtrlStruct
{
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs; ///< controller outputs
	CtrlOut *py_outputs;

	RobotPosition *rob_pos; ///< robot position
	OpponentsPosition *opp_pos; ///< opponents position
	SpeedRegulation *sp_reg; ///< speed regulation
	RobotCalibration *calib; ///< calibration
	PathPlanning *path; ///< path-planning
	Strategy *strat; ///< strategy
    Triangulation *myTriangulation; ///< Triangulation
    Odometry *myOdometry; ///< Odometry
    Kalman *kalman; ///< kalman filter

    
    int main_state; ///< main state
	int robot_id;   ///< ID of the robot
	int team_id;    ///< ID of the team
	double x_out;
	double y_out;
	double x_base;
	double y_base;
    double dt; /// time interval
	double path_planning_l_speed;
	double path_planning_r_speed;
 
} CtrlStruct;

// function prototypes
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs, CtrlOut *py_outputs);
void free_CtrlStruct(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
