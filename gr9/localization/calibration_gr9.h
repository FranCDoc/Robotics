/*! 
 * \author Nicolas Van der Noot
 * \file calibration_ex.h
 * \brief calibration of the robot to know its accurate initial position
 */

#ifndef _CALIBRATION_GR9_H_
#define _CALIBRATION_GR9_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr9.h"

NAMESPACE_INIT(ctrlGr9);

/// robot calibration
typedef struct RobotCalibration
{
	double t_flag; ///< time to save
	int flag; ///< flag for calibration
	double X_calib;
	double Y_calib;
	double calib_step;
	double x_base_update;
	double y_base_update;

} RobotCalibration;

// function prototype
void calibration(CtrlStruct *cvs);
int init_choice_maker(double target_vect[6][2], double x, double y);

NAMESPACE_CLOSE();

#endif
