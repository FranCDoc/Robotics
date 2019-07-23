/*! 
 * \author Nicolas Van der Noot
 * \file speed_regulation_ex.h
 * \brief speed regulation to track wheel speed references
 */

#ifndef _SPEED_REGULATION_GR9_H_
#define _SPEED_REGULATION_GR9_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr9.h"

NAMESPACE_INIT(ctrlGr9);

/// speed regulation
typedef struct SpeedRegulation
{
	double last_t; ///< last time the speed regulation was updated
	double previous_error; /// previous position error 
	double integral_error; /// integral of error (obtained as an iterative sum)
	double error_previous_r;
	double error_previous_l;
	double error_sum_r;
	double error_sum_l;
	double Ki;
	double Kp;
	double rho_wheel;
	double command_up_limit;
	double command_down_limit;
	double output_R;
	double output_L;
	double output_R_saturated;
	double output_L_saturated;
} SpeedRegulation;

// function prototype
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref);
double saturator(double up_limit, double down_limit, double output);

NAMESPACE_CLOSE();

#endif
