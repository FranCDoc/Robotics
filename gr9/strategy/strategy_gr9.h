/*!
 * \author Nicolas Van der Noot
 * \file strategy_ex.h
 * \brief strategy during the game
 */

#ifndef _STRATEGY_GR9_H_
#define _STRATEGY_GR9_H_

#include "CtrlStruct_gr9.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr9);

// strategy main states
enum {Target_Drive, Target_Grab, Base_or_Next, Go_in_Base, Go_out_Base, No_more_Target, STRAT_STATE_DEBUG};

/// strategy
typedef struct Strategy
{
	int state; ///< main state of the strategy
	int counter;
	int index;
	double x_target;
	double y_target;
	double t_state;
	double t_stop;
	double t_out;
	double tol;
	double tol2;

} Strategy;

// function prototype
void main_strategy(CtrlStruct *cvs);
int target_choice_maker(double target_vect[7][3], CtrlStruct *cvs);


NAMESPACE_CLOSE();

#endif