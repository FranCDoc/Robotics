/*!
 * \author Nicolas Van der Noot
 * \file opp_pos_ex.h
 * \brief opponents position detection
 */

#ifndef OPP_POS_GR9_H
#define OPP_POS_GR9_H 

#include "namespace_ctrl.h"
#include "CtrlStruct_gr9.h"

NAMESPACE_INIT(ctrlGr9);

/// opponents position
typedef struct OpponentsPosition
{
	double x; ///< x position of opponents [m]
	double y; ///< y position of opponents [m]
	int nb_opp; ///< number of opponents
	double R_opp; ///< radius of the beacon tower [m]

} OpponentsPosition;

// function prototype
void opponents_tower(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif