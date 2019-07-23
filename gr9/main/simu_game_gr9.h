/*! 
 * \author Nicolas Van der Noot
 * \file simu_game_ex.h
 * \brief choose between controller for simulation game or for the real robot
 */

#ifndef _SIMU_GAME_GR9_H_
#define _SIMU_GAME_GR9_H_

#include "namespace_ctrl.h"

NAMESPACE_INIT(ctrlGr9);

#ifdef SIMU_PROJECT
	#define SIMU_GAME // comment this line to see in simulation the controller which will be tested on the real robot
#endif

NAMESPACE_CLOSE();

#endif
