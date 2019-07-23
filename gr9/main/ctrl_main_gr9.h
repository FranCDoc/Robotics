/*!
 * \file ctrl_main_gr9.h
 * \brief main header of the controller
 */

#ifndef CTRL_MAIN_GR9_H
#define CTRL_MAIN_GR9_H

#include "CtrlStruct_gr9.h"
#include "namespace_ctrl.h"
#include <stdlib.h>

NAMESPACE_INIT(ctrlGr9);

// function prototypes
void controller_init(CtrlStruct *cvs);
void controller_loop(CtrlStruct *cvs);
void controller_finish(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif