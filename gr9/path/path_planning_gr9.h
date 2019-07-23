/*!
 * \author Nicolas Van der Noot
 * \file path_planning_ex.h
 * \brief path-planning algorithm
 */

#ifndef PATH_PLANNING_GR9_H
#define PATH_PLANNING_GR9_H

#include "CtrlStruct_gr9.h"
#include "namespace_ctrl.h"

NAMESPACE_INIT(ctrlGr9);

/// path-planning main structure
struct PathPlanning
{
	double F_rep_x;
	double F_rep_y;
	double F_att_x;
	double F_att_y;
	double F_x_tot;
	double F_y_tot;
	double k_att;
	double k_rep;
	double rho_0;
	double k_rho;
	double k_alpha;
	double k_beta;
	double rho;
	double alpha_tot;
	double alpha;
	double beta;
	double y_dot;
	double theta_dot;
	double F_bound;
	double rho_0_opp;
};

// function prototype
void path_planning_update(CtrlStruct *cvs, double x_target, double y_target);
double Fatt(double x, double k_att, double x_target);
double min_dist(RobotPosition *rob_pos, double x2, double y2);
double Frep(CtrlStruct *cvs, double k_rep, double rho_0, int direction);
double limit_alpha(double alpha);
double force_saturator(double up_limit, double down_limit, double force);

NAMESPACE_CLOSE();

#endif