#ifndef KALMAN_GR9_H
#define KALMAN_GR9_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr9.h"
#include "matrix_opperations_gr9.h"

NAMESPACE_INIT(ctrlGr9);

// function prototype


void kalman_filter(CtrlStruct *cvs);
int validity_check(double a, double b);

//void matrix_printer(Matrix* A, int n_row, int n_column);

// Triangulation
typedef struct Kalman
{
	double x; /// x position of the robot
	double y; /// y position of the robot
	double theta; /// angular position of the robot

	double kr;
	double kl;
	double R;
	double b;

	Matrix* L;
	Matrix* grad_p_f;
	Matrix* grad_dRL_f;
	Matrix* covar;
	Matrix* t1;
	Matrix* t2;
	Matrix* t3;
	Matrix* t4;
	Matrix* t5;
	Matrix* t6;
	Matrix* P_hat;
	Matrix* z;
	Matrix* z_hat;
	Matrix* minus_z_hat;
	Matrix* v_ij;
	Matrix* grad_h;
	Matrix* Rr;
	Matrix* grad_h_transpose;
	Matrix* grad_h_P_hat;
	Matrix* product;
	Matrix* innov_cov;
	Matrix* inverse_innov_cov;
	Matrix* P_hat_grad_h_transpose;
	Matrix* K;
	Matrix* K_transpose;
	Matrix* K_inovcov;
	Matrix* K_inovcov_K_transpose;
	Matrix* P_next;
	Matrix* Kv; 
	Matrix* state_update;
	Matrix* P; 


} Kalman;

NAMESPACE_CLOSE();

#endif