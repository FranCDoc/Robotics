#include "odometry_gr9.h"
#include "limit_angle_gr9.h"
#include "matrix_opperations_gr9.h"
#include "kalman_gr9.h"
#include "init_pos_gr9.h"
#include "triangulation_gr9.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr9);

/*! \brief kalman filter
 *
 * Kalman filter main structure
 */



 // cvs => sacado de CtrlStruct
void kalman_filter(CtrlStruct *cvs)
{
	// variables declaration
	CtrlIn *inputs;           ///< controller inputs
	Kalman *kalman;           ///< Kalman filter
	Odometry *myOdometry; // robot odometry
	RobotPosition *rob_pos; ///< robot position
	Triangulation *rob_tri; ///< robot triangulation

	// variables initialization
	inputs = cvs->inputs;
	myOdometry = cvs->myOdometry;
	kalman = cvs->kalman;
	rob_tri = cvs->myTriangulation;
	rob_pos = cvs->rob_pos;

	double R = cvs->kalman->R;
	double dt = cvs->dt;
	double linear_r_speed = R * inputs->r_wheel_speed;
	double linear_l_speed = R * inputs->l_wheel_speed;
	double dSR = linear_r_speed * dt;
	double dSL = linear_l_speed * dt;
	double theta = myOdometry->theta;
	double b = cvs->kalman->b;
	


	// ---------------------- Part I ----------------------- // 

	// error model for odometric position estimation (slides 14 L4)
	double dS = (dSR + dSL) / 2;
	double dTheta = (dSR - dSL) / b;
	double phiK = theta + dTheta / 2; // = theta prime
	double l1 = dS * cos(phiK);
	double l2 = dS * sin(phiK);
	double l3 = dTheta;
	Matrix* L = cvs->kalman->L;
	fill_matrix(L, 1, 1, l1);
	fill_matrix(L, 2, 1, l2);
	fill_matrix(L, 3, 1, l3);

	// error model for odometric position estimation (slides 15 L4)

	Matrix* grad_p_f = cvs->kalman->grad_p_f;
	fill_matrix(grad_p_f, 1, 1, 1);
	fill_matrix(grad_p_f, 2, 2, 1);
	fill_matrix(grad_p_f, 3, 3, 1);
	fill_matrix(grad_p_f, 1, 3, -dS * sin(phiK));
	fill_matrix(grad_p_f, 2, 3, dS*cos(phiK));

	Matrix* grad_dRL_f = cvs->kalman->grad_dRL_f;
	fill_matrix(grad_dRL_f, 1, 1, cos(phiK) / 2 - dS * sin(phiK) / (2 * b));
	fill_matrix(grad_dRL_f, 1, 2, cos(phiK) / 2 + dS * sin(phiK) / (2 * b));
	fill_matrix(grad_dRL_f, 2, 1, sin(phiK) / 2 + dS * cos(phiK) / (2 * b));
	fill_matrix(grad_dRL_f, 2, 2, sin(phiK) / 2 - dS * cos(phiK) / (2 * b));
	fill_matrix(grad_dRL_f, 3, 1, 1 / b);
	fill_matrix(grad_dRL_f, 3, 2, -1 / b);

	Matrix* covar = cvs->kalman->covar;
	fill_matrix(covar, 1, 1, cvs->kalman->kr*fabs(dSR));
	fill_matrix(covar, 2, 2, cvs->kalman->kl*fabs(dSL));

	// computation of error propagation law : (mn)*(np)=(mp)
	// grad_p_f * Kalman->p * grad_p_f_transpose + grad_dRL_f * covar * grad_dRL_f_transpose
	Matrix* t1 = cvs->kalman->t1; // grad_dRL_f * covar
	Matrix* t2 = cvs->kalman->t2; // grad_dRL_f_transpose
	Matrix* t3 = cvs->kalman->t3; // grad_dRL_f * covar * grad_dRL_f_transpose
	mat_product(t1, grad_dRL_f, covar);
	transpose(t2, grad_dRL_f);
	mat_product(t3, t1, t2); // grad_dRL_f * covar * grad_dRL_f_transpose

	Matrix* t4 = cvs->kalman->t4; // grad_p_f * Kalman->p
	Matrix* t5 = cvs->kalman->t5; // grad_p_f_transpose
	Matrix* t6 = cvs->kalman->t6; // grad_p_f * Kalman->p * grad_p_f_transpose
	mat_product(t4, grad_p_f, kalman->P);
	transpose(t5, grad_p_f);
	mat_product(t6, t4, t5); // grad_p_f * Kalman->p * grad_p_f_transpose

	Matrix* P_hat = cvs->kalman->P_hat;
	mat_sum(P_hat, t6, t3); // Kalman->P (t+1) will be found later on with this P_hat



	 // ---------------------- Part II ----------------------- // 

	 /// observation (done by triangulation) : z ==> triangulation 

	Matrix* z = cvs->kalman->z;
	fill_matrix(z, 1, 1, rob_tri->x);
	fill_matrix(z, 2, 1, rob_tri->y);
	fill_matrix(z, 3, 1, myOdometry->theta);

	///// measurement prediction (done by the odometry) : z_hat ==> odometry

	Matrix* z_hat = cvs->kalman->z_hat; // minus sign to allow diff in mat sum 
	fill_matrix(z_hat, 1, 1, myOdometry->x);
	fill_matrix(z_hat, 2, 1, myOdometry->y);
	fill_matrix(z_hat, 3, 1, myOdometry->theta);

	///// Matching 
	Matrix* minus_z_hat = cvs->kalman->minus_z_hat; // minus sign to allow diff in mat sum 
	fill_matrix(minus_z_hat, 1, 1, -myOdometry->x);
	fill_matrix(minus_z_hat, 2, 1, -myOdometry->y);
	fill_matrix(minus_z_hat, 3, 1, -myOdometry->theta);
	Matrix* v_ij = cvs->kalman->v_ij;
	mat_sum(v_ij, z, minus_z_hat);

	Matrix* grad_h = cvs->kalman->grad_h;
	Matrix* Rr = cvs->kalman->Rr; // à determiner par essai dans simulateur , R depend de la position (on peut prendre les val au pire cas)


	/// computation innovation covariance matrix : grad_h * P_hat * grad_h_transpose + R
	Matrix* grad_h_transpose = cvs->kalman->grad_h_transpose;
	transpose(grad_h_transpose, grad_h);
	Matrix* grad_h_P_hat = cvs->kalman->grad_h_P_hat;
	mat_product(grad_h_P_hat, grad_h, P_hat);
	Matrix* product = cvs->kalman->product;
	mat_product(product, grad_h_P_hat, grad_h_transpose);

	Matrix* innov_cov = cvs->kalman->innov_cov;
	mat_sum(innov_cov, product, Rr);

	//// Computation of the validation gate 

	double v_ij_1 = v_ij->m[0][0];
	double v_ij_2 = v_ij->m[1][0];
	double v_ij_3 = v_ij->m[2][0];

	double inov_cov_11 = innov_cov->m[0][0];
	double inov_cov_12 = innov_cov->m[0][1];
	double inov_cov_13 = innov_cov->m[0][2];
	double inov_cov_21 = innov_cov->m[1][0];
	double inov_cov_22 = innov_cov->m[1][1];
	double inov_cov_23 = innov_cov->m[1][2];
	double inov_cov_31 = innov_cov->m[2][0];
	double inov_cov_32 = innov_cov->m[2][1];
	double inov_cov_33 = innov_cov->m[2][2];

	double l1c1 = v_ij_1 * inov_cov_11 + v_ij_2 * inov_cov_21 + v_ij_3 * inov_cov_31;
	double l1c2 = v_ij_1 * inov_cov_12 + v_ij_2 * inov_cov_22 + v_ij_3 + inov_cov_32;
	double l1c3 = v_ij_1 * inov_cov_13 + v_ij_2 * inov_cov_23 + v_ij_3 * inov_cov_33;

	double Mahalanobis_dist = l1c1 * v_ij_1 + l1c2 * v_ij_2 + l1c3 * v_ij_3;


	double g = 0.10; /// to redefine properly if needed
	int validity_checker = validity_check(Mahalanobis_dist, g);


	////////////// Update (if validity is checked)

	if (validity_checker == 1)
	{
		//// Computation Kalman gain matrix

		Matrix* inverse_innov_cov = cvs->kalman->inverse_innov_cov;
		inverse(inverse_innov_cov, innov_cov);
		Matrix* P_hat_grad_h_transpose = cvs->kalman->P_hat_grad_h_transpose;
		mat_product(P_hat_grad_h_transpose, P_hat, grad_h_transpose);
		Matrix *K = cvs->kalman->K;
		mat_product(K, P_hat_grad_h_transpose, inverse_innov_cov);

		//// update kalman->P

		Matrix* K_transpose = cvs->kalman->K_transpose;
		transpose(K_transpose, K);

		Matrix* K_inovcov = cvs->kalman->K_inovcov;
		mat_product(K_inovcov, K, innov_cov);
		Matrix* K_inovcov_K_transpose = cvs->kalman->K_inovcov_K_transpose;
		mat_product(K_inovcov_K_transpose, K_inovcov, K_transpose);
		mat_const_product(K_inovcov_K_transpose, -1);
		Matrix* P_next = cvs->kalman->P_next;
		mat_sum(P_next, P_hat, K_inovcov_K_transpose);

		kalman->P = P_next;

		//// update x,y,theta ==> if validation gate 

		Matrix* Kv = cvs->kalman->Kv;
		mat_product(Kv, K, v_ij);
		Matrix* state_update = cvs->kalman->state_update;
		mat_sum(state_update, z_hat, Kv);

		kalman->x = state_update->m[0][0];
		kalman->y = state_update->m[1][0];
		kalman->theta = state_update->m[2][0];

	}
	else
	{
		kalman->x = myOdometry->x;
		kalman->y = myOdometry->y;
		kalman->theta = myOdometry->theta;
	}

}

int validity_check(double a, double b)
{
	// return 1 if a <= b, 0 otherwise
	if (a <= (b))
	{
		return 1;
	}
	else
	{
		return -1;
	}

}


NAMESPACE_CLOSE();