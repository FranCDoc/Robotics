#include "CtrlStruct_gr9.h"
#include "namespace_ctrl.h"
#include "init_pos_gr9.h"
#include "odometry_gr9.h"
#include "opp_pos_gr9.h"
#include "matrix_opperations_gr9.h"
#include "triangulation_gr9.h"
#include "speed_regulation_gr9.h"
#include "calibration_gr9.h"
#include "strategy_gr9.h"
#include "path_planning_gr9.h"
#include "math.h"
#include "kalman_gr9.h"

NAMESPACE_INIT(ctrlGr9);

/*! \brief initialize the controller structure
 *
 * \param[in] inputs inputs of the controller
 * \param[in] outputs outputs of the controller
 * \return controller main structure
 *
 * Many parameters are set to arbitrary values, which will be corrected in 'controller_init'.
 */
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs, CtrlOut *py_outputs)
{
	//int i;

	CtrlStruct *cvs;

	cvs = (CtrlStruct*)malloc(sizeof(CtrlStruct));

	// io
	cvs->inputs = inputs;
	cvs->outputs = outputs;
	cvs->py_outputs = py_outputs;

	// global variables
	cvs->dt = 0.001;//[s]

	// states
	cvs->main_state = CALIB_STATE;

	// IDs
	cvs->robot_id = ROBOT_B;
	cvs->team_id = TEAM_A;

	// Base coordinates and way out point (point to aim to get out of the given base)
	cvs->x_out = 0.0;
	cvs->y_out = 0.0;
	cvs->x_base = 0.0;
	cvs->y_base = 0.0;

	// robot position variables
	cvs->rob_pos = (RobotPosition*)malloc(sizeof(RobotPosition));

	cvs->rob_pos->x = 0.0;
	cvs->rob_pos->y = 0.0;

	cvs->rob_pos->theta = -M_PI / 2;
	cvs->rob_pos->last_t = 0.0;


	// ----------------------------------------------------------------- Odometry ---------------------------------------------------------------- //


	cvs->myOdometry = (Odometry*)malloc(sizeof(Odometry));

	cvs->myOdometry->x = 0.0;
	cvs->myOdometry->y = 0.0;
	cvs->myOdometry->theta = -M_PI / 2;
	cvs->myOdometry->delta_theta = 0.0;
	cvs->myOdometry->temporal_theta = 0.0;

	cvs->myOdometry->delta_s = 0.0;
	cvs->myOdometry->delta_x = 0.0;
	cvs->myOdometry->delta_y = 0.0;

	cvs->myOdometry->speed = 0.0;
	cvs->myOdometry->last_t = 0.0;

	cvs->myOdometry->d = 112.5e-3;
	cvs->myOdometry->R = 0.03;


	// ----------------------------------------------------------------- Triangulation ---------------------------------------------------------------- //


	cvs->myTriangulation = (Triangulation*)malloc(sizeof(Triangulation));
	cvs->myTriangulation->x = 0.0;
	cvs->myTriangulation->y = 0.0;
	cvs->myTriangulation->theta = 0.0;

	cvs->myTriangulation->beacon_1_x = 0.0;
	cvs->myTriangulation->beacon_1_y = 0.0;
	cvs->myTriangulation->beacon_2_x = 0.0;
	cvs->myTriangulation->beacon_2_y = 0.0;
	cvs->myTriangulation->beacon_3_x = 0.0;
	cvs->myTriangulation->beacon_3_y = 0.0;

	cvs->myTriangulation->theorical_beacon_1_angle = 0.0;
	cvs->myTriangulation->theorical_beacon_2_angle = 0.0;
	cvs->myTriangulation->theorical_beacon_3_angle = 0.0;

	cvs->myTriangulation->measured_beacon_1_angle = 0.0; // will be used to compare to the practical one to identify which angle correspond to which beacon
	cvs->myTriangulation->measured_beacon_2_angle = 0.0;
	cvs->myTriangulation->measured_beacon_3_angle = 0.0;

	cvs->myTriangulation->last_rising_1 = 0.0; // most recent rising edge detected
	cvs->myTriangulation->last_rising_2 = 0.0;
	cvs->myTriangulation->last_rising_3 = 0.0;

	cvs->myTriangulation->last_falling_1 = 0.0;  // most recent falling edge detected
	cvs->myTriangulation->last_falling_2 = 0.0;
	cvs->myTriangulation->last_falling_3 = 0.0;

	cvs->myTriangulation->beacon_1_angle_theta = 0.0; // angle between robot directoin and the beacon
	cvs->myTriangulation->beacon_2_angle_theta = 0.0;
	cvs->myTriangulation->beacon_3_angle_theta = 0.0;

	cvs->myTriangulation->beacon_index_1 = 0; // will be used to stock the number of the beacon corresponding to the first detection
	cvs->myTriangulation->R_beacon = 0.004; // beacon radius
	cvs->myTriangulation->number_beacon = 0; // to stock the number of beacon(s) that will be used for triangulation

	// needed variables if 2 detected beacons
	cvs->myTriangulation->distance_1 = 0.0;
	cvs->myTriangulation->distance_2 = 0.0;
	cvs->myTriangulation->distance_3 = 0.0;

	// variables if 3 detected beacons
	cvs->myTriangulation->real_beacon_1_angle = 0.0;
	cvs->myTriangulation->real_beacon_2_angle = 0.0;
	cvs->myTriangulation->real_beacon_3_angle = 0.0;


	// ----------------------------------------------------------------- Kalman ---------------------------------------------------------------- //


	cvs->kalman = (Kalman*)malloc(sizeof(Kalman));
	cvs->kalman->x = 0.0;
	cvs->kalman->y = 0.0;
	cvs->kalman->theta = 0.0;
	cvs->kalman->P = init_mat(3, 3);
	zeros(cvs->kalman->P);
	cvs->kalman->kr = 0.007;
	cvs->kalman->kl = 0.007;
	cvs->kalman->R = 30.0e-3; // radius of wheel
	cvs->kalman->b = 2 * 112.5e-3;
	cvs->kalman->grad_p_f = init_mat(3, 3);
	zeros(cvs->kalman->grad_p_f);
	cvs->kalman->L = init_mat(3, 1);
	cvs->kalman->grad_dRL_f = init_mat(3, 2);
	zeros(cvs->kalman->grad_dRL_f);
	cvs->kalman->covar = init_mat(2, 2);
	zeros(cvs->kalman->covar);
	cvs->kalman->t1 = init_mat(3, 2);
	cvs->kalman->t2 = init_mat(2, 3);
	cvs->kalman->t3 = init_mat(3, 3);
	cvs->kalman->t4 = init_mat(3, 3);
	cvs->kalman->t5 = init_mat(3, 3);
	cvs->kalman->t6 = init_mat(3, 3);
	cvs->kalman->P_hat = init_mat(3, 3);
	cvs->kalman->z = init_mat(3, 1);
	cvs->kalman->z_hat = init_mat(3, 1);
	cvs->kalman->minus_z_hat = init_mat(3, 1);
	cvs->kalman->v_ij = init_mat(3, 1);
	cvs->kalman->grad_h = init_mat(3, 3);
	eye(cvs->kalman->grad_h);
	cvs->kalman->Rr = init_mat(3, 3);
	zeros(cvs->kalman->Rr);
	fill_matrix(cvs->kalman->Rr, 1, 1, 0.0050); // computed in matlab
	fill_matrix(cvs->kalman->Rr, 2, 2, 0.0098); // computed in matlab
	fill_matrix(cvs->kalman->Rr, 3, 3, 0.015); // tower noise
	cvs->kalman->grad_h_transpose = init_mat(3, 3);
	cvs->kalman->grad_h_P_hat = init_mat(3, 3);
	cvs->kalman->product = init_mat(3, 3);
	cvs->kalman->innov_cov = init_mat(3, 3);
	cvs->kalman->inverse_innov_cov = init_mat(3, 3);
	cvs->kalman->P_hat_grad_h_transpose = init_mat(3, 3);
	cvs->kalman->K = init_mat(3, 3);
	cvs->kalman->K_transpose = init_mat(3, 3);
	cvs->kalman->K_inovcov = init_mat(3, 3);
	cvs->kalman->K_inovcov_K_transpose = init_mat(3, 3);
	cvs->kalman->P_next = init_mat(3, 3);
	cvs->kalman->Kv = init_mat(3, 1);
	cvs->kalman->state_update = init_mat(3, 1);


	// ----------------------------------------------------------------- Opponent position ---------------------------------------------------------------- //


	cvs->opp_pos = (OpponentsPosition*)malloc(sizeof(OpponentsPosition));


	cvs->opp_pos->x = 0.0;
	cvs->opp_pos->y = 0.0;
	cvs->opp_pos->nb_opp = inputs->nb_opponents;
	cvs->opp_pos->R_opp = 0.04;


	// ----------------------------------------------------------------- Speed Regulation ---------------------------------------------------------------- //


	cvs->sp_reg = (SpeedRegulation*)malloc(sizeof(SpeedRegulation));

	cvs->sp_reg->last_t = 0.0;

	cvs->sp_reg->error_previous_r = 0;
	cvs->sp_reg->error_previous_l = 0;
	cvs->sp_reg->error_sum_r = 0;
	cvs->sp_reg->error_sum_l = 0;
	// Two followings : manual tuning to avoid overshoot and having fast reactivity
	cvs->sp_reg->Kp = 0.142;
	cvs->sp_reg->Ki = 2.0555;
	cvs->sp_reg->rho_wheel = 14.0;
	cvs->sp_reg->command_down_limit = -80.0;
	cvs->sp_reg->command_up_limit = 80.0;
	cvs->sp_reg->output_R = 0.0;
	cvs->sp_reg->output_L = 0.0;
	cvs->sp_reg->output_R_saturated = 0.0;
	cvs->sp_reg->output_L_saturated = 0.0;



	// ----------------------------------------------------------------- Calibration ---------------------------------------------------------------- //


	cvs->calib = (RobotCalibration*)malloc(sizeof(RobotCalibration));

	cvs->calib->flag = 0;
	cvs->calib->t_flag = 0.0;
	cvs->calib->X_calib = 0.0;
	cvs->calib->Y_calib = 0.0;
	cvs->calib->x_base_update = 0.0;
	cvs->calib->y_base_update = 0.0;
	cvs->calib->calib_step = 0.0;


	// ----------------------------------------------------------------- Strategy ---------------------------------------------------------------- //


	cvs->strat = (Strategy*)malloc(sizeof(Strategy));
	cvs->strat->state = Target_Drive;
	cvs->strat->counter = 0;
	cvs->strat->index = 0;
	cvs->strat->tol = 0.05;
	cvs->strat->tol2 = 0.15;
	cvs->strat->t_out = 0.0;
	cvs->strat->t_stop = 0.0;
	cvs->strat->t_state = 0.0;


	// ----------------------------------------------------------------- Path Planning ---------------------------------------------------------------- //


	cvs->path = (PathPlanning*)malloc(sizeof(PathPlanning));
	cvs->path->k_att = 10.0;
	cvs->path->k_rep = 2.0;
	cvs->path->rho_0 = 0.19;
	cvs->path->F_bound = 100.0; // bound of the saturation of the forces
	cvs->path->k_rho = 3.; // course value, to be modified if needed
	cvs->path->k_alpha = 8.; // course value, to be modified if needed
	cvs->path->k_beta = -5.; // course value, to be modified if needed
	cvs->path->rho_0_opp = 0.4;



	return cvs;
}

/*! \brief release controller main structure memory
 *
 * \param[in] cvs controller main structure
 */
void free_CtrlStruct(CtrlStruct *cvs)
{
	free(cvs->path);
	free(cvs->myTriangulation);
	free(cvs->strat);
	free(cvs->calib);
	free(cvs->sp_reg);
	free(cvs->opp_pos);
	free(cvs->rob_pos);
	free(cvs->myOdometry);
	free(cvs->kalman);

	free(cvs);
}

NAMESPACE_CLOSE();