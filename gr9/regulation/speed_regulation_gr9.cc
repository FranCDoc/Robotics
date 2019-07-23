#include "speed_regulation_gr9.h"
#include "user_realtime.h"
#include "set_output.h"
#include "odometry_gr9.h"
#include "path_planning_gr9.h"
#include "init_pos_gr9.h"
#include "triangulation_gr9.h"
#include "kalman_gr9.h"

NAMESPACE_INIT(ctrlGr9);

#define DUMMY_VAR 10.0 ///< dummy variable

double previous_error_sum_l = 0;
double previous_error_sum_r = 0;

/*! \brief wheels speed regulation
 *
 * \param[in,out] cvs controller main structure
 * \parem[in] r_sp_ref right wheel speed reference [rad/s]
 * \parem[in] l_sp_ref left wheel speed reference [rad/s]
 */
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref)
{
	// ------------------------------------------------ Controller Equation --------------------------------------------- //
	//																													  //
	//											e = w_ref - w_m												              //
	//											U = Kp * e + Ki * int(e) + Kd * de/dt									  //
	//																													  //
	// ------------------------------------------------------------------------------------------------------------------ //


	// ---------------------------------------------- Variables Declaration --------------------------------------------- //


	CtrlIn  *inputs;
	CtrlOut *outputs;
	SpeedRegulation *sp_reg;

	// variables initialization
	inputs = cvs->inputs;
	outputs = cvs->outputs;
	sp_reg = cvs->sp_reg;
	double Ki = cvs->sp_reg->Ki;
	double Kp = cvs->sp_reg->Kp;
	double dt = cvs->dt;
	double rho_wheel = cvs->sp_reg->rho_wheel;

	// needed informations
	double r_wheel_speed = inputs->r_wheel_speed;
	double l_wheel_speed = inputs->l_wheel_speed;

	// needed variable 
	double e_r = rho_wheel * (r_sp_ref - r_wheel_speed);
	double e_l = rho_wheel * (l_sp_ref - l_wheel_speed);
	double e_r_previous = sp_reg->error_previous_r;
	double e_l_previous = sp_reg->error_previous_l;
	double output_R = cvs->sp_reg->output_R;
	double output_L = cvs->sp_reg->output_L;
	double output_R_saturated = cvs->sp_reg->output_R_saturated;
	double output_L_saturated = cvs->sp_reg->output_L_saturated;

	// needed integral for integrate term
	sp_reg->error_sum_r += (e_r + e_r_previous) * dt / 2;
	sp_reg->error_sum_l += (e_l + e_l_previous) * dt / 2;

	// needed derivate for derivate term
	double de_r = (e_r - e_r_previous) / dt;
	double de_l = (e_l - e_l_previous) / dt;


	// ------------------------------------------------- Computing and updating the wheel commands ------------------------------------------------- //


	double command_up_limit = cvs->sp_reg->command_up_limit; // command is controlled with[-100;100] and system is staurated for a voltage 0.8*[-24;24]
	double command_down_limit = cvs->sp_reg->command_down_limit; // so these are the limit for the command
	output_R = (Kp * e_r + Ki * sp_reg->error_sum_r); // +Kd * de_r; // for one PID
	output_L = (Kp * e_l + Ki * sp_reg->error_sum_l); // +Kd * de_l; // for one PID
	output_R_saturated = saturator(command_up_limit, command_down_limit, output_R); // to saturate tension if too big
	output_L_saturated = saturator(command_up_limit, command_down_limit, output_L);
	outputs->wheel_commands[R_ID] = output_R_saturated; // the command is the duty cycle : between -100 and 100 where 100 correspond to 24*0.8
	outputs->wheel_commands[L_ID] = output_L_saturated;


	// ---------------------------------------------------- Updating variables ---------------------------------------------------------- // 


	sp_reg->error_previous_r = e_r;
	sp_reg->error_previous_l = e_l;
	sp_reg->last_t = inputs->t;

}



//////////////////////////////////////////////////////// Needed Functions /////////////////////////////////////////////////////////


double saturator(double up_limit, double down_limit, double output) {
	if (output > up_limit) {
		return up_limit;
	}
	else if (output < down_limit) {
		return down_limit;
	}
	else {
		return output;
	}
}

NAMESPACE_CLOSE();
