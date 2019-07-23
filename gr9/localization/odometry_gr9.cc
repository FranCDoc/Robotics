#include "odometry_gr9.h"
#include "init_pos_gr9.h"
#include "limit_angle_gr9.h"
#include "set_output.h"
#include "user_realtime.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr9);

/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */

void update_odometry(CtrlStruct *cvs)
{
	// -------------------------------------- Variables Declaration ------------------------- //


	CtrlIn *inputs; /// controller inputs
	RobotPosition *rob_pos; ///< robot initial position
    Odometry *myOdometry; /// robot odometry
    
	inputs  = cvs->inputs;
    rob_pos = cvs->rob_pos;
	myOdometry = cvs->myOdometry;
   
    double dt = cvs->dt;
	double d = myOdometry->d;
	double R = myOdometry->R; // R = wheel radius = 30mm
    double linear_r_speed = R*inputs->r_wheel_speed; // vr = R*wr // inputs => r_wheel_speed
    double linear_l_speed = R*inputs->l_wheel_speed; // vl = R*wl // inputs => l_wheel_speed
    double delta_sr = dt*linear_r_speed; // dsr = dt*vr
    double delta_sl = dt*linear_l_speed; // dsl = dt*vl
    
    myOdometry->delta_s = (delta_sr + delta_sl)/2.0; // ds = (dsr + dsl)/2
    myOdometry->speed = (linear_l_speed + linear_r_speed)/2; // V = (vr + vl)/2

    
    // ---------------------------------- Updating x, y and theta ----------------------------------- //


    myOdometry->delta_theta = (delta_sr - delta_sl)/(2.0*d); // dtheta = (dsr - dsl)/(2*d)

    double delta_x = myOdometry->delta_s*cos(myOdometry->theta + myOdometry->delta_theta/2); // dx = dV*cos(theta + dtheta/2)
	myOdometry->x += delta_x; 

    double delta_y = myOdometry->delta_s*sin(myOdometry->theta + myOdometry->delta_theta/2); // dy = dV*sin(theta + dtheta/2)
    myOdometry->y += delta_y; 

	myOdometry->theta += myOdometry->delta_theta;
	myOdometry->theta = limit_angle(myOdometry->theta);


    // ------------------------------------ Updating the robot's position ------------------------ //


	cvs->rob_pos->x = myOdometry->x;
    cvs->rob_pos->y = myOdometry->y;
    cvs->rob_pos->theta = myOdometry->theta;
    
    rob_pos->last_t = inputs->t;
}

NAMESPACE_CLOSE();
