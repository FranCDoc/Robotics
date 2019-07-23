#include "odometry_gr9.h"
#include "triangulation_gr9.h"
#include "set_output.h"
#include "user_realtime.h"
#include <cstdlib>
#include "init_pos_gr9.h"
#include "limit_angle_gr9.h"

#include <math.h>

NAMESPACE_INIT(ctrlGr9);

void update_triangulation(CtrlStruct *cvs){

	CtrlIn *inputs;
	RobotPosition *rob_pos; ///< robot position = odometry
	Odometry *myOdometry ;
	Triangulation *rob_tri;

	// variables initialization
	inputs = cvs->inputs;
	rob_pos = cvs->rob_pos;
	rob_tri = cvs->myTriangulation;
	myOdometry = cvs->myOdometry;
  
	//last_rising     	rotating list with the last rising edges detected [rad]
	//last_falling 		rotating list with the last falling edges detected [rad]
	//rising_index 		index in 'last_rising' of the last element added
	//falling_index		index in 'last_falling' of the last element added
	//nb_rising 		number of rising edges detected during the last laser revolution
	//nb_falling 		number of falling edges detected during the last laser revolution
  

    //determine positions of the beacons

	double beacon_1_x = cvs->myTriangulation->beacon_1_x;
	double beacon_1_y = cvs->myTriangulation->beacon_1_y;
	double beacon_2_x = cvs->myTriangulation->beacon_2_x;
	double beacon_2_y = cvs->myTriangulation->beacon_2_y;
	double beacon_3_x = cvs->myTriangulation->beacon_3_x;
	double beacon_3_y = cvs->myTriangulation->beacon_3_y;

    switch(cvs->team_id){
       case TEAM_A:
		   beacon_1_x = 1.062;
		   beacon_1_y = 1.562;
		   
		   beacon_2_x = -1.062;
           beacon_2_y = 1.562;

           beacon_3_x = 0.0;
           beacon_3_y = -1.562;
           
		   break;

       case TEAM_B:
		   beacon_1_x = 1.062;
		   beacon_1_y = -1.562;

           beacon_2_x = -1.062;
           beacon_2_y = -1.562;

           beacon_3_x = 0.0;
           beacon_3_y = 1.562;

           break;

       default:
            // printf("Error team detetion: unknown robot ID: %d !\n", inputs->robot_id);
            exit(EXIT_FAILURE);
	}


	double theorical_beacon_1_angle = atan2(beacon_1_y - cvs->rob_pos->y, beacon_1_x - cvs->rob_pos->x); // theorical angle between x axis and beacon
	double theorical_beacon_2_angle = atan2(beacon_2_y - cvs->rob_pos->y, beacon_2_x - cvs->rob_pos->x);
	double theorical_beacon_3_angle = atan2(beacon_3_y - cvs->rob_pos->y, beacon_3_x - cvs->rob_pos->x);

	double measured_beacon_1_angle = cvs->myTriangulation->measured_beacon_1_angle; // will be used to compare to the practical one to identify which angle correspond to which beacon
	double measured_beacon_2_angle = cvs->myTriangulation->measured_beacon_2_angle;
	double measured_beacon_3_angle = cvs->myTriangulation->measured_beacon_3_angle;
  
    double last_rising_1 = cvs->myTriangulation->last_rising_1; // most recent rising edge detected
    double last_rising_2 = cvs->myTriangulation->last_rising_2;
    double last_rising_3 = cvs->myTriangulation->last_rising_3;
    
    double last_falling_1 = cvs->myTriangulation->last_falling_1;  // most recent falling edge detected
    double last_falling_2 = cvs->myTriangulation->last_falling_2;
    double last_falling_3 = cvs->myTriangulation->last_falling_3;
    
    double beacon_1_angle_theta = cvs->myTriangulation->beacon_1_angle_theta; // angle between robot directoin and the beacon
    double beacon_2_angle_theta = cvs->myTriangulation->beacon_2_angle_theta;
    double beacon_3_angle_theta = cvs->myTriangulation->beacon_3_angle_theta;

	int beacon_index_1 = cvs->myTriangulation->beacon_index_1; // will be used to stock the number of the beacon corresponding to the first detection
	double R_beacon = cvs->myTriangulation->R_beacon; // beacon radius
	int number_beacon = cvs->myTriangulation->number_beacon; // to stock the number of beacon(s) that will be used for triangulation

	// variables if 2 detected beacons
	float distance_1 = sqrt((beacon_1_x - cvs->rob_pos->x)*(beacon_1_x - cvs->rob_pos->x) + (beacon_1_y - cvs->rob_pos->y)*(beacon_1_y - cvs->rob_pos->y));
	float distance_2 = sqrt((beacon_2_x - cvs->rob_pos->x)*(beacon_2_x - cvs->rob_pos->x) + (beacon_2_y - cvs->rob_pos->y)*(beacon_2_y - cvs->rob_pos->y));
	float distance_3 = sqrt((beacon_3_x - cvs->rob_pos->x)*(beacon_3_x - cvs->rob_pos->x) + (beacon_3_y - cvs->rob_pos->y)*(beacon_3_y - cvs->rob_pos->y));

	// variables if 3 detected beacons
	double real_beacon_1_angle = cvs->myTriangulation->real_beacon_1_angle;
	double real_beacon_2_angle = cvs->myTriangulation->real_beacon_2_angle;
	double real_beacon_3_angle = cvs->myTriangulation->real_beacon_3_angle;

    
    //----------------------- Angle of the rising edges of the beacon measured ------------------------//

	
    
	if (cvs->inputs->nb_rising_fixed >= 3) // 3 rising detected, 3 OR 2 falling can be detected
	{
		if (cvs->inputs->nb_falling_fixed == 3) { // 3 rising and falling detected
			last_rising_1 = inputs->last_rising_fixed[inputs->rising_index_fixed];
			last_rising_2 = inputs->last_rising_fixed[inputs->rising_index_fixed - 1];
			last_rising_3 = inputs->last_rising_fixed[inputs->rising_index_fixed - 2];

			last_falling_1 = inputs->last_falling_fixed[inputs->falling_index_fixed];
			last_falling_2 = inputs->last_falling_fixed[inputs->falling_index_fixed - 1];
			last_falling_3 = inputs->last_falling_fixed[inputs->falling_index_fixed - 2];


			beacon_1_angle_theta = (last_falling_1 + last_rising_1) / 2.0; // theta = angle pointing directly at the middle of the beacon
			beacon_2_angle_theta = (last_falling_2 + last_rising_2) / 2.0;
			beacon_3_angle_theta = (last_falling_3 + last_rising_3) / 2.0;
			

			number_beacon = 3;

		}
		else if (cvs->inputs->nb_falling_fixed == 2) { // 3 rising detected while only 2 falling detected : last rising does not to be taken into account
			last_rising_1 = inputs->last_rising_fixed[inputs->rising_index_fixed - 1];
			last_rising_2 = inputs->last_rising_fixed[inputs->rising_index_fixed - 2];

			last_falling_1 = inputs->last_falling_fixed[inputs->falling_index_fixed];
			last_falling_2 = inputs->last_falling_fixed[inputs->falling_index_fixed - 1];

			beacon_1_angle_theta = (last_falling_1 + last_rising_1) / 2.0; // theta = angle pointing directly at the middle of the beacon
			beacon_2_angle_theta = (last_falling_2 + last_rising_2) / 2.0;

			number_beacon = 2;

		}
		else
		{
			// this case should no append logically, to be secure there is put values that kalman will reject
			cvs->myTriangulation->x = 100.0;
			cvs->myTriangulation->y = 100.0;
			cvs->myTriangulation->theta = 10 * M_PI;

			number_beacon = -1;
		}
    }
    else if ( cvs->inputs->nb_rising_fixed == 2) // 2 rising detected, 1 OR 2 falling can be detected
	{
		if (cvs->inputs->nb_falling_fixed == 2) { // 2 rising and falling detected
			last_rising_1 = inputs->last_rising_fixed[inputs->rising_index_fixed];
			last_rising_2 = inputs->last_rising_fixed[inputs->rising_index_fixed - 1];

			last_falling_1 = inputs->last_falling_fixed[inputs->falling_index_fixed];
			last_falling_2 = inputs->last_falling_fixed[inputs->falling_index_fixed - 1];

			beacon_1_angle_theta = (last_falling_1 + last_rising_1) / 2.0; // theta = angle pointing directly at the middle of the beacon
			beacon_2_angle_theta = (last_falling_2 + last_rising_2) / 2.0;

			number_beacon = 2;

		}
		else if (cvs->inputs->nb_falling_fixed == 1) { // 2 rising detected while only 1 falling detected : last rising does not to be taken into account
			last_rising_1 = inputs->last_rising_fixed[inputs->rising_index_fixed - 1];

			last_falling_1 = inputs->last_falling_fixed[inputs->falling_index_fixed];

			beacon_1_angle_theta = (last_falling_1 + last_rising_1) / 2.0; // theta = angle pointing directly at the middle of the beacon

			number_beacon = 1;
		}
		else
		{
			// this case should no append logically, to be secure there is put values that kalman will reject
			cvs->myTriangulation->x = 100.0;
			cvs->myTriangulation->y = 100.0;
			cvs->myTriangulation->theta = 10 * M_PI;

			number_beacon = -1;
		}
    }
	else if ( cvs->inputs->nb_rising_fixed == 1) // 1 rising detected, 1 OR 0 falling can be detected
	{
		if (cvs->inputs->nb_falling_fixed >=1) { // 1 rising and falling detected
			last_rising_1 = inputs->last_rising_fixed[inputs->rising_index_fixed];

			last_falling_1 = inputs->last_falling_fixed[inputs->falling_index_fixed];

			beacon_1_angle_theta = (last_falling_1 + last_rising_1) / 2.0; // theta = angle pointing directly at the middle of the beacon

			number_beacon = 1;

		}
		else if (cvs->inputs->nb_falling_fixed <= 0)// 1 rising detected while no falling detected : trinagulation is impossible
		{ 
			// triangulation impossible, put values that kalman will reject
			cvs->myTriangulation->x = 100.0;
			cvs->myTriangulation->y = 100.0;
			cvs->myTriangulation->theta = 4 * M_PI;

			number_beacon = -1;
		}
	}
	else
	{
		// no beacon detected at this step, tringualation impossible, put values that kalman will reject
		cvs->myTriangulation->x = 100.0;
		cvs->myTriangulation->y = 100.0;
		cvs->myTriangulation->theta = 10 * M_PI;

		// printf("0 rising edge \n");
		number_beacon = -1;
	}


   //------------------------------------- Identification of Beacons + localization---------------------------------------//


   // identify the detected angle from the x axis of the global frame by the robot
   // then , by comparing the identified angle with the real angle, one can deduct which angle correspond to which beacon
   if (number_beacon == 3) {
	   measured_beacon_1_angle = limit_angle(cvs->rob_pos->theta + beacon_1_angle_theta); // angle between the Ox axis and the beacon
	   measured_beacon_2_angle = limit_angle(cvs->rob_pos->theta + beacon_2_angle_theta);
	   measured_beacon_3_angle = limit_angle(cvs->rob_pos->theta + beacon_3_angle_theta);

	   // association to real_beacon_i_angle of the angle between the robot and the beacon i 
	   int beacon_index_1 = beacon_identifier(measured_beacon_1_angle, theorical_beacon_1_angle, theorical_beacon_2_angle, theorical_beacon_3_angle); // beacon_index_1 contain the number of the beacon to which correspond identified_beacon_1_angle
	   // printf("beacon_index_1 = %d \n", beacon_index_1);
	   if (beacon_index_1 == 1) // mean that beacon_1_angle_theta orrespond to beacon 1
	   {
		   real_beacon_1_angle = measured_beacon_1_angle;
		   real_beacon_2_angle = measured_beacon_3_angle;
		   real_beacon_3_angle = measured_beacon_2_angle;
	   }
	   else if (beacon_index_1 == 2) // mean that beacon_1_angle_theta orrespond to beacon 2
	   {
		   real_beacon_1_angle = measured_beacon_3_angle;
		   real_beacon_2_angle = measured_beacon_1_angle;
		   real_beacon_3_angle = measured_beacon_2_angle;
	   }
	   else if (beacon_index_1 == 3) // mean that beacon_1_angle_theta orrespond to beacon 3
	   {
		   real_beacon_1_angle = measured_beacon_2_angle;
		   real_beacon_2_angle = measured_beacon_3_angle;
		   real_beacon_3_angle = measured_beacon_1_angle;
	   }
		else 
	   {
		   // printf("no angle correlation \n");
		}

 
	   // Localization /!\ ALGORITHME  inspired by  * @file hmam.c of author Vincent Pierlot at http://www.telecom.ulg.ac.be/triangulation/ " 

	   float d12 = sqrt((beacon_2_x - beacon_1_x)*(beacon_2_x - beacon_1_x) + (beacon_2_y - beacon_1_y)*(beacon_2_y - beacon_1_y));
	   float d23 = sqrt((beacon_3_x - beacon_2_x)*(beacon_3_x - beacon_2_x) + (beacon_3_y - beacon_2_y)*(beacon_3_y - beacon_2_y));

	   float lambda12 = real_beacon_2_angle - real_beacon_1_angle;
	   float lambda23 = real_beacon_3_angle - real_beacon_2_angle;

	   float mu = sin(lambda12) / d12;
	   float eta = sin(lambda23) / d23;

	   float phi = atan2((beacon_1_y - beacon_2_y), (beacon_1_x - beacon_2_x));

	   float beta = phi - atan2((beacon_3_y - beacon_2_y), (beacon_3_x - beacon_2_x));

	   float r2 = fabs(sin(beta + lambda12 + lambda23)) / sqrt(mu*mu + eta * eta + 2 * mu*eta*cos(beta + lambda12 + lambda23));

	   float theta12 = asin(mu*r2);

	   float beta12 = M_PI - lambda12 - theta12;

	   cvs->myTriangulation->x = beacon_2_x + r2 * cos(phi - beta12); //- 0.083*cos(myOdometry->theta);
	   cvs->myTriangulation->y = beacon_2_y + r2 * sin(phi - beta12) - 0.083*sin(myOdometry->theta);

   }
   else if (number_beacon == 2) {
	   measured_beacon_1_angle = limit_angle(cvs->rob_pos->theta + beacon_1_angle_theta);
	   measured_beacon_2_angle = limit_angle(cvs->rob_pos->theta + beacon_2_angle_theta);


	   // localization
	   beacon_index_1 = beacon_identifier(measured_beacon_1_angle, theorical_beacon_1_angle, theorical_beacon_2_angle, theorical_beacon_3_angle); // beacon_index_1 contain the number of the beacon to which correspond identified_beacon_1_angle
	   if (beacon_index_1 == 1) // mean that distance to detected beacon 1 is the distance to beacon 1
	   {
		   double x1 = beacon_1_x - distance_1 * cos(measured_beacon_1_angle);
		   double x2 = beacon_2_x - distance_2 * cos(measured_beacon_2_angle);
		   cvs->myTriangulation->x = (x1 + x2) / 2.0; // take the mean

		   double y1 = beacon_1_y - distance_1 * sin(measured_beacon_1_angle);
		   double y2 = beacon_2_y - distance_2 * sin(measured_beacon_2_angle);
		   cvs->myTriangulation->y = (y1 + y2) / 2.0; // take the mean

		}
	   if (beacon_index_1 == 2) // mean that distance to detected beacon 1 is the distance to beacon 2
	   {
		   double x1 = beacon_1_x - distance_2 * cos(measured_beacon_2_angle);
		   double x2 = beacon_2_x - distance_3 * cos(measured_beacon_3_angle);
		   cvs->myTriangulation->x = (x1 + x2) / 2.0; // take the mean

		   double y1 = beacon_1_y - distance_2 * sin(measured_beacon_2_angle);
		   double y2 = beacon_2_y - distance_3 * sin(measured_beacon_3_angle);
		   cvs->myTriangulation->y = (y1 + y2) / 2.0; // take the mean
	   }
	   if (beacon_index_1 == 3) // mean that distance to detected beacon  1 is the distance to beacon 3
	   {
		   double x1 = beacon_1_x - distance_3 * cos(measured_beacon_3_angle);
		   double x2 = beacon_2_x - distance_1 * cos(measured_beacon_1_angle);
		   cvs->myTriangulation->x = (x1 + x2) / 2.0; // take the mean

		   double y1 = beacon_1_y - distance_3 * sin(measured_beacon_2_angle);
		   double y2 = beacon_2_y - distance_1 * sin(measured_beacon_1_angle);
		   cvs->myTriangulation->y = (y1 + y2) / 2.0; // take the mean
	   }
   }
   else if (number_beacon == 1) 
		{

	   
	   // localization
	   beacon_index_1 = beacon_identifier(measured_beacon_1_angle, theorical_beacon_1_angle, theorical_beacon_2_angle, theorical_beacon_3_angle); // beacon_index_1 contain the number of the beacon to which correspond identified_beacon_1_angle
	   if (beacon_index_1 == 1) // mean that distance to detected beacon 1 is the distance to beacon 1
	   {
		   cvs->myTriangulation->x = beacon_1_x - distance_1 * cos(measured_beacon_1_angle);
		   cvs->myTriangulation->y = beacon_1_y - distance_1 * sin(measured_beacon_1_angle);
	   }
	   if (beacon_index_1 == 2) // mean that distance to detected beacon 1 is the distance to beacon 2
	   {
		   cvs->myTriangulation->x = beacon_2_x - distance_2 * cos(measured_beacon_2_angle);
		   cvs->myTriangulation->y = beacon_2_y - distance_2 * sin(measured_beacon_2_angle);
	   }
	   if (beacon_index_1 == 3) // mean that distance to detected beacon  1 is the distance to beacon 3
	   {
		   cvs->myTriangulation->x = beacon_3_x - distance_3 * cos(measured_beacon_3_angle);
		   cvs->myTriangulation->y = beacon_3_y - distance_3 * sin(measured_beacon_3_angle);
	   }
	   
   }
   else if (number_beacon == -1) {

   }


}





//////////////////////////////////////////////// Needed Functions /////////////////////////////////////////////


int beacon_identifier(double angle, double beacon_1_angle, double beacon_2_angle, double beacon_3_angle){
	if (fabs(angle - beacon_1_angle) < 0.451) { //15 degrees = 0.261799 7.5 degrees = 0.13089
		return 1;
	}
	else if (fabs(angle - beacon_2_angle) < 0.451) { //15 degrees = 0.261799 7.5 degrees = 0.13089
		return 2;
	}
	else if (fabs(angle - beacon_3_angle) < 0.451) { //15 degrees = 0.261799 7.5 degrees = 0.13089
		return 3;
	}
	else {
		return -1;
	}
}
NAMESPACE_CLOSE();





