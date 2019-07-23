#ifndef _TRIANGULATION_GR9_H_
#define _TRIANGULATION_GR9_H_ 

#include "CtrlStruct_gr9.h"
#include "namespace_ctrl.h"

NAMESPACE_INIT(ctrlGr9);

void update_triangulation(CtrlStruct *cvs);

typedef struct Triangulation
{
    double x; /// x position of the robot measured by triangulation
    double y; /// y position of the robot measured by the triangulation
    double theta; /// angular position of the robot measured by the triangulation

	double beacon_1_x; // used to store the postion of all beacons
	double beacon_1_y;
	double beacon_2_x;
	double beacon_2_y;
	double beacon_3_x;
	double beacon_3_y;

	double theorical_beacon_1_angle; // theorical angle between x axis and beacon
	double theorical_beacon_2_angle;
	double theorical_beacon_3_angle;

	double measured_beacon_1_angle; // will be used to compare to the practical one to identify which angle correspond to which beacon
	double measured_beacon_2_angle;
	double measured_beacon_3_angle;

	double last_rising_1; // most recent rising edge detected
	double last_rising_2;
	double last_rising_3;

	double last_falling_1;  // most recent falling edge detected
	double last_falling_2;
	double last_falling_3;

	double beacon_1_angle_theta; // angle between robot directoin and the beacon
	double beacon_2_angle_theta;
	double beacon_3_angle_theta;

	int beacon_index_1; // will be used to stock the number of the beacon corresponding to the first detection
	double R_beacon; // beacon radius
	int number_beacon; // to stock the number of beacon(s) that will be used for triangulation

	// used variables if 2 detected beacons
	float distance_1;
	float distance_2;
	float distance_3;

	// variables if 3 detected beacons
	double real_beacon_1_angle;
	double real_beacon_2_angle;
	double real_beacon_3_angle;

} Triangulation;

// functions
int beacon_identifier(double angle, double beacon_1_angle, double beacon_2_angle, double beacon_3_angle);

NAMESPACE_CLOSE();

#endif

