

#ifndef _ODOMETRY_GR9_H_
#define _ODOMETRY_GR9_H_ 

#include "namespace_ctrl.h"
#include "CtrlStruct_gr9.h"

NAMESPACE_INIT(ctrlGr9);

// function prototype
void update_odometry(CtrlStruct *cvs);

typedef struct Odometry
{
    double x;
    double y;
    double theta;
    double delta_theta;
    double temporal_theta;  
    double delta_s;
    double delta_x;
    double delta_y;
    double speed;
    double last_t;
	double d; 
	double R;

	
    
} Odometry;

NAMESPACE_CLOSE();

#endif
