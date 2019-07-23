#include "limit_angle_gr9.h"
#include <math.h>
#include "strategy_gr9.h"
#include "path_planning_gr9.h"
#include "speed_regulation_gr9.h"
#include "odometry_gr9.h"
#include "init_pos_gr9.h"

NAMESPACE_INIT(ctrlGr9);

/*! \brief set an angle in the in [-pi;pi[ range
 * 
 * \param[in] x angle to limit
 * \return angle limited in ]-pi;pi]
 */
double limit_angle(double x)
{
	if (x < -M_PI)
	{
		// printf("angle = %lf, angle +2pi \n", x);
		x += 2.0*M_PI;
	}
	else if (x >= M_PI)
	{
		// printf("angle -2pi \n");
		x -= 2.0*M_PI;
	}

	return x;
}

NAMESPACE_CLOSE();
