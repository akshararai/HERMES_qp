/*
 * FallDetector.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: perry
 */

#include "FallDetector.h"
#include <math.h>
#include "SL.h"
#include "SL_user.h"
#include <iostream>

FallDetector::FallDetector(int debounce_count):debounce_count(20) {
	current_count = 0;
}

FallDetector::~FallDetector() {
	// TODO Auto-generated destructor stub
}

int FallDetector::test_fall(SL_Cstate* cog_)
{
	int retval = FALSE;
	int i;

	//Calculate capture point, based on x_cap = x_current + x_dot*(sqrt (z0 / g))
	double x_capture[4];
	for (i = 1; i <=2; i++){
		x_capture[i] = cog_->x[i] + cog_->xd[i]*sqrt(abs(cog_->x[_Z_]/ gravity));
	}

	static bool printed = false;

	//Check if capturepoint is beyond the toe, assuming alignment of y axis
//	if (x_capture[_Y_] > cart_state[LEFT_FOOT].x[_Y_]+YTOE){
	if (!check_support_polygon(x_capture)){
		current_count++;
		if (!printed){
			printf("\npositive %i...\n", current_count);
		}
		//If we've had enough consecutive alerts, assume we're falling
		if (current_count >= debounce_count){
			current_count = debounce_count; //keep from overflowing
			retval = TRUE;
			if (!printed){
				printf("\nFALLING!!!! \n");
				printed = true;
			}
		}

	} else {
		current_count = 0;
		printed = false;
	}

	cog_->xdd[1] = current_count;


	static int printcounter =100;
	printcounter ++;
	if (printcounter > 100){
//		printf("y_edge = %f, y_capture = %f \n", cart_state[LEFT_FOOT].x[_Y_]+YTOE ,x_capture[_Y_]);
		printcounter = 0;
		if(retval){
			printf("FALLING...");
		}
	}



  return retval;
}

bool FallDetector::check_support_polygon(double capturePoint[4]){
	return (check_support_vector(capturePoint, link_pos[L_OUT_TOE], link_pos[R_OUT_TOE]) &&
			check_support_vector(capturePoint, link_pos[R_OUT_TOE], link_pos[R_OUT_HEEL]) &&
			check_support_vector(capturePoint, link_pos[R_OUT_HEEL], link_pos[L_OUT_HEEL]) &&
			check_support_vector(capturePoint, link_pos[L_OUT_HEEL], link_pos[L_OUT_TOE])    );
}

bool FallDetector::check_support_vector(double capturePoint[4], double ccw_point[4], double cw_point[4]){
	//Check if capturePoint is on the correct side of the line defined by ccw_point and cw_point
	//Named based on order from desired side of line
	//So if want to be behind the toes, ccw_point = left toe and cw_point = right toe

	//Based on line equation { 0 = (x1 - x2)(x - x1) - (y1 - y2)(y - y1)}

	double a = (ccw_point[_Y_] - cw_point[_Y_]) * (capturePoint[_X_] - ccw_point[_X_])
			-(ccw_point[_X_] - cw_point[_X_]) * (capturePoint[_Y_] - ccw_point[_Y_]);

	if (a < 0){
		return true;
	}
	else{
		return false;
	}


}

/////////////// C interface //////////
FallDetector fd;
extern "C"
{
void initialize_fall_detector();
void test_fall(SL_Cstate* cog_);
} // extern "C"

void initialize_fall_detector()
{

}

void test_fall(SL_Cstate* cog_)
{
	fd.test_fall(cog_);
}


