/*
 * FallDetector.h
 *
 *  Created on: Nov 18, 2015
 *      Author: perry
 */

#ifndef FALLDETECTOR_H_
#define FALLDETECTOR_H_

#include "SL.h"

class FallDetector {
public:
	FallDetector(int debounce_count = 20);
	virtual ~FallDetector();
	int test_fall(SL_Cstate *cog_);

protected:
	int debounce_count;
	int current_count;

	bool check_support_polygon(double capturePoint[4]);

	bool check_support_vector(double capturePoint[4], double ccw_point[4], double cw_point[4]);

};

#endif /* FALLDETECTOR_H_ */
