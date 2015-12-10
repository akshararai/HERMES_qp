/*
 * COGDifferentiator.h
 *
 *  Created on: Nov 17, 2015
 *      Author: perry
 */

#ifndef COGDIFFERENTIATOR_H_
#define COGDIFFERENTIATOR_H_

#include "SL.h"
#include "SL_filters.h"

class COG_Differentiator {
public:
	COG_Differentiator();
	virtual ~COG_Differentiator();

	void apply_cog_diff(SL_Cstate *cog);

private:

	Filter cog_dot_filter[3];
	SL_Cstate prev_cog;

};

#endif /* COGDIFFERENTIATOR_H_ */
