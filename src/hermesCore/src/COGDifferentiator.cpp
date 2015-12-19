/*
 * COGDifferentiator.cpp
 *
 *  Created on: Nov 17, 2015
 *      Author: perry
 */

#include "COGDifferentiator.h"
#include "SL_user.h"

COG_Differentiator::COG_Differentiator() {
	for (int i = 0; i < 3; i++){
		cog_dot_filter[i].cutoff = 100;
		for (int j = 0; j< 3; j++){
			cog_dot_filter[i].filt[j] = 0;
			cog_dot_filter[i].raw[j] = 0;
		}
	}

}

COG_Differentiator::~COG_Differentiator() {

}

void COG_Differentiator::apply_cog_diff(SL_Cstate* cog){

	for(int i = 0; i < 3; i++){
		cog->xd[i+1] = filt((cog->x[i+1] - prev_cog.x[i + 1])*0.004, &cog_dot_filter[i]);
	}

	prev_cog = *cog;
}

/////////////// C interface //////////
COG_Differentiator diff;
extern "C"
{
void initialize_cog_differentiator();
void differentiate_cog(SL_Cstate* cog);
} // extern "C"

void initialize_cog_differentiator()
{

}

void differentiate_cog(SL_Cstate * cog)
{
	diff.apply_cog_diff(cog);
}

