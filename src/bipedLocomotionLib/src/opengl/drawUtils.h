/*
 * drawUtils.h
 *
 *  Created on: May 1, 2013
 *      Author: righetti
 */

#ifndef DRAWUTILS_H_
#define DRAWUTILS_H_


struct BallsInfo
{
  int numBalls;
  double balls[5][3+4];//maximum of 5 balls can be drawn
};

struct AxesInfo
{
  double length;
  double width;
  double transform[(4+1)*(4+1)];
  char name[20];
};

#ifdef __cplusplus
extern "C" {
#endif

extern void drawBall(void *buf);
extern void drawAxes(void *buf);

#ifdef __cplusplus
}
#endif

#endif /* DRAWUTILS_H_ */
