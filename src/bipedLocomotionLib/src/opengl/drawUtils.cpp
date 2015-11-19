/*
 * drawUtils.cpp
 *
 *  Created on: May 1, 2013
 *      Author: righetti
 */

#include "GL/freeglut_std.h"
#include "GL/freeglut_ext.h"
#include "GL/glu.h"
#include <X11/Xlib.h>


#include <SL.h>
#include <SL_openGL.h>

#include "utility_macros.h"

#include "drawUtils.h"

void drawBall(void *buf)
{
  BallsInfo &data = *(struct BallsInfo*)buf;


  for(int i=0; i<data.numBalls; ++i)
  {

    GLfloat color_point[4]={float(data.balls[i][3]),float(data.balls[i][4]),float(data.balls[i][5]),float(data.balls[i][6])};
    double radius = 0.04;

    glPushMatrix();
    glTranslated((GLdouble)data.balls[i][0],(GLdouble)data.balls[i][1],(GLdouble)data.balls[i][2]);
    glColor4fv(color_point);
    if (solid)
      glutSolidSphere(radius,10,10);
    else
      glutWireSphere(radius,10,10);
    glPopMatrix();
  }
}


void drawAxes(void *buf)
{

  int    i,j;
  double v[N_CART+1+1];
  double r[N_CART+1+1];
  double s[N_CART+1+1];
  double arrow_width = 0.005;

  AxesInfo &data = *(struct AxesInfo*)buf;

  double length = data.length;

  MY_MATRIX(A, 1, 4, 1, 4);
  for(int i=0; i<4; ++i)
    for(int j=0; j<4; ++j)
      A[i+1][j+1] = data.transform[4*i+1+j];

  // draw the coordinate systems
  glPushMatrix();
  glLineWidth(2.0);

  v[_X_] = length;
  v[_Y_] = v[_Z_] = 0.0;
  v[_Z_+1] = 1.0;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  for (i=1; i<=N_CART; ++i)
    s[i] = A[i][4];

  glColor4f (1.0,0.0,0.0,0.0);
  drawArrow(s,r,arrow_width);

  v[_X_] = length+0.1;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  glRasterPos3f(r[_X_],r[_Y_],r[_Z_]);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,'X');

  v[_Y_] = length;
  v[_X_] = v[_Z_] = 0.0;
  v[_Z_+1] = 1.0;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  for (i=1; i<=N_CART; ++i)
    s[i] = A[i][4];

  glColor4f (0.0,1.0,0.0,0.0);
  drawArrow(s,r,arrow_width);

  v[_Y_] = length+0.1;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  glRasterPos3f(r[_X_],r[_Y_],r[_Z_]);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,'Y');

  v[_Z_] = length;
  v[_Y_] = v[_X_] = 0.0;
  v[_Z_+1] = 1.0;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  for (i=1; i<=N_CART; ++i)
    s[i] = A[i][4];

  glColor4f (0.0,0.0,1.0,0.0);
  drawArrow(s,r,arrow_width);

  v[_Z_] = length+0.1;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  glRasterPos3f(r[_X_],r[_Y_],r[_Z_]);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,'Z');

  glColor4f (0.0,0.0,0.0,0.0);
  glBegin(GL_LINES);
  glVertex3d(A[_X_][4],A[_Y_][4],A[_Z_][4]);
  glVertex3d(A[_X_][4],A[_Y_][4],A[_Z_][4]+0.1);
  glEnd();
  glRasterPos3f(A[_X_][4],A[_Y_][4],A[_Z_][4]+0.1);
  glutBitmapString(GLUT_BITMAP_HELVETICA_18,(const unsigned char *)data.name);

  glLineWidth(1.0);
  glPopMatrix();

}
