/* this function generates simple OpenGL graphics code to draw each link */


glPushMatrix();
myDrawGLElement((int)999,(double)0.0,(int)1);
glPopMatrix();

/* JointID = 0 */

glPushMatrix();
glPushMatrix();
if (basec[0].x[1]==0 && basec[0].x[2]==0)
glRotated((GLdouble)90.*(-1. + basec[0].x[3]/Sqrt(Power(basec[0].x[1],2) + Power(basec[0].x[2],2) + Power(basec[0].x[3],2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*basec[0].x[1],(GLdouble)0.5*basec[0].x[2],(GLdouble)0.5*(basec[0].x[3] + Sqrt(Power(basec[0].x[1],2) + Power(basec[0].x[2],2) + Power(basec[0].x[3],2))));
myDrawGLElement((int)0,(double)Sqrt(Power(basec[0].x[1],2) + Power(basec[0].x[2],2) + Power(basec[0].x[3],2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)basec[0].x[1],(GLdouble)basec[0].x[2],(GLdouble)basec[0].x[3]);
glRotated((GLdouble)114.59155902616465*ArcCos(baseo[0].q[1]),(GLdouble)baseo[0].q[2],(GLdouble)baseo[0].q[3],(GLdouble)baseo[0].q[4]);

/* JointID = 29 */

glPushMatrix();
glPushMatrix();
if (0==0 && -PELVISOFFSET==0)
glRotated((GLdouble)90.*(-1. + PELVIS2THORAX/Sqrt(Power(PELVIS2THORAX,2) + Power(PELVISOFFSET,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)-0.5*PELVISOFFSET,(GLdouble)0.5*(PELVIS2THORAX + Sqrt(Power(PELVIS2THORAX,2) + Power(PELVISOFFSET,2))));
myDrawGLElement((int)29,(double)Sqrt(Power(PELVIS2THORAX,2) + Power(PELVISOFFSET,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)-PELVISOFFSET,(GLdouble)PELVIS2THORAX);
glRotated((GLdouble)180.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[29].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 30 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)30,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[30].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 31 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)31,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[31].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 1 */

glPushMatrix();
glPushMatrix();
if (-THORAX2SHOULDER==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*THORAX2SHOULDER,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(THORAX2SHOULDER,2)));
myDrawGLElement((int)1,(double)Sqrt(Power(THORAX2SHOULDER,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-THORAX2SHOULDER,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)180.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)45.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[1].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 2 */

glPushMatrix();
glPushMatrix();
if (0==0 && 0==0)
glRotated((GLdouble)90.*(-1. - SHOULDERX/Sqrt(Power(SHOULDERX,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)0,(GLdouble)0.5*(-SHOULDERX + Sqrt(Power(SHOULDERX,2))));
myDrawGLElement((int)2,(double)Sqrt(Power(SHOULDERX,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)0,(GLdouble)-SHOULDERX);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)135.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[2].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 3 */

glPushMatrix();
glPushMatrix();
if (-SHOULDERY==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*SHOULDERY,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(SHOULDERY,2)));
myDrawGLElement((int)3,(double)Sqrt(Power(SHOULDERY,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-SHOULDERY,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[3].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 4 */

glPushMatrix();
glPushMatrix();
if (0==0 && 0==0)
glRotated((GLdouble)90.*(-1. - UPPERARM/Sqrt(Power(UPPERARM,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)0,(GLdouble)0.5*(-UPPERARM + Sqrt(Power(UPPERARM,2))));
myDrawGLElement((int)4,(double)Sqrt(Power(UPPERARM,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)0,(GLdouble)-UPPERARM);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[4].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 5 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)5,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[5].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 6 */

glPushMatrix();
glPushMatrix();
if (0==0 && WRISTY==0)
glRotated((GLdouble)90.*(-1. - LOWERARM/Sqrt(Power(LOWERARM,2) + Power(WRISTY,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)0.5*WRISTY,(GLdouble)0.5*(-LOWERARM + Sqrt(Power(LOWERARM,2) + Power(WRISTY,2))));
myDrawGLElement((int)6,(double)Sqrt(Power(LOWERARM,2) + Power(WRISTY,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)WRISTY,(GLdouble)-LOWERARM);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[6].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 7 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)7,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[7].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 108 */

glPushMatrix();
glPushMatrix();
if (eff[2].x[1]==0 && eff[2].x[2]==0)
glRotated((GLdouble)90.*(-1. + eff[2].x[3]/Sqrt(Power(eff[2].x[1],2) + Power(eff[2].x[2],2) + Power(eff[2].x[3],2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*eff[2].x[1],(GLdouble)0.5*eff[2].x[2],(GLdouble)0.5*(eff[2].x[3] + Sqrt(Power(eff[2].x[1],2) + Power(eff[2].x[2],2) + Power(eff[2].x[3],2))));
myDrawGLElement((int)108,(double)Sqrt(Power(eff[2].x[1],2) + Power(eff[2].x[2],2) + Power(eff[2].x[3],2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 39 */

glPushMatrix();
glPushMatrix();
if (XTHUMB==0 && YTHUMB==0)
glRotated((GLdouble)90.*(-1. - ZTHUMB/Sqrt(Power(XTHUMB,2) + Power(YTHUMB,2) + Power(ZTHUMB,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XTHUMB,(GLdouble)0.5*YTHUMB,(GLdouble)0.5*(-ZTHUMB + Sqrt(Power(XTHUMB,2) + Power(YTHUMB,2) + Power(ZTHUMB,2))));
myDrawGLElement((int)39,(double)Sqrt(Power(XTHUMB,2) + Power(YTHUMB,2) + Power(ZTHUMB,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XTHUMB,(GLdouble)YTHUMB,(GLdouble)-ZTHUMB);
glRotated((GLdouble)57.29577951308232*HANDXROT,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*THUMBZROT,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[39].th,(GLdouble)1,(GLdouble)0,(GLdouble)0);

/* JointID = 40 */

glPushMatrix();
glPushMatrix();
if (XTHUMBFLEX==0 && YTHUMBFLEX==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XTHUMBFLEX,(GLdouble)0.5*YTHUMBFLEX,(GLdouble)0.5*Sqrt(Power(XTHUMBFLEX,2) + Power(YTHUMBFLEX,2)));
myDrawGLElement((int)40,(double)Sqrt(Power(XTHUMBFLEX,2) + Power(YTHUMBFLEX,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XTHUMBFLEX,(GLdouble)YTHUMBFLEX,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*THUMBANGLEOFF,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[40].th,(GLdouble)0,(GLdouble)0,(GLdouble)-1);

/* JointID = 165 */

glPushMatrix();
glPushMatrix();
if (TH1SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*TH1SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(TH1SEG,2)));
myDrawGLElement((int)165,(double)Sqrt(Power(TH1SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)TH1SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)-57.29577951308232*state[40].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 166 */

glPushMatrix();
glPushMatrix();
if (TH2SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*TH2SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(TH2SEG,2)));
myDrawGLElement((int)166,(double)Sqrt(Power(TH2SEG,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 41 */

glPushMatrix();
glPushMatrix();
if (XIF==0 && YIF==0)
glRotated((GLdouble)90.*(-1. - ZIF/Sqrt(Power(XIF,2) + Power(YIF,2) + Power(ZIF,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XIF,(GLdouble)0.5*YIF,(GLdouble)0.5*(-ZIF + Sqrt(Power(XIF,2) + Power(YIF,2) + Power(ZIF,2))));
myDrawGLElement((int)41,(double)Sqrt(Power(XIF,2) + Power(YIF,2) + Power(ZIF,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XIF,(GLdouble)YIF,(GLdouble)-ZIF);
glRotated((GLdouble)57.29577951308232*HANDXROT,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[41].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 167 */

glPushMatrix();
glPushMatrix();
if (F1SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F1SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F1SEG,2)));
myDrawGLElement((int)167,(double)Sqrt(Power(F1SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F1SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[41].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 168 */

glPushMatrix();
glPushMatrix();
if (F2SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F2SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F2SEG,2)));
myDrawGLElement((int)168,(double)Sqrt(Power(F2SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F2SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[41].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 169 */

glPushMatrix();
glPushMatrix();
if (F3SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F3SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F3SEG,2)));
myDrawGLElement((int)169,(double)Sqrt(Power(F3SEG,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 42 */

glPushMatrix();
glPushMatrix();
if (XMF==0 && YMF==0)
glRotated((GLdouble)90.*(-1. - ZMF/Sqrt(Power(XMF,2) + Power(YMF,2) + Power(ZMF,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XMF,(GLdouble)0.5*YMF,(GLdouble)0.5*(-ZMF + Sqrt(Power(XMF,2) + Power(YMF,2) + Power(ZMF,2))));
myDrawGLElement((int)42,(double)Sqrt(Power(XMF,2) + Power(YMF,2) + Power(ZMF,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XMF,(GLdouble)YMF,(GLdouble)-ZMF);
glRotated((GLdouble)57.29577951308232*HANDXROT,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[42].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 170 */

glPushMatrix();
glPushMatrix();
if (F1SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F1SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F1SEG,2)));
myDrawGLElement((int)170,(double)Sqrt(Power(F1SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F1SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[42].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 171 */

glPushMatrix();
glPushMatrix();
if (F2SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F2SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F2SEG,2)));
myDrawGLElement((int)171,(double)Sqrt(Power(F2SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F2SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[42].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 172 */

glPushMatrix();
glPushMatrix();
if (F3SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F3SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F3SEG,2)));
myDrawGLElement((int)172,(double)Sqrt(Power(F3SEG,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 43 */

glPushMatrix();
glPushMatrix();
if (XRF==0 && YRF==0)
glRotated((GLdouble)90.*(-1. - ZRF/Sqrt(Power(XRF,2) + Power(YRF,2) + Power(ZRF,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XRF,(GLdouble)0.5*YRF,(GLdouble)0.5*(-ZRF + Sqrt(Power(XRF,2) + Power(YRF,2) + Power(ZRF,2))));
myDrawGLElement((int)43,(double)Sqrt(Power(XRF,2) + Power(YRF,2) + Power(ZRF,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XRF,(GLdouble)YRF,(GLdouble)-ZRF);
glRotated((GLdouble)57.29577951308232*HANDXROT,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[43].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 173 */

glPushMatrix();
glPushMatrix();
if (F1SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F1SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F1SEG,2)));
myDrawGLElement((int)173,(double)Sqrt(Power(F1SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F1SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[43].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 174 */

glPushMatrix();
glPushMatrix();
if (F2SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F2SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F2SEG,2)));
myDrawGLElement((int)174,(double)Sqrt(Power(F2SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F2SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[43].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 175 */

glPushMatrix();
glPushMatrix();
if (F3SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F3SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F3SEG,2)));
myDrawGLElement((int)175,(double)Sqrt(Power(F3SEG,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 44 */

glPushMatrix();
glPushMatrix();
if (XLF==0 && YLF==0)
glRotated((GLdouble)90.*(-1. - ZLF/Sqrt(Power(XLF,2) + Power(YLF,2) + Power(ZLF,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XLF,(GLdouble)0.5*YLF,(GLdouble)0.5*(-ZLF + Sqrt(Power(XLF,2) + Power(YLF,2) + Power(ZLF,2))));
myDrawGLElement((int)44,(double)Sqrt(Power(XLF,2) + Power(YLF,2) + Power(ZLF,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XLF,(GLdouble)YLF,(GLdouble)-ZLF);
glRotated((GLdouble)57.29577951308232*HANDXROT,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[44].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 176 */

glPushMatrix();
glPushMatrix();
if (F1SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F1SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F1SEG,2)));
myDrawGLElement((int)176,(double)Sqrt(Power(F1SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F1SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[44].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 177 */

glPushMatrix();
glPushMatrix();
if (F2SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F2SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F2SEG,2)));
myDrawGLElement((int)177,(double)Sqrt(Power(F2SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F2SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[44].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 178 */

glPushMatrix();
glPushMatrix();
if (F3SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F3SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F3SEG,2)));
myDrawGLElement((int)178,(double)Sqrt(Power(F3SEG,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 8 */

glPushMatrix();
glPushMatrix();
if (-THORAX2SHOULDER==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*THORAX2SHOULDER,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(THORAX2SHOULDER,2)));
myDrawGLElement((int)8,(double)Sqrt(Power(THORAX2SHOULDER,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-THORAX2SHOULDER,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)180.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)-45.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[8].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 9 */

glPushMatrix();
glPushMatrix();
if (0==0 && 0==0)
glRotated((GLdouble)90.*(-1. + SHOULDERX/Sqrt(Power(SHOULDERX,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)0,(GLdouble)0.5*(SHOULDERX + Sqrt(Power(SHOULDERX,2))));
myDrawGLElement((int)9,(double)Sqrt(Power(SHOULDERX,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)0,(GLdouble)SHOULDERX);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)135.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[9].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 10 */

glPushMatrix();
glPushMatrix();
if (-SHOULDERY==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*SHOULDERY,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(SHOULDERY,2)));
myDrawGLElement((int)10,(double)Sqrt(Power(SHOULDERY,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-SHOULDERY,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[10].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 11 */

glPushMatrix();
glPushMatrix();
if (0==0 && 0==0)
glRotated((GLdouble)90.*(-1. + UPPERARM/Sqrt(Power(UPPERARM,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)0,(GLdouble)0.5*(UPPERARM + Sqrt(Power(UPPERARM,2))));
myDrawGLElement((int)11,(double)Sqrt(Power(UPPERARM,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)0,(GLdouble)UPPERARM);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[11].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 12 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)12,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[12].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 13 */

glPushMatrix();
glPushMatrix();
if (0==0 && WRISTY==0)
glRotated((GLdouble)90.*(-1. + LOWERARM/Sqrt(Power(LOWERARM,2) + Power(WRISTY,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)0.5*WRISTY,(GLdouble)0.5*(LOWERARM + Sqrt(Power(LOWERARM,2) + Power(WRISTY,2))));
myDrawGLElement((int)13,(double)Sqrt(Power(LOWERARM,2) + Power(WRISTY,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)WRISTY,(GLdouble)LOWERARM);
glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[13].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 14 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)14,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[14].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 115 */

glPushMatrix();
glPushMatrix();
if (eff[1].x[1]==0 && eff[1].x[2]==0)
glRotated((GLdouble)90.*(-1. + eff[1].x[3]/Sqrt(Power(eff[1].x[1],2) + Power(eff[1].x[2],2) + Power(eff[1].x[3],2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*eff[1].x[1],(GLdouble)0.5*eff[1].x[2],(GLdouble)0.5*(eff[1].x[3] + Sqrt(Power(eff[1].x[1],2) + Power(eff[1].x[2],2) + Power(eff[1].x[3],2))));
myDrawGLElement((int)115,(double)Sqrt(Power(eff[1].x[1],2) + Power(eff[1].x[2],2) + Power(eff[1].x[3],2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 45 */

glPushMatrix();
glPushMatrix();
if (XTHUMB==0 && YTHUMB==0)
glRotated((GLdouble)90.*(-1. + ZTHUMB/Sqrt(Power(XTHUMB,2) + Power(YTHUMB,2) + Power(ZTHUMB,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XTHUMB,(GLdouble)0.5*YTHUMB,(GLdouble)0.5*(ZTHUMB + Sqrt(Power(XTHUMB,2) + Power(YTHUMB,2) + Power(ZTHUMB,2))));
myDrawGLElement((int)45,(double)Sqrt(Power(XTHUMB,2) + Power(YTHUMB,2) + Power(ZTHUMB,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XTHUMB,(GLdouble)YTHUMB,(GLdouble)ZTHUMB);
glRotated((GLdouble)-57.29577951308232*HANDXROT,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*THUMBZROT,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[45].th,(GLdouble)1,(GLdouble)0,(GLdouble)0);

/* JointID = 46 */

glPushMatrix();
glPushMatrix();
if (XTHUMBFLEX==0 && YTHUMBFLEX==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XTHUMBFLEX,(GLdouble)0.5*YTHUMBFLEX,(GLdouble)0.5*Sqrt(Power(XTHUMBFLEX,2) + Power(YTHUMBFLEX,2)));
myDrawGLElement((int)46,(double)Sqrt(Power(XTHUMBFLEX,2) + Power(YTHUMBFLEX,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XTHUMBFLEX,(GLdouble)YTHUMBFLEX,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*THUMBANGLEOFF,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[46].th,(GLdouble)0,(GLdouble)0,(GLdouble)-1);

/* JointID = 151 */

glPushMatrix();
glPushMatrix();
if (TH1SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*TH1SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(TH1SEG,2)));
myDrawGLElement((int)151,(double)Sqrt(Power(TH1SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)TH1SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)-57.29577951308232*state[46].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 152 */

glPushMatrix();
glPushMatrix();
if (TH2SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*TH2SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(TH2SEG,2)));
myDrawGLElement((int)152,(double)Sqrt(Power(TH2SEG,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 47 */

glPushMatrix();
glPushMatrix();
if (XIF==0 && YIF==0)
glRotated((GLdouble)90.*(-1. + ZIF/Sqrt(Power(XIF,2) + Power(YIF,2) + Power(ZIF,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XIF,(GLdouble)0.5*YIF,(GLdouble)0.5*(ZIF + Sqrt(Power(XIF,2) + Power(YIF,2) + Power(ZIF,2))));
myDrawGLElement((int)47,(double)Sqrt(Power(XIF,2) + Power(YIF,2) + Power(ZIF,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XIF,(GLdouble)YIF,(GLdouble)ZIF);
glRotated((GLdouble)-57.29577951308232*HANDXROT,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[47].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 153 */

glPushMatrix();
glPushMatrix();
if (F1SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F1SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F1SEG,2)));
myDrawGLElement((int)153,(double)Sqrt(Power(F1SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F1SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[47].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 154 */

glPushMatrix();
glPushMatrix();
if (F2SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F2SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F2SEG,2)));
myDrawGLElement((int)154,(double)Sqrt(Power(F2SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F2SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[47].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 155 */

glPushMatrix();
glPushMatrix();
if (F3SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F3SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F3SEG,2)));
myDrawGLElement((int)155,(double)Sqrt(Power(F3SEG,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 48 */

glPushMatrix();
glPushMatrix();
if (XMF==0 && YMF==0)
glRotated((GLdouble)90.*(-1. + ZMF/Sqrt(Power(XMF,2) + Power(YMF,2) + Power(ZMF,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XMF,(GLdouble)0.5*YMF,(GLdouble)0.5*(ZMF + Sqrt(Power(XMF,2) + Power(YMF,2) + Power(ZMF,2))));
myDrawGLElement((int)48,(double)Sqrt(Power(XMF,2) + Power(YMF,2) + Power(ZMF,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XMF,(GLdouble)YMF,(GLdouble)ZMF);
glRotated((GLdouble)-57.29577951308232*HANDXROT,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[48].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 156 */

glPushMatrix();
glPushMatrix();
if (F1SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F1SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F1SEG,2)));
myDrawGLElement((int)156,(double)Sqrt(Power(F1SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F1SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[48].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 157 */

glPushMatrix();
glPushMatrix();
if (F2SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F2SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F2SEG,2)));
myDrawGLElement((int)157,(double)Sqrt(Power(F2SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F2SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[48].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 158 */

glPushMatrix();
glPushMatrix();
if (F3SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F3SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F3SEG,2)));
myDrawGLElement((int)158,(double)Sqrt(Power(F3SEG,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 49 */

glPushMatrix();
glPushMatrix();
if (XRF==0 && YRF==0)
glRotated((GLdouble)90.*(-1. + ZRF/Sqrt(Power(XRF,2) + Power(YRF,2) + Power(ZRF,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XRF,(GLdouble)0.5*YRF,(GLdouble)0.5*(ZRF + Sqrt(Power(XRF,2) + Power(YRF,2) + Power(ZRF,2))));
myDrawGLElement((int)49,(double)Sqrt(Power(XRF,2) + Power(YRF,2) + Power(ZRF,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XRF,(GLdouble)YRF,(GLdouble)ZRF);
glRotated((GLdouble)-57.29577951308232*HANDXROT,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[49].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 159 */

glPushMatrix();
glPushMatrix();
if (F1SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F1SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F1SEG,2)));
myDrawGLElement((int)159,(double)Sqrt(Power(F1SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F1SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[49].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 160 */

glPushMatrix();
glPushMatrix();
if (F2SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F2SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F2SEG,2)));
myDrawGLElement((int)160,(double)Sqrt(Power(F2SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F2SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[49].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 161 */

glPushMatrix();
glPushMatrix();
if (F3SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F3SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F3SEG,2)));
myDrawGLElement((int)161,(double)Sqrt(Power(F3SEG,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 50 */

glPushMatrix();
glPushMatrix();
if (XLF==0 && YLF==0)
glRotated((GLdouble)90.*(-1. + ZLF/Sqrt(Power(XLF,2) + Power(YLF,2) + Power(ZLF,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XLF,(GLdouble)0.5*YLF,(GLdouble)0.5*(ZLF + Sqrt(Power(XLF,2) + Power(YLF,2) + Power(ZLF,2))));
myDrawGLElement((int)50,(double)Sqrt(Power(XLF,2) + Power(YLF,2) + Power(ZLF,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XLF,(GLdouble)YLF,(GLdouble)ZLF);
glRotated((GLdouble)-57.29577951308232*HANDXROT,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[50].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 162 */

glPushMatrix();
glPushMatrix();
if (F1SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F1SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F1SEG,2)));
myDrawGLElement((int)162,(double)Sqrt(Power(F1SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F1SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[50].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 163 */

glPushMatrix();
glPushMatrix();
if (F2SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F2SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F2SEG,2)));
myDrawGLElement((int)163,(double)Sqrt(Power(F2SEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)F2SEG,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[50].th,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 164 */

glPushMatrix();
glPushMatrix();
if (F3SEG==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*F3SEG,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(F3SEG,2)));
myDrawGLElement((int)164,(double)Sqrt(Power(F3SEG,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 32 */

glPushMatrix();
glPushMatrix();
if (-THORAX2NECK==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*THORAX2NECK,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(THORAX2NECK,2)));
myDrawGLElement((int)32,(double)Sqrt(Power(THORAX2NECK,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-THORAX2NECK,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[32].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 33 */

glPushMatrix();
glPushMatrix();
if (0==0 && -CERVICAL==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)-0.5*CERVICAL,(GLdouble)0.5*Sqrt(Power(CERVICAL,2)));
myDrawGLElement((int)33,(double)Sqrt(Power(CERVICAL,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)-CERVICAL,(GLdouble)0);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[33].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 34 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)34,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[34].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 35 */

glPushMatrix();
glPushMatrix();
if (EYEXOFF==0 && -EYEYOFF==0)
glRotated((GLdouble)90.*(-1. - HEAD/Sqrt(Power(EYEXOFF,2) + Power(EYEYOFF,2) + Power(HEAD,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*EYEXOFF,(GLdouble)-0.5*EYEYOFF,(GLdouble)0.5*(-HEAD + Sqrt(Power(EYEXOFF,2) + Power(EYEYOFF,2) + Power(HEAD,2))));
myDrawGLElement((int)35,(double)Sqrt(Power(EYEXOFF,2) + Power(EYEYOFF,2) + Power(HEAD,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)EYEXOFF,(GLdouble)-EYEYOFF,(GLdouble)-HEAD);
glRotated((GLdouble)57.29577951308232*state[35].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 36 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)36,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[36].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 137 */

glPushMatrix();
glPushMatrix();
if (0==0 && -EYE==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)-0.5*EYE,(GLdouble)0.5*Sqrt(Power(EYE,2)));
myDrawGLElement((int)137,(double)Sqrt(Power(EYE,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 37 */

glPushMatrix();
glPushMatrix();
if (-EYEXOFF==0 && -EYEYOFF==0)
glRotated((GLdouble)90.*(-1. - HEAD/Sqrt(Power(EYEXOFF,2) + Power(EYEYOFF,2) + Power(HEAD,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*EYEXOFF,(GLdouble)-0.5*EYEYOFF,(GLdouble)0.5*(-HEAD + Sqrt(Power(EYEXOFF,2) + Power(EYEYOFF,2) + Power(HEAD,2))));
myDrawGLElement((int)37,(double)Sqrt(Power(EYEXOFF,2) + Power(EYEYOFF,2) + Power(HEAD,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-EYEXOFF,(GLdouble)-EYEYOFF,(GLdouble)-HEAD);
glRotated((GLdouble)57.29577951308232*state[37].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 38 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)38,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[38].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 139 */

glPushMatrix();
glPushMatrix();
if (0==0 && -EYE==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)-0.5*EYE,(GLdouble)0.5*Sqrt(Power(EYE,2)));
myDrawGLElement((int)139,(double)Sqrt(Power(EYE,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 138 */

glPushMatrix();
glPushMatrix();
if (0==0 && 0==0)
glRotated((GLdouble)90.*(-1. - TOPofHEAD/Sqrt(Power(TOPofHEAD,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)0,(GLdouble)0.5*(-TOPofHEAD + Sqrt(Power(TOPofHEAD,2))));
myDrawGLElement((int)138,(double)Sqrt(Power(TOPofHEAD,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 23 */

glPushMatrix();
glPushMatrix();
if (XHIP==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*XHIP,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(XHIP,2)));
myDrawGLElement((int)23,(double)Sqrt(Power(XHIP,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)XHIP,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[23].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 22 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)22,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[22].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 24 */

glPushMatrix();
glPushMatrix();
if (YHIP==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*YHIP,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(YHIP,2)));
myDrawGLElement((int)24,(double)Sqrt(Power(YHIP,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)YHIP,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)-57.29577951308232*THIGHANGLEOFF,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[24].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 25 */

glPushMatrix();
glPushMatrix();
if (YKNEE==0 && 0==0)
glRotated((GLdouble)90.*(-1. + UPPERLEGMOD/Sqrt(Power(UPPERLEGMOD,2) + Power(YKNEE,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*YKNEE,(GLdouble)0,(GLdouble)0.5*(UPPERLEGMOD + Sqrt(Power(UPPERLEGMOD,2) + Power(YKNEE,2))));
myDrawGLElement((int)25,(double)Sqrt(Power(UPPERLEGMOD,2) + Power(YKNEE,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)YKNEE,(GLdouble)0,(GLdouble)UPPERLEGMOD);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*THIGHANGLEOFF,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[25].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 26 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)26,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[26].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 27 */

glPushMatrix();
glPushMatrix();
if (0==0 && 0==0)
glRotated((GLdouble)90.*(-1. + LOWERLEG/Sqrt(Power(LOWERLEG,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)0,(GLdouble)0.5*(LOWERLEG + Sqrt(Power(LOWERLEG,2))));
myDrawGLElement((int)27,(double)Sqrt(Power(LOWERLEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)0,(GLdouble)LOWERLEG);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[27].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 28 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)28,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[28].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 129 */

glPushMatrix();
glPushMatrix();
if (ZTOE==0 && -XTOEOUTER==0)
glRotated((GLdouble)90.*(-1. + YTOE/Sqrt(Power(XTOEOUTER,2) + Power(YTOE,2) + Power(ZTOE,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZTOE,(GLdouble)-0.5*XTOEOUTER,(GLdouble)0.5*(YTOE + Sqrt(Power(XTOEOUTER,2) + Power(YTOE,2) + Power(ZTOE,2))));
myDrawGLElement((int)129,(double)Sqrt(Power(XTOEOUTER,2) + Power(YTOE,2) + Power(ZTOE,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 130 */

glPushMatrix();
glPushMatrix();
if (ZTOE==0 && XTOEINNER==0)
glRotated((GLdouble)90.*(-1. + YTOE/Sqrt(Power(XTOEINNER,2) + Power(YTOE,2) + Power(ZTOE,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZTOE,(GLdouble)0.5*XTOEINNER,(GLdouble)0.5*(YTOE + Sqrt(Power(XTOEINNER,2) + Power(YTOE,2) + Power(ZTOE,2))));
myDrawGLElement((int)130,(double)Sqrt(Power(XTOEINNER,2) + Power(YTOE,2) + Power(ZTOE,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 131 */

glPushMatrix();
glPushMatrix();
if (ZTOE==0 && -XMETATARSALOUTER==0)
glRotated((GLdouble)90.*(-1. + YMETATARSAL/Sqrt(Power(XMETATARSALOUTER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZTOE,(GLdouble)-0.5*XMETATARSALOUTER,(GLdouble)0.5*(YMETATARSAL + Sqrt(Power(XMETATARSALOUTER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2))));
myDrawGLElement((int)131,(double)Sqrt(Power(XMETATARSALOUTER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 132 */

glPushMatrix();
glPushMatrix();
if (ZTOE==0 && XMETATARSALINNER==0)
glRotated((GLdouble)90.*(-1. + YMETATARSAL/Sqrt(Power(XMETATARSALINNER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZTOE,(GLdouble)0.5*XMETATARSALINNER,(GLdouble)0.5*(YMETATARSAL + Sqrt(Power(XMETATARSALINNER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2))));
myDrawGLElement((int)132,(double)Sqrt(Power(XMETATARSALINNER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 133 */

glPushMatrix();
glPushMatrix();
if (ZHEEL==0 && -XHEELOUTER==0)
glRotated((GLdouble)90.*(-1. - YHEEL/Sqrt(Power(XHEELOUTER,2) + Power(YHEEL,2) + Power(ZHEEL,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZHEEL,(GLdouble)-0.5*XHEELOUTER,(GLdouble)0.5*(-YHEEL + Sqrt(Power(XHEELOUTER,2) + Power(YHEEL,2) + Power(ZHEEL,2))));
myDrawGLElement((int)133,(double)Sqrt(Power(XHEELOUTER,2) + Power(YHEEL,2) + Power(ZHEEL,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 134 */

glPushMatrix();
glPushMatrix();
if (ZHEEL==0 && XHEELINNER==0)
glRotated((GLdouble)90.*(-1. - YHEEL/Sqrt(Power(XHEELINNER,2) + Power(YHEEL,2) + Power(ZHEEL,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZHEEL,(GLdouble)0.5*XHEELINNER,(GLdouble)0.5*(-YHEEL + Sqrt(Power(XHEELINNER,2) + Power(YHEEL,2) + Power(ZHEEL,2))));
myDrawGLElement((int)134,(double)Sqrt(Power(XHEELINNER,2) + Power(YHEEL,2) + Power(ZHEEL,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 135 */

glPushMatrix();
glPushMatrix();
if (eff[3].x[1]==0 && eff[3].x[2]==0)
glRotated((GLdouble)90.*(-1. + eff[3].x[3]/Sqrt(Power(eff[3].x[1],2) + Power(eff[3].x[2],2) + Power(eff[3].x[3],2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*eff[3].x[1],(GLdouble)0.5*eff[3].x[2],(GLdouble)0.5*(eff[3].x[3] + Sqrt(Power(eff[3].x[1],2) + Power(eff[3].x[2],2) + Power(eff[3].x[3],2))));
myDrawGLElement((int)135,(double)Sqrt(Power(eff[3].x[1],2) + Power(eff[3].x[2],2) + Power(eff[3].x[3],2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 16 */

glPushMatrix();
glPushMatrix();
if (-XHIP==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*XHIP,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(XHIP,2)));
myDrawGLElement((int)16,(double)Sqrt(Power(XHIP,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-XHIP,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[16].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 15 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)15,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[15].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 17 */

glPushMatrix();
glPushMatrix();
if (YHIP==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*YHIP,(GLdouble)0,(GLdouble)0.5*Sqrt(Power(YHIP,2)));
myDrawGLElement((int)17,(double)Sqrt(Power(YHIP,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)YHIP,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*THIGHANGLEOFF,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[17].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 18 */

glPushMatrix();
glPushMatrix();
if (YKNEE==0 && 0==0)
glRotated((GLdouble)90.*(-1. - UPPERLEGMOD/Sqrt(Power(UPPERLEGMOD,2) + Power(YKNEE,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*YKNEE,(GLdouble)0,(GLdouble)0.5*(-UPPERLEGMOD + Sqrt(Power(UPPERLEGMOD,2) + Power(YKNEE,2))));
myDrawGLElement((int)18,(double)Sqrt(Power(UPPERLEGMOD,2) + Power(YKNEE,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)YKNEE,(GLdouble)0,(GLdouble)-UPPERLEGMOD);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*THIGHANGLEOFF,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[18].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 19 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)19,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[19].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 20 */

glPushMatrix();
glPushMatrix();
if (0==0 && 0==0)
glRotated((GLdouble)90.*(-1. - LOWERLEG/Sqrt(Power(LOWERLEG,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0,(GLdouble)0,(GLdouble)0.5*(-LOWERLEG + Sqrt(Power(LOWERLEG,2))));
myDrawGLElement((int)20,(double)Sqrt(Power(LOWERLEG,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)0,(GLdouble)-LOWERLEG);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[20].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 21 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)21,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[21].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 122 */

glPushMatrix();
glPushMatrix();
if (ZTOE==0 && -XTOEOUTER==0)
glRotated((GLdouble)90.*(-1. - YTOE/Sqrt(Power(XTOEOUTER,2) + Power(YTOE,2) + Power(ZTOE,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZTOE,(GLdouble)-0.5*XTOEOUTER,(GLdouble)0.5*(-YTOE + Sqrt(Power(XTOEOUTER,2) + Power(YTOE,2) + Power(ZTOE,2))));
myDrawGLElement((int)122,(double)Sqrt(Power(XTOEOUTER,2) + Power(YTOE,2) + Power(ZTOE,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 123 */

glPushMatrix();
glPushMatrix();
if (ZTOE==0 && XTOEINNER==0)
glRotated((GLdouble)90.*(-1. - YTOE/Sqrt(Power(XTOEINNER,2) + Power(YTOE,2) + Power(ZTOE,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZTOE,(GLdouble)0.5*XTOEINNER,(GLdouble)0.5*(-YTOE + Sqrt(Power(XTOEINNER,2) + Power(YTOE,2) + Power(ZTOE,2))));
myDrawGLElement((int)123,(double)Sqrt(Power(XTOEINNER,2) + Power(YTOE,2) + Power(ZTOE,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 124 */

glPushMatrix();
glPushMatrix();
if (ZTOE==0 && -XMETATARSALOUTER==0)
glRotated((GLdouble)90.*(-1. - YMETATARSAL/Sqrt(Power(XMETATARSALOUTER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZTOE,(GLdouble)-0.5*XMETATARSALOUTER,(GLdouble)0.5*(-YMETATARSAL + Sqrt(Power(XMETATARSALOUTER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2))));
myDrawGLElement((int)124,(double)Sqrt(Power(XMETATARSALOUTER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 125 */

glPushMatrix();
glPushMatrix();
if (ZTOE==0 && XMETATARSALINNER==0)
glRotated((GLdouble)90.*(-1. - YMETATARSAL/Sqrt(Power(XMETATARSALINNER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZTOE,(GLdouble)0.5*XMETATARSALINNER,(GLdouble)0.5*(-YMETATARSAL + Sqrt(Power(XMETATARSALINNER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2))));
myDrawGLElement((int)125,(double)Sqrt(Power(XMETATARSALINNER,2) + Power(YMETATARSAL,2) + Power(ZTOE,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 126 */

glPushMatrix();
glPushMatrix();
if (ZHEEL==0 && -XHEELOUTER==0)
glRotated((GLdouble)90.*(-1. + YHEEL/Sqrt(Power(XHEELOUTER,2) + Power(YHEEL,2) + Power(ZHEEL,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZHEEL,(GLdouble)-0.5*XHEELOUTER,(GLdouble)0.5*(YHEEL + Sqrt(Power(XHEELOUTER,2) + Power(YHEEL,2) + Power(ZHEEL,2))));
myDrawGLElement((int)126,(double)Sqrt(Power(XHEELOUTER,2) + Power(YHEEL,2) + Power(ZHEEL,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 127 */

glPushMatrix();
glPushMatrix();
if (ZHEEL==0 && XHEELINNER==0)
glRotated((GLdouble)90.*(-1. + YHEEL/Sqrt(Power(XHEELINNER,2) + Power(YHEEL,2) + Power(ZHEEL,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*ZHEEL,(GLdouble)0.5*XHEELINNER,(GLdouble)0.5*(YHEEL + Sqrt(Power(XHEELINNER,2) + Power(YHEEL,2) + Power(ZHEEL,2))));
myDrawGLElement((int)127,(double)Sqrt(Power(XHEELINNER,2) + Power(YHEEL,2) + Power(ZHEEL,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 128 */

glPushMatrix();
glPushMatrix();
if (eff[4].x[1]==0 && eff[4].x[2]==0)
glRotated((GLdouble)90.*(-1. + eff[4].x[3]/Sqrt(Power(eff[4].x[1],2) + Power(eff[4].x[2],2) + Power(eff[4].x[3],2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*eff[4].x[1],(GLdouble)0.5*eff[4].x[2],(GLdouble)0.5*(eff[4].x[3] + Sqrt(Power(eff[4].x[1],2) + Power(eff[4].x[2],2) + Power(eff[4].x[3],2))));
myDrawGLElement((int)128,(double)Sqrt(Power(eff[4].x[1],2) + Power(eff[4].x[2],2) + Power(eff[4].x[3],2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
/*glutSwapBuffers();*/

