/* Need [n_joints+1]x[3+1] matrices: Xorigin,Xmcog,Xaxis, and Xlink[nLinks+1][3+1] */

/* sine and cosine precomputation */
sstate29th=Sin(state[29].th);
cstate29th=Cos(state[29].th);

sstate30th=Sin(state[30].th);
cstate30th=Cos(state[30].th);

sstate31th=Sin(state[31].th);
cstate31th=Cos(state[31].th);

sstate1th=Sin(state[1].th);
cstate1th=Cos(state[1].th);

sstate2th=Sin(state[2].th);
cstate2th=Cos(state[2].th);

sstate3th=Sin(state[3].th);
cstate3th=Cos(state[3].th);

sstate4th=Sin(state[4].th);
cstate4th=Cos(state[4].th);

sstate5th=Sin(state[5].th);
cstate5th=Cos(state[5].th);

sstate6th=Sin(state[6].th);
cstate6th=Cos(state[6].th);

sstate7th=Sin(state[7].th);
cstate7th=Cos(state[7].th);

sstate39th=Sin(state[39].th);
cstate39th=Cos(state[39].th);

sstate40th=Sin(state[40].th);
cstate40th=Cos(state[40].th);

sstate41th=Sin(state[41].th);
cstate41th=Cos(state[41].th);

sstate42th=Sin(state[42].th);
cstate42th=Cos(state[42].th);

sstate43th=Sin(state[43].th);
cstate43th=Cos(state[43].th);

sstate44th=Sin(state[44].th);
cstate44th=Cos(state[44].th);

sstate8th=Sin(state[8].th);
cstate8th=Cos(state[8].th);

sstate9th=Sin(state[9].th);
cstate9th=Cos(state[9].th);

sstate10th=Sin(state[10].th);
cstate10th=Cos(state[10].th);

sstate11th=Sin(state[11].th);
cstate11th=Cos(state[11].th);

sstate12th=Sin(state[12].th);
cstate12th=Cos(state[12].th);

sstate13th=Sin(state[13].th);
cstate13th=Cos(state[13].th);

sstate14th=Sin(state[14].th);
cstate14th=Cos(state[14].th);

sstate45th=Sin(state[45].th);
cstate45th=Cos(state[45].th);

sstate46th=Sin(state[46].th);
cstate46th=Cos(state[46].th);

sstate47th=Sin(state[47].th);
cstate47th=Cos(state[47].th);

sstate48th=Sin(state[48].th);
cstate48th=Cos(state[48].th);

sstate49th=Sin(state[49].th);
cstate49th=Cos(state[49].th);

sstate50th=Sin(state[50].th);
cstate50th=Cos(state[50].th);

sstate32th=Sin(state[32].th);
cstate32th=Cos(state[32].th);

sstate33th=Sin(state[33].th);
cstate33th=Cos(state[33].th);

sstate34th=Sin(state[34].th);
cstate34th=Cos(state[34].th);

sstate35th=Sin(state[35].th);
cstate35th=Cos(state[35].th);

sstate36th=Sin(state[36].th);
cstate36th=Cos(state[36].th);

sstate37th=Sin(state[37].th);
cstate37th=Cos(state[37].th);

sstate38th=Sin(state[38].th);
cstate38th=Cos(state[38].th);

sstate23th=Sin(state[23].th);
cstate23th=Cos(state[23].th);

sstate22th=Sin(state[22].th);
cstate22th=Cos(state[22].th);

sstate24th=Sin(state[24].th);
cstate24th=Cos(state[24].th);

sstate25th=Sin(state[25].th);
cstate25th=Cos(state[25].th);

sstate26th=Sin(state[26].th);
cstate26th=Cos(state[26].th);

sstate27th=Sin(state[27].th);
cstate27th=Cos(state[27].th);

sstate28th=Sin(state[28].th);
cstate28th=Cos(state[28].th);

sstate16th=Sin(state[16].th);
cstate16th=Cos(state[16].th);

sstate15th=Sin(state[15].th);
cstate15th=Cos(state[15].th);

sstate17th=Sin(state[17].th);
cstate17th=Cos(state[17].th);

sstate18th=Sin(state[18].th);
cstate18th=Cos(state[18].th);

sstate19th=Sin(state[19].th);
cstate19th=Cos(state[19].th);

sstate20th=Sin(state[20].th);
cstate20th=Cos(state[20].th);

sstate21th=Sin(state[21].th);
cstate21th=Cos(state[21].th);


/* rotation matrix sine and cosine precomputation */










rseff2a1=Sin(eff[2].a[1]);
rceff2a1=Cos(eff[2].a[1]);

rseff2a2=Sin(eff[2].a[2]);
rceff2a2=Cos(eff[2].a[2]);

rseff2a3=Sin(eff[2].a[3]);
rceff2a3=Cos(eff[2].a[3]);


rsHANDXROT=Sin(HANDXROT);
rcHANDXROT=Cos(HANDXROT);

rsTHUMBZROT=Sin(THUMBZROT);
rcTHUMBZROT=Cos(THUMBZROT);


rsTHUMBANGLEOFF=Sin(THUMBANGLEOFF);
rcTHUMBANGLEOFF=Cos(THUMBANGLEOFF);


rsMinusstate40th=-Sin(state[40].th);
rcMinusstate40th=Cos(state[40].th);



rsHANDXROT=Sin(HANDXROT);
rcHANDXROT=Cos(HANDXROT);


rsstate41th=Sin(state[41].th);
rcstate41th=Cos(state[41].th);


rsstate41th=Sin(state[41].th);
rcstate41th=Cos(state[41].th);



rsHANDXROT=Sin(HANDXROT);
rcHANDXROT=Cos(HANDXROT);


rsstate42th=Sin(state[42].th);
rcstate42th=Cos(state[42].th);


rsstate42th=Sin(state[42].th);
rcstate42th=Cos(state[42].th);



rsHANDXROT=Sin(HANDXROT);
rcHANDXROT=Cos(HANDXROT);


rsstate43th=Sin(state[43].th);
rcstate43th=Cos(state[43].th);


rsstate43th=Sin(state[43].th);
rcstate43th=Cos(state[43].th);



rsHANDXROT=Sin(HANDXROT);
rcHANDXROT=Cos(HANDXROT);


rsstate44th=Sin(state[44].th);
rcstate44th=Cos(state[44].th);


rsstate44th=Sin(state[44].th);
rcstate44th=Cos(state[44].th);










rseff1a1=Sin(eff[1].a[1]);
rceff1a1=Cos(eff[1].a[1]);

rseff1a2=Sin(eff[1].a[2]);
rceff1a2=Cos(eff[1].a[2]);

rseff1a3=Sin(eff[1].a[3]);
rceff1a3=Cos(eff[1].a[3]);


rsMinusHANDXROT=-Sin(HANDXROT);
rcMinusHANDXROT=Cos(HANDXROT);

rsTHUMBZROT=Sin(THUMBZROT);
rcTHUMBZROT=Cos(THUMBZROT);


rsTHUMBANGLEOFF=Sin(THUMBANGLEOFF);
rcTHUMBANGLEOFF=Cos(THUMBANGLEOFF);


rsMinusstate46th=-Sin(state[46].th);
rcMinusstate46th=Cos(state[46].th);



rsMinusHANDXROT=-Sin(HANDXROT);
rcMinusHANDXROT=Cos(HANDXROT);


rsstate47th=Sin(state[47].th);
rcstate47th=Cos(state[47].th);


rsstate47th=Sin(state[47].th);
rcstate47th=Cos(state[47].th);



rsMinusHANDXROT=-Sin(HANDXROT);
rcMinusHANDXROT=Cos(HANDXROT);


rsstate48th=Sin(state[48].th);
rcstate48th=Cos(state[48].th);


rsstate48th=Sin(state[48].th);
rcstate48th=Cos(state[48].th);



rsMinusHANDXROT=-Sin(HANDXROT);
rcMinusHANDXROT=Cos(HANDXROT);


rsstate49th=Sin(state[49].th);
rcstate49th=Cos(state[49].th);


rsstate49th=Sin(state[49].th);
rcstate49th=Cos(state[49].th);



rsMinusHANDXROT=-Sin(HANDXROT);
rcMinusHANDXROT=Cos(HANDXROT);


rsstate50th=Sin(state[50].th);
rcstate50th=Cos(state[50].th);


rsstate50th=Sin(state[50].th);
rcstate50th=Cos(state[50].th);















rsMinusTHIGHANGLEOFF=-Sin(THIGHANGLEOFF);
rcMinusTHIGHANGLEOFF=Cos(THIGHANGLEOFF);


rsTHIGHANGLEOFF=Sin(THIGHANGLEOFF);
rcTHIGHANGLEOFF=Cos(THIGHANGLEOFF);











rseff3a1=Sin(eff[3].a[1]);
rceff3a1=Cos(eff[3].a[1]);

rseff3a2=Sin(eff[3].a[2]);
rceff3a2=Cos(eff[3].a[2]);

rseff3a3=Sin(eff[3].a[3]);
rceff3a3=Cos(eff[3].a[3]);




rsTHIGHANGLEOFF=Sin(THIGHANGLEOFF);
rcTHIGHANGLEOFF=Cos(THIGHANGLEOFF);


rsTHIGHANGLEOFF=Sin(THIGHANGLEOFF);
rcTHIGHANGLEOFF=Cos(THIGHANGLEOFF);











rseff4a1=Sin(eff[4].a[1]);
rceff4a1=Cos(eff[4].a[1]);

rseff4a2=Sin(eff[4].a[2]);
rceff4a2=Cos(eff[4].a[2]);

rseff4a3=Sin(eff[4].a[3]);
rceff4a3=Cos(eff[4].a[3]);



/* inverse homogeneous rotation matrices */
Hi00[1][1]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[2],2);
Hi00[1][2]=2*(baseo[0].q[2]*baseo[0].q[3] - baseo[0].q[1]*baseo[0].q[4]);
Hi00[1][3]=2*(baseo[0].q[1]*baseo[0].q[3] + baseo[0].q[2]*baseo[0].q[4]);
Hi00[1][4]=basec[0].x[1];

Hi00[2][1]=2*(baseo[0].q[2]*baseo[0].q[3] + baseo[0].q[1]*baseo[0].q[4]);
Hi00[2][2]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[3],2);
Hi00[2][3]=2*(-(baseo[0].q[1]*baseo[0].q[2]) + baseo[0].q[3]*baseo[0].q[4]);
Hi00[2][4]=basec[0].x[2];

Hi00[3][1]=2*(-(baseo[0].q[1]*baseo[0].q[3]) + baseo[0].q[2]*baseo[0].q[4]);
Hi00[3][2]=2*(baseo[0].q[1]*baseo[0].q[2] + baseo[0].q[3]*baseo[0].q[4]);
Hi00[3][3]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[4],2);
Hi00[3][4]=basec[0].x[3];


Hi01[1][1]=cstate29th;
Hi01[1][2]=-sstate29th;

Hi01[2][1]=-sstate29th;
Hi01[2][2]=-cstate29th;
Hi01[2][4]=-PELVISOFFSET;

Hi01[3][4]=PELVIS2THORAX;


Hi12[1][1]=-sstate30th;
Hi12[1][2]=-cstate30th;

Hi12[3][1]=cstate30th;
Hi12[3][2]=-sstate30th;


Hi23[1][1]=cstate31th;
Hi23[1][2]=-sstate31th;

Hi23[3][1]=-sstate31th;
Hi23[3][2]=-cstate31th;


Hi34[1][1]=0.7071067811865475*cstate1th;
Hi34[1][2]=-0.7071067811865475*sstate1th;
Hi34[1][4]=-THORAX2SHOULDER;

Hi34[2][1]=-sstate1th;
Hi34[2][2]=-cstate1th;

Hi34[3][1]=0.7071067811865475*cstate1th;
Hi34[3][2]=-0.7071067811865475*sstate1th;


Hi45[1][1]=-0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;
Hi45[1][2]=-0.7071067811865475*cstate2th + 0.7071067811865475*sstate2th;

Hi45[3][1]=0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;
Hi45[3][2]=-0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;
Hi45[3][4]=-SHOULDERX;


Hi56[1][1]=cstate3th;
Hi56[1][2]=-sstate3th;
Hi56[1][4]=-SHOULDERY;

Hi56[3][1]=-sstate3th;
Hi56[3][2]=-cstate3th;


Hi67[2][1]=cstate4th;
Hi67[2][2]=-sstate4th;

Hi67[3][1]=sstate4th;
Hi67[3][2]=cstate4th;
Hi67[3][4]=-UPPERARM;


Hi78[1][1]=cstate5th;
Hi78[1][2]=-sstate5th;

Hi78[3][1]=-sstate5th;
Hi78[3][2]=-cstate5th;


Hi89[2][1]=sstate6th;
Hi89[2][2]=cstate6th;
Hi89[2][4]=WRISTY;

Hi89[3][1]=-cstate6th;
Hi89[3][2]=sstate6th;
Hi89[3][4]=-LOWERARM;


Hi910[1][1]=cstate7th;
Hi910[1][2]=-sstate7th;

Hi910[3][1]=sstate7th;
Hi910[3][2]=cstate7th;


Hi1011[1][1]=rceff2a2*rceff2a3;
Hi1011[1][2]=-(rceff2a2*rseff2a3);
Hi1011[1][3]=rseff2a2;
Hi1011[1][4]=eff[2].x[1];

Hi1011[2][1]=rceff2a3*rseff2a1*rseff2a2 + rceff2a1*rseff2a3;
Hi1011[2][2]=rceff2a1*rceff2a3 - rseff2a1*rseff2a2*rseff2a3;
Hi1011[2][3]=-(rceff2a2*rseff2a1);
Hi1011[2][4]=eff[2].x[2];

Hi1011[3][1]=-(rceff2a1*rceff2a3*rseff2a2) + rseff2a1*rseff2a3;
Hi1011[3][2]=rceff2a3*rseff2a1 + rceff2a1*rseff2a2*rseff2a3;
Hi1011[3][3]=rceff2a1*rceff2a2;
Hi1011[3][4]=eff[2].x[3];


Hi1012[1][1]=rcTHUMBZROT;
Hi1012[1][2]=-(cstate39th*rsTHUMBZROT);
Hi1012[1][3]=rsTHUMBZROT*sstate39th;
Hi1012[1][4]=XTHUMB;

Hi1012[2][1]=rcHANDXROT*rsTHUMBZROT;
Hi1012[2][2]=cstate39th*rcHANDXROT*rcTHUMBZROT - rsHANDXROT*sstate39th;
Hi1012[2][3]=-(cstate39th*rsHANDXROT) - rcHANDXROT*rcTHUMBZROT*sstate39th;
Hi1012[2][4]=YTHUMB;

Hi1012[3][1]=rsHANDXROT*rsTHUMBZROT;
Hi1012[3][2]=cstate39th*rcTHUMBZROT*rsHANDXROT + rcHANDXROT*sstate39th;
Hi1012[3][3]=cstate39th*rcHANDXROT - rcTHUMBZROT*rsHANDXROT*sstate39th;
Hi1012[3][4]=-ZTHUMB;


Hi1213[1][1]=cstate40th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate40th;
Hi1213[1][2]=-(cstate40th*rsTHUMBANGLEOFF) + rcTHUMBANGLEOFF*sstate40th;
Hi1213[1][4]=XTHUMBFLEX;

Hi1213[2][1]=cstate40th*rsTHUMBANGLEOFF - rcTHUMBANGLEOFF*sstate40th;
Hi1213[2][2]=cstate40th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate40th;
Hi1213[2][4]=YTHUMBFLEX;


Hi1314[1][1]=rcMinusstate40th;
Hi1314[1][2]=-rsMinusstate40th;
Hi1314[1][4]=TH1SEG;

Hi1314[2][1]=rsMinusstate40th;
Hi1314[2][2]=rcMinusstate40th;


Hi1415[1][4]=TH2SEG;


Hi1016[1][1]=cstate41th;
Hi1016[1][2]=-sstate41th;
Hi1016[1][4]=XIF;

Hi1016[2][1]=rcHANDXROT*sstate41th;
Hi1016[2][2]=cstate41th*rcHANDXROT;
Hi1016[2][3]=-rsHANDXROT;
Hi1016[2][4]=YIF;

Hi1016[3][1]=rsHANDXROT*sstate41th;
Hi1016[3][2]=cstate41th*rsHANDXROT;
Hi1016[3][3]=rcHANDXROT;
Hi1016[3][4]=-ZIF;


Hi1617[1][1]=rcstate41th;
Hi1617[1][2]=-rsstate41th;
Hi1617[1][4]=F1SEG;

Hi1617[2][1]=rsstate41th;
Hi1617[2][2]=rcstate41th;


Hi1718[1][1]=rcstate41th;
Hi1718[1][2]=-rsstate41th;
Hi1718[1][4]=F2SEG;

Hi1718[2][1]=rsstate41th;
Hi1718[2][2]=rcstate41th;


Hi1819[1][4]=F3SEG;


Hi1020[1][1]=cstate42th;
Hi1020[1][2]=-sstate42th;
Hi1020[1][4]=XMF;

Hi1020[2][1]=rcHANDXROT*sstate42th;
Hi1020[2][2]=cstate42th*rcHANDXROT;
Hi1020[2][3]=-rsHANDXROT;
Hi1020[2][4]=YMF;

Hi1020[3][1]=rsHANDXROT*sstate42th;
Hi1020[3][2]=cstate42th*rsHANDXROT;
Hi1020[3][3]=rcHANDXROT;
Hi1020[3][4]=-ZMF;


Hi2021[1][1]=rcstate42th;
Hi2021[1][2]=-rsstate42th;
Hi2021[1][4]=F1SEG;

Hi2021[2][1]=rsstate42th;
Hi2021[2][2]=rcstate42th;


Hi2122[1][1]=rcstate42th;
Hi2122[1][2]=-rsstate42th;
Hi2122[1][4]=F2SEG;

Hi2122[2][1]=rsstate42th;
Hi2122[2][2]=rcstate42th;


Hi2223[1][4]=F3SEG;


Hi1024[1][1]=cstate43th;
Hi1024[1][2]=-sstate43th;
Hi1024[1][4]=XRF;

Hi1024[2][1]=rcHANDXROT*sstate43th;
Hi1024[2][2]=cstate43th*rcHANDXROT;
Hi1024[2][3]=-rsHANDXROT;
Hi1024[2][4]=YRF;

Hi1024[3][1]=rsHANDXROT*sstate43th;
Hi1024[3][2]=cstate43th*rsHANDXROT;
Hi1024[3][3]=rcHANDXROT;
Hi1024[3][4]=-ZRF;


Hi2425[1][1]=rcstate43th;
Hi2425[1][2]=-rsstate43th;
Hi2425[1][4]=F1SEG;

Hi2425[2][1]=rsstate43th;
Hi2425[2][2]=rcstate43th;


Hi2526[1][1]=rcstate43th;
Hi2526[1][2]=-rsstate43th;
Hi2526[1][4]=F2SEG;

Hi2526[2][1]=rsstate43th;
Hi2526[2][2]=rcstate43th;


Hi2627[1][4]=F3SEG;


Hi1028[1][1]=cstate44th;
Hi1028[1][2]=-sstate44th;
Hi1028[1][4]=XLF;

Hi1028[2][1]=rcHANDXROT*sstate44th;
Hi1028[2][2]=cstate44th*rcHANDXROT;
Hi1028[2][3]=-rsHANDXROT;
Hi1028[2][4]=YLF;

Hi1028[3][1]=rsHANDXROT*sstate44th;
Hi1028[3][2]=cstate44th*rsHANDXROT;
Hi1028[3][3]=rcHANDXROT;
Hi1028[3][4]=-ZLF;


Hi2829[1][1]=rcstate44th;
Hi2829[1][2]=-rsstate44th;
Hi2829[1][4]=F1SEG;

Hi2829[2][1]=rsstate44th;
Hi2829[2][2]=rcstate44th;


Hi2930[1][1]=rcstate44th;
Hi2930[1][2]=-rsstate44th;
Hi2930[1][4]=F2SEG;

Hi2930[2][1]=rsstate44th;
Hi2930[2][2]=rcstate44th;


Hi3031[1][4]=F3SEG;


Hi332[1][1]=0.7071067811865475*cstate8th;
Hi332[1][2]=-0.7071067811865475*sstate8th;
Hi332[1][4]=-THORAX2SHOULDER;

Hi332[2][1]=-sstate8th;
Hi332[2][2]=-cstate8th;

Hi332[3][1]=-0.7071067811865475*cstate8th;
Hi332[3][2]=0.7071067811865475*sstate8th;


Hi3233[1][1]=-0.7071067811865475*cstate9th - 0.7071067811865475*sstate9th;
Hi3233[1][2]=-0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;

Hi3233[3][1]=-0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;
Hi3233[3][2]=0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;
Hi3233[3][4]=SHOULDERX;


Hi3334[1][1]=cstate10th;
Hi3334[1][2]=-sstate10th;
Hi3334[1][4]=-SHOULDERY;

Hi3334[3][1]=sstate10th;
Hi3334[3][2]=cstate10th;


Hi3435[2][1]=cstate11th;
Hi3435[2][2]=-sstate11th;

Hi3435[3][1]=-sstate11th;
Hi3435[3][2]=-cstate11th;
Hi3435[3][4]=UPPERARM;


Hi3536[1][1]=cstate12th;
Hi3536[1][2]=-sstate12th;

Hi3536[3][1]=sstate12th;
Hi3536[3][2]=cstate12th;


Hi3637[2][1]=sstate13th;
Hi3637[2][2]=cstate13th;
Hi3637[2][4]=WRISTY;

Hi3637[3][1]=cstate13th;
Hi3637[3][2]=-sstate13th;
Hi3637[3][4]=LOWERARM;


Hi3738[1][1]=cstate14th;
Hi3738[1][2]=-sstate14th;

Hi3738[3][1]=-sstate14th;
Hi3738[3][2]=-cstate14th;


Hi3839[1][1]=rceff1a2*rceff1a3;
Hi3839[1][2]=-(rceff1a2*rseff1a3);
Hi3839[1][3]=rseff1a2;
Hi3839[1][4]=eff[1].x[1];

Hi3839[2][1]=rceff1a3*rseff1a1*rseff1a2 + rceff1a1*rseff1a3;
Hi3839[2][2]=rceff1a1*rceff1a3 - rseff1a1*rseff1a2*rseff1a3;
Hi3839[2][3]=-(rceff1a2*rseff1a1);
Hi3839[2][4]=eff[1].x[2];

Hi3839[3][1]=-(rceff1a1*rceff1a3*rseff1a2) + rseff1a1*rseff1a3;
Hi3839[3][2]=rceff1a3*rseff1a1 + rceff1a1*rseff1a2*rseff1a3;
Hi3839[3][3]=rceff1a1*rceff1a2;
Hi3839[3][4]=eff[1].x[3];


Hi3840[1][1]=rcTHUMBZROT;
Hi3840[1][2]=-(cstate45th*rsTHUMBZROT);
Hi3840[1][3]=rsTHUMBZROT*sstate45th;
Hi3840[1][4]=XTHUMB;

Hi3840[2][1]=rcMinusHANDXROT*rsTHUMBZROT;
Hi3840[2][2]=cstate45th*rcMinusHANDXROT*rcTHUMBZROT - rsMinusHANDXROT*sstate45th;
Hi3840[2][3]=-(cstate45th*rsMinusHANDXROT) - rcMinusHANDXROT*rcTHUMBZROT*sstate45th;
Hi3840[2][4]=YTHUMB;

Hi3840[3][1]=rsMinusHANDXROT*rsTHUMBZROT;
Hi3840[3][2]=cstate45th*rcTHUMBZROT*rsMinusHANDXROT + rcMinusHANDXROT*sstate45th;
Hi3840[3][3]=cstate45th*rcMinusHANDXROT - rcTHUMBZROT*rsMinusHANDXROT*sstate45th;
Hi3840[3][4]=ZTHUMB;


Hi4041[1][1]=cstate46th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate46th;
Hi4041[1][2]=-(cstate46th*rsTHUMBANGLEOFF) + rcTHUMBANGLEOFF*sstate46th;
Hi4041[1][4]=XTHUMBFLEX;

Hi4041[2][1]=cstate46th*rsTHUMBANGLEOFF - rcTHUMBANGLEOFF*sstate46th;
Hi4041[2][2]=cstate46th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate46th;
Hi4041[2][4]=YTHUMBFLEX;


Hi4142[1][1]=rcMinusstate46th;
Hi4142[1][2]=-rsMinusstate46th;
Hi4142[1][4]=TH1SEG;

Hi4142[2][1]=rsMinusstate46th;
Hi4142[2][2]=rcMinusstate46th;


Hi4243[1][4]=TH2SEG;


Hi3844[1][1]=cstate47th;
Hi3844[1][2]=-sstate47th;
Hi3844[1][4]=XIF;

Hi3844[2][1]=rcMinusHANDXROT*sstate47th;
Hi3844[2][2]=cstate47th*rcMinusHANDXROT;
Hi3844[2][3]=-rsMinusHANDXROT;
Hi3844[2][4]=YIF;

Hi3844[3][1]=rsMinusHANDXROT*sstate47th;
Hi3844[3][2]=cstate47th*rsMinusHANDXROT;
Hi3844[3][3]=rcMinusHANDXROT;
Hi3844[3][4]=ZIF;


Hi4445[1][1]=rcstate47th;
Hi4445[1][2]=-rsstate47th;
Hi4445[1][4]=F1SEG;

Hi4445[2][1]=rsstate47th;
Hi4445[2][2]=rcstate47th;


Hi4546[1][1]=rcstate47th;
Hi4546[1][2]=-rsstate47th;
Hi4546[1][4]=F2SEG;

Hi4546[2][1]=rsstate47th;
Hi4546[2][2]=rcstate47th;


Hi4647[1][4]=F3SEG;


Hi3848[1][1]=cstate48th;
Hi3848[1][2]=-sstate48th;
Hi3848[1][4]=XMF;

Hi3848[2][1]=rcMinusHANDXROT*sstate48th;
Hi3848[2][2]=cstate48th*rcMinusHANDXROT;
Hi3848[2][3]=-rsMinusHANDXROT;
Hi3848[2][4]=YMF;

Hi3848[3][1]=rsMinusHANDXROT*sstate48th;
Hi3848[3][2]=cstate48th*rsMinusHANDXROT;
Hi3848[3][3]=rcMinusHANDXROT;
Hi3848[3][4]=ZMF;


Hi4849[1][1]=rcstate48th;
Hi4849[1][2]=-rsstate48th;
Hi4849[1][4]=F1SEG;

Hi4849[2][1]=rsstate48th;
Hi4849[2][2]=rcstate48th;


Hi4950[1][1]=rcstate48th;
Hi4950[1][2]=-rsstate48th;
Hi4950[1][4]=F2SEG;

Hi4950[2][1]=rsstate48th;
Hi4950[2][2]=rcstate48th;


Hi5051[1][4]=F3SEG;


Hi3852[1][1]=cstate49th;
Hi3852[1][2]=-sstate49th;
Hi3852[1][4]=XRF;

Hi3852[2][1]=rcMinusHANDXROT*sstate49th;
Hi3852[2][2]=cstate49th*rcMinusHANDXROT;
Hi3852[2][3]=-rsMinusHANDXROT;
Hi3852[2][4]=YRF;

Hi3852[3][1]=rsMinusHANDXROT*sstate49th;
Hi3852[3][2]=cstate49th*rsMinusHANDXROT;
Hi3852[3][3]=rcMinusHANDXROT;
Hi3852[3][4]=ZRF;


Hi5253[1][1]=rcstate49th;
Hi5253[1][2]=-rsstate49th;
Hi5253[1][4]=F1SEG;

Hi5253[2][1]=rsstate49th;
Hi5253[2][2]=rcstate49th;


Hi5354[1][1]=rcstate49th;
Hi5354[1][2]=-rsstate49th;
Hi5354[1][4]=F2SEG;

Hi5354[2][1]=rsstate49th;
Hi5354[2][2]=rcstate49th;


Hi5455[1][4]=F3SEG;


Hi3856[1][1]=cstate50th;
Hi3856[1][2]=-sstate50th;
Hi3856[1][4]=XLF;

Hi3856[2][1]=rcMinusHANDXROT*sstate50th;
Hi3856[2][2]=cstate50th*rcMinusHANDXROT;
Hi3856[2][3]=-rsMinusHANDXROT;
Hi3856[2][4]=YLF;

Hi3856[3][1]=rsMinusHANDXROT*sstate50th;
Hi3856[3][2]=cstate50th*rsMinusHANDXROT;
Hi3856[3][3]=rcMinusHANDXROT;
Hi3856[3][4]=ZLF;


Hi5657[1][1]=rcstate50th;
Hi5657[1][2]=-rsstate50th;
Hi5657[1][4]=F1SEG;

Hi5657[2][1]=rsstate50th;
Hi5657[2][2]=rcstate50th;


Hi5758[1][1]=rcstate50th;
Hi5758[1][2]=-rsstate50th;
Hi5758[1][4]=F2SEG;

Hi5758[2][1]=rsstate50th;
Hi5758[2][2]=rcstate50th;


Hi5859[1][4]=F3SEG;


Hi360[1][1]=sstate32th;
Hi360[1][2]=cstate32th;
Hi360[1][4]=-THORAX2NECK;

Hi360[2][1]=-cstate32th;
Hi360[2][2]=sstate32th;


Hi6061[2][1]=sstate33th;
Hi6061[2][2]=cstate33th;
Hi6061[2][4]=-CERVICAL;

Hi6061[3][1]=-cstate33th;
Hi6061[3][2]=sstate33th;


Hi6162[1][1]=cstate34th;
Hi6162[1][2]=-sstate34th;

Hi6162[3][1]=-sstate34th;
Hi6162[3][2]=-cstate34th;


Hi6263[1][1]=cstate35th;
Hi6263[1][2]=-sstate35th;
Hi6263[1][4]=EYEXOFF;

Hi6263[2][1]=sstate35th;
Hi6263[2][2]=cstate35th;
Hi6263[2][4]=-EYEYOFF;

Hi6263[3][4]=-HEAD;


Hi6364[2][1]=sstate36th;
Hi6364[2][2]=cstate36th;

Hi6364[3][1]=cstate36th;
Hi6364[3][2]=-sstate36th;


Hi6465[2][4]=-EYE;


Hi6266[1][1]=cstate37th;
Hi6266[1][2]=-sstate37th;
Hi6266[1][4]=-EYEXOFF;

Hi6266[2][1]=sstate37th;
Hi6266[2][2]=cstate37th;
Hi6266[2][4]=-EYEYOFF;

Hi6266[3][4]=-HEAD;


Hi6667[2][1]=sstate38th;
Hi6667[2][2]=cstate38th;

Hi6667[3][1]=cstate38th;
Hi6667[3][2]=-sstate38th;


Hi6768[2][4]=-EYE;


Hi6269[3][4]=-TOPofHEAD;


Hi070[1][1]=-sstate23th;
Hi070[1][2]=-cstate23th;
Hi070[1][4]=XHIP;

Hi070[3][1]=-cstate23th;
Hi070[3][2]=sstate23th;


Hi7071[1][1]=sstate22th;
Hi7071[1][2]=cstate22th;

Hi7071[3][1]=-cstate22th;
Hi7071[3][2]=sstate22th;


Hi7172[1][1]=cstate24th*rcMinusTHIGHANGLEOFF;
Hi7172[1][2]=-(rcMinusTHIGHANGLEOFF*sstate24th);
Hi7172[1][3]=rsMinusTHIGHANGLEOFF;
Hi7172[1][4]=YHIP;

Hi7172[2][1]=-(cstate24th*rsMinusTHIGHANGLEOFF);
Hi7172[2][2]=rsMinusTHIGHANGLEOFF*sstate24th;
Hi7172[2][3]=rcMinusTHIGHANGLEOFF;

Hi7172[3][1]=-sstate24th;
Hi7172[3][2]=-cstate24th;


Hi7273[1][1]=cstate25th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate25th;
Hi7273[1][2]=-(cstate25th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate25th;
Hi7273[1][4]=YKNEE;

Hi7273[3][1]=-(cstate25th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate25th;
Hi7273[3][2]=-(cstate25th*rcTHIGHANGLEOFF) + rsTHIGHANGLEOFF*sstate25th;
Hi7273[3][4]=UPPERLEGMOD;


Hi7374[1][1]=cstate26th;
Hi7374[1][2]=-sstate26th;

Hi7374[3][1]=sstate26th;
Hi7374[3][2]=cstate26th;


Hi7475[1][1]=-sstate27th;
Hi7475[1][2]=-cstate27th;

Hi7475[3][1]=cstate27th;
Hi7475[3][2]=-sstate27th;
Hi7475[3][4]=LOWERLEG;


Hi7576[1][1]=cstate28th;
Hi7576[1][2]=-sstate28th;

Hi7576[3][1]=-sstate28th;
Hi7576[3][2]=-cstate28th;


Hi7677[1][4]=ZTOE;

Hi7677[2][4]=-XTOEOUTER;

Hi7677[3][4]=YTOE;


Hi7678[1][4]=ZTOE;

Hi7678[2][4]=XTOEINNER;

Hi7678[3][4]=YTOE;


Hi7679[1][4]=ZTOE;

Hi7679[2][4]=-XMETATARSALOUTER;

Hi7679[3][4]=YMETATARSAL;


Hi7680[1][4]=ZTOE;

Hi7680[2][4]=XMETATARSALINNER;

Hi7680[3][4]=YMETATARSAL;


Hi7681[1][4]=ZHEEL;

Hi7681[2][4]=-XHEELOUTER;

Hi7681[3][4]=-YHEEL;


Hi7682[1][4]=ZHEEL;

Hi7682[2][4]=XHEELINNER;

Hi7682[3][4]=-YHEEL;


Hi7683[1][1]=rceff3a2*rceff3a3;
Hi7683[1][2]=-(rceff3a2*rseff3a3);
Hi7683[1][3]=rseff3a2;
Hi7683[1][4]=eff[3].x[1];

Hi7683[2][1]=rceff3a3*rseff3a1*rseff3a2 + rceff3a1*rseff3a3;
Hi7683[2][2]=rceff3a1*rceff3a3 - rseff3a1*rseff3a2*rseff3a3;
Hi7683[2][3]=-(rceff3a2*rseff3a1);
Hi7683[2][4]=eff[3].x[2];

Hi7683[3][1]=-(rceff3a1*rceff3a3*rseff3a2) + rseff3a1*rseff3a3;
Hi7683[3][2]=rceff3a3*rseff3a1 + rceff3a1*rseff3a2*rseff3a3;
Hi7683[3][3]=rceff3a1*rceff3a2;
Hi7683[3][4]=eff[3].x[3];


Hi084[1][1]=sstate16th;
Hi084[1][2]=cstate16th;
Hi084[1][4]=-XHIP;

Hi084[3][1]=-cstate16th;
Hi084[3][2]=sstate16th;


Hi8485[1][1]=sstate15th;
Hi8485[1][2]=cstate15th;

Hi8485[3][1]=cstate15th;
Hi8485[3][2]=-sstate15th;


Hi8586[1][1]=cstate17th*rcTHIGHANGLEOFF;
Hi8586[1][2]=-(rcTHIGHANGLEOFF*sstate17th);
Hi8586[1][3]=rsTHIGHANGLEOFF;
Hi8586[1][4]=YHIP;

Hi8586[2][1]=cstate17th*rsTHIGHANGLEOFF;
Hi8586[2][2]=-(rsTHIGHANGLEOFF*sstate17th);
Hi8586[2][3]=-rcTHIGHANGLEOFF;

Hi8586[3][1]=sstate17th;
Hi8586[3][2]=cstate17th;


Hi8687[1][1]=cstate18th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate18th;
Hi8687[1][2]=-(cstate18th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate18th;
Hi8687[1][4]=YKNEE;

Hi8687[3][1]=cstate18th*rsTHIGHANGLEOFF + rcTHIGHANGLEOFF*sstate18th;
Hi8687[3][2]=cstate18th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate18th;
Hi8687[3][4]=-UPPERLEGMOD;


Hi8788[1][1]=cstate19th;
Hi8788[1][2]=-sstate19th;

Hi8788[3][1]=-sstate19th;
Hi8788[3][2]=-cstate19th;


Hi8889[1][1]=-sstate20th;
Hi8889[1][2]=-cstate20th;

Hi8889[3][1]=-cstate20th;
Hi8889[3][2]=sstate20th;
Hi8889[3][4]=-LOWERLEG;


Hi8990[1][1]=cstate21th;
Hi8990[1][2]=-sstate21th;

Hi8990[3][1]=sstate21th;
Hi8990[3][2]=cstate21th;


Hi9091[1][4]=ZTOE;

Hi9091[2][4]=-XTOEOUTER;

Hi9091[3][4]=-YTOE;


Hi9092[1][4]=ZTOE;

Hi9092[2][4]=XTOEINNER;

Hi9092[3][4]=-YTOE;


Hi9093[1][4]=ZTOE;

Hi9093[2][4]=-XMETATARSALOUTER;

Hi9093[3][4]=-YMETATARSAL;


Hi9094[1][4]=ZTOE;

Hi9094[2][4]=XMETATARSALINNER;

Hi9094[3][4]=-YMETATARSAL;


Hi9095[1][4]=ZHEEL;

Hi9095[2][4]=-XHEELOUTER;

Hi9095[3][4]=YHEEL;


Hi9096[1][4]=ZHEEL;

Hi9096[2][4]=XHEELINNER;

Hi9096[3][4]=YHEEL;


Hi9097[1][1]=rceff4a2*rceff4a3;
Hi9097[1][2]=-(rceff4a2*rseff4a3);
Hi9097[1][3]=rseff4a2;
Hi9097[1][4]=eff[4].x[1];

Hi9097[2][1]=rceff4a3*rseff4a1*rseff4a2 + rceff4a1*rseff4a3;
Hi9097[2][2]=rceff4a1*rceff4a3 - rseff4a1*rseff4a2*rseff4a3;
Hi9097[2][3]=-(rceff4a2*rseff4a1);
Hi9097[2][4]=eff[4].x[2];

Hi9097[3][1]=-(rceff4a1*rceff4a3*rseff4a2) + rseff4a1*rseff4a3;
Hi9097[3][2]=rceff4a3*rseff4a1 + rceff4a1*rseff4a2*rseff4a3;
Hi9097[3][3]=rceff4a1*rceff4a2;
Hi9097[3][4]=eff[4].x[3];



/* per link inverse homogeneous rotation matrices */
Ai01[1][1]=Hi00[1][1]*Hi01[1][1] + Hi00[1][2]*Hi01[2][1];
Ai01[1][2]=Hi00[1][1]*Hi01[1][2] + Hi00[1][2]*Hi01[2][2];
Ai01[1][3]=-Hi00[1][3];
Ai01[1][4]=Hi00[1][4] + Hi00[1][2]*Hi01[2][4] + Hi00[1][3]*Hi01[3][4];

Ai01[2][1]=Hi00[2][1]*Hi01[1][1] + Hi00[2][2]*Hi01[2][1];
Ai01[2][2]=Hi00[2][1]*Hi01[1][2] + Hi00[2][2]*Hi01[2][2];
Ai01[2][3]=-Hi00[2][3];
Ai01[2][4]=Hi00[2][4] + Hi00[2][2]*Hi01[2][4] + Hi00[2][3]*Hi01[3][4];

Ai01[3][1]=Hi00[3][1]*Hi01[1][1] + Hi00[3][2]*Hi01[2][1];
Ai01[3][2]=Hi00[3][1]*Hi01[1][2] + Hi00[3][2]*Hi01[2][2];
Ai01[3][3]=-Hi00[3][3];
Ai01[3][4]=Hi00[3][4] + Hi00[3][2]*Hi01[2][4] + Hi00[3][3]*Hi01[3][4];


Ai02[1][1]=Ai01[1][1]*Hi12[1][1] + Ai01[1][3]*Hi12[3][1];
Ai02[1][2]=Ai01[1][1]*Hi12[1][2] + Ai01[1][3]*Hi12[3][2];
Ai02[1][3]=-Ai01[1][2];
Ai02[1][4]=Ai01[1][4];

Ai02[2][1]=Ai01[2][1]*Hi12[1][1] + Ai01[2][3]*Hi12[3][1];
Ai02[2][2]=Ai01[2][1]*Hi12[1][2] + Ai01[2][3]*Hi12[3][2];
Ai02[2][3]=-Ai01[2][2];
Ai02[2][4]=Ai01[2][4];

Ai02[3][1]=Ai01[3][1]*Hi12[1][1] + Ai01[3][3]*Hi12[3][1];
Ai02[3][2]=Ai01[3][1]*Hi12[1][2] + Ai01[3][3]*Hi12[3][2];
Ai02[3][3]=-Ai01[3][2];
Ai02[3][4]=Ai01[3][4];


Ai03[1][1]=Ai02[1][1]*Hi23[1][1] + Ai02[1][3]*Hi23[3][1];
Ai03[1][2]=Ai02[1][1]*Hi23[1][2] + Ai02[1][3]*Hi23[3][2];
Ai03[1][3]=Ai02[1][2];
Ai03[1][4]=Ai02[1][4];

Ai03[2][1]=Ai02[2][1]*Hi23[1][1] + Ai02[2][3]*Hi23[3][1];
Ai03[2][2]=Ai02[2][1]*Hi23[1][2] + Ai02[2][3]*Hi23[3][2];
Ai03[2][3]=Ai02[2][2];
Ai03[2][4]=Ai02[2][4];

Ai03[3][1]=Ai02[3][1]*Hi23[1][1] + Ai02[3][3]*Hi23[3][1];
Ai03[3][2]=Ai02[3][1]*Hi23[1][2] + Ai02[3][3]*Hi23[3][2];
Ai03[3][3]=Ai02[3][2];
Ai03[3][4]=Ai02[3][4];


Ai04[1][1]=Ai03[1][1]*Hi34[1][1] + Ai03[1][2]*Hi34[2][1] + Ai03[1][3]*Hi34[3][1];
Ai04[1][2]=Ai03[1][1]*Hi34[1][2] + Ai03[1][2]*Hi34[2][2] + Ai03[1][3]*Hi34[3][2];
Ai04[1][3]=0.7071067811865475*Ai03[1][1] - 0.7071067811865475*Ai03[1][3];
Ai04[1][4]=Ai03[1][4] + Ai03[1][1]*Hi34[1][4];

Ai04[2][1]=Ai03[2][1]*Hi34[1][1] + Ai03[2][2]*Hi34[2][1] + Ai03[2][3]*Hi34[3][1];
Ai04[2][2]=Ai03[2][1]*Hi34[1][2] + Ai03[2][2]*Hi34[2][2] + Ai03[2][3]*Hi34[3][2];
Ai04[2][3]=0.7071067811865475*Ai03[2][1] - 0.7071067811865475*Ai03[2][3];
Ai04[2][4]=Ai03[2][4] + Ai03[2][1]*Hi34[1][4];

Ai04[3][1]=Ai03[3][1]*Hi34[1][1] + Ai03[3][2]*Hi34[2][1] + Ai03[3][3]*Hi34[3][1];
Ai04[3][2]=Ai03[3][1]*Hi34[1][2] + Ai03[3][2]*Hi34[2][2] + Ai03[3][3]*Hi34[3][2];
Ai04[3][3]=0.7071067811865475*Ai03[3][1] - 0.7071067811865475*Ai03[3][3];
Ai04[3][4]=Ai03[3][4] + Ai03[3][1]*Hi34[1][4];


Ai05[1][1]=Ai04[1][1]*Hi45[1][1] + Ai04[1][3]*Hi45[3][1];
Ai05[1][2]=Ai04[1][1]*Hi45[1][2] + Ai04[1][3]*Hi45[3][2];
Ai05[1][3]=-Ai04[1][2];
Ai05[1][4]=Ai04[1][4] + Ai04[1][3]*Hi45[3][4];

Ai05[2][1]=Ai04[2][1]*Hi45[1][1] + Ai04[2][3]*Hi45[3][1];
Ai05[2][2]=Ai04[2][1]*Hi45[1][2] + Ai04[2][3]*Hi45[3][2];
Ai05[2][3]=-Ai04[2][2];
Ai05[2][4]=Ai04[2][4] + Ai04[2][3]*Hi45[3][4];

Ai05[3][1]=Ai04[3][1]*Hi45[1][1] + Ai04[3][3]*Hi45[3][1];
Ai05[3][2]=Ai04[3][1]*Hi45[1][2] + Ai04[3][3]*Hi45[3][2];
Ai05[3][3]=-Ai04[3][2];
Ai05[3][4]=Ai04[3][4] + Ai04[3][3]*Hi45[3][4];


Ai06[1][1]=Ai05[1][1]*Hi56[1][1] + Ai05[1][3]*Hi56[3][1];
Ai06[1][2]=Ai05[1][1]*Hi56[1][2] + Ai05[1][3]*Hi56[3][2];
Ai06[1][3]=Ai05[1][2];
Ai06[1][4]=Ai05[1][4] + Ai05[1][1]*Hi56[1][4];

Ai06[2][1]=Ai05[2][1]*Hi56[1][1] + Ai05[2][3]*Hi56[3][1];
Ai06[2][2]=Ai05[2][1]*Hi56[1][2] + Ai05[2][3]*Hi56[3][2];
Ai06[2][3]=Ai05[2][2];
Ai06[2][4]=Ai05[2][4] + Ai05[2][1]*Hi56[1][4];

Ai06[3][1]=Ai05[3][1]*Hi56[1][1] + Ai05[3][3]*Hi56[3][1];
Ai06[3][2]=Ai05[3][1]*Hi56[1][2] + Ai05[3][3]*Hi56[3][2];
Ai06[3][3]=Ai05[3][2];
Ai06[3][4]=Ai05[3][4] + Ai05[3][1]*Hi56[1][4];


Ai07[1][1]=Ai06[1][2]*Hi67[2][1] + Ai06[1][3]*Hi67[3][1];
Ai07[1][2]=Ai06[1][2]*Hi67[2][2] + Ai06[1][3]*Hi67[3][2];
Ai07[1][3]=Ai06[1][1];
Ai07[1][4]=Ai06[1][4] + Ai06[1][3]*Hi67[3][4];

Ai07[2][1]=Ai06[2][2]*Hi67[2][1] + Ai06[2][3]*Hi67[3][1];
Ai07[2][2]=Ai06[2][2]*Hi67[2][2] + Ai06[2][3]*Hi67[3][2];
Ai07[2][3]=Ai06[2][1];
Ai07[2][4]=Ai06[2][4] + Ai06[2][3]*Hi67[3][4];

Ai07[3][1]=Ai06[3][2]*Hi67[2][1] + Ai06[3][3]*Hi67[3][1];
Ai07[3][2]=Ai06[3][2]*Hi67[2][2] + Ai06[3][3]*Hi67[3][2];
Ai07[3][3]=Ai06[3][1];
Ai07[3][4]=Ai06[3][4] + Ai06[3][3]*Hi67[3][4];


Ai08[1][1]=Ai07[1][1]*Hi78[1][1] + Ai07[1][3]*Hi78[3][1];
Ai08[1][2]=Ai07[1][1]*Hi78[1][2] + Ai07[1][3]*Hi78[3][2];
Ai08[1][3]=Ai07[1][2];
Ai08[1][4]=Ai07[1][4];

Ai08[2][1]=Ai07[2][1]*Hi78[1][1] + Ai07[2][3]*Hi78[3][1];
Ai08[2][2]=Ai07[2][1]*Hi78[1][2] + Ai07[2][3]*Hi78[3][2];
Ai08[2][3]=Ai07[2][2];
Ai08[2][4]=Ai07[2][4];

Ai08[3][1]=Ai07[3][1]*Hi78[1][1] + Ai07[3][3]*Hi78[3][1];
Ai08[3][2]=Ai07[3][1]*Hi78[1][2] + Ai07[3][3]*Hi78[3][2];
Ai08[3][3]=Ai07[3][2];
Ai08[3][4]=Ai07[3][4];


Ai09[1][1]=Ai08[1][2]*Hi89[2][1] + Ai08[1][3]*Hi89[3][1];
Ai09[1][2]=Ai08[1][2]*Hi89[2][2] + Ai08[1][3]*Hi89[3][2];
Ai09[1][3]=Ai08[1][1];
Ai09[1][4]=Ai08[1][4] + Ai08[1][2]*Hi89[2][4] + Ai08[1][3]*Hi89[3][4];

Ai09[2][1]=Ai08[2][2]*Hi89[2][1] + Ai08[2][3]*Hi89[3][1];
Ai09[2][2]=Ai08[2][2]*Hi89[2][2] + Ai08[2][3]*Hi89[3][2];
Ai09[2][3]=Ai08[2][1];
Ai09[2][4]=Ai08[2][4] + Ai08[2][2]*Hi89[2][4] + Ai08[2][3]*Hi89[3][4];

Ai09[3][1]=Ai08[3][2]*Hi89[2][1] + Ai08[3][3]*Hi89[3][1];
Ai09[3][2]=Ai08[3][2]*Hi89[2][2] + Ai08[3][3]*Hi89[3][2];
Ai09[3][3]=Ai08[3][1];
Ai09[3][4]=Ai08[3][4] + Ai08[3][2]*Hi89[2][4] + Ai08[3][3]*Hi89[3][4];


Ai010[1][1]=Ai09[1][1]*Hi910[1][1] + Ai09[1][3]*Hi910[3][1];
Ai010[1][2]=Ai09[1][1]*Hi910[1][2] + Ai09[1][3]*Hi910[3][2];
Ai010[1][3]=-Ai09[1][2];
Ai010[1][4]=Ai09[1][4];

Ai010[2][1]=Ai09[2][1]*Hi910[1][1] + Ai09[2][3]*Hi910[3][1];
Ai010[2][2]=Ai09[2][1]*Hi910[1][2] + Ai09[2][3]*Hi910[3][2];
Ai010[2][3]=-Ai09[2][2];
Ai010[2][4]=Ai09[2][4];

Ai010[3][1]=Ai09[3][1]*Hi910[1][1] + Ai09[3][3]*Hi910[3][1];
Ai010[3][2]=Ai09[3][1]*Hi910[1][2] + Ai09[3][3]*Hi910[3][2];
Ai010[3][3]=-Ai09[3][2];
Ai010[3][4]=Ai09[3][4];


Ai011[1][1]=Ai010[1][1]*Hi1011[1][1] + Ai010[1][2]*Hi1011[2][1] + Ai010[1][3]*Hi1011[3][1];
Ai011[1][2]=Ai010[1][1]*Hi1011[1][2] + Ai010[1][2]*Hi1011[2][2] + Ai010[1][3]*Hi1011[3][2];
Ai011[1][3]=Ai010[1][1]*Hi1011[1][3] + Ai010[1][2]*Hi1011[2][3] + Ai010[1][3]*Hi1011[3][3];
Ai011[1][4]=Ai010[1][4] + Ai010[1][1]*Hi1011[1][4] + Ai010[1][2]*Hi1011[2][4] + Ai010[1][3]*Hi1011[3][4];

Ai011[2][1]=Ai010[2][1]*Hi1011[1][1] + Ai010[2][2]*Hi1011[2][1] + Ai010[2][3]*Hi1011[3][1];
Ai011[2][2]=Ai010[2][1]*Hi1011[1][2] + Ai010[2][2]*Hi1011[2][2] + Ai010[2][3]*Hi1011[3][2];
Ai011[2][3]=Ai010[2][1]*Hi1011[1][3] + Ai010[2][2]*Hi1011[2][3] + Ai010[2][3]*Hi1011[3][3];
Ai011[2][4]=Ai010[2][4] + Ai010[2][1]*Hi1011[1][4] + Ai010[2][2]*Hi1011[2][4] + Ai010[2][3]*Hi1011[3][4];

Ai011[3][1]=Ai010[3][1]*Hi1011[1][1] + Ai010[3][2]*Hi1011[2][1] + Ai010[3][3]*Hi1011[3][1];
Ai011[3][2]=Ai010[3][1]*Hi1011[1][2] + Ai010[3][2]*Hi1011[2][2] + Ai010[3][3]*Hi1011[3][2];
Ai011[3][3]=Ai010[3][1]*Hi1011[1][3] + Ai010[3][2]*Hi1011[2][3] + Ai010[3][3]*Hi1011[3][3];
Ai011[3][4]=Ai010[3][4] + Ai010[3][1]*Hi1011[1][4] + Ai010[3][2]*Hi1011[2][4] + Ai010[3][3]*Hi1011[3][4];


Ai012[1][1]=Ai010[1][1]*Hi1012[1][1] + Ai010[1][2]*Hi1012[2][1] + Ai010[1][3]*Hi1012[3][1];
Ai012[1][2]=Ai010[1][1]*Hi1012[1][2] + Ai010[1][2]*Hi1012[2][2] + Ai010[1][3]*Hi1012[3][2];
Ai012[1][3]=Ai010[1][1]*Hi1012[1][3] + Ai010[1][2]*Hi1012[2][3] + Ai010[1][3]*Hi1012[3][3];
Ai012[1][4]=Ai010[1][4] + Ai010[1][1]*Hi1012[1][4] + Ai010[1][2]*Hi1012[2][4] + Ai010[1][3]*Hi1012[3][4];

Ai012[2][1]=Ai010[2][1]*Hi1012[1][1] + Ai010[2][2]*Hi1012[2][1] + Ai010[2][3]*Hi1012[3][1];
Ai012[2][2]=Ai010[2][1]*Hi1012[1][2] + Ai010[2][2]*Hi1012[2][2] + Ai010[2][3]*Hi1012[3][2];
Ai012[2][3]=Ai010[2][1]*Hi1012[1][3] + Ai010[2][2]*Hi1012[2][3] + Ai010[2][3]*Hi1012[3][3];
Ai012[2][4]=Ai010[2][4] + Ai010[2][1]*Hi1012[1][4] + Ai010[2][2]*Hi1012[2][4] + Ai010[2][3]*Hi1012[3][4];

Ai012[3][1]=Ai010[3][1]*Hi1012[1][1] + Ai010[3][2]*Hi1012[2][1] + Ai010[3][3]*Hi1012[3][1];
Ai012[3][2]=Ai010[3][1]*Hi1012[1][2] + Ai010[3][2]*Hi1012[2][2] + Ai010[3][3]*Hi1012[3][2];
Ai012[3][3]=Ai010[3][1]*Hi1012[1][3] + Ai010[3][2]*Hi1012[2][3] + Ai010[3][3]*Hi1012[3][3];
Ai012[3][4]=Ai010[3][4] + Ai010[3][1]*Hi1012[1][4] + Ai010[3][2]*Hi1012[2][4] + Ai010[3][3]*Hi1012[3][4];


Ai013[1][1]=Ai012[1][1]*Hi1213[1][1] + Ai012[1][2]*Hi1213[2][1];
Ai013[1][2]=Ai012[1][1]*Hi1213[1][2] + Ai012[1][2]*Hi1213[2][2];
Ai013[1][3]=Ai012[1][3];
Ai013[1][4]=Ai012[1][4] + Ai012[1][1]*Hi1213[1][4] + Ai012[1][2]*Hi1213[2][4];

Ai013[2][1]=Ai012[2][1]*Hi1213[1][1] + Ai012[2][2]*Hi1213[2][1];
Ai013[2][2]=Ai012[2][1]*Hi1213[1][2] + Ai012[2][2]*Hi1213[2][2];
Ai013[2][3]=Ai012[2][3];
Ai013[2][4]=Ai012[2][4] + Ai012[2][1]*Hi1213[1][4] + Ai012[2][2]*Hi1213[2][4];

Ai013[3][1]=Ai012[3][1]*Hi1213[1][1] + Ai012[3][2]*Hi1213[2][1];
Ai013[3][2]=Ai012[3][1]*Hi1213[1][2] + Ai012[3][2]*Hi1213[2][2];
Ai013[3][3]=Ai012[3][3];
Ai013[3][4]=Ai012[3][4] + Ai012[3][1]*Hi1213[1][4] + Ai012[3][2]*Hi1213[2][4];


Ai014[1][1]=Ai013[1][1]*Hi1314[1][1] + Ai013[1][2]*Hi1314[2][1];
Ai014[1][2]=Ai013[1][1]*Hi1314[1][2] + Ai013[1][2]*Hi1314[2][2];
Ai014[1][3]=Ai013[1][3];
Ai014[1][4]=Ai013[1][4] + Ai013[1][1]*Hi1314[1][4];

Ai014[2][1]=Ai013[2][1]*Hi1314[1][1] + Ai013[2][2]*Hi1314[2][1];
Ai014[2][2]=Ai013[2][1]*Hi1314[1][2] + Ai013[2][2]*Hi1314[2][2];
Ai014[2][3]=Ai013[2][3];
Ai014[2][4]=Ai013[2][4] + Ai013[2][1]*Hi1314[1][4];

Ai014[3][1]=Ai013[3][1]*Hi1314[1][1] + Ai013[3][2]*Hi1314[2][1];
Ai014[3][2]=Ai013[3][1]*Hi1314[1][2] + Ai013[3][2]*Hi1314[2][2];
Ai014[3][3]=Ai013[3][3];
Ai014[3][4]=Ai013[3][4] + Ai013[3][1]*Hi1314[1][4];


Ai015[1][1]=Ai014[1][1];
Ai015[1][2]=Ai014[1][2];
Ai015[1][3]=Ai014[1][3];
Ai015[1][4]=Ai014[1][4] + Ai014[1][1]*Hi1415[1][4];

Ai015[2][1]=Ai014[2][1];
Ai015[2][2]=Ai014[2][2];
Ai015[2][3]=Ai014[2][3];
Ai015[2][4]=Ai014[2][4] + Ai014[2][1]*Hi1415[1][4];

Ai015[3][1]=Ai014[3][1];
Ai015[3][2]=Ai014[3][2];
Ai015[3][3]=Ai014[3][3];
Ai015[3][4]=Ai014[3][4] + Ai014[3][1]*Hi1415[1][4];


Ai016[1][1]=Ai010[1][1]*Hi1016[1][1] + Ai010[1][2]*Hi1016[2][1] + Ai010[1][3]*Hi1016[3][1];
Ai016[1][2]=Ai010[1][1]*Hi1016[1][2] + Ai010[1][2]*Hi1016[2][2] + Ai010[1][3]*Hi1016[3][2];
Ai016[1][3]=Ai010[1][2]*Hi1016[2][3] + Ai010[1][3]*Hi1016[3][3];
Ai016[1][4]=Ai010[1][4] + Ai010[1][1]*Hi1016[1][4] + Ai010[1][2]*Hi1016[2][4] + Ai010[1][3]*Hi1016[3][4];

Ai016[2][1]=Ai010[2][1]*Hi1016[1][1] + Ai010[2][2]*Hi1016[2][1] + Ai010[2][3]*Hi1016[3][1];
Ai016[2][2]=Ai010[2][1]*Hi1016[1][2] + Ai010[2][2]*Hi1016[2][2] + Ai010[2][3]*Hi1016[3][2];
Ai016[2][3]=Ai010[2][2]*Hi1016[2][3] + Ai010[2][3]*Hi1016[3][3];
Ai016[2][4]=Ai010[2][4] + Ai010[2][1]*Hi1016[1][4] + Ai010[2][2]*Hi1016[2][4] + Ai010[2][3]*Hi1016[3][4];

Ai016[3][1]=Ai010[3][1]*Hi1016[1][1] + Ai010[3][2]*Hi1016[2][1] + Ai010[3][3]*Hi1016[3][1];
Ai016[3][2]=Ai010[3][1]*Hi1016[1][2] + Ai010[3][2]*Hi1016[2][2] + Ai010[3][3]*Hi1016[3][2];
Ai016[3][3]=Ai010[3][2]*Hi1016[2][3] + Ai010[3][3]*Hi1016[3][3];
Ai016[3][4]=Ai010[3][4] + Ai010[3][1]*Hi1016[1][4] + Ai010[3][2]*Hi1016[2][4] + Ai010[3][3]*Hi1016[3][4];


Ai017[1][1]=Ai016[1][1]*Hi1617[1][1] + Ai016[1][2]*Hi1617[2][1];
Ai017[1][2]=Ai016[1][1]*Hi1617[1][2] + Ai016[1][2]*Hi1617[2][2];
Ai017[1][3]=Ai016[1][3];
Ai017[1][4]=Ai016[1][4] + Ai016[1][1]*Hi1617[1][4];

Ai017[2][1]=Ai016[2][1]*Hi1617[1][1] + Ai016[2][2]*Hi1617[2][1];
Ai017[2][2]=Ai016[2][1]*Hi1617[1][2] + Ai016[2][2]*Hi1617[2][2];
Ai017[2][3]=Ai016[2][3];
Ai017[2][4]=Ai016[2][4] + Ai016[2][1]*Hi1617[1][4];

Ai017[3][1]=Ai016[3][1]*Hi1617[1][1] + Ai016[3][2]*Hi1617[2][1];
Ai017[3][2]=Ai016[3][1]*Hi1617[1][2] + Ai016[3][2]*Hi1617[2][2];
Ai017[3][3]=Ai016[3][3];
Ai017[3][4]=Ai016[3][4] + Ai016[3][1]*Hi1617[1][4];


Ai018[1][1]=Ai017[1][1]*Hi1718[1][1] + Ai017[1][2]*Hi1718[2][1];
Ai018[1][2]=Ai017[1][1]*Hi1718[1][2] + Ai017[1][2]*Hi1718[2][2];
Ai018[1][3]=Ai017[1][3];
Ai018[1][4]=Ai017[1][4] + Ai017[1][1]*Hi1718[1][4];

Ai018[2][1]=Ai017[2][1]*Hi1718[1][1] + Ai017[2][2]*Hi1718[2][1];
Ai018[2][2]=Ai017[2][1]*Hi1718[1][2] + Ai017[2][2]*Hi1718[2][2];
Ai018[2][3]=Ai017[2][3];
Ai018[2][4]=Ai017[2][4] + Ai017[2][1]*Hi1718[1][4];

Ai018[3][1]=Ai017[3][1]*Hi1718[1][1] + Ai017[3][2]*Hi1718[2][1];
Ai018[3][2]=Ai017[3][1]*Hi1718[1][2] + Ai017[3][2]*Hi1718[2][2];
Ai018[3][3]=Ai017[3][3];
Ai018[3][4]=Ai017[3][4] + Ai017[3][1]*Hi1718[1][4];


Ai019[1][1]=Ai018[1][1];
Ai019[1][2]=Ai018[1][2];
Ai019[1][3]=Ai018[1][3];
Ai019[1][4]=Ai018[1][4] + Ai018[1][1]*Hi1819[1][4];

Ai019[2][1]=Ai018[2][1];
Ai019[2][2]=Ai018[2][2];
Ai019[2][3]=Ai018[2][3];
Ai019[2][4]=Ai018[2][4] + Ai018[2][1]*Hi1819[1][4];

Ai019[3][1]=Ai018[3][1];
Ai019[3][2]=Ai018[3][2];
Ai019[3][3]=Ai018[3][3];
Ai019[3][4]=Ai018[3][4] + Ai018[3][1]*Hi1819[1][4];


Ai020[1][1]=Ai010[1][1]*Hi1020[1][1] + Ai010[1][2]*Hi1020[2][1] + Ai010[1][3]*Hi1020[3][1];
Ai020[1][2]=Ai010[1][1]*Hi1020[1][2] + Ai010[1][2]*Hi1020[2][2] + Ai010[1][3]*Hi1020[3][2];
Ai020[1][3]=Ai010[1][2]*Hi1020[2][3] + Ai010[1][3]*Hi1020[3][3];
Ai020[1][4]=Ai010[1][4] + Ai010[1][1]*Hi1020[1][4] + Ai010[1][2]*Hi1020[2][4] + Ai010[1][3]*Hi1020[3][4];

Ai020[2][1]=Ai010[2][1]*Hi1020[1][1] + Ai010[2][2]*Hi1020[2][1] + Ai010[2][3]*Hi1020[3][1];
Ai020[2][2]=Ai010[2][1]*Hi1020[1][2] + Ai010[2][2]*Hi1020[2][2] + Ai010[2][3]*Hi1020[3][2];
Ai020[2][3]=Ai010[2][2]*Hi1020[2][3] + Ai010[2][3]*Hi1020[3][3];
Ai020[2][4]=Ai010[2][4] + Ai010[2][1]*Hi1020[1][4] + Ai010[2][2]*Hi1020[2][4] + Ai010[2][3]*Hi1020[3][4];

Ai020[3][1]=Ai010[3][1]*Hi1020[1][1] + Ai010[3][2]*Hi1020[2][1] + Ai010[3][3]*Hi1020[3][1];
Ai020[3][2]=Ai010[3][1]*Hi1020[1][2] + Ai010[3][2]*Hi1020[2][2] + Ai010[3][3]*Hi1020[3][2];
Ai020[3][3]=Ai010[3][2]*Hi1020[2][3] + Ai010[3][3]*Hi1020[3][3];
Ai020[3][4]=Ai010[3][4] + Ai010[3][1]*Hi1020[1][4] + Ai010[3][2]*Hi1020[2][4] + Ai010[3][3]*Hi1020[3][4];


Ai021[1][1]=Ai020[1][1]*Hi2021[1][1] + Ai020[1][2]*Hi2021[2][1];
Ai021[1][2]=Ai020[1][1]*Hi2021[1][2] + Ai020[1][2]*Hi2021[2][2];
Ai021[1][3]=Ai020[1][3];
Ai021[1][4]=Ai020[1][4] + Ai020[1][1]*Hi2021[1][4];

Ai021[2][1]=Ai020[2][1]*Hi2021[1][1] + Ai020[2][2]*Hi2021[2][1];
Ai021[2][2]=Ai020[2][1]*Hi2021[1][2] + Ai020[2][2]*Hi2021[2][2];
Ai021[2][3]=Ai020[2][3];
Ai021[2][4]=Ai020[2][4] + Ai020[2][1]*Hi2021[1][4];

Ai021[3][1]=Ai020[3][1]*Hi2021[1][1] + Ai020[3][2]*Hi2021[2][1];
Ai021[3][2]=Ai020[3][1]*Hi2021[1][2] + Ai020[3][2]*Hi2021[2][2];
Ai021[3][3]=Ai020[3][3];
Ai021[3][4]=Ai020[3][4] + Ai020[3][1]*Hi2021[1][4];


Ai022[1][1]=Ai021[1][1]*Hi2122[1][1] + Ai021[1][2]*Hi2122[2][1];
Ai022[1][2]=Ai021[1][1]*Hi2122[1][2] + Ai021[1][2]*Hi2122[2][2];
Ai022[1][3]=Ai021[1][3];
Ai022[1][4]=Ai021[1][4] + Ai021[1][1]*Hi2122[1][4];

Ai022[2][1]=Ai021[2][1]*Hi2122[1][1] + Ai021[2][2]*Hi2122[2][1];
Ai022[2][2]=Ai021[2][1]*Hi2122[1][2] + Ai021[2][2]*Hi2122[2][2];
Ai022[2][3]=Ai021[2][3];
Ai022[2][4]=Ai021[2][4] + Ai021[2][1]*Hi2122[1][4];

Ai022[3][1]=Ai021[3][1]*Hi2122[1][1] + Ai021[3][2]*Hi2122[2][1];
Ai022[3][2]=Ai021[3][1]*Hi2122[1][2] + Ai021[3][2]*Hi2122[2][2];
Ai022[3][3]=Ai021[3][3];
Ai022[3][4]=Ai021[3][4] + Ai021[3][1]*Hi2122[1][4];


Ai023[1][1]=Ai022[1][1];
Ai023[1][2]=Ai022[1][2];
Ai023[1][3]=Ai022[1][3];
Ai023[1][4]=Ai022[1][4] + Ai022[1][1]*Hi2223[1][4];

Ai023[2][1]=Ai022[2][1];
Ai023[2][2]=Ai022[2][2];
Ai023[2][3]=Ai022[2][3];
Ai023[2][4]=Ai022[2][4] + Ai022[2][1]*Hi2223[1][4];

Ai023[3][1]=Ai022[3][1];
Ai023[3][2]=Ai022[3][2];
Ai023[3][3]=Ai022[3][3];
Ai023[3][4]=Ai022[3][4] + Ai022[3][1]*Hi2223[1][4];


Ai024[1][1]=Ai010[1][1]*Hi1024[1][1] + Ai010[1][2]*Hi1024[2][1] + Ai010[1][3]*Hi1024[3][1];
Ai024[1][2]=Ai010[1][1]*Hi1024[1][2] + Ai010[1][2]*Hi1024[2][2] + Ai010[1][3]*Hi1024[3][2];
Ai024[1][3]=Ai010[1][2]*Hi1024[2][3] + Ai010[1][3]*Hi1024[3][3];
Ai024[1][4]=Ai010[1][4] + Ai010[1][1]*Hi1024[1][4] + Ai010[1][2]*Hi1024[2][4] + Ai010[1][3]*Hi1024[3][4];

Ai024[2][1]=Ai010[2][1]*Hi1024[1][1] + Ai010[2][2]*Hi1024[2][1] + Ai010[2][3]*Hi1024[3][1];
Ai024[2][2]=Ai010[2][1]*Hi1024[1][2] + Ai010[2][2]*Hi1024[2][2] + Ai010[2][3]*Hi1024[3][2];
Ai024[2][3]=Ai010[2][2]*Hi1024[2][3] + Ai010[2][3]*Hi1024[3][3];
Ai024[2][4]=Ai010[2][4] + Ai010[2][1]*Hi1024[1][4] + Ai010[2][2]*Hi1024[2][4] + Ai010[2][3]*Hi1024[3][4];

Ai024[3][1]=Ai010[3][1]*Hi1024[1][1] + Ai010[3][2]*Hi1024[2][1] + Ai010[3][3]*Hi1024[3][1];
Ai024[3][2]=Ai010[3][1]*Hi1024[1][2] + Ai010[3][2]*Hi1024[2][2] + Ai010[3][3]*Hi1024[3][2];
Ai024[3][3]=Ai010[3][2]*Hi1024[2][3] + Ai010[3][3]*Hi1024[3][3];
Ai024[3][4]=Ai010[3][4] + Ai010[3][1]*Hi1024[1][4] + Ai010[3][2]*Hi1024[2][4] + Ai010[3][3]*Hi1024[3][4];


Ai025[1][1]=Ai024[1][1]*Hi2425[1][1] + Ai024[1][2]*Hi2425[2][1];
Ai025[1][2]=Ai024[1][1]*Hi2425[1][2] + Ai024[1][2]*Hi2425[2][2];
Ai025[1][3]=Ai024[1][3];
Ai025[1][4]=Ai024[1][4] + Ai024[1][1]*Hi2425[1][4];

Ai025[2][1]=Ai024[2][1]*Hi2425[1][1] + Ai024[2][2]*Hi2425[2][1];
Ai025[2][2]=Ai024[2][1]*Hi2425[1][2] + Ai024[2][2]*Hi2425[2][2];
Ai025[2][3]=Ai024[2][3];
Ai025[2][4]=Ai024[2][4] + Ai024[2][1]*Hi2425[1][4];

Ai025[3][1]=Ai024[3][1]*Hi2425[1][1] + Ai024[3][2]*Hi2425[2][1];
Ai025[3][2]=Ai024[3][1]*Hi2425[1][2] + Ai024[3][2]*Hi2425[2][2];
Ai025[3][3]=Ai024[3][3];
Ai025[3][4]=Ai024[3][4] + Ai024[3][1]*Hi2425[1][4];


Ai026[1][1]=Ai025[1][1]*Hi2526[1][1] + Ai025[1][2]*Hi2526[2][1];
Ai026[1][2]=Ai025[1][1]*Hi2526[1][2] + Ai025[1][2]*Hi2526[2][2];
Ai026[1][3]=Ai025[1][3];
Ai026[1][4]=Ai025[1][4] + Ai025[1][1]*Hi2526[1][4];

Ai026[2][1]=Ai025[2][1]*Hi2526[1][1] + Ai025[2][2]*Hi2526[2][1];
Ai026[2][2]=Ai025[2][1]*Hi2526[1][2] + Ai025[2][2]*Hi2526[2][2];
Ai026[2][3]=Ai025[2][3];
Ai026[2][4]=Ai025[2][4] + Ai025[2][1]*Hi2526[1][4];

Ai026[3][1]=Ai025[3][1]*Hi2526[1][1] + Ai025[3][2]*Hi2526[2][1];
Ai026[3][2]=Ai025[3][1]*Hi2526[1][2] + Ai025[3][2]*Hi2526[2][2];
Ai026[3][3]=Ai025[3][3];
Ai026[3][4]=Ai025[3][4] + Ai025[3][1]*Hi2526[1][4];


Ai027[1][1]=Ai026[1][1];
Ai027[1][2]=Ai026[1][2];
Ai027[1][3]=Ai026[1][3];
Ai027[1][4]=Ai026[1][4] + Ai026[1][1]*Hi2627[1][4];

Ai027[2][1]=Ai026[2][1];
Ai027[2][2]=Ai026[2][2];
Ai027[2][3]=Ai026[2][3];
Ai027[2][4]=Ai026[2][4] + Ai026[2][1]*Hi2627[1][4];

Ai027[3][1]=Ai026[3][1];
Ai027[3][2]=Ai026[3][2];
Ai027[3][3]=Ai026[3][3];
Ai027[3][4]=Ai026[3][4] + Ai026[3][1]*Hi2627[1][4];


Ai028[1][1]=Ai010[1][1]*Hi1028[1][1] + Ai010[1][2]*Hi1028[2][1] + Ai010[1][3]*Hi1028[3][1];
Ai028[1][2]=Ai010[1][1]*Hi1028[1][2] + Ai010[1][2]*Hi1028[2][2] + Ai010[1][3]*Hi1028[3][2];
Ai028[1][3]=Ai010[1][2]*Hi1028[2][3] + Ai010[1][3]*Hi1028[3][3];
Ai028[1][4]=Ai010[1][4] + Ai010[1][1]*Hi1028[1][4] + Ai010[1][2]*Hi1028[2][4] + Ai010[1][3]*Hi1028[3][4];

Ai028[2][1]=Ai010[2][1]*Hi1028[1][1] + Ai010[2][2]*Hi1028[2][1] + Ai010[2][3]*Hi1028[3][1];
Ai028[2][2]=Ai010[2][1]*Hi1028[1][2] + Ai010[2][2]*Hi1028[2][2] + Ai010[2][3]*Hi1028[3][2];
Ai028[2][3]=Ai010[2][2]*Hi1028[2][3] + Ai010[2][3]*Hi1028[3][3];
Ai028[2][4]=Ai010[2][4] + Ai010[2][1]*Hi1028[1][4] + Ai010[2][2]*Hi1028[2][4] + Ai010[2][3]*Hi1028[3][4];

Ai028[3][1]=Ai010[3][1]*Hi1028[1][1] + Ai010[3][2]*Hi1028[2][1] + Ai010[3][3]*Hi1028[3][1];
Ai028[3][2]=Ai010[3][1]*Hi1028[1][2] + Ai010[3][2]*Hi1028[2][2] + Ai010[3][3]*Hi1028[3][2];
Ai028[3][3]=Ai010[3][2]*Hi1028[2][3] + Ai010[3][3]*Hi1028[3][3];
Ai028[3][4]=Ai010[3][4] + Ai010[3][1]*Hi1028[1][4] + Ai010[3][2]*Hi1028[2][4] + Ai010[3][3]*Hi1028[3][4];


Ai029[1][1]=Ai028[1][1]*Hi2829[1][1] + Ai028[1][2]*Hi2829[2][1];
Ai029[1][2]=Ai028[1][1]*Hi2829[1][2] + Ai028[1][2]*Hi2829[2][2];
Ai029[1][3]=Ai028[1][3];
Ai029[1][4]=Ai028[1][4] + Ai028[1][1]*Hi2829[1][4];

Ai029[2][1]=Ai028[2][1]*Hi2829[1][1] + Ai028[2][2]*Hi2829[2][1];
Ai029[2][2]=Ai028[2][1]*Hi2829[1][2] + Ai028[2][2]*Hi2829[2][2];
Ai029[2][3]=Ai028[2][3];
Ai029[2][4]=Ai028[2][4] + Ai028[2][1]*Hi2829[1][4];

Ai029[3][1]=Ai028[3][1]*Hi2829[1][1] + Ai028[3][2]*Hi2829[2][1];
Ai029[3][2]=Ai028[3][1]*Hi2829[1][2] + Ai028[3][2]*Hi2829[2][2];
Ai029[3][3]=Ai028[3][3];
Ai029[3][4]=Ai028[3][4] + Ai028[3][1]*Hi2829[1][4];


Ai030[1][1]=Ai029[1][1]*Hi2930[1][1] + Ai029[1][2]*Hi2930[2][1];
Ai030[1][2]=Ai029[1][1]*Hi2930[1][2] + Ai029[1][2]*Hi2930[2][2];
Ai030[1][3]=Ai029[1][3];
Ai030[1][4]=Ai029[1][4] + Ai029[1][1]*Hi2930[1][4];

Ai030[2][1]=Ai029[2][1]*Hi2930[1][1] + Ai029[2][2]*Hi2930[2][1];
Ai030[2][2]=Ai029[2][1]*Hi2930[1][2] + Ai029[2][2]*Hi2930[2][2];
Ai030[2][3]=Ai029[2][3];
Ai030[2][4]=Ai029[2][4] + Ai029[2][1]*Hi2930[1][4];

Ai030[3][1]=Ai029[3][1]*Hi2930[1][1] + Ai029[3][2]*Hi2930[2][1];
Ai030[3][2]=Ai029[3][1]*Hi2930[1][2] + Ai029[3][2]*Hi2930[2][2];
Ai030[3][3]=Ai029[3][3];
Ai030[3][4]=Ai029[3][4] + Ai029[3][1]*Hi2930[1][4];


Ai031[1][1]=Ai030[1][1];
Ai031[1][2]=Ai030[1][2];
Ai031[1][3]=Ai030[1][3];
Ai031[1][4]=Ai030[1][4] + Ai030[1][1]*Hi3031[1][4];

Ai031[2][1]=Ai030[2][1];
Ai031[2][2]=Ai030[2][2];
Ai031[2][3]=Ai030[2][3];
Ai031[2][4]=Ai030[2][4] + Ai030[2][1]*Hi3031[1][4];

Ai031[3][1]=Ai030[3][1];
Ai031[3][2]=Ai030[3][2];
Ai031[3][3]=Ai030[3][3];
Ai031[3][4]=Ai030[3][4] + Ai030[3][1]*Hi3031[1][4];


Ai032[1][1]=Ai03[1][1]*Hi332[1][1] + Ai03[1][2]*Hi332[2][1] + Ai03[1][3]*Hi332[3][1];
Ai032[1][2]=Ai03[1][1]*Hi332[1][2] + Ai03[1][2]*Hi332[2][2] + Ai03[1][3]*Hi332[3][2];
Ai032[1][3]=-0.7071067811865475*Ai03[1][1] - 0.7071067811865475*Ai03[1][3];
Ai032[1][4]=Ai03[1][4] + Ai03[1][1]*Hi332[1][4];

Ai032[2][1]=Ai03[2][1]*Hi332[1][1] + Ai03[2][2]*Hi332[2][1] + Ai03[2][3]*Hi332[3][1];
Ai032[2][2]=Ai03[2][1]*Hi332[1][2] + Ai03[2][2]*Hi332[2][2] + Ai03[2][3]*Hi332[3][2];
Ai032[2][3]=-0.7071067811865475*Ai03[2][1] - 0.7071067811865475*Ai03[2][3];
Ai032[2][4]=Ai03[2][4] + Ai03[2][1]*Hi332[1][4];

Ai032[3][1]=Ai03[3][1]*Hi332[1][1] + Ai03[3][2]*Hi332[2][1] + Ai03[3][3]*Hi332[3][1];
Ai032[3][2]=Ai03[3][1]*Hi332[1][2] + Ai03[3][2]*Hi332[2][2] + Ai03[3][3]*Hi332[3][2];
Ai032[3][3]=-0.7071067811865475*Ai03[3][1] - 0.7071067811865475*Ai03[3][3];
Ai032[3][4]=Ai03[3][4] + Ai03[3][1]*Hi332[1][4];


Ai033[1][1]=Ai032[1][1]*Hi3233[1][1] + Ai032[1][3]*Hi3233[3][1];
Ai033[1][2]=Ai032[1][1]*Hi3233[1][2] + Ai032[1][3]*Hi3233[3][2];
Ai033[1][3]=Ai032[1][2];
Ai033[1][4]=Ai032[1][4] + Ai032[1][3]*Hi3233[3][4];

Ai033[2][1]=Ai032[2][1]*Hi3233[1][1] + Ai032[2][3]*Hi3233[3][1];
Ai033[2][2]=Ai032[2][1]*Hi3233[1][2] + Ai032[2][3]*Hi3233[3][2];
Ai033[2][3]=Ai032[2][2];
Ai033[2][4]=Ai032[2][4] + Ai032[2][3]*Hi3233[3][4];

Ai033[3][1]=Ai032[3][1]*Hi3233[1][1] + Ai032[3][3]*Hi3233[3][1];
Ai033[3][2]=Ai032[3][1]*Hi3233[1][2] + Ai032[3][3]*Hi3233[3][2];
Ai033[3][3]=Ai032[3][2];
Ai033[3][4]=Ai032[3][4] + Ai032[3][3]*Hi3233[3][4];


Ai034[1][1]=Ai033[1][1]*Hi3334[1][1] + Ai033[1][3]*Hi3334[3][1];
Ai034[1][2]=Ai033[1][1]*Hi3334[1][2] + Ai033[1][3]*Hi3334[3][2];
Ai034[1][3]=-Ai033[1][2];
Ai034[1][4]=Ai033[1][4] + Ai033[1][1]*Hi3334[1][4];

Ai034[2][1]=Ai033[2][1]*Hi3334[1][1] + Ai033[2][3]*Hi3334[3][1];
Ai034[2][2]=Ai033[2][1]*Hi3334[1][2] + Ai033[2][3]*Hi3334[3][2];
Ai034[2][3]=-Ai033[2][2];
Ai034[2][4]=Ai033[2][4] + Ai033[2][1]*Hi3334[1][4];

Ai034[3][1]=Ai033[3][1]*Hi3334[1][1] + Ai033[3][3]*Hi3334[3][1];
Ai034[3][2]=Ai033[3][1]*Hi3334[1][2] + Ai033[3][3]*Hi3334[3][2];
Ai034[3][3]=-Ai033[3][2];
Ai034[3][4]=Ai033[3][4] + Ai033[3][1]*Hi3334[1][4];


Ai035[1][1]=Ai034[1][2]*Hi3435[2][1] + Ai034[1][3]*Hi3435[3][1];
Ai035[1][2]=Ai034[1][2]*Hi3435[2][2] + Ai034[1][3]*Hi3435[3][2];
Ai035[1][3]=-Ai034[1][1];
Ai035[1][4]=Ai034[1][4] + Ai034[1][3]*Hi3435[3][4];

Ai035[2][1]=Ai034[2][2]*Hi3435[2][1] + Ai034[2][3]*Hi3435[3][1];
Ai035[2][2]=Ai034[2][2]*Hi3435[2][2] + Ai034[2][3]*Hi3435[3][2];
Ai035[2][3]=-Ai034[2][1];
Ai035[2][4]=Ai034[2][4] + Ai034[2][3]*Hi3435[3][4];

Ai035[3][1]=Ai034[3][2]*Hi3435[2][1] + Ai034[3][3]*Hi3435[3][1];
Ai035[3][2]=Ai034[3][2]*Hi3435[2][2] + Ai034[3][3]*Hi3435[3][2];
Ai035[3][3]=-Ai034[3][1];
Ai035[3][4]=Ai034[3][4] + Ai034[3][3]*Hi3435[3][4];


Ai036[1][1]=Ai035[1][1]*Hi3536[1][1] + Ai035[1][3]*Hi3536[3][1];
Ai036[1][2]=Ai035[1][1]*Hi3536[1][2] + Ai035[1][3]*Hi3536[3][2];
Ai036[1][3]=-Ai035[1][2];
Ai036[1][4]=Ai035[1][4];

Ai036[2][1]=Ai035[2][1]*Hi3536[1][1] + Ai035[2][3]*Hi3536[3][1];
Ai036[2][2]=Ai035[2][1]*Hi3536[1][2] + Ai035[2][3]*Hi3536[3][2];
Ai036[2][3]=-Ai035[2][2];
Ai036[2][4]=Ai035[2][4];

Ai036[3][1]=Ai035[3][1]*Hi3536[1][1] + Ai035[3][3]*Hi3536[3][1];
Ai036[3][2]=Ai035[3][1]*Hi3536[1][2] + Ai035[3][3]*Hi3536[3][2];
Ai036[3][3]=-Ai035[3][2];
Ai036[3][4]=Ai035[3][4];


Ai037[1][1]=Ai036[1][2]*Hi3637[2][1] + Ai036[1][3]*Hi3637[3][1];
Ai037[1][2]=Ai036[1][2]*Hi3637[2][2] + Ai036[1][3]*Hi3637[3][2];
Ai037[1][3]=-Ai036[1][1];
Ai037[1][4]=Ai036[1][4] + Ai036[1][2]*Hi3637[2][4] + Ai036[1][3]*Hi3637[3][4];

Ai037[2][1]=Ai036[2][2]*Hi3637[2][1] + Ai036[2][3]*Hi3637[3][1];
Ai037[2][2]=Ai036[2][2]*Hi3637[2][2] + Ai036[2][3]*Hi3637[3][2];
Ai037[2][3]=-Ai036[2][1];
Ai037[2][4]=Ai036[2][4] + Ai036[2][2]*Hi3637[2][4] + Ai036[2][3]*Hi3637[3][4];

Ai037[3][1]=Ai036[3][2]*Hi3637[2][1] + Ai036[3][3]*Hi3637[3][1];
Ai037[3][2]=Ai036[3][2]*Hi3637[2][2] + Ai036[3][3]*Hi3637[3][2];
Ai037[3][3]=-Ai036[3][1];
Ai037[3][4]=Ai036[3][4] + Ai036[3][2]*Hi3637[2][4] + Ai036[3][3]*Hi3637[3][4];


Ai038[1][1]=Ai037[1][1]*Hi3738[1][1] + Ai037[1][3]*Hi3738[3][1];
Ai038[1][2]=Ai037[1][1]*Hi3738[1][2] + Ai037[1][3]*Hi3738[3][2];
Ai038[1][3]=Ai037[1][2];
Ai038[1][4]=Ai037[1][4];

Ai038[2][1]=Ai037[2][1]*Hi3738[1][1] + Ai037[2][3]*Hi3738[3][1];
Ai038[2][2]=Ai037[2][1]*Hi3738[1][2] + Ai037[2][3]*Hi3738[3][2];
Ai038[2][3]=Ai037[2][2];
Ai038[2][4]=Ai037[2][4];

Ai038[3][1]=Ai037[3][1]*Hi3738[1][1] + Ai037[3][3]*Hi3738[3][1];
Ai038[3][2]=Ai037[3][1]*Hi3738[1][2] + Ai037[3][3]*Hi3738[3][2];
Ai038[3][3]=Ai037[3][2];
Ai038[3][4]=Ai037[3][4];


Ai039[1][1]=Ai038[1][1]*Hi3839[1][1] + Ai038[1][2]*Hi3839[2][1] + Ai038[1][3]*Hi3839[3][1];
Ai039[1][2]=Ai038[1][1]*Hi3839[1][2] + Ai038[1][2]*Hi3839[2][2] + Ai038[1][3]*Hi3839[3][2];
Ai039[1][3]=Ai038[1][1]*Hi3839[1][3] + Ai038[1][2]*Hi3839[2][3] + Ai038[1][3]*Hi3839[3][3];
Ai039[1][4]=Ai038[1][4] + Ai038[1][1]*Hi3839[1][4] + Ai038[1][2]*Hi3839[2][4] + Ai038[1][3]*Hi3839[3][4];

Ai039[2][1]=Ai038[2][1]*Hi3839[1][1] + Ai038[2][2]*Hi3839[2][1] + Ai038[2][3]*Hi3839[3][1];
Ai039[2][2]=Ai038[2][1]*Hi3839[1][2] + Ai038[2][2]*Hi3839[2][2] + Ai038[2][3]*Hi3839[3][2];
Ai039[2][3]=Ai038[2][1]*Hi3839[1][3] + Ai038[2][2]*Hi3839[2][3] + Ai038[2][3]*Hi3839[3][3];
Ai039[2][4]=Ai038[2][4] + Ai038[2][1]*Hi3839[1][4] + Ai038[2][2]*Hi3839[2][4] + Ai038[2][3]*Hi3839[3][4];

Ai039[3][1]=Ai038[3][1]*Hi3839[1][1] + Ai038[3][2]*Hi3839[2][1] + Ai038[3][3]*Hi3839[3][1];
Ai039[3][2]=Ai038[3][1]*Hi3839[1][2] + Ai038[3][2]*Hi3839[2][2] + Ai038[3][3]*Hi3839[3][2];
Ai039[3][3]=Ai038[3][1]*Hi3839[1][3] + Ai038[3][2]*Hi3839[2][3] + Ai038[3][3]*Hi3839[3][3];
Ai039[3][4]=Ai038[3][4] + Ai038[3][1]*Hi3839[1][4] + Ai038[3][2]*Hi3839[2][4] + Ai038[3][3]*Hi3839[3][4];


Ai040[1][1]=Ai038[1][1]*Hi3840[1][1] + Ai038[1][2]*Hi3840[2][1] + Ai038[1][3]*Hi3840[3][1];
Ai040[1][2]=Ai038[1][1]*Hi3840[1][2] + Ai038[1][2]*Hi3840[2][2] + Ai038[1][3]*Hi3840[3][2];
Ai040[1][3]=Ai038[1][1]*Hi3840[1][3] + Ai038[1][2]*Hi3840[2][3] + Ai038[1][3]*Hi3840[3][3];
Ai040[1][4]=Ai038[1][4] + Ai038[1][1]*Hi3840[1][4] + Ai038[1][2]*Hi3840[2][4] + Ai038[1][3]*Hi3840[3][4];

Ai040[2][1]=Ai038[2][1]*Hi3840[1][1] + Ai038[2][2]*Hi3840[2][1] + Ai038[2][3]*Hi3840[3][1];
Ai040[2][2]=Ai038[2][1]*Hi3840[1][2] + Ai038[2][2]*Hi3840[2][2] + Ai038[2][3]*Hi3840[3][2];
Ai040[2][3]=Ai038[2][1]*Hi3840[1][3] + Ai038[2][2]*Hi3840[2][3] + Ai038[2][3]*Hi3840[3][3];
Ai040[2][4]=Ai038[2][4] + Ai038[2][1]*Hi3840[1][4] + Ai038[2][2]*Hi3840[2][4] + Ai038[2][3]*Hi3840[3][4];

Ai040[3][1]=Ai038[3][1]*Hi3840[1][1] + Ai038[3][2]*Hi3840[2][1] + Ai038[3][3]*Hi3840[3][1];
Ai040[3][2]=Ai038[3][1]*Hi3840[1][2] + Ai038[3][2]*Hi3840[2][2] + Ai038[3][3]*Hi3840[3][2];
Ai040[3][3]=Ai038[3][1]*Hi3840[1][3] + Ai038[3][2]*Hi3840[2][3] + Ai038[3][3]*Hi3840[3][3];
Ai040[3][4]=Ai038[3][4] + Ai038[3][1]*Hi3840[1][4] + Ai038[3][2]*Hi3840[2][4] + Ai038[3][3]*Hi3840[3][4];


Ai041[1][1]=Ai040[1][1]*Hi4041[1][1] + Ai040[1][2]*Hi4041[2][1];
Ai041[1][2]=Ai040[1][1]*Hi4041[1][2] + Ai040[1][2]*Hi4041[2][2];
Ai041[1][3]=Ai040[1][3];
Ai041[1][4]=Ai040[1][4] + Ai040[1][1]*Hi4041[1][4] + Ai040[1][2]*Hi4041[2][4];

Ai041[2][1]=Ai040[2][1]*Hi4041[1][1] + Ai040[2][2]*Hi4041[2][1];
Ai041[2][2]=Ai040[2][1]*Hi4041[1][2] + Ai040[2][2]*Hi4041[2][2];
Ai041[2][3]=Ai040[2][3];
Ai041[2][4]=Ai040[2][4] + Ai040[2][1]*Hi4041[1][4] + Ai040[2][2]*Hi4041[2][4];

Ai041[3][1]=Ai040[3][1]*Hi4041[1][1] + Ai040[3][2]*Hi4041[2][1];
Ai041[3][2]=Ai040[3][1]*Hi4041[1][2] + Ai040[3][2]*Hi4041[2][2];
Ai041[3][3]=Ai040[3][3];
Ai041[3][4]=Ai040[3][4] + Ai040[3][1]*Hi4041[1][4] + Ai040[3][2]*Hi4041[2][4];


Ai042[1][1]=Ai041[1][1]*Hi4142[1][1] + Ai041[1][2]*Hi4142[2][1];
Ai042[1][2]=Ai041[1][1]*Hi4142[1][2] + Ai041[1][2]*Hi4142[2][2];
Ai042[1][3]=Ai041[1][3];
Ai042[1][4]=Ai041[1][4] + Ai041[1][1]*Hi4142[1][4];

Ai042[2][1]=Ai041[2][1]*Hi4142[1][1] + Ai041[2][2]*Hi4142[2][1];
Ai042[2][2]=Ai041[2][1]*Hi4142[1][2] + Ai041[2][2]*Hi4142[2][2];
Ai042[2][3]=Ai041[2][3];
Ai042[2][4]=Ai041[2][4] + Ai041[2][1]*Hi4142[1][4];

Ai042[3][1]=Ai041[3][1]*Hi4142[1][1] + Ai041[3][2]*Hi4142[2][1];
Ai042[3][2]=Ai041[3][1]*Hi4142[1][2] + Ai041[3][2]*Hi4142[2][2];
Ai042[3][3]=Ai041[3][3];
Ai042[3][4]=Ai041[3][4] + Ai041[3][1]*Hi4142[1][4];


Ai043[1][1]=Ai042[1][1];
Ai043[1][2]=Ai042[1][2];
Ai043[1][3]=Ai042[1][3];
Ai043[1][4]=Ai042[1][4] + Ai042[1][1]*Hi4243[1][4];

Ai043[2][1]=Ai042[2][1];
Ai043[2][2]=Ai042[2][2];
Ai043[2][3]=Ai042[2][3];
Ai043[2][4]=Ai042[2][4] + Ai042[2][1]*Hi4243[1][4];

Ai043[3][1]=Ai042[3][1];
Ai043[3][2]=Ai042[3][2];
Ai043[3][3]=Ai042[3][3];
Ai043[3][4]=Ai042[3][4] + Ai042[3][1]*Hi4243[1][4];


Ai044[1][1]=Ai038[1][1]*Hi3844[1][1] + Ai038[1][2]*Hi3844[2][1] + Ai038[1][3]*Hi3844[3][1];
Ai044[1][2]=Ai038[1][1]*Hi3844[1][2] + Ai038[1][2]*Hi3844[2][2] + Ai038[1][3]*Hi3844[3][2];
Ai044[1][3]=Ai038[1][2]*Hi3844[2][3] + Ai038[1][3]*Hi3844[3][3];
Ai044[1][4]=Ai038[1][4] + Ai038[1][1]*Hi3844[1][4] + Ai038[1][2]*Hi3844[2][4] + Ai038[1][3]*Hi3844[3][4];

Ai044[2][1]=Ai038[2][1]*Hi3844[1][1] + Ai038[2][2]*Hi3844[2][1] + Ai038[2][3]*Hi3844[3][1];
Ai044[2][2]=Ai038[2][1]*Hi3844[1][2] + Ai038[2][2]*Hi3844[2][2] + Ai038[2][3]*Hi3844[3][2];
Ai044[2][3]=Ai038[2][2]*Hi3844[2][3] + Ai038[2][3]*Hi3844[3][3];
Ai044[2][4]=Ai038[2][4] + Ai038[2][1]*Hi3844[1][4] + Ai038[2][2]*Hi3844[2][4] + Ai038[2][3]*Hi3844[3][4];

Ai044[3][1]=Ai038[3][1]*Hi3844[1][1] + Ai038[3][2]*Hi3844[2][1] + Ai038[3][3]*Hi3844[3][1];
Ai044[3][2]=Ai038[3][1]*Hi3844[1][2] + Ai038[3][2]*Hi3844[2][2] + Ai038[3][3]*Hi3844[3][2];
Ai044[3][3]=Ai038[3][2]*Hi3844[2][3] + Ai038[3][3]*Hi3844[3][3];
Ai044[3][4]=Ai038[3][4] + Ai038[3][1]*Hi3844[1][4] + Ai038[3][2]*Hi3844[2][4] + Ai038[3][3]*Hi3844[3][4];


Ai045[1][1]=Ai044[1][1]*Hi4445[1][1] + Ai044[1][2]*Hi4445[2][1];
Ai045[1][2]=Ai044[1][1]*Hi4445[1][2] + Ai044[1][2]*Hi4445[2][2];
Ai045[1][3]=Ai044[1][3];
Ai045[1][4]=Ai044[1][4] + Ai044[1][1]*Hi4445[1][4];

Ai045[2][1]=Ai044[2][1]*Hi4445[1][1] + Ai044[2][2]*Hi4445[2][1];
Ai045[2][2]=Ai044[2][1]*Hi4445[1][2] + Ai044[2][2]*Hi4445[2][2];
Ai045[2][3]=Ai044[2][3];
Ai045[2][4]=Ai044[2][4] + Ai044[2][1]*Hi4445[1][4];

Ai045[3][1]=Ai044[3][1]*Hi4445[1][1] + Ai044[3][2]*Hi4445[2][1];
Ai045[3][2]=Ai044[3][1]*Hi4445[1][2] + Ai044[3][2]*Hi4445[2][2];
Ai045[3][3]=Ai044[3][3];
Ai045[3][4]=Ai044[3][4] + Ai044[3][1]*Hi4445[1][4];


Ai046[1][1]=Ai045[1][1]*Hi4546[1][1] + Ai045[1][2]*Hi4546[2][1];
Ai046[1][2]=Ai045[1][1]*Hi4546[1][2] + Ai045[1][2]*Hi4546[2][2];
Ai046[1][3]=Ai045[1][3];
Ai046[1][4]=Ai045[1][4] + Ai045[1][1]*Hi4546[1][4];

Ai046[2][1]=Ai045[2][1]*Hi4546[1][1] + Ai045[2][2]*Hi4546[2][1];
Ai046[2][2]=Ai045[2][1]*Hi4546[1][2] + Ai045[2][2]*Hi4546[2][2];
Ai046[2][3]=Ai045[2][3];
Ai046[2][4]=Ai045[2][4] + Ai045[2][1]*Hi4546[1][4];

Ai046[3][1]=Ai045[3][1]*Hi4546[1][1] + Ai045[3][2]*Hi4546[2][1];
Ai046[3][2]=Ai045[3][1]*Hi4546[1][2] + Ai045[3][2]*Hi4546[2][2];
Ai046[3][3]=Ai045[3][3];
Ai046[3][4]=Ai045[3][4] + Ai045[3][1]*Hi4546[1][4];


Ai047[1][1]=Ai046[1][1];
Ai047[1][2]=Ai046[1][2];
Ai047[1][3]=Ai046[1][3];
Ai047[1][4]=Ai046[1][4] + Ai046[1][1]*Hi4647[1][4];

Ai047[2][1]=Ai046[2][1];
Ai047[2][2]=Ai046[2][2];
Ai047[2][3]=Ai046[2][3];
Ai047[2][4]=Ai046[2][4] + Ai046[2][1]*Hi4647[1][4];

Ai047[3][1]=Ai046[3][1];
Ai047[3][2]=Ai046[3][2];
Ai047[3][3]=Ai046[3][3];
Ai047[3][4]=Ai046[3][4] + Ai046[3][1]*Hi4647[1][4];


Ai048[1][1]=Ai038[1][1]*Hi3848[1][1] + Ai038[1][2]*Hi3848[2][1] + Ai038[1][3]*Hi3848[3][1];
Ai048[1][2]=Ai038[1][1]*Hi3848[1][2] + Ai038[1][2]*Hi3848[2][2] + Ai038[1][3]*Hi3848[3][2];
Ai048[1][3]=Ai038[1][2]*Hi3848[2][3] + Ai038[1][3]*Hi3848[3][3];
Ai048[1][4]=Ai038[1][4] + Ai038[1][1]*Hi3848[1][4] + Ai038[1][2]*Hi3848[2][4] + Ai038[1][3]*Hi3848[3][4];

Ai048[2][1]=Ai038[2][1]*Hi3848[1][1] + Ai038[2][2]*Hi3848[2][1] + Ai038[2][3]*Hi3848[3][1];
Ai048[2][2]=Ai038[2][1]*Hi3848[1][2] + Ai038[2][2]*Hi3848[2][2] + Ai038[2][3]*Hi3848[3][2];
Ai048[2][3]=Ai038[2][2]*Hi3848[2][3] + Ai038[2][3]*Hi3848[3][3];
Ai048[2][4]=Ai038[2][4] + Ai038[2][1]*Hi3848[1][4] + Ai038[2][2]*Hi3848[2][4] + Ai038[2][3]*Hi3848[3][4];

Ai048[3][1]=Ai038[3][1]*Hi3848[1][1] + Ai038[3][2]*Hi3848[2][1] + Ai038[3][3]*Hi3848[3][1];
Ai048[3][2]=Ai038[3][1]*Hi3848[1][2] + Ai038[3][2]*Hi3848[2][2] + Ai038[3][3]*Hi3848[3][2];
Ai048[3][3]=Ai038[3][2]*Hi3848[2][3] + Ai038[3][3]*Hi3848[3][3];
Ai048[3][4]=Ai038[3][4] + Ai038[3][1]*Hi3848[1][4] + Ai038[3][2]*Hi3848[2][4] + Ai038[3][3]*Hi3848[3][4];


Ai049[1][1]=Ai048[1][1]*Hi4849[1][1] + Ai048[1][2]*Hi4849[2][1];
Ai049[1][2]=Ai048[1][1]*Hi4849[1][2] + Ai048[1][2]*Hi4849[2][2];
Ai049[1][3]=Ai048[1][3];
Ai049[1][4]=Ai048[1][4] + Ai048[1][1]*Hi4849[1][4];

Ai049[2][1]=Ai048[2][1]*Hi4849[1][1] + Ai048[2][2]*Hi4849[2][1];
Ai049[2][2]=Ai048[2][1]*Hi4849[1][2] + Ai048[2][2]*Hi4849[2][2];
Ai049[2][3]=Ai048[2][3];
Ai049[2][4]=Ai048[2][4] + Ai048[2][1]*Hi4849[1][4];

Ai049[3][1]=Ai048[3][1]*Hi4849[1][1] + Ai048[3][2]*Hi4849[2][1];
Ai049[3][2]=Ai048[3][1]*Hi4849[1][2] + Ai048[3][2]*Hi4849[2][2];
Ai049[3][3]=Ai048[3][3];
Ai049[3][4]=Ai048[3][4] + Ai048[3][1]*Hi4849[1][4];


Ai050[1][1]=Ai049[1][1]*Hi4950[1][1] + Ai049[1][2]*Hi4950[2][1];
Ai050[1][2]=Ai049[1][1]*Hi4950[1][2] + Ai049[1][2]*Hi4950[2][2];
Ai050[1][3]=Ai049[1][3];
Ai050[1][4]=Ai049[1][4] + Ai049[1][1]*Hi4950[1][4];

Ai050[2][1]=Ai049[2][1]*Hi4950[1][1] + Ai049[2][2]*Hi4950[2][1];
Ai050[2][2]=Ai049[2][1]*Hi4950[1][2] + Ai049[2][2]*Hi4950[2][2];
Ai050[2][3]=Ai049[2][3];
Ai050[2][4]=Ai049[2][4] + Ai049[2][1]*Hi4950[1][4];

Ai050[3][1]=Ai049[3][1]*Hi4950[1][1] + Ai049[3][2]*Hi4950[2][1];
Ai050[3][2]=Ai049[3][1]*Hi4950[1][2] + Ai049[3][2]*Hi4950[2][2];
Ai050[3][3]=Ai049[3][3];
Ai050[3][4]=Ai049[3][4] + Ai049[3][1]*Hi4950[1][4];


Ai051[1][1]=Ai050[1][1];
Ai051[1][2]=Ai050[1][2];
Ai051[1][3]=Ai050[1][3];
Ai051[1][4]=Ai050[1][4] + Ai050[1][1]*Hi5051[1][4];

Ai051[2][1]=Ai050[2][1];
Ai051[2][2]=Ai050[2][2];
Ai051[2][3]=Ai050[2][3];
Ai051[2][4]=Ai050[2][4] + Ai050[2][1]*Hi5051[1][4];

Ai051[3][1]=Ai050[3][1];
Ai051[3][2]=Ai050[3][2];
Ai051[3][3]=Ai050[3][3];
Ai051[3][4]=Ai050[3][4] + Ai050[3][1]*Hi5051[1][4];


Ai052[1][1]=Ai038[1][1]*Hi3852[1][1] + Ai038[1][2]*Hi3852[2][1] + Ai038[1][3]*Hi3852[3][1];
Ai052[1][2]=Ai038[1][1]*Hi3852[1][2] + Ai038[1][2]*Hi3852[2][2] + Ai038[1][3]*Hi3852[3][2];
Ai052[1][3]=Ai038[1][2]*Hi3852[2][3] + Ai038[1][3]*Hi3852[3][3];
Ai052[1][4]=Ai038[1][4] + Ai038[1][1]*Hi3852[1][4] + Ai038[1][2]*Hi3852[2][4] + Ai038[1][3]*Hi3852[3][4];

Ai052[2][1]=Ai038[2][1]*Hi3852[1][1] + Ai038[2][2]*Hi3852[2][1] + Ai038[2][3]*Hi3852[3][1];
Ai052[2][2]=Ai038[2][1]*Hi3852[1][2] + Ai038[2][2]*Hi3852[2][2] + Ai038[2][3]*Hi3852[3][2];
Ai052[2][3]=Ai038[2][2]*Hi3852[2][3] + Ai038[2][3]*Hi3852[3][3];
Ai052[2][4]=Ai038[2][4] + Ai038[2][1]*Hi3852[1][4] + Ai038[2][2]*Hi3852[2][4] + Ai038[2][3]*Hi3852[3][4];

Ai052[3][1]=Ai038[3][1]*Hi3852[1][1] + Ai038[3][2]*Hi3852[2][1] + Ai038[3][3]*Hi3852[3][1];
Ai052[3][2]=Ai038[3][1]*Hi3852[1][2] + Ai038[3][2]*Hi3852[2][2] + Ai038[3][3]*Hi3852[3][2];
Ai052[3][3]=Ai038[3][2]*Hi3852[2][3] + Ai038[3][3]*Hi3852[3][3];
Ai052[3][4]=Ai038[3][4] + Ai038[3][1]*Hi3852[1][4] + Ai038[3][2]*Hi3852[2][4] + Ai038[3][3]*Hi3852[3][4];


Ai053[1][1]=Ai052[1][1]*Hi5253[1][1] + Ai052[1][2]*Hi5253[2][1];
Ai053[1][2]=Ai052[1][1]*Hi5253[1][2] + Ai052[1][2]*Hi5253[2][2];
Ai053[1][3]=Ai052[1][3];
Ai053[1][4]=Ai052[1][4] + Ai052[1][1]*Hi5253[1][4];

Ai053[2][1]=Ai052[2][1]*Hi5253[1][1] + Ai052[2][2]*Hi5253[2][1];
Ai053[2][2]=Ai052[2][1]*Hi5253[1][2] + Ai052[2][2]*Hi5253[2][2];
Ai053[2][3]=Ai052[2][3];
Ai053[2][4]=Ai052[2][4] + Ai052[2][1]*Hi5253[1][4];

Ai053[3][1]=Ai052[3][1]*Hi5253[1][1] + Ai052[3][2]*Hi5253[2][1];
Ai053[3][2]=Ai052[3][1]*Hi5253[1][2] + Ai052[3][2]*Hi5253[2][2];
Ai053[3][3]=Ai052[3][3];
Ai053[3][4]=Ai052[3][4] + Ai052[3][1]*Hi5253[1][4];


Ai054[1][1]=Ai053[1][1]*Hi5354[1][1] + Ai053[1][2]*Hi5354[2][1];
Ai054[1][2]=Ai053[1][1]*Hi5354[1][2] + Ai053[1][2]*Hi5354[2][2];
Ai054[1][3]=Ai053[1][3];
Ai054[1][4]=Ai053[1][4] + Ai053[1][1]*Hi5354[1][4];

Ai054[2][1]=Ai053[2][1]*Hi5354[1][1] + Ai053[2][2]*Hi5354[2][1];
Ai054[2][2]=Ai053[2][1]*Hi5354[1][2] + Ai053[2][2]*Hi5354[2][2];
Ai054[2][3]=Ai053[2][3];
Ai054[2][4]=Ai053[2][4] + Ai053[2][1]*Hi5354[1][4];

Ai054[3][1]=Ai053[3][1]*Hi5354[1][1] + Ai053[3][2]*Hi5354[2][1];
Ai054[3][2]=Ai053[3][1]*Hi5354[1][2] + Ai053[3][2]*Hi5354[2][2];
Ai054[3][3]=Ai053[3][3];
Ai054[3][4]=Ai053[3][4] + Ai053[3][1]*Hi5354[1][4];


Ai055[1][1]=Ai054[1][1];
Ai055[1][2]=Ai054[1][2];
Ai055[1][3]=Ai054[1][3];
Ai055[1][4]=Ai054[1][4] + Ai054[1][1]*Hi5455[1][4];

Ai055[2][1]=Ai054[2][1];
Ai055[2][2]=Ai054[2][2];
Ai055[2][3]=Ai054[2][3];
Ai055[2][4]=Ai054[2][4] + Ai054[2][1]*Hi5455[1][4];

Ai055[3][1]=Ai054[3][1];
Ai055[3][2]=Ai054[3][2];
Ai055[3][3]=Ai054[3][3];
Ai055[3][4]=Ai054[3][4] + Ai054[3][1]*Hi5455[1][4];


Ai056[1][1]=Ai038[1][1]*Hi3856[1][1] + Ai038[1][2]*Hi3856[2][1] + Ai038[1][3]*Hi3856[3][1];
Ai056[1][2]=Ai038[1][1]*Hi3856[1][2] + Ai038[1][2]*Hi3856[2][2] + Ai038[1][3]*Hi3856[3][2];
Ai056[1][3]=Ai038[1][2]*Hi3856[2][3] + Ai038[1][3]*Hi3856[3][3];
Ai056[1][4]=Ai038[1][4] + Ai038[1][1]*Hi3856[1][4] + Ai038[1][2]*Hi3856[2][4] + Ai038[1][3]*Hi3856[3][4];

Ai056[2][1]=Ai038[2][1]*Hi3856[1][1] + Ai038[2][2]*Hi3856[2][1] + Ai038[2][3]*Hi3856[3][1];
Ai056[2][2]=Ai038[2][1]*Hi3856[1][2] + Ai038[2][2]*Hi3856[2][2] + Ai038[2][3]*Hi3856[3][2];
Ai056[2][3]=Ai038[2][2]*Hi3856[2][3] + Ai038[2][3]*Hi3856[3][3];
Ai056[2][4]=Ai038[2][4] + Ai038[2][1]*Hi3856[1][4] + Ai038[2][2]*Hi3856[2][4] + Ai038[2][3]*Hi3856[3][4];

Ai056[3][1]=Ai038[3][1]*Hi3856[1][1] + Ai038[3][2]*Hi3856[2][1] + Ai038[3][3]*Hi3856[3][1];
Ai056[3][2]=Ai038[3][1]*Hi3856[1][2] + Ai038[3][2]*Hi3856[2][2] + Ai038[3][3]*Hi3856[3][2];
Ai056[3][3]=Ai038[3][2]*Hi3856[2][3] + Ai038[3][3]*Hi3856[3][3];
Ai056[3][4]=Ai038[3][4] + Ai038[3][1]*Hi3856[1][4] + Ai038[3][2]*Hi3856[2][4] + Ai038[3][3]*Hi3856[3][4];


Ai057[1][1]=Ai056[1][1]*Hi5657[1][1] + Ai056[1][2]*Hi5657[2][1];
Ai057[1][2]=Ai056[1][1]*Hi5657[1][2] + Ai056[1][2]*Hi5657[2][2];
Ai057[1][3]=Ai056[1][3];
Ai057[1][4]=Ai056[1][4] + Ai056[1][1]*Hi5657[1][4];

Ai057[2][1]=Ai056[2][1]*Hi5657[1][1] + Ai056[2][2]*Hi5657[2][1];
Ai057[2][2]=Ai056[2][1]*Hi5657[1][2] + Ai056[2][2]*Hi5657[2][2];
Ai057[2][3]=Ai056[2][3];
Ai057[2][4]=Ai056[2][4] + Ai056[2][1]*Hi5657[1][4];

Ai057[3][1]=Ai056[3][1]*Hi5657[1][1] + Ai056[3][2]*Hi5657[2][1];
Ai057[3][2]=Ai056[3][1]*Hi5657[1][2] + Ai056[3][2]*Hi5657[2][2];
Ai057[3][3]=Ai056[3][3];
Ai057[3][4]=Ai056[3][4] + Ai056[3][1]*Hi5657[1][4];


Ai058[1][1]=Ai057[1][1]*Hi5758[1][1] + Ai057[1][2]*Hi5758[2][1];
Ai058[1][2]=Ai057[1][1]*Hi5758[1][2] + Ai057[1][2]*Hi5758[2][2];
Ai058[1][3]=Ai057[1][3];
Ai058[1][4]=Ai057[1][4] + Ai057[1][1]*Hi5758[1][4];

Ai058[2][1]=Ai057[2][1]*Hi5758[1][1] + Ai057[2][2]*Hi5758[2][1];
Ai058[2][2]=Ai057[2][1]*Hi5758[1][2] + Ai057[2][2]*Hi5758[2][2];
Ai058[2][3]=Ai057[2][3];
Ai058[2][4]=Ai057[2][4] + Ai057[2][1]*Hi5758[1][4];

Ai058[3][1]=Ai057[3][1]*Hi5758[1][1] + Ai057[3][2]*Hi5758[2][1];
Ai058[3][2]=Ai057[3][1]*Hi5758[1][2] + Ai057[3][2]*Hi5758[2][2];
Ai058[3][3]=Ai057[3][3];
Ai058[3][4]=Ai057[3][4] + Ai057[3][1]*Hi5758[1][4];


Ai059[1][1]=Ai058[1][1];
Ai059[1][2]=Ai058[1][2];
Ai059[1][3]=Ai058[1][3];
Ai059[1][4]=Ai058[1][4] + Ai058[1][1]*Hi5859[1][4];

Ai059[2][1]=Ai058[2][1];
Ai059[2][2]=Ai058[2][2];
Ai059[2][3]=Ai058[2][3];
Ai059[2][4]=Ai058[2][4] + Ai058[2][1]*Hi5859[1][4];

Ai059[3][1]=Ai058[3][1];
Ai059[3][2]=Ai058[3][2];
Ai059[3][3]=Ai058[3][3];
Ai059[3][4]=Ai058[3][4] + Ai058[3][1]*Hi5859[1][4];


Ai060[1][1]=Ai03[1][1]*Hi360[1][1] + Ai03[1][2]*Hi360[2][1];
Ai060[1][2]=Ai03[1][1]*Hi360[1][2] + Ai03[1][2]*Hi360[2][2];
Ai060[1][3]=Ai03[1][3];
Ai060[1][4]=Ai03[1][4] + Ai03[1][1]*Hi360[1][4];

Ai060[2][1]=Ai03[2][1]*Hi360[1][1] + Ai03[2][2]*Hi360[2][1];
Ai060[2][2]=Ai03[2][1]*Hi360[1][2] + Ai03[2][2]*Hi360[2][2];
Ai060[2][3]=Ai03[2][3];
Ai060[2][4]=Ai03[2][4] + Ai03[2][1]*Hi360[1][4];

Ai060[3][1]=Ai03[3][1]*Hi360[1][1] + Ai03[3][2]*Hi360[2][1];
Ai060[3][2]=Ai03[3][1]*Hi360[1][2] + Ai03[3][2]*Hi360[2][2];
Ai060[3][3]=Ai03[3][3];
Ai060[3][4]=Ai03[3][4] + Ai03[3][1]*Hi360[1][4];


Ai061[1][1]=Ai060[1][2]*Hi6061[2][1] + Ai060[1][3]*Hi6061[3][1];
Ai061[1][2]=Ai060[1][2]*Hi6061[2][2] + Ai060[1][3]*Hi6061[3][2];
Ai061[1][3]=Ai060[1][1];
Ai061[1][4]=Ai060[1][4] + Ai060[1][2]*Hi6061[2][4];

Ai061[2][1]=Ai060[2][2]*Hi6061[2][1] + Ai060[2][3]*Hi6061[3][1];
Ai061[2][2]=Ai060[2][2]*Hi6061[2][2] + Ai060[2][3]*Hi6061[3][2];
Ai061[2][3]=Ai060[2][1];
Ai061[2][4]=Ai060[2][4] + Ai060[2][2]*Hi6061[2][4];

Ai061[3][1]=Ai060[3][2]*Hi6061[2][1] + Ai060[3][3]*Hi6061[3][1];
Ai061[3][2]=Ai060[3][2]*Hi6061[2][2] + Ai060[3][3]*Hi6061[3][2];
Ai061[3][3]=Ai060[3][1];
Ai061[3][4]=Ai060[3][4] + Ai060[3][2]*Hi6061[2][4];


Ai062[1][1]=Ai061[1][1]*Hi6162[1][1] + Ai061[1][3]*Hi6162[3][1];
Ai062[1][2]=Ai061[1][1]*Hi6162[1][2] + Ai061[1][3]*Hi6162[3][2];
Ai062[1][3]=Ai061[1][2];
Ai062[1][4]=Ai061[1][4];

Ai062[2][1]=Ai061[2][1]*Hi6162[1][1] + Ai061[2][3]*Hi6162[3][1];
Ai062[2][2]=Ai061[2][1]*Hi6162[1][2] + Ai061[2][3]*Hi6162[3][2];
Ai062[2][3]=Ai061[2][2];
Ai062[2][4]=Ai061[2][4];

Ai062[3][1]=Ai061[3][1]*Hi6162[1][1] + Ai061[3][3]*Hi6162[3][1];
Ai062[3][2]=Ai061[3][1]*Hi6162[1][2] + Ai061[3][3]*Hi6162[3][2];
Ai062[3][3]=Ai061[3][2];
Ai062[3][4]=Ai061[3][4];


Ai063[1][1]=Ai062[1][1]*Hi6263[1][1] + Ai062[1][2]*Hi6263[2][1];
Ai063[1][2]=Ai062[1][1]*Hi6263[1][2] + Ai062[1][2]*Hi6263[2][2];
Ai063[1][3]=Ai062[1][3];
Ai063[1][4]=Ai062[1][4] + Ai062[1][1]*Hi6263[1][4] + Ai062[1][2]*Hi6263[2][4] + Ai062[1][3]*Hi6263[3][4];

Ai063[2][1]=Ai062[2][1]*Hi6263[1][1] + Ai062[2][2]*Hi6263[2][1];
Ai063[2][2]=Ai062[2][1]*Hi6263[1][2] + Ai062[2][2]*Hi6263[2][2];
Ai063[2][3]=Ai062[2][3];
Ai063[2][4]=Ai062[2][4] + Ai062[2][1]*Hi6263[1][4] + Ai062[2][2]*Hi6263[2][4] + Ai062[2][3]*Hi6263[3][4];

Ai063[3][1]=Ai062[3][1]*Hi6263[1][1] + Ai062[3][2]*Hi6263[2][1];
Ai063[3][2]=Ai062[3][1]*Hi6263[1][2] + Ai062[3][2]*Hi6263[2][2];
Ai063[3][3]=Ai062[3][3];
Ai063[3][4]=Ai062[3][4] + Ai062[3][1]*Hi6263[1][4] + Ai062[3][2]*Hi6263[2][4] + Ai062[3][3]*Hi6263[3][4];


Ai064[1][1]=Ai063[1][2]*Hi6364[2][1] + Ai063[1][3]*Hi6364[3][1];
Ai064[1][2]=Ai063[1][2]*Hi6364[2][2] + Ai063[1][3]*Hi6364[3][2];
Ai064[1][3]=-Ai063[1][1];
Ai064[1][4]=Ai063[1][4];

Ai064[2][1]=Ai063[2][2]*Hi6364[2][1] + Ai063[2][3]*Hi6364[3][1];
Ai064[2][2]=Ai063[2][2]*Hi6364[2][2] + Ai063[2][3]*Hi6364[3][2];
Ai064[2][3]=-Ai063[2][1];
Ai064[2][4]=Ai063[2][4];

Ai064[3][1]=Ai063[3][2]*Hi6364[2][1] + Ai063[3][3]*Hi6364[3][1];
Ai064[3][2]=Ai063[3][2]*Hi6364[2][2] + Ai063[3][3]*Hi6364[3][2];
Ai064[3][3]=-Ai063[3][1];
Ai064[3][4]=Ai063[3][4];


Ai065[1][1]=-Ai064[1][3];
Ai065[1][2]=-Ai064[1][2];
Ai065[1][3]=-Ai064[1][1];
Ai065[1][4]=Ai064[1][4] + Ai064[1][2]*Hi6465[2][4];

Ai065[2][1]=-Ai064[2][3];
Ai065[2][2]=-Ai064[2][2];
Ai065[2][3]=-Ai064[2][1];
Ai065[2][4]=Ai064[2][4] + Ai064[2][2]*Hi6465[2][4];

Ai065[3][1]=-Ai064[3][3];
Ai065[3][2]=-Ai064[3][2];
Ai065[3][3]=-Ai064[3][1];
Ai065[3][4]=Ai064[3][4] + Ai064[3][2]*Hi6465[2][4];


Ai066[1][1]=Ai062[1][1]*Hi6266[1][1] + Ai062[1][2]*Hi6266[2][1];
Ai066[1][2]=Ai062[1][1]*Hi6266[1][2] + Ai062[1][2]*Hi6266[2][2];
Ai066[1][3]=Ai062[1][3];
Ai066[1][4]=Ai062[1][4] + Ai062[1][1]*Hi6266[1][4] + Ai062[1][2]*Hi6266[2][4] + Ai062[1][3]*Hi6266[3][4];

Ai066[2][1]=Ai062[2][1]*Hi6266[1][1] + Ai062[2][2]*Hi6266[2][1];
Ai066[2][2]=Ai062[2][1]*Hi6266[1][2] + Ai062[2][2]*Hi6266[2][2];
Ai066[2][3]=Ai062[2][3];
Ai066[2][4]=Ai062[2][4] + Ai062[2][1]*Hi6266[1][4] + Ai062[2][2]*Hi6266[2][4] + Ai062[2][3]*Hi6266[3][4];

Ai066[3][1]=Ai062[3][1]*Hi6266[1][1] + Ai062[3][2]*Hi6266[2][1];
Ai066[3][2]=Ai062[3][1]*Hi6266[1][2] + Ai062[3][2]*Hi6266[2][2];
Ai066[3][3]=Ai062[3][3];
Ai066[3][4]=Ai062[3][4] + Ai062[3][1]*Hi6266[1][4] + Ai062[3][2]*Hi6266[2][4] + Ai062[3][3]*Hi6266[3][4];


Ai067[1][1]=Ai066[1][2]*Hi6667[2][1] + Ai066[1][3]*Hi6667[3][1];
Ai067[1][2]=Ai066[1][2]*Hi6667[2][2] + Ai066[1][3]*Hi6667[3][2];
Ai067[1][3]=-Ai066[1][1];
Ai067[1][4]=Ai066[1][4];

Ai067[2][1]=Ai066[2][2]*Hi6667[2][1] + Ai066[2][3]*Hi6667[3][1];
Ai067[2][2]=Ai066[2][2]*Hi6667[2][2] + Ai066[2][3]*Hi6667[3][2];
Ai067[2][3]=-Ai066[2][1];
Ai067[2][4]=Ai066[2][4];

Ai067[3][1]=Ai066[3][2]*Hi6667[2][1] + Ai066[3][3]*Hi6667[3][1];
Ai067[3][2]=Ai066[3][2]*Hi6667[2][2] + Ai066[3][3]*Hi6667[3][2];
Ai067[3][3]=-Ai066[3][1];
Ai067[3][4]=Ai066[3][4];


Ai068[1][1]=-Ai067[1][3];
Ai068[1][2]=-Ai067[1][2];
Ai068[1][3]=-Ai067[1][1];
Ai068[1][4]=Ai067[1][4] + Ai067[1][2]*Hi6768[2][4];

Ai068[2][1]=-Ai067[2][3];
Ai068[2][2]=-Ai067[2][2];
Ai068[2][3]=-Ai067[2][1];
Ai068[2][4]=Ai067[2][4] + Ai067[2][2]*Hi6768[2][4];

Ai068[3][1]=-Ai067[3][3];
Ai068[3][2]=-Ai067[3][2];
Ai068[3][3]=-Ai067[3][1];
Ai068[3][4]=Ai067[3][4] + Ai067[3][2]*Hi6768[2][4];


Ai069[1][1]=Ai062[1][1];
Ai069[1][2]=-Ai062[1][2];
Ai069[1][3]=-Ai062[1][3];
Ai069[1][4]=Ai062[1][4] + Ai062[1][3]*Hi6269[3][4];

Ai069[2][1]=Ai062[2][1];
Ai069[2][2]=-Ai062[2][2];
Ai069[2][3]=-Ai062[2][3];
Ai069[2][4]=Ai062[2][4] + Ai062[2][3]*Hi6269[3][4];

Ai069[3][1]=Ai062[3][1];
Ai069[3][2]=-Ai062[3][2];
Ai069[3][3]=-Ai062[3][3];
Ai069[3][4]=Ai062[3][4] + Ai062[3][3]*Hi6269[3][4];


Ai070[1][1]=Hi00[1][1]*Hi070[1][1] + Hi00[1][3]*Hi070[3][1];
Ai070[1][2]=Hi00[1][1]*Hi070[1][2] + Hi00[1][3]*Hi070[3][2];
Ai070[1][3]=Hi00[1][2];
Ai070[1][4]=Hi00[1][4] + Hi00[1][1]*Hi070[1][4];

Ai070[2][1]=Hi00[2][1]*Hi070[1][1] + Hi00[2][3]*Hi070[3][1];
Ai070[2][2]=Hi00[2][1]*Hi070[1][2] + Hi00[2][3]*Hi070[3][2];
Ai070[2][3]=Hi00[2][2];
Ai070[2][4]=Hi00[2][4] + Hi00[2][1]*Hi070[1][4];

Ai070[3][1]=Hi00[3][1]*Hi070[1][1] + Hi00[3][3]*Hi070[3][1];
Ai070[3][2]=Hi00[3][1]*Hi070[1][2] + Hi00[3][3]*Hi070[3][2];
Ai070[3][3]=Hi00[3][2];
Ai070[3][4]=Hi00[3][4] + Hi00[3][1]*Hi070[1][4];


Ai071[1][1]=Ai070[1][1]*Hi7071[1][1] + Ai070[1][3]*Hi7071[3][1];
Ai071[1][2]=Ai070[1][1]*Hi7071[1][2] + Ai070[1][3]*Hi7071[3][2];
Ai071[1][3]=-Ai070[1][2];
Ai071[1][4]=Ai070[1][4];

Ai071[2][1]=Ai070[2][1]*Hi7071[1][1] + Ai070[2][3]*Hi7071[3][1];
Ai071[2][2]=Ai070[2][1]*Hi7071[1][2] + Ai070[2][3]*Hi7071[3][2];
Ai071[2][3]=-Ai070[2][2];
Ai071[2][4]=Ai070[2][4];

Ai071[3][1]=Ai070[3][1]*Hi7071[1][1] + Ai070[3][3]*Hi7071[3][1];
Ai071[3][2]=Ai070[3][1]*Hi7071[1][2] + Ai070[3][3]*Hi7071[3][2];
Ai071[3][3]=-Ai070[3][2];
Ai071[3][4]=Ai070[3][4];


Ai072[1][1]=Ai071[1][1]*Hi7172[1][1] + Ai071[1][2]*Hi7172[2][1] + Ai071[1][3]*Hi7172[3][1];
Ai072[1][2]=Ai071[1][1]*Hi7172[1][2] + Ai071[1][2]*Hi7172[2][2] + Ai071[1][3]*Hi7172[3][2];
Ai072[1][3]=Ai071[1][1]*Hi7172[1][3] + Ai071[1][2]*Hi7172[2][3];
Ai072[1][4]=Ai071[1][4] + Ai071[1][1]*Hi7172[1][4];

Ai072[2][1]=Ai071[2][1]*Hi7172[1][1] + Ai071[2][2]*Hi7172[2][1] + Ai071[2][3]*Hi7172[3][1];
Ai072[2][2]=Ai071[2][1]*Hi7172[1][2] + Ai071[2][2]*Hi7172[2][2] + Ai071[2][3]*Hi7172[3][2];
Ai072[2][3]=Ai071[2][1]*Hi7172[1][3] + Ai071[2][2]*Hi7172[2][3];
Ai072[2][4]=Ai071[2][4] + Ai071[2][1]*Hi7172[1][4];

Ai072[3][1]=Ai071[3][1]*Hi7172[1][1] + Ai071[3][2]*Hi7172[2][1] + Ai071[3][3]*Hi7172[3][1];
Ai072[3][2]=Ai071[3][1]*Hi7172[1][2] + Ai071[3][2]*Hi7172[2][2] + Ai071[3][3]*Hi7172[3][2];
Ai072[3][3]=Ai071[3][1]*Hi7172[1][3] + Ai071[3][2]*Hi7172[2][3];
Ai072[3][4]=Ai071[3][4] + Ai071[3][1]*Hi7172[1][4];


Ai073[1][1]=Ai072[1][1]*Hi7273[1][1] + Ai072[1][3]*Hi7273[3][1];
Ai073[1][2]=Ai072[1][1]*Hi7273[1][2] + Ai072[1][3]*Hi7273[3][2];
Ai073[1][3]=Ai072[1][2];
Ai073[1][4]=Ai072[1][4] + Ai072[1][1]*Hi7273[1][4] + Ai072[1][3]*Hi7273[3][4];

Ai073[2][1]=Ai072[2][1]*Hi7273[1][1] + Ai072[2][3]*Hi7273[3][1];
Ai073[2][2]=Ai072[2][1]*Hi7273[1][2] + Ai072[2][3]*Hi7273[3][2];
Ai073[2][3]=Ai072[2][2];
Ai073[2][4]=Ai072[2][4] + Ai072[2][1]*Hi7273[1][4] + Ai072[2][3]*Hi7273[3][4];

Ai073[3][1]=Ai072[3][1]*Hi7273[1][1] + Ai072[3][3]*Hi7273[3][1];
Ai073[3][2]=Ai072[3][1]*Hi7273[1][2] + Ai072[3][3]*Hi7273[3][2];
Ai073[3][3]=Ai072[3][2];
Ai073[3][4]=Ai072[3][4] + Ai072[3][1]*Hi7273[1][4] + Ai072[3][3]*Hi7273[3][4];


Ai074[1][1]=Ai073[1][1]*Hi7374[1][1] + Ai073[1][3]*Hi7374[3][1];
Ai074[1][2]=Ai073[1][1]*Hi7374[1][2] + Ai073[1][3]*Hi7374[3][2];
Ai074[1][3]=-Ai073[1][2];
Ai074[1][4]=Ai073[1][4];

Ai074[2][1]=Ai073[2][1]*Hi7374[1][1] + Ai073[2][3]*Hi7374[3][1];
Ai074[2][2]=Ai073[2][1]*Hi7374[1][2] + Ai073[2][3]*Hi7374[3][2];
Ai074[2][3]=-Ai073[2][2];
Ai074[2][4]=Ai073[2][4];

Ai074[3][1]=Ai073[3][1]*Hi7374[1][1] + Ai073[3][3]*Hi7374[3][1];
Ai074[3][2]=Ai073[3][1]*Hi7374[1][2] + Ai073[3][3]*Hi7374[3][2];
Ai074[3][3]=-Ai073[3][2];
Ai074[3][4]=Ai073[3][4];


Ai075[1][1]=Ai074[1][1]*Hi7475[1][1] + Ai074[1][3]*Hi7475[3][1];
Ai075[1][2]=Ai074[1][1]*Hi7475[1][2] + Ai074[1][3]*Hi7475[3][2];
Ai075[1][3]=-Ai074[1][2];
Ai075[1][4]=Ai074[1][4] + Ai074[1][3]*Hi7475[3][4];

Ai075[2][1]=Ai074[2][1]*Hi7475[1][1] + Ai074[2][3]*Hi7475[3][1];
Ai075[2][2]=Ai074[2][1]*Hi7475[1][2] + Ai074[2][3]*Hi7475[3][2];
Ai075[2][3]=-Ai074[2][2];
Ai075[2][4]=Ai074[2][4] + Ai074[2][3]*Hi7475[3][4];

Ai075[3][1]=Ai074[3][1]*Hi7475[1][1] + Ai074[3][3]*Hi7475[3][1];
Ai075[3][2]=Ai074[3][1]*Hi7475[1][2] + Ai074[3][3]*Hi7475[3][2];
Ai075[3][3]=-Ai074[3][2];
Ai075[3][4]=Ai074[3][4] + Ai074[3][3]*Hi7475[3][4];


Ai076[1][1]=Ai075[1][1]*Hi7576[1][1] + Ai075[1][3]*Hi7576[3][1];
Ai076[1][2]=Ai075[1][1]*Hi7576[1][2] + Ai075[1][3]*Hi7576[3][2];
Ai076[1][3]=Ai075[1][2];
Ai076[1][4]=Ai075[1][4];

Ai076[2][1]=Ai075[2][1]*Hi7576[1][1] + Ai075[2][3]*Hi7576[3][1];
Ai076[2][2]=Ai075[2][1]*Hi7576[1][2] + Ai075[2][3]*Hi7576[3][2];
Ai076[2][3]=Ai075[2][2];
Ai076[2][4]=Ai075[2][4];

Ai076[3][1]=Ai075[3][1]*Hi7576[1][1] + Ai075[3][3]*Hi7576[3][1];
Ai076[3][2]=Ai075[3][1]*Hi7576[1][2] + Ai075[3][3]*Hi7576[3][2];
Ai076[3][3]=Ai075[3][2];
Ai076[3][4]=Ai075[3][4];


Ai077[1][1]=Ai076[1][1];
Ai077[1][2]=Ai076[1][2];
Ai077[1][3]=Ai076[1][3];
Ai077[1][4]=Ai076[1][4] + Ai076[1][1]*Hi7677[1][4] + Ai076[1][2]*Hi7677[2][4] + Ai076[1][3]*Hi7677[3][4];

Ai077[2][1]=Ai076[2][1];
Ai077[2][2]=Ai076[2][2];
Ai077[2][3]=Ai076[2][3];
Ai077[2][4]=Ai076[2][4] + Ai076[2][1]*Hi7677[1][4] + Ai076[2][2]*Hi7677[2][4] + Ai076[2][3]*Hi7677[3][4];

Ai077[3][1]=Ai076[3][1];
Ai077[3][2]=Ai076[3][2];
Ai077[3][3]=Ai076[3][3];
Ai077[3][4]=Ai076[3][4] + Ai076[3][1]*Hi7677[1][4] + Ai076[3][2]*Hi7677[2][4] + Ai076[3][3]*Hi7677[3][4];


Ai078[1][1]=Ai076[1][1];
Ai078[1][2]=Ai076[1][2];
Ai078[1][3]=Ai076[1][3];
Ai078[1][4]=Ai076[1][4] + Ai076[1][1]*Hi7678[1][4] + Ai076[1][2]*Hi7678[2][4] + Ai076[1][3]*Hi7678[3][4];

Ai078[2][1]=Ai076[2][1];
Ai078[2][2]=Ai076[2][2];
Ai078[2][3]=Ai076[2][3];
Ai078[2][4]=Ai076[2][4] + Ai076[2][1]*Hi7678[1][4] + Ai076[2][2]*Hi7678[2][4] + Ai076[2][3]*Hi7678[3][4];

Ai078[3][1]=Ai076[3][1];
Ai078[3][2]=Ai076[3][2];
Ai078[3][3]=Ai076[3][3];
Ai078[3][4]=Ai076[3][4] + Ai076[3][1]*Hi7678[1][4] + Ai076[3][2]*Hi7678[2][4] + Ai076[3][3]*Hi7678[3][4];


Ai079[1][1]=Ai076[1][1];
Ai079[1][2]=Ai076[1][2];
Ai079[1][3]=Ai076[1][3];
Ai079[1][4]=Ai076[1][4] + Ai076[1][1]*Hi7679[1][4] + Ai076[1][2]*Hi7679[2][4] + Ai076[1][3]*Hi7679[3][4];

Ai079[2][1]=Ai076[2][1];
Ai079[2][2]=Ai076[2][2];
Ai079[2][3]=Ai076[2][3];
Ai079[2][4]=Ai076[2][4] + Ai076[2][1]*Hi7679[1][4] + Ai076[2][2]*Hi7679[2][4] + Ai076[2][3]*Hi7679[3][4];

Ai079[3][1]=Ai076[3][1];
Ai079[3][2]=Ai076[3][2];
Ai079[3][3]=Ai076[3][3];
Ai079[3][4]=Ai076[3][4] + Ai076[3][1]*Hi7679[1][4] + Ai076[3][2]*Hi7679[2][4] + Ai076[3][3]*Hi7679[3][4];


Ai080[1][1]=Ai076[1][1];
Ai080[1][2]=Ai076[1][2];
Ai080[1][3]=Ai076[1][3];
Ai080[1][4]=Ai076[1][4] + Ai076[1][1]*Hi7680[1][4] + Ai076[1][2]*Hi7680[2][4] + Ai076[1][3]*Hi7680[3][4];

Ai080[2][1]=Ai076[2][1];
Ai080[2][2]=Ai076[2][2];
Ai080[2][3]=Ai076[2][3];
Ai080[2][4]=Ai076[2][4] + Ai076[2][1]*Hi7680[1][4] + Ai076[2][2]*Hi7680[2][4] + Ai076[2][3]*Hi7680[3][4];

Ai080[3][1]=Ai076[3][1];
Ai080[3][2]=Ai076[3][2];
Ai080[3][3]=Ai076[3][3];
Ai080[3][4]=Ai076[3][4] + Ai076[3][1]*Hi7680[1][4] + Ai076[3][2]*Hi7680[2][4] + Ai076[3][3]*Hi7680[3][4];


Ai081[1][1]=Ai076[1][1];
Ai081[1][2]=Ai076[1][2];
Ai081[1][3]=Ai076[1][3];
Ai081[1][4]=Ai076[1][4] + Ai076[1][1]*Hi7681[1][4] + Ai076[1][2]*Hi7681[2][4] + Ai076[1][3]*Hi7681[3][4];

Ai081[2][1]=Ai076[2][1];
Ai081[2][2]=Ai076[2][2];
Ai081[2][3]=Ai076[2][3];
Ai081[2][4]=Ai076[2][4] + Ai076[2][1]*Hi7681[1][4] + Ai076[2][2]*Hi7681[2][4] + Ai076[2][3]*Hi7681[3][4];

Ai081[3][1]=Ai076[3][1];
Ai081[3][2]=Ai076[3][2];
Ai081[3][3]=Ai076[3][3];
Ai081[3][4]=Ai076[3][4] + Ai076[3][1]*Hi7681[1][4] + Ai076[3][2]*Hi7681[2][4] + Ai076[3][3]*Hi7681[3][4];


Ai082[1][1]=Ai076[1][1];
Ai082[1][2]=Ai076[1][2];
Ai082[1][3]=Ai076[1][3];
Ai082[1][4]=Ai076[1][4] + Ai076[1][1]*Hi7682[1][4] + Ai076[1][2]*Hi7682[2][4] + Ai076[1][3]*Hi7682[3][4];

Ai082[2][1]=Ai076[2][1];
Ai082[2][2]=Ai076[2][2];
Ai082[2][3]=Ai076[2][3];
Ai082[2][4]=Ai076[2][4] + Ai076[2][1]*Hi7682[1][4] + Ai076[2][2]*Hi7682[2][4] + Ai076[2][3]*Hi7682[3][4];

Ai082[3][1]=Ai076[3][1];
Ai082[3][2]=Ai076[3][2];
Ai082[3][3]=Ai076[3][3];
Ai082[3][4]=Ai076[3][4] + Ai076[3][1]*Hi7682[1][4] + Ai076[3][2]*Hi7682[2][4] + Ai076[3][3]*Hi7682[3][4];


Ai083[1][1]=Ai076[1][1]*Hi7683[1][1] + Ai076[1][2]*Hi7683[2][1] + Ai076[1][3]*Hi7683[3][1];
Ai083[1][2]=Ai076[1][1]*Hi7683[1][2] + Ai076[1][2]*Hi7683[2][2] + Ai076[1][3]*Hi7683[3][2];
Ai083[1][3]=Ai076[1][1]*Hi7683[1][3] + Ai076[1][2]*Hi7683[2][3] + Ai076[1][3]*Hi7683[3][3];
Ai083[1][4]=Ai076[1][4] + Ai076[1][1]*Hi7683[1][4] + Ai076[1][2]*Hi7683[2][4] + Ai076[1][3]*Hi7683[3][4];

Ai083[2][1]=Ai076[2][1]*Hi7683[1][1] + Ai076[2][2]*Hi7683[2][1] + Ai076[2][3]*Hi7683[3][1];
Ai083[2][2]=Ai076[2][1]*Hi7683[1][2] + Ai076[2][2]*Hi7683[2][2] + Ai076[2][3]*Hi7683[3][2];
Ai083[2][3]=Ai076[2][1]*Hi7683[1][3] + Ai076[2][2]*Hi7683[2][3] + Ai076[2][3]*Hi7683[3][3];
Ai083[2][4]=Ai076[2][4] + Ai076[2][1]*Hi7683[1][4] + Ai076[2][2]*Hi7683[2][4] + Ai076[2][3]*Hi7683[3][4];

Ai083[3][1]=Ai076[3][1]*Hi7683[1][1] + Ai076[3][2]*Hi7683[2][1] + Ai076[3][3]*Hi7683[3][1];
Ai083[3][2]=Ai076[3][1]*Hi7683[1][2] + Ai076[3][2]*Hi7683[2][2] + Ai076[3][3]*Hi7683[3][2];
Ai083[3][3]=Ai076[3][1]*Hi7683[1][3] + Ai076[3][2]*Hi7683[2][3] + Ai076[3][3]*Hi7683[3][3];
Ai083[3][4]=Ai076[3][4] + Ai076[3][1]*Hi7683[1][4] + Ai076[3][2]*Hi7683[2][4] + Ai076[3][3]*Hi7683[3][4];


Ai084[1][1]=Hi00[1][1]*Hi084[1][1] + Hi00[1][3]*Hi084[3][1];
Ai084[1][2]=Hi00[1][1]*Hi084[1][2] + Hi00[1][3]*Hi084[3][2];
Ai084[1][3]=-Hi00[1][2];
Ai084[1][4]=Hi00[1][4] + Hi00[1][1]*Hi084[1][4];

Ai084[2][1]=Hi00[2][1]*Hi084[1][1] + Hi00[2][3]*Hi084[3][1];
Ai084[2][2]=Hi00[2][1]*Hi084[1][2] + Hi00[2][3]*Hi084[3][2];
Ai084[2][3]=-Hi00[2][2];
Ai084[2][4]=Hi00[2][4] + Hi00[2][1]*Hi084[1][4];

Ai084[3][1]=Hi00[3][1]*Hi084[1][1] + Hi00[3][3]*Hi084[3][1];
Ai084[3][2]=Hi00[3][1]*Hi084[1][2] + Hi00[3][3]*Hi084[3][2];
Ai084[3][3]=-Hi00[3][2];
Ai084[3][4]=Hi00[3][4] + Hi00[3][1]*Hi084[1][4];


Ai085[1][1]=Ai084[1][1]*Hi8485[1][1] + Ai084[1][3]*Hi8485[3][1];
Ai085[1][2]=Ai084[1][1]*Hi8485[1][2] + Ai084[1][3]*Hi8485[3][2];
Ai085[1][3]=Ai084[1][2];
Ai085[1][4]=Ai084[1][4];

Ai085[2][1]=Ai084[2][1]*Hi8485[1][1] + Ai084[2][3]*Hi8485[3][1];
Ai085[2][2]=Ai084[2][1]*Hi8485[1][2] + Ai084[2][3]*Hi8485[3][2];
Ai085[2][3]=Ai084[2][2];
Ai085[2][4]=Ai084[2][4];

Ai085[3][1]=Ai084[3][1]*Hi8485[1][1] + Ai084[3][3]*Hi8485[3][1];
Ai085[3][2]=Ai084[3][1]*Hi8485[1][2] + Ai084[3][3]*Hi8485[3][2];
Ai085[3][3]=Ai084[3][2];
Ai085[3][4]=Ai084[3][4];


Ai086[1][1]=Ai085[1][1]*Hi8586[1][1] + Ai085[1][2]*Hi8586[2][1] + Ai085[1][3]*Hi8586[3][1];
Ai086[1][2]=Ai085[1][1]*Hi8586[1][2] + Ai085[1][2]*Hi8586[2][2] + Ai085[1][3]*Hi8586[3][2];
Ai086[1][3]=Ai085[1][1]*Hi8586[1][3] + Ai085[1][2]*Hi8586[2][3];
Ai086[1][4]=Ai085[1][4] + Ai085[1][1]*Hi8586[1][4];

Ai086[2][1]=Ai085[2][1]*Hi8586[1][1] + Ai085[2][2]*Hi8586[2][1] + Ai085[2][3]*Hi8586[3][1];
Ai086[2][2]=Ai085[2][1]*Hi8586[1][2] + Ai085[2][2]*Hi8586[2][2] + Ai085[2][3]*Hi8586[3][2];
Ai086[2][3]=Ai085[2][1]*Hi8586[1][3] + Ai085[2][2]*Hi8586[2][3];
Ai086[2][4]=Ai085[2][4] + Ai085[2][1]*Hi8586[1][4];

Ai086[3][1]=Ai085[3][1]*Hi8586[1][1] + Ai085[3][2]*Hi8586[2][1] + Ai085[3][3]*Hi8586[3][1];
Ai086[3][2]=Ai085[3][1]*Hi8586[1][2] + Ai085[3][2]*Hi8586[2][2] + Ai085[3][3]*Hi8586[3][2];
Ai086[3][3]=Ai085[3][1]*Hi8586[1][3] + Ai085[3][2]*Hi8586[2][3];
Ai086[3][4]=Ai085[3][4] + Ai085[3][1]*Hi8586[1][4];


Ai087[1][1]=Ai086[1][1]*Hi8687[1][1] + Ai086[1][3]*Hi8687[3][1];
Ai087[1][2]=Ai086[1][1]*Hi8687[1][2] + Ai086[1][3]*Hi8687[3][2];
Ai087[1][3]=-Ai086[1][2];
Ai087[1][4]=Ai086[1][4] + Ai086[1][1]*Hi8687[1][4] + Ai086[1][3]*Hi8687[3][4];

Ai087[2][1]=Ai086[2][1]*Hi8687[1][1] + Ai086[2][3]*Hi8687[3][1];
Ai087[2][2]=Ai086[2][1]*Hi8687[1][2] + Ai086[2][3]*Hi8687[3][2];
Ai087[2][3]=-Ai086[2][2];
Ai087[2][4]=Ai086[2][4] + Ai086[2][1]*Hi8687[1][4] + Ai086[2][3]*Hi8687[3][4];

Ai087[3][1]=Ai086[3][1]*Hi8687[1][1] + Ai086[3][3]*Hi8687[3][1];
Ai087[3][2]=Ai086[3][1]*Hi8687[1][2] + Ai086[3][3]*Hi8687[3][2];
Ai087[3][3]=-Ai086[3][2];
Ai087[3][4]=Ai086[3][4] + Ai086[3][1]*Hi8687[1][4] + Ai086[3][3]*Hi8687[3][4];


Ai088[1][1]=Ai087[1][1]*Hi8788[1][1] + Ai087[1][3]*Hi8788[3][1];
Ai088[1][2]=Ai087[1][1]*Hi8788[1][2] + Ai087[1][3]*Hi8788[3][2];
Ai088[1][3]=Ai087[1][2];
Ai088[1][4]=Ai087[1][4];

Ai088[2][1]=Ai087[2][1]*Hi8788[1][1] + Ai087[2][3]*Hi8788[3][1];
Ai088[2][2]=Ai087[2][1]*Hi8788[1][2] + Ai087[2][3]*Hi8788[3][2];
Ai088[2][3]=Ai087[2][2];
Ai088[2][4]=Ai087[2][4];

Ai088[3][1]=Ai087[3][1]*Hi8788[1][1] + Ai087[3][3]*Hi8788[3][1];
Ai088[3][2]=Ai087[3][1]*Hi8788[1][2] + Ai087[3][3]*Hi8788[3][2];
Ai088[3][3]=Ai087[3][2];
Ai088[3][4]=Ai087[3][4];


Ai089[1][1]=Ai088[1][1]*Hi8889[1][1] + Ai088[1][3]*Hi8889[3][1];
Ai089[1][2]=Ai088[1][1]*Hi8889[1][2] + Ai088[1][3]*Hi8889[3][2];
Ai089[1][3]=Ai088[1][2];
Ai089[1][4]=Ai088[1][4] + Ai088[1][3]*Hi8889[3][4];

Ai089[2][1]=Ai088[2][1]*Hi8889[1][1] + Ai088[2][3]*Hi8889[3][1];
Ai089[2][2]=Ai088[2][1]*Hi8889[1][2] + Ai088[2][3]*Hi8889[3][2];
Ai089[2][3]=Ai088[2][2];
Ai089[2][4]=Ai088[2][4] + Ai088[2][3]*Hi8889[3][4];

Ai089[3][1]=Ai088[3][1]*Hi8889[1][1] + Ai088[3][3]*Hi8889[3][1];
Ai089[3][2]=Ai088[3][1]*Hi8889[1][2] + Ai088[3][3]*Hi8889[3][2];
Ai089[3][3]=Ai088[3][2];
Ai089[3][4]=Ai088[3][4] + Ai088[3][3]*Hi8889[3][4];


Ai090[1][1]=Ai089[1][1]*Hi8990[1][1] + Ai089[1][3]*Hi8990[3][1];
Ai090[1][2]=Ai089[1][1]*Hi8990[1][2] + Ai089[1][3]*Hi8990[3][2];
Ai090[1][3]=-Ai089[1][2];
Ai090[1][4]=Ai089[1][4];

Ai090[2][1]=Ai089[2][1]*Hi8990[1][1] + Ai089[2][3]*Hi8990[3][1];
Ai090[2][2]=Ai089[2][1]*Hi8990[1][2] + Ai089[2][3]*Hi8990[3][2];
Ai090[2][3]=-Ai089[2][2];
Ai090[2][4]=Ai089[2][4];

Ai090[3][1]=Ai089[3][1]*Hi8990[1][1] + Ai089[3][3]*Hi8990[3][1];
Ai090[3][2]=Ai089[3][1]*Hi8990[1][2] + Ai089[3][3]*Hi8990[3][2];
Ai090[3][3]=-Ai089[3][2];
Ai090[3][4]=Ai089[3][4];


Ai091[1][1]=Ai090[1][1];
Ai091[1][2]=Ai090[1][2];
Ai091[1][3]=Ai090[1][3];
Ai091[1][4]=Ai090[1][4] + Ai090[1][1]*Hi9091[1][4] + Ai090[1][2]*Hi9091[2][4] + Ai090[1][3]*Hi9091[3][4];

Ai091[2][1]=Ai090[2][1];
Ai091[2][2]=Ai090[2][2];
Ai091[2][3]=Ai090[2][3];
Ai091[2][4]=Ai090[2][4] + Ai090[2][1]*Hi9091[1][4] + Ai090[2][2]*Hi9091[2][4] + Ai090[2][3]*Hi9091[3][4];

Ai091[3][1]=Ai090[3][1];
Ai091[3][2]=Ai090[3][2];
Ai091[3][3]=Ai090[3][3];
Ai091[3][4]=Ai090[3][4] + Ai090[3][1]*Hi9091[1][4] + Ai090[3][2]*Hi9091[2][4] + Ai090[3][3]*Hi9091[3][4];


Ai092[1][1]=Ai090[1][1];
Ai092[1][2]=Ai090[1][2];
Ai092[1][3]=Ai090[1][3];
Ai092[1][4]=Ai090[1][4] + Ai090[1][1]*Hi9092[1][4] + Ai090[1][2]*Hi9092[2][4] + Ai090[1][3]*Hi9092[3][4];

Ai092[2][1]=Ai090[2][1];
Ai092[2][2]=Ai090[2][2];
Ai092[2][3]=Ai090[2][3];
Ai092[2][4]=Ai090[2][4] + Ai090[2][1]*Hi9092[1][4] + Ai090[2][2]*Hi9092[2][4] + Ai090[2][3]*Hi9092[3][4];

Ai092[3][1]=Ai090[3][1];
Ai092[3][2]=Ai090[3][2];
Ai092[3][3]=Ai090[3][3];
Ai092[3][4]=Ai090[3][4] + Ai090[3][1]*Hi9092[1][4] + Ai090[3][2]*Hi9092[2][4] + Ai090[3][3]*Hi9092[3][4];


Ai093[1][1]=Ai090[1][1];
Ai093[1][2]=Ai090[1][2];
Ai093[1][3]=Ai090[1][3];
Ai093[1][4]=Ai090[1][4] + Ai090[1][1]*Hi9093[1][4] + Ai090[1][2]*Hi9093[2][4] + Ai090[1][3]*Hi9093[3][4];

Ai093[2][1]=Ai090[2][1];
Ai093[2][2]=Ai090[2][2];
Ai093[2][3]=Ai090[2][3];
Ai093[2][4]=Ai090[2][4] + Ai090[2][1]*Hi9093[1][4] + Ai090[2][2]*Hi9093[2][4] + Ai090[2][3]*Hi9093[3][4];

Ai093[3][1]=Ai090[3][1];
Ai093[3][2]=Ai090[3][2];
Ai093[3][3]=Ai090[3][3];
Ai093[3][4]=Ai090[3][4] + Ai090[3][1]*Hi9093[1][4] + Ai090[3][2]*Hi9093[2][4] + Ai090[3][3]*Hi9093[3][4];


Ai094[1][1]=Ai090[1][1];
Ai094[1][2]=Ai090[1][2];
Ai094[1][3]=Ai090[1][3];
Ai094[1][4]=Ai090[1][4] + Ai090[1][1]*Hi9094[1][4] + Ai090[1][2]*Hi9094[2][4] + Ai090[1][3]*Hi9094[3][4];

Ai094[2][1]=Ai090[2][1];
Ai094[2][2]=Ai090[2][2];
Ai094[2][3]=Ai090[2][3];
Ai094[2][4]=Ai090[2][4] + Ai090[2][1]*Hi9094[1][4] + Ai090[2][2]*Hi9094[2][4] + Ai090[2][3]*Hi9094[3][4];

Ai094[3][1]=Ai090[3][1];
Ai094[3][2]=Ai090[3][2];
Ai094[3][3]=Ai090[3][3];
Ai094[3][4]=Ai090[3][4] + Ai090[3][1]*Hi9094[1][4] + Ai090[3][2]*Hi9094[2][4] + Ai090[3][3]*Hi9094[3][4];


Ai095[1][1]=Ai090[1][1];
Ai095[1][2]=Ai090[1][2];
Ai095[1][3]=Ai090[1][3];
Ai095[1][4]=Ai090[1][4] + Ai090[1][1]*Hi9095[1][4] + Ai090[1][2]*Hi9095[2][4] + Ai090[1][3]*Hi9095[3][4];

Ai095[2][1]=Ai090[2][1];
Ai095[2][2]=Ai090[2][2];
Ai095[2][3]=Ai090[2][3];
Ai095[2][4]=Ai090[2][4] + Ai090[2][1]*Hi9095[1][4] + Ai090[2][2]*Hi9095[2][4] + Ai090[2][3]*Hi9095[3][4];

Ai095[3][1]=Ai090[3][1];
Ai095[3][2]=Ai090[3][2];
Ai095[3][3]=Ai090[3][3];
Ai095[3][4]=Ai090[3][4] + Ai090[3][1]*Hi9095[1][4] + Ai090[3][2]*Hi9095[2][4] + Ai090[3][3]*Hi9095[3][4];


Ai096[1][1]=Ai090[1][1];
Ai096[1][2]=Ai090[1][2];
Ai096[1][3]=Ai090[1][3];
Ai096[1][4]=Ai090[1][4] + Ai090[1][1]*Hi9096[1][4] + Ai090[1][2]*Hi9096[2][4] + Ai090[1][3]*Hi9096[3][4];

Ai096[2][1]=Ai090[2][1];
Ai096[2][2]=Ai090[2][2];
Ai096[2][3]=Ai090[2][3];
Ai096[2][4]=Ai090[2][4] + Ai090[2][1]*Hi9096[1][4] + Ai090[2][2]*Hi9096[2][4] + Ai090[2][3]*Hi9096[3][4];

Ai096[3][1]=Ai090[3][1];
Ai096[3][2]=Ai090[3][2];
Ai096[3][3]=Ai090[3][3];
Ai096[3][4]=Ai090[3][4] + Ai090[3][1]*Hi9096[1][4] + Ai090[3][2]*Hi9096[2][4] + Ai090[3][3]*Hi9096[3][4];


Ai097[1][1]=Ai090[1][1]*Hi9097[1][1] + Ai090[1][2]*Hi9097[2][1] + Ai090[1][3]*Hi9097[3][1];
Ai097[1][2]=Ai090[1][1]*Hi9097[1][2] + Ai090[1][2]*Hi9097[2][2] + Ai090[1][3]*Hi9097[3][2];
Ai097[1][3]=Ai090[1][1]*Hi9097[1][3] + Ai090[1][2]*Hi9097[2][3] + Ai090[1][3]*Hi9097[3][3];
Ai097[1][4]=Ai090[1][4] + Ai090[1][1]*Hi9097[1][4] + Ai090[1][2]*Hi9097[2][4] + Ai090[1][3]*Hi9097[3][4];

Ai097[2][1]=Ai090[2][1]*Hi9097[1][1] + Ai090[2][2]*Hi9097[2][1] + Ai090[2][3]*Hi9097[3][1];
Ai097[2][2]=Ai090[2][1]*Hi9097[1][2] + Ai090[2][2]*Hi9097[2][2] + Ai090[2][3]*Hi9097[3][2];
Ai097[2][3]=Ai090[2][1]*Hi9097[1][3] + Ai090[2][2]*Hi9097[2][3] + Ai090[2][3]*Hi9097[3][3];
Ai097[2][4]=Ai090[2][4] + Ai090[2][1]*Hi9097[1][4] + Ai090[2][2]*Hi9097[2][4] + Ai090[2][3]*Hi9097[3][4];

Ai097[3][1]=Ai090[3][1]*Hi9097[1][1] + Ai090[3][2]*Hi9097[2][1] + Ai090[3][3]*Hi9097[3][1];
Ai097[3][2]=Ai090[3][1]*Hi9097[1][2] + Ai090[3][2]*Hi9097[2][2] + Ai090[3][3]*Hi9097[3][2];
Ai097[3][3]=Ai090[3][1]*Hi9097[1][3] + Ai090[3][2]*Hi9097[2][3] + Ai090[3][3]*Hi9097[3][3];
Ai097[3][4]=Ai090[3][4] + Ai090[3][1]*Hi9097[1][4] + Ai090[3][2]*Hi9097[2][4] + Ai090[3][3]*Hi9097[3][4];



/* joint ID: 0 */
Xorigin[0][1]=Hi00[1][4];
Xorigin[0][2]=Hi00[2][4];
Xorigin[0][3]=Hi00[3][4];

Xmcog[0][1]=links[0].mcm[1]*Hi00[1][1] + links[0].mcm[2]*Hi00[1][2] + links[0].mcm[3]*Hi00[1][3] + links[0].m*Hi00[1][4];
Xmcog[0][2]=links[0].mcm[1]*Hi00[2][1] + links[0].mcm[2]*Hi00[2][2] + links[0].mcm[3]*Hi00[2][3] + links[0].m*Hi00[2][4];
Xmcog[0][3]=links[0].mcm[1]*Hi00[3][1] + links[0].mcm[2]*Hi00[3][2] + links[0].mcm[3]*Hi00[3][3] + links[0].m*Hi00[3][4];

Xaxis[0][1]=0;
Xaxis[0][2]=0;
Xaxis[0][3]=0;

Ahmatdof[0][1][1]=Hi00[1][1];
Ahmatdof[0][1][2]=Hi00[1][2];
Ahmatdof[0][1][3]=Hi00[1][3];
Ahmatdof[0][1][4]=Hi00[1][4];

Ahmatdof[0][2][1]=Hi00[2][1];
Ahmatdof[0][2][2]=Hi00[2][2];
Ahmatdof[0][2][3]=Hi00[2][3];
Ahmatdof[0][2][4]=Hi00[2][4];

Ahmatdof[0][3][1]=Hi00[3][1];
Ahmatdof[0][3][2]=Hi00[3][2];
Ahmatdof[0][3][3]=Hi00[3][3];
Ahmatdof[0][3][4]=Hi00[3][4];

Ahmatdof[0][4][1]=0.;
Ahmatdof[0][4][2]=0.;
Ahmatdof[0][4][3]=0.;
Ahmatdof[0][4][4]=1;


/* link 0: {basec$0$$x[[1]], basec$0$$x[[2]], basec$0$$x[[3]]} */
Xlink[0][1]=Hi00[1][4];
Xlink[0][2]=Hi00[2][4];
Xlink[0][3]=Hi00[3][4];

Ahmat[0][1][1]=Hi00[1][1];
Ahmat[0][1][2]=Hi00[1][2];
Ahmat[0][1][3]=Hi00[1][3];
Ahmat[0][1][4]=Hi00[1][4];

Ahmat[0][2][1]=Hi00[2][1];
Ahmat[0][2][2]=Hi00[2][2];
Ahmat[0][2][3]=Hi00[2][3];
Ahmat[0][2][4]=Hi00[2][4];

Ahmat[0][3][1]=Hi00[3][1];
Ahmat[0][3][2]=Hi00[3][2];
Ahmat[0][3][3]=Hi00[3][3];
Ahmat[0][3][4]=Hi00[3][4];

Ahmat[0][4][1]=0.;
Ahmat[0][4][2]=0.;
Ahmat[0][4][3]=0.;
Ahmat[0][4][4]=1;


/* joint ID: 29 */
Xorigin[29][1]=Ai01[1][4];
Xorigin[29][2]=Ai01[2][4];
Xorigin[29][3]=Ai01[3][4];

Xmcog[29][1]=links[29].mcm[1]*Ai01[1][1] + links[29].mcm[2]*Ai01[1][2] + links[29].mcm[3]*Ai01[1][3] + links[29].m*Ai01[1][4];
Xmcog[29][2]=links[29].mcm[1]*Ai01[2][1] + links[29].mcm[2]*Ai01[2][2] + links[29].mcm[3]*Ai01[2][3] + links[29].m*Ai01[2][4];
Xmcog[29][3]=links[29].mcm[1]*Ai01[3][1] + links[29].mcm[2]*Ai01[3][2] + links[29].mcm[3]*Ai01[3][3] + links[29].m*Ai01[3][4];

Xaxis[29][1]=Ai01[1][3];
Xaxis[29][2]=Ai01[2][3];
Xaxis[29][3]=Ai01[3][3];

Ahmatdof[29][1][1]=Ai01[1][1];
Ahmatdof[29][1][2]=Ai01[1][2];
Ahmatdof[29][1][3]=Ai01[1][3];
Ahmatdof[29][1][4]=Ai01[1][4];

Ahmatdof[29][2][1]=Ai01[2][1];
Ahmatdof[29][2][2]=Ai01[2][2];
Ahmatdof[29][2][3]=Ai01[2][3];
Ahmatdof[29][2][4]=Ai01[2][4];

Ahmatdof[29][3][1]=Ai01[3][1];
Ahmatdof[29][3][2]=Ai01[3][2];
Ahmatdof[29][3][3]=Ai01[3][3];
Ahmatdof[29][3][4]=Ai01[3][4];

Ahmatdof[29][4][1]=0.;
Ahmatdof[29][4][2]=0.;
Ahmatdof[29][4][3]=0.;
Ahmatdof[29][4][4]=1;


/* link 1: {0, -PELVISOFFSET, PELVIS2THORAX} */
Xlink[1][1]=Ai01[1][4];
Xlink[1][2]=Ai01[2][4];
Xlink[1][3]=Ai01[3][4];

Ahmat[1][1][1]=Ai03[1][1];
Ahmat[1][1][2]=Ai03[1][2];
Ahmat[1][1][3]=Ai03[1][3];
Ahmat[1][1][4]=Ai03[1][4];

Ahmat[1][2][1]=Ai03[2][1];
Ahmat[1][2][2]=Ai03[2][2];
Ahmat[1][2][3]=Ai03[2][3];
Ahmat[1][2][4]=Ai03[2][4];

Ahmat[1][3][1]=Ai03[3][1];
Ahmat[1][3][2]=Ai03[3][2];
Ahmat[1][3][3]=Ai03[3][3];
Ahmat[1][3][4]=Ai03[3][4];

Ahmat[1][4][1]=0.;
Ahmat[1][4][2]=0.;
Ahmat[1][4][3]=0.;
Ahmat[1][4][4]=1;


/* joint ID: 30 */
Xorigin[30][1]=Ai02[1][4];
Xorigin[30][2]=Ai02[2][4];
Xorigin[30][3]=Ai02[3][4];

Xmcog[30][1]=links[30].mcm[1]*Ai02[1][1] + links[30].mcm[2]*Ai02[1][2] + links[30].mcm[3]*Ai02[1][3] + links[30].m*Ai02[1][4];
Xmcog[30][2]=links[30].mcm[1]*Ai02[2][1] + links[30].mcm[2]*Ai02[2][2] + links[30].mcm[3]*Ai02[2][3] + links[30].m*Ai02[2][4];
Xmcog[30][3]=links[30].mcm[1]*Ai02[3][1] + links[30].mcm[2]*Ai02[3][2] + links[30].mcm[3]*Ai02[3][3] + links[30].m*Ai02[3][4];

Xaxis[30][1]=Ai02[1][3];
Xaxis[30][2]=Ai02[2][3];
Xaxis[30][3]=Ai02[3][3];

Ahmatdof[30][1][1]=Ai02[1][1];
Ahmatdof[30][1][2]=Ai02[1][2];
Ahmatdof[30][1][3]=Ai02[1][3];
Ahmatdof[30][1][4]=Ai02[1][4];

Ahmatdof[30][2][1]=Ai02[2][1];
Ahmatdof[30][2][2]=Ai02[2][2];
Ahmatdof[30][2][3]=Ai02[2][3];
Ahmatdof[30][2][4]=Ai02[2][4];

Ahmatdof[30][3][1]=Ai02[3][1];
Ahmatdof[30][3][2]=Ai02[3][2];
Ahmatdof[30][3][3]=Ai02[3][3];
Ahmatdof[30][3][4]=Ai02[3][4];

Ahmatdof[30][4][1]=0.;
Ahmatdof[30][4][2]=0.;
Ahmatdof[30][4][3]=0.;
Ahmatdof[30][4][4]=1;


/* joint ID: 31 */
Xorigin[31][1]=Ai03[1][4];
Xorigin[31][2]=Ai03[2][4];
Xorigin[31][3]=Ai03[3][4];

Xmcog[31][1]=links[31].mcm[1]*Ai03[1][1] + links[31].mcm[2]*Ai03[1][2] + links[31].mcm[3]*Ai03[1][3] + links[31].m*Ai03[1][4];
Xmcog[31][2]=links[31].mcm[1]*Ai03[2][1] + links[31].mcm[2]*Ai03[2][2] + links[31].mcm[3]*Ai03[2][3] + links[31].m*Ai03[2][4];
Xmcog[31][3]=links[31].mcm[1]*Ai03[3][1] + links[31].mcm[2]*Ai03[3][2] + links[31].mcm[3]*Ai03[3][3] + links[31].m*Ai03[3][4];

Xaxis[31][1]=Ai03[1][3];
Xaxis[31][2]=Ai03[2][3];
Xaxis[31][3]=Ai03[3][3];

Ahmatdof[31][1][1]=Ai03[1][1];
Ahmatdof[31][1][2]=Ai03[1][2];
Ahmatdof[31][1][3]=Ai03[1][3];
Ahmatdof[31][1][4]=Ai03[1][4];

Ahmatdof[31][2][1]=Ai03[2][1];
Ahmatdof[31][2][2]=Ai03[2][2];
Ahmatdof[31][2][3]=Ai03[2][3];
Ahmatdof[31][2][4]=Ai03[2][4];

Ahmatdof[31][3][1]=Ai03[3][1];
Ahmatdof[31][3][2]=Ai03[3][2];
Ahmatdof[31][3][3]=Ai03[3][3];
Ahmatdof[31][3][4]=Ai03[3][4];

Ahmatdof[31][4][1]=0.;
Ahmatdof[31][4][2]=0.;
Ahmatdof[31][4][3]=0.;
Ahmatdof[31][4][4]=1;


/* joint ID: 1 */
Xorigin[1][1]=Ai04[1][4];
Xorigin[1][2]=Ai04[2][4];
Xorigin[1][3]=Ai04[3][4];

Xmcog[1][1]=links[1].mcm[1]*Ai04[1][1] + links[1].mcm[2]*Ai04[1][2] + links[1].mcm[3]*Ai04[1][3] + links[1].m*Ai04[1][4];
Xmcog[1][2]=links[1].mcm[1]*Ai04[2][1] + links[1].mcm[2]*Ai04[2][2] + links[1].mcm[3]*Ai04[2][3] + links[1].m*Ai04[2][4];
Xmcog[1][3]=links[1].mcm[1]*Ai04[3][1] + links[1].mcm[2]*Ai04[3][2] + links[1].mcm[3]*Ai04[3][3] + links[1].m*Ai04[3][4];

Xaxis[1][1]=Ai04[1][3];
Xaxis[1][2]=Ai04[2][3];
Xaxis[1][3]=Ai04[3][3];

Ahmatdof[1][1][1]=Ai04[1][1];
Ahmatdof[1][1][2]=Ai04[1][2];
Ahmatdof[1][1][3]=Ai04[1][3];
Ahmatdof[1][1][4]=Ai04[1][4];

Ahmatdof[1][2][1]=Ai04[2][1];
Ahmatdof[1][2][2]=Ai04[2][2];
Ahmatdof[1][2][3]=Ai04[2][3];
Ahmatdof[1][2][4]=Ai04[2][4];

Ahmatdof[1][3][1]=Ai04[3][1];
Ahmatdof[1][3][2]=Ai04[3][2];
Ahmatdof[1][3][3]=Ai04[3][3];
Ahmatdof[1][3][4]=Ai04[3][4];

Ahmatdof[1][4][1]=0.;
Ahmatdof[1][4][2]=0.;
Ahmatdof[1][4][3]=0.;
Ahmatdof[1][4][4]=1;


/* link 2: {-THORAX2SHOULDER, 0, 0} */
Xlink[2][1]=Ai04[1][4];
Xlink[2][2]=Ai04[2][4];
Xlink[2][3]=Ai04[3][4];

Ahmat[2][1][1]=Ai04[1][1];
Ahmat[2][1][2]=Ai04[1][2];
Ahmat[2][1][3]=Ai04[1][3];
Ahmat[2][1][4]=Ai04[1][4];

Ahmat[2][2][1]=Ai04[2][1];
Ahmat[2][2][2]=Ai04[2][2];
Ahmat[2][2][3]=Ai04[2][3];
Ahmat[2][2][4]=Ai04[2][4];

Ahmat[2][3][1]=Ai04[3][1];
Ahmat[2][3][2]=Ai04[3][2];
Ahmat[2][3][3]=Ai04[3][3];
Ahmat[2][3][4]=Ai04[3][4];

Ahmat[2][4][1]=0.;
Ahmat[2][4][2]=0.;
Ahmat[2][4][3]=0.;
Ahmat[2][4][4]=1;


/* joint ID: 2 */
Xorigin[2][1]=Ai05[1][4];
Xorigin[2][2]=Ai05[2][4];
Xorigin[2][3]=Ai05[3][4];

Xmcog[2][1]=links[2].mcm[1]*Ai05[1][1] + links[2].mcm[2]*Ai05[1][2] + links[2].mcm[3]*Ai05[1][3] + links[2].m*Ai05[1][4];
Xmcog[2][2]=links[2].mcm[1]*Ai05[2][1] + links[2].mcm[2]*Ai05[2][2] + links[2].mcm[3]*Ai05[2][3] + links[2].m*Ai05[2][4];
Xmcog[2][3]=links[2].mcm[1]*Ai05[3][1] + links[2].mcm[2]*Ai05[3][2] + links[2].mcm[3]*Ai05[3][3] + links[2].m*Ai05[3][4];

Xaxis[2][1]=Ai05[1][3];
Xaxis[2][2]=Ai05[2][3];
Xaxis[2][3]=Ai05[3][3];

Ahmatdof[2][1][1]=Ai05[1][1];
Ahmatdof[2][1][2]=Ai05[1][2];
Ahmatdof[2][1][3]=Ai05[1][3];
Ahmatdof[2][1][4]=Ai05[1][4];

Ahmatdof[2][2][1]=Ai05[2][1];
Ahmatdof[2][2][2]=Ai05[2][2];
Ahmatdof[2][2][3]=Ai05[2][3];
Ahmatdof[2][2][4]=Ai05[2][4];

Ahmatdof[2][3][1]=Ai05[3][1];
Ahmatdof[2][3][2]=Ai05[3][2];
Ahmatdof[2][3][3]=Ai05[3][3];
Ahmatdof[2][3][4]=Ai05[3][4];

Ahmatdof[2][4][1]=0.;
Ahmatdof[2][4][2]=0.;
Ahmatdof[2][4][3]=0.;
Ahmatdof[2][4][4]=1;


/* link 3: {0, 0, -SHOULDERX} */
Xlink[3][1]=Ai05[1][4];
Xlink[3][2]=Ai05[2][4];
Xlink[3][3]=Ai05[3][4];

Ahmat[3][1][1]=Ai05[1][1];
Ahmat[3][1][2]=Ai05[1][2];
Ahmat[3][1][3]=Ai05[1][3];
Ahmat[3][1][4]=Ai05[1][4];

Ahmat[3][2][1]=Ai05[2][1];
Ahmat[3][2][2]=Ai05[2][2];
Ahmat[3][2][3]=Ai05[2][3];
Ahmat[3][2][4]=Ai05[2][4];

Ahmat[3][3][1]=Ai05[3][1];
Ahmat[3][3][2]=Ai05[3][2];
Ahmat[3][3][3]=Ai05[3][3];
Ahmat[3][3][4]=Ai05[3][4];

Ahmat[3][4][1]=0.;
Ahmat[3][4][2]=0.;
Ahmat[3][4][3]=0.;
Ahmat[3][4][4]=1;


/* joint ID: 3 */
Xorigin[3][1]=Ai06[1][4];
Xorigin[3][2]=Ai06[2][4];
Xorigin[3][3]=Ai06[3][4];

Xmcog[3][1]=links[3].mcm[1]*Ai06[1][1] + links[3].mcm[2]*Ai06[1][2] + links[3].mcm[3]*Ai06[1][3] + links[3].m*Ai06[1][4];
Xmcog[3][2]=links[3].mcm[1]*Ai06[2][1] + links[3].mcm[2]*Ai06[2][2] + links[3].mcm[3]*Ai06[2][3] + links[3].m*Ai06[2][4];
Xmcog[3][3]=links[3].mcm[1]*Ai06[3][1] + links[3].mcm[2]*Ai06[3][2] + links[3].mcm[3]*Ai06[3][3] + links[3].m*Ai06[3][4];

Xaxis[3][1]=Ai06[1][3];
Xaxis[3][2]=Ai06[2][3];
Xaxis[3][3]=Ai06[3][3];

Ahmatdof[3][1][1]=Ai06[1][1];
Ahmatdof[3][1][2]=Ai06[1][2];
Ahmatdof[3][1][3]=Ai06[1][3];
Ahmatdof[3][1][4]=Ai06[1][4];

Ahmatdof[3][2][1]=Ai06[2][1];
Ahmatdof[3][2][2]=Ai06[2][2];
Ahmatdof[3][2][3]=Ai06[2][3];
Ahmatdof[3][2][4]=Ai06[2][4];

Ahmatdof[3][3][1]=Ai06[3][1];
Ahmatdof[3][3][2]=Ai06[3][2];
Ahmatdof[3][3][3]=Ai06[3][3];
Ahmatdof[3][3][4]=Ai06[3][4];

Ahmatdof[3][4][1]=0.;
Ahmatdof[3][4][2]=0.;
Ahmatdof[3][4][3]=0.;
Ahmatdof[3][4][4]=1;


/* link 4: {-SHOULDERY, 0, 0} */
Xlink[4][1]=Ai06[1][4];
Xlink[4][2]=Ai06[2][4];
Xlink[4][3]=Ai06[3][4];

Ahmat[4][1][1]=Ai06[1][1];
Ahmat[4][1][2]=Ai06[1][2];
Ahmat[4][1][3]=Ai06[1][3];
Ahmat[4][1][4]=Ai06[1][4];

Ahmat[4][2][1]=Ai06[2][1];
Ahmat[4][2][2]=Ai06[2][2];
Ahmat[4][2][3]=Ai06[2][3];
Ahmat[4][2][4]=Ai06[2][4];

Ahmat[4][3][1]=Ai06[3][1];
Ahmat[4][3][2]=Ai06[3][2];
Ahmat[4][3][3]=Ai06[3][3];
Ahmat[4][3][4]=Ai06[3][4];

Ahmat[4][4][1]=0.;
Ahmat[4][4][2]=0.;
Ahmat[4][4][3]=0.;
Ahmat[4][4][4]=1;


/* joint ID: 4 */
Xorigin[4][1]=Ai07[1][4];
Xorigin[4][2]=Ai07[2][4];
Xorigin[4][3]=Ai07[3][4];

Xmcog[4][1]=links[4].mcm[1]*Ai07[1][1] + links[4].mcm[2]*Ai07[1][2] + links[4].mcm[3]*Ai07[1][3] + links[4].m*Ai07[1][4];
Xmcog[4][2]=links[4].mcm[1]*Ai07[2][1] + links[4].mcm[2]*Ai07[2][2] + links[4].mcm[3]*Ai07[2][3] + links[4].m*Ai07[2][4];
Xmcog[4][3]=links[4].mcm[1]*Ai07[3][1] + links[4].mcm[2]*Ai07[3][2] + links[4].mcm[3]*Ai07[3][3] + links[4].m*Ai07[3][4];

Xaxis[4][1]=Ai07[1][3];
Xaxis[4][2]=Ai07[2][3];
Xaxis[4][3]=Ai07[3][3];

Ahmatdof[4][1][1]=Ai07[1][1];
Ahmatdof[4][1][2]=Ai07[1][2];
Ahmatdof[4][1][3]=Ai07[1][3];
Ahmatdof[4][1][4]=Ai07[1][4];

Ahmatdof[4][2][1]=Ai07[2][1];
Ahmatdof[4][2][2]=Ai07[2][2];
Ahmatdof[4][2][3]=Ai07[2][3];
Ahmatdof[4][2][4]=Ai07[2][4];

Ahmatdof[4][3][1]=Ai07[3][1];
Ahmatdof[4][3][2]=Ai07[3][2];
Ahmatdof[4][3][3]=Ai07[3][3];
Ahmatdof[4][3][4]=Ai07[3][4];

Ahmatdof[4][4][1]=0.;
Ahmatdof[4][4][2]=0.;
Ahmatdof[4][4][3]=0.;
Ahmatdof[4][4][4]=1;


/* link 5: {0, 0, -UPPERARM} */
Xlink[5][1]=Ai07[1][4];
Xlink[5][2]=Ai07[2][4];
Xlink[5][3]=Ai07[3][4];

Ahmat[5][1][1]=Ai08[1][1];
Ahmat[5][1][2]=Ai08[1][2];
Ahmat[5][1][3]=Ai08[1][3];
Ahmat[5][1][4]=Ai08[1][4];

Ahmat[5][2][1]=Ai08[2][1];
Ahmat[5][2][2]=Ai08[2][2];
Ahmat[5][2][3]=Ai08[2][3];
Ahmat[5][2][4]=Ai08[2][4];

Ahmat[5][3][1]=Ai08[3][1];
Ahmat[5][3][2]=Ai08[3][2];
Ahmat[5][3][3]=Ai08[3][3];
Ahmat[5][3][4]=Ai08[3][4];

Ahmat[5][4][1]=0.;
Ahmat[5][4][2]=0.;
Ahmat[5][4][3]=0.;
Ahmat[5][4][4]=1;


/* joint ID: 5 */
Xorigin[5][1]=Ai08[1][4];
Xorigin[5][2]=Ai08[2][4];
Xorigin[5][3]=Ai08[3][4];

Xmcog[5][1]=links[5].mcm[1]*Ai08[1][1] + links[5].mcm[2]*Ai08[1][2] + links[5].mcm[3]*Ai08[1][3] + links[5].m*Ai08[1][4];
Xmcog[5][2]=links[5].mcm[1]*Ai08[2][1] + links[5].mcm[2]*Ai08[2][2] + links[5].mcm[3]*Ai08[2][3] + links[5].m*Ai08[2][4];
Xmcog[5][3]=links[5].mcm[1]*Ai08[3][1] + links[5].mcm[2]*Ai08[3][2] + links[5].mcm[3]*Ai08[3][3] + links[5].m*Ai08[3][4];

Xaxis[5][1]=Ai08[1][3];
Xaxis[5][2]=Ai08[2][3];
Xaxis[5][3]=Ai08[3][3];

Ahmatdof[5][1][1]=Ai08[1][1];
Ahmatdof[5][1][2]=Ai08[1][2];
Ahmatdof[5][1][3]=Ai08[1][3];
Ahmatdof[5][1][4]=Ai08[1][4];

Ahmatdof[5][2][1]=Ai08[2][1];
Ahmatdof[5][2][2]=Ai08[2][2];
Ahmatdof[5][2][3]=Ai08[2][3];
Ahmatdof[5][2][4]=Ai08[2][4];

Ahmatdof[5][3][1]=Ai08[3][1];
Ahmatdof[5][3][2]=Ai08[3][2];
Ahmatdof[5][3][3]=Ai08[3][3];
Ahmatdof[5][3][4]=Ai08[3][4];

Ahmatdof[5][4][1]=0.;
Ahmatdof[5][4][2]=0.;
Ahmatdof[5][4][3]=0.;
Ahmatdof[5][4][4]=1;


/* joint ID: 6 */
Xorigin[6][1]=Ai09[1][4];
Xorigin[6][2]=Ai09[2][4];
Xorigin[6][3]=Ai09[3][4];

Xmcog[6][1]=links[6].mcm[1]*Ai09[1][1] + links[6].mcm[2]*Ai09[1][2] + links[6].mcm[3]*Ai09[1][3] + links[6].m*Ai09[1][4];
Xmcog[6][2]=links[6].mcm[1]*Ai09[2][1] + links[6].mcm[2]*Ai09[2][2] + links[6].mcm[3]*Ai09[2][3] + links[6].m*Ai09[2][4];
Xmcog[6][3]=links[6].mcm[1]*Ai09[3][1] + links[6].mcm[2]*Ai09[3][2] + links[6].mcm[3]*Ai09[3][3] + links[6].m*Ai09[3][4];

Xaxis[6][1]=Ai09[1][3];
Xaxis[6][2]=Ai09[2][3];
Xaxis[6][3]=Ai09[3][3];

Ahmatdof[6][1][1]=Ai09[1][1];
Ahmatdof[6][1][2]=Ai09[1][2];
Ahmatdof[6][1][3]=Ai09[1][3];
Ahmatdof[6][1][4]=Ai09[1][4];

Ahmatdof[6][2][1]=Ai09[2][1];
Ahmatdof[6][2][2]=Ai09[2][2];
Ahmatdof[6][2][3]=Ai09[2][3];
Ahmatdof[6][2][4]=Ai09[2][4];

Ahmatdof[6][3][1]=Ai09[3][1];
Ahmatdof[6][3][2]=Ai09[3][2];
Ahmatdof[6][3][3]=Ai09[3][3];
Ahmatdof[6][3][4]=Ai09[3][4];

Ahmatdof[6][4][1]=0.;
Ahmatdof[6][4][2]=0.;
Ahmatdof[6][4][3]=0.;
Ahmatdof[6][4][4]=1;


/* link 6: {0, WRISTY, -LOWERARM} */
Xlink[6][1]=Ai09[1][4];
Xlink[6][2]=Ai09[2][4];
Xlink[6][3]=Ai09[3][4];

Ahmat[6][1][1]=Ai010[1][1];
Ahmat[6][1][2]=Ai010[1][2];
Ahmat[6][1][3]=Ai010[1][3];
Ahmat[6][1][4]=Ai010[1][4];

Ahmat[6][2][1]=Ai010[2][1];
Ahmat[6][2][2]=Ai010[2][2];
Ahmat[6][2][3]=Ai010[2][3];
Ahmat[6][2][4]=Ai010[2][4];

Ahmat[6][3][1]=Ai010[3][1];
Ahmat[6][3][2]=Ai010[3][2];
Ahmat[6][3][3]=Ai010[3][3];
Ahmat[6][3][4]=Ai010[3][4];

Ahmat[6][4][1]=0.;
Ahmat[6][4][2]=0.;
Ahmat[6][4][3]=0.;
Ahmat[6][4][4]=1;


/* joint ID: 7 */
Xorigin[7][1]=Ai010[1][4];
Xorigin[7][2]=Ai010[2][4];
Xorigin[7][3]=Ai010[3][4];

Xmcog[7][1]=links[7].mcm[1]*Ai010[1][1] + links[7].mcm[2]*Ai010[1][2] + links[7].mcm[3]*Ai010[1][3] + links[7].m*Ai010[1][4];
Xmcog[7][2]=links[7].mcm[1]*Ai010[2][1] + links[7].mcm[2]*Ai010[2][2] + links[7].mcm[3]*Ai010[2][3] + links[7].m*Ai010[2][4];
Xmcog[7][3]=links[7].mcm[1]*Ai010[3][1] + links[7].mcm[2]*Ai010[3][2] + links[7].mcm[3]*Ai010[3][3] + links[7].m*Ai010[3][4];

Xaxis[7][1]=Ai010[1][3];
Xaxis[7][2]=Ai010[2][3];
Xaxis[7][3]=Ai010[3][3];

Ahmatdof[7][1][1]=Ai010[1][1];
Ahmatdof[7][1][2]=Ai010[1][2];
Ahmatdof[7][1][3]=Ai010[1][3];
Ahmatdof[7][1][4]=Ai010[1][4];

Ahmatdof[7][2][1]=Ai010[2][1];
Ahmatdof[7][2][2]=Ai010[2][2];
Ahmatdof[7][2][3]=Ai010[2][3];
Ahmatdof[7][2][4]=Ai010[2][4];

Ahmatdof[7][3][1]=Ai010[3][1];
Ahmatdof[7][3][2]=Ai010[3][2];
Ahmatdof[7][3][3]=Ai010[3][3];
Ahmatdof[7][3][4]=Ai010[3][4];

Ahmatdof[7][4][1]=0.;
Ahmatdof[7][4][2]=0.;
Ahmatdof[7][4][3]=0.;
Ahmatdof[7][4][4]=1;


/* link 7: {eff$2$$x[[1]], eff$2$$x[[2]], eff$2$$x[[3]]} */
Xlink[7][1]=Ai011[1][4];
Xlink[7][2]=Ai011[2][4];
Xlink[7][3]=Ai011[3][4];

Ahmat[7][1][1]=Ai011[1][1];
Ahmat[7][1][2]=Ai011[1][2];
Ahmat[7][1][3]=Ai011[1][3];
Ahmat[7][1][4]=Ai011[1][4];

Ahmat[7][2][1]=Ai011[2][1];
Ahmat[7][2][2]=Ai011[2][2];
Ahmat[7][2][3]=Ai011[2][3];
Ahmat[7][2][4]=Ai011[2][4];

Ahmat[7][3][1]=Ai011[3][1];
Ahmat[7][3][2]=Ai011[3][2];
Ahmat[7][3][3]=Ai011[3][3];
Ahmat[7][3][4]=Ai011[3][4];

Ahmat[7][4][1]=0.;
Ahmat[7][4][2]=0.;
Ahmat[7][4][3]=0.;
Ahmat[7][4][4]=1;


/* joint ID: 39 */
Xorigin[39][1]=Ai012[1][4];
Xorigin[39][2]=Ai012[2][4];
Xorigin[39][3]=Ai012[3][4];

Xmcog[39][1]=links[39].mcm[1]*Ai012[1][1] + links[39].mcm[2]*Ai012[1][2] + links[39].mcm[3]*Ai012[1][3] + links[39].m*Ai012[1][4];
Xmcog[39][2]=links[39].mcm[1]*Ai012[2][1] + links[39].mcm[2]*Ai012[2][2] + links[39].mcm[3]*Ai012[2][3] + links[39].m*Ai012[2][4];
Xmcog[39][3]=links[39].mcm[1]*Ai012[3][1] + links[39].mcm[2]*Ai012[3][2] + links[39].mcm[3]*Ai012[3][3] + links[39].m*Ai012[3][4];

Xaxis[39][1]=Ai012[1][1];
Xaxis[39][2]=Ai012[2][1];
Xaxis[39][3]=Ai012[3][1];

Ahmatdof[39][1][1]=Ai012[1][1];
Ahmatdof[39][1][2]=Ai012[1][2];
Ahmatdof[39][1][3]=Ai012[1][3];
Ahmatdof[39][1][4]=Ai012[1][4];

Ahmatdof[39][2][1]=Ai012[2][1];
Ahmatdof[39][2][2]=Ai012[2][2];
Ahmatdof[39][2][3]=Ai012[2][3];
Ahmatdof[39][2][4]=Ai012[2][4];

Ahmatdof[39][3][1]=Ai012[3][1];
Ahmatdof[39][3][2]=Ai012[3][2];
Ahmatdof[39][3][3]=Ai012[3][3];
Ahmatdof[39][3][4]=Ai012[3][4];

Ahmatdof[39][4][1]=0.;
Ahmatdof[39][4][2]=0.;
Ahmatdof[39][4][3]=0.;
Ahmatdof[39][4][4]=1;


/* link 8: {XTHUMB, YTHUMB, -ZTHUMB} */
Xlink[8][1]=Ai012[1][4];
Xlink[8][2]=Ai012[2][4];
Xlink[8][3]=Ai012[3][4];

Ahmat[8][1][1]=Ai012[1][1];
Ahmat[8][1][2]=Ai012[1][2];
Ahmat[8][1][3]=Ai012[1][3];
Ahmat[8][1][4]=Ai012[1][4];

Ahmat[8][2][1]=Ai012[2][1];
Ahmat[8][2][2]=Ai012[2][2];
Ahmat[8][2][3]=Ai012[2][3];
Ahmat[8][2][4]=Ai012[2][4];

Ahmat[8][3][1]=Ai012[3][1];
Ahmat[8][3][2]=Ai012[3][2];
Ahmat[8][3][3]=Ai012[3][3];
Ahmat[8][3][4]=Ai012[3][4];

Ahmat[8][4][1]=0.;
Ahmat[8][4][2]=0.;
Ahmat[8][4][3]=0.;
Ahmat[8][4][4]=1;


/* joint ID: 40 */
Xorigin[40][1]=Ai013[1][4];
Xorigin[40][2]=Ai013[2][4];
Xorigin[40][3]=Ai013[3][4];

Xmcog[40][1]=links[40].mcm[1]*Ai013[1][1] + links[40].mcm[2]*Ai013[1][2] + links[40].mcm[3]*Ai013[1][3] + links[40].m*Ai013[1][4];
Xmcog[40][2]=links[40].mcm[1]*Ai013[2][1] + links[40].mcm[2]*Ai013[2][2] + links[40].mcm[3]*Ai013[2][3] + links[40].m*Ai013[2][4];
Xmcog[40][3]=links[40].mcm[1]*Ai013[3][1] + links[40].mcm[2]*Ai013[3][2] + links[40].mcm[3]*Ai013[3][3] + links[40].m*Ai013[3][4];

Xaxis[40][1]=-Ai013[1][3];
Xaxis[40][2]=-Ai013[2][3];
Xaxis[40][3]=-Ai013[3][3];

Ahmatdof[40][1][1]=Ai013[1][1];
Ahmatdof[40][1][2]=Ai013[1][2];
Ahmatdof[40][1][3]=Ai013[1][3];
Ahmatdof[40][1][4]=Ai013[1][4];

Ahmatdof[40][2][1]=Ai013[2][1];
Ahmatdof[40][2][2]=Ai013[2][2];
Ahmatdof[40][2][3]=Ai013[2][3];
Ahmatdof[40][2][4]=Ai013[2][4];

Ahmatdof[40][3][1]=Ai013[3][1];
Ahmatdof[40][3][2]=Ai013[3][2];
Ahmatdof[40][3][3]=Ai013[3][3];
Ahmatdof[40][3][4]=Ai013[3][4];

Ahmatdof[40][4][1]=0.;
Ahmatdof[40][4][2]=0.;
Ahmatdof[40][4][3]=0.;
Ahmatdof[40][4][4]=1;


/* link 9: {XTHUMBFLEX, YTHUMBFLEX, 0} */
Xlink[9][1]=Ai013[1][4];
Xlink[9][2]=Ai013[2][4];
Xlink[9][3]=Ai013[3][4];

Ahmat[9][1][1]=Ai013[1][1];
Ahmat[9][1][2]=Ai013[1][2];
Ahmat[9][1][3]=Ai013[1][3];
Ahmat[9][1][4]=Ai013[1][4];

Ahmat[9][2][1]=Ai013[2][1];
Ahmat[9][2][2]=Ai013[2][2];
Ahmat[9][2][3]=Ai013[2][3];
Ahmat[9][2][4]=Ai013[2][4];

Ahmat[9][3][1]=Ai013[3][1];
Ahmat[9][3][2]=Ai013[3][2];
Ahmat[9][3][3]=Ai013[3][3];
Ahmat[9][3][4]=Ai013[3][4];

Ahmat[9][4][1]=0.;
Ahmat[9][4][2]=0.;
Ahmat[9][4][3]=0.;
Ahmat[9][4][4]=1;


/* link 10: {TH1SEG, 0, 0} */
Xlink[10][1]=Ai014[1][4];
Xlink[10][2]=Ai014[2][4];
Xlink[10][3]=Ai014[3][4];

Ahmat[10][1][1]=Ai014[1][1];
Ahmat[10][1][2]=Ai014[1][2];
Ahmat[10][1][3]=Ai014[1][3];
Ahmat[10][1][4]=Ai014[1][4];

Ahmat[10][2][1]=Ai014[2][1];
Ahmat[10][2][2]=Ai014[2][2];
Ahmat[10][2][3]=Ai014[2][3];
Ahmat[10][2][4]=Ai014[2][4];

Ahmat[10][3][1]=Ai014[3][1];
Ahmat[10][3][2]=Ai014[3][2];
Ahmat[10][3][3]=Ai014[3][3];
Ahmat[10][3][4]=Ai014[3][4];

Ahmat[10][4][1]=0.;
Ahmat[10][4][2]=0.;
Ahmat[10][4][3]=0.;
Ahmat[10][4][4]=1;


/* link 11: {TH2SEG, 0, 0} */
Xlink[11][1]=Ai015[1][4];
Xlink[11][2]=Ai015[2][4];
Xlink[11][3]=Ai015[3][4];

Ahmat[11][1][1]=Ai015[1][1];
Ahmat[11][1][2]=Ai015[1][2];
Ahmat[11][1][3]=Ai015[1][3];
Ahmat[11][1][4]=Ai015[1][4];

Ahmat[11][2][1]=Ai015[2][1];
Ahmat[11][2][2]=Ai015[2][2];
Ahmat[11][2][3]=Ai015[2][3];
Ahmat[11][2][4]=Ai015[2][4];

Ahmat[11][3][1]=Ai015[3][1];
Ahmat[11][3][2]=Ai015[3][2];
Ahmat[11][3][3]=Ai015[3][3];
Ahmat[11][3][4]=Ai015[3][4];

Ahmat[11][4][1]=0.;
Ahmat[11][4][2]=0.;
Ahmat[11][4][3]=0.;
Ahmat[11][4][4]=1;


/* joint ID: 41 */
Xorigin[41][1]=Ai016[1][4];
Xorigin[41][2]=Ai016[2][4];
Xorigin[41][3]=Ai016[3][4];

Xmcog[41][1]=links[41].mcm[1]*Ai016[1][1] + links[41].mcm[2]*Ai016[1][2] + links[41].mcm[3]*Ai016[1][3] + links[41].m*Ai016[1][4];
Xmcog[41][2]=links[41].mcm[1]*Ai016[2][1] + links[41].mcm[2]*Ai016[2][2] + links[41].mcm[3]*Ai016[2][3] + links[41].m*Ai016[2][4];
Xmcog[41][3]=links[41].mcm[1]*Ai016[3][1] + links[41].mcm[2]*Ai016[3][2] + links[41].mcm[3]*Ai016[3][3] + links[41].m*Ai016[3][4];

Xaxis[41][1]=Ai016[1][3];
Xaxis[41][2]=Ai016[2][3];
Xaxis[41][3]=Ai016[3][3];

Ahmatdof[41][1][1]=Ai016[1][1];
Ahmatdof[41][1][2]=Ai016[1][2];
Ahmatdof[41][1][3]=Ai016[1][3];
Ahmatdof[41][1][4]=Ai016[1][4];

Ahmatdof[41][2][1]=Ai016[2][1];
Ahmatdof[41][2][2]=Ai016[2][2];
Ahmatdof[41][2][3]=Ai016[2][3];
Ahmatdof[41][2][4]=Ai016[2][4];

Ahmatdof[41][3][1]=Ai016[3][1];
Ahmatdof[41][3][2]=Ai016[3][2];
Ahmatdof[41][3][3]=Ai016[3][3];
Ahmatdof[41][3][4]=Ai016[3][4];

Ahmatdof[41][4][1]=0.;
Ahmatdof[41][4][2]=0.;
Ahmatdof[41][4][3]=0.;
Ahmatdof[41][4][4]=1;


/* link 12: {XIF, YIF, -ZIF} */
Xlink[12][1]=Ai016[1][4];
Xlink[12][2]=Ai016[2][4];
Xlink[12][3]=Ai016[3][4];

Ahmat[12][1][1]=Ai016[1][1];
Ahmat[12][1][2]=Ai016[1][2];
Ahmat[12][1][3]=Ai016[1][3];
Ahmat[12][1][4]=Ai016[1][4];

Ahmat[12][2][1]=Ai016[2][1];
Ahmat[12][2][2]=Ai016[2][2];
Ahmat[12][2][3]=Ai016[2][3];
Ahmat[12][2][4]=Ai016[2][4];

Ahmat[12][3][1]=Ai016[3][1];
Ahmat[12][3][2]=Ai016[3][2];
Ahmat[12][3][3]=Ai016[3][3];
Ahmat[12][3][4]=Ai016[3][4];

Ahmat[12][4][1]=0.;
Ahmat[12][4][2]=0.;
Ahmat[12][4][3]=0.;
Ahmat[12][4][4]=1;


/* link 13: {F1SEG, 0, 0} */
Xlink[13][1]=Ai017[1][4];
Xlink[13][2]=Ai017[2][4];
Xlink[13][3]=Ai017[3][4];

Ahmat[13][1][1]=Ai017[1][1];
Ahmat[13][1][2]=Ai017[1][2];
Ahmat[13][1][3]=Ai017[1][3];
Ahmat[13][1][4]=Ai017[1][4];

Ahmat[13][2][1]=Ai017[2][1];
Ahmat[13][2][2]=Ai017[2][2];
Ahmat[13][2][3]=Ai017[2][3];
Ahmat[13][2][4]=Ai017[2][4];

Ahmat[13][3][1]=Ai017[3][1];
Ahmat[13][3][2]=Ai017[3][2];
Ahmat[13][3][3]=Ai017[3][3];
Ahmat[13][3][4]=Ai017[3][4];

Ahmat[13][4][1]=0.;
Ahmat[13][4][2]=0.;
Ahmat[13][4][3]=0.;
Ahmat[13][4][4]=1;


/* link 14: {F2SEG, 0, 0} */
Xlink[14][1]=Ai018[1][4];
Xlink[14][2]=Ai018[2][4];
Xlink[14][3]=Ai018[3][4];

Ahmat[14][1][1]=Ai018[1][1];
Ahmat[14][1][2]=Ai018[1][2];
Ahmat[14][1][3]=Ai018[1][3];
Ahmat[14][1][4]=Ai018[1][4];

Ahmat[14][2][1]=Ai018[2][1];
Ahmat[14][2][2]=Ai018[2][2];
Ahmat[14][2][3]=Ai018[2][3];
Ahmat[14][2][4]=Ai018[2][4];

Ahmat[14][3][1]=Ai018[3][1];
Ahmat[14][3][2]=Ai018[3][2];
Ahmat[14][3][3]=Ai018[3][3];
Ahmat[14][3][4]=Ai018[3][4];

Ahmat[14][4][1]=0.;
Ahmat[14][4][2]=0.;
Ahmat[14][4][3]=0.;
Ahmat[14][4][4]=1;


/* link 15: {F3SEG, 0, 0} */
Xlink[15][1]=Ai019[1][4];
Xlink[15][2]=Ai019[2][4];
Xlink[15][3]=Ai019[3][4];

Ahmat[15][1][1]=Ai019[1][1];
Ahmat[15][1][2]=Ai019[1][2];
Ahmat[15][1][3]=Ai019[1][3];
Ahmat[15][1][4]=Ai019[1][4];

Ahmat[15][2][1]=Ai019[2][1];
Ahmat[15][2][2]=Ai019[2][2];
Ahmat[15][2][3]=Ai019[2][3];
Ahmat[15][2][4]=Ai019[2][4];

Ahmat[15][3][1]=Ai019[3][1];
Ahmat[15][3][2]=Ai019[3][2];
Ahmat[15][3][3]=Ai019[3][3];
Ahmat[15][3][4]=Ai019[3][4];

Ahmat[15][4][1]=0.;
Ahmat[15][4][2]=0.;
Ahmat[15][4][3]=0.;
Ahmat[15][4][4]=1;


/* joint ID: 42 */
Xorigin[42][1]=Ai020[1][4];
Xorigin[42][2]=Ai020[2][4];
Xorigin[42][3]=Ai020[3][4];

Xmcog[42][1]=links[42].mcm[1]*Ai020[1][1] + links[42].mcm[2]*Ai020[1][2] + links[42].mcm[3]*Ai020[1][3] + links[42].m*Ai020[1][4];
Xmcog[42][2]=links[42].mcm[1]*Ai020[2][1] + links[42].mcm[2]*Ai020[2][2] + links[42].mcm[3]*Ai020[2][3] + links[42].m*Ai020[2][4];
Xmcog[42][3]=links[42].mcm[1]*Ai020[3][1] + links[42].mcm[2]*Ai020[3][2] + links[42].mcm[3]*Ai020[3][3] + links[42].m*Ai020[3][4];

Xaxis[42][1]=Ai020[1][3];
Xaxis[42][2]=Ai020[2][3];
Xaxis[42][3]=Ai020[3][3];

Ahmatdof[42][1][1]=Ai020[1][1];
Ahmatdof[42][1][2]=Ai020[1][2];
Ahmatdof[42][1][3]=Ai020[1][3];
Ahmatdof[42][1][4]=Ai020[1][4];

Ahmatdof[42][2][1]=Ai020[2][1];
Ahmatdof[42][2][2]=Ai020[2][2];
Ahmatdof[42][2][3]=Ai020[2][3];
Ahmatdof[42][2][4]=Ai020[2][4];

Ahmatdof[42][3][1]=Ai020[3][1];
Ahmatdof[42][3][2]=Ai020[3][2];
Ahmatdof[42][3][3]=Ai020[3][3];
Ahmatdof[42][3][4]=Ai020[3][4];

Ahmatdof[42][4][1]=0.;
Ahmatdof[42][4][2]=0.;
Ahmatdof[42][4][3]=0.;
Ahmatdof[42][4][4]=1;


/* link 16: {XMF, YMF, -ZMF} */
Xlink[16][1]=Ai020[1][4];
Xlink[16][2]=Ai020[2][4];
Xlink[16][3]=Ai020[3][4];

Ahmat[16][1][1]=Ai020[1][1];
Ahmat[16][1][2]=Ai020[1][2];
Ahmat[16][1][3]=Ai020[1][3];
Ahmat[16][1][4]=Ai020[1][4];

Ahmat[16][2][1]=Ai020[2][1];
Ahmat[16][2][2]=Ai020[2][2];
Ahmat[16][2][3]=Ai020[2][3];
Ahmat[16][2][4]=Ai020[2][4];

Ahmat[16][3][1]=Ai020[3][1];
Ahmat[16][3][2]=Ai020[3][2];
Ahmat[16][3][3]=Ai020[3][3];
Ahmat[16][3][4]=Ai020[3][4];

Ahmat[16][4][1]=0.;
Ahmat[16][4][2]=0.;
Ahmat[16][4][3]=0.;
Ahmat[16][4][4]=1;


/* link 17: {F1SEG, 0, 0} */
Xlink[17][1]=Ai021[1][4];
Xlink[17][2]=Ai021[2][4];
Xlink[17][3]=Ai021[3][4];

Ahmat[17][1][1]=Ai021[1][1];
Ahmat[17][1][2]=Ai021[1][2];
Ahmat[17][1][3]=Ai021[1][3];
Ahmat[17][1][4]=Ai021[1][4];

Ahmat[17][2][1]=Ai021[2][1];
Ahmat[17][2][2]=Ai021[2][2];
Ahmat[17][2][3]=Ai021[2][3];
Ahmat[17][2][4]=Ai021[2][4];

Ahmat[17][3][1]=Ai021[3][1];
Ahmat[17][3][2]=Ai021[3][2];
Ahmat[17][3][3]=Ai021[3][3];
Ahmat[17][3][4]=Ai021[3][4];

Ahmat[17][4][1]=0.;
Ahmat[17][4][2]=0.;
Ahmat[17][4][3]=0.;
Ahmat[17][4][4]=1;


/* link 18: {F2SEG, 0, 0} */
Xlink[18][1]=Ai022[1][4];
Xlink[18][2]=Ai022[2][4];
Xlink[18][3]=Ai022[3][4];

Ahmat[18][1][1]=Ai022[1][1];
Ahmat[18][1][2]=Ai022[1][2];
Ahmat[18][1][3]=Ai022[1][3];
Ahmat[18][1][4]=Ai022[1][4];

Ahmat[18][2][1]=Ai022[2][1];
Ahmat[18][2][2]=Ai022[2][2];
Ahmat[18][2][3]=Ai022[2][3];
Ahmat[18][2][4]=Ai022[2][4];

Ahmat[18][3][1]=Ai022[3][1];
Ahmat[18][3][2]=Ai022[3][2];
Ahmat[18][3][3]=Ai022[3][3];
Ahmat[18][3][4]=Ai022[3][4];

Ahmat[18][4][1]=0.;
Ahmat[18][4][2]=0.;
Ahmat[18][4][3]=0.;
Ahmat[18][4][4]=1;


/* link 19: {F3SEG, 0, 0} */
Xlink[19][1]=Ai023[1][4];
Xlink[19][2]=Ai023[2][4];
Xlink[19][3]=Ai023[3][4];

Ahmat[19][1][1]=Ai023[1][1];
Ahmat[19][1][2]=Ai023[1][2];
Ahmat[19][1][3]=Ai023[1][3];
Ahmat[19][1][4]=Ai023[1][4];

Ahmat[19][2][1]=Ai023[2][1];
Ahmat[19][2][2]=Ai023[2][2];
Ahmat[19][2][3]=Ai023[2][3];
Ahmat[19][2][4]=Ai023[2][4];

Ahmat[19][3][1]=Ai023[3][1];
Ahmat[19][3][2]=Ai023[3][2];
Ahmat[19][3][3]=Ai023[3][3];
Ahmat[19][3][4]=Ai023[3][4];

Ahmat[19][4][1]=0.;
Ahmat[19][4][2]=0.;
Ahmat[19][4][3]=0.;
Ahmat[19][4][4]=1;


/* joint ID: 43 */
Xorigin[43][1]=Ai024[1][4];
Xorigin[43][2]=Ai024[2][4];
Xorigin[43][3]=Ai024[3][4];

Xmcog[43][1]=links[43].mcm[1]*Ai024[1][1] + links[43].mcm[2]*Ai024[1][2] + links[43].mcm[3]*Ai024[1][3] + links[43].m*Ai024[1][4];
Xmcog[43][2]=links[43].mcm[1]*Ai024[2][1] + links[43].mcm[2]*Ai024[2][2] + links[43].mcm[3]*Ai024[2][3] + links[43].m*Ai024[2][4];
Xmcog[43][3]=links[43].mcm[1]*Ai024[3][1] + links[43].mcm[2]*Ai024[3][2] + links[43].mcm[3]*Ai024[3][3] + links[43].m*Ai024[3][4];

Xaxis[43][1]=Ai024[1][3];
Xaxis[43][2]=Ai024[2][3];
Xaxis[43][3]=Ai024[3][3];

Ahmatdof[43][1][1]=Ai024[1][1];
Ahmatdof[43][1][2]=Ai024[1][2];
Ahmatdof[43][1][3]=Ai024[1][3];
Ahmatdof[43][1][4]=Ai024[1][4];

Ahmatdof[43][2][1]=Ai024[2][1];
Ahmatdof[43][2][2]=Ai024[2][2];
Ahmatdof[43][2][3]=Ai024[2][3];
Ahmatdof[43][2][4]=Ai024[2][4];

Ahmatdof[43][3][1]=Ai024[3][1];
Ahmatdof[43][3][2]=Ai024[3][2];
Ahmatdof[43][3][3]=Ai024[3][3];
Ahmatdof[43][3][4]=Ai024[3][4];

Ahmatdof[43][4][1]=0.;
Ahmatdof[43][4][2]=0.;
Ahmatdof[43][4][3]=0.;
Ahmatdof[43][4][4]=1;


/* link 20: {XRF, YRF, -ZRF} */
Xlink[20][1]=Ai024[1][4];
Xlink[20][2]=Ai024[2][4];
Xlink[20][3]=Ai024[3][4];

Ahmat[20][1][1]=Ai024[1][1];
Ahmat[20][1][2]=Ai024[1][2];
Ahmat[20][1][3]=Ai024[1][3];
Ahmat[20][1][4]=Ai024[1][4];

Ahmat[20][2][1]=Ai024[2][1];
Ahmat[20][2][2]=Ai024[2][2];
Ahmat[20][2][3]=Ai024[2][3];
Ahmat[20][2][4]=Ai024[2][4];

Ahmat[20][3][1]=Ai024[3][1];
Ahmat[20][3][2]=Ai024[3][2];
Ahmat[20][3][3]=Ai024[3][3];
Ahmat[20][3][4]=Ai024[3][4];

Ahmat[20][4][1]=0.;
Ahmat[20][4][2]=0.;
Ahmat[20][4][3]=0.;
Ahmat[20][4][4]=1;


/* link 21: {F1SEG, 0, 0} */
Xlink[21][1]=Ai025[1][4];
Xlink[21][2]=Ai025[2][4];
Xlink[21][3]=Ai025[3][4];

Ahmat[21][1][1]=Ai025[1][1];
Ahmat[21][1][2]=Ai025[1][2];
Ahmat[21][1][3]=Ai025[1][3];
Ahmat[21][1][4]=Ai025[1][4];

Ahmat[21][2][1]=Ai025[2][1];
Ahmat[21][2][2]=Ai025[2][2];
Ahmat[21][2][3]=Ai025[2][3];
Ahmat[21][2][4]=Ai025[2][4];

Ahmat[21][3][1]=Ai025[3][1];
Ahmat[21][3][2]=Ai025[3][2];
Ahmat[21][3][3]=Ai025[3][3];
Ahmat[21][3][4]=Ai025[3][4];

Ahmat[21][4][1]=0.;
Ahmat[21][4][2]=0.;
Ahmat[21][4][3]=0.;
Ahmat[21][4][4]=1;


/* link 22: {F2SEG, 0, 0} */
Xlink[22][1]=Ai026[1][4];
Xlink[22][2]=Ai026[2][4];
Xlink[22][3]=Ai026[3][4];

Ahmat[22][1][1]=Ai026[1][1];
Ahmat[22][1][2]=Ai026[1][2];
Ahmat[22][1][3]=Ai026[1][3];
Ahmat[22][1][4]=Ai026[1][4];

Ahmat[22][2][1]=Ai026[2][1];
Ahmat[22][2][2]=Ai026[2][2];
Ahmat[22][2][3]=Ai026[2][3];
Ahmat[22][2][4]=Ai026[2][4];

Ahmat[22][3][1]=Ai026[3][1];
Ahmat[22][3][2]=Ai026[3][2];
Ahmat[22][3][3]=Ai026[3][3];
Ahmat[22][3][4]=Ai026[3][4];

Ahmat[22][4][1]=0.;
Ahmat[22][4][2]=0.;
Ahmat[22][4][3]=0.;
Ahmat[22][4][4]=1;


/* link 23: {F3SEG, 0, 0} */
Xlink[23][1]=Ai027[1][4];
Xlink[23][2]=Ai027[2][4];
Xlink[23][3]=Ai027[3][4];

Ahmat[23][1][1]=Ai027[1][1];
Ahmat[23][1][2]=Ai027[1][2];
Ahmat[23][1][3]=Ai027[1][3];
Ahmat[23][1][4]=Ai027[1][4];

Ahmat[23][2][1]=Ai027[2][1];
Ahmat[23][2][2]=Ai027[2][2];
Ahmat[23][2][3]=Ai027[2][3];
Ahmat[23][2][4]=Ai027[2][4];

Ahmat[23][3][1]=Ai027[3][1];
Ahmat[23][3][2]=Ai027[3][2];
Ahmat[23][3][3]=Ai027[3][3];
Ahmat[23][3][4]=Ai027[3][4];

Ahmat[23][4][1]=0.;
Ahmat[23][4][2]=0.;
Ahmat[23][4][3]=0.;
Ahmat[23][4][4]=1;


/* joint ID: 44 */
Xorigin[44][1]=Ai028[1][4];
Xorigin[44][2]=Ai028[2][4];
Xorigin[44][3]=Ai028[3][4];

Xmcog[44][1]=links[44].mcm[1]*Ai028[1][1] + links[44].mcm[2]*Ai028[1][2] + links[44].mcm[3]*Ai028[1][3] + links[44].m*Ai028[1][4];
Xmcog[44][2]=links[44].mcm[1]*Ai028[2][1] + links[44].mcm[2]*Ai028[2][2] + links[44].mcm[3]*Ai028[2][3] + links[44].m*Ai028[2][4];
Xmcog[44][3]=links[44].mcm[1]*Ai028[3][1] + links[44].mcm[2]*Ai028[3][2] + links[44].mcm[3]*Ai028[3][3] + links[44].m*Ai028[3][4];

Xaxis[44][1]=Ai028[1][3];
Xaxis[44][2]=Ai028[2][3];
Xaxis[44][3]=Ai028[3][3];

Ahmatdof[44][1][1]=Ai028[1][1];
Ahmatdof[44][1][2]=Ai028[1][2];
Ahmatdof[44][1][3]=Ai028[1][3];
Ahmatdof[44][1][4]=Ai028[1][4];

Ahmatdof[44][2][1]=Ai028[2][1];
Ahmatdof[44][2][2]=Ai028[2][2];
Ahmatdof[44][2][3]=Ai028[2][3];
Ahmatdof[44][2][4]=Ai028[2][4];

Ahmatdof[44][3][1]=Ai028[3][1];
Ahmatdof[44][3][2]=Ai028[3][2];
Ahmatdof[44][3][3]=Ai028[3][3];
Ahmatdof[44][3][4]=Ai028[3][4];

Ahmatdof[44][4][1]=0.;
Ahmatdof[44][4][2]=0.;
Ahmatdof[44][4][3]=0.;
Ahmatdof[44][4][4]=1;


/* link 24: {XLF, YLF, -ZLF} */
Xlink[24][1]=Ai028[1][4];
Xlink[24][2]=Ai028[2][4];
Xlink[24][3]=Ai028[3][4];

Ahmat[24][1][1]=Ai028[1][1];
Ahmat[24][1][2]=Ai028[1][2];
Ahmat[24][1][3]=Ai028[1][3];
Ahmat[24][1][4]=Ai028[1][4];

Ahmat[24][2][1]=Ai028[2][1];
Ahmat[24][2][2]=Ai028[2][2];
Ahmat[24][2][3]=Ai028[2][3];
Ahmat[24][2][4]=Ai028[2][4];

Ahmat[24][3][1]=Ai028[3][1];
Ahmat[24][3][2]=Ai028[3][2];
Ahmat[24][3][3]=Ai028[3][3];
Ahmat[24][3][4]=Ai028[3][4];

Ahmat[24][4][1]=0.;
Ahmat[24][4][2]=0.;
Ahmat[24][4][3]=0.;
Ahmat[24][4][4]=1;


/* link 25: {F1SEG, 0, 0} */
Xlink[25][1]=Ai029[1][4];
Xlink[25][2]=Ai029[2][4];
Xlink[25][3]=Ai029[3][4];

Ahmat[25][1][1]=Ai029[1][1];
Ahmat[25][1][2]=Ai029[1][2];
Ahmat[25][1][3]=Ai029[1][3];
Ahmat[25][1][4]=Ai029[1][4];

Ahmat[25][2][1]=Ai029[2][1];
Ahmat[25][2][2]=Ai029[2][2];
Ahmat[25][2][3]=Ai029[2][3];
Ahmat[25][2][4]=Ai029[2][4];

Ahmat[25][3][1]=Ai029[3][1];
Ahmat[25][3][2]=Ai029[3][2];
Ahmat[25][3][3]=Ai029[3][3];
Ahmat[25][3][4]=Ai029[3][4];

Ahmat[25][4][1]=0.;
Ahmat[25][4][2]=0.;
Ahmat[25][4][3]=0.;
Ahmat[25][4][4]=1;


/* link 26: {F2SEG, 0, 0} */
Xlink[26][1]=Ai030[1][4];
Xlink[26][2]=Ai030[2][4];
Xlink[26][3]=Ai030[3][4];

Ahmat[26][1][1]=Ai030[1][1];
Ahmat[26][1][2]=Ai030[1][2];
Ahmat[26][1][3]=Ai030[1][3];
Ahmat[26][1][4]=Ai030[1][4];

Ahmat[26][2][1]=Ai030[2][1];
Ahmat[26][2][2]=Ai030[2][2];
Ahmat[26][2][3]=Ai030[2][3];
Ahmat[26][2][4]=Ai030[2][4];

Ahmat[26][3][1]=Ai030[3][1];
Ahmat[26][3][2]=Ai030[3][2];
Ahmat[26][3][3]=Ai030[3][3];
Ahmat[26][3][4]=Ai030[3][4];

Ahmat[26][4][1]=0.;
Ahmat[26][4][2]=0.;
Ahmat[26][4][3]=0.;
Ahmat[26][4][4]=1;


/* link 27: {F3SEG, 0, 0} */
Xlink[27][1]=Ai031[1][4];
Xlink[27][2]=Ai031[2][4];
Xlink[27][3]=Ai031[3][4];

Ahmat[27][1][1]=Ai031[1][1];
Ahmat[27][1][2]=Ai031[1][2];
Ahmat[27][1][3]=Ai031[1][3];
Ahmat[27][1][4]=Ai031[1][4];

Ahmat[27][2][1]=Ai031[2][1];
Ahmat[27][2][2]=Ai031[2][2];
Ahmat[27][2][3]=Ai031[2][3];
Ahmat[27][2][4]=Ai031[2][4];

Ahmat[27][3][1]=Ai031[3][1];
Ahmat[27][3][2]=Ai031[3][2];
Ahmat[27][3][3]=Ai031[3][3];
Ahmat[27][3][4]=Ai031[3][4];

Ahmat[27][4][1]=0.;
Ahmat[27][4][2]=0.;
Ahmat[27][4][3]=0.;
Ahmat[27][4][4]=1;


/* joint ID: 8 */
Xorigin[8][1]=Ai032[1][4];
Xorigin[8][2]=Ai032[2][4];
Xorigin[8][3]=Ai032[3][4];

Xmcog[8][1]=links[8].mcm[1]*Ai032[1][1] + links[8].mcm[2]*Ai032[1][2] + links[8].mcm[3]*Ai032[1][3] + links[8].m*Ai032[1][4];
Xmcog[8][2]=links[8].mcm[1]*Ai032[2][1] + links[8].mcm[2]*Ai032[2][2] + links[8].mcm[3]*Ai032[2][3] + links[8].m*Ai032[2][4];
Xmcog[8][3]=links[8].mcm[1]*Ai032[3][1] + links[8].mcm[2]*Ai032[3][2] + links[8].mcm[3]*Ai032[3][3] + links[8].m*Ai032[3][4];

Xaxis[8][1]=Ai032[1][3];
Xaxis[8][2]=Ai032[2][3];
Xaxis[8][3]=Ai032[3][3];

Ahmatdof[8][1][1]=Ai032[1][1];
Ahmatdof[8][1][2]=Ai032[1][2];
Ahmatdof[8][1][3]=Ai032[1][3];
Ahmatdof[8][1][4]=Ai032[1][4];

Ahmatdof[8][2][1]=Ai032[2][1];
Ahmatdof[8][2][2]=Ai032[2][2];
Ahmatdof[8][2][3]=Ai032[2][3];
Ahmatdof[8][2][4]=Ai032[2][4];

Ahmatdof[8][3][1]=Ai032[3][1];
Ahmatdof[8][3][2]=Ai032[3][2];
Ahmatdof[8][3][3]=Ai032[3][3];
Ahmatdof[8][3][4]=Ai032[3][4];

Ahmatdof[8][4][1]=0.;
Ahmatdof[8][4][2]=0.;
Ahmatdof[8][4][3]=0.;
Ahmatdof[8][4][4]=1;


/* link 28: {-THORAX2SHOULDER, 0, 0} */
Xlink[28][1]=Ai032[1][4];
Xlink[28][2]=Ai032[2][4];
Xlink[28][3]=Ai032[3][4];

Ahmat[28][1][1]=Ai032[1][1];
Ahmat[28][1][2]=Ai032[1][2];
Ahmat[28][1][3]=Ai032[1][3];
Ahmat[28][1][4]=Ai032[1][4];

Ahmat[28][2][1]=Ai032[2][1];
Ahmat[28][2][2]=Ai032[2][2];
Ahmat[28][2][3]=Ai032[2][3];
Ahmat[28][2][4]=Ai032[2][4];

Ahmat[28][3][1]=Ai032[3][1];
Ahmat[28][3][2]=Ai032[3][2];
Ahmat[28][3][3]=Ai032[3][3];
Ahmat[28][3][4]=Ai032[3][4];

Ahmat[28][4][1]=0.;
Ahmat[28][4][2]=0.;
Ahmat[28][4][3]=0.;
Ahmat[28][4][4]=1;


/* joint ID: 9 */
Xorigin[9][1]=Ai033[1][4];
Xorigin[9][2]=Ai033[2][4];
Xorigin[9][3]=Ai033[3][4];

Xmcog[9][1]=links[9].mcm[1]*Ai033[1][1] + links[9].mcm[2]*Ai033[1][2] + links[9].mcm[3]*Ai033[1][3] + links[9].m*Ai033[1][4];
Xmcog[9][2]=links[9].mcm[1]*Ai033[2][1] + links[9].mcm[2]*Ai033[2][2] + links[9].mcm[3]*Ai033[2][3] + links[9].m*Ai033[2][4];
Xmcog[9][3]=links[9].mcm[1]*Ai033[3][1] + links[9].mcm[2]*Ai033[3][2] + links[9].mcm[3]*Ai033[3][3] + links[9].m*Ai033[3][4];

Xaxis[9][1]=Ai033[1][3];
Xaxis[9][2]=Ai033[2][3];
Xaxis[9][3]=Ai033[3][3];

Ahmatdof[9][1][1]=Ai033[1][1];
Ahmatdof[9][1][2]=Ai033[1][2];
Ahmatdof[9][1][3]=Ai033[1][3];
Ahmatdof[9][1][4]=Ai033[1][4];

Ahmatdof[9][2][1]=Ai033[2][1];
Ahmatdof[9][2][2]=Ai033[2][2];
Ahmatdof[9][2][3]=Ai033[2][3];
Ahmatdof[9][2][4]=Ai033[2][4];

Ahmatdof[9][3][1]=Ai033[3][1];
Ahmatdof[9][3][2]=Ai033[3][2];
Ahmatdof[9][3][3]=Ai033[3][3];
Ahmatdof[9][3][4]=Ai033[3][4];

Ahmatdof[9][4][1]=0.;
Ahmatdof[9][4][2]=0.;
Ahmatdof[9][4][3]=0.;
Ahmatdof[9][4][4]=1;


/* link 29: {0, 0, SHOULDERX} */
Xlink[29][1]=Ai033[1][4];
Xlink[29][2]=Ai033[2][4];
Xlink[29][3]=Ai033[3][4];

Ahmat[29][1][1]=Ai033[1][1];
Ahmat[29][1][2]=Ai033[1][2];
Ahmat[29][1][3]=Ai033[1][3];
Ahmat[29][1][4]=Ai033[1][4];

Ahmat[29][2][1]=Ai033[2][1];
Ahmat[29][2][2]=Ai033[2][2];
Ahmat[29][2][3]=Ai033[2][3];
Ahmat[29][2][4]=Ai033[2][4];

Ahmat[29][3][1]=Ai033[3][1];
Ahmat[29][3][2]=Ai033[3][2];
Ahmat[29][3][3]=Ai033[3][3];
Ahmat[29][3][4]=Ai033[3][4];

Ahmat[29][4][1]=0.;
Ahmat[29][4][2]=0.;
Ahmat[29][4][3]=0.;
Ahmat[29][4][4]=1;


/* joint ID: 10 */
Xorigin[10][1]=Ai034[1][4];
Xorigin[10][2]=Ai034[2][4];
Xorigin[10][3]=Ai034[3][4];

Xmcog[10][1]=links[10].mcm[1]*Ai034[1][1] + links[10].mcm[2]*Ai034[1][2] + links[10].mcm[3]*Ai034[1][3] + links[10].m*Ai034[1][4];
Xmcog[10][2]=links[10].mcm[1]*Ai034[2][1] + links[10].mcm[2]*Ai034[2][2] + links[10].mcm[3]*Ai034[2][3] + links[10].m*Ai034[2][4];
Xmcog[10][3]=links[10].mcm[1]*Ai034[3][1] + links[10].mcm[2]*Ai034[3][2] + links[10].mcm[3]*Ai034[3][3] + links[10].m*Ai034[3][4];

Xaxis[10][1]=Ai034[1][3];
Xaxis[10][2]=Ai034[2][3];
Xaxis[10][3]=Ai034[3][3];

Ahmatdof[10][1][1]=Ai034[1][1];
Ahmatdof[10][1][2]=Ai034[1][2];
Ahmatdof[10][1][3]=Ai034[1][3];
Ahmatdof[10][1][4]=Ai034[1][4];

Ahmatdof[10][2][1]=Ai034[2][1];
Ahmatdof[10][2][2]=Ai034[2][2];
Ahmatdof[10][2][3]=Ai034[2][3];
Ahmatdof[10][2][4]=Ai034[2][4];

Ahmatdof[10][3][1]=Ai034[3][1];
Ahmatdof[10][3][2]=Ai034[3][2];
Ahmatdof[10][3][3]=Ai034[3][3];
Ahmatdof[10][3][4]=Ai034[3][4];

Ahmatdof[10][4][1]=0.;
Ahmatdof[10][4][2]=0.;
Ahmatdof[10][4][3]=0.;
Ahmatdof[10][4][4]=1;


/* link 30: {-SHOULDERY, 0, 0} */
Xlink[30][1]=Ai034[1][4];
Xlink[30][2]=Ai034[2][4];
Xlink[30][3]=Ai034[3][4];

Ahmat[30][1][1]=Ai034[1][1];
Ahmat[30][1][2]=Ai034[1][2];
Ahmat[30][1][3]=Ai034[1][3];
Ahmat[30][1][4]=Ai034[1][4];

Ahmat[30][2][1]=Ai034[2][1];
Ahmat[30][2][2]=Ai034[2][2];
Ahmat[30][2][3]=Ai034[2][3];
Ahmat[30][2][4]=Ai034[2][4];

Ahmat[30][3][1]=Ai034[3][1];
Ahmat[30][3][2]=Ai034[3][2];
Ahmat[30][3][3]=Ai034[3][3];
Ahmat[30][3][4]=Ai034[3][4];

Ahmat[30][4][1]=0.;
Ahmat[30][4][2]=0.;
Ahmat[30][4][3]=0.;
Ahmat[30][4][4]=1;


/* joint ID: 11 */
Xorigin[11][1]=Ai035[1][4];
Xorigin[11][2]=Ai035[2][4];
Xorigin[11][3]=Ai035[3][4];

Xmcog[11][1]=links[11].mcm[1]*Ai035[1][1] + links[11].mcm[2]*Ai035[1][2] + links[11].mcm[3]*Ai035[1][3] + links[11].m*Ai035[1][4];
Xmcog[11][2]=links[11].mcm[1]*Ai035[2][1] + links[11].mcm[2]*Ai035[2][2] + links[11].mcm[3]*Ai035[2][3] + links[11].m*Ai035[2][4];
Xmcog[11][3]=links[11].mcm[1]*Ai035[3][1] + links[11].mcm[2]*Ai035[3][2] + links[11].mcm[3]*Ai035[3][3] + links[11].m*Ai035[3][4];

Xaxis[11][1]=Ai035[1][3];
Xaxis[11][2]=Ai035[2][3];
Xaxis[11][3]=Ai035[3][3];

Ahmatdof[11][1][1]=Ai035[1][1];
Ahmatdof[11][1][2]=Ai035[1][2];
Ahmatdof[11][1][3]=Ai035[1][3];
Ahmatdof[11][1][4]=Ai035[1][4];

Ahmatdof[11][2][1]=Ai035[2][1];
Ahmatdof[11][2][2]=Ai035[2][2];
Ahmatdof[11][2][3]=Ai035[2][3];
Ahmatdof[11][2][4]=Ai035[2][4];

Ahmatdof[11][3][1]=Ai035[3][1];
Ahmatdof[11][3][2]=Ai035[3][2];
Ahmatdof[11][3][3]=Ai035[3][3];
Ahmatdof[11][3][4]=Ai035[3][4];

Ahmatdof[11][4][1]=0.;
Ahmatdof[11][4][2]=0.;
Ahmatdof[11][4][3]=0.;
Ahmatdof[11][4][4]=1;


/* link 31: {0, 0, UPPERARM} */
Xlink[31][1]=Ai035[1][4];
Xlink[31][2]=Ai035[2][4];
Xlink[31][3]=Ai035[3][4];

Ahmat[31][1][1]=Ai036[1][1];
Ahmat[31][1][2]=Ai036[1][2];
Ahmat[31][1][3]=Ai036[1][3];
Ahmat[31][1][4]=Ai036[1][4];

Ahmat[31][2][1]=Ai036[2][1];
Ahmat[31][2][2]=Ai036[2][2];
Ahmat[31][2][3]=Ai036[2][3];
Ahmat[31][2][4]=Ai036[2][4];

Ahmat[31][3][1]=Ai036[3][1];
Ahmat[31][3][2]=Ai036[3][2];
Ahmat[31][3][3]=Ai036[3][3];
Ahmat[31][3][4]=Ai036[3][4];

Ahmat[31][4][1]=0.;
Ahmat[31][4][2]=0.;
Ahmat[31][4][3]=0.;
Ahmat[31][4][4]=1;


/* joint ID: 12 */
Xorigin[12][1]=Ai036[1][4];
Xorigin[12][2]=Ai036[2][4];
Xorigin[12][3]=Ai036[3][4];

Xmcog[12][1]=links[12].mcm[1]*Ai036[1][1] + links[12].mcm[2]*Ai036[1][2] + links[12].mcm[3]*Ai036[1][3] + links[12].m*Ai036[1][4];
Xmcog[12][2]=links[12].mcm[1]*Ai036[2][1] + links[12].mcm[2]*Ai036[2][2] + links[12].mcm[3]*Ai036[2][3] + links[12].m*Ai036[2][4];
Xmcog[12][3]=links[12].mcm[1]*Ai036[3][1] + links[12].mcm[2]*Ai036[3][2] + links[12].mcm[3]*Ai036[3][3] + links[12].m*Ai036[3][4];

Xaxis[12][1]=Ai036[1][3];
Xaxis[12][2]=Ai036[2][3];
Xaxis[12][3]=Ai036[3][3];

Ahmatdof[12][1][1]=Ai036[1][1];
Ahmatdof[12][1][2]=Ai036[1][2];
Ahmatdof[12][1][3]=Ai036[1][3];
Ahmatdof[12][1][4]=Ai036[1][4];

Ahmatdof[12][2][1]=Ai036[2][1];
Ahmatdof[12][2][2]=Ai036[2][2];
Ahmatdof[12][2][3]=Ai036[2][3];
Ahmatdof[12][2][4]=Ai036[2][4];

Ahmatdof[12][3][1]=Ai036[3][1];
Ahmatdof[12][3][2]=Ai036[3][2];
Ahmatdof[12][3][3]=Ai036[3][3];
Ahmatdof[12][3][4]=Ai036[3][4];

Ahmatdof[12][4][1]=0.;
Ahmatdof[12][4][2]=0.;
Ahmatdof[12][4][3]=0.;
Ahmatdof[12][4][4]=1;


/* joint ID: 13 */
Xorigin[13][1]=Ai037[1][4];
Xorigin[13][2]=Ai037[2][4];
Xorigin[13][3]=Ai037[3][4];

Xmcog[13][1]=links[13].mcm[1]*Ai037[1][1] + links[13].mcm[2]*Ai037[1][2] + links[13].mcm[3]*Ai037[1][3] + links[13].m*Ai037[1][4];
Xmcog[13][2]=links[13].mcm[1]*Ai037[2][1] + links[13].mcm[2]*Ai037[2][2] + links[13].mcm[3]*Ai037[2][3] + links[13].m*Ai037[2][4];
Xmcog[13][3]=links[13].mcm[1]*Ai037[3][1] + links[13].mcm[2]*Ai037[3][2] + links[13].mcm[3]*Ai037[3][3] + links[13].m*Ai037[3][4];

Xaxis[13][1]=Ai037[1][3];
Xaxis[13][2]=Ai037[2][3];
Xaxis[13][3]=Ai037[3][3];

Ahmatdof[13][1][1]=Ai037[1][1];
Ahmatdof[13][1][2]=Ai037[1][2];
Ahmatdof[13][1][3]=Ai037[1][3];
Ahmatdof[13][1][4]=Ai037[1][4];

Ahmatdof[13][2][1]=Ai037[2][1];
Ahmatdof[13][2][2]=Ai037[2][2];
Ahmatdof[13][2][3]=Ai037[2][3];
Ahmatdof[13][2][4]=Ai037[2][4];

Ahmatdof[13][3][1]=Ai037[3][1];
Ahmatdof[13][3][2]=Ai037[3][2];
Ahmatdof[13][3][3]=Ai037[3][3];
Ahmatdof[13][3][4]=Ai037[3][4];

Ahmatdof[13][4][1]=0.;
Ahmatdof[13][4][2]=0.;
Ahmatdof[13][4][3]=0.;
Ahmatdof[13][4][4]=1;


/* link 32: {0, WRISTY, LOWERARM} */
Xlink[32][1]=Ai037[1][4];
Xlink[32][2]=Ai037[2][4];
Xlink[32][3]=Ai037[3][4];

Ahmat[32][1][1]=Ai038[1][1];
Ahmat[32][1][2]=Ai038[1][2];
Ahmat[32][1][3]=Ai038[1][3];
Ahmat[32][1][4]=Ai038[1][4];

Ahmat[32][2][1]=Ai038[2][1];
Ahmat[32][2][2]=Ai038[2][2];
Ahmat[32][2][3]=Ai038[2][3];
Ahmat[32][2][4]=Ai038[2][4];

Ahmat[32][3][1]=Ai038[3][1];
Ahmat[32][3][2]=Ai038[3][2];
Ahmat[32][3][3]=Ai038[3][3];
Ahmat[32][3][4]=Ai038[3][4];

Ahmat[32][4][1]=0.;
Ahmat[32][4][2]=0.;
Ahmat[32][4][3]=0.;
Ahmat[32][4][4]=1;


/* joint ID: 14 */
Xorigin[14][1]=Ai038[1][4];
Xorigin[14][2]=Ai038[2][4];
Xorigin[14][3]=Ai038[3][4];

Xmcog[14][1]=links[14].mcm[1]*Ai038[1][1] + links[14].mcm[2]*Ai038[1][2] + links[14].mcm[3]*Ai038[1][3] + links[14].m*Ai038[1][4];
Xmcog[14][2]=links[14].mcm[1]*Ai038[2][1] + links[14].mcm[2]*Ai038[2][2] + links[14].mcm[3]*Ai038[2][3] + links[14].m*Ai038[2][4];
Xmcog[14][3]=links[14].mcm[1]*Ai038[3][1] + links[14].mcm[2]*Ai038[3][2] + links[14].mcm[3]*Ai038[3][3] + links[14].m*Ai038[3][4];

Xaxis[14][1]=Ai038[1][3];
Xaxis[14][2]=Ai038[2][3];
Xaxis[14][3]=Ai038[3][3];

Ahmatdof[14][1][1]=Ai038[1][1];
Ahmatdof[14][1][2]=Ai038[1][2];
Ahmatdof[14][1][3]=Ai038[1][3];
Ahmatdof[14][1][4]=Ai038[1][4];

Ahmatdof[14][2][1]=Ai038[2][1];
Ahmatdof[14][2][2]=Ai038[2][2];
Ahmatdof[14][2][3]=Ai038[2][3];
Ahmatdof[14][2][4]=Ai038[2][4];

Ahmatdof[14][3][1]=Ai038[3][1];
Ahmatdof[14][3][2]=Ai038[3][2];
Ahmatdof[14][3][3]=Ai038[3][3];
Ahmatdof[14][3][4]=Ai038[3][4];

Ahmatdof[14][4][1]=0.;
Ahmatdof[14][4][2]=0.;
Ahmatdof[14][4][3]=0.;
Ahmatdof[14][4][4]=1;


/* link 33: {eff$1$$x[[1]], eff$1$$x[[2]], eff$1$$x[[3]]} */
Xlink[33][1]=Ai039[1][4];
Xlink[33][2]=Ai039[2][4];
Xlink[33][3]=Ai039[3][4];

Ahmat[33][1][1]=Ai039[1][1];
Ahmat[33][1][2]=Ai039[1][2];
Ahmat[33][1][3]=Ai039[1][3];
Ahmat[33][1][4]=Ai039[1][4];

Ahmat[33][2][1]=Ai039[2][1];
Ahmat[33][2][2]=Ai039[2][2];
Ahmat[33][2][3]=Ai039[2][3];
Ahmat[33][2][4]=Ai039[2][4];

Ahmat[33][3][1]=Ai039[3][1];
Ahmat[33][3][2]=Ai039[3][2];
Ahmat[33][3][3]=Ai039[3][3];
Ahmat[33][3][4]=Ai039[3][4];

Ahmat[33][4][1]=0.;
Ahmat[33][4][2]=0.;
Ahmat[33][4][3]=0.;
Ahmat[33][4][4]=1;


/* joint ID: 45 */
Xorigin[45][1]=Ai040[1][4];
Xorigin[45][2]=Ai040[2][4];
Xorigin[45][3]=Ai040[3][4];

Xmcog[45][1]=links[45].mcm[1]*Ai040[1][1] + links[45].mcm[2]*Ai040[1][2] + links[45].mcm[3]*Ai040[1][3] + links[45].m*Ai040[1][4];
Xmcog[45][2]=links[45].mcm[1]*Ai040[2][1] + links[45].mcm[2]*Ai040[2][2] + links[45].mcm[3]*Ai040[2][3] + links[45].m*Ai040[2][4];
Xmcog[45][3]=links[45].mcm[1]*Ai040[3][1] + links[45].mcm[2]*Ai040[3][2] + links[45].mcm[3]*Ai040[3][3] + links[45].m*Ai040[3][4];

Xaxis[45][1]=Ai040[1][1];
Xaxis[45][2]=Ai040[2][1];
Xaxis[45][3]=Ai040[3][1];

Ahmatdof[45][1][1]=Ai040[1][1];
Ahmatdof[45][1][2]=Ai040[1][2];
Ahmatdof[45][1][3]=Ai040[1][3];
Ahmatdof[45][1][4]=Ai040[1][4];

Ahmatdof[45][2][1]=Ai040[2][1];
Ahmatdof[45][2][2]=Ai040[2][2];
Ahmatdof[45][2][3]=Ai040[2][3];
Ahmatdof[45][2][4]=Ai040[2][4];

Ahmatdof[45][3][1]=Ai040[3][1];
Ahmatdof[45][3][2]=Ai040[3][2];
Ahmatdof[45][3][3]=Ai040[3][3];
Ahmatdof[45][3][4]=Ai040[3][4];

Ahmatdof[45][4][1]=0.;
Ahmatdof[45][4][2]=0.;
Ahmatdof[45][4][3]=0.;
Ahmatdof[45][4][4]=1;


/* link 34: {XTHUMB, YTHUMB, ZTHUMB} */
Xlink[34][1]=Ai040[1][4];
Xlink[34][2]=Ai040[2][4];
Xlink[34][3]=Ai040[3][4];

Ahmat[34][1][1]=Ai040[1][1];
Ahmat[34][1][2]=Ai040[1][2];
Ahmat[34][1][3]=Ai040[1][3];
Ahmat[34][1][4]=Ai040[1][4];

Ahmat[34][2][1]=Ai040[2][1];
Ahmat[34][2][2]=Ai040[2][2];
Ahmat[34][2][3]=Ai040[2][3];
Ahmat[34][2][4]=Ai040[2][4];

Ahmat[34][3][1]=Ai040[3][1];
Ahmat[34][3][2]=Ai040[3][2];
Ahmat[34][3][3]=Ai040[3][3];
Ahmat[34][3][4]=Ai040[3][4];

Ahmat[34][4][1]=0.;
Ahmat[34][4][2]=0.;
Ahmat[34][4][3]=0.;
Ahmat[34][4][4]=1;


/* joint ID: 46 */
Xorigin[46][1]=Ai041[1][4];
Xorigin[46][2]=Ai041[2][4];
Xorigin[46][3]=Ai041[3][4];

Xmcog[46][1]=links[46].mcm[1]*Ai041[1][1] + links[46].mcm[2]*Ai041[1][2] + links[46].mcm[3]*Ai041[1][3] + links[46].m*Ai041[1][4];
Xmcog[46][2]=links[46].mcm[1]*Ai041[2][1] + links[46].mcm[2]*Ai041[2][2] + links[46].mcm[3]*Ai041[2][3] + links[46].m*Ai041[2][4];
Xmcog[46][3]=links[46].mcm[1]*Ai041[3][1] + links[46].mcm[2]*Ai041[3][2] + links[46].mcm[3]*Ai041[3][3] + links[46].m*Ai041[3][4];

Xaxis[46][1]=-Ai041[1][3];
Xaxis[46][2]=-Ai041[2][3];
Xaxis[46][3]=-Ai041[3][3];

Ahmatdof[46][1][1]=Ai041[1][1];
Ahmatdof[46][1][2]=Ai041[1][2];
Ahmatdof[46][1][3]=Ai041[1][3];
Ahmatdof[46][1][4]=Ai041[1][4];

Ahmatdof[46][2][1]=Ai041[2][1];
Ahmatdof[46][2][2]=Ai041[2][2];
Ahmatdof[46][2][3]=Ai041[2][3];
Ahmatdof[46][2][4]=Ai041[2][4];

Ahmatdof[46][3][1]=Ai041[3][1];
Ahmatdof[46][3][2]=Ai041[3][2];
Ahmatdof[46][3][3]=Ai041[3][3];
Ahmatdof[46][3][4]=Ai041[3][4];

Ahmatdof[46][4][1]=0.;
Ahmatdof[46][4][2]=0.;
Ahmatdof[46][4][3]=0.;
Ahmatdof[46][4][4]=1;


/* link 35: {XTHUMBFLEX, YTHUMBFLEX, 0} */
Xlink[35][1]=Ai041[1][4];
Xlink[35][2]=Ai041[2][4];
Xlink[35][3]=Ai041[3][4];

Ahmat[35][1][1]=Ai041[1][1];
Ahmat[35][1][2]=Ai041[1][2];
Ahmat[35][1][3]=Ai041[1][3];
Ahmat[35][1][4]=Ai041[1][4];

Ahmat[35][2][1]=Ai041[2][1];
Ahmat[35][2][2]=Ai041[2][2];
Ahmat[35][2][3]=Ai041[2][3];
Ahmat[35][2][4]=Ai041[2][4];

Ahmat[35][3][1]=Ai041[3][1];
Ahmat[35][3][2]=Ai041[3][2];
Ahmat[35][3][3]=Ai041[3][3];
Ahmat[35][3][4]=Ai041[3][4];

Ahmat[35][4][1]=0.;
Ahmat[35][4][2]=0.;
Ahmat[35][4][3]=0.;
Ahmat[35][4][4]=1;


/* link 36: {TH1SEG, 0, 0} */
Xlink[36][1]=Ai042[1][4];
Xlink[36][2]=Ai042[2][4];
Xlink[36][3]=Ai042[3][4];

Ahmat[36][1][1]=Ai042[1][1];
Ahmat[36][1][2]=Ai042[1][2];
Ahmat[36][1][3]=Ai042[1][3];
Ahmat[36][1][4]=Ai042[1][4];

Ahmat[36][2][1]=Ai042[2][1];
Ahmat[36][2][2]=Ai042[2][2];
Ahmat[36][2][3]=Ai042[2][3];
Ahmat[36][2][4]=Ai042[2][4];

Ahmat[36][3][1]=Ai042[3][1];
Ahmat[36][3][2]=Ai042[3][2];
Ahmat[36][3][3]=Ai042[3][3];
Ahmat[36][3][4]=Ai042[3][4];

Ahmat[36][4][1]=0.;
Ahmat[36][4][2]=0.;
Ahmat[36][4][3]=0.;
Ahmat[36][4][4]=1;


/* link 37: {TH2SEG, 0, 0} */
Xlink[37][1]=Ai043[1][4];
Xlink[37][2]=Ai043[2][4];
Xlink[37][3]=Ai043[3][4];

Ahmat[37][1][1]=Ai043[1][1];
Ahmat[37][1][2]=Ai043[1][2];
Ahmat[37][1][3]=Ai043[1][3];
Ahmat[37][1][4]=Ai043[1][4];

Ahmat[37][2][1]=Ai043[2][1];
Ahmat[37][2][2]=Ai043[2][2];
Ahmat[37][2][3]=Ai043[2][3];
Ahmat[37][2][4]=Ai043[2][4];

Ahmat[37][3][1]=Ai043[3][1];
Ahmat[37][3][2]=Ai043[3][2];
Ahmat[37][3][3]=Ai043[3][3];
Ahmat[37][3][4]=Ai043[3][4];

Ahmat[37][4][1]=0.;
Ahmat[37][4][2]=0.;
Ahmat[37][4][3]=0.;
Ahmat[37][4][4]=1;


/* joint ID: 47 */
Xorigin[47][1]=Ai044[1][4];
Xorigin[47][2]=Ai044[2][4];
Xorigin[47][3]=Ai044[3][4];

Xmcog[47][1]=links[47].mcm[1]*Ai044[1][1] + links[47].mcm[2]*Ai044[1][2] + links[47].mcm[3]*Ai044[1][3] + links[47].m*Ai044[1][4];
Xmcog[47][2]=links[47].mcm[1]*Ai044[2][1] + links[47].mcm[2]*Ai044[2][2] + links[47].mcm[3]*Ai044[2][3] + links[47].m*Ai044[2][4];
Xmcog[47][3]=links[47].mcm[1]*Ai044[3][1] + links[47].mcm[2]*Ai044[3][2] + links[47].mcm[3]*Ai044[3][3] + links[47].m*Ai044[3][4];

Xaxis[47][1]=Ai044[1][3];
Xaxis[47][2]=Ai044[2][3];
Xaxis[47][3]=Ai044[3][3];

Ahmatdof[47][1][1]=Ai044[1][1];
Ahmatdof[47][1][2]=Ai044[1][2];
Ahmatdof[47][1][3]=Ai044[1][3];
Ahmatdof[47][1][4]=Ai044[1][4];

Ahmatdof[47][2][1]=Ai044[2][1];
Ahmatdof[47][2][2]=Ai044[2][2];
Ahmatdof[47][2][3]=Ai044[2][3];
Ahmatdof[47][2][4]=Ai044[2][4];

Ahmatdof[47][3][1]=Ai044[3][1];
Ahmatdof[47][3][2]=Ai044[3][2];
Ahmatdof[47][3][3]=Ai044[3][3];
Ahmatdof[47][3][4]=Ai044[3][4];

Ahmatdof[47][4][1]=0.;
Ahmatdof[47][4][2]=0.;
Ahmatdof[47][4][3]=0.;
Ahmatdof[47][4][4]=1;


/* link 38: {XIF, YIF, ZIF} */
Xlink[38][1]=Ai044[1][4];
Xlink[38][2]=Ai044[2][4];
Xlink[38][3]=Ai044[3][4];

Ahmat[38][1][1]=Ai044[1][1];
Ahmat[38][1][2]=Ai044[1][2];
Ahmat[38][1][3]=Ai044[1][3];
Ahmat[38][1][4]=Ai044[1][4];

Ahmat[38][2][1]=Ai044[2][1];
Ahmat[38][2][2]=Ai044[2][2];
Ahmat[38][2][3]=Ai044[2][3];
Ahmat[38][2][4]=Ai044[2][4];

Ahmat[38][3][1]=Ai044[3][1];
Ahmat[38][3][2]=Ai044[3][2];
Ahmat[38][3][3]=Ai044[3][3];
Ahmat[38][3][4]=Ai044[3][4];

Ahmat[38][4][1]=0.;
Ahmat[38][4][2]=0.;
Ahmat[38][4][3]=0.;
Ahmat[38][4][4]=1;


/* link 39: {F1SEG, 0, 0} */
Xlink[39][1]=Ai045[1][4];
Xlink[39][2]=Ai045[2][4];
Xlink[39][3]=Ai045[3][4];

Ahmat[39][1][1]=Ai045[1][1];
Ahmat[39][1][2]=Ai045[1][2];
Ahmat[39][1][3]=Ai045[1][3];
Ahmat[39][1][4]=Ai045[1][4];

Ahmat[39][2][1]=Ai045[2][1];
Ahmat[39][2][2]=Ai045[2][2];
Ahmat[39][2][3]=Ai045[2][3];
Ahmat[39][2][4]=Ai045[2][4];

Ahmat[39][3][1]=Ai045[3][1];
Ahmat[39][3][2]=Ai045[3][2];
Ahmat[39][3][3]=Ai045[3][3];
Ahmat[39][3][4]=Ai045[3][4];

Ahmat[39][4][1]=0.;
Ahmat[39][4][2]=0.;
Ahmat[39][4][3]=0.;
Ahmat[39][4][4]=1;


/* link 40: {F2SEG, 0, 0} */
Xlink[40][1]=Ai046[1][4];
Xlink[40][2]=Ai046[2][4];
Xlink[40][3]=Ai046[3][4];

Ahmat[40][1][1]=Ai046[1][1];
Ahmat[40][1][2]=Ai046[1][2];
Ahmat[40][1][3]=Ai046[1][3];
Ahmat[40][1][4]=Ai046[1][4];

Ahmat[40][2][1]=Ai046[2][1];
Ahmat[40][2][2]=Ai046[2][2];
Ahmat[40][2][3]=Ai046[2][3];
Ahmat[40][2][4]=Ai046[2][4];

Ahmat[40][3][1]=Ai046[3][1];
Ahmat[40][3][2]=Ai046[3][2];
Ahmat[40][3][3]=Ai046[3][3];
Ahmat[40][3][4]=Ai046[3][4];

Ahmat[40][4][1]=0.;
Ahmat[40][4][2]=0.;
Ahmat[40][4][3]=0.;
Ahmat[40][4][4]=1;


/* link 41: {F3SEG, 0, 0} */
Xlink[41][1]=Ai047[1][4];
Xlink[41][2]=Ai047[2][4];
Xlink[41][3]=Ai047[3][4];

Ahmat[41][1][1]=Ai047[1][1];
Ahmat[41][1][2]=Ai047[1][2];
Ahmat[41][1][3]=Ai047[1][3];
Ahmat[41][1][4]=Ai047[1][4];

Ahmat[41][2][1]=Ai047[2][1];
Ahmat[41][2][2]=Ai047[2][2];
Ahmat[41][2][3]=Ai047[2][3];
Ahmat[41][2][4]=Ai047[2][4];

Ahmat[41][3][1]=Ai047[3][1];
Ahmat[41][3][2]=Ai047[3][2];
Ahmat[41][3][3]=Ai047[3][3];
Ahmat[41][3][4]=Ai047[3][4];

Ahmat[41][4][1]=0.;
Ahmat[41][4][2]=0.;
Ahmat[41][4][3]=0.;
Ahmat[41][4][4]=1;


/* joint ID: 48 */
Xorigin[48][1]=Ai048[1][4];
Xorigin[48][2]=Ai048[2][4];
Xorigin[48][3]=Ai048[3][4];

Xmcog[48][1]=links[48].mcm[1]*Ai048[1][1] + links[48].mcm[2]*Ai048[1][2] + links[48].mcm[3]*Ai048[1][3] + links[48].m*Ai048[1][4];
Xmcog[48][2]=links[48].mcm[1]*Ai048[2][1] + links[48].mcm[2]*Ai048[2][2] + links[48].mcm[3]*Ai048[2][3] + links[48].m*Ai048[2][4];
Xmcog[48][3]=links[48].mcm[1]*Ai048[3][1] + links[48].mcm[2]*Ai048[3][2] + links[48].mcm[3]*Ai048[3][3] + links[48].m*Ai048[3][4];

Xaxis[48][1]=Ai048[1][3];
Xaxis[48][2]=Ai048[2][3];
Xaxis[48][3]=Ai048[3][3];

Ahmatdof[48][1][1]=Ai048[1][1];
Ahmatdof[48][1][2]=Ai048[1][2];
Ahmatdof[48][1][3]=Ai048[1][3];
Ahmatdof[48][1][4]=Ai048[1][4];

Ahmatdof[48][2][1]=Ai048[2][1];
Ahmatdof[48][2][2]=Ai048[2][2];
Ahmatdof[48][2][3]=Ai048[2][3];
Ahmatdof[48][2][4]=Ai048[2][4];

Ahmatdof[48][3][1]=Ai048[3][1];
Ahmatdof[48][3][2]=Ai048[3][2];
Ahmatdof[48][3][3]=Ai048[3][3];
Ahmatdof[48][3][4]=Ai048[3][4];

Ahmatdof[48][4][1]=0.;
Ahmatdof[48][4][2]=0.;
Ahmatdof[48][4][3]=0.;
Ahmatdof[48][4][4]=1;


/* link 42: {XMF, YMF, ZMF} */
Xlink[42][1]=Ai048[1][4];
Xlink[42][2]=Ai048[2][4];
Xlink[42][3]=Ai048[3][4];

Ahmat[42][1][1]=Ai048[1][1];
Ahmat[42][1][2]=Ai048[1][2];
Ahmat[42][1][3]=Ai048[1][3];
Ahmat[42][1][4]=Ai048[1][4];

Ahmat[42][2][1]=Ai048[2][1];
Ahmat[42][2][2]=Ai048[2][2];
Ahmat[42][2][3]=Ai048[2][3];
Ahmat[42][2][4]=Ai048[2][4];

Ahmat[42][3][1]=Ai048[3][1];
Ahmat[42][3][2]=Ai048[3][2];
Ahmat[42][3][3]=Ai048[3][3];
Ahmat[42][3][4]=Ai048[3][4];

Ahmat[42][4][1]=0.;
Ahmat[42][4][2]=0.;
Ahmat[42][4][3]=0.;
Ahmat[42][4][4]=1;


/* link 43: {F1SEG, 0, 0} */
Xlink[43][1]=Ai049[1][4];
Xlink[43][2]=Ai049[2][4];
Xlink[43][3]=Ai049[3][4];

Ahmat[43][1][1]=Ai049[1][1];
Ahmat[43][1][2]=Ai049[1][2];
Ahmat[43][1][3]=Ai049[1][3];
Ahmat[43][1][4]=Ai049[1][4];

Ahmat[43][2][1]=Ai049[2][1];
Ahmat[43][2][2]=Ai049[2][2];
Ahmat[43][2][3]=Ai049[2][3];
Ahmat[43][2][4]=Ai049[2][4];

Ahmat[43][3][1]=Ai049[3][1];
Ahmat[43][3][2]=Ai049[3][2];
Ahmat[43][3][3]=Ai049[3][3];
Ahmat[43][3][4]=Ai049[3][4];

Ahmat[43][4][1]=0.;
Ahmat[43][4][2]=0.;
Ahmat[43][4][3]=0.;
Ahmat[43][4][4]=1;


/* link 44: {F2SEG, 0, 0} */
Xlink[44][1]=Ai050[1][4];
Xlink[44][2]=Ai050[2][4];
Xlink[44][3]=Ai050[3][4];

Ahmat[44][1][1]=Ai050[1][1];
Ahmat[44][1][2]=Ai050[1][2];
Ahmat[44][1][3]=Ai050[1][3];
Ahmat[44][1][4]=Ai050[1][4];

Ahmat[44][2][1]=Ai050[2][1];
Ahmat[44][2][2]=Ai050[2][2];
Ahmat[44][2][3]=Ai050[2][3];
Ahmat[44][2][4]=Ai050[2][4];

Ahmat[44][3][1]=Ai050[3][1];
Ahmat[44][3][2]=Ai050[3][2];
Ahmat[44][3][3]=Ai050[3][3];
Ahmat[44][3][4]=Ai050[3][4];

Ahmat[44][4][1]=0.;
Ahmat[44][4][2]=0.;
Ahmat[44][4][3]=0.;
Ahmat[44][4][4]=1;


/* link 45: {F3SEG, 0, 0} */
Xlink[45][1]=Ai051[1][4];
Xlink[45][2]=Ai051[2][4];
Xlink[45][3]=Ai051[3][4];

Ahmat[45][1][1]=Ai051[1][1];
Ahmat[45][1][2]=Ai051[1][2];
Ahmat[45][1][3]=Ai051[1][3];
Ahmat[45][1][4]=Ai051[1][4];

Ahmat[45][2][1]=Ai051[2][1];
Ahmat[45][2][2]=Ai051[2][2];
Ahmat[45][2][3]=Ai051[2][3];
Ahmat[45][2][4]=Ai051[2][4];

Ahmat[45][3][1]=Ai051[3][1];
Ahmat[45][3][2]=Ai051[3][2];
Ahmat[45][3][3]=Ai051[3][3];
Ahmat[45][3][4]=Ai051[3][4];

Ahmat[45][4][1]=0.;
Ahmat[45][4][2]=0.;
Ahmat[45][4][3]=0.;
Ahmat[45][4][4]=1;


/* joint ID: 49 */
Xorigin[49][1]=Ai052[1][4];
Xorigin[49][2]=Ai052[2][4];
Xorigin[49][3]=Ai052[3][4];

Xmcog[49][1]=links[49].mcm[1]*Ai052[1][1] + links[49].mcm[2]*Ai052[1][2] + links[49].mcm[3]*Ai052[1][3] + links[49].m*Ai052[1][4];
Xmcog[49][2]=links[49].mcm[1]*Ai052[2][1] + links[49].mcm[2]*Ai052[2][2] + links[49].mcm[3]*Ai052[2][3] + links[49].m*Ai052[2][4];
Xmcog[49][3]=links[49].mcm[1]*Ai052[3][1] + links[49].mcm[2]*Ai052[3][2] + links[49].mcm[3]*Ai052[3][3] + links[49].m*Ai052[3][4];

Xaxis[49][1]=Ai052[1][3];
Xaxis[49][2]=Ai052[2][3];
Xaxis[49][3]=Ai052[3][3];

Ahmatdof[49][1][1]=Ai052[1][1];
Ahmatdof[49][1][2]=Ai052[1][2];
Ahmatdof[49][1][3]=Ai052[1][3];
Ahmatdof[49][1][4]=Ai052[1][4];

Ahmatdof[49][2][1]=Ai052[2][1];
Ahmatdof[49][2][2]=Ai052[2][2];
Ahmatdof[49][2][3]=Ai052[2][3];
Ahmatdof[49][2][4]=Ai052[2][4];

Ahmatdof[49][3][1]=Ai052[3][1];
Ahmatdof[49][3][2]=Ai052[3][2];
Ahmatdof[49][3][3]=Ai052[3][3];
Ahmatdof[49][3][4]=Ai052[3][4];

Ahmatdof[49][4][1]=0.;
Ahmatdof[49][4][2]=0.;
Ahmatdof[49][4][3]=0.;
Ahmatdof[49][4][4]=1;


/* link 46: {XRF, YRF, ZRF} */
Xlink[46][1]=Ai052[1][4];
Xlink[46][2]=Ai052[2][4];
Xlink[46][3]=Ai052[3][4];

Ahmat[46][1][1]=Ai052[1][1];
Ahmat[46][1][2]=Ai052[1][2];
Ahmat[46][1][3]=Ai052[1][3];
Ahmat[46][1][4]=Ai052[1][4];

Ahmat[46][2][1]=Ai052[2][1];
Ahmat[46][2][2]=Ai052[2][2];
Ahmat[46][2][3]=Ai052[2][3];
Ahmat[46][2][4]=Ai052[2][4];

Ahmat[46][3][1]=Ai052[3][1];
Ahmat[46][3][2]=Ai052[3][2];
Ahmat[46][3][3]=Ai052[3][3];
Ahmat[46][3][4]=Ai052[3][4];

Ahmat[46][4][1]=0.;
Ahmat[46][4][2]=0.;
Ahmat[46][4][3]=0.;
Ahmat[46][4][4]=1;


/* link 47: {F1SEG, 0, 0} */
Xlink[47][1]=Ai053[1][4];
Xlink[47][2]=Ai053[2][4];
Xlink[47][3]=Ai053[3][4];

Ahmat[47][1][1]=Ai053[1][1];
Ahmat[47][1][2]=Ai053[1][2];
Ahmat[47][1][3]=Ai053[1][3];
Ahmat[47][1][4]=Ai053[1][4];

Ahmat[47][2][1]=Ai053[2][1];
Ahmat[47][2][2]=Ai053[2][2];
Ahmat[47][2][3]=Ai053[2][3];
Ahmat[47][2][4]=Ai053[2][4];

Ahmat[47][3][1]=Ai053[3][1];
Ahmat[47][3][2]=Ai053[3][2];
Ahmat[47][3][3]=Ai053[3][3];
Ahmat[47][3][4]=Ai053[3][4];

Ahmat[47][4][1]=0.;
Ahmat[47][4][2]=0.;
Ahmat[47][4][3]=0.;
Ahmat[47][4][4]=1;


/* link 48: {F2SEG, 0, 0} */
Xlink[48][1]=Ai054[1][4];
Xlink[48][2]=Ai054[2][4];
Xlink[48][3]=Ai054[3][4];

Ahmat[48][1][1]=Ai054[1][1];
Ahmat[48][1][2]=Ai054[1][2];
Ahmat[48][1][3]=Ai054[1][3];
Ahmat[48][1][4]=Ai054[1][4];

Ahmat[48][2][1]=Ai054[2][1];
Ahmat[48][2][2]=Ai054[2][2];
Ahmat[48][2][3]=Ai054[2][3];
Ahmat[48][2][4]=Ai054[2][4];

Ahmat[48][3][1]=Ai054[3][1];
Ahmat[48][3][2]=Ai054[3][2];
Ahmat[48][3][3]=Ai054[3][3];
Ahmat[48][3][4]=Ai054[3][4];

Ahmat[48][4][1]=0.;
Ahmat[48][4][2]=0.;
Ahmat[48][4][3]=0.;
Ahmat[48][4][4]=1;


/* link 49: {F3SEG, 0, 0} */
Xlink[49][1]=Ai055[1][4];
Xlink[49][2]=Ai055[2][4];
Xlink[49][3]=Ai055[3][4];

Ahmat[49][1][1]=Ai055[1][1];
Ahmat[49][1][2]=Ai055[1][2];
Ahmat[49][1][3]=Ai055[1][3];
Ahmat[49][1][4]=Ai055[1][4];

Ahmat[49][2][1]=Ai055[2][1];
Ahmat[49][2][2]=Ai055[2][2];
Ahmat[49][2][3]=Ai055[2][3];
Ahmat[49][2][4]=Ai055[2][4];

Ahmat[49][3][1]=Ai055[3][1];
Ahmat[49][3][2]=Ai055[3][2];
Ahmat[49][3][3]=Ai055[3][3];
Ahmat[49][3][4]=Ai055[3][4];

Ahmat[49][4][1]=0.;
Ahmat[49][4][2]=0.;
Ahmat[49][4][3]=0.;
Ahmat[49][4][4]=1;


/* joint ID: 50 */
Xorigin[50][1]=Ai056[1][4];
Xorigin[50][2]=Ai056[2][4];
Xorigin[50][3]=Ai056[3][4];

Xmcog[50][1]=links[50].mcm[1]*Ai056[1][1] + links[50].mcm[2]*Ai056[1][2] + links[50].mcm[3]*Ai056[1][3] + links[50].m*Ai056[1][4];
Xmcog[50][2]=links[50].mcm[1]*Ai056[2][1] + links[50].mcm[2]*Ai056[2][2] + links[50].mcm[3]*Ai056[2][3] + links[50].m*Ai056[2][4];
Xmcog[50][3]=links[50].mcm[1]*Ai056[3][1] + links[50].mcm[2]*Ai056[3][2] + links[50].mcm[3]*Ai056[3][3] + links[50].m*Ai056[3][4];

Xaxis[50][1]=Ai056[1][3];
Xaxis[50][2]=Ai056[2][3];
Xaxis[50][3]=Ai056[3][3];

Ahmatdof[50][1][1]=Ai056[1][1];
Ahmatdof[50][1][2]=Ai056[1][2];
Ahmatdof[50][1][3]=Ai056[1][3];
Ahmatdof[50][1][4]=Ai056[1][4];

Ahmatdof[50][2][1]=Ai056[2][1];
Ahmatdof[50][2][2]=Ai056[2][2];
Ahmatdof[50][2][3]=Ai056[2][3];
Ahmatdof[50][2][4]=Ai056[2][4];

Ahmatdof[50][3][1]=Ai056[3][1];
Ahmatdof[50][3][2]=Ai056[3][2];
Ahmatdof[50][3][3]=Ai056[3][3];
Ahmatdof[50][3][4]=Ai056[3][4];

Ahmatdof[50][4][1]=0.;
Ahmatdof[50][4][2]=0.;
Ahmatdof[50][4][3]=0.;
Ahmatdof[50][4][4]=1;


/* link 50: {XLF, YLF, ZLF} */
Xlink[50][1]=Ai056[1][4];
Xlink[50][2]=Ai056[2][4];
Xlink[50][3]=Ai056[3][4];

Ahmat[50][1][1]=Ai056[1][1];
Ahmat[50][1][2]=Ai056[1][2];
Ahmat[50][1][3]=Ai056[1][3];
Ahmat[50][1][4]=Ai056[1][4];

Ahmat[50][2][1]=Ai056[2][1];
Ahmat[50][2][2]=Ai056[2][2];
Ahmat[50][2][3]=Ai056[2][3];
Ahmat[50][2][4]=Ai056[2][4];

Ahmat[50][3][1]=Ai056[3][1];
Ahmat[50][3][2]=Ai056[3][2];
Ahmat[50][3][3]=Ai056[3][3];
Ahmat[50][3][4]=Ai056[3][4];

Ahmat[50][4][1]=0.;
Ahmat[50][4][2]=0.;
Ahmat[50][4][3]=0.;
Ahmat[50][4][4]=1;


/* link 51: {F1SEG, 0, 0} */
Xlink[51][1]=Ai057[1][4];
Xlink[51][2]=Ai057[2][4];
Xlink[51][3]=Ai057[3][4];

Ahmat[51][1][1]=Ai057[1][1];
Ahmat[51][1][2]=Ai057[1][2];
Ahmat[51][1][3]=Ai057[1][3];
Ahmat[51][1][4]=Ai057[1][4];

Ahmat[51][2][1]=Ai057[2][1];
Ahmat[51][2][2]=Ai057[2][2];
Ahmat[51][2][3]=Ai057[2][3];
Ahmat[51][2][4]=Ai057[2][4];

Ahmat[51][3][1]=Ai057[3][1];
Ahmat[51][3][2]=Ai057[3][2];
Ahmat[51][3][3]=Ai057[3][3];
Ahmat[51][3][4]=Ai057[3][4];

Ahmat[51][4][1]=0.;
Ahmat[51][4][2]=0.;
Ahmat[51][4][3]=0.;
Ahmat[51][4][4]=1;


/* link 52: {F2SEG, 0, 0} */
Xlink[52][1]=Ai058[1][4];
Xlink[52][2]=Ai058[2][4];
Xlink[52][3]=Ai058[3][4];

Ahmat[52][1][1]=Ai058[1][1];
Ahmat[52][1][2]=Ai058[1][2];
Ahmat[52][1][3]=Ai058[1][3];
Ahmat[52][1][4]=Ai058[1][4];

Ahmat[52][2][1]=Ai058[2][1];
Ahmat[52][2][2]=Ai058[2][2];
Ahmat[52][2][3]=Ai058[2][3];
Ahmat[52][2][4]=Ai058[2][4];

Ahmat[52][3][1]=Ai058[3][1];
Ahmat[52][3][2]=Ai058[3][2];
Ahmat[52][3][3]=Ai058[3][3];
Ahmat[52][3][4]=Ai058[3][4];

Ahmat[52][4][1]=0.;
Ahmat[52][4][2]=0.;
Ahmat[52][4][3]=0.;
Ahmat[52][4][4]=1;


/* link 53: {F3SEG, 0, 0} */
Xlink[53][1]=Ai059[1][4];
Xlink[53][2]=Ai059[2][4];
Xlink[53][3]=Ai059[3][4];

Ahmat[53][1][1]=Ai059[1][1];
Ahmat[53][1][2]=Ai059[1][2];
Ahmat[53][1][3]=Ai059[1][3];
Ahmat[53][1][4]=Ai059[1][4];

Ahmat[53][2][1]=Ai059[2][1];
Ahmat[53][2][2]=Ai059[2][2];
Ahmat[53][2][3]=Ai059[2][3];
Ahmat[53][2][4]=Ai059[2][4];

Ahmat[53][3][1]=Ai059[3][1];
Ahmat[53][3][2]=Ai059[3][2];
Ahmat[53][3][3]=Ai059[3][3];
Ahmat[53][3][4]=Ai059[3][4];

Ahmat[53][4][1]=0.;
Ahmat[53][4][2]=0.;
Ahmat[53][4][3]=0.;
Ahmat[53][4][4]=1;


/* joint ID: 32 */
Xorigin[32][1]=Ai060[1][4];
Xorigin[32][2]=Ai060[2][4];
Xorigin[32][3]=Ai060[3][4];

Xmcog[32][1]=links[32].mcm[1]*Ai060[1][1] + links[32].mcm[2]*Ai060[1][2] + links[32].mcm[3]*Ai060[1][3] + links[32].m*Ai060[1][4];
Xmcog[32][2]=links[32].mcm[1]*Ai060[2][1] + links[32].mcm[2]*Ai060[2][2] + links[32].mcm[3]*Ai060[2][3] + links[32].m*Ai060[2][4];
Xmcog[32][3]=links[32].mcm[1]*Ai060[3][1] + links[32].mcm[2]*Ai060[3][2] + links[32].mcm[3]*Ai060[3][3] + links[32].m*Ai060[3][4];

Xaxis[32][1]=Ai060[1][3];
Xaxis[32][2]=Ai060[2][3];
Xaxis[32][3]=Ai060[3][3];

Ahmatdof[32][1][1]=Ai060[1][1];
Ahmatdof[32][1][2]=Ai060[1][2];
Ahmatdof[32][1][3]=Ai060[1][3];
Ahmatdof[32][1][4]=Ai060[1][4];

Ahmatdof[32][2][1]=Ai060[2][1];
Ahmatdof[32][2][2]=Ai060[2][2];
Ahmatdof[32][2][3]=Ai060[2][3];
Ahmatdof[32][2][4]=Ai060[2][4];

Ahmatdof[32][3][1]=Ai060[3][1];
Ahmatdof[32][3][2]=Ai060[3][2];
Ahmatdof[32][3][3]=Ai060[3][3];
Ahmatdof[32][3][4]=Ai060[3][4];

Ahmatdof[32][4][1]=0.;
Ahmatdof[32][4][2]=0.;
Ahmatdof[32][4][3]=0.;
Ahmatdof[32][4][4]=1;


/* link 54: {-THORAX2NECK, 0, 0} */
Xlink[54][1]=Ai060[1][4];
Xlink[54][2]=Ai060[2][4];
Xlink[54][3]=Ai060[3][4];

Ahmat[54][1][1]=Ai060[1][1];
Ahmat[54][1][2]=Ai060[1][2];
Ahmat[54][1][3]=Ai060[1][3];
Ahmat[54][1][4]=Ai060[1][4];

Ahmat[54][2][1]=Ai060[2][1];
Ahmat[54][2][2]=Ai060[2][2];
Ahmat[54][2][3]=Ai060[2][3];
Ahmat[54][2][4]=Ai060[2][4];

Ahmat[54][3][1]=Ai060[3][1];
Ahmat[54][3][2]=Ai060[3][2];
Ahmat[54][3][3]=Ai060[3][3];
Ahmat[54][3][4]=Ai060[3][4];

Ahmat[54][4][1]=0.;
Ahmat[54][4][2]=0.;
Ahmat[54][4][3]=0.;
Ahmat[54][4][4]=1;


/* joint ID: 33 */
Xorigin[33][1]=Ai061[1][4];
Xorigin[33][2]=Ai061[2][4];
Xorigin[33][3]=Ai061[3][4];

Xmcog[33][1]=links[33].mcm[1]*Ai061[1][1] + links[33].mcm[2]*Ai061[1][2] + links[33].mcm[3]*Ai061[1][3] + links[33].m*Ai061[1][4];
Xmcog[33][2]=links[33].mcm[1]*Ai061[2][1] + links[33].mcm[2]*Ai061[2][2] + links[33].mcm[3]*Ai061[2][3] + links[33].m*Ai061[2][4];
Xmcog[33][3]=links[33].mcm[1]*Ai061[3][1] + links[33].mcm[2]*Ai061[3][2] + links[33].mcm[3]*Ai061[3][3] + links[33].m*Ai061[3][4];

Xaxis[33][1]=Ai061[1][3];
Xaxis[33][2]=Ai061[2][3];
Xaxis[33][3]=Ai061[3][3];

Ahmatdof[33][1][1]=Ai061[1][1];
Ahmatdof[33][1][2]=Ai061[1][2];
Ahmatdof[33][1][3]=Ai061[1][3];
Ahmatdof[33][1][4]=Ai061[1][4];

Ahmatdof[33][2][1]=Ai061[2][1];
Ahmatdof[33][2][2]=Ai061[2][2];
Ahmatdof[33][2][3]=Ai061[2][3];
Ahmatdof[33][2][4]=Ai061[2][4];

Ahmatdof[33][3][1]=Ai061[3][1];
Ahmatdof[33][3][2]=Ai061[3][2];
Ahmatdof[33][3][3]=Ai061[3][3];
Ahmatdof[33][3][4]=Ai061[3][4];

Ahmatdof[33][4][1]=0.;
Ahmatdof[33][4][2]=0.;
Ahmatdof[33][4][3]=0.;
Ahmatdof[33][4][4]=1;


/* link 55: {0, -CERVICAL, 0} */
Xlink[55][1]=Ai061[1][4];
Xlink[55][2]=Ai061[2][4];
Xlink[55][3]=Ai061[3][4];

Ahmat[55][1][1]=Ai062[1][1];
Ahmat[55][1][2]=Ai062[1][2];
Ahmat[55][1][3]=Ai062[1][3];
Ahmat[55][1][4]=Ai062[1][4];

Ahmat[55][2][1]=Ai062[2][1];
Ahmat[55][2][2]=Ai062[2][2];
Ahmat[55][2][3]=Ai062[2][3];
Ahmat[55][2][4]=Ai062[2][4];

Ahmat[55][3][1]=Ai062[3][1];
Ahmat[55][3][2]=Ai062[3][2];
Ahmat[55][3][3]=Ai062[3][3];
Ahmat[55][3][4]=Ai062[3][4];

Ahmat[55][4][1]=0.;
Ahmat[55][4][2]=0.;
Ahmat[55][4][3]=0.;
Ahmat[55][4][4]=1;


/* joint ID: 34 */
Xorigin[34][1]=Ai062[1][4];
Xorigin[34][2]=Ai062[2][4];
Xorigin[34][3]=Ai062[3][4];

Xmcog[34][1]=links[34].mcm[1]*Ai062[1][1] + links[34].mcm[2]*Ai062[1][2] + links[34].mcm[3]*Ai062[1][3] + links[34].m*Ai062[1][4];
Xmcog[34][2]=links[34].mcm[1]*Ai062[2][1] + links[34].mcm[2]*Ai062[2][2] + links[34].mcm[3]*Ai062[2][3] + links[34].m*Ai062[2][4];
Xmcog[34][3]=links[34].mcm[1]*Ai062[3][1] + links[34].mcm[2]*Ai062[3][2] + links[34].mcm[3]*Ai062[3][3] + links[34].m*Ai062[3][4];

Xaxis[34][1]=Ai062[1][3];
Xaxis[34][2]=Ai062[2][3];
Xaxis[34][3]=Ai062[3][3];

Ahmatdof[34][1][1]=Ai062[1][1];
Ahmatdof[34][1][2]=Ai062[1][2];
Ahmatdof[34][1][3]=Ai062[1][3];
Ahmatdof[34][1][4]=Ai062[1][4];

Ahmatdof[34][2][1]=Ai062[2][1];
Ahmatdof[34][2][2]=Ai062[2][2];
Ahmatdof[34][2][3]=Ai062[2][3];
Ahmatdof[34][2][4]=Ai062[2][4];

Ahmatdof[34][3][1]=Ai062[3][1];
Ahmatdof[34][3][2]=Ai062[3][2];
Ahmatdof[34][3][3]=Ai062[3][3];
Ahmatdof[34][3][4]=Ai062[3][4];

Ahmatdof[34][4][1]=0.;
Ahmatdof[34][4][2]=0.;
Ahmatdof[34][4][3]=0.;
Ahmatdof[34][4][4]=1;


/* joint ID: 35 */
Xorigin[35][1]=Ai063[1][4];
Xorigin[35][2]=Ai063[2][4];
Xorigin[35][3]=Ai063[3][4];

Xmcog[35][1]=links[35].mcm[1]*Ai063[1][1] + links[35].mcm[2]*Ai063[1][2] + links[35].mcm[3]*Ai063[1][3] + links[35].m*Ai063[1][4];
Xmcog[35][2]=links[35].mcm[1]*Ai063[2][1] + links[35].mcm[2]*Ai063[2][2] + links[35].mcm[3]*Ai063[2][3] + links[35].m*Ai063[2][4];
Xmcog[35][3]=links[35].mcm[1]*Ai063[3][1] + links[35].mcm[2]*Ai063[3][2] + links[35].mcm[3]*Ai063[3][3] + links[35].m*Ai063[3][4];

Xaxis[35][1]=Ai063[1][3];
Xaxis[35][2]=Ai063[2][3];
Xaxis[35][3]=Ai063[3][3];

Ahmatdof[35][1][1]=Ai063[1][1];
Ahmatdof[35][1][2]=Ai063[1][2];
Ahmatdof[35][1][3]=Ai063[1][3];
Ahmatdof[35][1][4]=Ai063[1][4];

Ahmatdof[35][2][1]=Ai063[2][1];
Ahmatdof[35][2][2]=Ai063[2][2];
Ahmatdof[35][2][3]=Ai063[2][3];
Ahmatdof[35][2][4]=Ai063[2][4];

Ahmatdof[35][3][1]=Ai063[3][1];
Ahmatdof[35][3][2]=Ai063[3][2];
Ahmatdof[35][3][3]=Ai063[3][3];
Ahmatdof[35][3][4]=Ai063[3][4];

Ahmatdof[35][4][1]=0.;
Ahmatdof[35][4][2]=0.;
Ahmatdof[35][4][3]=0.;
Ahmatdof[35][4][4]=1;


/* link 56: {EYEXOFF, -EYEYOFF, -HEAD} */
Xlink[56][1]=Ai063[1][4];
Xlink[56][2]=Ai063[2][4];
Xlink[56][3]=Ai063[3][4];

Ahmat[56][1][1]=Ai064[1][1];
Ahmat[56][1][2]=Ai064[1][2];
Ahmat[56][1][3]=Ai064[1][3];
Ahmat[56][1][4]=Ai064[1][4];

Ahmat[56][2][1]=Ai064[2][1];
Ahmat[56][2][2]=Ai064[2][2];
Ahmat[56][2][3]=Ai064[2][3];
Ahmat[56][2][4]=Ai064[2][4];

Ahmat[56][3][1]=Ai064[3][1];
Ahmat[56][3][2]=Ai064[3][2];
Ahmat[56][3][3]=Ai064[3][3];
Ahmat[56][3][4]=Ai064[3][4];

Ahmat[56][4][1]=0.;
Ahmat[56][4][2]=0.;
Ahmat[56][4][3]=0.;
Ahmat[56][4][4]=1;


/* joint ID: 36 */
Xorigin[36][1]=Ai064[1][4];
Xorigin[36][2]=Ai064[2][4];
Xorigin[36][3]=Ai064[3][4];

Xmcog[36][1]=links[36].mcm[1]*Ai064[1][1] + links[36].mcm[2]*Ai064[1][2] + links[36].mcm[3]*Ai064[1][3] + links[36].m*Ai064[1][4];
Xmcog[36][2]=links[36].mcm[1]*Ai064[2][1] + links[36].mcm[2]*Ai064[2][2] + links[36].mcm[3]*Ai064[2][3] + links[36].m*Ai064[2][4];
Xmcog[36][3]=links[36].mcm[1]*Ai064[3][1] + links[36].mcm[2]*Ai064[3][2] + links[36].mcm[3]*Ai064[3][3] + links[36].m*Ai064[3][4];

Xaxis[36][1]=Ai064[1][3];
Xaxis[36][2]=Ai064[2][3];
Xaxis[36][3]=Ai064[3][3];

Ahmatdof[36][1][1]=Ai064[1][1];
Ahmatdof[36][1][2]=Ai064[1][2];
Ahmatdof[36][1][3]=Ai064[1][3];
Ahmatdof[36][1][4]=Ai064[1][4];

Ahmatdof[36][2][1]=Ai064[2][1];
Ahmatdof[36][2][2]=Ai064[2][2];
Ahmatdof[36][2][3]=Ai064[2][3];
Ahmatdof[36][2][4]=Ai064[2][4];

Ahmatdof[36][3][1]=Ai064[3][1];
Ahmatdof[36][3][2]=Ai064[3][2];
Ahmatdof[36][3][3]=Ai064[3][3];
Ahmatdof[36][3][4]=Ai064[3][4];

Ahmatdof[36][4][1]=0.;
Ahmatdof[36][4][2]=0.;
Ahmatdof[36][4][3]=0.;
Ahmatdof[36][4][4]=1;


/* link 57: {0, -EYE, 0} */
Xlink[57][1]=Ai065[1][4];
Xlink[57][2]=Ai065[2][4];
Xlink[57][3]=Ai065[3][4];

Ahmat[57][1][1]=Ai065[1][1];
Ahmat[57][1][2]=Ai065[1][2];
Ahmat[57][1][3]=Ai065[1][3];
Ahmat[57][1][4]=Ai065[1][4];

Ahmat[57][2][1]=Ai065[2][1];
Ahmat[57][2][2]=Ai065[2][2];
Ahmat[57][2][3]=Ai065[2][3];
Ahmat[57][2][4]=Ai065[2][4];

Ahmat[57][3][1]=Ai065[3][1];
Ahmat[57][3][2]=Ai065[3][2];
Ahmat[57][3][3]=Ai065[3][3];
Ahmat[57][3][4]=Ai065[3][4];

Ahmat[57][4][1]=0.;
Ahmat[57][4][2]=0.;
Ahmat[57][4][3]=0.;
Ahmat[57][4][4]=1;


/* joint ID: 37 */
Xorigin[37][1]=Ai066[1][4];
Xorigin[37][2]=Ai066[2][4];
Xorigin[37][3]=Ai066[3][4];

Xmcog[37][1]=links[37].mcm[1]*Ai066[1][1] + links[37].mcm[2]*Ai066[1][2] + links[37].mcm[3]*Ai066[1][3] + links[37].m*Ai066[1][4];
Xmcog[37][2]=links[37].mcm[1]*Ai066[2][1] + links[37].mcm[2]*Ai066[2][2] + links[37].mcm[3]*Ai066[2][3] + links[37].m*Ai066[2][4];
Xmcog[37][3]=links[37].mcm[1]*Ai066[3][1] + links[37].mcm[2]*Ai066[3][2] + links[37].mcm[3]*Ai066[3][3] + links[37].m*Ai066[3][4];

Xaxis[37][1]=Ai066[1][3];
Xaxis[37][2]=Ai066[2][3];
Xaxis[37][3]=Ai066[3][3];

Ahmatdof[37][1][1]=Ai066[1][1];
Ahmatdof[37][1][2]=Ai066[1][2];
Ahmatdof[37][1][3]=Ai066[1][3];
Ahmatdof[37][1][4]=Ai066[1][4];

Ahmatdof[37][2][1]=Ai066[2][1];
Ahmatdof[37][2][2]=Ai066[2][2];
Ahmatdof[37][2][3]=Ai066[2][3];
Ahmatdof[37][2][4]=Ai066[2][4];

Ahmatdof[37][3][1]=Ai066[3][1];
Ahmatdof[37][3][2]=Ai066[3][2];
Ahmatdof[37][3][3]=Ai066[3][3];
Ahmatdof[37][3][4]=Ai066[3][4];

Ahmatdof[37][4][1]=0.;
Ahmatdof[37][4][2]=0.;
Ahmatdof[37][4][3]=0.;
Ahmatdof[37][4][4]=1;


/* link 58: {-EYEXOFF, -EYEYOFF, -HEAD} */
Xlink[58][1]=Ai066[1][4];
Xlink[58][2]=Ai066[2][4];
Xlink[58][3]=Ai066[3][4];

Ahmat[58][1][1]=Ai067[1][1];
Ahmat[58][1][2]=Ai067[1][2];
Ahmat[58][1][3]=Ai067[1][3];
Ahmat[58][1][4]=Ai067[1][4];

Ahmat[58][2][1]=Ai067[2][1];
Ahmat[58][2][2]=Ai067[2][2];
Ahmat[58][2][3]=Ai067[2][3];
Ahmat[58][2][4]=Ai067[2][4];

Ahmat[58][3][1]=Ai067[3][1];
Ahmat[58][3][2]=Ai067[3][2];
Ahmat[58][3][3]=Ai067[3][3];
Ahmat[58][3][4]=Ai067[3][4];

Ahmat[58][4][1]=0.;
Ahmat[58][4][2]=0.;
Ahmat[58][4][3]=0.;
Ahmat[58][4][4]=1;


/* joint ID: 38 */
Xorigin[38][1]=Ai067[1][4];
Xorigin[38][2]=Ai067[2][4];
Xorigin[38][3]=Ai067[3][4];

Xmcog[38][1]=links[38].mcm[1]*Ai067[1][1] + links[38].mcm[2]*Ai067[1][2] + links[38].mcm[3]*Ai067[1][3] + links[38].m*Ai067[1][4];
Xmcog[38][2]=links[38].mcm[1]*Ai067[2][1] + links[38].mcm[2]*Ai067[2][2] + links[38].mcm[3]*Ai067[2][3] + links[38].m*Ai067[2][4];
Xmcog[38][3]=links[38].mcm[1]*Ai067[3][1] + links[38].mcm[2]*Ai067[3][2] + links[38].mcm[3]*Ai067[3][3] + links[38].m*Ai067[3][4];

Xaxis[38][1]=Ai067[1][3];
Xaxis[38][2]=Ai067[2][3];
Xaxis[38][3]=Ai067[3][3];

Ahmatdof[38][1][1]=Ai067[1][1];
Ahmatdof[38][1][2]=Ai067[1][2];
Ahmatdof[38][1][3]=Ai067[1][3];
Ahmatdof[38][1][4]=Ai067[1][4];

Ahmatdof[38][2][1]=Ai067[2][1];
Ahmatdof[38][2][2]=Ai067[2][2];
Ahmatdof[38][2][3]=Ai067[2][3];
Ahmatdof[38][2][4]=Ai067[2][4];

Ahmatdof[38][3][1]=Ai067[3][1];
Ahmatdof[38][3][2]=Ai067[3][2];
Ahmatdof[38][3][3]=Ai067[3][3];
Ahmatdof[38][3][4]=Ai067[3][4];

Ahmatdof[38][4][1]=0.;
Ahmatdof[38][4][2]=0.;
Ahmatdof[38][4][3]=0.;
Ahmatdof[38][4][4]=1;


/* link 59: {0, -EYE, 0} */
Xlink[59][1]=Ai068[1][4];
Xlink[59][2]=Ai068[2][4];
Xlink[59][3]=Ai068[3][4];

Ahmat[59][1][1]=Ai068[1][1];
Ahmat[59][1][2]=Ai068[1][2];
Ahmat[59][1][3]=Ai068[1][3];
Ahmat[59][1][4]=Ai068[1][4];

Ahmat[59][2][1]=Ai068[2][1];
Ahmat[59][2][2]=Ai068[2][2];
Ahmat[59][2][3]=Ai068[2][3];
Ahmat[59][2][4]=Ai068[2][4];

Ahmat[59][3][1]=Ai068[3][1];
Ahmat[59][3][2]=Ai068[3][2];
Ahmat[59][3][3]=Ai068[3][3];
Ahmat[59][3][4]=Ai068[3][4];

Ahmat[59][4][1]=0.;
Ahmat[59][4][2]=0.;
Ahmat[59][4][3]=0.;
Ahmat[59][4][4]=1;


/* link 60: {0, 0, -TOPofHEAD} */
Xlink[60][1]=Ai069[1][4];
Xlink[60][2]=Ai069[2][4];
Xlink[60][3]=Ai069[3][4];

Ahmat[60][1][1]=Ai069[1][1];
Ahmat[60][1][2]=Ai069[1][2];
Ahmat[60][1][3]=Ai069[1][3];
Ahmat[60][1][4]=Ai069[1][4];

Ahmat[60][2][1]=Ai069[2][1];
Ahmat[60][2][2]=Ai069[2][2];
Ahmat[60][2][3]=Ai069[2][3];
Ahmat[60][2][4]=Ai069[2][4];

Ahmat[60][3][1]=Ai069[3][1];
Ahmat[60][3][2]=Ai069[3][2];
Ahmat[60][3][3]=Ai069[3][3];
Ahmat[60][3][4]=Ai069[3][4];

Ahmat[60][4][1]=0.;
Ahmat[60][4][2]=0.;
Ahmat[60][4][3]=0.;
Ahmat[60][4][4]=1;


/* joint ID: 23 */
Xorigin[23][1]=Ai070[1][4];
Xorigin[23][2]=Ai070[2][4];
Xorigin[23][3]=Ai070[3][4];

Xmcog[23][1]=links[23].mcm[1]*Ai070[1][1] + links[23].mcm[2]*Ai070[1][2] + links[23].mcm[3]*Ai070[1][3] + links[23].m*Ai070[1][4];
Xmcog[23][2]=links[23].mcm[1]*Ai070[2][1] + links[23].mcm[2]*Ai070[2][2] + links[23].mcm[3]*Ai070[2][3] + links[23].m*Ai070[2][4];
Xmcog[23][3]=links[23].mcm[1]*Ai070[3][1] + links[23].mcm[2]*Ai070[3][2] + links[23].mcm[3]*Ai070[3][3] + links[23].m*Ai070[3][4];

Xaxis[23][1]=Ai070[1][3];
Xaxis[23][2]=Ai070[2][3];
Xaxis[23][3]=Ai070[3][3];

Ahmatdof[23][1][1]=Ai070[1][1];
Ahmatdof[23][1][2]=Ai070[1][2];
Ahmatdof[23][1][3]=Ai070[1][3];
Ahmatdof[23][1][4]=Ai070[1][4];

Ahmatdof[23][2][1]=Ai070[2][1];
Ahmatdof[23][2][2]=Ai070[2][2];
Ahmatdof[23][2][3]=Ai070[2][3];
Ahmatdof[23][2][4]=Ai070[2][4];

Ahmatdof[23][3][1]=Ai070[3][1];
Ahmatdof[23][3][2]=Ai070[3][2];
Ahmatdof[23][3][3]=Ai070[3][3];
Ahmatdof[23][3][4]=Ai070[3][4];

Ahmatdof[23][4][1]=0.;
Ahmatdof[23][4][2]=0.;
Ahmatdof[23][4][3]=0.;
Ahmatdof[23][4][4]=1;


/* link 61: {XHIP, 0, 0} */
Xlink[61][1]=Ai070[1][4];
Xlink[61][2]=Ai070[2][4];
Xlink[61][3]=Ai070[3][4];

Ahmat[61][1][1]=Ai071[1][1];
Ahmat[61][1][2]=Ai071[1][2];
Ahmat[61][1][3]=Ai071[1][3];
Ahmat[61][1][4]=Ai071[1][4];

Ahmat[61][2][1]=Ai071[2][1];
Ahmat[61][2][2]=Ai071[2][2];
Ahmat[61][2][3]=Ai071[2][3];
Ahmat[61][2][4]=Ai071[2][4];

Ahmat[61][3][1]=Ai071[3][1];
Ahmat[61][3][2]=Ai071[3][2];
Ahmat[61][3][3]=Ai071[3][3];
Ahmat[61][3][4]=Ai071[3][4];

Ahmat[61][4][1]=0.;
Ahmat[61][4][2]=0.;
Ahmat[61][4][3]=0.;
Ahmat[61][4][4]=1;


/* joint ID: 22 */
Xorigin[22][1]=Ai071[1][4];
Xorigin[22][2]=Ai071[2][4];
Xorigin[22][3]=Ai071[3][4];

Xmcog[22][1]=links[22].mcm[1]*Ai071[1][1] + links[22].mcm[2]*Ai071[1][2] + links[22].mcm[3]*Ai071[1][3] + links[22].m*Ai071[1][4];
Xmcog[22][2]=links[22].mcm[1]*Ai071[2][1] + links[22].mcm[2]*Ai071[2][2] + links[22].mcm[3]*Ai071[2][3] + links[22].m*Ai071[2][4];
Xmcog[22][3]=links[22].mcm[1]*Ai071[3][1] + links[22].mcm[2]*Ai071[3][2] + links[22].mcm[3]*Ai071[3][3] + links[22].m*Ai071[3][4];

Xaxis[22][1]=Ai071[1][3];
Xaxis[22][2]=Ai071[2][3];
Xaxis[22][3]=Ai071[3][3];

Ahmatdof[22][1][1]=Ai071[1][1];
Ahmatdof[22][1][2]=Ai071[1][2];
Ahmatdof[22][1][3]=Ai071[1][3];
Ahmatdof[22][1][4]=Ai071[1][4];

Ahmatdof[22][2][1]=Ai071[2][1];
Ahmatdof[22][2][2]=Ai071[2][2];
Ahmatdof[22][2][3]=Ai071[2][3];
Ahmatdof[22][2][4]=Ai071[2][4];

Ahmatdof[22][3][1]=Ai071[3][1];
Ahmatdof[22][3][2]=Ai071[3][2];
Ahmatdof[22][3][3]=Ai071[3][3];
Ahmatdof[22][3][4]=Ai071[3][4];

Ahmatdof[22][4][1]=0.;
Ahmatdof[22][4][2]=0.;
Ahmatdof[22][4][3]=0.;
Ahmatdof[22][4][4]=1;


/* joint ID: 24 */
Xorigin[24][1]=Ai072[1][4];
Xorigin[24][2]=Ai072[2][4];
Xorigin[24][3]=Ai072[3][4];

Xmcog[24][1]=links[24].mcm[1]*Ai072[1][1] + links[24].mcm[2]*Ai072[1][2] + links[24].mcm[3]*Ai072[1][3] + links[24].m*Ai072[1][4];
Xmcog[24][2]=links[24].mcm[1]*Ai072[2][1] + links[24].mcm[2]*Ai072[2][2] + links[24].mcm[3]*Ai072[2][3] + links[24].m*Ai072[2][4];
Xmcog[24][3]=links[24].mcm[1]*Ai072[3][1] + links[24].mcm[2]*Ai072[3][2] + links[24].mcm[3]*Ai072[3][3] + links[24].m*Ai072[3][4];

Xaxis[24][1]=Ai072[1][3];
Xaxis[24][2]=Ai072[2][3];
Xaxis[24][3]=Ai072[3][3];

Ahmatdof[24][1][1]=Ai072[1][1];
Ahmatdof[24][1][2]=Ai072[1][2];
Ahmatdof[24][1][3]=Ai072[1][3];
Ahmatdof[24][1][4]=Ai072[1][4];

Ahmatdof[24][2][1]=Ai072[2][1];
Ahmatdof[24][2][2]=Ai072[2][2];
Ahmatdof[24][2][3]=Ai072[2][3];
Ahmatdof[24][2][4]=Ai072[2][4];

Ahmatdof[24][3][1]=Ai072[3][1];
Ahmatdof[24][3][2]=Ai072[3][2];
Ahmatdof[24][3][3]=Ai072[3][3];
Ahmatdof[24][3][4]=Ai072[3][4];

Ahmatdof[24][4][1]=0.;
Ahmatdof[24][4][2]=0.;
Ahmatdof[24][4][3]=0.;
Ahmatdof[24][4][4]=1;


/* link 62: {YHIP, 0, 0} */
Xlink[62][1]=Ai072[1][4];
Xlink[62][2]=Ai072[2][4];
Xlink[62][3]=Ai072[3][4];

Ahmat[62][1][1]=Ai072[1][1];
Ahmat[62][1][2]=Ai072[1][2];
Ahmat[62][1][3]=Ai072[1][3];
Ahmat[62][1][4]=Ai072[1][4];

Ahmat[62][2][1]=Ai072[2][1];
Ahmat[62][2][2]=Ai072[2][2];
Ahmat[62][2][3]=Ai072[2][3];
Ahmat[62][2][4]=Ai072[2][4];

Ahmat[62][3][1]=Ai072[3][1];
Ahmat[62][3][2]=Ai072[3][2];
Ahmat[62][3][3]=Ai072[3][3];
Ahmat[62][3][4]=Ai072[3][4];

Ahmat[62][4][1]=0.;
Ahmat[62][4][2]=0.;
Ahmat[62][4][3]=0.;
Ahmat[62][4][4]=1;


/* joint ID: 25 */
Xorigin[25][1]=Ai073[1][4];
Xorigin[25][2]=Ai073[2][4];
Xorigin[25][3]=Ai073[3][4];

Xmcog[25][1]=links[25].mcm[1]*Ai073[1][1] + links[25].mcm[2]*Ai073[1][2] + links[25].mcm[3]*Ai073[1][3] + links[25].m*Ai073[1][4];
Xmcog[25][2]=links[25].mcm[1]*Ai073[2][1] + links[25].mcm[2]*Ai073[2][2] + links[25].mcm[3]*Ai073[2][3] + links[25].m*Ai073[2][4];
Xmcog[25][3]=links[25].mcm[1]*Ai073[3][1] + links[25].mcm[2]*Ai073[3][2] + links[25].mcm[3]*Ai073[3][3] + links[25].m*Ai073[3][4];

Xaxis[25][1]=Ai073[1][3];
Xaxis[25][2]=Ai073[2][3];
Xaxis[25][3]=Ai073[3][3];

Ahmatdof[25][1][1]=Ai073[1][1];
Ahmatdof[25][1][2]=Ai073[1][2];
Ahmatdof[25][1][3]=Ai073[1][3];
Ahmatdof[25][1][4]=Ai073[1][4];

Ahmatdof[25][2][1]=Ai073[2][1];
Ahmatdof[25][2][2]=Ai073[2][2];
Ahmatdof[25][2][3]=Ai073[2][3];
Ahmatdof[25][2][4]=Ai073[2][4];

Ahmatdof[25][3][1]=Ai073[3][1];
Ahmatdof[25][3][2]=Ai073[3][2];
Ahmatdof[25][3][3]=Ai073[3][3];
Ahmatdof[25][3][4]=Ai073[3][4];

Ahmatdof[25][4][1]=0.;
Ahmatdof[25][4][2]=0.;
Ahmatdof[25][4][3]=0.;
Ahmatdof[25][4][4]=1;


/* link 63: {YKNEE, 0, UPPERLEGMOD} */
Xlink[63][1]=Ai073[1][4];
Xlink[63][2]=Ai073[2][4];
Xlink[63][3]=Ai073[3][4];

Ahmat[63][1][1]=Ai074[1][1];
Ahmat[63][1][2]=Ai074[1][2];
Ahmat[63][1][3]=Ai074[1][3];
Ahmat[63][1][4]=Ai074[1][4];

Ahmat[63][2][1]=Ai074[2][1];
Ahmat[63][2][2]=Ai074[2][2];
Ahmat[63][2][3]=Ai074[2][3];
Ahmat[63][2][4]=Ai074[2][4];

Ahmat[63][3][1]=Ai074[3][1];
Ahmat[63][3][2]=Ai074[3][2];
Ahmat[63][3][3]=Ai074[3][3];
Ahmat[63][3][4]=Ai074[3][4];

Ahmat[63][4][1]=0.;
Ahmat[63][4][2]=0.;
Ahmat[63][4][3]=0.;
Ahmat[63][4][4]=1;


/* joint ID: 26 */
Xorigin[26][1]=Ai074[1][4];
Xorigin[26][2]=Ai074[2][4];
Xorigin[26][3]=Ai074[3][4];

Xmcog[26][1]=links[26].mcm[1]*Ai074[1][1] + links[26].mcm[2]*Ai074[1][2] + links[26].mcm[3]*Ai074[1][3] + links[26].m*Ai074[1][4];
Xmcog[26][2]=links[26].mcm[1]*Ai074[2][1] + links[26].mcm[2]*Ai074[2][2] + links[26].mcm[3]*Ai074[2][3] + links[26].m*Ai074[2][4];
Xmcog[26][3]=links[26].mcm[1]*Ai074[3][1] + links[26].mcm[2]*Ai074[3][2] + links[26].mcm[3]*Ai074[3][3] + links[26].m*Ai074[3][4];

Xaxis[26][1]=Ai074[1][3];
Xaxis[26][2]=Ai074[2][3];
Xaxis[26][3]=Ai074[3][3];

Ahmatdof[26][1][1]=Ai074[1][1];
Ahmatdof[26][1][2]=Ai074[1][2];
Ahmatdof[26][1][3]=Ai074[1][3];
Ahmatdof[26][1][4]=Ai074[1][4];

Ahmatdof[26][2][1]=Ai074[2][1];
Ahmatdof[26][2][2]=Ai074[2][2];
Ahmatdof[26][2][3]=Ai074[2][3];
Ahmatdof[26][2][4]=Ai074[2][4];

Ahmatdof[26][3][1]=Ai074[3][1];
Ahmatdof[26][3][2]=Ai074[3][2];
Ahmatdof[26][3][3]=Ai074[3][3];
Ahmatdof[26][3][4]=Ai074[3][4];

Ahmatdof[26][4][1]=0.;
Ahmatdof[26][4][2]=0.;
Ahmatdof[26][4][3]=0.;
Ahmatdof[26][4][4]=1;


/* joint ID: 27 */
Xorigin[27][1]=Ai075[1][4];
Xorigin[27][2]=Ai075[2][4];
Xorigin[27][3]=Ai075[3][4];

Xmcog[27][1]=links[27].mcm[1]*Ai075[1][1] + links[27].mcm[2]*Ai075[1][2] + links[27].mcm[3]*Ai075[1][3] + links[27].m*Ai075[1][4];
Xmcog[27][2]=links[27].mcm[1]*Ai075[2][1] + links[27].mcm[2]*Ai075[2][2] + links[27].mcm[3]*Ai075[2][3] + links[27].m*Ai075[2][4];
Xmcog[27][3]=links[27].mcm[1]*Ai075[3][1] + links[27].mcm[2]*Ai075[3][2] + links[27].mcm[3]*Ai075[3][3] + links[27].m*Ai075[3][4];

Xaxis[27][1]=Ai075[1][3];
Xaxis[27][2]=Ai075[2][3];
Xaxis[27][3]=Ai075[3][3];

Ahmatdof[27][1][1]=Ai075[1][1];
Ahmatdof[27][1][2]=Ai075[1][2];
Ahmatdof[27][1][3]=Ai075[1][3];
Ahmatdof[27][1][4]=Ai075[1][4];

Ahmatdof[27][2][1]=Ai075[2][1];
Ahmatdof[27][2][2]=Ai075[2][2];
Ahmatdof[27][2][3]=Ai075[2][3];
Ahmatdof[27][2][4]=Ai075[2][4];

Ahmatdof[27][3][1]=Ai075[3][1];
Ahmatdof[27][3][2]=Ai075[3][2];
Ahmatdof[27][3][3]=Ai075[3][3];
Ahmatdof[27][3][4]=Ai075[3][4];

Ahmatdof[27][4][1]=0.;
Ahmatdof[27][4][2]=0.;
Ahmatdof[27][4][3]=0.;
Ahmatdof[27][4][4]=1;


/* link 64: {0, 0, LOWERLEG} */
Xlink[64][1]=Ai075[1][4];
Xlink[64][2]=Ai075[2][4];
Xlink[64][3]=Ai075[3][4];

Ahmat[64][1][1]=Ai076[1][1];
Ahmat[64][1][2]=Ai076[1][2];
Ahmat[64][1][3]=Ai076[1][3];
Ahmat[64][1][4]=Ai076[1][4];

Ahmat[64][2][1]=Ai076[2][1];
Ahmat[64][2][2]=Ai076[2][2];
Ahmat[64][2][3]=Ai076[2][3];
Ahmat[64][2][4]=Ai076[2][4];

Ahmat[64][3][1]=Ai076[3][1];
Ahmat[64][3][2]=Ai076[3][2];
Ahmat[64][3][3]=Ai076[3][3];
Ahmat[64][3][4]=Ai076[3][4];

Ahmat[64][4][1]=0.;
Ahmat[64][4][2]=0.;
Ahmat[64][4][3]=0.;
Ahmat[64][4][4]=1;


/* joint ID: 28 */
Xorigin[28][1]=Ai076[1][4];
Xorigin[28][2]=Ai076[2][4];
Xorigin[28][3]=Ai076[3][4];

Xmcog[28][1]=links[28].mcm[1]*Ai076[1][1] + links[28].mcm[2]*Ai076[1][2] + links[28].mcm[3]*Ai076[1][3] + links[28].m*Ai076[1][4];
Xmcog[28][2]=links[28].mcm[1]*Ai076[2][1] + links[28].mcm[2]*Ai076[2][2] + links[28].mcm[3]*Ai076[2][3] + links[28].m*Ai076[2][4];
Xmcog[28][3]=links[28].mcm[1]*Ai076[3][1] + links[28].mcm[2]*Ai076[3][2] + links[28].mcm[3]*Ai076[3][3] + links[28].m*Ai076[3][4];

Xaxis[28][1]=Ai076[1][3];
Xaxis[28][2]=Ai076[2][3];
Xaxis[28][3]=Ai076[3][3];

Ahmatdof[28][1][1]=Ai076[1][1];
Ahmatdof[28][1][2]=Ai076[1][2];
Ahmatdof[28][1][3]=Ai076[1][3];
Ahmatdof[28][1][4]=Ai076[1][4];

Ahmatdof[28][2][1]=Ai076[2][1];
Ahmatdof[28][2][2]=Ai076[2][2];
Ahmatdof[28][2][3]=Ai076[2][3];
Ahmatdof[28][2][4]=Ai076[2][4];

Ahmatdof[28][3][1]=Ai076[3][1];
Ahmatdof[28][3][2]=Ai076[3][2];
Ahmatdof[28][3][3]=Ai076[3][3];
Ahmatdof[28][3][4]=Ai076[3][4];

Ahmatdof[28][4][1]=0.;
Ahmatdof[28][4][2]=0.;
Ahmatdof[28][4][3]=0.;
Ahmatdof[28][4][4]=1;


/* link 65: {ZTOE, -XTOEOUTER, YTOE} */
Xlink[65][1]=Ai077[1][4];
Xlink[65][2]=Ai077[2][4];
Xlink[65][3]=Ai077[3][4];

Ahmat[65][1][1]=Ai077[1][1];
Ahmat[65][1][2]=Ai077[1][2];
Ahmat[65][1][3]=Ai077[1][3];
Ahmat[65][1][4]=Ai077[1][4];

Ahmat[65][2][1]=Ai077[2][1];
Ahmat[65][2][2]=Ai077[2][2];
Ahmat[65][2][3]=Ai077[2][3];
Ahmat[65][2][4]=Ai077[2][4];

Ahmat[65][3][1]=Ai077[3][1];
Ahmat[65][3][2]=Ai077[3][2];
Ahmat[65][3][3]=Ai077[3][3];
Ahmat[65][3][4]=Ai077[3][4];

Ahmat[65][4][1]=0.;
Ahmat[65][4][2]=0.;
Ahmat[65][4][3]=0.;
Ahmat[65][4][4]=1;


/* link 66: {ZTOE, XTOEINNER, YTOE} */
Xlink[66][1]=Ai078[1][4];
Xlink[66][2]=Ai078[2][4];
Xlink[66][3]=Ai078[3][4];

Ahmat[66][1][1]=Ai078[1][1];
Ahmat[66][1][2]=Ai078[1][2];
Ahmat[66][1][3]=Ai078[1][3];
Ahmat[66][1][4]=Ai078[1][4];

Ahmat[66][2][1]=Ai078[2][1];
Ahmat[66][2][2]=Ai078[2][2];
Ahmat[66][2][3]=Ai078[2][3];
Ahmat[66][2][4]=Ai078[2][4];

Ahmat[66][3][1]=Ai078[3][1];
Ahmat[66][3][2]=Ai078[3][2];
Ahmat[66][3][3]=Ai078[3][3];
Ahmat[66][3][4]=Ai078[3][4];

Ahmat[66][4][1]=0.;
Ahmat[66][4][2]=0.;
Ahmat[66][4][3]=0.;
Ahmat[66][4][4]=1;


/* link 67: {ZTOE, -XMETATARSALOUTER, YMETATARSAL} */
Xlink[67][1]=Ai079[1][4];
Xlink[67][2]=Ai079[2][4];
Xlink[67][3]=Ai079[3][4];

Ahmat[67][1][1]=Ai079[1][1];
Ahmat[67][1][2]=Ai079[1][2];
Ahmat[67][1][3]=Ai079[1][3];
Ahmat[67][1][4]=Ai079[1][4];

Ahmat[67][2][1]=Ai079[2][1];
Ahmat[67][2][2]=Ai079[2][2];
Ahmat[67][2][3]=Ai079[2][3];
Ahmat[67][2][4]=Ai079[2][4];

Ahmat[67][3][1]=Ai079[3][1];
Ahmat[67][3][2]=Ai079[3][2];
Ahmat[67][3][3]=Ai079[3][3];
Ahmat[67][3][4]=Ai079[3][4];

Ahmat[67][4][1]=0.;
Ahmat[67][4][2]=0.;
Ahmat[67][4][3]=0.;
Ahmat[67][4][4]=1;


/* link 68: {ZTOE, XMETATARSALINNER, YMETATARSAL} */
Xlink[68][1]=Ai080[1][4];
Xlink[68][2]=Ai080[2][4];
Xlink[68][3]=Ai080[3][4];

Ahmat[68][1][1]=Ai080[1][1];
Ahmat[68][1][2]=Ai080[1][2];
Ahmat[68][1][3]=Ai080[1][3];
Ahmat[68][1][4]=Ai080[1][4];

Ahmat[68][2][1]=Ai080[2][1];
Ahmat[68][2][2]=Ai080[2][2];
Ahmat[68][2][3]=Ai080[2][3];
Ahmat[68][2][4]=Ai080[2][4];

Ahmat[68][3][1]=Ai080[3][1];
Ahmat[68][3][2]=Ai080[3][2];
Ahmat[68][3][3]=Ai080[3][3];
Ahmat[68][3][4]=Ai080[3][4];

Ahmat[68][4][1]=0.;
Ahmat[68][4][2]=0.;
Ahmat[68][4][3]=0.;
Ahmat[68][4][4]=1;


/* link 69: {ZHEEL, -XHEELOUTER, -YHEEL} */
Xlink[69][1]=Ai081[1][4];
Xlink[69][2]=Ai081[2][4];
Xlink[69][3]=Ai081[3][4];

Ahmat[69][1][1]=Ai081[1][1];
Ahmat[69][1][2]=Ai081[1][2];
Ahmat[69][1][3]=Ai081[1][3];
Ahmat[69][1][4]=Ai081[1][4];

Ahmat[69][2][1]=Ai081[2][1];
Ahmat[69][2][2]=Ai081[2][2];
Ahmat[69][2][3]=Ai081[2][3];
Ahmat[69][2][4]=Ai081[2][4];

Ahmat[69][3][1]=Ai081[3][1];
Ahmat[69][3][2]=Ai081[3][2];
Ahmat[69][3][3]=Ai081[3][3];
Ahmat[69][3][4]=Ai081[3][4];

Ahmat[69][4][1]=0.;
Ahmat[69][4][2]=0.;
Ahmat[69][4][3]=0.;
Ahmat[69][4][4]=1;


/* link 70: {ZHEEL, XHEELINNER, -YHEEL} */
Xlink[70][1]=Ai082[1][4];
Xlink[70][2]=Ai082[2][4];
Xlink[70][3]=Ai082[3][4];

Ahmat[70][1][1]=Ai082[1][1];
Ahmat[70][1][2]=Ai082[1][2];
Ahmat[70][1][3]=Ai082[1][3];
Ahmat[70][1][4]=Ai082[1][4];

Ahmat[70][2][1]=Ai082[2][1];
Ahmat[70][2][2]=Ai082[2][2];
Ahmat[70][2][3]=Ai082[2][3];
Ahmat[70][2][4]=Ai082[2][4];

Ahmat[70][3][1]=Ai082[3][1];
Ahmat[70][3][2]=Ai082[3][2];
Ahmat[70][3][3]=Ai082[3][3];
Ahmat[70][3][4]=Ai082[3][4];

Ahmat[70][4][1]=0.;
Ahmat[70][4][2]=0.;
Ahmat[70][4][3]=0.;
Ahmat[70][4][4]=1;


/* link 71: {eff$3$$x[[1]], eff$3$$x[[2]], eff$3$$x[[3]]} */
Xlink[71][1]=Ai083[1][4];
Xlink[71][2]=Ai083[2][4];
Xlink[71][3]=Ai083[3][4];

Ahmat[71][1][1]=Ai083[1][1];
Ahmat[71][1][2]=Ai083[1][2];
Ahmat[71][1][3]=Ai083[1][3];
Ahmat[71][1][4]=Ai083[1][4];

Ahmat[71][2][1]=Ai083[2][1];
Ahmat[71][2][2]=Ai083[2][2];
Ahmat[71][2][3]=Ai083[2][3];
Ahmat[71][2][4]=Ai083[2][4];

Ahmat[71][3][1]=Ai083[3][1];
Ahmat[71][3][2]=Ai083[3][2];
Ahmat[71][3][3]=Ai083[3][3];
Ahmat[71][3][4]=Ai083[3][4];

Ahmat[71][4][1]=0.;
Ahmat[71][4][2]=0.;
Ahmat[71][4][3]=0.;
Ahmat[71][4][4]=1;


/* joint ID: 16 */
Xorigin[16][1]=Ai084[1][4];
Xorigin[16][2]=Ai084[2][4];
Xorigin[16][3]=Ai084[3][4];

Xmcog[16][1]=links[16].mcm[1]*Ai084[1][1] + links[16].mcm[2]*Ai084[1][2] + links[16].mcm[3]*Ai084[1][3] + links[16].m*Ai084[1][4];
Xmcog[16][2]=links[16].mcm[1]*Ai084[2][1] + links[16].mcm[2]*Ai084[2][2] + links[16].mcm[3]*Ai084[2][3] + links[16].m*Ai084[2][4];
Xmcog[16][3]=links[16].mcm[1]*Ai084[3][1] + links[16].mcm[2]*Ai084[3][2] + links[16].mcm[3]*Ai084[3][3] + links[16].m*Ai084[3][4];

Xaxis[16][1]=Ai084[1][3];
Xaxis[16][2]=Ai084[2][3];
Xaxis[16][3]=Ai084[3][3];

Ahmatdof[16][1][1]=Ai084[1][1];
Ahmatdof[16][1][2]=Ai084[1][2];
Ahmatdof[16][1][3]=Ai084[1][3];
Ahmatdof[16][1][4]=Ai084[1][4];

Ahmatdof[16][2][1]=Ai084[2][1];
Ahmatdof[16][2][2]=Ai084[2][2];
Ahmatdof[16][2][3]=Ai084[2][3];
Ahmatdof[16][2][4]=Ai084[2][4];

Ahmatdof[16][3][1]=Ai084[3][1];
Ahmatdof[16][3][2]=Ai084[3][2];
Ahmatdof[16][3][3]=Ai084[3][3];
Ahmatdof[16][3][4]=Ai084[3][4];

Ahmatdof[16][4][1]=0.;
Ahmatdof[16][4][2]=0.;
Ahmatdof[16][4][3]=0.;
Ahmatdof[16][4][4]=1;


/* link 72: {-XHIP, 0, 0} */
Xlink[72][1]=Ai084[1][4];
Xlink[72][2]=Ai084[2][4];
Xlink[72][3]=Ai084[3][4];

Ahmat[72][1][1]=Ai085[1][1];
Ahmat[72][1][2]=Ai085[1][2];
Ahmat[72][1][3]=Ai085[1][3];
Ahmat[72][1][4]=Ai085[1][4];

Ahmat[72][2][1]=Ai085[2][1];
Ahmat[72][2][2]=Ai085[2][2];
Ahmat[72][2][3]=Ai085[2][3];
Ahmat[72][2][4]=Ai085[2][4];

Ahmat[72][3][1]=Ai085[3][1];
Ahmat[72][3][2]=Ai085[3][2];
Ahmat[72][3][3]=Ai085[3][3];
Ahmat[72][3][4]=Ai085[3][4];

Ahmat[72][4][1]=0.;
Ahmat[72][4][2]=0.;
Ahmat[72][4][3]=0.;
Ahmat[72][4][4]=1;


/* joint ID: 15 */
Xorigin[15][1]=Ai085[1][4];
Xorigin[15][2]=Ai085[2][4];
Xorigin[15][3]=Ai085[3][4];

Xmcog[15][1]=links[15].mcm[1]*Ai085[1][1] + links[15].mcm[2]*Ai085[1][2] + links[15].mcm[3]*Ai085[1][3] + links[15].m*Ai085[1][4];
Xmcog[15][2]=links[15].mcm[1]*Ai085[2][1] + links[15].mcm[2]*Ai085[2][2] + links[15].mcm[3]*Ai085[2][3] + links[15].m*Ai085[2][4];
Xmcog[15][3]=links[15].mcm[1]*Ai085[3][1] + links[15].mcm[2]*Ai085[3][2] + links[15].mcm[3]*Ai085[3][3] + links[15].m*Ai085[3][4];

Xaxis[15][1]=Ai085[1][3];
Xaxis[15][2]=Ai085[2][3];
Xaxis[15][3]=Ai085[3][3];

Ahmatdof[15][1][1]=Ai085[1][1];
Ahmatdof[15][1][2]=Ai085[1][2];
Ahmatdof[15][1][3]=Ai085[1][3];
Ahmatdof[15][1][4]=Ai085[1][4];

Ahmatdof[15][2][1]=Ai085[2][1];
Ahmatdof[15][2][2]=Ai085[2][2];
Ahmatdof[15][2][3]=Ai085[2][3];
Ahmatdof[15][2][4]=Ai085[2][4];

Ahmatdof[15][3][1]=Ai085[3][1];
Ahmatdof[15][3][2]=Ai085[3][2];
Ahmatdof[15][3][3]=Ai085[3][3];
Ahmatdof[15][3][4]=Ai085[3][4];

Ahmatdof[15][4][1]=0.;
Ahmatdof[15][4][2]=0.;
Ahmatdof[15][4][3]=0.;
Ahmatdof[15][4][4]=1;


/* joint ID: 17 */
Xorigin[17][1]=Ai086[1][4];
Xorigin[17][2]=Ai086[2][4];
Xorigin[17][3]=Ai086[3][4];

Xmcog[17][1]=links[17].mcm[1]*Ai086[1][1] + links[17].mcm[2]*Ai086[1][2] + links[17].mcm[3]*Ai086[1][3] + links[17].m*Ai086[1][4];
Xmcog[17][2]=links[17].mcm[1]*Ai086[2][1] + links[17].mcm[2]*Ai086[2][2] + links[17].mcm[3]*Ai086[2][3] + links[17].m*Ai086[2][4];
Xmcog[17][3]=links[17].mcm[1]*Ai086[3][1] + links[17].mcm[2]*Ai086[3][2] + links[17].mcm[3]*Ai086[3][3] + links[17].m*Ai086[3][4];

Xaxis[17][1]=Ai086[1][3];
Xaxis[17][2]=Ai086[2][3];
Xaxis[17][3]=Ai086[3][3];

Ahmatdof[17][1][1]=Ai086[1][1];
Ahmatdof[17][1][2]=Ai086[1][2];
Ahmatdof[17][1][3]=Ai086[1][3];
Ahmatdof[17][1][4]=Ai086[1][4];

Ahmatdof[17][2][1]=Ai086[2][1];
Ahmatdof[17][2][2]=Ai086[2][2];
Ahmatdof[17][2][3]=Ai086[2][3];
Ahmatdof[17][2][4]=Ai086[2][4];

Ahmatdof[17][3][1]=Ai086[3][1];
Ahmatdof[17][3][2]=Ai086[3][2];
Ahmatdof[17][3][3]=Ai086[3][3];
Ahmatdof[17][3][4]=Ai086[3][4];

Ahmatdof[17][4][1]=0.;
Ahmatdof[17][4][2]=0.;
Ahmatdof[17][4][3]=0.;
Ahmatdof[17][4][4]=1;


/* link 73: {YHIP, 0, 0} */
Xlink[73][1]=Ai086[1][4];
Xlink[73][2]=Ai086[2][4];
Xlink[73][3]=Ai086[3][4];

Ahmat[73][1][1]=Ai086[1][1];
Ahmat[73][1][2]=Ai086[1][2];
Ahmat[73][1][3]=Ai086[1][3];
Ahmat[73][1][4]=Ai086[1][4];

Ahmat[73][2][1]=Ai086[2][1];
Ahmat[73][2][2]=Ai086[2][2];
Ahmat[73][2][3]=Ai086[2][3];
Ahmat[73][2][4]=Ai086[2][4];

Ahmat[73][3][1]=Ai086[3][1];
Ahmat[73][3][2]=Ai086[3][2];
Ahmat[73][3][3]=Ai086[3][3];
Ahmat[73][3][4]=Ai086[3][4];

Ahmat[73][4][1]=0.;
Ahmat[73][4][2]=0.;
Ahmat[73][4][3]=0.;
Ahmat[73][4][4]=1;


/* joint ID: 18 */
Xorigin[18][1]=Ai087[1][4];
Xorigin[18][2]=Ai087[2][4];
Xorigin[18][3]=Ai087[3][4];

Xmcog[18][1]=links[18].mcm[1]*Ai087[1][1] + links[18].mcm[2]*Ai087[1][2] + links[18].mcm[3]*Ai087[1][3] + links[18].m*Ai087[1][4];
Xmcog[18][2]=links[18].mcm[1]*Ai087[2][1] + links[18].mcm[2]*Ai087[2][2] + links[18].mcm[3]*Ai087[2][3] + links[18].m*Ai087[2][4];
Xmcog[18][3]=links[18].mcm[1]*Ai087[3][1] + links[18].mcm[2]*Ai087[3][2] + links[18].mcm[3]*Ai087[3][3] + links[18].m*Ai087[3][4];

Xaxis[18][1]=Ai087[1][3];
Xaxis[18][2]=Ai087[2][3];
Xaxis[18][3]=Ai087[3][3];

Ahmatdof[18][1][1]=Ai087[1][1];
Ahmatdof[18][1][2]=Ai087[1][2];
Ahmatdof[18][1][3]=Ai087[1][3];
Ahmatdof[18][1][4]=Ai087[1][4];

Ahmatdof[18][2][1]=Ai087[2][1];
Ahmatdof[18][2][2]=Ai087[2][2];
Ahmatdof[18][2][3]=Ai087[2][3];
Ahmatdof[18][2][4]=Ai087[2][4];

Ahmatdof[18][3][1]=Ai087[3][1];
Ahmatdof[18][3][2]=Ai087[3][2];
Ahmatdof[18][3][3]=Ai087[3][3];
Ahmatdof[18][3][4]=Ai087[3][4];

Ahmatdof[18][4][1]=0.;
Ahmatdof[18][4][2]=0.;
Ahmatdof[18][4][3]=0.;
Ahmatdof[18][4][4]=1;


/* link 74: {YKNEE, 0, -UPPERLEGMOD} */
Xlink[74][1]=Ai087[1][4];
Xlink[74][2]=Ai087[2][4];
Xlink[74][3]=Ai087[3][4];

Ahmat[74][1][1]=Ai088[1][1];
Ahmat[74][1][2]=Ai088[1][2];
Ahmat[74][1][3]=Ai088[1][3];
Ahmat[74][1][4]=Ai088[1][4];

Ahmat[74][2][1]=Ai088[2][1];
Ahmat[74][2][2]=Ai088[2][2];
Ahmat[74][2][3]=Ai088[2][3];
Ahmat[74][2][4]=Ai088[2][4];

Ahmat[74][3][1]=Ai088[3][1];
Ahmat[74][3][2]=Ai088[3][2];
Ahmat[74][3][3]=Ai088[3][3];
Ahmat[74][3][4]=Ai088[3][4];

Ahmat[74][4][1]=0.;
Ahmat[74][4][2]=0.;
Ahmat[74][4][3]=0.;
Ahmat[74][4][4]=1;


/* joint ID: 19 */
Xorigin[19][1]=Ai088[1][4];
Xorigin[19][2]=Ai088[2][4];
Xorigin[19][3]=Ai088[3][4];

Xmcog[19][1]=links[19].mcm[1]*Ai088[1][1] + links[19].mcm[2]*Ai088[1][2] + links[19].mcm[3]*Ai088[1][3] + links[19].m*Ai088[1][4];
Xmcog[19][2]=links[19].mcm[1]*Ai088[2][1] + links[19].mcm[2]*Ai088[2][2] + links[19].mcm[3]*Ai088[2][3] + links[19].m*Ai088[2][4];
Xmcog[19][3]=links[19].mcm[1]*Ai088[3][1] + links[19].mcm[2]*Ai088[3][2] + links[19].mcm[3]*Ai088[3][3] + links[19].m*Ai088[3][4];

Xaxis[19][1]=Ai088[1][3];
Xaxis[19][2]=Ai088[2][3];
Xaxis[19][3]=Ai088[3][3];

Ahmatdof[19][1][1]=Ai088[1][1];
Ahmatdof[19][1][2]=Ai088[1][2];
Ahmatdof[19][1][3]=Ai088[1][3];
Ahmatdof[19][1][4]=Ai088[1][4];

Ahmatdof[19][2][1]=Ai088[2][1];
Ahmatdof[19][2][2]=Ai088[2][2];
Ahmatdof[19][2][3]=Ai088[2][3];
Ahmatdof[19][2][4]=Ai088[2][4];

Ahmatdof[19][3][1]=Ai088[3][1];
Ahmatdof[19][3][2]=Ai088[3][2];
Ahmatdof[19][3][3]=Ai088[3][3];
Ahmatdof[19][3][4]=Ai088[3][4];

Ahmatdof[19][4][1]=0.;
Ahmatdof[19][4][2]=0.;
Ahmatdof[19][4][3]=0.;
Ahmatdof[19][4][4]=1;


/* joint ID: 20 */
Xorigin[20][1]=Ai089[1][4];
Xorigin[20][2]=Ai089[2][4];
Xorigin[20][3]=Ai089[3][4];

Xmcog[20][1]=links[20].mcm[1]*Ai089[1][1] + links[20].mcm[2]*Ai089[1][2] + links[20].mcm[3]*Ai089[1][3] + links[20].m*Ai089[1][4];
Xmcog[20][2]=links[20].mcm[1]*Ai089[2][1] + links[20].mcm[2]*Ai089[2][2] + links[20].mcm[3]*Ai089[2][3] + links[20].m*Ai089[2][4];
Xmcog[20][3]=links[20].mcm[1]*Ai089[3][1] + links[20].mcm[2]*Ai089[3][2] + links[20].mcm[3]*Ai089[3][3] + links[20].m*Ai089[3][4];

Xaxis[20][1]=Ai089[1][3];
Xaxis[20][2]=Ai089[2][3];
Xaxis[20][3]=Ai089[3][3];

Ahmatdof[20][1][1]=Ai089[1][1];
Ahmatdof[20][1][2]=Ai089[1][2];
Ahmatdof[20][1][3]=Ai089[1][3];
Ahmatdof[20][1][4]=Ai089[1][4];

Ahmatdof[20][2][1]=Ai089[2][1];
Ahmatdof[20][2][2]=Ai089[2][2];
Ahmatdof[20][2][3]=Ai089[2][3];
Ahmatdof[20][2][4]=Ai089[2][4];

Ahmatdof[20][3][1]=Ai089[3][1];
Ahmatdof[20][3][2]=Ai089[3][2];
Ahmatdof[20][3][3]=Ai089[3][3];
Ahmatdof[20][3][4]=Ai089[3][4];

Ahmatdof[20][4][1]=0.;
Ahmatdof[20][4][2]=0.;
Ahmatdof[20][4][3]=0.;
Ahmatdof[20][4][4]=1;


/* link 75: {0, 0, -LOWERLEG} */
Xlink[75][1]=Ai089[1][4];
Xlink[75][2]=Ai089[2][4];
Xlink[75][3]=Ai089[3][4];

Ahmat[75][1][1]=Ai090[1][1];
Ahmat[75][1][2]=Ai090[1][2];
Ahmat[75][1][3]=Ai090[1][3];
Ahmat[75][1][4]=Ai090[1][4];

Ahmat[75][2][1]=Ai090[2][1];
Ahmat[75][2][2]=Ai090[2][2];
Ahmat[75][2][3]=Ai090[2][3];
Ahmat[75][2][4]=Ai090[2][4];

Ahmat[75][3][1]=Ai090[3][1];
Ahmat[75][3][2]=Ai090[3][2];
Ahmat[75][3][3]=Ai090[3][3];
Ahmat[75][3][4]=Ai090[3][4];

Ahmat[75][4][1]=0.;
Ahmat[75][4][2]=0.;
Ahmat[75][4][3]=0.;
Ahmat[75][4][4]=1;


/* joint ID: 21 */
Xorigin[21][1]=Ai090[1][4];
Xorigin[21][2]=Ai090[2][4];
Xorigin[21][3]=Ai090[3][4];

Xmcog[21][1]=links[21].mcm[1]*Ai090[1][1] + links[21].mcm[2]*Ai090[1][2] + links[21].mcm[3]*Ai090[1][3] + links[21].m*Ai090[1][4];
Xmcog[21][2]=links[21].mcm[1]*Ai090[2][1] + links[21].mcm[2]*Ai090[2][2] + links[21].mcm[3]*Ai090[2][3] + links[21].m*Ai090[2][4];
Xmcog[21][3]=links[21].mcm[1]*Ai090[3][1] + links[21].mcm[2]*Ai090[3][2] + links[21].mcm[3]*Ai090[3][3] + links[21].m*Ai090[3][4];

Xaxis[21][1]=Ai090[1][3];
Xaxis[21][2]=Ai090[2][3];
Xaxis[21][3]=Ai090[3][3];

Ahmatdof[21][1][1]=Ai090[1][1];
Ahmatdof[21][1][2]=Ai090[1][2];
Ahmatdof[21][1][3]=Ai090[1][3];
Ahmatdof[21][1][4]=Ai090[1][4];

Ahmatdof[21][2][1]=Ai090[2][1];
Ahmatdof[21][2][2]=Ai090[2][2];
Ahmatdof[21][2][3]=Ai090[2][3];
Ahmatdof[21][2][4]=Ai090[2][4];

Ahmatdof[21][3][1]=Ai090[3][1];
Ahmatdof[21][3][2]=Ai090[3][2];
Ahmatdof[21][3][3]=Ai090[3][3];
Ahmatdof[21][3][4]=Ai090[3][4];

Ahmatdof[21][4][1]=0.;
Ahmatdof[21][4][2]=0.;
Ahmatdof[21][4][3]=0.;
Ahmatdof[21][4][4]=1;


/* link 76: {ZTOE, -XTOEOUTER, -YTOE} */
Xlink[76][1]=Ai091[1][4];
Xlink[76][2]=Ai091[2][4];
Xlink[76][3]=Ai091[3][4];

Ahmat[76][1][1]=Ai091[1][1];
Ahmat[76][1][2]=Ai091[1][2];
Ahmat[76][1][3]=Ai091[1][3];
Ahmat[76][1][4]=Ai091[1][4];

Ahmat[76][2][1]=Ai091[2][1];
Ahmat[76][2][2]=Ai091[2][2];
Ahmat[76][2][3]=Ai091[2][3];
Ahmat[76][2][4]=Ai091[2][4];

Ahmat[76][3][1]=Ai091[3][1];
Ahmat[76][3][2]=Ai091[3][2];
Ahmat[76][3][3]=Ai091[3][3];
Ahmat[76][3][4]=Ai091[3][4];

Ahmat[76][4][1]=0.;
Ahmat[76][4][2]=0.;
Ahmat[76][4][3]=0.;
Ahmat[76][4][4]=1;


/* link 77: {ZTOE, XTOEINNER, -YTOE} */
Xlink[77][1]=Ai092[1][4];
Xlink[77][2]=Ai092[2][4];
Xlink[77][3]=Ai092[3][4];

Ahmat[77][1][1]=Ai092[1][1];
Ahmat[77][1][2]=Ai092[1][2];
Ahmat[77][1][3]=Ai092[1][3];
Ahmat[77][1][4]=Ai092[1][4];

Ahmat[77][2][1]=Ai092[2][1];
Ahmat[77][2][2]=Ai092[2][2];
Ahmat[77][2][3]=Ai092[2][3];
Ahmat[77][2][4]=Ai092[2][4];

Ahmat[77][3][1]=Ai092[3][1];
Ahmat[77][3][2]=Ai092[3][2];
Ahmat[77][3][3]=Ai092[3][3];
Ahmat[77][3][4]=Ai092[3][4];

Ahmat[77][4][1]=0.;
Ahmat[77][4][2]=0.;
Ahmat[77][4][3]=0.;
Ahmat[77][4][4]=1;


/* link 78: {ZTOE, -XMETATARSALOUTER, -YMETATARSAL} */
Xlink[78][1]=Ai093[1][4];
Xlink[78][2]=Ai093[2][4];
Xlink[78][3]=Ai093[3][4];

Ahmat[78][1][1]=Ai093[1][1];
Ahmat[78][1][2]=Ai093[1][2];
Ahmat[78][1][3]=Ai093[1][3];
Ahmat[78][1][4]=Ai093[1][4];

Ahmat[78][2][1]=Ai093[2][1];
Ahmat[78][2][2]=Ai093[2][2];
Ahmat[78][2][3]=Ai093[2][3];
Ahmat[78][2][4]=Ai093[2][4];

Ahmat[78][3][1]=Ai093[3][1];
Ahmat[78][3][2]=Ai093[3][2];
Ahmat[78][3][3]=Ai093[3][3];
Ahmat[78][3][4]=Ai093[3][4];

Ahmat[78][4][1]=0.;
Ahmat[78][4][2]=0.;
Ahmat[78][4][3]=0.;
Ahmat[78][4][4]=1;


/* link 79: {ZTOE, XMETATARSALINNER, -YMETATARSAL} */
Xlink[79][1]=Ai094[1][4];
Xlink[79][2]=Ai094[2][4];
Xlink[79][3]=Ai094[3][4];

Ahmat[79][1][1]=Ai094[1][1];
Ahmat[79][1][2]=Ai094[1][2];
Ahmat[79][1][3]=Ai094[1][3];
Ahmat[79][1][4]=Ai094[1][4];

Ahmat[79][2][1]=Ai094[2][1];
Ahmat[79][2][2]=Ai094[2][2];
Ahmat[79][2][3]=Ai094[2][3];
Ahmat[79][2][4]=Ai094[2][4];

Ahmat[79][3][1]=Ai094[3][1];
Ahmat[79][3][2]=Ai094[3][2];
Ahmat[79][3][3]=Ai094[3][3];
Ahmat[79][3][4]=Ai094[3][4];

Ahmat[79][4][1]=0.;
Ahmat[79][4][2]=0.;
Ahmat[79][4][3]=0.;
Ahmat[79][4][4]=1;


/* link 80: {ZHEEL, -XHEELOUTER, YHEEL} */
Xlink[80][1]=Ai095[1][4];
Xlink[80][2]=Ai095[2][4];
Xlink[80][3]=Ai095[3][4];

Ahmat[80][1][1]=Ai095[1][1];
Ahmat[80][1][2]=Ai095[1][2];
Ahmat[80][1][3]=Ai095[1][3];
Ahmat[80][1][4]=Ai095[1][4];

Ahmat[80][2][1]=Ai095[2][1];
Ahmat[80][2][2]=Ai095[2][2];
Ahmat[80][2][3]=Ai095[2][3];
Ahmat[80][2][4]=Ai095[2][4];

Ahmat[80][3][1]=Ai095[3][1];
Ahmat[80][3][2]=Ai095[3][2];
Ahmat[80][3][3]=Ai095[3][3];
Ahmat[80][3][4]=Ai095[3][4];

Ahmat[80][4][1]=0.;
Ahmat[80][4][2]=0.;
Ahmat[80][4][3]=0.;
Ahmat[80][4][4]=1;


/* link 81: {ZHEEL, XHEELINNER, YHEEL} */
Xlink[81][1]=Ai096[1][4];
Xlink[81][2]=Ai096[2][4];
Xlink[81][3]=Ai096[3][4];

Ahmat[81][1][1]=Ai096[1][1];
Ahmat[81][1][2]=Ai096[1][2];
Ahmat[81][1][3]=Ai096[1][3];
Ahmat[81][1][4]=Ai096[1][4];

Ahmat[81][2][1]=Ai096[2][1];
Ahmat[81][2][2]=Ai096[2][2];
Ahmat[81][2][3]=Ai096[2][3];
Ahmat[81][2][4]=Ai096[2][4];

Ahmat[81][3][1]=Ai096[3][1];
Ahmat[81][3][2]=Ai096[3][2];
Ahmat[81][3][3]=Ai096[3][3];
Ahmat[81][3][4]=Ai096[3][4];

Ahmat[81][4][1]=0.;
Ahmat[81][4][2]=0.;
Ahmat[81][4][3]=0.;
Ahmat[81][4][4]=1;


/* link 82: {eff$4$$x[[1]], eff$4$$x[[2]], eff$4$$x[[3]]} */
Xlink[82][1]=Ai097[1][4];
Xlink[82][2]=Ai097[2][4];
Xlink[82][3]=Ai097[3][4];

Ahmat[82][1][1]=Ai097[1][1];
Ahmat[82][1][2]=Ai097[1][2];
Ahmat[82][1][3]=Ai097[1][3];
Ahmat[82][1][4]=Ai097[1][4];

Ahmat[82][2][1]=Ai097[2][1];
Ahmat[82][2][2]=Ai097[2][2];
Ahmat[82][2][3]=Ai097[2][3];
Ahmat[82][2][4]=Ai097[2][4];

Ahmat[82][3][1]=Ai097[3][1];
Ahmat[82][3][2]=Ai097[3][2];
Ahmat[82][3][3]=Ai097[3][3];
Ahmat[82][3][4]=Ai097[3][4];

Ahmat[82][4][1]=0.;
Ahmat[82][4][2]=0.;
Ahmat[82][4][3]=0.;
Ahmat[82][4][4]=1;


