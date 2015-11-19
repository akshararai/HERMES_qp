/* this function assumes that the array XRot[n_links+1][3+1][3+1]
 is passed as an argument */

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



/* inverse rotation matrices */
Si00[1][1]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[2],2);
Si00[1][2]=2*(baseo[0].q[2]*baseo[0].q[3] - baseo[0].q[1]*baseo[0].q[4]);
Si00[1][3]=2*(baseo[0].q[1]*baseo[0].q[3] + baseo[0].q[2]*baseo[0].q[4]);

Si00[2][1]=2*(baseo[0].q[2]*baseo[0].q[3] + baseo[0].q[1]*baseo[0].q[4]);
Si00[2][2]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[3],2);
Si00[2][3]=2*(-(baseo[0].q[1]*baseo[0].q[2]) + baseo[0].q[3]*baseo[0].q[4]);

Si00[3][1]=2*(-(baseo[0].q[1]*baseo[0].q[3]) + baseo[0].q[2]*baseo[0].q[4]);
Si00[3][2]=2*(baseo[0].q[1]*baseo[0].q[2] + baseo[0].q[3]*baseo[0].q[4]);
Si00[3][3]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[4],2);


Si01[1][1]=cstate29th;
Si01[1][2]=-sstate29th;

Si01[2][1]=-sstate29th;
Si01[2][2]=-cstate29th;


Si12[1][1]=-sstate30th;
Si12[1][2]=-cstate30th;

Si12[3][1]=cstate30th;
Si12[3][2]=-sstate30th;


Si23[1][1]=cstate31th;
Si23[1][2]=-sstate31th;

Si23[3][1]=-sstate31th;
Si23[3][2]=-cstate31th;


Si34[1][1]=0.7071067811865475*cstate1th;
Si34[1][2]=-0.7071067811865475*sstate1th;

Si34[2][1]=-sstate1th;
Si34[2][2]=-cstate1th;

Si34[3][1]=0.7071067811865475*cstate1th;
Si34[3][2]=-0.7071067811865475*sstate1th;


Si45[1][1]=-0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;
Si45[1][2]=-0.7071067811865475*cstate2th + 0.7071067811865475*sstate2th;

Si45[3][1]=0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;
Si45[3][2]=-0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;


Si56[1][1]=cstate3th;
Si56[1][2]=-sstate3th;

Si56[3][1]=-sstate3th;
Si56[3][2]=-cstate3th;


Si67[2][1]=cstate4th;
Si67[2][2]=-sstate4th;

Si67[3][1]=sstate4th;
Si67[3][2]=cstate4th;


Si78[1][1]=cstate5th;
Si78[1][2]=-sstate5th;

Si78[3][1]=-sstate5th;
Si78[3][2]=-cstate5th;


Si89[2][1]=sstate6th;
Si89[2][2]=cstate6th;

Si89[3][1]=-cstate6th;
Si89[3][2]=sstate6th;


Si910[1][1]=cstate7th;
Si910[1][2]=-sstate7th;

Si910[3][1]=sstate7th;
Si910[3][2]=cstate7th;


Si1011[1][1]=rceff2a2*rceff2a3;
Si1011[1][2]=-(rceff2a2*rseff2a3);
Si1011[1][3]=rseff2a2;

Si1011[2][1]=rceff2a3*rseff2a1*rseff2a2 + rceff2a1*rseff2a3;
Si1011[2][2]=rceff2a1*rceff2a3 - rseff2a1*rseff2a2*rseff2a3;
Si1011[2][3]=-(rceff2a2*rseff2a1);

Si1011[3][1]=-(rceff2a1*rceff2a3*rseff2a2) + rseff2a1*rseff2a3;
Si1011[3][2]=rceff2a3*rseff2a1 + rceff2a1*rseff2a2*rseff2a3;
Si1011[3][3]=rceff2a1*rceff2a2;


Si1012[1][1]=rcTHUMBZROT;
Si1012[1][2]=-(cstate39th*rsTHUMBZROT);
Si1012[1][3]=rsTHUMBZROT*sstate39th;

Si1012[2][1]=rcHANDXROT*rsTHUMBZROT;
Si1012[2][2]=cstate39th*rcHANDXROT*rcTHUMBZROT - rsHANDXROT*sstate39th;
Si1012[2][3]=-(cstate39th*rsHANDXROT) - rcHANDXROT*rcTHUMBZROT*sstate39th;

Si1012[3][1]=rsHANDXROT*rsTHUMBZROT;
Si1012[3][2]=cstate39th*rcTHUMBZROT*rsHANDXROT + rcHANDXROT*sstate39th;
Si1012[3][3]=cstate39th*rcHANDXROT - rcTHUMBZROT*rsHANDXROT*sstate39th;


Si1213[1][1]=cstate40th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate40th;
Si1213[1][2]=-(cstate40th*rsTHUMBANGLEOFF) + rcTHUMBANGLEOFF*sstate40th;

Si1213[2][1]=cstate40th*rsTHUMBANGLEOFF - rcTHUMBANGLEOFF*sstate40th;
Si1213[2][2]=cstate40th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate40th;


Si1314[1][1]=rcMinusstate40th;
Si1314[1][2]=-rsMinusstate40th;

Si1314[2][1]=rsMinusstate40th;
Si1314[2][2]=rcMinusstate40th;



Si1016[1][1]=cstate41th;
Si1016[1][2]=-sstate41th;

Si1016[2][1]=rcHANDXROT*sstate41th;
Si1016[2][2]=cstate41th*rcHANDXROT;
Si1016[2][3]=-rsHANDXROT;

Si1016[3][1]=rsHANDXROT*sstate41th;
Si1016[3][2]=cstate41th*rsHANDXROT;
Si1016[3][3]=rcHANDXROT;


Si1617[1][1]=rcstate41th;
Si1617[1][2]=-rsstate41th;

Si1617[2][1]=rsstate41th;
Si1617[2][2]=rcstate41th;


Si1718[1][1]=rcstate41th;
Si1718[1][2]=-rsstate41th;

Si1718[2][1]=rsstate41th;
Si1718[2][2]=rcstate41th;



Si1020[1][1]=cstate42th;
Si1020[1][2]=-sstate42th;

Si1020[2][1]=rcHANDXROT*sstate42th;
Si1020[2][2]=cstate42th*rcHANDXROT;
Si1020[2][3]=-rsHANDXROT;

Si1020[3][1]=rsHANDXROT*sstate42th;
Si1020[3][2]=cstate42th*rsHANDXROT;
Si1020[3][3]=rcHANDXROT;


Si2021[1][1]=rcstate42th;
Si2021[1][2]=-rsstate42th;

Si2021[2][1]=rsstate42th;
Si2021[2][2]=rcstate42th;


Si2122[1][1]=rcstate42th;
Si2122[1][2]=-rsstate42th;

Si2122[2][1]=rsstate42th;
Si2122[2][2]=rcstate42th;



Si1024[1][1]=cstate43th;
Si1024[1][2]=-sstate43th;

Si1024[2][1]=rcHANDXROT*sstate43th;
Si1024[2][2]=cstate43th*rcHANDXROT;
Si1024[2][3]=-rsHANDXROT;

Si1024[3][1]=rsHANDXROT*sstate43th;
Si1024[3][2]=cstate43th*rsHANDXROT;
Si1024[3][3]=rcHANDXROT;


Si2425[1][1]=rcstate43th;
Si2425[1][2]=-rsstate43th;

Si2425[2][1]=rsstate43th;
Si2425[2][2]=rcstate43th;


Si2526[1][1]=rcstate43th;
Si2526[1][2]=-rsstate43th;

Si2526[2][1]=rsstate43th;
Si2526[2][2]=rcstate43th;



Si1028[1][1]=cstate44th;
Si1028[1][2]=-sstate44th;

Si1028[2][1]=rcHANDXROT*sstate44th;
Si1028[2][2]=cstate44th*rcHANDXROT;
Si1028[2][3]=-rsHANDXROT;

Si1028[3][1]=rsHANDXROT*sstate44th;
Si1028[3][2]=cstate44th*rsHANDXROT;
Si1028[3][3]=rcHANDXROT;


Si2829[1][1]=rcstate44th;
Si2829[1][2]=-rsstate44th;

Si2829[2][1]=rsstate44th;
Si2829[2][2]=rcstate44th;


Si2930[1][1]=rcstate44th;
Si2930[1][2]=-rsstate44th;

Si2930[2][1]=rsstate44th;
Si2930[2][2]=rcstate44th;



Si332[1][1]=0.7071067811865475*cstate8th;
Si332[1][2]=-0.7071067811865475*sstate8th;

Si332[2][1]=-sstate8th;
Si332[2][2]=-cstate8th;

Si332[3][1]=-0.7071067811865475*cstate8th;
Si332[3][2]=0.7071067811865475*sstate8th;


Si3233[1][1]=-0.7071067811865475*cstate9th - 0.7071067811865475*sstate9th;
Si3233[1][2]=-0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;

Si3233[3][1]=-0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;
Si3233[3][2]=0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;


Si3334[1][1]=cstate10th;
Si3334[1][2]=-sstate10th;

Si3334[3][1]=sstate10th;
Si3334[3][2]=cstate10th;


Si3435[2][1]=cstate11th;
Si3435[2][2]=-sstate11th;

Si3435[3][1]=-sstate11th;
Si3435[3][2]=-cstate11th;


Si3536[1][1]=cstate12th;
Si3536[1][2]=-sstate12th;

Si3536[3][1]=sstate12th;
Si3536[3][2]=cstate12th;


Si3637[2][1]=sstate13th;
Si3637[2][2]=cstate13th;

Si3637[3][1]=cstate13th;
Si3637[3][2]=-sstate13th;


Si3738[1][1]=cstate14th;
Si3738[1][2]=-sstate14th;

Si3738[3][1]=-sstate14th;
Si3738[3][2]=-cstate14th;


Si3839[1][1]=rceff1a2*rceff1a3;
Si3839[1][2]=-(rceff1a2*rseff1a3);
Si3839[1][3]=rseff1a2;

Si3839[2][1]=rceff1a3*rseff1a1*rseff1a2 + rceff1a1*rseff1a3;
Si3839[2][2]=rceff1a1*rceff1a3 - rseff1a1*rseff1a2*rseff1a3;
Si3839[2][3]=-(rceff1a2*rseff1a1);

Si3839[3][1]=-(rceff1a1*rceff1a3*rseff1a2) + rseff1a1*rseff1a3;
Si3839[3][2]=rceff1a3*rseff1a1 + rceff1a1*rseff1a2*rseff1a3;
Si3839[3][3]=rceff1a1*rceff1a2;


Si3840[1][1]=rcTHUMBZROT;
Si3840[1][2]=-(cstate45th*rsTHUMBZROT);
Si3840[1][3]=rsTHUMBZROT*sstate45th;

Si3840[2][1]=rcMinusHANDXROT*rsTHUMBZROT;
Si3840[2][2]=cstate45th*rcMinusHANDXROT*rcTHUMBZROT - rsMinusHANDXROT*sstate45th;
Si3840[2][3]=-(cstate45th*rsMinusHANDXROT) - rcMinusHANDXROT*rcTHUMBZROT*sstate45th;

Si3840[3][1]=rsMinusHANDXROT*rsTHUMBZROT;
Si3840[3][2]=cstate45th*rcTHUMBZROT*rsMinusHANDXROT + rcMinusHANDXROT*sstate45th;
Si3840[3][3]=cstate45th*rcMinusHANDXROT - rcTHUMBZROT*rsMinusHANDXROT*sstate45th;


Si4041[1][1]=cstate46th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate46th;
Si4041[1][2]=-(cstate46th*rsTHUMBANGLEOFF) + rcTHUMBANGLEOFF*sstate46th;

Si4041[2][1]=cstate46th*rsTHUMBANGLEOFF - rcTHUMBANGLEOFF*sstate46th;
Si4041[2][2]=cstate46th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate46th;


Si4142[1][1]=rcMinusstate46th;
Si4142[1][2]=-rsMinusstate46th;

Si4142[2][1]=rsMinusstate46th;
Si4142[2][2]=rcMinusstate46th;



Si3844[1][1]=cstate47th;
Si3844[1][2]=-sstate47th;

Si3844[2][1]=rcMinusHANDXROT*sstate47th;
Si3844[2][2]=cstate47th*rcMinusHANDXROT;
Si3844[2][3]=-rsMinusHANDXROT;

Si3844[3][1]=rsMinusHANDXROT*sstate47th;
Si3844[3][2]=cstate47th*rsMinusHANDXROT;
Si3844[3][3]=rcMinusHANDXROT;


Si4445[1][1]=rcstate47th;
Si4445[1][2]=-rsstate47th;

Si4445[2][1]=rsstate47th;
Si4445[2][2]=rcstate47th;


Si4546[1][1]=rcstate47th;
Si4546[1][2]=-rsstate47th;

Si4546[2][1]=rsstate47th;
Si4546[2][2]=rcstate47th;



Si3848[1][1]=cstate48th;
Si3848[1][2]=-sstate48th;

Si3848[2][1]=rcMinusHANDXROT*sstate48th;
Si3848[2][2]=cstate48th*rcMinusHANDXROT;
Si3848[2][3]=-rsMinusHANDXROT;

Si3848[3][1]=rsMinusHANDXROT*sstate48th;
Si3848[3][2]=cstate48th*rsMinusHANDXROT;
Si3848[3][3]=rcMinusHANDXROT;


Si4849[1][1]=rcstate48th;
Si4849[1][2]=-rsstate48th;

Si4849[2][1]=rsstate48th;
Si4849[2][2]=rcstate48th;


Si4950[1][1]=rcstate48th;
Si4950[1][2]=-rsstate48th;

Si4950[2][1]=rsstate48th;
Si4950[2][2]=rcstate48th;



Si3852[1][1]=cstate49th;
Si3852[1][2]=-sstate49th;

Si3852[2][1]=rcMinusHANDXROT*sstate49th;
Si3852[2][2]=cstate49th*rcMinusHANDXROT;
Si3852[2][3]=-rsMinusHANDXROT;

Si3852[3][1]=rsMinusHANDXROT*sstate49th;
Si3852[3][2]=cstate49th*rsMinusHANDXROT;
Si3852[3][3]=rcMinusHANDXROT;


Si5253[1][1]=rcstate49th;
Si5253[1][2]=-rsstate49th;

Si5253[2][1]=rsstate49th;
Si5253[2][2]=rcstate49th;


Si5354[1][1]=rcstate49th;
Si5354[1][2]=-rsstate49th;

Si5354[2][1]=rsstate49th;
Si5354[2][2]=rcstate49th;



Si3856[1][1]=cstate50th;
Si3856[1][2]=-sstate50th;

Si3856[2][1]=rcMinusHANDXROT*sstate50th;
Si3856[2][2]=cstate50th*rcMinusHANDXROT;
Si3856[2][3]=-rsMinusHANDXROT;

Si3856[3][1]=rsMinusHANDXROT*sstate50th;
Si3856[3][2]=cstate50th*rsMinusHANDXROT;
Si3856[3][3]=rcMinusHANDXROT;


Si5657[1][1]=rcstate50th;
Si5657[1][2]=-rsstate50th;

Si5657[2][1]=rsstate50th;
Si5657[2][2]=rcstate50th;


Si5758[1][1]=rcstate50th;
Si5758[1][2]=-rsstate50th;

Si5758[2][1]=rsstate50th;
Si5758[2][2]=rcstate50th;



Si360[1][1]=sstate32th;
Si360[1][2]=cstate32th;

Si360[2][1]=-cstate32th;
Si360[2][2]=sstate32th;


Si6061[2][1]=sstate33th;
Si6061[2][2]=cstate33th;

Si6061[3][1]=-cstate33th;
Si6061[3][2]=sstate33th;


Si6162[1][1]=cstate34th;
Si6162[1][2]=-sstate34th;

Si6162[3][1]=-sstate34th;
Si6162[3][2]=-cstate34th;


Si6263[1][1]=cstate35th;
Si6263[1][2]=-sstate35th;

Si6263[2][1]=sstate35th;
Si6263[2][2]=cstate35th;


Si6364[2][1]=sstate36th;
Si6364[2][2]=cstate36th;

Si6364[3][1]=cstate36th;
Si6364[3][2]=-sstate36th;



Si6266[1][1]=cstate37th;
Si6266[1][2]=-sstate37th;

Si6266[2][1]=sstate37th;
Si6266[2][2]=cstate37th;


Si6667[2][1]=sstate38th;
Si6667[2][2]=cstate38th;

Si6667[3][1]=cstate38th;
Si6667[3][2]=-sstate38th;




Si070[1][1]=-sstate23th;
Si070[1][2]=-cstate23th;

Si070[3][1]=-cstate23th;
Si070[3][2]=sstate23th;


Si7071[1][1]=sstate22th;
Si7071[1][2]=cstate22th;

Si7071[3][1]=-cstate22th;
Si7071[3][2]=sstate22th;


Si7172[1][1]=cstate24th*rcMinusTHIGHANGLEOFF;
Si7172[1][2]=-(rcMinusTHIGHANGLEOFF*sstate24th);
Si7172[1][3]=rsMinusTHIGHANGLEOFF;

Si7172[2][1]=-(cstate24th*rsMinusTHIGHANGLEOFF);
Si7172[2][2]=rsMinusTHIGHANGLEOFF*sstate24th;
Si7172[2][3]=rcMinusTHIGHANGLEOFF;

Si7172[3][1]=-sstate24th;
Si7172[3][2]=-cstate24th;


Si7273[1][1]=cstate25th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate25th;
Si7273[1][2]=-(cstate25th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate25th;

Si7273[3][1]=-(cstate25th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate25th;
Si7273[3][2]=-(cstate25th*rcTHIGHANGLEOFF) + rsTHIGHANGLEOFF*sstate25th;


Si7374[1][1]=cstate26th;
Si7374[1][2]=-sstate26th;

Si7374[3][1]=sstate26th;
Si7374[3][2]=cstate26th;


Si7475[1][1]=-sstate27th;
Si7475[1][2]=-cstate27th;

Si7475[3][1]=cstate27th;
Si7475[3][2]=-sstate27th;


Si7576[1][1]=cstate28th;
Si7576[1][2]=-sstate28th;

Si7576[3][1]=-sstate28th;
Si7576[3][2]=-cstate28th;








Si7683[1][1]=rceff3a2*rceff3a3;
Si7683[1][2]=-(rceff3a2*rseff3a3);
Si7683[1][3]=rseff3a2;

Si7683[2][1]=rceff3a3*rseff3a1*rseff3a2 + rceff3a1*rseff3a3;
Si7683[2][2]=rceff3a1*rceff3a3 - rseff3a1*rseff3a2*rseff3a3;
Si7683[2][3]=-(rceff3a2*rseff3a1);

Si7683[3][1]=-(rceff3a1*rceff3a3*rseff3a2) + rseff3a1*rseff3a3;
Si7683[3][2]=rceff3a3*rseff3a1 + rceff3a1*rseff3a2*rseff3a3;
Si7683[3][3]=rceff3a1*rceff3a2;


Si084[1][1]=sstate16th;
Si084[1][2]=cstate16th;

Si084[3][1]=-cstate16th;
Si084[3][2]=sstate16th;


Si8485[1][1]=sstate15th;
Si8485[1][2]=cstate15th;

Si8485[3][1]=cstate15th;
Si8485[3][2]=-sstate15th;


Si8586[1][1]=cstate17th*rcTHIGHANGLEOFF;
Si8586[1][2]=-(rcTHIGHANGLEOFF*sstate17th);
Si8586[1][3]=rsTHIGHANGLEOFF;

Si8586[2][1]=cstate17th*rsTHIGHANGLEOFF;
Si8586[2][2]=-(rsTHIGHANGLEOFF*sstate17th);
Si8586[2][3]=-rcTHIGHANGLEOFF;

Si8586[3][1]=sstate17th;
Si8586[3][2]=cstate17th;


Si8687[1][1]=cstate18th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate18th;
Si8687[1][2]=-(cstate18th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate18th;

Si8687[3][1]=cstate18th*rsTHIGHANGLEOFF + rcTHIGHANGLEOFF*sstate18th;
Si8687[3][2]=cstate18th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate18th;


Si8788[1][1]=cstate19th;
Si8788[1][2]=-sstate19th;

Si8788[3][1]=-sstate19th;
Si8788[3][2]=-cstate19th;


Si8889[1][1]=-sstate20th;
Si8889[1][2]=-cstate20th;

Si8889[3][1]=-cstate20th;
Si8889[3][2]=sstate20th;


Si8990[1][1]=cstate21th;
Si8990[1][2]=-sstate21th;

Si8990[3][1]=sstate21th;
Si8990[3][2]=cstate21th;








Si9097[1][1]=rceff4a2*rceff4a3;
Si9097[1][2]=-(rceff4a2*rseff4a3);
Si9097[1][3]=rseff4a2;

Si9097[2][1]=rceff4a3*rseff4a1*rseff4a2 + rceff4a1*rseff4a3;
Si9097[2][2]=rceff4a1*rceff4a3 - rseff4a1*rseff4a2*rseff4a3;
Si9097[2][3]=-(rceff4a2*rseff4a1);

Si9097[3][1]=-(rceff4a1*rceff4a3*rseff4a2) + rseff4a1*rseff4a3;
Si9097[3][2]=rceff4a3*rseff4a1 + rceff4a1*rseff4a2*rseff4a3;
Si9097[3][3]=rceff4a1*rceff4a2;



/* calculation of rotation matrices */
v[1][1]=Si6162[1][1];
v[1][2]=-Si6162[1][2];

v[3][1]=Si6162[3][1];
v[3][2]=-Si6162[3][2];


vv[1][1]=v[3][1];
vv[1][2]=v[3][2];

vv[2][1]=Si6061[2][1]*v[1][1];
vv[2][2]=Si6061[2][1]*v[1][2];
vv[2][3]=-Si6061[2][2];

vv[3][1]=Si6061[3][1]*v[1][1];
vv[3][2]=Si6061[3][1]*v[1][2];
vv[3][3]=-Si6061[3][2];


v[1][1]=Si360[1][1]*vv[1][1] + Si360[1][2]*vv[2][1];
v[1][2]=Si360[1][1]*vv[1][2] + Si360[1][2]*vv[2][2];
v[1][3]=Si360[1][2]*vv[2][3];

v[2][1]=Si360[2][1]*vv[1][1] + Si360[2][2]*vv[2][1];
v[2][2]=Si360[2][1]*vv[1][2] + Si360[2][2]*vv[2][2];
v[2][3]=Si360[2][2]*vv[2][3];

v[3][1]=vv[3][1];
v[3][2]=vv[3][2];
v[3][3]=vv[3][3];


vv[1][1]=Si23[1][1]*v[1][1] + Si23[1][2]*v[2][1];
vv[1][2]=Si23[1][1]*v[1][2] + Si23[1][2]*v[2][2];
vv[1][3]=Si23[1][1]*v[1][3] + Si23[1][2]*v[2][3];

vv[2][1]=v[3][1];
vv[2][2]=v[3][2];
vv[2][3]=v[3][3];

vv[3][1]=Si23[3][1]*v[1][1] + Si23[3][2]*v[2][1];
vv[3][2]=Si23[3][1]*v[1][2] + Si23[3][2]*v[2][2];
vv[3][3]=Si23[3][1]*v[1][3] + Si23[3][2]*v[2][3];


v[1][1]=Si12[1][1]*vv[1][1] + Si12[1][2]*vv[2][1];
v[1][2]=Si12[1][1]*vv[1][2] + Si12[1][2]*vv[2][2];
v[1][3]=Si12[1][1]*vv[1][3] + Si12[1][2]*vv[2][3];

v[2][1]=-vv[3][1];
v[2][2]=-vv[3][2];
v[2][3]=-vv[3][3];

v[3][1]=Si12[3][1]*vv[1][1] + Si12[3][2]*vv[2][1];
v[3][2]=Si12[3][1]*vv[1][2] + Si12[3][2]*vv[2][2];
v[3][3]=Si12[3][1]*vv[1][3] + Si12[3][2]*vv[2][3];


vv[1][1]=Si01[1][1]*v[1][1] + Si01[1][2]*v[2][1];
vv[1][2]=Si01[1][1]*v[1][2] + Si01[1][2]*v[2][2];
vv[1][3]=Si01[1][1]*v[1][3] + Si01[1][2]*v[2][3];

vv[2][1]=Si01[2][1]*v[1][1] + Si01[2][2]*v[2][1];
vv[2][2]=Si01[2][1]*v[1][2] + Si01[2][2]*v[2][2];
vv[2][3]=Si01[2][1]*v[1][3] + Si01[2][2]*v[2][3];

vv[3][1]=-v[3][1];
vv[3][2]=-v[3][2];
vv[3][3]=-v[3][3];


v[1][1]=Si00[1][1]*vv[1][1] + Si00[1][2]*vv[2][1] + Si00[1][3]*vv[3][1];
v[1][2]=Si00[1][1]*vv[1][2] + Si00[1][2]*vv[2][2] + Si00[1][3]*vv[3][2];
v[1][3]=Si00[1][1]*vv[1][3] + Si00[1][2]*vv[2][3] + Si00[1][3]*vv[3][3];

v[2][1]=Si00[2][1]*vv[1][1] + Si00[2][2]*vv[2][1] + Si00[2][3]*vv[3][1];
v[2][2]=Si00[2][1]*vv[1][2] + Si00[2][2]*vv[2][2] + Si00[2][3]*vv[3][2];
v[2][3]=Si00[2][1]*vv[1][3] + Si00[2][2]*vv[2][3] + Si00[2][3]*vv[3][3];

v[3][1]=Si00[3][1]*vv[1][1] + Si00[3][2]*vv[2][1] + Si00[3][3]*vv[3][1];
v[3][2]=Si00[3][1]*vv[1][2] + Si00[3][2]*vv[2][2] + Si00[3][3]*vv[3][2];
v[3][3]=Si00[3][1]*vv[1][3] + Si00[3][2]*vv[2][3] + Si00[3][3]*vv[3][3];


/* Rotation Matrix: 138 */
XRot[1][1][1]=v[1][1];
XRot[1][1][2]=v[1][2];
XRot[1][1][3]=v[1][3];

XRot[1][2][1]=v[2][1];
XRot[1][2][2]=v[2][2];
XRot[1][2][3]=v[2][3];

XRot[1][3][1]=v[3][1];
XRot[1][3][2]=v[3][2];
XRot[1][3][3]=v[3][3];


