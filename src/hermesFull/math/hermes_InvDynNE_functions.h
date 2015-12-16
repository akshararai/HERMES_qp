
void
hermes_InvDynNEfunc1(void)
     {
/* rotation matrices */
S00[1][1]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[2],2);
S00[1][2]=2*(baseo[0].q[2]*baseo[0].q[3] + baseo[0].q[1]*baseo[0].q[4]);
S00[1][3]=2*(-(baseo[0].q[1]*baseo[0].q[3]) + baseo[0].q[2]*baseo[0].q[4]);

S00[2][1]=2*(baseo[0].q[2]*baseo[0].q[3] - baseo[0].q[1]*baseo[0].q[4]);
S00[2][2]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[3],2);
S00[2][3]=2*(baseo[0].q[1]*baseo[0].q[2] + baseo[0].q[3]*baseo[0].q[4]);

S00[3][1]=2*(baseo[0].q[1]*baseo[0].q[3] + baseo[0].q[2]*baseo[0].q[4]);
S00[3][2]=2*(-(baseo[0].q[1]*baseo[0].q[2]) + baseo[0].q[3]*baseo[0].q[4]);
S00[3][3]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[4],2);


S10[1][1]=cstate29th;
S10[1][2]=-sstate29th;

S10[2][1]=-sstate29th;
S10[2][2]=-cstate29th;


S21[1][1]=-sstate30th;
S21[1][3]=cstate30th;

S21[2][1]=-cstate30th;
S21[2][3]=-sstate30th;


S32[1][1]=cstate31th;
S32[1][3]=-sstate31th;

S32[2][1]=-sstate31th;
S32[2][3]=-cstate31th;


S43[1][1]=0.7071067811865475*cstate1th;
S43[1][2]=-sstate1th;
S43[1][3]=0.7071067811865475*cstate1th;

S43[2][1]=-0.7071067811865475*sstate1th;
S43[2][2]=-cstate1th;
S43[2][3]=-0.7071067811865475*sstate1th;


S54[1][1]=-0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;
S54[1][3]=0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;

S54[2][1]=-0.7071067811865475*cstate2th + 0.7071067811865475*sstate2th;
S54[2][3]=-0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;


S65[1][1]=cstate3th;
S65[1][3]=-sstate3th;

S65[2][1]=-sstate3th;
S65[2][3]=-cstate3th;


S76[1][2]=cstate4th;
S76[1][3]=sstate4th;

S76[2][2]=-sstate4th;
S76[2][3]=cstate4th;


S87[1][1]=cstate5th;
S87[1][3]=-sstate5th;

S87[2][1]=-sstate5th;
S87[2][3]=-cstate5th;


S98[1][2]=sstate6th;
S98[1][3]=-cstate6th;

S98[2][2]=cstate6th;
S98[2][3]=sstate6th;


S109[1][1]=cstate7th;
S109[1][3]=sstate7th;

S109[2][1]=-sstate7th;
S109[2][3]=cstate7th;


S1110[1][1]=rceff2a2*rceff2a3;
S1110[1][2]=rceff2a3*rseff2a1*rseff2a2 + rceff2a1*rseff2a3;
S1110[1][3]=-(rceff2a1*rceff2a3*rseff2a2) + rseff2a1*rseff2a3;

S1110[2][1]=-(rceff2a2*rseff2a3);
S1110[2][2]=rceff2a1*rceff2a3 - rseff2a1*rseff2a2*rseff2a3;
S1110[2][3]=rceff2a3*rseff2a1 + rceff2a1*rseff2a2*rseff2a3;

S1110[3][1]=rseff2a2;
S1110[3][2]=-(rceff2a2*rseff2a1);
S1110[3][3]=rceff2a1*rceff2a2;


S1210[1][1]=rcTHUMBZROT;
S1210[1][2]=rcHANDXROT*rsTHUMBZROT;
S1210[1][3]=rsHANDXROT*rsTHUMBZROT;

S1210[2][1]=-(cstate39th*rsTHUMBZROT);
S1210[2][2]=cstate39th*rcHANDXROT*rcTHUMBZROT - rsHANDXROT*sstate39th;
S1210[2][3]=cstate39th*rcTHUMBZROT*rsHANDXROT + rcHANDXROT*sstate39th;

S1210[3][1]=rsTHUMBZROT*sstate39th;
S1210[3][2]=-(cstate39th*rsHANDXROT) - rcHANDXROT*rcTHUMBZROT*sstate39th;
S1210[3][3]=cstate39th*rcHANDXROT - rcTHUMBZROT*rsHANDXROT*sstate39th;


S1312[1][1]=cstate40th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate40th;
S1312[1][2]=cstate40th*rsTHUMBANGLEOFF - rcTHUMBANGLEOFF*sstate40th;

S1312[2][1]=-(cstate40th*rsTHUMBANGLEOFF) + rcTHUMBANGLEOFF*sstate40th;
S1312[2][2]=cstate40th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate40th;


S1413[1][1]=rcMinusstate40th;
S1413[1][2]=rsMinusstate40th;

S1413[2][1]=-rsMinusstate40th;
S1413[2][2]=rcMinusstate40th;



S1610[1][1]=cstate41th;
S1610[1][2]=rcHANDXROT*sstate41th;
S1610[1][3]=rsHANDXROT*sstate41th;

S1610[2][1]=-sstate41th;
S1610[2][2]=cstate41th*rcHANDXROT;
S1610[2][3]=cstate41th*rsHANDXROT;

S1610[3][2]=-rsHANDXROT;
S1610[3][3]=rcHANDXROT;


S1716[1][1]=rcstate41th;
S1716[1][2]=rsstate41th;

S1716[2][1]=-rsstate41th;
S1716[2][2]=rcstate41th;


S1817[1][1]=rcstate41th;
S1817[1][2]=rsstate41th;

S1817[2][1]=-rsstate41th;
S1817[2][2]=rcstate41th;



S2010[1][1]=cstate42th;
S2010[1][2]=rcHANDXROT*sstate42th;
S2010[1][3]=rsHANDXROT*sstate42th;

S2010[2][1]=-sstate42th;
S2010[2][2]=cstate42th*rcHANDXROT;
S2010[2][3]=cstate42th*rsHANDXROT;

S2010[3][2]=-rsHANDXROT;
S2010[3][3]=rcHANDXROT;


S2120[1][1]=rcstate42th;
S2120[1][2]=rsstate42th;

S2120[2][1]=-rsstate42th;
S2120[2][2]=rcstate42th;


S2221[1][1]=rcstate42th;
S2221[1][2]=rsstate42th;

S2221[2][1]=-rsstate42th;
S2221[2][2]=rcstate42th;



S2410[1][1]=cstate43th;
S2410[1][2]=rcHANDXROT*sstate43th;
S2410[1][3]=rsHANDXROT*sstate43th;

S2410[2][1]=-sstate43th;
S2410[2][2]=cstate43th*rcHANDXROT;
S2410[2][3]=cstate43th*rsHANDXROT;

S2410[3][2]=-rsHANDXROT;
S2410[3][3]=rcHANDXROT;


S2524[1][1]=rcstate43th;
S2524[1][2]=rsstate43th;

S2524[2][1]=-rsstate43th;
S2524[2][2]=rcstate43th;


S2625[1][1]=rcstate43th;
S2625[1][2]=rsstate43th;

S2625[2][1]=-rsstate43th;
S2625[2][2]=rcstate43th;



S2810[1][1]=cstate44th;
S2810[1][2]=rcHANDXROT*sstate44th;
S2810[1][3]=rsHANDXROT*sstate44th;

S2810[2][1]=-sstate44th;
S2810[2][2]=cstate44th*rcHANDXROT;
S2810[2][3]=cstate44th*rsHANDXROT;

S2810[3][2]=-rsHANDXROT;
S2810[3][3]=rcHANDXROT;


S2928[1][1]=rcstate44th;
S2928[1][2]=rsstate44th;

S2928[2][1]=-rsstate44th;
S2928[2][2]=rcstate44th;


S3029[1][1]=rcstate44th;
S3029[1][2]=rsstate44th;

S3029[2][1]=-rsstate44th;
S3029[2][2]=rcstate44th;



S323[1][1]=0.7071067811865475*cstate8th;
S323[1][2]=-sstate8th;
S323[1][3]=-0.7071067811865475*cstate8th;

S323[2][1]=-0.7071067811865475*sstate8th;
S323[2][2]=-cstate8th;
S323[2][3]=0.7071067811865475*sstate8th;


S3332[1][1]=-0.7071067811865475*cstate9th - 0.7071067811865475*sstate9th;
S3332[1][3]=-0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;

S3332[2][1]=-0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;
S3332[2][3]=0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;


S3433[1][1]=cstate10th;
S3433[1][3]=sstate10th;

S3433[2][1]=-sstate10th;
S3433[2][3]=cstate10th;


S3534[1][2]=cstate11th;
S3534[1][3]=-sstate11th;

S3534[2][2]=-sstate11th;
S3534[2][3]=-cstate11th;


S3635[1][1]=cstate12th;
S3635[1][3]=sstate12th;

S3635[2][1]=-sstate12th;
S3635[2][3]=cstate12th;


S3736[1][2]=sstate13th;
S3736[1][3]=cstate13th;

S3736[2][2]=cstate13th;
S3736[2][3]=-sstate13th;


S3837[1][1]=cstate14th;
S3837[1][3]=-sstate14th;

S3837[2][1]=-sstate14th;
S3837[2][3]=-cstate14th;


S3938[1][1]=rceff1a2*rceff1a3;
S3938[1][2]=rceff1a3*rseff1a1*rseff1a2 + rceff1a1*rseff1a3;
S3938[1][3]=-(rceff1a1*rceff1a3*rseff1a2) + rseff1a1*rseff1a3;

S3938[2][1]=-(rceff1a2*rseff1a3);
S3938[2][2]=rceff1a1*rceff1a3 - rseff1a1*rseff1a2*rseff1a3;
S3938[2][3]=rceff1a3*rseff1a1 + rceff1a1*rseff1a2*rseff1a3;

S3938[3][1]=rseff1a2;
S3938[3][2]=-(rceff1a2*rseff1a1);
S3938[3][3]=rceff1a1*rceff1a2;


S4038[1][1]=rcTHUMBZROT;
S4038[1][2]=rcMinusHANDXROT*rsTHUMBZROT;
S4038[1][3]=rsMinusHANDXROT*rsTHUMBZROT;

S4038[2][1]=-(cstate45th*rsTHUMBZROT);
S4038[2][2]=cstate45th*rcMinusHANDXROT*rcTHUMBZROT - rsMinusHANDXROT*sstate45th;
S4038[2][3]=cstate45th*rcTHUMBZROT*rsMinusHANDXROT + rcMinusHANDXROT*sstate45th;

S4038[3][1]=rsTHUMBZROT*sstate45th;
S4038[3][2]=-(cstate45th*rsMinusHANDXROT) - rcMinusHANDXROT*rcTHUMBZROT*sstate45th;
S4038[3][3]=cstate45th*rcMinusHANDXROT - rcTHUMBZROT*rsMinusHANDXROT*sstate45th;


S4140[1][1]=cstate46th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate46th;
S4140[1][2]=cstate46th*rsTHUMBANGLEOFF - rcTHUMBANGLEOFF*sstate46th;

S4140[2][1]=-(cstate46th*rsTHUMBANGLEOFF) + rcTHUMBANGLEOFF*sstate46th;
S4140[2][2]=cstate46th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate46th;


S4241[1][1]=rcMinusstate46th;
S4241[1][2]=rsMinusstate46th;

S4241[2][1]=-rsMinusstate46th;
S4241[2][2]=rcMinusstate46th;



S4438[1][1]=cstate47th;
S4438[1][2]=rcMinusHANDXROT*sstate47th;
S4438[1][3]=rsMinusHANDXROT*sstate47th;

S4438[2][1]=-sstate47th;
S4438[2][2]=cstate47th*rcMinusHANDXROT;
S4438[2][3]=cstate47th*rsMinusHANDXROT;

S4438[3][2]=-rsMinusHANDXROT;
S4438[3][3]=rcMinusHANDXROT;


S4544[1][1]=rcstate47th;
S4544[1][2]=rsstate47th;

S4544[2][1]=-rsstate47th;
S4544[2][2]=rcstate47th;


S4645[1][1]=rcstate47th;
S4645[1][2]=rsstate47th;

S4645[2][1]=-rsstate47th;
S4645[2][2]=rcstate47th;



S4838[1][1]=cstate48th;
S4838[1][2]=rcMinusHANDXROT*sstate48th;
S4838[1][3]=rsMinusHANDXROT*sstate48th;

S4838[2][1]=-sstate48th;
S4838[2][2]=cstate48th*rcMinusHANDXROT;
S4838[2][3]=cstate48th*rsMinusHANDXROT;

S4838[3][2]=-rsMinusHANDXROT;
S4838[3][3]=rcMinusHANDXROT;


S4948[1][1]=rcstate48th;
S4948[1][2]=rsstate48th;

S4948[2][1]=-rsstate48th;
S4948[2][2]=rcstate48th;


S5049[1][1]=rcstate48th;
S5049[1][2]=rsstate48th;

S5049[2][1]=-rsstate48th;
S5049[2][2]=rcstate48th;



S5238[1][1]=cstate49th;
S5238[1][2]=rcMinusHANDXROT*sstate49th;
S5238[1][3]=rsMinusHANDXROT*sstate49th;

S5238[2][1]=-sstate49th;
S5238[2][2]=cstate49th*rcMinusHANDXROT;
S5238[2][3]=cstate49th*rsMinusHANDXROT;

S5238[3][2]=-rsMinusHANDXROT;
S5238[3][3]=rcMinusHANDXROT;


S5352[1][1]=rcstate49th;
S5352[1][2]=rsstate49th;

S5352[2][1]=-rsstate49th;
S5352[2][2]=rcstate49th;


S5453[1][1]=rcstate49th;
S5453[1][2]=rsstate49th;

S5453[2][1]=-rsstate49th;
S5453[2][2]=rcstate49th;



S5638[1][1]=cstate50th;
S5638[1][2]=rcMinusHANDXROT*sstate50th;
S5638[1][3]=rsMinusHANDXROT*sstate50th;

S5638[2][1]=-sstate50th;
S5638[2][2]=cstate50th*rcMinusHANDXROT;
S5638[2][3]=cstate50th*rsMinusHANDXROT;

S5638[3][2]=-rsMinusHANDXROT;
S5638[3][3]=rcMinusHANDXROT;


S5756[1][1]=rcstate50th;
S5756[1][2]=rsstate50th;

S5756[2][1]=-rsstate50th;
S5756[2][2]=rcstate50th;


S5857[1][1]=rcstate50th;
S5857[1][2]=rsstate50th;

S5857[2][1]=-rsstate50th;
S5857[2][2]=rcstate50th;



S603[1][1]=sstate32th;
S603[1][2]=-cstate32th;

S603[2][1]=cstate32th;
S603[2][2]=sstate32th;


S6160[1][2]=sstate33th;
S6160[1][3]=-cstate33th;

S6160[2][2]=cstate33th;
S6160[2][3]=sstate33th;


S6261[1][1]=cstate34th;
S6261[1][3]=-sstate34th;

S6261[2][1]=-sstate34th;
S6261[2][3]=-cstate34th;


S6362[1][1]=cstate35th;
S6362[1][2]=sstate35th;

S6362[2][1]=-sstate35th;
S6362[2][2]=cstate35th;


S6463[1][2]=sstate36th;
S6463[1][3]=cstate36th;

S6463[2][2]=cstate36th;
S6463[2][3]=-sstate36th;



S6662[1][1]=cstate37th;
S6662[1][2]=sstate37th;

S6662[2][1]=-sstate37th;
S6662[2][2]=cstate37th;


S6766[1][2]=sstate38th;
S6766[1][3]=cstate38th;

S6766[2][2]=cstate38th;
S6766[2][3]=-sstate38th;




S700[1][1]=-sstate23th;
S700[1][3]=-cstate23th;

S700[2][1]=-cstate23th;
S700[2][3]=sstate23th;


S7170[1][1]=sstate22th;
S7170[1][3]=-cstate22th;

S7170[2][1]=cstate22th;
S7170[2][3]=sstate22th;


S7271[1][1]=cstate24th*rcMinusTHIGHANGLEOFF;
S7271[1][2]=-(cstate24th*rsMinusTHIGHANGLEOFF);
S7271[1][3]=-sstate24th;

S7271[2][1]=-(rcMinusTHIGHANGLEOFF*sstate24th);
S7271[2][2]=rsMinusTHIGHANGLEOFF*sstate24th;
S7271[2][3]=-cstate24th;

S7271[3][1]=rsMinusTHIGHANGLEOFF;
S7271[3][2]=rcMinusTHIGHANGLEOFF;


S7372[1][1]=cstate25th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate25th;
S7372[1][3]=-(cstate25th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate25th;

S7372[2][1]=-(cstate25th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate25th;
S7372[2][3]=-(cstate25th*rcTHIGHANGLEOFF) + rsTHIGHANGLEOFF*sstate25th;


S7473[1][1]=cstate26th;
S7473[1][3]=sstate26th;

S7473[2][1]=-sstate26th;
S7473[2][3]=cstate26th;


S7574[1][1]=-sstate27th;
S7574[1][3]=cstate27th;

S7574[2][1]=-cstate27th;
S7574[2][3]=-sstate27th;


S7675[1][1]=cstate28th;
S7675[1][3]=-sstate28th;

S7675[2][1]=-sstate28th;
S7675[2][3]=-cstate28th;








S8376[1][1]=rceff3a2*rceff3a3;
S8376[1][2]=rceff3a3*rseff3a1*rseff3a2 + rceff3a1*rseff3a3;
S8376[1][3]=-(rceff3a1*rceff3a3*rseff3a2) + rseff3a1*rseff3a3;

S8376[2][1]=-(rceff3a2*rseff3a3);
S8376[2][2]=rceff3a1*rceff3a3 - rseff3a1*rseff3a2*rseff3a3;
S8376[2][3]=rceff3a3*rseff3a1 + rceff3a1*rseff3a2*rseff3a3;

S8376[3][1]=rseff3a2;
S8376[3][2]=-(rceff3a2*rseff3a1);
S8376[3][3]=rceff3a1*rceff3a2;


S840[1][1]=sstate16th;
S840[1][3]=-cstate16th;

S840[2][1]=cstate16th;
S840[2][3]=sstate16th;


S8584[1][1]=sstate15th;
S8584[1][3]=cstate15th;

S8584[2][1]=cstate15th;
S8584[2][3]=-sstate15th;


S8685[1][1]=cstate17th*rcTHIGHANGLEOFF;
S8685[1][2]=cstate17th*rsTHIGHANGLEOFF;
S8685[1][3]=sstate17th;

S8685[2][1]=-(rcTHIGHANGLEOFF*sstate17th);
S8685[2][2]=-(rsTHIGHANGLEOFF*sstate17th);
S8685[2][3]=cstate17th;

S8685[3][1]=rsTHIGHANGLEOFF;
S8685[3][2]=-rcTHIGHANGLEOFF;


S8786[1][1]=cstate18th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate18th;
S8786[1][3]=cstate18th*rsTHIGHANGLEOFF + rcTHIGHANGLEOFF*sstate18th;

S8786[2][1]=-(cstate18th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate18th;
S8786[2][3]=cstate18th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate18th;


S8887[1][1]=cstate19th;
S8887[1][3]=-sstate19th;

S8887[2][1]=-sstate19th;
S8887[2][3]=-cstate19th;


S8988[1][1]=-sstate20th;
S8988[1][3]=-cstate20th;

S8988[2][1]=-cstate20th;
S8988[2][3]=sstate20th;


S9089[1][1]=cstate21th;
S9089[1][3]=sstate21th;

S9089[2][1]=-sstate21th;
S9089[2][3]=cstate21th;








S9790[1][1]=rceff4a2*rceff4a3;
S9790[1][2]=rceff4a3*rseff4a1*rseff4a2 + rceff4a1*rseff4a3;
S9790[1][3]=-(rceff4a1*rceff4a3*rseff4a2) + rseff4a1*rseff4a3;

S9790[2][1]=-(rceff4a2*rseff4a3);
S9790[2][2]=rceff4a1*rceff4a3 - rseff4a1*rseff4a2*rseff4a3;
S9790[2][3]=rceff4a3*rseff4a1 + rceff4a1*rseff4a2*rseff4a3;

S9790[3][1]=rseff4a2;
S9790[3][2]=-(rceff4a2*rseff4a1);
S9790[3][3]=rceff4a1*rceff4a2;




}


void
hermes_InvDynNEfunc2(void)
     {
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




}


void
hermes_InvDynNEfunc3(void)
     {
/* rotation matrices from global to link coordinates */
SG10[1][1]=S00[1][1]*S10[1][1] + S00[2][1]*S10[1][2];
SG10[1][2]=S00[1][2]*S10[1][1] + S00[2][2]*S10[1][2];
SG10[1][3]=S00[1][3]*S10[1][1] + S00[2][3]*S10[1][2];

SG10[2][1]=S00[1][1]*S10[2][1] + S00[2][1]*S10[2][2];
SG10[2][2]=S00[1][2]*S10[2][1] + S00[2][2]*S10[2][2];
SG10[2][3]=S00[1][3]*S10[2][1] + S00[2][3]*S10[2][2];

SG10[3][1]=-S00[3][1];
SG10[3][2]=-S00[3][2];
SG10[3][3]=-S00[3][3];


SG20[1][1]=S21[1][1]*SG10[1][1] + S21[1][3]*SG10[3][1];
SG20[1][2]=S21[1][1]*SG10[1][2] + S21[1][3]*SG10[3][2];
SG20[1][3]=S21[1][1]*SG10[1][3] + S21[1][3]*SG10[3][3];

SG20[2][1]=S21[2][1]*SG10[1][1] + S21[2][3]*SG10[3][1];
SG20[2][2]=S21[2][1]*SG10[1][2] + S21[2][3]*SG10[3][2];
SG20[2][3]=S21[2][1]*SG10[1][3] + S21[2][3]*SG10[3][3];

SG20[3][1]=-SG10[2][1];
SG20[3][2]=-SG10[2][2];
SG20[3][3]=-SG10[2][3];


SG30[1][1]=S32[1][1]*SG20[1][1] + S32[1][3]*SG20[3][1];
SG30[1][2]=S32[1][1]*SG20[1][2] + S32[1][3]*SG20[3][2];
SG30[1][3]=S32[1][1]*SG20[1][3] + S32[1][3]*SG20[3][3];

SG30[2][1]=S32[2][1]*SG20[1][1] + S32[2][3]*SG20[3][1];
SG30[2][2]=S32[2][1]*SG20[1][2] + S32[2][3]*SG20[3][2];
SG30[2][3]=S32[2][1]*SG20[1][3] + S32[2][3]*SG20[3][3];

SG30[3][1]=SG20[2][1];
SG30[3][2]=SG20[2][2];
SG30[3][3]=SG20[2][3];


SG40[1][1]=S43[1][1]*SG30[1][1] + S43[1][2]*SG30[2][1] + S43[1][3]*SG30[3][1];
SG40[1][2]=S43[1][1]*SG30[1][2] + S43[1][2]*SG30[2][2] + S43[1][3]*SG30[3][2];
SG40[1][3]=S43[1][1]*SG30[1][3] + S43[1][2]*SG30[2][3] + S43[1][3]*SG30[3][3];

SG40[2][1]=S43[2][1]*SG30[1][1] + S43[2][2]*SG30[2][1] + S43[2][3]*SG30[3][1];
SG40[2][2]=S43[2][1]*SG30[1][2] + S43[2][2]*SG30[2][2] + S43[2][3]*SG30[3][2];
SG40[2][3]=S43[2][1]*SG30[1][3] + S43[2][2]*SG30[2][3] + S43[2][3]*SG30[3][3];

SG40[3][1]=0.7071067811865475*SG30[1][1] - 0.7071067811865475*SG30[3][1];
SG40[3][2]=0.7071067811865475*SG30[1][2] - 0.7071067811865475*SG30[3][2];
SG40[3][3]=0.7071067811865475*SG30[1][3] - 0.7071067811865475*SG30[3][3];


SG50[1][1]=S54[1][1]*SG40[1][1] + S54[1][3]*SG40[3][1];
SG50[1][2]=S54[1][1]*SG40[1][2] + S54[1][3]*SG40[3][2];
SG50[1][3]=S54[1][1]*SG40[1][3] + S54[1][3]*SG40[3][3];

SG50[2][1]=S54[2][1]*SG40[1][1] + S54[2][3]*SG40[3][1];
SG50[2][2]=S54[2][1]*SG40[1][2] + S54[2][3]*SG40[3][2];
SG50[2][3]=S54[2][1]*SG40[1][3] + S54[2][3]*SG40[3][3];

SG50[3][1]=-SG40[2][1];
SG50[3][2]=-SG40[2][2];
SG50[3][3]=-SG40[2][3];


SG60[1][1]=S65[1][1]*SG50[1][1] + S65[1][3]*SG50[3][1];
SG60[1][2]=S65[1][1]*SG50[1][2] + S65[1][3]*SG50[3][2];
SG60[1][3]=S65[1][1]*SG50[1][3] + S65[1][3]*SG50[3][3];

SG60[2][1]=S65[2][1]*SG50[1][1] + S65[2][3]*SG50[3][1];
SG60[2][2]=S65[2][1]*SG50[1][2] + S65[2][3]*SG50[3][2];
SG60[2][3]=S65[2][1]*SG50[1][3] + S65[2][3]*SG50[3][3];

SG60[3][1]=SG50[2][1];
SG60[3][2]=SG50[2][2];
SG60[3][3]=SG50[2][3];


SG70[1][1]=S76[1][2]*SG60[2][1] + S76[1][3]*SG60[3][1];
SG70[1][2]=S76[1][2]*SG60[2][2] + S76[1][3]*SG60[3][2];
SG70[1][3]=S76[1][2]*SG60[2][3] + S76[1][3]*SG60[3][3];

SG70[2][1]=S76[2][2]*SG60[2][1] + S76[2][3]*SG60[3][1];
SG70[2][2]=S76[2][2]*SG60[2][2] + S76[2][3]*SG60[3][2];
SG70[2][3]=S76[2][2]*SG60[2][3] + S76[2][3]*SG60[3][3];

SG70[3][1]=SG60[1][1];
SG70[3][2]=SG60[1][2];
SG70[3][3]=SG60[1][3];


SG80[1][1]=S87[1][1]*SG70[1][1] + S87[1][3]*SG70[3][1];
SG80[1][2]=S87[1][1]*SG70[1][2] + S87[1][3]*SG70[3][2];
SG80[1][3]=S87[1][1]*SG70[1][3] + S87[1][3]*SG70[3][3];

SG80[2][1]=S87[2][1]*SG70[1][1] + S87[2][3]*SG70[3][1];
SG80[2][2]=S87[2][1]*SG70[1][2] + S87[2][3]*SG70[3][2];
SG80[2][3]=S87[2][1]*SG70[1][3] + S87[2][3]*SG70[3][3];

SG80[3][1]=SG70[2][1];
SG80[3][2]=SG70[2][2];
SG80[3][3]=SG70[2][3];


SG90[1][1]=S98[1][2]*SG80[2][1] + S98[1][3]*SG80[3][1];
SG90[1][2]=S98[1][2]*SG80[2][2] + S98[1][3]*SG80[3][2];
SG90[1][3]=S98[1][2]*SG80[2][3] + S98[1][3]*SG80[3][3];

SG90[2][1]=S98[2][2]*SG80[2][1] + S98[2][3]*SG80[3][1];
SG90[2][2]=S98[2][2]*SG80[2][2] + S98[2][3]*SG80[3][2];
SG90[2][3]=S98[2][2]*SG80[2][3] + S98[2][3]*SG80[3][3];

SG90[3][1]=SG80[1][1];
SG90[3][2]=SG80[1][2];
SG90[3][3]=SG80[1][3];


SG100[1][1]=S109[1][1]*SG90[1][1] + S109[1][3]*SG90[3][1];
SG100[1][2]=S109[1][1]*SG90[1][2] + S109[1][3]*SG90[3][2];
SG100[1][3]=S109[1][1]*SG90[1][3] + S109[1][3]*SG90[3][3];

SG100[2][1]=S109[2][1]*SG90[1][1] + S109[2][3]*SG90[3][1];
SG100[2][2]=S109[2][1]*SG90[1][2] + S109[2][3]*SG90[3][2];
SG100[2][3]=S109[2][1]*SG90[1][3] + S109[2][3]*SG90[3][3];

SG100[3][1]=-SG90[2][1];
SG100[3][2]=-SG90[2][2];
SG100[3][3]=-SG90[2][3];


SG110[1][1]=S1110[1][1]*SG100[1][1] + S1110[1][2]*SG100[2][1] + S1110[1][3]*SG100[3][1];
SG110[1][2]=S1110[1][1]*SG100[1][2] + S1110[1][2]*SG100[2][2] + S1110[1][3]*SG100[3][2];
SG110[1][3]=S1110[1][1]*SG100[1][3] + S1110[1][2]*SG100[2][3] + S1110[1][3]*SG100[3][3];

SG110[2][1]=S1110[2][1]*SG100[1][1] + S1110[2][2]*SG100[2][1] + S1110[2][3]*SG100[3][1];
SG110[2][2]=S1110[2][1]*SG100[1][2] + S1110[2][2]*SG100[2][2] + S1110[2][3]*SG100[3][2];
SG110[2][3]=S1110[2][1]*SG100[1][3] + S1110[2][2]*SG100[2][3] + S1110[2][3]*SG100[3][3];

SG110[3][1]=S1110[3][1]*SG100[1][1] + S1110[3][2]*SG100[2][1] + S1110[3][3]*SG100[3][1];
SG110[3][2]=S1110[3][1]*SG100[1][2] + S1110[3][2]*SG100[2][2] + S1110[3][3]*SG100[3][2];
SG110[3][3]=S1110[3][1]*SG100[1][3] + S1110[3][2]*SG100[2][3] + S1110[3][3]*SG100[3][3];


SG120[1][1]=S1210[1][1]*SG100[1][1] + S1210[1][2]*SG100[2][1] + S1210[1][3]*SG100[3][1];
SG120[1][2]=S1210[1][1]*SG100[1][2] + S1210[1][2]*SG100[2][2] + S1210[1][3]*SG100[3][2];
SG120[1][3]=S1210[1][1]*SG100[1][3] + S1210[1][2]*SG100[2][3] + S1210[1][3]*SG100[3][3];

SG120[2][1]=S1210[2][1]*SG100[1][1] + S1210[2][2]*SG100[2][1] + S1210[2][3]*SG100[3][1];
SG120[2][2]=S1210[2][1]*SG100[1][2] + S1210[2][2]*SG100[2][2] + S1210[2][3]*SG100[3][2];
SG120[2][3]=S1210[2][1]*SG100[1][3] + S1210[2][2]*SG100[2][3] + S1210[2][3]*SG100[3][3];

SG120[3][1]=S1210[3][1]*SG100[1][1] + S1210[3][2]*SG100[2][1] + S1210[3][3]*SG100[3][1];
SG120[3][2]=S1210[3][1]*SG100[1][2] + S1210[3][2]*SG100[2][2] + S1210[3][3]*SG100[3][2];
SG120[3][3]=S1210[3][1]*SG100[1][3] + S1210[3][2]*SG100[2][3] + S1210[3][3]*SG100[3][3];


SG130[1][1]=S1312[1][1]*SG120[1][1] + S1312[1][2]*SG120[2][1];
SG130[1][2]=S1312[1][1]*SG120[1][2] + S1312[1][2]*SG120[2][2];
SG130[1][3]=S1312[1][1]*SG120[1][3] + S1312[1][2]*SG120[2][3];

SG130[2][1]=S1312[2][1]*SG120[1][1] + S1312[2][2]*SG120[2][1];
SG130[2][2]=S1312[2][1]*SG120[1][2] + S1312[2][2]*SG120[2][2];
SG130[2][3]=S1312[2][1]*SG120[1][3] + S1312[2][2]*SG120[2][3];

SG130[3][1]=SG120[3][1];
SG130[3][2]=SG120[3][2];
SG130[3][3]=SG120[3][3];


SG140[1][1]=S1413[1][1]*SG130[1][1] + S1413[1][2]*SG130[2][1];
SG140[1][2]=S1413[1][1]*SG130[1][2] + S1413[1][2]*SG130[2][2];
SG140[1][3]=S1413[1][1]*SG130[1][3] + S1413[1][2]*SG130[2][3];

SG140[2][1]=S1413[2][1]*SG130[1][1] + S1413[2][2]*SG130[2][1];
SG140[2][2]=S1413[2][1]*SG130[1][2] + S1413[2][2]*SG130[2][2];
SG140[2][3]=S1413[2][1]*SG130[1][3] + S1413[2][2]*SG130[2][3];

SG140[3][1]=SG130[3][1];
SG140[3][2]=SG130[3][2];
SG140[3][3]=SG130[3][3];


SG150[1][1]=SG140[1][1];
SG150[1][2]=SG140[1][2];
SG150[1][3]=SG140[1][3];

SG150[2][1]=SG140[2][1];
SG150[2][2]=SG140[2][2];
SG150[2][3]=SG140[2][3];

SG150[3][1]=SG140[3][1];
SG150[3][2]=SG140[3][2];
SG150[3][3]=SG140[3][3];


SG160[1][1]=S1610[1][1]*SG100[1][1] + S1610[1][2]*SG100[2][1] + S1610[1][3]*SG100[3][1];
SG160[1][2]=S1610[1][1]*SG100[1][2] + S1610[1][2]*SG100[2][2] + S1610[1][3]*SG100[3][2];
SG160[1][3]=S1610[1][1]*SG100[1][3] + S1610[1][2]*SG100[2][3] + S1610[1][3]*SG100[3][3];

SG160[2][1]=S1610[2][1]*SG100[1][1] + S1610[2][2]*SG100[2][1] + S1610[2][3]*SG100[3][1];
SG160[2][2]=S1610[2][1]*SG100[1][2] + S1610[2][2]*SG100[2][2] + S1610[2][3]*SG100[3][2];
SG160[2][3]=S1610[2][1]*SG100[1][3] + S1610[2][2]*SG100[2][3] + S1610[2][3]*SG100[3][3];

SG160[3][1]=S1610[3][2]*SG100[2][1] + S1610[3][3]*SG100[3][1];
SG160[3][2]=S1610[3][2]*SG100[2][2] + S1610[3][3]*SG100[3][2];
SG160[3][3]=S1610[3][2]*SG100[2][3] + S1610[3][3]*SG100[3][3];


SG170[1][1]=S1716[1][1]*SG160[1][1] + S1716[1][2]*SG160[2][1];
SG170[1][2]=S1716[1][1]*SG160[1][2] + S1716[1][2]*SG160[2][2];
SG170[1][3]=S1716[1][1]*SG160[1][3] + S1716[1][2]*SG160[2][3];

SG170[2][1]=S1716[2][1]*SG160[1][1] + S1716[2][2]*SG160[2][1];
SG170[2][2]=S1716[2][1]*SG160[1][2] + S1716[2][2]*SG160[2][2];
SG170[2][3]=S1716[2][1]*SG160[1][3] + S1716[2][2]*SG160[2][3];

SG170[3][1]=SG160[3][1];
SG170[3][2]=SG160[3][2];
SG170[3][3]=SG160[3][3];


SG180[1][1]=S1817[1][1]*SG170[1][1] + S1817[1][2]*SG170[2][1];
SG180[1][2]=S1817[1][1]*SG170[1][2] + S1817[1][2]*SG170[2][2];
SG180[1][3]=S1817[1][1]*SG170[1][3] + S1817[1][2]*SG170[2][3];

SG180[2][1]=S1817[2][1]*SG170[1][1] + S1817[2][2]*SG170[2][1];
SG180[2][2]=S1817[2][1]*SG170[1][2] + S1817[2][2]*SG170[2][2];
SG180[2][3]=S1817[2][1]*SG170[1][3] + S1817[2][2]*SG170[2][3];

SG180[3][1]=SG170[3][1];
SG180[3][2]=SG170[3][2];
SG180[3][3]=SG170[3][3];


SG190[1][1]=SG180[1][1];
SG190[1][2]=SG180[1][2];
SG190[1][3]=SG180[1][3];

SG190[2][1]=SG180[2][1];
SG190[2][2]=SG180[2][2];
SG190[2][3]=SG180[2][3];

SG190[3][1]=SG180[3][1];
SG190[3][2]=SG180[3][2];
SG190[3][3]=SG180[3][3];


SG200[1][1]=S2010[1][1]*SG100[1][1] + S2010[1][2]*SG100[2][1] + S2010[1][3]*SG100[3][1];
SG200[1][2]=S2010[1][1]*SG100[1][2] + S2010[1][2]*SG100[2][2] + S2010[1][3]*SG100[3][2];
SG200[1][3]=S2010[1][1]*SG100[1][3] + S2010[1][2]*SG100[2][3] + S2010[1][3]*SG100[3][3];

SG200[2][1]=S2010[2][1]*SG100[1][1] + S2010[2][2]*SG100[2][1] + S2010[2][3]*SG100[3][1];
SG200[2][2]=S2010[2][1]*SG100[1][2] + S2010[2][2]*SG100[2][2] + S2010[2][3]*SG100[3][2];
SG200[2][3]=S2010[2][1]*SG100[1][3] + S2010[2][2]*SG100[2][3] + S2010[2][3]*SG100[3][3];

SG200[3][1]=S2010[3][2]*SG100[2][1] + S2010[3][3]*SG100[3][1];
SG200[3][2]=S2010[3][2]*SG100[2][2] + S2010[3][3]*SG100[3][2];
SG200[3][3]=S2010[3][2]*SG100[2][3] + S2010[3][3]*SG100[3][3];


SG210[1][1]=S2120[1][1]*SG200[1][1] + S2120[1][2]*SG200[2][1];
SG210[1][2]=S2120[1][1]*SG200[1][2] + S2120[1][2]*SG200[2][2];
SG210[1][3]=S2120[1][1]*SG200[1][3] + S2120[1][2]*SG200[2][3];

SG210[2][1]=S2120[2][1]*SG200[1][1] + S2120[2][2]*SG200[2][1];
SG210[2][2]=S2120[2][1]*SG200[1][2] + S2120[2][2]*SG200[2][2];
SG210[2][3]=S2120[2][1]*SG200[1][3] + S2120[2][2]*SG200[2][3];

SG210[3][1]=SG200[3][1];
SG210[3][2]=SG200[3][2];
SG210[3][3]=SG200[3][3];


SG220[1][1]=S2221[1][1]*SG210[1][1] + S2221[1][2]*SG210[2][1];
SG220[1][2]=S2221[1][1]*SG210[1][2] + S2221[1][2]*SG210[2][2];
SG220[1][3]=S2221[1][1]*SG210[1][3] + S2221[1][2]*SG210[2][3];

SG220[2][1]=S2221[2][1]*SG210[1][1] + S2221[2][2]*SG210[2][1];
SG220[2][2]=S2221[2][1]*SG210[1][2] + S2221[2][2]*SG210[2][2];
SG220[2][3]=S2221[2][1]*SG210[1][3] + S2221[2][2]*SG210[2][3];

SG220[3][1]=SG210[3][1];
SG220[3][2]=SG210[3][2];
SG220[3][3]=SG210[3][3];


SG230[1][1]=SG220[1][1];
SG230[1][2]=SG220[1][2];
SG230[1][3]=SG220[1][3];

SG230[2][1]=SG220[2][1];
SG230[2][2]=SG220[2][2];
SG230[2][3]=SG220[2][3];

SG230[3][1]=SG220[3][1];
SG230[3][2]=SG220[3][2];
SG230[3][3]=SG220[3][3];


SG240[1][1]=S2410[1][1]*SG100[1][1] + S2410[1][2]*SG100[2][1] + S2410[1][3]*SG100[3][1];
SG240[1][2]=S2410[1][1]*SG100[1][2] + S2410[1][2]*SG100[2][2] + S2410[1][3]*SG100[3][2];
SG240[1][3]=S2410[1][1]*SG100[1][3] + S2410[1][2]*SG100[2][3] + S2410[1][3]*SG100[3][3];

SG240[2][1]=S2410[2][1]*SG100[1][1] + S2410[2][2]*SG100[2][1] + S2410[2][3]*SG100[3][1];
SG240[2][2]=S2410[2][1]*SG100[1][2] + S2410[2][2]*SG100[2][2] + S2410[2][3]*SG100[3][2];
SG240[2][3]=S2410[2][1]*SG100[1][3] + S2410[2][2]*SG100[2][3] + S2410[2][3]*SG100[3][3];

SG240[3][1]=S2410[3][2]*SG100[2][1] + S2410[3][3]*SG100[3][1];
SG240[3][2]=S2410[3][2]*SG100[2][2] + S2410[3][3]*SG100[3][2];
SG240[3][3]=S2410[3][2]*SG100[2][3] + S2410[3][3]*SG100[3][3];


SG250[1][1]=S2524[1][1]*SG240[1][1] + S2524[1][2]*SG240[2][1];
SG250[1][2]=S2524[1][1]*SG240[1][2] + S2524[1][2]*SG240[2][2];
SG250[1][3]=S2524[1][1]*SG240[1][3] + S2524[1][2]*SG240[2][3];

SG250[2][1]=S2524[2][1]*SG240[1][1] + S2524[2][2]*SG240[2][1];
SG250[2][2]=S2524[2][1]*SG240[1][2] + S2524[2][2]*SG240[2][2];
SG250[2][3]=S2524[2][1]*SG240[1][3] + S2524[2][2]*SG240[2][3];

SG250[3][1]=SG240[3][1];
SG250[3][2]=SG240[3][2];
SG250[3][3]=SG240[3][3];


SG260[1][1]=S2625[1][1]*SG250[1][1] + S2625[1][2]*SG250[2][1];
SG260[1][2]=S2625[1][1]*SG250[1][2] + S2625[1][2]*SG250[2][2];
SG260[1][3]=S2625[1][1]*SG250[1][3] + S2625[1][2]*SG250[2][3];

SG260[2][1]=S2625[2][1]*SG250[1][1] + S2625[2][2]*SG250[2][1];
SG260[2][2]=S2625[2][1]*SG250[1][2] + S2625[2][2]*SG250[2][2];
SG260[2][3]=S2625[2][1]*SG250[1][3] + S2625[2][2]*SG250[2][3];

SG260[3][1]=SG250[3][1];
SG260[3][2]=SG250[3][2];
SG260[3][3]=SG250[3][3];


SG270[1][1]=SG260[1][1];
SG270[1][2]=SG260[1][2];
SG270[1][3]=SG260[1][3];

SG270[2][1]=SG260[2][1];
SG270[2][2]=SG260[2][2];
SG270[2][3]=SG260[2][3];

SG270[3][1]=SG260[3][1];
SG270[3][2]=SG260[3][2];
SG270[3][3]=SG260[3][3];


SG280[1][1]=S2810[1][1]*SG100[1][1] + S2810[1][2]*SG100[2][1] + S2810[1][3]*SG100[3][1];
SG280[1][2]=S2810[1][1]*SG100[1][2] + S2810[1][2]*SG100[2][2] + S2810[1][3]*SG100[3][2];
SG280[1][3]=S2810[1][1]*SG100[1][3] + S2810[1][2]*SG100[2][3] + S2810[1][3]*SG100[3][3];

SG280[2][1]=S2810[2][1]*SG100[1][1] + S2810[2][2]*SG100[2][1] + S2810[2][3]*SG100[3][1];
SG280[2][2]=S2810[2][1]*SG100[1][2] + S2810[2][2]*SG100[2][2] + S2810[2][3]*SG100[3][2];
SG280[2][3]=S2810[2][1]*SG100[1][3] + S2810[2][2]*SG100[2][3] + S2810[2][3]*SG100[3][3];

SG280[3][1]=S2810[3][2]*SG100[2][1] + S2810[3][3]*SG100[3][1];
SG280[3][2]=S2810[3][2]*SG100[2][2] + S2810[3][3]*SG100[3][2];
SG280[3][3]=S2810[3][2]*SG100[2][3] + S2810[3][3]*SG100[3][3];


SG290[1][1]=S2928[1][1]*SG280[1][1] + S2928[1][2]*SG280[2][1];
SG290[1][2]=S2928[1][1]*SG280[1][2] + S2928[1][2]*SG280[2][2];
SG290[1][3]=S2928[1][1]*SG280[1][3] + S2928[1][2]*SG280[2][3];

SG290[2][1]=S2928[2][1]*SG280[1][1] + S2928[2][2]*SG280[2][1];
SG290[2][2]=S2928[2][1]*SG280[1][2] + S2928[2][2]*SG280[2][2];
SG290[2][3]=S2928[2][1]*SG280[1][3] + S2928[2][2]*SG280[2][3];

SG290[3][1]=SG280[3][1];
SG290[3][2]=SG280[3][2];
SG290[3][3]=SG280[3][3];


SG300[1][1]=S3029[1][1]*SG290[1][1] + S3029[1][2]*SG290[2][1];
SG300[1][2]=S3029[1][1]*SG290[1][2] + S3029[1][2]*SG290[2][2];
SG300[1][3]=S3029[1][1]*SG290[1][3] + S3029[1][2]*SG290[2][3];

SG300[2][1]=S3029[2][1]*SG290[1][1] + S3029[2][2]*SG290[2][1];
SG300[2][2]=S3029[2][1]*SG290[1][2] + S3029[2][2]*SG290[2][2];
SG300[2][3]=S3029[2][1]*SG290[1][3] + S3029[2][2]*SG290[2][3];

SG300[3][1]=SG290[3][1];
SG300[3][2]=SG290[3][2];
SG300[3][3]=SG290[3][3];


SG310[1][1]=SG300[1][1];
SG310[1][2]=SG300[1][2];
SG310[1][3]=SG300[1][3];

SG310[2][1]=SG300[2][1];
SG310[2][2]=SG300[2][2];
SG310[2][3]=SG300[2][3];

SG310[3][1]=SG300[3][1];
SG310[3][2]=SG300[3][2];
SG310[3][3]=SG300[3][3];


SG320[1][1]=S323[1][1]*SG30[1][1] + S323[1][2]*SG30[2][1] + S323[1][3]*SG30[3][1];
SG320[1][2]=S323[1][1]*SG30[1][2] + S323[1][2]*SG30[2][2] + S323[1][3]*SG30[3][2];
SG320[1][3]=S323[1][1]*SG30[1][3] + S323[1][2]*SG30[2][3] + S323[1][3]*SG30[3][3];

SG320[2][1]=S323[2][1]*SG30[1][1] + S323[2][2]*SG30[2][1] + S323[2][3]*SG30[3][1];
SG320[2][2]=S323[2][1]*SG30[1][2] + S323[2][2]*SG30[2][2] + S323[2][3]*SG30[3][2];
SG320[2][3]=S323[2][1]*SG30[1][3] + S323[2][2]*SG30[2][3] + S323[2][3]*SG30[3][3];

SG320[3][1]=-0.7071067811865475*SG30[1][1] - 0.7071067811865475*SG30[3][1];
SG320[3][2]=-0.7071067811865475*SG30[1][2] - 0.7071067811865475*SG30[3][2];
SG320[3][3]=-0.7071067811865475*SG30[1][3] - 0.7071067811865475*SG30[3][3];


SG330[1][1]=S3332[1][1]*SG320[1][1] + S3332[1][3]*SG320[3][1];
SG330[1][2]=S3332[1][1]*SG320[1][2] + S3332[1][3]*SG320[3][2];
SG330[1][3]=S3332[1][1]*SG320[1][3] + S3332[1][3]*SG320[3][3];

SG330[2][1]=S3332[2][1]*SG320[1][1] + S3332[2][3]*SG320[3][1];
SG330[2][2]=S3332[2][1]*SG320[1][2] + S3332[2][3]*SG320[3][2];
SG330[2][3]=S3332[2][1]*SG320[1][3] + S3332[2][3]*SG320[3][3];

SG330[3][1]=SG320[2][1];
SG330[3][2]=SG320[2][2];
SG330[3][3]=SG320[2][3];


SG340[1][1]=S3433[1][1]*SG330[1][1] + S3433[1][3]*SG330[3][1];
SG340[1][2]=S3433[1][1]*SG330[1][2] + S3433[1][3]*SG330[3][2];
SG340[1][3]=S3433[1][1]*SG330[1][3] + S3433[1][3]*SG330[3][3];

SG340[2][1]=S3433[2][1]*SG330[1][1] + S3433[2][3]*SG330[3][1];
SG340[2][2]=S3433[2][1]*SG330[1][2] + S3433[2][3]*SG330[3][2];
SG340[2][3]=S3433[2][1]*SG330[1][3] + S3433[2][3]*SG330[3][3];

SG340[3][1]=-SG330[2][1];
SG340[3][2]=-SG330[2][2];
SG340[3][3]=-SG330[2][3];


SG350[1][1]=S3534[1][2]*SG340[2][1] + S3534[1][3]*SG340[3][1];
SG350[1][2]=S3534[1][2]*SG340[2][2] + S3534[1][3]*SG340[3][2];
SG350[1][3]=S3534[1][2]*SG340[2][3] + S3534[1][3]*SG340[3][3];

SG350[2][1]=S3534[2][2]*SG340[2][1] + S3534[2][3]*SG340[3][1];
SG350[2][2]=S3534[2][2]*SG340[2][2] + S3534[2][3]*SG340[3][2];
SG350[2][3]=S3534[2][2]*SG340[2][3] + S3534[2][3]*SG340[3][3];

SG350[3][1]=-SG340[1][1];
SG350[3][2]=-SG340[1][2];
SG350[3][3]=-SG340[1][3];


SG360[1][1]=S3635[1][1]*SG350[1][1] + S3635[1][3]*SG350[3][1];
SG360[1][2]=S3635[1][1]*SG350[1][2] + S3635[1][3]*SG350[3][2];
SG360[1][3]=S3635[1][1]*SG350[1][3] + S3635[1][3]*SG350[3][3];

SG360[2][1]=S3635[2][1]*SG350[1][1] + S3635[2][3]*SG350[3][1];
SG360[2][2]=S3635[2][1]*SG350[1][2] + S3635[2][3]*SG350[3][2];
SG360[2][3]=S3635[2][1]*SG350[1][3] + S3635[2][3]*SG350[3][3];

SG360[3][1]=-SG350[2][1];
SG360[3][2]=-SG350[2][2];
SG360[3][3]=-SG350[2][3];


SG370[1][1]=S3736[1][2]*SG360[2][1] + S3736[1][3]*SG360[3][1];
SG370[1][2]=S3736[1][2]*SG360[2][2] + S3736[1][3]*SG360[3][2];
SG370[1][3]=S3736[1][2]*SG360[2][3] + S3736[1][3]*SG360[3][3];

SG370[2][1]=S3736[2][2]*SG360[2][1] + S3736[2][3]*SG360[3][1];
SG370[2][2]=S3736[2][2]*SG360[2][2] + S3736[2][3]*SG360[3][2];
SG370[2][3]=S3736[2][2]*SG360[2][3] + S3736[2][3]*SG360[3][3];

SG370[3][1]=-SG360[1][1];
SG370[3][2]=-SG360[1][2];
SG370[3][3]=-SG360[1][3];


SG380[1][1]=S3837[1][1]*SG370[1][1] + S3837[1][3]*SG370[3][1];
SG380[1][2]=S3837[1][1]*SG370[1][2] + S3837[1][3]*SG370[3][2];
SG380[1][3]=S3837[1][1]*SG370[1][3] + S3837[1][3]*SG370[3][3];

SG380[2][1]=S3837[2][1]*SG370[1][1] + S3837[2][3]*SG370[3][1];
SG380[2][2]=S3837[2][1]*SG370[1][2] + S3837[2][3]*SG370[3][2];
SG380[2][3]=S3837[2][1]*SG370[1][3] + S3837[2][3]*SG370[3][3];

SG380[3][1]=SG370[2][1];
SG380[3][2]=SG370[2][2];
SG380[3][3]=SG370[2][3];


SG390[1][1]=S3938[1][1]*SG380[1][1] + S3938[1][2]*SG380[2][1] + S3938[1][3]*SG380[3][1];
SG390[1][2]=S3938[1][1]*SG380[1][2] + S3938[1][2]*SG380[2][2] + S3938[1][3]*SG380[3][2];
SG390[1][3]=S3938[1][1]*SG380[1][3] + S3938[1][2]*SG380[2][3] + S3938[1][3]*SG380[3][3];

SG390[2][1]=S3938[2][1]*SG380[1][1] + S3938[2][2]*SG380[2][1] + S3938[2][3]*SG380[3][1];
SG390[2][2]=S3938[2][1]*SG380[1][2] + S3938[2][2]*SG380[2][2] + S3938[2][3]*SG380[3][2];
SG390[2][3]=S3938[2][1]*SG380[1][3] + S3938[2][2]*SG380[2][3] + S3938[2][3]*SG380[3][3];

SG390[3][1]=S3938[3][1]*SG380[1][1] + S3938[3][2]*SG380[2][1] + S3938[3][3]*SG380[3][1];
SG390[3][2]=S3938[3][1]*SG380[1][2] + S3938[3][2]*SG380[2][2] + S3938[3][3]*SG380[3][2];
SG390[3][3]=S3938[3][1]*SG380[1][3] + S3938[3][2]*SG380[2][3] + S3938[3][3]*SG380[3][3];


SG400[1][1]=S4038[1][1]*SG380[1][1] + S4038[1][2]*SG380[2][1] + S4038[1][3]*SG380[3][1];
SG400[1][2]=S4038[1][1]*SG380[1][2] + S4038[1][2]*SG380[2][2] + S4038[1][3]*SG380[3][2];
SG400[1][3]=S4038[1][1]*SG380[1][3] + S4038[1][2]*SG380[2][3] + S4038[1][3]*SG380[3][3];

SG400[2][1]=S4038[2][1]*SG380[1][1] + S4038[2][2]*SG380[2][1] + S4038[2][3]*SG380[3][1];
SG400[2][2]=S4038[2][1]*SG380[1][2] + S4038[2][2]*SG380[2][2] + S4038[2][3]*SG380[3][2];
SG400[2][3]=S4038[2][1]*SG380[1][3] + S4038[2][2]*SG380[2][3] + S4038[2][3]*SG380[3][3];

SG400[3][1]=S4038[3][1]*SG380[1][1] + S4038[3][2]*SG380[2][1] + S4038[3][3]*SG380[3][1];
SG400[3][2]=S4038[3][1]*SG380[1][2] + S4038[3][2]*SG380[2][2] + S4038[3][3]*SG380[3][2];
SG400[3][3]=S4038[3][1]*SG380[1][3] + S4038[3][2]*SG380[2][3] + S4038[3][3]*SG380[3][3];


SG410[1][1]=S4140[1][1]*SG400[1][1] + S4140[1][2]*SG400[2][1];
SG410[1][2]=S4140[1][1]*SG400[1][2] + S4140[1][2]*SG400[2][2];
SG410[1][3]=S4140[1][1]*SG400[1][3] + S4140[1][2]*SG400[2][3];

SG410[2][1]=S4140[2][1]*SG400[1][1] + S4140[2][2]*SG400[2][1];
SG410[2][2]=S4140[2][1]*SG400[1][2] + S4140[2][2]*SG400[2][2];
SG410[2][3]=S4140[2][1]*SG400[1][3] + S4140[2][2]*SG400[2][3];

SG410[3][1]=SG400[3][1];
SG410[3][2]=SG400[3][2];
SG410[3][3]=SG400[3][3];


SG420[1][1]=S4241[1][1]*SG410[1][1] + S4241[1][2]*SG410[2][1];
SG420[1][2]=S4241[1][1]*SG410[1][2] + S4241[1][2]*SG410[2][2];
SG420[1][3]=S4241[1][1]*SG410[1][3] + S4241[1][2]*SG410[2][3];

SG420[2][1]=S4241[2][1]*SG410[1][1] + S4241[2][2]*SG410[2][1];
SG420[2][2]=S4241[2][1]*SG410[1][2] + S4241[2][2]*SG410[2][2];
SG420[2][3]=S4241[2][1]*SG410[1][3] + S4241[2][2]*SG410[2][3];

SG420[3][1]=SG410[3][1];
SG420[3][2]=SG410[3][2];
SG420[3][3]=SG410[3][3];


SG430[1][1]=SG420[1][1];
SG430[1][2]=SG420[1][2];
SG430[1][3]=SG420[1][3];

SG430[2][1]=SG420[2][1];
SG430[2][2]=SG420[2][2];
SG430[2][3]=SG420[2][3];

SG430[3][1]=SG420[3][1];
SG430[3][2]=SG420[3][2];
SG430[3][3]=SG420[3][3];


SG440[1][1]=S4438[1][1]*SG380[1][1] + S4438[1][2]*SG380[2][1] + S4438[1][3]*SG380[3][1];
SG440[1][2]=S4438[1][1]*SG380[1][2] + S4438[1][2]*SG380[2][2] + S4438[1][3]*SG380[3][2];
SG440[1][3]=S4438[1][1]*SG380[1][3] + S4438[1][2]*SG380[2][3] + S4438[1][3]*SG380[3][3];

SG440[2][1]=S4438[2][1]*SG380[1][1] + S4438[2][2]*SG380[2][1] + S4438[2][3]*SG380[3][1];
SG440[2][2]=S4438[2][1]*SG380[1][2] + S4438[2][2]*SG380[2][2] + S4438[2][3]*SG380[3][2];
SG440[2][3]=S4438[2][1]*SG380[1][3] + S4438[2][2]*SG380[2][3] + S4438[2][3]*SG380[3][3];

SG440[3][1]=S4438[3][2]*SG380[2][1] + S4438[3][3]*SG380[3][1];
SG440[3][2]=S4438[3][2]*SG380[2][2] + S4438[3][3]*SG380[3][2];
SG440[3][3]=S4438[3][2]*SG380[2][3] + S4438[3][3]*SG380[3][3];


SG450[1][1]=S4544[1][1]*SG440[1][1] + S4544[1][2]*SG440[2][1];
SG450[1][2]=S4544[1][1]*SG440[1][2] + S4544[1][2]*SG440[2][2];
SG450[1][3]=S4544[1][1]*SG440[1][3] + S4544[1][2]*SG440[2][3];

SG450[2][1]=S4544[2][1]*SG440[1][1] + S4544[2][2]*SG440[2][1];
SG450[2][2]=S4544[2][1]*SG440[1][2] + S4544[2][2]*SG440[2][2];
SG450[2][3]=S4544[2][1]*SG440[1][3] + S4544[2][2]*SG440[2][3];

SG450[3][1]=SG440[3][1];
SG450[3][2]=SG440[3][2];
SG450[3][3]=SG440[3][3];


SG460[1][1]=S4645[1][1]*SG450[1][1] + S4645[1][2]*SG450[2][1];
SG460[1][2]=S4645[1][1]*SG450[1][2] + S4645[1][2]*SG450[2][2];
SG460[1][3]=S4645[1][1]*SG450[1][3] + S4645[1][2]*SG450[2][3];

SG460[2][1]=S4645[2][1]*SG450[1][1] + S4645[2][2]*SG450[2][1];
SG460[2][2]=S4645[2][1]*SG450[1][2] + S4645[2][2]*SG450[2][2];
SG460[2][3]=S4645[2][1]*SG450[1][3] + S4645[2][2]*SG450[2][3];

SG460[3][1]=SG450[3][1];
SG460[3][2]=SG450[3][2];
SG460[3][3]=SG450[3][3];


SG470[1][1]=SG460[1][1];
SG470[1][2]=SG460[1][2];
SG470[1][3]=SG460[1][3];

SG470[2][1]=SG460[2][1];
SG470[2][2]=SG460[2][2];
SG470[2][3]=SG460[2][3];

SG470[3][1]=SG460[3][1];
SG470[3][2]=SG460[3][2];
SG470[3][3]=SG460[3][3];


SG480[1][1]=S4838[1][1]*SG380[1][1] + S4838[1][2]*SG380[2][1] + S4838[1][3]*SG380[3][1];
SG480[1][2]=S4838[1][1]*SG380[1][2] + S4838[1][2]*SG380[2][2] + S4838[1][3]*SG380[3][2];
SG480[1][3]=S4838[1][1]*SG380[1][3] + S4838[1][2]*SG380[2][3] + S4838[1][3]*SG380[3][3];

SG480[2][1]=S4838[2][1]*SG380[1][1] + S4838[2][2]*SG380[2][1] + S4838[2][3]*SG380[3][1];
SG480[2][2]=S4838[2][1]*SG380[1][2] + S4838[2][2]*SG380[2][2] + S4838[2][3]*SG380[3][2];
SG480[2][3]=S4838[2][1]*SG380[1][3] + S4838[2][2]*SG380[2][3] + S4838[2][3]*SG380[3][3];

SG480[3][1]=S4838[3][2]*SG380[2][1] + S4838[3][3]*SG380[3][1];
SG480[3][2]=S4838[3][2]*SG380[2][2] + S4838[3][3]*SG380[3][2];
SG480[3][3]=S4838[3][2]*SG380[2][3] + S4838[3][3]*SG380[3][3];


SG490[1][1]=S4948[1][1]*SG480[1][1] + S4948[1][2]*SG480[2][1];
SG490[1][2]=S4948[1][1]*SG480[1][2] + S4948[1][2]*SG480[2][2];
SG490[1][3]=S4948[1][1]*SG480[1][3] + S4948[1][2]*SG480[2][3];

SG490[2][1]=S4948[2][1]*SG480[1][1] + S4948[2][2]*SG480[2][1];
SG490[2][2]=S4948[2][1]*SG480[1][2] + S4948[2][2]*SG480[2][2];
SG490[2][3]=S4948[2][1]*SG480[1][3] + S4948[2][2]*SG480[2][3];

SG490[3][1]=SG480[3][1];
SG490[3][2]=SG480[3][2];
SG490[3][3]=SG480[3][3];


SG500[1][1]=S5049[1][1]*SG490[1][1] + S5049[1][2]*SG490[2][1];
SG500[1][2]=S5049[1][1]*SG490[1][2] + S5049[1][2]*SG490[2][2];
SG500[1][3]=S5049[1][1]*SG490[1][3] + S5049[1][2]*SG490[2][3];

SG500[2][1]=S5049[2][1]*SG490[1][1] + S5049[2][2]*SG490[2][1];
SG500[2][2]=S5049[2][1]*SG490[1][2] + S5049[2][2]*SG490[2][2];
SG500[2][3]=S5049[2][1]*SG490[1][3] + S5049[2][2]*SG490[2][3];

SG500[3][1]=SG490[3][1];
SG500[3][2]=SG490[3][2];
SG500[3][3]=SG490[3][3];


SG510[1][1]=SG500[1][1];
SG510[1][2]=SG500[1][2];
SG510[1][3]=SG500[1][3];

SG510[2][1]=SG500[2][1];
SG510[2][2]=SG500[2][2];
SG510[2][3]=SG500[2][3];

SG510[3][1]=SG500[3][1];
SG510[3][2]=SG500[3][2];
SG510[3][3]=SG500[3][3];


SG520[1][1]=S5238[1][1]*SG380[1][1] + S5238[1][2]*SG380[2][1] + S5238[1][3]*SG380[3][1];
SG520[1][2]=S5238[1][1]*SG380[1][2] + S5238[1][2]*SG380[2][2] + S5238[1][3]*SG380[3][2];
SG520[1][3]=S5238[1][1]*SG380[1][3] + S5238[1][2]*SG380[2][3] + S5238[1][3]*SG380[3][3];

SG520[2][1]=S5238[2][1]*SG380[1][1] + S5238[2][2]*SG380[2][1] + S5238[2][3]*SG380[3][1];
SG520[2][2]=S5238[2][1]*SG380[1][2] + S5238[2][2]*SG380[2][2] + S5238[2][3]*SG380[3][2];
SG520[2][3]=S5238[2][1]*SG380[1][3] + S5238[2][2]*SG380[2][3] + S5238[2][3]*SG380[3][3];

SG520[3][1]=S5238[3][2]*SG380[2][1] + S5238[3][3]*SG380[3][1];
SG520[3][2]=S5238[3][2]*SG380[2][2] + S5238[3][3]*SG380[3][2];
SG520[3][3]=S5238[3][2]*SG380[2][3] + S5238[3][3]*SG380[3][3];


SG530[1][1]=S5352[1][1]*SG520[1][1] + S5352[1][2]*SG520[2][1];
SG530[1][2]=S5352[1][1]*SG520[1][2] + S5352[1][2]*SG520[2][2];
SG530[1][3]=S5352[1][1]*SG520[1][3] + S5352[1][2]*SG520[2][3];

SG530[2][1]=S5352[2][1]*SG520[1][1] + S5352[2][2]*SG520[2][1];
SG530[2][2]=S5352[2][1]*SG520[1][2] + S5352[2][2]*SG520[2][2];
SG530[2][3]=S5352[2][1]*SG520[1][3] + S5352[2][2]*SG520[2][3];

SG530[3][1]=SG520[3][1];
SG530[3][2]=SG520[3][2];
SG530[3][3]=SG520[3][3];


SG540[1][1]=S5453[1][1]*SG530[1][1] + S5453[1][2]*SG530[2][1];
SG540[1][2]=S5453[1][1]*SG530[1][2] + S5453[1][2]*SG530[2][2];
SG540[1][3]=S5453[1][1]*SG530[1][3] + S5453[1][2]*SG530[2][3];

SG540[2][1]=S5453[2][1]*SG530[1][1] + S5453[2][2]*SG530[2][1];
SG540[2][2]=S5453[2][1]*SG530[1][2] + S5453[2][2]*SG530[2][2];
SG540[2][3]=S5453[2][1]*SG530[1][3] + S5453[2][2]*SG530[2][3];

SG540[3][1]=SG530[3][1];
SG540[3][2]=SG530[3][2];
SG540[3][3]=SG530[3][3];


SG550[1][1]=SG540[1][1];
SG550[1][2]=SG540[1][2];
SG550[1][3]=SG540[1][3];

SG550[2][1]=SG540[2][1];
SG550[2][2]=SG540[2][2];
SG550[2][3]=SG540[2][3];

SG550[3][1]=SG540[3][1];
SG550[3][2]=SG540[3][2];
SG550[3][3]=SG540[3][3];


SG560[1][1]=S5638[1][1]*SG380[1][1] + S5638[1][2]*SG380[2][1] + S5638[1][3]*SG380[3][1];
SG560[1][2]=S5638[1][1]*SG380[1][2] + S5638[1][2]*SG380[2][2] + S5638[1][3]*SG380[3][2];
SG560[1][3]=S5638[1][1]*SG380[1][3] + S5638[1][2]*SG380[2][3] + S5638[1][3]*SG380[3][3];

SG560[2][1]=S5638[2][1]*SG380[1][1] + S5638[2][2]*SG380[2][1] + S5638[2][3]*SG380[3][1];
SG560[2][2]=S5638[2][1]*SG380[1][2] + S5638[2][2]*SG380[2][2] + S5638[2][3]*SG380[3][2];
SG560[2][3]=S5638[2][1]*SG380[1][3] + S5638[2][2]*SG380[2][3] + S5638[2][3]*SG380[3][3];

SG560[3][1]=S5638[3][2]*SG380[2][1] + S5638[3][3]*SG380[3][1];
SG560[3][2]=S5638[3][2]*SG380[2][2] + S5638[3][3]*SG380[3][2];
SG560[3][3]=S5638[3][2]*SG380[2][3] + S5638[3][3]*SG380[3][3];


SG570[1][1]=S5756[1][1]*SG560[1][1] + S5756[1][2]*SG560[2][1];
SG570[1][2]=S5756[1][1]*SG560[1][2] + S5756[1][2]*SG560[2][2];
SG570[1][3]=S5756[1][1]*SG560[1][3] + S5756[1][2]*SG560[2][3];

SG570[2][1]=S5756[2][1]*SG560[1][1] + S5756[2][2]*SG560[2][1];
SG570[2][2]=S5756[2][1]*SG560[1][2] + S5756[2][2]*SG560[2][2];
SG570[2][3]=S5756[2][1]*SG560[1][3] + S5756[2][2]*SG560[2][3];

SG570[3][1]=SG560[3][1];
SG570[3][2]=SG560[3][2];
SG570[3][3]=SG560[3][3];


SG580[1][1]=S5857[1][1]*SG570[1][1] + S5857[1][2]*SG570[2][1];
SG580[1][2]=S5857[1][1]*SG570[1][2] + S5857[1][2]*SG570[2][2];
SG580[1][3]=S5857[1][1]*SG570[1][3] + S5857[1][2]*SG570[2][3];

SG580[2][1]=S5857[2][1]*SG570[1][1] + S5857[2][2]*SG570[2][1];
SG580[2][2]=S5857[2][1]*SG570[1][2] + S5857[2][2]*SG570[2][2];
SG580[2][3]=S5857[2][1]*SG570[1][3] + S5857[2][2]*SG570[2][3];

SG580[3][1]=SG570[3][1];
SG580[3][2]=SG570[3][2];
SG580[3][3]=SG570[3][3];


SG590[1][1]=SG580[1][1];
SG590[1][2]=SG580[1][2];
SG590[1][3]=SG580[1][3];

SG590[2][1]=SG580[2][1];
SG590[2][2]=SG580[2][2];
SG590[2][3]=SG580[2][3];

SG590[3][1]=SG580[3][1];
SG590[3][2]=SG580[3][2];
SG590[3][3]=SG580[3][3];


SG600[1][1]=S603[1][1]*SG30[1][1] + S603[1][2]*SG30[2][1];
SG600[1][2]=S603[1][1]*SG30[1][2] + S603[1][2]*SG30[2][2];
SG600[1][3]=S603[1][1]*SG30[1][3] + S603[1][2]*SG30[2][3];

SG600[2][1]=S603[2][1]*SG30[1][1] + S603[2][2]*SG30[2][1];
SG600[2][2]=S603[2][1]*SG30[1][2] + S603[2][2]*SG30[2][2];
SG600[2][3]=S603[2][1]*SG30[1][3] + S603[2][2]*SG30[2][3];

SG600[3][1]=SG30[3][1];
SG600[3][2]=SG30[3][2];
SG600[3][3]=SG30[3][3];


SG610[1][1]=S6160[1][2]*SG600[2][1] + S6160[1][3]*SG600[3][1];
SG610[1][2]=S6160[1][2]*SG600[2][2] + S6160[1][3]*SG600[3][2];
SG610[1][3]=S6160[1][2]*SG600[2][3] + S6160[1][3]*SG600[3][3];

SG610[2][1]=S6160[2][2]*SG600[2][1] + S6160[2][3]*SG600[3][1];
SG610[2][2]=S6160[2][2]*SG600[2][2] + S6160[2][3]*SG600[3][2];
SG610[2][3]=S6160[2][2]*SG600[2][3] + S6160[2][3]*SG600[3][3];

SG610[3][1]=SG600[1][1];
SG610[3][2]=SG600[1][2];
SG610[3][3]=SG600[1][3];


SG620[1][1]=S6261[1][1]*SG610[1][1] + S6261[1][3]*SG610[3][1];
SG620[1][2]=S6261[1][1]*SG610[1][2] + S6261[1][3]*SG610[3][2];
SG620[1][3]=S6261[1][1]*SG610[1][3] + S6261[1][3]*SG610[3][3];

SG620[2][1]=S6261[2][1]*SG610[1][1] + S6261[2][3]*SG610[3][1];
SG620[2][2]=S6261[2][1]*SG610[1][2] + S6261[2][3]*SG610[3][2];
SG620[2][3]=S6261[2][1]*SG610[1][3] + S6261[2][3]*SG610[3][3];

SG620[3][1]=SG610[2][1];
SG620[3][2]=SG610[2][2];
SG620[3][3]=SG610[2][3];


SG630[1][1]=S6362[1][1]*SG620[1][1] + S6362[1][2]*SG620[2][1];
SG630[1][2]=S6362[1][1]*SG620[1][2] + S6362[1][2]*SG620[2][2];
SG630[1][3]=S6362[1][1]*SG620[1][3] + S6362[1][2]*SG620[2][3];

SG630[2][1]=S6362[2][1]*SG620[1][1] + S6362[2][2]*SG620[2][1];
SG630[2][2]=S6362[2][1]*SG620[1][2] + S6362[2][2]*SG620[2][2];
SG630[2][3]=S6362[2][1]*SG620[1][3] + S6362[2][2]*SG620[2][3];

SG630[3][1]=SG620[3][1];
SG630[3][2]=SG620[3][2];
SG630[3][3]=SG620[3][3];


SG640[1][1]=S6463[1][2]*SG630[2][1] + S6463[1][3]*SG630[3][1];
SG640[1][2]=S6463[1][2]*SG630[2][2] + S6463[1][3]*SG630[3][2];
SG640[1][3]=S6463[1][2]*SG630[2][3] + S6463[1][3]*SG630[3][3];

SG640[2][1]=S6463[2][2]*SG630[2][1] + S6463[2][3]*SG630[3][1];
SG640[2][2]=S6463[2][2]*SG630[2][2] + S6463[2][3]*SG630[3][2];
SG640[2][3]=S6463[2][2]*SG630[2][3] + S6463[2][3]*SG630[3][3];

SG640[3][1]=-SG630[1][1];
SG640[3][2]=-SG630[1][2];
SG640[3][3]=-SG630[1][3];


SG650[1][1]=-SG640[3][1];
SG650[1][2]=-SG640[3][2];
SG650[1][3]=-SG640[3][3];

SG650[2][1]=-SG640[2][1];
SG650[2][2]=-SG640[2][2];
SG650[2][3]=-SG640[2][3];

SG650[3][1]=-SG640[1][1];
SG650[3][2]=-SG640[1][2];
SG650[3][3]=-SG640[1][3];


SG660[1][1]=S6662[1][1]*SG620[1][1] + S6662[1][2]*SG620[2][1];
SG660[1][2]=S6662[1][1]*SG620[1][2] + S6662[1][2]*SG620[2][2];
SG660[1][3]=S6662[1][1]*SG620[1][3] + S6662[1][2]*SG620[2][3];

SG660[2][1]=S6662[2][1]*SG620[1][1] + S6662[2][2]*SG620[2][1];
SG660[2][2]=S6662[2][1]*SG620[1][2] + S6662[2][2]*SG620[2][2];
SG660[2][3]=S6662[2][1]*SG620[1][3] + S6662[2][2]*SG620[2][3];

SG660[3][1]=SG620[3][1];
SG660[3][2]=SG620[3][2];
SG660[3][3]=SG620[3][3];


SG670[1][1]=S6766[1][2]*SG660[2][1] + S6766[1][3]*SG660[3][1];
SG670[1][2]=S6766[1][2]*SG660[2][2] + S6766[1][3]*SG660[3][2];
SG670[1][3]=S6766[1][2]*SG660[2][3] + S6766[1][3]*SG660[3][3];

SG670[2][1]=S6766[2][2]*SG660[2][1] + S6766[2][3]*SG660[3][1];
SG670[2][2]=S6766[2][2]*SG660[2][2] + S6766[2][3]*SG660[3][2];
SG670[2][3]=S6766[2][2]*SG660[2][3] + S6766[2][3]*SG660[3][3];

SG670[3][1]=-SG660[1][1];
SG670[3][2]=-SG660[1][2];
SG670[3][3]=-SG660[1][3];


SG680[1][1]=-SG670[3][1];
SG680[1][2]=-SG670[3][2];
SG680[1][3]=-SG670[3][3];

SG680[2][1]=-SG670[2][1];
SG680[2][2]=-SG670[2][2];
SG680[2][3]=-SG670[2][3];

SG680[3][1]=-SG670[1][1];
SG680[3][2]=-SG670[1][2];
SG680[3][3]=-SG670[1][3];


SG690[1][1]=SG620[1][1];
SG690[1][2]=SG620[1][2];
SG690[1][3]=SG620[1][3];

SG690[2][1]=-SG620[2][1];
SG690[2][2]=-SG620[2][2];
SG690[2][3]=-SG620[2][3];

SG690[3][1]=-SG620[3][1];
SG690[3][2]=-SG620[3][2];
SG690[3][3]=-SG620[3][3];


SG700[1][1]=S00[1][1]*S700[1][1] + S00[3][1]*S700[1][3];
SG700[1][2]=S00[1][2]*S700[1][1] + S00[3][2]*S700[1][3];
SG700[1][3]=S00[1][3]*S700[1][1] + S00[3][3]*S700[1][3];

SG700[2][1]=S00[1][1]*S700[2][1] + S00[3][1]*S700[2][3];
SG700[2][2]=S00[1][2]*S700[2][1] + S00[3][2]*S700[2][3];
SG700[2][3]=S00[1][3]*S700[2][1] + S00[3][3]*S700[2][3];

SG700[3][1]=S00[2][1];
SG700[3][2]=S00[2][2];
SG700[3][3]=S00[2][3];


SG710[1][1]=S7170[1][1]*SG700[1][1] + S7170[1][3]*SG700[3][1];
SG710[1][2]=S7170[1][1]*SG700[1][2] + S7170[1][3]*SG700[3][2];
SG710[1][3]=S7170[1][1]*SG700[1][3] + S7170[1][3]*SG700[3][3];

SG710[2][1]=S7170[2][1]*SG700[1][1] + S7170[2][3]*SG700[3][1];
SG710[2][2]=S7170[2][1]*SG700[1][2] + S7170[2][3]*SG700[3][2];
SG710[2][3]=S7170[2][1]*SG700[1][3] + S7170[2][3]*SG700[3][3];

SG710[3][1]=-SG700[2][1];
SG710[3][2]=-SG700[2][2];
SG710[3][3]=-SG700[2][3];


SG720[1][1]=S7271[1][1]*SG710[1][1] + S7271[1][2]*SG710[2][1] + S7271[1][3]*SG710[3][1];
SG720[1][2]=S7271[1][1]*SG710[1][2] + S7271[1][2]*SG710[2][2] + S7271[1][3]*SG710[3][2];
SG720[1][3]=S7271[1][1]*SG710[1][3] + S7271[1][2]*SG710[2][3] + S7271[1][3]*SG710[3][3];

SG720[2][1]=S7271[2][1]*SG710[1][1] + S7271[2][2]*SG710[2][1] + S7271[2][3]*SG710[3][1];
SG720[2][2]=S7271[2][1]*SG710[1][2] + S7271[2][2]*SG710[2][2] + S7271[2][3]*SG710[3][2];
SG720[2][3]=S7271[2][1]*SG710[1][3] + S7271[2][2]*SG710[2][3] + S7271[2][3]*SG710[3][3];

SG720[3][1]=S7271[3][1]*SG710[1][1] + S7271[3][2]*SG710[2][1];
SG720[3][2]=S7271[3][1]*SG710[1][2] + S7271[3][2]*SG710[2][2];
SG720[3][3]=S7271[3][1]*SG710[1][3] + S7271[3][2]*SG710[2][3];


SG730[1][1]=S7372[1][1]*SG720[1][1] + S7372[1][3]*SG720[3][1];
SG730[1][2]=S7372[1][1]*SG720[1][2] + S7372[1][3]*SG720[3][2];
SG730[1][3]=S7372[1][1]*SG720[1][3] + S7372[1][3]*SG720[3][3];

SG730[2][1]=S7372[2][1]*SG720[1][1] + S7372[2][3]*SG720[3][1];
SG730[2][2]=S7372[2][1]*SG720[1][2] + S7372[2][3]*SG720[3][2];
SG730[2][3]=S7372[2][1]*SG720[1][3] + S7372[2][3]*SG720[3][3];

SG730[3][1]=SG720[2][1];
SG730[3][2]=SG720[2][2];
SG730[3][3]=SG720[2][3];


SG740[1][1]=S7473[1][1]*SG730[1][1] + S7473[1][3]*SG730[3][1];
SG740[1][2]=S7473[1][1]*SG730[1][2] + S7473[1][3]*SG730[3][2];
SG740[1][3]=S7473[1][1]*SG730[1][3] + S7473[1][3]*SG730[3][3];

SG740[2][1]=S7473[2][1]*SG730[1][1] + S7473[2][3]*SG730[3][1];
SG740[2][2]=S7473[2][1]*SG730[1][2] + S7473[2][3]*SG730[3][2];
SG740[2][3]=S7473[2][1]*SG730[1][3] + S7473[2][3]*SG730[3][3];

SG740[3][1]=-SG730[2][1];
SG740[3][2]=-SG730[2][2];
SG740[3][3]=-SG730[2][3];


SG750[1][1]=S7574[1][1]*SG740[1][1] + S7574[1][3]*SG740[3][1];
SG750[1][2]=S7574[1][1]*SG740[1][2] + S7574[1][3]*SG740[3][2];
SG750[1][3]=S7574[1][1]*SG740[1][3] + S7574[1][3]*SG740[3][3];

SG750[2][1]=S7574[2][1]*SG740[1][1] + S7574[2][3]*SG740[3][1];
SG750[2][2]=S7574[2][1]*SG740[1][2] + S7574[2][3]*SG740[3][2];
SG750[2][3]=S7574[2][1]*SG740[1][3] + S7574[2][3]*SG740[3][3];

SG750[3][1]=-SG740[2][1];
SG750[3][2]=-SG740[2][2];
SG750[3][3]=-SG740[2][3];


SG760[1][1]=S7675[1][1]*SG750[1][1] + S7675[1][3]*SG750[3][1];
SG760[1][2]=S7675[1][1]*SG750[1][2] + S7675[1][3]*SG750[3][2];
SG760[1][3]=S7675[1][1]*SG750[1][3] + S7675[1][3]*SG750[3][3];

SG760[2][1]=S7675[2][1]*SG750[1][1] + S7675[2][3]*SG750[3][1];
SG760[2][2]=S7675[2][1]*SG750[1][2] + S7675[2][3]*SG750[3][2];
SG760[2][3]=S7675[2][1]*SG750[1][3] + S7675[2][3]*SG750[3][3];

SG760[3][1]=SG750[2][1];
SG760[3][2]=SG750[2][2];
SG760[3][3]=SG750[2][3];


SG770[1][1]=SG760[1][1];
SG770[1][2]=SG760[1][2];
SG770[1][3]=SG760[1][3];

SG770[2][1]=SG760[2][1];
SG770[2][2]=SG760[2][2];
SG770[2][3]=SG760[2][3];

SG770[3][1]=SG760[3][1];
SG770[3][2]=SG760[3][2];
SG770[3][3]=SG760[3][3];


SG780[1][1]=SG760[1][1];
SG780[1][2]=SG760[1][2];
SG780[1][3]=SG760[1][3];

SG780[2][1]=SG760[2][1];
SG780[2][2]=SG760[2][2];
SG780[2][3]=SG760[2][3];

SG780[3][1]=SG760[3][1];
SG780[3][2]=SG760[3][2];
SG780[3][3]=SG760[3][3];


SG790[1][1]=SG760[1][1];
SG790[1][2]=SG760[1][2];
SG790[1][3]=SG760[1][3];

SG790[2][1]=SG760[2][1];
SG790[2][2]=SG760[2][2];
SG790[2][3]=SG760[2][3];

SG790[3][1]=SG760[3][1];
SG790[3][2]=SG760[3][2];
SG790[3][3]=SG760[3][3];


SG800[1][1]=SG760[1][1];
SG800[1][2]=SG760[1][2];
SG800[1][3]=SG760[1][3];

SG800[2][1]=SG760[2][1];
SG800[2][2]=SG760[2][2];
SG800[2][3]=SG760[2][3];

SG800[3][1]=SG760[3][1];
SG800[3][2]=SG760[3][2];
SG800[3][3]=SG760[3][3];


SG810[1][1]=SG760[1][1];
SG810[1][2]=SG760[1][2];
SG810[1][3]=SG760[1][3];

SG810[2][1]=SG760[2][1];
SG810[2][2]=SG760[2][2];
SG810[2][3]=SG760[2][3];

SG810[3][1]=SG760[3][1];
SG810[3][2]=SG760[3][2];
SG810[3][3]=SG760[3][3];


SG820[1][1]=SG760[1][1];
SG820[1][2]=SG760[1][2];
SG820[1][3]=SG760[1][3];

SG820[2][1]=SG760[2][1];
SG820[2][2]=SG760[2][2];
SG820[2][3]=SG760[2][3];

SG820[3][1]=SG760[3][1];
SG820[3][2]=SG760[3][2];
SG820[3][3]=SG760[3][3];


SG830[1][1]=S8376[1][1]*SG760[1][1] + S8376[1][2]*SG760[2][1] + S8376[1][3]*SG760[3][1];
SG830[1][2]=S8376[1][1]*SG760[1][2] + S8376[1][2]*SG760[2][2] + S8376[1][3]*SG760[3][2];
SG830[1][3]=S8376[1][1]*SG760[1][3] + S8376[1][2]*SG760[2][3] + S8376[1][3]*SG760[3][3];

SG830[2][1]=S8376[2][1]*SG760[1][1] + S8376[2][2]*SG760[2][1] + S8376[2][3]*SG760[3][1];
SG830[2][2]=S8376[2][1]*SG760[1][2] + S8376[2][2]*SG760[2][2] + S8376[2][3]*SG760[3][2];
SG830[2][3]=S8376[2][1]*SG760[1][3] + S8376[2][2]*SG760[2][3] + S8376[2][3]*SG760[3][3];

SG830[3][1]=S8376[3][1]*SG760[1][1] + S8376[3][2]*SG760[2][1] + S8376[3][3]*SG760[3][1];
SG830[3][2]=S8376[3][1]*SG760[1][2] + S8376[3][2]*SG760[2][2] + S8376[3][3]*SG760[3][2];
SG830[3][3]=S8376[3][1]*SG760[1][3] + S8376[3][2]*SG760[2][3] + S8376[3][3]*SG760[3][3];


SG840[1][1]=S00[1][1]*S840[1][1] + S00[3][1]*S840[1][3];
SG840[1][2]=S00[1][2]*S840[1][1] + S00[3][2]*S840[1][3];
SG840[1][3]=S00[1][3]*S840[1][1] + S00[3][3]*S840[1][3];

SG840[2][1]=S00[1][1]*S840[2][1] + S00[3][1]*S840[2][3];
SG840[2][2]=S00[1][2]*S840[2][1] + S00[3][2]*S840[2][3];
SG840[2][3]=S00[1][3]*S840[2][1] + S00[3][3]*S840[2][3];

SG840[3][1]=-S00[2][1];
SG840[3][2]=-S00[2][2];
SG840[3][3]=-S00[2][3];


SG850[1][1]=S8584[1][1]*SG840[1][1] + S8584[1][3]*SG840[3][1];
SG850[1][2]=S8584[1][1]*SG840[1][2] + S8584[1][3]*SG840[3][2];
SG850[1][3]=S8584[1][1]*SG840[1][3] + S8584[1][3]*SG840[3][3];

SG850[2][1]=S8584[2][1]*SG840[1][1] + S8584[2][3]*SG840[3][1];
SG850[2][2]=S8584[2][1]*SG840[1][2] + S8584[2][3]*SG840[3][2];
SG850[2][3]=S8584[2][1]*SG840[1][3] + S8584[2][3]*SG840[3][3];

SG850[3][1]=SG840[2][1];
SG850[3][2]=SG840[2][2];
SG850[3][3]=SG840[2][3];


SG860[1][1]=S8685[1][1]*SG850[1][1] + S8685[1][2]*SG850[2][1] + S8685[1][3]*SG850[3][1];
SG860[1][2]=S8685[1][1]*SG850[1][2] + S8685[1][2]*SG850[2][2] + S8685[1][3]*SG850[3][2];
SG860[1][3]=S8685[1][1]*SG850[1][3] + S8685[1][2]*SG850[2][3] + S8685[1][3]*SG850[3][3];

SG860[2][1]=S8685[2][1]*SG850[1][1] + S8685[2][2]*SG850[2][1] + S8685[2][3]*SG850[3][1];
SG860[2][2]=S8685[2][1]*SG850[1][2] + S8685[2][2]*SG850[2][2] + S8685[2][3]*SG850[3][2];
SG860[2][3]=S8685[2][1]*SG850[1][3] + S8685[2][2]*SG850[2][3] + S8685[2][3]*SG850[3][3];

SG860[3][1]=S8685[3][1]*SG850[1][1] + S8685[3][2]*SG850[2][1];
SG860[3][2]=S8685[3][1]*SG850[1][2] + S8685[3][2]*SG850[2][2];
SG860[3][3]=S8685[3][1]*SG850[1][3] + S8685[3][2]*SG850[2][3];


SG870[1][1]=S8786[1][1]*SG860[1][1] + S8786[1][3]*SG860[3][1];
SG870[1][2]=S8786[1][1]*SG860[1][2] + S8786[1][3]*SG860[3][2];
SG870[1][3]=S8786[1][1]*SG860[1][3] + S8786[1][3]*SG860[3][3];

SG870[2][1]=S8786[2][1]*SG860[1][1] + S8786[2][3]*SG860[3][1];
SG870[2][2]=S8786[2][1]*SG860[1][2] + S8786[2][3]*SG860[3][2];
SG870[2][3]=S8786[2][1]*SG860[1][3] + S8786[2][3]*SG860[3][3];

SG870[3][1]=-SG860[2][1];
SG870[3][2]=-SG860[2][2];
SG870[3][3]=-SG860[2][3];


SG880[1][1]=S8887[1][1]*SG870[1][1] + S8887[1][3]*SG870[3][1];
SG880[1][2]=S8887[1][1]*SG870[1][2] + S8887[1][3]*SG870[3][2];
SG880[1][3]=S8887[1][1]*SG870[1][3] + S8887[1][3]*SG870[3][3];

SG880[2][1]=S8887[2][1]*SG870[1][1] + S8887[2][3]*SG870[3][1];
SG880[2][2]=S8887[2][1]*SG870[1][2] + S8887[2][3]*SG870[3][2];
SG880[2][3]=S8887[2][1]*SG870[1][3] + S8887[2][3]*SG870[3][3];

SG880[3][1]=SG870[2][1];
SG880[3][2]=SG870[2][2];
SG880[3][3]=SG870[2][3];


SG890[1][1]=S8988[1][1]*SG880[1][1] + S8988[1][3]*SG880[3][1];
SG890[1][2]=S8988[1][1]*SG880[1][2] + S8988[1][3]*SG880[3][2];
SG890[1][3]=S8988[1][1]*SG880[1][3] + S8988[1][3]*SG880[3][3];

SG890[2][1]=S8988[2][1]*SG880[1][1] + S8988[2][3]*SG880[3][1];
SG890[2][2]=S8988[2][1]*SG880[1][2] + S8988[2][3]*SG880[3][2];
SG890[2][3]=S8988[2][1]*SG880[1][3] + S8988[2][3]*SG880[3][3];

SG890[3][1]=SG880[2][1];
SG890[3][2]=SG880[2][2];
SG890[3][3]=SG880[2][3];


SG900[1][1]=S9089[1][1]*SG890[1][1] + S9089[1][3]*SG890[3][1];
SG900[1][2]=S9089[1][1]*SG890[1][2] + S9089[1][3]*SG890[3][2];
SG900[1][3]=S9089[1][1]*SG890[1][3] + S9089[1][3]*SG890[3][3];

SG900[2][1]=S9089[2][1]*SG890[1][1] + S9089[2][3]*SG890[3][1];
SG900[2][2]=S9089[2][1]*SG890[1][2] + S9089[2][3]*SG890[3][2];
SG900[2][3]=S9089[2][1]*SG890[1][3] + S9089[2][3]*SG890[3][3];

SG900[3][1]=-SG890[2][1];
SG900[3][2]=-SG890[2][2];
SG900[3][3]=-SG890[2][3];


SG910[1][1]=SG900[1][1];
SG910[1][2]=SG900[1][2];
SG910[1][3]=SG900[1][3];

SG910[2][1]=SG900[2][1];
SG910[2][2]=SG900[2][2];
SG910[2][3]=SG900[2][3];

SG910[3][1]=SG900[3][1];
SG910[3][2]=SG900[3][2];
SG910[3][3]=SG900[3][3];


SG920[1][1]=SG900[1][1];
SG920[1][2]=SG900[1][2];
SG920[1][3]=SG900[1][3];

SG920[2][1]=SG900[2][1];
SG920[2][2]=SG900[2][2];
SG920[2][3]=SG900[2][3];

SG920[3][1]=SG900[3][1];
SG920[3][2]=SG900[3][2];
SG920[3][3]=SG900[3][3];


SG930[1][1]=SG900[1][1];
SG930[1][2]=SG900[1][2];
SG930[1][3]=SG900[1][3];

SG930[2][1]=SG900[2][1];
SG930[2][2]=SG900[2][2];
SG930[2][3]=SG900[2][3];

SG930[3][1]=SG900[3][1];
SG930[3][2]=SG900[3][2];
SG930[3][3]=SG900[3][3];


SG940[1][1]=SG900[1][1];
SG940[1][2]=SG900[1][2];
SG940[1][3]=SG900[1][3];

SG940[2][1]=SG900[2][1];
SG940[2][2]=SG900[2][2];
SG940[2][3]=SG900[2][3];

SG940[3][1]=SG900[3][1];
SG940[3][2]=SG900[3][2];
SG940[3][3]=SG900[3][3];


SG950[1][1]=SG900[1][1];
SG950[1][2]=SG900[1][2];
SG950[1][3]=SG900[1][3];

SG950[2][1]=SG900[2][1];
SG950[2][2]=SG900[2][2];
SG950[2][3]=SG900[2][3];

SG950[3][1]=SG900[3][1];
SG950[3][2]=SG900[3][2];
SG950[3][3]=SG900[3][3];


SG960[1][1]=SG900[1][1];
SG960[1][2]=SG900[1][2];
SG960[1][3]=SG900[1][3];

SG960[2][1]=SG900[2][1];
SG960[2][2]=SG900[2][2];
SG960[2][3]=SG900[2][3];

SG960[3][1]=SG900[3][1];
SG960[3][2]=SG900[3][2];
SG960[3][3]=SG900[3][3];


SG970[1][1]=S9790[1][1]*SG900[1][1] + S9790[1][2]*SG900[2][1] + S9790[1][3]*SG900[3][1];
SG970[1][2]=S9790[1][1]*SG900[1][2] + S9790[1][2]*SG900[2][2] + S9790[1][3]*SG900[3][2];
SG970[1][3]=S9790[1][1]*SG900[1][3] + S9790[1][2]*SG900[2][3] + S9790[1][3]*SG900[3][3];

SG970[2][1]=S9790[2][1]*SG900[1][1] + S9790[2][2]*SG900[2][1] + S9790[2][3]*SG900[3][1];
SG970[2][2]=S9790[2][1]*SG900[1][2] + S9790[2][2]*SG900[2][2] + S9790[2][3]*SG900[3][2];
SG970[2][3]=S9790[2][1]*SG900[1][3] + S9790[2][2]*SG900[2][3] + S9790[2][3]*SG900[3][3];

SG970[3][1]=S9790[3][1]*SG900[1][1] + S9790[3][2]*SG900[2][1] + S9790[3][3]*SG900[3][1];
SG970[3][2]=S9790[3][1]*SG900[1][2] + S9790[3][2]*SG900[2][2] + S9790[3][3]*SG900[3][2];
SG970[3][3]=S9790[3][1]*SG900[1][3] + S9790[3][2]*SG900[2][3] + S9790[3][3]*SG900[3][3];




}


void
hermes_InvDynNEfunc4(void)
     {
/* velocity vectors */
v0[1]=baseo[0].ad[1]*S00[1][1] + baseo[0].ad[2]*S00[1][2] + baseo[0].ad[3]*S00[1][3];
v0[2]=baseo[0].ad[1]*S00[2][1] + baseo[0].ad[2]*S00[2][2] + baseo[0].ad[3]*S00[2][3];
v0[3]=baseo[0].ad[1]*S00[3][1] + baseo[0].ad[2]*S00[3][2] + baseo[0].ad[3]*S00[3][3];
v0[4]=basec[0].xd[1]*S00[1][1] + basec[0].xd[2]*S00[1][2] + basec[0].xd[3]*S00[1][3];
v0[5]=basec[0].xd[1]*S00[2][1] + basec[0].xd[2]*S00[2][2] + basec[0].xd[3]*S00[2][3];
v0[6]=basec[0].xd[1]*S00[3][1] + basec[0].xd[2]*S00[3][2] + basec[0].xd[3]*S00[3][3];

v1[1]=v0[1]*S10[1][1] + v0[2]*S10[1][2];
v1[2]=v0[1]*S10[2][1] + v0[2]*S10[2][2];
v1[3]=state[29].thd - v0[3];
v1[4]=PELVIS2THORAX*v0[2]*S10[1][1] + PELVISOFFSET*v0[3]*S10[1][1] + v0[4]*S10[1][1] - PELVIS2THORAX*v0[1]*S10[1][2] + v0[5]*S10[1][2];
v1[5]=PELVIS2THORAX*v0[2]*S10[2][1] + PELVISOFFSET*v0[3]*S10[2][1] + v0[4]*S10[2][1] - PELVIS2THORAX*v0[1]*S10[2][2] + v0[5]*S10[2][2];
v1[6]=PELVISOFFSET*v0[1] - v0[6];

v2[1]=v1[1]*S21[1][1] + v1[3]*S21[1][3];
v2[2]=v1[1]*S21[2][1] + v1[3]*S21[2][3];
v2[3]=state[30].thd - v1[2];
v2[4]=v1[4]*S21[1][1] + v1[6]*S21[1][3];
v2[5]=v1[4]*S21[2][1] + v1[6]*S21[2][3];
v2[6]=-v1[5];

v3[1]=v2[1]*S32[1][1] + v2[3]*S32[1][3];
v3[2]=v2[1]*S32[2][1] + v2[3]*S32[2][3];
v3[3]=state[31].thd + v2[2];
v3[4]=v2[4]*S32[1][1] + v2[6]*S32[1][3];
v3[5]=v2[4]*S32[2][1] + v2[6]*S32[2][3];
v3[6]=v2[5];

v4[1]=v3[1]*S43[1][1] + v3[2]*S43[1][2] + v3[3]*S43[1][3];
v4[2]=v3[1]*S43[2][1] + v3[2]*S43[2][2] + v3[3]*S43[2][3];
v4[3]=state[1].thd + 0.7071067811865475*v3[1] - 0.7071067811865475*v3[3];
v4[4]=v3[4]*S43[1][1] - THORAX2SHOULDER*v3[3]*S43[1][2] + v3[5]*S43[1][2] + THORAX2SHOULDER*v3[2]*S43[1][3] + v3[6]*S43[1][3];
v4[5]=v3[4]*S43[2][1] - THORAX2SHOULDER*v3[3]*S43[2][2] + v3[5]*S43[2][2] + THORAX2SHOULDER*v3[2]*S43[2][3] + v3[6]*S43[2][3];
v4[6]=-0.7071067811865475*THORAX2SHOULDER*v3[2] + 0.7071067811865475*v3[4] - 0.7071067811865475*v3[6];

v5[1]=v4[1]*S54[1][1] + v4[3]*S54[1][3];
v5[2]=v4[1]*S54[2][1] + v4[3]*S54[2][3];
v5[3]=state[2].thd - v4[2];
v5[4]=-(SHOULDERX*v4[2]*S54[1][1]) + v4[4]*S54[1][1] + v4[6]*S54[1][3];
v5[5]=-(SHOULDERX*v4[2]*S54[2][1]) + v4[4]*S54[2][1] + v4[6]*S54[2][3];
v5[6]=-(SHOULDERX*v4[1]) - v4[5];

v6[1]=v5[1]*S65[1][1] + v5[3]*S65[1][3];
v6[2]=v5[1]*S65[2][1] + v5[3]*S65[2][3];
v6[3]=state[3].thd + v5[2];
v6[4]=v5[4]*S65[1][1] + SHOULDERY*v5[2]*S65[1][3] + v5[6]*S65[1][3];
v6[5]=v5[4]*S65[2][1] + SHOULDERY*v5[2]*S65[2][3] + v5[6]*S65[2][3];
v6[6]=-(SHOULDERY*v5[3]) + v5[5];

v7[1]=v6[2]*S76[1][2] + v6[3]*S76[1][3];
v7[2]=v6[2]*S76[2][2] + v6[3]*S76[2][3];
v7[3]=state[4].thd + v6[1];
v7[4]=UPPERARM*v6[1]*S76[1][2] + v6[5]*S76[1][2] + v6[6]*S76[1][3];
v7[5]=UPPERARM*v6[1]*S76[2][2] + v6[5]*S76[2][2] + v6[6]*S76[2][3];
v7[6]=-(UPPERARM*v6[2]) + v6[4];

v8[1]=v7[1]*S87[1][1] + v7[3]*S87[1][3];
v8[2]=v7[1]*S87[2][1] + v7[3]*S87[2][3];
v8[3]=state[5].thd + v7[2];
v8[4]=v7[4]*S87[1][1] + v7[6]*S87[1][3];
v8[5]=v7[4]*S87[2][1] + v7[6]*S87[2][3];
v8[6]=v7[5];

v9[1]=v8[2]*S98[1][2] + v8[3]*S98[1][3];
v9[2]=v8[2]*S98[2][2] + v8[3]*S98[2][3];
v9[3]=state[6].thd + v8[1];
v9[4]=v8[5]*S98[1][2] + v8[6]*S98[1][3] + v8[1]*(LOWERARM*S98[1][2] + WRISTY*S98[1][3]);
v9[5]=v8[5]*S98[2][2] + v8[6]*S98[2][3] + v8[1]*(LOWERARM*S98[2][2] + WRISTY*S98[2][3]);
v9[6]=-(LOWERARM*v8[2]) - WRISTY*v8[3] + v8[4];

v10[1]=v9[1]*S109[1][1] + v9[3]*S109[1][3];
v10[2]=v9[1]*S109[2][1] + v9[3]*S109[2][3];
v10[3]=state[7].thd - v9[2];
v10[4]=v9[4]*S109[1][1] + v9[6]*S109[1][3];
v10[5]=v9[4]*S109[2][1] + v9[6]*S109[2][3];
v10[6]=-v9[5];

v11[1]=v10[1]*S1110[1][1] + v10[2]*S1110[1][2] + v10[3]*S1110[1][3];
v11[2]=v10[1]*S1110[2][1] + v10[2]*S1110[2][2] + v10[3]*S1110[2][3];
v11[3]=v10[1]*S1110[3][1] + v10[2]*S1110[3][2] + v10[3]*S1110[3][3];
v11[4]=v10[4]*S1110[1][1] + v10[5]*S1110[1][2] + v10[3]*(-(eff[2].x[2]*S1110[1][1]) + eff[2].x[1]*S1110[1][2]) + v10[6]*S1110[1][3] + v10[2]*(eff[2].x[3]*S1110[1][1] - eff[2].x[1]*S1110[1][3]) + v10[1]*(-(eff[2].x[3]*S1110[1][2]) + eff[2].x[2]*S1110[1][3]);
v11[5]=v10[4]*S1110[2][1] + v10[5]*S1110[2][2] + v10[3]*(-(eff[2].x[2]*S1110[2][1]) + eff[2].x[1]*S1110[2][2]) + v10[6]*S1110[2][3] + v10[2]*(eff[2].x[3]*S1110[2][1] - eff[2].x[1]*S1110[2][3]) + v10[1]*(-(eff[2].x[3]*S1110[2][2]) + eff[2].x[2]*S1110[2][3]);
v11[6]=v10[4]*S1110[3][1] + v10[5]*S1110[3][2] + v10[3]*(-(eff[2].x[2]*S1110[3][1]) + eff[2].x[1]*S1110[3][2]) + v10[6]*S1110[3][3] + v10[2]*(eff[2].x[3]*S1110[3][1] - eff[2].x[1]*S1110[3][3]) + v10[1]*(-(eff[2].x[3]*S1110[3][2]) + eff[2].x[2]*S1110[3][3]);

v12[1]=state[39].thd + v10[1]*S1210[1][1] + v10[2]*S1210[1][2] + v10[3]*S1210[1][3];
v12[2]=v10[1]*S1210[2][1] + v10[2]*S1210[2][2] + v10[3]*S1210[2][3];
v12[3]=v10[1]*S1210[3][1] + v10[2]*S1210[3][2] + v10[3]*S1210[3][3];
v12[4]=v10[4]*S1210[1][1] + v10[5]*S1210[1][2] + v10[3]*(-(YTHUMB*S1210[1][1]) + XTHUMB*S1210[1][2]) + v10[6]*S1210[1][3] + v10[2]*(-(ZTHUMB*S1210[1][1]) - XTHUMB*S1210[1][3]) + v10[1]*(ZTHUMB*S1210[1][2] + YTHUMB*S1210[1][3]);
v12[5]=v10[4]*S1210[2][1] + v10[5]*S1210[2][2] + v10[3]*(-(YTHUMB*S1210[2][1]) + XTHUMB*S1210[2][2]) + v10[6]*S1210[2][3] + v10[2]*(-(ZTHUMB*S1210[2][1]) - XTHUMB*S1210[2][3]) + v10[1]*(ZTHUMB*S1210[2][2] + YTHUMB*S1210[2][3]);
v12[6]=v10[4]*S1210[3][1] + v10[5]*S1210[3][2] + v10[3]*(-(YTHUMB*S1210[3][1]) + XTHUMB*S1210[3][2]) + v10[6]*S1210[3][3] + v10[2]*(-(ZTHUMB*S1210[3][1]) - XTHUMB*S1210[3][3]) + v10[1]*(ZTHUMB*S1210[3][2] + YTHUMB*S1210[3][3]);

v13[1]=v12[1]*S1312[1][1] + v12[2]*S1312[1][2];
v13[2]=v12[1]*S1312[2][1] + v12[2]*S1312[2][2];
v13[3]=-state[40].thd + v12[3];
v13[4]=v12[4]*S1312[1][1] + v12[5]*S1312[1][2] + v12[3]*(-(YTHUMBFLEX*S1312[1][1]) + XTHUMBFLEX*S1312[1][2]);
v13[5]=v12[4]*S1312[2][1] + v12[5]*S1312[2][2] + v12[3]*(-(YTHUMBFLEX*S1312[2][1]) + XTHUMBFLEX*S1312[2][2]);
v13[6]=YTHUMBFLEX*v12[1] - XTHUMBFLEX*v12[2] + v12[6];

v14[1]=v13[1]*S1413[1][1] + v13[2]*S1413[1][2];
v14[2]=v13[1]*S1413[2][1] + v13[2]*S1413[2][2];
v14[3]=v13[3];
v14[4]=v13[4]*S1413[1][1] + TH1SEG*v13[3]*S1413[1][2] + v13[5]*S1413[1][2];
v14[5]=v13[4]*S1413[2][1] + TH1SEG*v13[3]*S1413[2][2] + v13[5]*S1413[2][2];
v14[6]=-(TH1SEG*v13[2]) + v13[6];

v15[1]=v14[1];
v15[2]=v14[2];
v15[3]=v14[3];
v15[4]=v14[4];
v15[5]=TH2SEG*v14[3] + v14[5];
v15[6]=-(TH2SEG*v14[2]) + v14[6];

v16[1]=v10[1]*S1610[1][1] + v10[2]*S1610[1][2] + v10[3]*S1610[1][3];
v16[2]=v10[1]*S1610[2][1] + v10[2]*S1610[2][2] + v10[3]*S1610[2][3];
v16[3]=state[41].thd + v10[2]*S1610[3][2] + v10[3]*S1610[3][3];
v16[4]=v10[4]*S1610[1][1] + v10[5]*S1610[1][2] + v10[3]*(-(YIF*S1610[1][1]) + XIF*S1610[1][2]) + v10[6]*S1610[1][3] + v10[2]*(-(ZIF*S1610[1][1]) - XIF*S1610[1][3]) + v10[1]*(ZIF*S1610[1][2] + YIF*S1610[1][3]);
v16[5]=v10[4]*S1610[2][1] + v10[5]*S1610[2][2] + v10[3]*(-(YIF*S1610[2][1]) + XIF*S1610[2][2]) + v10[6]*S1610[2][3] + v10[2]*(-(ZIF*S1610[2][1]) - XIF*S1610[2][3]) + v10[1]*(ZIF*S1610[2][2] + YIF*S1610[2][3]);
v16[6]=XIF*v10[3]*S1610[3][2] + v10[5]*S1610[3][2] - XIF*v10[2]*S1610[3][3] + v10[6]*S1610[3][3] + v10[1]*(ZIF*S1610[3][2] + YIF*S1610[3][3]);

v17[1]=v16[1]*S1716[1][1] + v16[2]*S1716[1][2];
v17[2]=v16[1]*S1716[2][1] + v16[2]*S1716[2][2];
v17[3]=v16[3];
v17[4]=v16[4]*S1716[1][1] + F1SEG*v16[3]*S1716[1][2] + v16[5]*S1716[1][2];
v17[5]=v16[4]*S1716[2][1] + F1SEG*v16[3]*S1716[2][2] + v16[5]*S1716[2][2];
v17[6]=-(F1SEG*v16[2]) + v16[6];

v18[1]=v17[1]*S1817[1][1] + v17[2]*S1817[1][2];
v18[2]=v17[1]*S1817[2][1] + v17[2]*S1817[2][2];
v18[3]=v17[3];
v18[4]=v17[4]*S1817[1][1] + F2SEG*v17[3]*S1817[1][2] + v17[5]*S1817[1][2];
v18[5]=v17[4]*S1817[2][1] + F2SEG*v17[3]*S1817[2][2] + v17[5]*S1817[2][2];
v18[6]=-(F2SEG*v17[2]) + v17[6];

v19[1]=v18[1];
v19[2]=v18[2];
v19[3]=v18[3];
v19[4]=v18[4];
v19[5]=F3SEG*v18[3] + v18[5];
v19[6]=-(F3SEG*v18[2]) + v18[6];

v20[1]=v10[1]*S2010[1][1] + v10[2]*S2010[1][2] + v10[3]*S2010[1][3];
v20[2]=v10[1]*S2010[2][1] + v10[2]*S2010[2][2] + v10[3]*S2010[2][3];
v20[3]=state[42].thd + v10[2]*S2010[3][2] + v10[3]*S2010[3][3];
v20[4]=v10[4]*S2010[1][1] + v10[5]*S2010[1][2] + v10[3]*(-(YMF*S2010[1][1]) + XMF*S2010[1][2]) + v10[6]*S2010[1][3] + v10[2]*(-(ZMF*S2010[1][1]) - XMF*S2010[1][3]) + v10[1]*(ZMF*S2010[1][2] + YMF*S2010[1][3]);
v20[5]=v10[4]*S2010[2][1] + v10[5]*S2010[2][2] + v10[3]*(-(YMF*S2010[2][1]) + XMF*S2010[2][2]) + v10[6]*S2010[2][3] + v10[2]*(-(ZMF*S2010[2][1]) - XMF*S2010[2][3]) + v10[1]*(ZMF*S2010[2][2] + YMF*S2010[2][3]);
v20[6]=XMF*v10[3]*S2010[3][2] + v10[5]*S2010[3][2] - XMF*v10[2]*S2010[3][3] + v10[6]*S2010[3][3] + v10[1]*(ZMF*S2010[3][2] + YMF*S2010[3][3]);

v21[1]=v20[1]*S2120[1][1] + v20[2]*S2120[1][2];
v21[2]=v20[1]*S2120[2][1] + v20[2]*S2120[2][2];
v21[3]=v20[3];
v21[4]=v20[4]*S2120[1][1] + F1SEG*v20[3]*S2120[1][2] + v20[5]*S2120[1][2];
v21[5]=v20[4]*S2120[2][1] + F1SEG*v20[3]*S2120[2][2] + v20[5]*S2120[2][2];
v21[6]=-(F1SEG*v20[2]) + v20[6];

v22[1]=v21[1]*S2221[1][1] + v21[2]*S2221[1][2];
v22[2]=v21[1]*S2221[2][1] + v21[2]*S2221[2][2];
v22[3]=v21[3];
v22[4]=v21[4]*S2221[1][1] + F2SEG*v21[3]*S2221[1][2] + v21[5]*S2221[1][2];
v22[5]=v21[4]*S2221[2][1] + F2SEG*v21[3]*S2221[2][2] + v21[5]*S2221[2][2];
v22[6]=-(F2SEG*v21[2]) + v21[6];

v23[1]=v22[1];
v23[2]=v22[2];
v23[3]=v22[3];
v23[4]=v22[4];
v23[5]=F3SEG*v22[3] + v22[5];
v23[6]=-(F3SEG*v22[2]) + v22[6];

v24[1]=v10[1]*S2410[1][1] + v10[2]*S2410[1][2] + v10[3]*S2410[1][3];
v24[2]=v10[1]*S2410[2][1] + v10[2]*S2410[2][2] + v10[3]*S2410[2][3];
v24[3]=state[43].thd + v10[2]*S2410[3][2] + v10[3]*S2410[3][3];
v24[4]=v10[4]*S2410[1][1] + v10[5]*S2410[1][2] + v10[3]*(-(YRF*S2410[1][1]) + XRF*S2410[1][2]) + v10[6]*S2410[1][3] + v10[2]*(-(ZRF*S2410[1][1]) - XRF*S2410[1][3]) + v10[1]*(ZRF*S2410[1][2] + YRF*S2410[1][3]);
v24[5]=v10[4]*S2410[2][1] + v10[5]*S2410[2][2] + v10[3]*(-(YRF*S2410[2][1]) + XRF*S2410[2][2]) + v10[6]*S2410[2][3] + v10[2]*(-(ZRF*S2410[2][1]) - XRF*S2410[2][3]) + v10[1]*(ZRF*S2410[2][2] + YRF*S2410[2][3]);
v24[6]=XRF*v10[3]*S2410[3][2] + v10[5]*S2410[3][2] - XRF*v10[2]*S2410[3][3] + v10[6]*S2410[3][3] + v10[1]*(ZRF*S2410[3][2] + YRF*S2410[3][3]);

v25[1]=v24[1]*S2524[1][1] + v24[2]*S2524[1][2];
v25[2]=v24[1]*S2524[2][1] + v24[2]*S2524[2][2];
v25[3]=v24[3];
v25[4]=v24[4]*S2524[1][1] + F1SEG*v24[3]*S2524[1][2] + v24[5]*S2524[1][2];
v25[5]=v24[4]*S2524[2][1] + F1SEG*v24[3]*S2524[2][2] + v24[5]*S2524[2][2];
v25[6]=-(F1SEG*v24[2]) + v24[6];

v26[1]=v25[1]*S2625[1][1] + v25[2]*S2625[1][2];
v26[2]=v25[1]*S2625[2][1] + v25[2]*S2625[2][2];
v26[3]=v25[3];
v26[4]=v25[4]*S2625[1][1] + F2SEG*v25[3]*S2625[1][2] + v25[5]*S2625[1][2];
v26[5]=v25[4]*S2625[2][1] + F2SEG*v25[3]*S2625[2][2] + v25[5]*S2625[2][2];
v26[6]=-(F2SEG*v25[2]) + v25[6];

v27[1]=v26[1];
v27[2]=v26[2];
v27[3]=v26[3];
v27[4]=v26[4];
v27[5]=F3SEG*v26[3] + v26[5];
v27[6]=-(F3SEG*v26[2]) + v26[6];

v28[1]=v10[1]*S2810[1][1] + v10[2]*S2810[1][2] + v10[3]*S2810[1][3];
v28[2]=v10[1]*S2810[2][1] + v10[2]*S2810[2][2] + v10[3]*S2810[2][3];
v28[3]=state[44].thd + v10[2]*S2810[3][2] + v10[3]*S2810[3][3];
v28[4]=v10[4]*S2810[1][1] + v10[5]*S2810[1][2] + v10[3]*(-(YLF*S2810[1][1]) + XLF*S2810[1][2]) + v10[6]*S2810[1][3] + v10[2]*(-(ZLF*S2810[1][1]) - XLF*S2810[1][3]) + v10[1]*(ZLF*S2810[1][2] + YLF*S2810[1][3]);
v28[5]=v10[4]*S2810[2][1] + v10[5]*S2810[2][2] + v10[3]*(-(YLF*S2810[2][1]) + XLF*S2810[2][2]) + v10[6]*S2810[2][3] + v10[2]*(-(ZLF*S2810[2][1]) - XLF*S2810[2][3]) + v10[1]*(ZLF*S2810[2][2] + YLF*S2810[2][3]);
v28[6]=XLF*v10[3]*S2810[3][2] + v10[5]*S2810[3][2] - XLF*v10[2]*S2810[3][3] + v10[6]*S2810[3][3] + v10[1]*(ZLF*S2810[3][2] + YLF*S2810[3][3]);

v29[1]=v28[1]*S2928[1][1] + v28[2]*S2928[1][2];
v29[2]=v28[1]*S2928[2][1] + v28[2]*S2928[2][2];
v29[3]=v28[3];
v29[4]=v28[4]*S2928[1][1] + F1SEG*v28[3]*S2928[1][2] + v28[5]*S2928[1][2];
v29[5]=v28[4]*S2928[2][1] + F1SEG*v28[3]*S2928[2][2] + v28[5]*S2928[2][2];
v29[6]=-(F1SEG*v28[2]) + v28[6];

v30[1]=v29[1]*S3029[1][1] + v29[2]*S3029[1][2];
v30[2]=v29[1]*S3029[2][1] + v29[2]*S3029[2][2];
v30[3]=v29[3];
v30[4]=v29[4]*S3029[1][1] + F2SEG*v29[3]*S3029[1][2] + v29[5]*S3029[1][2];
v30[5]=v29[4]*S3029[2][1] + F2SEG*v29[3]*S3029[2][2] + v29[5]*S3029[2][2];
v30[6]=-(F2SEG*v29[2]) + v29[6];

v31[1]=v30[1];
v31[2]=v30[2];
v31[3]=v30[3];
v31[4]=v30[4];
v31[5]=F3SEG*v30[3] + v30[5];
v31[6]=-(F3SEG*v30[2]) + v30[6];

v32[1]=v3[1]*S323[1][1] + v3[2]*S323[1][2] + v3[3]*S323[1][3];
v32[2]=v3[1]*S323[2][1] + v3[2]*S323[2][2] + v3[3]*S323[2][3];
v32[3]=state[8].thd - 0.7071067811865475*v3[1] - 0.7071067811865475*v3[3];
v32[4]=v3[4]*S323[1][1] - THORAX2SHOULDER*v3[3]*S323[1][2] + v3[5]*S323[1][2] + THORAX2SHOULDER*v3[2]*S323[1][3] + v3[6]*S323[1][3];
v32[5]=v3[4]*S323[2][1] - THORAX2SHOULDER*v3[3]*S323[2][2] + v3[5]*S323[2][2] + THORAX2SHOULDER*v3[2]*S323[2][3] + v3[6]*S323[2][3];
v32[6]=-0.7071067811865475*THORAX2SHOULDER*v3[2] - 0.7071067811865475*v3[4] - 0.7071067811865475*v3[6];

v33[1]=v32[1]*S3332[1][1] + v32[3]*S3332[1][3];
v33[2]=v32[1]*S3332[2][1] + v32[3]*S3332[2][3];
v33[3]=state[9].thd + v32[2];
v33[4]=SHOULDERX*v32[2]*S3332[1][1] + v32[4]*S3332[1][1] + v32[6]*S3332[1][3];
v33[5]=SHOULDERX*v32[2]*S3332[2][1] + v32[4]*S3332[2][1] + v32[6]*S3332[2][3];
v33[6]=-(SHOULDERX*v32[1]) + v32[5];

v34[1]=v33[1]*S3433[1][1] + v33[3]*S3433[1][3];
v34[2]=v33[1]*S3433[2][1] + v33[3]*S3433[2][3];
v34[3]=state[10].thd - v33[2];
v34[4]=v33[4]*S3433[1][1] + SHOULDERY*v33[2]*S3433[1][3] + v33[6]*S3433[1][3];
v34[5]=v33[4]*S3433[2][1] + SHOULDERY*v33[2]*S3433[2][3] + v33[6]*S3433[2][3];
v34[6]=SHOULDERY*v33[3] - v33[5];

v35[1]=v34[2]*S3534[1][2] + v34[3]*S3534[1][3];
v35[2]=v34[2]*S3534[2][2] + v34[3]*S3534[2][3];
v35[3]=state[11].thd - v34[1];
v35[4]=-(UPPERARM*v34[1]*S3534[1][2]) + v34[5]*S3534[1][2] + v34[6]*S3534[1][3];
v35[5]=-(UPPERARM*v34[1]*S3534[2][2]) + v34[5]*S3534[2][2] + v34[6]*S3534[2][3];
v35[6]=-(UPPERARM*v34[2]) - v34[4];

v36[1]=v35[1]*S3635[1][1] + v35[3]*S3635[1][3];
v36[2]=v35[1]*S3635[2][1] + v35[3]*S3635[2][3];
v36[3]=state[12].thd - v35[2];
v36[4]=v35[4]*S3635[1][1] + v35[6]*S3635[1][3];
v36[5]=v35[4]*S3635[2][1] + v35[6]*S3635[2][3];
v36[6]=-v35[5];

v37[1]=v36[2]*S3736[1][2] + v36[3]*S3736[1][3];
v37[2]=v36[2]*S3736[2][2] + v36[3]*S3736[2][3];
v37[3]=state[13].thd - v36[1];
v37[4]=v36[5]*S3736[1][2] + v36[6]*S3736[1][3] + v36[1]*(-(LOWERARM*S3736[1][2]) + WRISTY*S3736[1][3]);
v37[5]=v36[5]*S3736[2][2] + v36[6]*S3736[2][3] + v36[1]*(-(LOWERARM*S3736[2][2]) + WRISTY*S3736[2][3]);
v37[6]=-(LOWERARM*v36[2]) + WRISTY*v36[3] - v36[4];

v38[1]=v37[1]*S3837[1][1] + v37[3]*S3837[1][3];
v38[2]=v37[1]*S3837[2][1] + v37[3]*S3837[2][3];
v38[3]=state[14].thd + v37[2];
v38[4]=v37[4]*S3837[1][1] + v37[6]*S3837[1][3];
v38[5]=v37[4]*S3837[2][1] + v37[6]*S3837[2][3];
v38[6]=v37[5];

v39[1]=v38[1]*S3938[1][1] + v38[2]*S3938[1][2] + v38[3]*S3938[1][3];
v39[2]=v38[1]*S3938[2][1] + v38[2]*S3938[2][2] + v38[3]*S3938[2][3];
v39[3]=v38[1]*S3938[3][1] + v38[2]*S3938[3][2] + v38[3]*S3938[3][3];
v39[4]=v38[4]*S3938[1][1] + v38[5]*S3938[1][2] + v38[3]*(-(eff[1].x[2]*S3938[1][1]) + eff[1].x[1]*S3938[1][2]) + v38[6]*S3938[1][3] + v38[2]*(eff[1].x[3]*S3938[1][1] - eff[1].x[1]*S3938[1][3]) + v38[1]*(-(eff[1].x[3]*S3938[1][2]) + eff[1].x[2]*S3938[1][3]);
v39[5]=v38[4]*S3938[2][1] + v38[5]*S3938[2][2] + v38[3]*(-(eff[1].x[2]*S3938[2][1]) + eff[1].x[1]*S3938[2][2]) + v38[6]*S3938[2][3] + v38[2]*(eff[1].x[3]*S3938[2][1] - eff[1].x[1]*S3938[2][3]) + v38[1]*(-(eff[1].x[3]*S3938[2][2]) + eff[1].x[2]*S3938[2][3]);
v39[6]=v38[4]*S3938[3][1] + v38[5]*S3938[3][2] + v38[3]*(-(eff[1].x[2]*S3938[3][1]) + eff[1].x[1]*S3938[3][2]) + v38[6]*S3938[3][3] + v38[2]*(eff[1].x[3]*S3938[3][1] - eff[1].x[1]*S3938[3][3]) + v38[1]*(-(eff[1].x[3]*S3938[3][2]) + eff[1].x[2]*S3938[3][3]);

v40[1]=state[45].thd + v38[1]*S4038[1][1] + v38[2]*S4038[1][2] + v38[3]*S4038[1][3];
v40[2]=v38[1]*S4038[2][1] + v38[2]*S4038[2][2] + v38[3]*S4038[2][3];
v40[3]=v38[1]*S4038[3][1] + v38[2]*S4038[3][2] + v38[3]*S4038[3][3];
v40[4]=v38[4]*S4038[1][1] + v38[5]*S4038[1][2] + v38[3]*(-(YTHUMB*S4038[1][1]) + XTHUMB*S4038[1][2]) + v38[6]*S4038[1][3] + v38[2]*(ZTHUMB*S4038[1][1] - XTHUMB*S4038[1][3]) + v38[1]*(-(ZTHUMB*S4038[1][2]) + YTHUMB*S4038[1][3]);
v40[5]=v38[4]*S4038[2][1] + v38[5]*S4038[2][2] + v38[3]*(-(YTHUMB*S4038[2][1]) + XTHUMB*S4038[2][2]) + v38[6]*S4038[2][3] + v38[2]*(ZTHUMB*S4038[2][1] - XTHUMB*S4038[2][3]) + v38[1]*(-(ZTHUMB*S4038[2][2]) + YTHUMB*S4038[2][3]);
v40[6]=v38[4]*S4038[3][1] + v38[5]*S4038[3][2] + v38[3]*(-(YTHUMB*S4038[3][1]) + XTHUMB*S4038[3][2]) + v38[6]*S4038[3][3] + v38[2]*(ZTHUMB*S4038[3][1] - XTHUMB*S4038[3][3]) + v38[1]*(-(ZTHUMB*S4038[3][2]) + YTHUMB*S4038[3][3]);

v41[1]=v40[1]*S4140[1][1] + v40[2]*S4140[1][2];
v41[2]=v40[1]*S4140[2][1] + v40[2]*S4140[2][2];
v41[3]=-state[46].thd + v40[3];
v41[4]=v40[4]*S4140[1][1] + v40[5]*S4140[1][2] + v40[3]*(-(YTHUMBFLEX*S4140[1][1]) + XTHUMBFLEX*S4140[1][2]);
v41[5]=v40[4]*S4140[2][1] + v40[5]*S4140[2][2] + v40[3]*(-(YTHUMBFLEX*S4140[2][1]) + XTHUMBFLEX*S4140[2][2]);
v41[6]=YTHUMBFLEX*v40[1] - XTHUMBFLEX*v40[2] + v40[6];

v42[1]=v41[1]*S4241[1][1] + v41[2]*S4241[1][2];
v42[2]=v41[1]*S4241[2][1] + v41[2]*S4241[2][2];
v42[3]=v41[3];
v42[4]=v41[4]*S4241[1][1] + TH1SEG*v41[3]*S4241[1][2] + v41[5]*S4241[1][2];
v42[5]=v41[4]*S4241[2][1] + TH1SEG*v41[3]*S4241[2][2] + v41[5]*S4241[2][2];
v42[6]=-(TH1SEG*v41[2]) + v41[6];

v43[1]=v42[1];
v43[2]=v42[2];
v43[3]=v42[3];
v43[4]=v42[4];
v43[5]=TH2SEG*v42[3] + v42[5];
v43[6]=-(TH2SEG*v42[2]) + v42[6];

v44[1]=v38[1]*S4438[1][1] + v38[2]*S4438[1][2] + v38[3]*S4438[1][3];
v44[2]=v38[1]*S4438[2][1] + v38[2]*S4438[2][2] + v38[3]*S4438[2][3];
v44[3]=state[47].thd + v38[2]*S4438[3][2] + v38[3]*S4438[3][3];
v44[4]=v38[4]*S4438[1][1] + v38[5]*S4438[1][2] + v38[3]*(-(YIF*S4438[1][1]) + XIF*S4438[1][2]) + v38[6]*S4438[1][3] + v38[2]*(ZIF*S4438[1][1] - XIF*S4438[1][3]) + v38[1]*(-(ZIF*S4438[1][2]) + YIF*S4438[1][3]);
v44[5]=v38[4]*S4438[2][1] + v38[5]*S4438[2][2] + v38[3]*(-(YIF*S4438[2][1]) + XIF*S4438[2][2]) + v38[6]*S4438[2][3] + v38[2]*(ZIF*S4438[2][1] - XIF*S4438[2][3]) + v38[1]*(-(ZIF*S4438[2][2]) + YIF*S4438[2][3]);
v44[6]=XIF*v38[3]*S4438[3][2] + v38[5]*S4438[3][2] - XIF*v38[2]*S4438[3][3] + v38[6]*S4438[3][3] + v38[1]*(-(ZIF*S4438[3][2]) + YIF*S4438[3][3]);

v45[1]=v44[1]*S4544[1][1] + v44[2]*S4544[1][2];
v45[2]=v44[1]*S4544[2][1] + v44[2]*S4544[2][2];
v45[3]=v44[3];
v45[4]=v44[4]*S4544[1][1] + F1SEG*v44[3]*S4544[1][2] + v44[5]*S4544[1][2];
v45[5]=v44[4]*S4544[2][1] + F1SEG*v44[3]*S4544[2][2] + v44[5]*S4544[2][2];
v45[6]=-(F1SEG*v44[2]) + v44[6];

v46[1]=v45[1]*S4645[1][1] + v45[2]*S4645[1][2];
v46[2]=v45[1]*S4645[2][1] + v45[2]*S4645[2][2];
v46[3]=v45[3];
v46[4]=v45[4]*S4645[1][1] + F2SEG*v45[3]*S4645[1][2] + v45[5]*S4645[1][2];
v46[5]=v45[4]*S4645[2][1] + F2SEG*v45[3]*S4645[2][2] + v45[5]*S4645[2][2];
v46[6]=-(F2SEG*v45[2]) + v45[6];

v47[1]=v46[1];
v47[2]=v46[2];
v47[3]=v46[3];
v47[4]=v46[4];
v47[5]=F3SEG*v46[3] + v46[5];
v47[6]=-(F3SEG*v46[2]) + v46[6];

v48[1]=v38[1]*S4838[1][1] + v38[2]*S4838[1][2] + v38[3]*S4838[1][3];
v48[2]=v38[1]*S4838[2][1] + v38[2]*S4838[2][2] + v38[3]*S4838[2][3];
v48[3]=state[48].thd + v38[2]*S4838[3][2] + v38[3]*S4838[3][3];
v48[4]=v38[4]*S4838[1][1] + v38[5]*S4838[1][2] + v38[3]*(-(YMF*S4838[1][1]) + XMF*S4838[1][2]) + v38[6]*S4838[1][3] + v38[2]*(ZMF*S4838[1][1] - XMF*S4838[1][3]) + v38[1]*(-(ZMF*S4838[1][2]) + YMF*S4838[1][3]);
v48[5]=v38[4]*S4838[2][1] + v38[5]*S4838[2][2] + v38[3]*(-(YMF*S4838[2][1]) + XMF*S4838[2][2]) + v38[6]*S4838[2][3] + v38[2]*(ZMF*S4838[2][1] - XMF*S4838[2][3]) + v38[1]*(-(ZMF*S4838[2][2]) + YMF*S4838[2][3]);
v48[6]=XMF*v38[3]*S4838[3][2] + v38[5]*S4838[3][2] - XMF*v38[2]*S4838[3][3] + v38[6]*S4838[3][3] + v38[1]*(-(ZMF*S4838[3][2]) + YMF*S4838[3][3]);

v49[1]=v48[1]*S4948[1][1] + v48[2]*S4948[1][2];
v49[2]=v48[1]*S4948[2][1] + v48[2]*S4948[2][2];
v49[3]=v48[3];
v49[4]=v48[4]*S4948[1][1] + F1SEG*v48[3]*S4948[1][2] + v48[5]*S4948[1][2];
v49[5]=v48[4]*S4948[2][1] + F1SEG*v48[3]*S4948[2][2] + v48[5]*S4948[2][2];
v49[6]=-(F1SEG*v48[2]) + v48[6];

v50[1]=v49[1]*S5049[1][1] + v49[2]*S5049[1][2];
v50[2]=v49[1]*S5049[2][1] + v49[2]*S5049[2][2];
v50[3]=v49[3];
v50[4]=v49[4]*S5049[1][1] + F2SEG*v49[3]*S5049[1][2] + v49[5]*S5049[1][2];
v50[5]=v49[4]*S5049[2][1] + F2SEG*v49[3]*S5049[2][2] + v49[5]*S5049[2][2];
v50[6]=-(F2SEG*v49[2]) + v49[6];

v51[1]=v50[1];
v51[2]=v50[2];
v51[3]=v50[3];
v51[4]=v50[4];
v51[5]=F3SEG*v50[3] + v50[5];
v51[6]=-(F3SEG*v50[2]) + v50[6];

v52[1]=v38[1]*S5238[1][1] + v38[2]*S5238[1][2] + v38[3]*S5238[1][3];
v52[2]=v38[1]*S5238[2][1] + v38[2]*S5238[2][2] + v38[3]*S5238[2][3];
v52[3]=state[49].thd + v38[2]*S5238[3][2] + v38[3]*S5238[3][3];
v52[4]=v38[4]*S5238[1][1] + v38[5]*S5238[1][2] + v38[3]*(-(YRF*S5238[1][1]) + XRF*S5238[1][2]) + v38[6]*S5238[1][3] + v38[2]*(ZRF*S5238[1][1] - XRF*S5238[1][3]) + v38[1]*(-(ZRF*S5238[1][2]) + YRF*S5238[1][3]);
v52[5]=v38[4]*S5238[2][1] + v38[5]*S5238[2][2] + v38[3]*(-(YRF*S5238[2][1]) + XRF*S5238[2][2]) + v38[6]*S5238[2][3] + v38[2]*(ZRF*S5238[2][1] - XRF*S5238[2][3]) + v38[1]*(-(ZRF*S5238[2][2]) + YRF*S5238[2][3]);
v52[6]=XRF*v38[3]*S5238[3][2] + v38[5]*S5238[3][2] - XRF*v38[2]*S5238[3][3] + v38[6]*S5238[3][3] + v38[1]*(-(ZRF*S5238[3][2]) + YRF*S5238[3][3]);

v53[1]=v52[1]*S5352[1][1] + v52[2]*S5352[1][2];
v53[2]=v52[1]*S5352[2][1] + v52[2]*S5352[2][2];
v53[3]=v52[3];
v53[4]=v52[4]*S5352[1][1] + F1SEG*v52[3]*S5352[1][2] + v52[5]*S5352[1][2];
v53[5]=v52[4]*S5352[2][1] + F1SEG*v52[3]*S5352[2][2] + v52[5]*S5352[2][2];
v53[6]=-(F1SEG*v52[2]) + v52[6];

v54[1]=v53[1]*S5453[1][1] + v53[2]*S5453[1][2];
v54[2]=v53[1]*S5453[2][1] + v53[2]*S5453[2][2];
v54[3]=v53[3];
v54[4]=v53[4]*S5453[1][1] + F2SEG*v53[3]*S5453[1][2] + v53[5]*S5453[1][2];
v54[5]=v53[4]*S5453[2][1] + F2SEG*v53[3]*S5453[2][2] + v53[5]*S5453[2][2];
v54[6]=-(F2SEG*v53[2]) + v53[6];

v55[1]=v54[1];
v55[2]=v54[2];
v55[3]=v54[3];
v55[4]=v54[4];
v55[5]=F3SEG*v54[3] + v54[5];
v55[6]=-(F3SEG*v54[2]) + v54[6];

v56[1]=v38[1]*S5638[1][1] + v38[2]*S5638[1][2] + v38[3]*S5638[1][3];
v56[2]=v38[1]*S5638[2][1] + v38[2]*S5638[2][2] + v38[3]*S5638[2][3];
v56[3]=state[50].thd + v38[2]*S5638[3][2] + v38[3]*S5638[3][3];
v56[4]=v38[4]*S5638[1][1] + v38[5]*S5638[1][2] + v38[3]*(-(YLF*S5638[1][1]) + XLF*S5638[1][2]) + v38[6]*S5638[1][3] + v38[2]*(ZLF*S5638[1][1] - XLF*S5638[1][3]) + v38[1]*(-(ZLF*S5638[1][2]) + YLF*S5638[1][3]);
v56[5]=v38[4]*S5638[2][1] + v38[5]*S5638[2][2] + v38[3]*(-(YLF*S5638[2][1]) + XLF*S5638[2][2]) + v38[6]*S5638[2][3] + v38[2]*(ZLF*S5638[2][1] - XLF*S5638[2][3]) + v38[1]*(-(ZLF*S5638[2][2]) + YLF*S5638[2][3]);
v56[6]=XLF*v38[3]*S5638[3][2] + v38[5]*S5638[3][2] - XLF*v38[2]*S5638[3][3] + v38[6]*S5638[3][3] + v38[1]*(-(ZLF*S5638[3][2]) + YLF*S5638[3][3]);

v57[1]=v56[1]*S5756[1][1] + v56[2]*S5756[1][2];
v57[2]=v56[1]*S5756[2][1] + v56[2]*S5756[2][2];
v57[3]=v56[3];
v57[4]=v56[4]*S5756[1][1] + F1SEG*v56[3]*S5756[1][2] + v56[5]*S5756[1][2];
v57[5]=v56[4]*S5756[2][1] + F1SEG*v56[3]*S5756[2][2] + v56[5]*S5756[2][2];
v57[6]=-(F1SEG*v56[2]) + v56[6];

v58[1]=v57[1]*S5857[1][1] + v57[2]*S5857[1][2];
v58[2]=v57[1]*S5857[2][1] + v57[2]*S5857[2][2];
v58[3]=v57[3];
v58[4]=v57[4]*S5857[1][1] + F2SEG*v57[3]*S5857[1][2] + v57[5]*S5857[1][2];
v58[5]=v57[4]*S5857[2][1] + F2SEG*v57[3]*S5857[2][2] + v57[5]*S5857[2][2];
v58[6]=-(F2SEG*v57[2]) + v57[6];

v59[1]=v58[1];
v59[2]=v58[2];
v59[3]=v58[3];
v59[4]=v58[4];
v59[5]=F3SEG*v58[3] + v58[5];
v59[6]=-(F3SEG*v58[2]) + v58[6];

v60[1]=v3[1]*S603[1][1] + v3[2]*S603[1][2];
v60[2]=v3[1]*S603[2][1] + v3[2]*S603[2][2];
v60[3]=state[32].thd + v3[3];
v60[4]=v3[4]*S603[1][1] - THORAX2NECK*v3[3]*S603[1][2] + v3[5]*S603[1][2];
v60[5]=v3[4]*S603[2][1] - THORAX2NECK*v3[3]*S603[2][2] + v3[5]*S603[2][2];
v60[6]=THORAX2NECK*v3[2] + v3[6];

v61[1]=v60[2]*S6160[1][2] + v60[3]*S6160[1][3];
v61[2]=v60[2]*S6160[2][2] + v60[3]*S6160[2][3];
v61[3]=state[33].thd + v60[1];
v61[4]=v60[5]*S6160[1][2] - CERVICAL*v60[1]*S6160[1][3] + v60[6]*S6160[1][3];
v61[5]=v60[5]*S6160[2][2] - CERVICAL*v60[1]*S6160[2][3] + v60[6]*S6160[2][3];
v61[6]=CERVICAL*v60[3] + v60[4];

v62[1]=v61[1]*S6261[1][1] + v61[3]*S6261[1][3];
v62[2]=v61[1]*S6261[2][1] + v61[3]*S6261[2][3];
v62[3]=state[34].thd + v61[2];
v62[4]=v61[4]*S6261[1][1] + v61[6]*S6261[1][3];
v62[5]=v61[4]*S6261[2][1] + v61[6]*S6261[2][3];
v62[6]=v61[5];

v63[1]=v62[1]*S6362[1][1] + v62[2]*S6362[1][2];
v63[2]=v62[1]*S6362[2][1] + v62[2]*S6362[2][2];
v63[3]=state[35].thd + v62[3];
v63[4]=-(HEAD*v62[2]*S6362[1][1]) + v62[4]*S6362[1][1] + HEAD*v62[1]*S6362[1][2] + v62[5]*S6362[1][2] + v62[3]*(EYEYOFF*S6362[1][1] + EYEXOFF*S6362[1][2]);
v63[5]=-(HEAD*v62[2]*S6362[2][1]) + v62[4]*S6362[2][1] + HEAD*v62[1]*S6362[2][2] + v62[5]*S6362[2][2] + v62[3]*(EYEYOFF*S6362[2][1] + EYEXOFF*S6362[2][2]);
v63[6]=-(EYEYOFF*v62[1]) - EYEXOFF*v62[2] + v62[6];

v64[1]=v63[2]*S6463[1][2] + v63[3]*S6463[1][3];
v64[2]=v63[2]*S6463[2][2] + v63[3]*S6463[2][3];
v64[3]=state[36].thd - v63[1];
v64[4]=v63[5]*S6463[1][2] + v63[6]*S6463[1][3];
v64[5]=v63[5]*S6463[2][2] + v63[6]*S6463[2][3];
v64[6]=-v63[4];

v65[1]=-v64[3];
v65[2]=-v64[2];
v65[3]=-v64[1];
v65[4]=EYE*v64[1] - v64[6];
v65[5]=-v64[5];
v65[6]=-(EYE*v64[3]) - v64[4];

v66[1]=v62[1]*S6662[1][1] + v62[2]*S6662[1][2];
v66[2]=v62[1]*S6662[2][1] + v62[2]*S6662[2][2];
v66[3]=state[37].thd + v62[3];
v66[4]=-(HEAD*v62[2]*S6662[1][1]) + v62[4]*S6662[1][1] + HEAD*v62[1]*S6662[1][2] + v62[5]*S6662[1][2] + v62[3]*(EYEYOFF*S6662[1][1] - EYEXOFF*S6662[1][2]);
v66[5]=-(HEAD*v62[2]*S6662[2][1]) + v62[4]*S6662[2][1] + HEAD*v62[1]*S6662[2][2] + v62[5]*S6662[2][2] + v62[3]*(EYEYOFF*S6662[2][1] - EYEXOFF*S6662[2][2]);
v66[6]=-(EYEYOFF*v62[1]) + EYEXOFF*v62[2] + v62[6];

v67[1]=v66[2]*S6766[1][2] + v66[3]*S6766[1][3];
v67[2]=v66[2]*S6766[2][2] + v66[3]*S6766[2][3];
v67[3]=state[38].thd - v66[1];
v67[4]=v66[5]*S6766[1][2] + v66[6]*S6766[1][3];
v67[5]=v66[5]*S6766[2][2] + v66[6]*S6766[2][3];
v67[6]=-v66[4];

v68[1]=-v67[3];
v68[2]=-v67[2];
v68[3]=-v67[1];
v68[4]=EYE*v67[1] - v67[6];
v68[5]=-v67[5];
v68[6]=-(EYE*v67[3]) - v67[4];

v69[1]=v62[1];
v69[2]=-v62[2];
v69[3]=-v62[3];
v69[4]=-(TOPofHEAD*v62[2]) + v62[4];
v69[5]=-(TOPofHEAD*v62[1]) - v62[5];
v69[6]=-v62[6];

v70[1]=v0[1]*S700[1][1] + v0[3]*S700[1][3];
v70[2]=v0[1]*S700[2][1] + v0[3]*S700[2][3];
v70[3]=state[23].thd + v0[2];
v70[4]=v0[4]*S700[1][1] - XHIP*v0[2]*S700[1][3] + v0[6]*S700[1][3];
v70[5]=v0[4]*S700[2][1] - XHIP*v0[2]*S700[2][3] + v0[6]*S700[2][3];
v70[6]=XHIP*v0[3] + v0[5];

v71[1]=v70[1]*S7170[1][1] + v70[3]*S7170[1][3];
v71[2]=v70[1]*S7170[2][1] + v70[3]*S7170[2][3];
v71[3]=state[22].thd - v70[2];
v71[4]=v70[4]*S7170[1][1] + v70[6]*S7170[1][3];
v71[5]=v70[4]*S7170[2][1] + v70[6]*S7170[2][3];
v71[6]=-v70[5];

v72[1]=v71[1]*S7271[1][1] + v71[2]*S7271[1][2] + v71[3]*S7271[1][3];
v72[2]=v71[1]*S7271[2][1] + v71[2]*S7271[2][2] + v71[3]*S7271[2][3];
v72[3]=state[24].thd + v71[1]*S7271[3][1] + v71[2]*S7271[3][2];
v72[4]=v71[4]*S7271[1][1] + YHIP*v71[3]*S7271[1][2] + v71[5]*S7271[1][2] - YHIP*v71[2]*S7271[1][3] + v71[6]*S7271[1][3];
v72[5]=v71[4]*S7271[2][1] + YHIP*v71[3]*S7271[2][2] + v71[5]*S7271[2][2] - YHIP*v71[2]*S7271[2][3] + v71[6]*S7271[2][3];
v72[6]=v71[4]*S7271[3][1] + YHIP*v71[3]*S7271[3][2] + v71[5]*S7271[3][2];

v73[1]=v72[1]*S7372[1][1] + v72[3]*S7372[1][3];
v73[2]=v72[1]*S7372[2][1] + v72[3]*S7372[2][3];
v73[3]=state[25].thd + v72[2];
v73[4]=v72[4]*S7372[1][1] + v72[6]*S7372[1][3] + v72[2]*(UPPERLEGMOD*S7372[1][1] - YKNEE*S7372[1][3]);
v73[5]=v72[4]*S7372[2][1] + v72[6]*S7372[2][3] + v72[2]*(UPPERLEGMOD*S7372[2][1] - YKNEE*S7372[2][3]);
v73[6]=-(UPPERLEGMOD*v72[1]) + YKNEE*v72[3] + v72[5];

v74[1]=v73[1]*S7473[1][1] + v73[3]*S7473[1][3];
v74[2]=v73[1]*S7473[2][1] + v73[3]*S7473[2][3];
v74[3]=state[26].thd - v73[2];
v74[4]=v73[4]*S7473[1][1] + v73[6]*S7473[1][3];
v74[5]=v73[4]*S7473[2][1] + v73[6]*S7473[2][3];
v74[6]=-v73[5];

v75[1]=v74[1]*S7574[1][1] + v74[3]*S7574[1][3];
v75[2]=v74[1]*S7574[2][1] + v74[3]*S7574[2][3];
v75[3]=state[27].thd - v74[2];
v75[4]=LOWERLEG*v74[2]*S7574[1][1] + v74[4]*S7574[1][1] + v74[6]*S7574[1][3];
v75[5]=LOWERLEG*v74[2]*S7574[2][1] + v74[4]*S7574[2][1] + v74[6]*S7574[2][3];
v75[6]=LOWERLEG*v74[1] - v74[5];

v76[1]=v75[1]*S7675[1][1] + v75[3]*S7675[1][3];
v76[2]=v75[1]*S7675[2][1] + v75[3]*S7675[2][3];
v76[3]=state[28].thd + v75[2];
v76[4]=v75[4]*S7675[1][1] + v75[6]*S7675[1][3];
v76[5]=v75[4]*S7675[2][1] + v75[6]*S7675[2][3];
v76[6]=v75[5];

v77[1]=v76[1];
v77[2]=v76[2];
v77[3]=v76[3];
v77[4]=YTOE*v76[2] + XTOEOUTER*v76[3] + v76[4];
v77[5]=-(YTOE*v76[1]) + ZTOE*v76[3] + v76[5];
v77[6]=-(XTOEOUTER*v76[1]) - ZTOE*v76[2] + v76[6];

v78[1]=v76[1];
v78[2]=v76[2];
v78[3]=v76[3];
v78[4]=YTOE*v76[2] - XTOEINNER*v76[3] + v76[4];
v78[5]=-(YTOE*v76[1]) + ZTOE*v76[3] + v76[5];
v78[6]=XTOEINNER*v76[1] - ZTOE*v76[2] + v76[6];

v79[1]=v76[1];
v79[2]=v76[2];
v79[3]=v76[3];
v79[4]=YMETATARSAL*v76[2] + XMETATARSALOUTER*v76[3] + v76[4];
v79[5]=-(YMETATARSAL*v76[1]) + ZTOE*v76[3] + v76[5];
v79[6]=-(XMETATARSALOUTER*v76[1]) - ZTOE*v76[2] + v76[6];

v80[1]=v76[1];
v80[2]=v76[2];
v80[3]=v76[3];
v80[4]=YMETATARSAL*v76[2] - XMETATARSALINNER*v76[3] + v76[4];
v80[5]=-(YMETATARSAL*v76[1]) + ZTOE*v76[3] + v76[5];
v80[6]=XMETATARSALINNER*v76[1] - ZTOE*v76[2] + v76[6];

v81[1]=v76[1];
v81[2]=v76[2];
v81[3]=v76[3];
v81[4]=-(YHEEL*v76[2]) + XHEELOUTER*v76[3] + v76[4];
v81[5]=YHEEL*v76[1] + ZHEEL*v76[3] + v76[5];
v81[6]=-(XHEELOUTER*v76[1]) - ZHEEL*v76[2] + v76[6];

v82[1]=v76[1];
v82[2]=v76[2];
v82[3]=v76[3];
v82[4]=-(YHEEL*v76[2]) - XHEELINNER*v76[3] + v76[4];
v82[5]=YHEEL*v76[1] + ZHEEL*v76[3] + v76[5];
v82[6]=XHEELINNER*v76[1] - ZHEEL*v76[2] + v76[6];

v83[1]=v76[1]*S8376[1][1] + v76[2]*S8376[1][2] + v76[3]*S8376[1][3];
v83[2]=v76[1]*S8376[2][1] + v76[2]*S8376[2][2] + v76[3]*S8376[2][3];
v83[3]=v76[1]*S8376[3][1] + v76[2]*S8376[3][2] + v76[3]*S8376[3][3];
v83[4]=v76[4]*S8376[1][1] + v76[5]*S8376[1][2] + v76[3]*(-(eff[3].x[2]*S8376[1][1]) + eff[3].x[1]*S8376[1][2]) + v76[6]*S8376[1][3] + v76[2]*(eff[3].x[3]*S8376[1][1] - eff[3].x[1]*S8376[1][3]) + v76[1]*(-(eff[3].x[3]*S8376[1][2]) + eff[3].x[2]*S8376[1][3]);
v83[5]=v76[4]*S8376[2][1] + v76[5]*S8376[2][2] + v76[3]*(-(eff[3].x[2]*S8376[2][1]) + eff[3].x[1]*S8376[2][2]) + v76[6]*S8376[2][3] + v76[2]*(eff[3].x[3]*S8376[2][1] - eff[3].x[1]*S8376[2][3]) + v76[1]*(-(eff[3].x[3]*S8376[2][2]) + eff[3].x[2]*S8376[2][3]);
v83[6]=v76[4]*S8376[3][1] + v76[5]*S8376[3][2] + v76[3]*(-(eff[3].x[2]*S8376[3][1]) + eff[3].x[1]*S8376[3][2]) + v76[6]*S8376[3][3] + v76[2]*(eff[3].x[3]*S8376[3][1] - eff[3].x[1]*S8376[3][3]) + v76[1]*(-(eff[3].x[3]*S8376[3][2]) + eff[3].x[2]*S8376[3][3]);

v84[1]=v0[1]*S840[1][1] + v0[3]*S840[1][3];
v84[2]=v0[1]*S840[2][1] + v0[3]*S840[2][3];
v84[3]=state[16].thd - v0[2];
v84[4]=v0[4]*S840[1][1] + XHIP*v0[2]*S840[1][3] + v0[6]*S840[1][3];
v84[5]=v0[4]*S840[2][1] + XHIP*v0[2]*S840[2][3] + v0[6]*S840[2][3];
v84[6]=XHIP*v0[3] - v0[5];

v85[1]=v84[1]*S8584[1][1] + v84[3]*S8584[1][3];
v85[2]=v84[1]*S8584[2][1] + v84[3]*S8584[2][3];
v85[3]=state[15].thd + v84[2];
v85[4]=v84[4]*S8584[1][1] + v84[6]*S8584[1][3];
v85[5]=v84[4]*S8584[2][1] + v84[6]*S8584[2][3];
v85[6]=v84[5];

v86[1]=v85[1]*S8685[1][1] + v85[2]*S8685[1][2] + v85[3]*S8685[1][3];
v86[2]=v85[1]*S8685[2][1] + v85[2]*S8685[2][2] + v85[3]*S8685[2][3];
v86[3]=state[17].thd + v85[1]*S8685[3][1] + v85[2]*S8685[3][2];
v86[4]=v85[4]*S8685[1][1] + YHIP*v85[3]*S8685[1][2] + v85[5]*S8685[1][2] - YHIP*v85[2]*S8685[1][3] + v85[6]*S8685[1][3];
v86[5]=v85[4]*S8685[2][1] + YHIP*v85[3]*S8685[2][2] + v85[5]*S8685[2][2] - YHIP*v85[2]*S8685[2][3] + v85[6]*S8685[2][3];
v86[6]=v85[4]*S8685[3][1] + YHIP*v85[3]*S8685[3][2] + v85[5]*S8685[3][2];

v87[1]=v86[1]*S8786[1][1] + v86[3]*S8786[1][3];
v87[2]=v86[1]*S8786[2][1] + v86[3]*S8786[2][3];
v87[3]=state[18].thd - v86[2];
v87[4]=v86[4]*S8786[1][1] + v86[6]*S8786[1][3] + v86[2]*(-(UPPERLEGMOD*S8786[1][1]) - YKNEE*S8786[1][3]);
v87[5]=v86[4]*S8786[2][1] + v86[6]*S8786[2][3] + v86[2]*(-(UPPERLEGMOD*S8786[2][1]) - YKNEE*S8786[2][3]);
v87[6]=-(UPPERLEGMOD*v86[1]) - YKNEE*v86[3] - v86[5];

v88[1]=v87[1]*S8887[1][1] + v87[3]*S8887[1][3];
v88[2]=v87[1]*S8887[2][1] + v87[3]*S8887[2][3];
v88[3]=state[19].thd + v87[2];
v88[4]=v87[4]*S8887[1][1] + v87[6]*S8887[1][3];
v88[5]=v87[4]*S8887[2][1] + v87[6]*S8887[2][3];
v88[6]=v87[5];

v89[1]=v88[1]*S8988[1][1] + v88[3]*S8988[1][3];
v89[2]=v88[1]*S8988[2][1] + v88[3]*S8988[2][3];
v89[3]=state[20].thd + v88[2];
v89[4]=-(LOWERLEG*v88[2]*S8988[1][1]) + v88[4]*S8988[1][1] + v88[6]*S8988[1][3];
v89[5]=-(LOWERLEG*v88[2]*S8988[2][1]) + v88[4]*S8988[2][1] + v88[6]*S8988[2][3];
v89[6]=LOWERLEG*v88[1] + v88[5];

v90[1]=v89[1]*S9089[1][1] + v89[3]*S9089[1][3];
v90[2]=v89[1]*S9089[2][1] + v89[3]*S9089[2][3];
v90[3]=state[21].thd - v89[2];
v90[4]=v89[4]*S9089[1][1] + v89[6]*S9089[1][3];
v90[5]=v89[4]*S9089[2][1] + v89[6]*S9089[2][3];
v90[6]=-v89[5];

v91[1]=v90[1];
v91[2]=v90[2];
v91[3]=v90[3];
v91[4]=-(YTOE*v90[2]) + XTOEOUTER*v90[3] + v90[4];
v91[5]=YTOE*v90[1] + ZTOE*v90[3] + v90[5];
v91[6]=-(XTOEOUTER*v90[1]) - ZTOE*v90[2] + v90[6];

v92[1]=v90[1];
v92[2]=v90[2];
v92[3]=v90[3];
v92[4]=-(YTOE*v90[2]) - XTOEINNER*v90[3] + v90[4];
v92[5]=YTOE*v90[1] + ZTOE*v90[3] + v90[5];
v92[6]=XTOEINNER*v90[1] - ZTOE*v90[2] + v90[6];

v93[1]=v90[1];
v93[2]=v90[2];
v93[3]=v90[3];
v93[4]=-(YMETATARSAL*v90[2]) + XMETATARSALOUTER*v90[3] + v90[4];
v93[5]=YMETATARSAL*v90[1] + ZTOE*v90[3] + v90[5];
v93[6]=-(XMETATARSALOUTER*v90[1]) - ZTOE*v90[2] + v90[6];

v94[1]=v90[1];
v94[2]=v90[2];
v94[3]=v90[3];
v94[4]=-(YMETATARSAL*v90[2]) - XMETATARSALINNER*v90[3] + v90[4];
v94[5]=YMETATARSAL*v90[1] + ZTOE*v90[3] + v90[5];
v94[6]=XMETATARSALINNER*v90[1] - ZTOE*v90[2] + v90[6];

v95[1]=v90[1];
v95[2]=v90[2];
v95[3]=v90[3];
v95[4]=YHEEL*v90[2] + XHEELOUTER*v90[3] + v90[4];
v95[5]=-(YHEEL*v90[1]) + ZHEEL*v90[3] + v90[5];
v95[6]=-(XHEELOUTER*v90[1]) - ZHEEL*v90[2] + v90[6];

v96[1]=v90[1];
v96[2]=v90[2];
v96[3]=v90[3];
v96[4]=YHEEL*v90[2] - XHEELINNER*v90[3] + v90[4];
v96[5]=-(YHEEL*v90[1]) + ZHEEL*v90[3] + v90[5];
v96[6]=XHEELINNER*v90[1] - ZHEEL*v90[2] + v90[6];

v97[1]=v90[1]*S9790[1][1] + v90[2]*S9790[1][2] + v90[3]*S9790[1][3];
v97[2]=v90[1]*S9790[2][1] + v90[2]*S9790[2][2] + v90[3]*S9790[2][3];
v97[3]=v90[1]*S9790[3][1] + v90[2]*S9790[3][2] + v90[3]*S9790[3][3];
v97[4]=v90[4]*S9790[1][1] + v90[5]*S9790[1][2] + v90[3]*(-(eff[4].x[2]*S9790[1][1]) + eff[4].x[1]*S9790[1][2]) + v90[6]*S9790[1][3] + v90[2]*(eff[4].x[3]*S9790[1][1] - eff[4].x[1]*S9790[1][3]) + v90[1]*(-(eff[4].x[3]*S9790[1][2]) + eff[4].x[2]*S9790[1][3]);
v97[5]=v90[4]*S9790[2][1] + v90[5]*S9790[2][2] + v90[3]*(-(eff[4].x[2]*S9790[2][1]) + eff[4].x[1]*S9790[2][2]) + v90[6]*S9790[2][3] + v90[2]*(eff[4].x[3]*S9790[2][1] - eff[4].x[1]*S9790[2][3]) + v90[1]*(-(eff[4].x[3]*S9790[2][2]) + eff[4].x[2]*S9790[2][3]);
v97[6]=v90[4]*S9790[3][1] + v90[5]*S9790[3][2] + v90[3]*(-(eff[4].x[2]*S9790[3][1]) + eff[4].x[1]*S9790[3][2]) + v90[6]*S9790[3][3] + v90[2]*(eff[4].x[3]*S9790[3][1] - eff[4].x[1]*S9790[3][3]) + v90[1]*(-(eff[4].x[3]*S9790[3][2]) + eff[4].x[2]*S9790[3][3]);



}


void
hermes_InvDynNEfunc5(void)
     {
/* acceleration vectors */
a0[1]=baseo[0].add[1]*S00[1][1] + baseo[0].add[2]*S00[1][2] + baseo[0].add[3]*S00[1][3];
a0[2]=baseo[0].add[1]*S00[2][1] + baseo[0].add[2]*S00[2][2] + baseo[0].add[3]*S00[2][3];
a0[3]=baseo[0].add[1]*S00[3][1] + baseo[0].add[2]*S00[3][2] + baseo[0].add[3]*S00[3][3];
a0[4]=basec[0].xdd[1]*S00[1][1] + basec[0].xdd[2]*S00[1][2] + (gravity + basec[0].xdd[3])*S00[1][3];
a0[5]=basec[0].xdd[1]*S00[2][1] + basec[0].xdd[2]*S00[2][2] + (gravity + basec[0].xdd[3])*S00[2][3];
a0[6]=basec[0].xdd[1]*S00[3][1] + basec[0].xdd[2]*S00[3][2] + (gravity + basec[0].xdd[3])*S00[3][3];

a1[1]=state[29].thd*v1[2] + a0[1]*S10[1][1] + a0[2]*S10[1][2];
a1[2]=-(state[29].thd*v1[1]) + a0[1]*S10[2][1] + a0[2]*S10[2][2];
a1[3]=state[29].thdd - a0[3];
a1[4]=state[29].thd*v1[5] + PELVIS2THORAX*a0[2]*S10[1][1] + PELVISOFFSET*a0[3]*S10[1][1] + a0[4]*S10[1][1] - PELVIS2THORAX*a0[1]*S10[1][2] + a0[5]*S10[1][2];
a1[5]=-(state[29].thd*v1[4]) + PELVIS2THORAX*a0[2]*S10[2][1] + PELVISOFFSET*a0[3]*S10[2][1] + a0[4]*S10[2][1] - PELVIS2THORAX*a0[1]*S10[2][2] + a0[5]*S10[2][2];
a1[6]=PELVISOFFSET*a0[1] - a0[6];

a2[1]=state[30].thd*v2[2] + a1[1]*S21[1][1] + a1[3]*S21[1][3];
a2[2]=-(state[30].thd*v2[1]) + a1[1]*S21[2][1] + a1[3]*S21[2][3];
a2[3]=state[30].thdd - a1[2];
a2[4]=state[30].thd*v2[5] + a1[4]*S21[1][1] + a1[6]*S21[1][3];
a2[5]=-(state[30].thd*v2[4]) + a1[4]*S21[2][1] + a1[6]*S21[2][3];
a2[6]=-a1[5];

a3[1]=state[31].thd*v3[2] + a2[1]*S32[1][1] + a2[3]*S32[1][3];
a3[2]=-(state[31].thd*v3[1]) + a2[1]*S32[2][1] + a2[3]*S32[2][3];
a3[3]=state[31].thdd + a2[2];
a3[4]=state[31].thd*v3[5] + a2[4]*S32[1][1] + a2[6]*S32[1][3];
a3[5]=-(state[31].thd*v3[4]) + a2[4]*S32[2][1] + a2[6]*S32[2][3];
a3[6]=a2[5];

a4[1]=state[1].thd*v4[2] + a3[1]*S43[1][1] + a3[2]*S43[1][2] + a3[3]*S43[1][3];
a4[2]=-(state[1].thd*v4[1]) + a3[1]*S43[2][1] + a3[2]*S43[2][2] + a3[3]*S43[2][3];
a4[3]=state[1].thdd + 0.7071067811865475*a3[1] - 0.7071067811865475*a3[3];
a4[4]=state[1].thd*v4[5] + a3[4]*S43[1][1] - THORAX2SHOULDER*a3[3]*S43[1][2] + a3[5]*S43[1][2] + THORAX2SHOULDER*a3[2]*S43[1][3] + a3[6]*S43[1][3];
a4[5]=-(state[1].thd*v4[4]) + a3[4]*S43[2][1] - THORAX2SHOULDER*a3[3]*S43[2][2] + a3[5]*S43[2][2] + THORAX2SHOULDER*a3[2]*S43[2][3] + a3[6]*S43[2][3];
a4[6]=-0.7071067811865475*THORAX2SHOULDER*a3[2] + 0.7071067811865475*a3[4] - 0.7071067811865475*a3[6];

a5[1]=state[2].thd*v5[2] + a4[1]*S54[1][1] + a4[3]*S54[1][3];
a5[2]=-(state[2].thd*v5[1]) + a4[1]*S54[2][1] + a4[3]*S54[2][3];
a5[3]=state[2].thdd - a4[2];
a5[4]=state[2].thd*v5[5] - SHOULDERX*a4[2]*S54[1][1] + a4[4]*S54[1][1] + a4[6]*S54[1][3];
a5[5]=-(state[2].thd*v5[4]) - SHOULDERX*a4[2]*S54[2][1] + a4[4]*S54[2][1] + a4[6]*S54[2][3];
a5[6]=-(SHOULDERX*a4[1]) - a4[5];

a6[1]=state[3].thd*v6[2] + a5[1]*S65[1][1] + a5[3]*S65[1][3];
a6[2]=-(state[3].thd*v6[1]) + a5[1]*S65[2][1] + a5[3]*S65[2][3];
a6[3]=state[3].thdd + a5[2];
a6[4]=state[3].thd*v6[5] + a5[4]*S65[1][1] + SHOULDERY*a5[2]*S65[1][3] + a5[6]*S65[1][3];
a6[5]=-(state[3].thd*v6[4]) + a5[4]*S65[2][1] + SHOULDERY*a5[2]*S65[2][3] + a5[6]*S65[2][3];
a6[6]=-(SHOULDERY*a5[3]) + a5[5];

a7[1]=state[4].thd*v7[2] + a6[2]*S76[1][2] + a6[3]*S76[1][3];
a7[2]=-(state[4].thd*v7[1]) + a6[2]*S76[2][2] + a6[3]*S76[2][3];
a7[3]=state[4].thdd + a6[1];
a7[4]=state[4].thd*v7[5] + UPPERARM*a6[1]*S76[1][2] + a6[5]*S76[1][2] + a6[6]*S76[1][3];
a7[5]=-(state[4].thd*v7[4]) + UPPERARM*a6[1]*S76[2][2] + a6[5]*S76[2][2] + a6[6]*S76[2][3];
a7[6]=-(UPPERARM*a6[2]) + a6[4];

a8[1]=state[5].thd*v8[2] + a7[1]*S87[1][1] + a7[3]*S87[1][3];
a8[2]=-(state[5].thd*v8[1]) + a7[1]*S87[2][1] + a7[3]*S87[2][3];
a8[3]=state[5].thdd + a7[2];
a8[4]=state[5].thd*v8[5] + a7[4]*S87[1][1] + a7[6]*S87[1][3];
a8[5]=-(state[5].thd*v8[4]) + a7[4]*S87[2][1] + a7[6]*S87[2][3];
a8[6]=a7[5];

a9[1]=state[6].thd*v9[2] + a8[2]*S98[1][2] + a8[3]*S98[1][3];
a9[2]=-(state[6].thd*v9[1]) + a8[2]*S98[2][2] + a8[3]*S98[2][3];
a9[3]=state[6].thdd + a8[1];
a9[4]=state[6].thd*v9[5] + a8[5]*S98[1][2] + a8[6]*S98[1][3] + a8[1]*(LOWERARM*S98[1][2] + WRISTY*S98[1][3]);
a9[5]=-(state[6].thd*v9[4]) + a8[5]*S98[2][2] + a8[6]*S98[2][3] + a8[1]*(LOWERARM*S98[2][2] + WRISTY*S98[2][3]);
a9[6]=-(LOWERARM*a8[2]) - WRISTY*a8[3] + a8[4];

a10[1]=state[7].thd*v10[2] + a9[1]*S109[1][1] + a9[3]*S109[1][3];
a10[2]=-(state[7].thd*v10[1]) + a9[1]*S109[2][1] + a9[3]*S109[2][3];
a10[3]=state[7].thdd - a9[2];
a10[4]=state[7].thd*v10[5] + a9[4]*S109[1][1] + a9[6]*S109[1][3];
a10[5]=-(state[7].thd*v10[4]) + a9[4]*S109[2][1] + a9[6]*S109[2][3];
a10[6]=-a9[5];

a11[1]=a10[1]*S1110[1][1] + a10[2]*S1110[1][2] + a10[3]*S1110[1][3];
a11[2]=a10[1]*S1110[2][1] + a10[2]*S1110[2][2] + a10[3]*S1110[2][3];
a11[3]=a10[1]*S1110[3][1] + a10[2]*S1110[3][2] + a10[3]*S1110[3][3];
a11[4]=a10[4]*S1110[1][1] + a10[5]*S1110[1][2] + a10[3]*(-(eff[2].x[2]*S1110[1][1]) + eff[2].x[1]*S1110[1][2]) + a10[6]*S1110[1][3] + a10[2]*(eff[2].x[3]*S1110[1][1] - eff[2].x[1]*S1110[1][3]) + a10[1]*(-(eff[2].x[3]*S1110[1][2]) + eff[2].x[2]*S1110[1][3]);
a11[5]=a10[4]*S1110[2][1] + a10[5]*S1110[2][2] + a10[3]*(-(eff[2].x[2]*S1110[2][1]) + eff[2].x[1]*S1110[2][2]) + a10[6]*S1110[2][3] + a10[2]*(eff[2].x[3]*S1110[2][1] - eff[2].x[1]*S1110[2][3]) + a10[1]*(-(eff[2].x[3]*S1110[2][2]) + eff[2].x[2]*S1110[2][3]);
a11[6]=a10[4]*S1110[3][1] + a10[5]*S1110[3][2] + a10[3]*(-(eff[2].x[2]*S1110[3][1]) + eff[2].x[1]*S1110[3][2]) + a10[6]*S1110[3][3] + a10[2]*(eff[2].x[3]*S1110[3][1] - eff[2].x[1]*S1110[3][3]) + a10[1]*(-(eff[2].x[3]*S1110[3][2]) + eff[2].x[2]*S1110[3][3]);

a12[1]=state[39].thdd + a10[1]*S1210[1][1] + a10[2]*S1210[1][2] + a10[3]*S1210[1][3];
a12[2]=state[39].thd*v12[3] + a10[1]*S1210[2][1] + a10[2]*S1210[2][2] + a10[3]*S1210[2][3];
a12[3]=-(state[39].thd*v12[2]) + a10[1]*S1210[3][1] + a10[2]*S1210[3][2] + a10[3]*S1210[3][3];
a12[4]=a10[4]*S1210[1][1] + a10[5]*S1210[1][2] + a10[3]*(-(YTHUMB*S1210[1][1]) + XTHUMB*S1210[1][2]) + a10[6]*S1210[1][3] + a10[2]*(-(ZTHUMB*S1210[1][1]) - XTHUMB*S1210[1][3]) + a10[1]*(ZTHUMB*S1210[1][2] + YTHUMB*S1210[1][3]);
a12[5]=state[39].thd*v12[6] + a10[4]*S1210[2][1] + a10[5]*S1210[2][2] + a10[3]*(-(YTHUMB*S1210[2][1]) + XTHUMB*S1210[2][2]) + a10[6]*S1210[2][3] + a10[2]*(-(ZTHUMB*S1210[2][1]) - XTHUMB*S1210[2][3]) + a10[1]*(ZTHUMB*S1210[2][2] + YTHUMB*S1210[2][3]);
a12[6]=-(state[39].thd*v12[5]) + a10[4]*S1210[3][1] + a10[5]*S1210[3][2] + a10[3]*(-(YTHUMB*S1210[3][1]) + XTHUMB*S1210[3][2]) + a10[6]*S1210[3][3] + a10[2]*(-(ZTHUMB*S1210[3][1]) - XTHUMB*S1210[3][3]) + a10[1]*(ZTHUMB*S1210[3][2] + YTHUMB*S1210[3][3]);

a13[1]=-(state[40].thd*v13[2]) + a12[1]*S1312[1][1] + a12[2]*S1312[1][2];
a13[2]=state[40].thd*v13[1] + a12[1]*S1312[2][1] + a12[2]*S1312[2][2];
a13[3]=-state[40].thdd + a12[3];
a13[4]=-(state[40].thd*v13[5]) + a12[4]*S1312[1][1] + a12[5]*S1312[1][2] + a12[3]*(-(YTHUMBFLEX*S1312[1][1]) + XTHUMBFLEX*S1312[1][2]);
a13[5]=state[40].thd*v13[4] + a12[4]*S1312[2][1] + a12[5]*S1312[2][2] + a12[3]*(-(YTHUMBFLEX*S1312[2][1]) + XTHUMBFLEX*S1312[2][2]);
a13[6]=YTHUMBFLEX*a12[1] - XTHUMBFLEX*a12[2] + a12[6];

a14[1]=a13[1]*S1413[1][1] + a13[2]*S1413[1][2];
a14[2]=a13[1]*S1413[2][1] + a13[2]*S1413[2][2];
a14[3]=a13[3];
a14[4]=a13[4]*S1413[1][1] + TH1SEG*a13[3]*S1413[1][2] + a13[5]*S1413[1][2];
a14[5]=a13[4]*S1413[2][1] + TH1SEG*a13[3]*S1413[2][2] + a13[5]*S1413[2][2];
a14[6]=-(TH1SEG*a13[2]) + a13[6];

a15[1]=a14[1];
a15[2]=a14[2];
a15[3]=a14[3];
a15[4]=a14[4];
a15[5]=TH2SEG*a14[3] + a14[5];
a15[6]=-(TH2SEG*a14[2]) + a14[6];

a16[1]=state[41].thd*v16[2] + a10[1]*S1610[1][1] + a10[2]*S1610[1][2] + a10[3]*S1610[1][3];
a16[2]=-(state[41].thd*v16[1]) + a10[1]*S1610[2][1] + a10[2]*S1610[2][2] + a10[3]*S1610[2][3];
a16[3]=state[41].thdd + a10[2]*S1610[3][2] + a10[3]*S1610[3][3];
a16[4]=state[41].thd*v16[5] + a10[4]*S1610[1][1] + a10[5]*S1610[1][2] + a10[3]*(-(YIF*S1610[1][1]) + XIF*S1610[1][2]) + a10[6]*S1610[1][3] + a10[2]*(-(ZIF*S1610[1][1]) - XIF*S1610[1][3]) + a10[1]*(ZIF*S1610[1][2] + YIF*S1610[1][3]);
a16[5]=-(state[41].thd*v16[4]) + a10[4]*S1610[2][1] + a10[5]*S1610[2][2] + a10[3]*(-(YIF*S1610[2][1]) + XIF*S1610[2][2]) + a10[6]*S1610[2][3] + a10[2]*(-(ZIF*S1610[2][1]) - XIF*S1610[2][3]) + a10[1]*(ZIF*S1610[2][2] + YIF*S1610[2][3]);
a16[6]=XIF*a10[3]*S1610[3][2] + a10[5]*S1610[3][2] - XIF*a10[2]*S1610[3][3] + a10[6]*S1610[3][3] + a10[1]*(ZIF*S1610[3][2] + YIF*S1610[3][3]);

a17[1]=a16[1]*S1716[1][1] + a16[2]*S1716[1][2];
a17[2]=a16[1]*S1716[2][1] + a16[2]*S1716[2][2];
a17[3]=a16[3];
a17[4]=a16[4]*S1716[1][1] + F1SEG*a16[3]*S1716[1][2] + a16[5]*S1716[1][2];
a17[5]=a16[4]*S1716[2][1] + F1SEG*a16[3]*S1716[2][2] + a16[5]*S1716[2][2];
a17[6]=-(F1SEG*a16[2]) + a16[6];

a18[1]=a17[1]*S1817[1][1] + a17[2]*S1817[1][2];
a18[2]=a17[1]*S1817[2][1] + a17[2]*S1817[2][2];
a18[3]=a17[3];
a18[4]=a17[4]*S1817[1][1] + F2SEG*a17[3]*S1817[1][2] + a17[5]*S1817[1][2];
a18[5]=a17[4]*S1817[2][1] + F2SEG*a17[3]*S1817[2][2] + a17[5]*S1817[2][2];
a18[6]=-(F2SEG*a17[2]) + a17[6];

a19[1]=a18[1];
a19[2]=a18[2];
a19[3]=a18[3];
a19[4]=a18[4];
a19[5]=F3SEG*a18[3] + a18[5];
a19[6]=-(F3SEG*a18[2]) + a18[6];

a20[1]=state[42].thd*v20[2] + a10[1]*S2010[1][1] + a10[2]*S2010[1][2] + a10[3]*S2010[1][3];
a20[2]=-(state[42].thd*v20[1]) + a10[1]*S2010[2][1] + a10[2]*S2010[2][2] + a10[3]*S2010[2][3];
a20[3]=state[42].thdd + a10[2]*S2010[3][2] + a10[3]*S2010[3][3];
a20[4]=state[42].thd*v20[5] + a10[4]*S2010[1][1] + a10[5]*S2010[1][2] + a10[3]*(-(YMF*S2010[1][1]) + XMF*S2010[1][2]) + a10[6]*S2010[1][3] + a10[2]*(-(ZMF*S2010[1][1]) - XMF*S2010[1][3]) + a10[1]*(ZMF*S2010[1][2] + YMF*S2010[1][3]);
a20[5]=-(state[42].thd*v20[4]) + a10[4]*S2010[2][1] + a10[5]*S2010[2][2] + a10[3]*(-(YMF*S2010[2][1]) + XMF*S2010[2][2]) + a10[6]*S2010[2][3] + a10[2]*(-(ZMF*S2010[2][1]) - XMF*S2010[2][3]) + a10[1]*(ZMF*S2010[2][2] + YMF*S2010[2][3]);
a20[6]=XMF*a10[3]*S2010[3][2] + a10[5]*S2010[3][2] - XMF*a10[2]*S2010[3][3] + a10[6]*S2010[3][3] + a10[1]*(ZMF*S2010[3][2] + YMF*S2010[3][3]);

a21[1]=a20[1]*S2120[1][1] + a20[2]*S2120[1][2];
a21[2]=a20[1]*S2120[2][1] + a20[2]*S2120[2][2];
a21[3]=a20[3];
a21[4]=a20[4]*S2120[1][1] + F1SEG*a20[3]*S2120[1][2] + a20[5]*S2120[1][2];
a21[5]=a20[4]*S2120[2][1] + F1SEG*a20[3]*S2120[2][2] + a20[5]*S2120[2][2];
a21[6]=-(F1SEG*a20[2]) + a20[6];

a22[1]=a21[1]*S2221[1][1] + a21[2]*S2221[1][2];
a22[2]=a21[1]*S2221[2][1] + a21[2]*S2221[2][2];
a22[3]=a21[3];
a22[4]=a21[4]*S2221[1][1] + F2SEG*a21[3]*S2221[1][2] + a21[5]*S2221[1][2];
a22[5]=a21[4]*S2221[2][1] + F2SEG*a21[3]*S2221[2][2] + a21[5]*S2221[2][2];
a22[6]=-(F2SEG*a21[2]) + a21[6];

a23[1]=a22[1];
a23[2]=a22[2];
a23[3]=a22[3];
a23[4]=a22[4];
a23[5]=F3SEG*a22[3] + a22[5];
a23[6]=-(F3SEG*a22[2]) + a22[6];

a24[1]=state[43].thd*v24[2] + a10[1]*S2410[1][1] + a10[2]*S2410[1][2] + a10[3]*S2410[1][3];
a24[2]=-(state[43].thd*v24[1]) + a10[1]*S2410[2][1] + a10[2]*S2410[2][2] + a10[3]*S2410[2][3];
a24[3]=state[43].thdd + a10[2]*S2410[3][2] + a10[3]*S2410[3][3];
a24[4]=state[43].thd*v24[5] + a10[4]*S2410[1][1] + a10[5]*S2410[1][2] + a10[3]*(-(YRF*S2410[1][1]) + XRF*S2410[1][2]) + a10[6]*S2410[1][3] + a10[2]*(-(ZRF*S2410[1][1]) - XRF*S2410[1][3]) + a10[1]*(ZRF*S2410[1][2] + YRF*S2410[1][3]);
a24[5]=-(state[43].thd*v24[4]) + a10[4]*S2410[2][1] + a10[5]*S2410[2][2] + a10[3]*(-(YRF*S2410[2][1]) + XRF*S2410[2][2]) + a10[6]*S2410[2][3] + a10[2]*(-(ZRF*S2410[2][1]) - XRF*S2410[2][3]) + a10[1]*(ZRF*S2410[2][2] + YRF*S2410[2][3]);
a24[6]=XRF*a10[3]*S2410[3][2] + a10[5]*S2410[3][2] - XRF*a10[2]*S2410[3][3] + a10[6]*S2410[3][3] + a10[1]*(ZRF*S2410[3][2] + YRF*S2410[3][3]);

a25[1]=a24[1]*S2524[1][1] + a24[2]*S2524[1][2];
a25[2]=a24[1]*S2524[2][1] + a24[2]*S2524[2][2];
a25[3]=a24[3];
a25[4]=a24[4]*S2524[1][1] + F1SEG*a24[3]*S2524[1][2] + a24[5]*S2524[1][2];
a25[5]=a24[4]*S2524[2][1] + F1SEG*a24[3]*S2524[2][2] + a24[5]*S2524[2][2];
a25[6]=-(F1SEG*a24[2]) + a24[6];

a26[1]=a25[1]*S2625[1][1] + a25[2]*S2625[1][2];
a26[2]=a25[1]*S2625[2][1] + a25[2]*S2625[2][2];
a26[3]=a25[3];
a26[4]=a25[4]*S2625[1][1] + F2SEG*a25[3]*S2625[1][2] + a25[5]*S2625[1][2];
a26[5]=a25[4]*S2625[2][1] + F2SEG*a25[3]*S2625[2][2] + a25[5]*S2625[2][2];
a26[6]=-(F2SEG*a25[2]) + a25[6];

a27[1]=a26[1];
a27[2]=a26[2];
a27[3]=a26[3];
a27[4]=a26[4];
a27[5]=F3SEG*a26[3] + a26[5];
a27[6]=-(F3SEG*a26[2]) + a26[6];

a28[1]=state[44].thd*v28[2] + a10[1]*S2810[1][1] + a10[2]*S2810[1][2] + a10[3]*S2810[1][3];
a28[2]=-(state[44].thd*v28[1]) + a10[1]*S2810[2][1] + a10[2]*S2810[2][2] + a10[3]*S2810[2][3];
a28[3]=state[44].thdd + a10[2]*S2810[3][2] + a10[3]*S2810[3][3];
a28[4]=state[44].thd*v28[5] + a10[4]*S2810[1][1] + a10[5]*S2810[1][2] + a10[3]*(-(YLF*S2810[1][1]) + XLF*S2810[1][2]) + a10[6]*S2810[1][3] + a10[2]*(-(ZLF*S2810[1][1]) - XLF*S2810[1][3]) + a10[1]*(ZLF*S2810[1][2] + YLF*S2810[1][3]);
a28[5]=-(state[44].thd*v28[4]) + a10[4]*S2810[2][1] + a10[5]*S2810[2][2] + a10[3]*(-(YLF*S2810[2][1]) + XLF*S2810[2][2]) + a10[6]*S2810[2][3] + a10[2]*(-(ZLF*S2810[2][1]) - XLF*S2810[2][3]) + a10[1]*(ZLF*S2810[2][2] + YLF*S2810[2][3]);
a28[6]=XLF*a10[3]*S2810[3][2] + a10[5]*S2810[3][2] - XLF*a10[2]*S2810[3][3] + a10[6]*S2810[3][3] + a10[1]*(ZLF*S2810[3][2] + YLF*S2810[3][3]);

a29[1]=a28[1]*S2928[1][1] + a28[2]*S2928[1][2];
a29[2]=a28[1]*S2928[2][1] + a28[2]*S2928[2][2];
a29[3]=a28[3];
a29[4]=a28[4]*S2928[1][1] + F1SEG*a28[3]*S2928[1][2] + a28[5]*S2928[1][2];
a29[5]=a28[4]*S2928[2][1] + F1SEG*a28[3]*S2928[2][2] + a28[5]*S2928[2][2];
a29[6]=-(F1SEG*a28[2]) + a28[6];

a30[1]=a29[1]*S3029[1][1] + a29[2]*S3029[1][2];
a30[2]=a29[1]*S3029[2][1] + a29[2]*S3029[2][2];
a30[3]=a29[3];
a30[4]=a29[4]*S3029[1][1] + F2SEG*a29[3]*S3029[1][2] + a29[5]*S3029[1][2];
a30[5]=a29[4]*S3029[2][1] + F2SEG*a29[3]*S3029[2][2] + a29[5]*S3029[2][2];
a30[6]=-(F2SEG*a29[2]) + a29[6];

a31[1]=a30[1];
a31[2]=a30[2];
a31[3]=a30[3];
a31[4]=a30[4];
a31[5]=F3SEG*a30[3] + a30[5];
a31[6]=-(F3SEG*a30[2]) + a30[6];

a32[1]=state[8].thd*v32[2] + a3[1]*S323[1][1] + a3[2]*S323[1][2] + a3[3]*S323[1][3];
a32[2]=-(state[8].thd*v32[1]) + a3[1]*S323[2][1] + a3[2]*S323[2][2] + a3[3]*S323[2][3];
a32[3]=state[8].thdd - 0.7071067811865475*a3[1] - 0.7071067811865475*a3[3];
a32[4]=state[8].thd*v32[5] + a3[4]*S323[1][1] - THORAX2SHOULDER*a3[3]*S323[1][2] + a3[5]*S323[1][2] + THORAX2SHOULDER*a3[2]*S323[1][3] + a3[6]*S323[1][3];
a32[5]=-(state[8].thd*v32[4]) + a3[4]*S323[2][1] - THORAX2SHOULDER*a3[3]*S323[2][2] + a3[5]*S323[2][2] + THORAX2SHOULDER*a3[2]*S323[2][3] + a3[6]*S323[2][3];
a32[6]=-0.7071067811865475*THORAX2SHOULDER*a3[2] - 0.7071067811865475*a3[4] - 0.7071067811865475*a3[6];

a33[1]=state[9].thd*v33[2] + a32[1]*S3332[1][1] + a32[3]*S3332[1][3];
a33[2]=-(state[9].thd*v33[1]) + a32[1]*S3332[2][1] + a32[3]*S3332[2][3];
a33[3]=state[9].thdd + a32[2];
a33[4]=state[9].thd*v33[5] + SHOULDERX*a32[2]*S3332[1][1] + a32[4]*S3332[1][1] + a32[6]*S3332[1][3];
a33[5]=-(state[9].thd*v33[4]) + SHOULDERX*a32[2]*S3332[2][1] + a32[4]*S3332[2][1] + a32[6]*S3332[2][3];
a33[6]=-(SHOULDERX*a32[1]) + a32[5];

a34[1]=state[10].thd*v34[2] + a33[1]*S3433[1][1] + a33[3]*S3433[1][3];
a34[2]=-(state[10].thd*v34[1]) + a33[1]*S3433[2][1] + a33[3]*S3433[2][3];
a34[3]=state[10].thdd - a33[2];
a34[4]=state[10].thd*v34[5] + a33[4]*S3433[1][1] + SHOULDERY*a33[2]*S3433[1][3] + a33[6]*S3433[1][3];
a34[5]=-(state[10].thd*v34[4]) + a33[4]*S3433[2][1] + SHOULDERY*a33[2]*S3433[2][3] + a33[6]*S3433[2][3];
a34[6]=SHOULDERY*a33[3] - a33[5];

a35[1]=state[11].thd*v35[2] + a34[2]*S3534[1][2] + a34[3]*S3534[1][3];
a35[2]=-(state[11].thd*v35[1]) + a34[2]*S3534[2][2] + a34[3]*S3534[2][3];
a35[3]=state[11].thdd - a34[1];
a35[4]=state[11].thd*v35[5] - UPPERARM*a34[1]*S3534[1][2] + a34[5]*S3534[1][2] + a34[6]*S3534[1][3];
a35[5]=-(state[11].thd*v35[4]) - UPPERARM*a34[1]*S3534[2][2] + a34[5]*S3534[2][2] + a34[6]*S3534[2][3];
a35[6]=-(UPPERARM*a34[2]) - a34[4];

a36[1]=state[12].thd*v36[2] + a35[1]*S3635[1][1] + a35[3]*S3635[1][3];
a36[2]=-(state[12].thd*v36[1]) + a35[1]*S3635[2][1] + a35[3]*S3635[2][3];
a36[3]=state[12].thdd - a35[2];
a36[4]=state[12].thd*v36[5] + a35[4]*S3635[1][1] + a35[6]*S3635[1][3];
a36[5]=-(state[12].thd*v36[4]) + a35[4]*S3635[2][1] + a35[6]*S3635[2][3];
a36[6]=-a35[5];

a37[1]=state[13].thd*v37[2] + a36[2]*S3736[1][2] + a36[3]*S3736[1][3];
a37[2]=-(state[13].thd*v37[1]) + a36[2]*S3736[2][2] + a36[3]*S3736[2][3];
a37[3]=state[13].thdd - a36[1];
a37[4]=state[13].thd*v37[5] + a36[5]*S3736[1][2] + a36[6]*S3736[1][3] + a36[1]*(-(LOWERARM*S3736[1][2]) + WRISTY*S3736[1][3]);
a37[5]=-(state[13].thd*v37[4]) + a36[5]*S3736[2][2] + a36[6]*S3736[2][3] + a36[1]*(-(LOWERARM*S3736[2][2]) + WRISTY*S3736[2][3]);
a37[6]=-(LOWERARM*a36[2]) + WRISTY*a36[3] - a36[4];

a38[1]=state[14].thd*v38[2] + a37[1]*S3837[1][1] + a37[3]*S3837[1][3];
a38[2]=-(state[14].thd*v38[1]) + a37[1]*S3837[2][1] + a37[3]*S3837[2][3];
a38[3]=state[14].thdd + a37[2];
a38[4]=state[14].thd*v38[5] + a37[4]*S3837[1][1] + a37[6]*S3837[1][3];
a38[5]=-(state[14].thd*v38[4]) + a37[4]*S3837[2][1] + a37[6]*S3837[2][3];
a38[6]=a37[5];

a39[1]=a38[1]*S3938[1][1] + a38[2]*S3938[1][2] + a38[3]*S3938[1][3];
a39[2]=a38[1]*S3938[2][1] + a38[2]*S3938[2][2] + a38[3]*S3938[2][3];
a39[3]=a38[1]*S3938[3][1] + a38[2]*S3938[3][2] + a38[3]*S3938[3][3];
a39[4]=a38[4]*S3938[1][1] + a38[5]*S3938[1][2] + a38[3]*(-(eff[1].x[2]*S3938[1][1]) + eff[1].x[1]*S3938[1][2]) + a38[6]*S3938[1][3] + a38[2]*(eff[1].x[3]*S3938[1][1] - eff[1].x[1]*S3938[1][3]) + a38[1]*(-(eff[1].x[3]*S3938[1][2]) + eff[1].x[2]*S3938[1][3]);
a39[5]=a38[4]*S3938[2][1] + a38[5]*S3938[2][2] + a38[3]*(-(eff[1].x[2]*S3938[2][1]) + eff[1].x[1]*S3938[2][2]) + a38[6]*S3938[2][3] + a38[2]*(eff[1].x[3]*S3938[2][1] - eff[1].x[1]*S3938[2][3]) + a38[1]*(-(eff[1].x[3]*S3938[2][2]) + eff[1].x[2]*S3938[2][3]);
a39[6]=a38[4]*S3938[3][1] + a38[5]*S3938[3][2] + a38[3]*(-(eff[1].x[2]*S3938[3][1]) + eff[1].x[1]*S3938[3][2]) + a38[6]*S3938[3][3] + a38[2]*(eff[1].x[3]*S3938[3][1] - eff[1].x[1]*S3938[3][3]) + a38[1]*(-(eff[1].x[3]*S3938[3][2]) + eff[1].x[2]*S3938[3][3]);

a40[1]=state[45].thdd + a38[1]*S4038[1][1] + a38[2]*S4038[1][2] + a38[3]*S4038[1][3];
a40[2]=state[45].thd*v40[3] + a38[1]*S4038[2][1] + a38[2]*S4038[2][2] + a38[3]*S4038[2][3];
a40[3]=-(state[45].thd*v40[2]) + a38[1]*S4038[3][1] + a38[2]*S4038[3][2] + a38[3]*S4038[3][3];
a40[4]=a38[4]*S4038[1][1] + a38[5]*S4038[1][2] + a38[3]*(-(YTHUMB*S4038[1][1]) + XTHUMB*S4038[1][2]) + a38[6]*S4038[1][3] + a38[2]*(ZTHUMB*S4038[1][1] - XTHUMB*S4038[1][3]) + a38[1]*(-(ZTHUMB*S4038[1][2]) + YTHUMB*S4038[1][3]);
a40[5]=state[45].thd*v40[6] + a38[4]*S4038[2][1] + a38[5]*S4038[2][2] + a38[3]*(-(YTHUMB*S4038[2][1]) + XTHUMB*S4038[2][2]) + a38[6]*S4038[2][3] + a38[2]*(ZTHUMB*S4038[2][1] - XTHUMB*S4038[2][3]) + a38[1]*(-(ZTHUMB*S4038[2][2]) + YTHUMB*S4038[2][3]);
a40[6]=-(state[45].thd*v40[5]) + a38[4]*S4038[3][1] + a38[5]*S4038[3][2] + a38[3]*(-(YTHUMB*S4038[3][1]) + XTHUMB*S4038[3][2]) + a38[6]*S4038[3][3] + a38[2]*(ZTHUMB*S4038[3][1] - XTHUMB*S4038[3][3]) + a38[1]*(-(ZTHUMB*S4038[3][2]) + YTHUMB*S4038[3][3]);

a41[1]=-(state[46].thd*v41[2]) + a40[1]*S4140[1][1] + a40[2]*S4140[1][2];
a41[2]=state[46].thd*v41[1] + a40[1]*S4140[2][1] + a40[2]*S4140[2][2];
a41[3]=-state[46].thdd + a40[3];
a41[4]=-(state[46].thd*v41[5]) + a40[4]*S4140[1][1] + a40[5]*S4140[1][2] + a40[3]*(-(YTHUMBFLEX*S4140[1][1]) + XTHUMBFLEX*S4140[1][2]);
a41[5]=state[46].thd*v41[4] + a40[4]*S4140[2][1] + a40[5]*S4140[2][2] + a40[3]*(-(YTHUMBFLEX*S4140[2][1]) + XTHUMBFLEX*S4140[2][2]);
a41[6]=YTHUMBFLEX*a40[1] - XTHUMBFLEX*a40[2] + a40[6];

a42[1]=a41[1]*S4241[1][1] + a41[2]*S4241[1][2];
a42[2]=a41[1]*S4241[2][1] + a41[2]*S4241[2][2];
a42[3]=a41[3];
a42[4]=a41[4]*S4241[1][1] + TH1SEG*a41[3]*S4241[1][2] + a41[5]*S4241[1][2];
a42[5]=a41[4]*S4241[2][1] + TH1SEG*a41[3]*S4241[2][2] + a41[5]*S4241[2][2];
a42[6]=-(TH1SEG*a41[2]) + a41[6];

a43[1]=a42[1];
a43[2]=a42[2];
a43[3]=a42[3];
a43[4]=a42[4];
a43[5]=TH2SEG*a42[3] + a42[5];
a43[6]=-(TH2SEG*a42[2]) + a42[6];

a44[1]=state[47].thd*v44[2] + a38[1]*S4438[1][1] + a38[2]*S4438[1][2] + a38[3]*S4438[1][3];
a44[2]=-(state[47].thd*v44[1]) + a38[1]*S4438[2][1] + a38[2]*S4438[2][2] + a38[3]*S4438[2][3];
a44[3]=state[47].thdd + a38[2]*S4438[3][2] + a38[3]*S4438[3][3];
a44[4]=state[47].thd*v44[5] + a38[4]*S4438[1][1] + a38[5]*S4438[1][2] + a38[3]*(-(YIF*S4438[1][1]) + XIF*S4438[1][2]) + a38[6]*S4438[1][3] + a38[2]*(ZIF*S4438[1][1] - XIF*S4438[1][3]) + a38[1]*(-(ZIF*S4438[1][2]) + YIF*S4438[1][3]);
a44[5]=-(state[47].thd*v44[4]) + a38[4]*S4438[2][1] + a38[5]*S4438[2][2] + a38[3]*(-(YIF*S4438[2][1]) + XIF*S4438[2][2]) + a38[6]*S4438[2][3] + a38[2]*(ZIF*S4438[2][1] - XIF*S4438[2][3]) + a38[1]*(-(ZIF*S4438[2][2]) + YIF*S4438[2][3]);
a44[6]=XIF*a38[3]*S4438[3][2] + a38[5]*S4438[3][2] - XIF*a38[2]*S4438[3][3] + a38[6]*S4438[3][3] + a38[1]*(-(ZIF*S4438[3][2]) + YIF*S4438[3][3]);

a45[1]=a44[1]*S4544[1][1] + a44[2]*S4544[1][2];
a45[2]=a44[1]*S4544[2][1] + a44[2]*S4544[2][2];
a45[3]=a44[3];
a45[4]=a44[4]*S4544[1][1] + F1SEG*a44[3]*S4544[1][2] + a44[5]*S4544[1][2];
a45[5]=a44[4]*S4544[2][1] + F1SEG*a44[3]*S4544[2][2] + a44[5]*S4544[2][2];
a45[6]=-(F1SEG*a44[2]) + a44[6];

a46[1]=a45[1]*S4645[1][1] + a45[2]*S4645[1][2];
a46[2]=a45[1]*S4645[2][1] + a45[2]*S4645[2][2];
a46[3]=a45[3];
a46[4]=a45[4]*S4645[1][1] + F2SEG*a45[3]*S4645[1][2] + a45[5]*S4645[1][2];
a46[5]=a45[4]*S4645[2][1] + F2SEG*a45[3]*S4645[2][2] + a45[5]*S4645[2][2];
a46[6]=-(F2SEG*a45[2]) + a45[6];

a47[1]=a46[1];
a47[2]=a46[2];
a47[3]=a46[3];
a47[4]=a46[4];
a47[5]=F3SEG*a46[3] + a46[5];
a47[6]=-(F3SEG*a46[2]) + a46[6];

a48[1]=state[48].thd*v48[2] + a38[1]*S4838[1][1] + a38[2]*S4838[1][2] + a38[3]*S4838[1][3];
a48[2]=-(state[48].thd*v48[1]) + a38[1]*S4838[2][1] + a38[2]*S4838[2][2] + a38[3]*S4838[2][3];
a48[3]=state[48].thdd + a38[2]*S4838[3][2] + a38[3]*S4838[3][3];
a48[4]=state[48].thd*v48[5] + a38[4]*S4838[1][1] + a38[5]*S4838[1][2] + a38[3]*(-(YMF*S4838[1][1]) + XMF*S4838[1][2]) + a38[6]*S4838[1][3] + a38[2]*(ZMF*S4838[1][1] - XMF*S4838[1][3]) + a38[1]*(-(ZMF*S4838[1][2]) + YMF*S4838[1][3]);
a48[5]=-(state[48].thd*v48[4]) + a38[4]*S4838[2][1] + a38[5]*S4838[2][2] + a38[3]*(-(YMF*S4838[2][1]) + XMF*S4838[2][2]) + a38[6]*S4838[2][3] + a38[2]*(ZMF*S4838[2][1] - XMF*S4838[2][3]) + a38[1]*(-(ZMF*S4838[2][2]) + YMF*S4838[2][3]);
a48[6]=XMF*a38[3]*S4838[3][2] + a38[5]*S4838[3][2] - XMF*a38[2]*S4838[3][3] + a38[6]*S4838[3][3] + a38[1]*(-(ZMF*S4838[3][2]) + YMF*S4838[3][3]);

a49[1]=a48[1]*S4948[1][1] + a48[2]*S4948[1][2];
a49[2]=a48[1]*S4948[2][1] + a48[2]*S4948[2][2];
a49[3]=a48[3];
a49[4]=a48[4]*S4948[1][1] + F1SEG*a48[3]*S4948[1][2] + a48[5]*S4948[1][2];
a49[5]=a48[4]*S4948[2][1] + F1SEG*a48[3]*S4948[2][2] + a48[5]*S4948[2][2];
a49[6]=-(F1SEG*a48[2]) + a48[6];

a50[1]=a49[1]*S5049[1][1] + a49[2]*S5049[1][2];
a50[2]=a49[1]*S5049[2][1] + a49[2]*S5049[2][2];
a50[3]=a49[3];
a50[4]=a49[4]*S5049[1][1] + F2SEG*a49[3]*S5049[1][2] + a49[5]*S5049[1][2];
a50[5]=a49[4]*S5049[2][1] + F2SEG*a49[3]*S5049[2][2] + a49[5]*S5049[2][2];
a50[6]=-(F2SEG*a49[2]) + a49[6];

a51[1]=a50[1];
a51[2]=a50[2];
a51[3]=a50[3];
a51[4]=a50[4];
a51[5]=F3SEG*a50[3] + a50[5];
a51[6]=-(F3SEG*a50[2]) + a50[6];

a52[1]=state[49].thd*v52[2] + a38[1]*S5238[1][1] + a38[2]*S5238[1][2] + a38[3]*S5238[1][3];
a52[2]=-(state[49].thd*v52[1]) + a38[1]*S5238[2][1] + a38[2]*S5238[2][2] + a38[3]*S5238[2][3];
a52[3]=state[49].thdd + a38[2]*S5238[3][2] + a38[3]*S5238[3][3];
a52[4]=state[49].thd*v52[5] + a38[4]*S5238[1][1] + a38[5]*S5238[1][2] + a38[3]*(-(YRF*S5238[1][1]) + XRF*S5238[1][2]) + a38[6]*S5238[1][3] + a38[2]*(ZRF*S5238[1][1] - XRF*S5238[1][3]) + a38[1]*(-(ZRF*S5238[1][2]) + YRF*S5238[1][3]);
a52[5]=-(state[49].thd*v52[4]) + a38[4]*S5238[2][1] + a38[5]*S5238[2][2] + a38[3]*(-(YRF*S5238[2][1]) + XRF*S5238[2][2]) + a38[6]*S5238[2][3] + a38[2]*(ZRF*S5238[2][1] - XRF*S5238[2][3]) + a38[1]*(-(ZRF*S5238[2][2]) + YRF*S5238[2][3]);
a52[6]=XRF*a38[3]*S5238[3][2] + a38[5]*S5238[3][2] - XRF*a38[2]*S5238[3][3] + a38[6]*S5238[3][3] + a38[1]*(-(ZRF*S5238[3][2]) + YRF*S5238[3][3]);

a53[1]=a52[1]*S5352[1][1] + a52[2]*S5352[1][2];
a53[2]=a52[1]*S5352[2][1] + a52[2]*S5352[2][2];
a53[3]=a52[3];
a53[4]=a52[4]*S5352[1][1] + F1SEG*a52[3]*S5352[1][2] + a52[5]*S5352[1][2];
a53[5]=a52[4]*S5352[2][1] + F1SEG*a52[3]*S5352[2][2] + a52[5]*S5352[2][2];
a53[6]=-(F1SEG*a52[2]) + a52[6];

a54[1]=a53[1]*S5453[1][1] + a53[2]*S5453[1][2];
a54[2]=a53[1]*S5453[2][1] + a53[2]*S5453[2][2];
a54[3]=a53[3];
a54[4]=a53[4]*S5453[1][1] + F2SEG*a53[3]*S5453[1][2] + a53[5]*S5453[1][2];
a54[5]=a53[4]*S5453[2][1] + F2SEG*a53[3]*S5453[2][2] + a53[5]*S5453[2][2];
a54[6]=-(F2SEG*a53[2]) + a53[6];

a55[1]=a54[1];
a55[2]=a54[2];
a55[3]=a54[3];
a55[4]=a54[4];
a55[5]=F3SEG*a54[3] + a54[5];
a55[6]=-(F3SEG*a54[2]) + a54[6];

a56[1]=state[50].thd*v56[2] + a38[1]*S5638[1][1] + a38[2]*S5638[1][2] + a38[3]*S5638[1][3];
a56[2]=-(state[50].thd*v56[1]) + a38[1]*S5638[2][1] + a38[2]*S5638[2][2] + a38[3]*S5638[2][3];
a56[3]=state[50].thdd + a38[2]*S5638[3][2] + a38[3]*S5638[3][3];
a56[4]=state[50].thd*v56[5] + a38[4]*S5638[1][1] + a38[5]*S5638[1][2] + a38[3]*(-(YLF*S5638[1][1]) + XLF*S5638[1][2]) + a38[6]*S5638[1][3] + a38[2]*(ZLF*S5638[1][1] - XLF*S5638[1][3]) + a38[1]*(-(ZLF*S5638[1][2]) + YLF*S5638[1][3]);
a56[5]=-(state[50].thd*v56[4]) + a38[4]*S5638[2][1] + a38[5]*S5638[2][2] + a38[3]*(-(YLF*S5638[2][1]) + XLF*S5638[2][2]) + a38[6]*S5638[2][3] + a38[2]*(ZLF*S5638[2][1] - XLF*S5638[2][3]) + a38[1]*(-(ZLF*S5638[2][2]) + YLF*S5638[2][3]);
a56[6]=XLF*a38[3]*S5638[3][2] + a38[5]*S5638[3][2] - XLF*a38[2]*S5638[3][3] + a38[6]*S5638[3][3] + a38[1]*(-(ZLF*S5638[3][2]) + YLF*S5638[3][3]);

a57[1]=a56[1]*S5756[1][1] + a56[2]*S5756[1][2];
a57[2]=a56[1]*S5756[2][1] + a56[2]*S5756[2][2];
a57[3]=a56[3];
a57[4]=a56[4]*S5756[1][1] + F1SEG*a56[3]*S5756[1][2] + a56[5]*S5756[1][2];
a57[5]=a56[4]*S5756[2][1] + F1SEG*a56[3]*S5756[2][2] + a56[5]*S5756[2][2];
a57[6]=-(F1SEG*a56[2]) + a56[6];

a58[1]=a57[1]*S5857[1][1] + a57[2]*S5857[1][2];
a58[2]=a57[1]*S5857[2][1] + a57[2]*S5857[2][2];
a58[3]=a57[3];
a58[4]=a57[4]*S5857[1][1] + F2SEG*a57[3]*S5857[1][2] + a57[5]*S5857[1][2];
a58[5]=a57[4]*S5857[2][1] + F2SEG*a57[3]*S5857[2][2] + a57[5]*S5857[2][2];
a58[6]=-(F2SEG*a57[2]) + a57[6];

a59[1]=a58[1];
a59[2]=a58[2];
a59[3]=a58[3];
a59[4]=a58[4];
a59[5]=F3SEG*a58[3] + a58[5];
a59[6]=-(F3SEG*a58[2]) + a58[6];

a60[1]=state[32].thd*v60[2] + a3[1]*S603[1][1] + a3[2]*S603[1][2];
a60[2]=-(state[32].thd*v60[1]) + a3[1]*S603[2][1] + a3[2]*S603[2][2];
a60[3]=state[32].thdd + a3[3];
a60[4]=state[32].thd*v60[5] + a3[4]*S603[1][1] - THORAX2NECK*a3[3]*S603[1][2] + a3[5]*S603[1][2];
a60[5]=-(state[32].thd*v60[4]) + a3[4]*S603[2][1] - THORAX2NECK*a3[3]*S603[2][2] + a3[5]*S603[2][2];
a60[6]=THORAX2NECK*a3[2] + a3[6];

a61[1]=state[33].thd*v61[2] + a60[2]*S6160[1][2] + a60[3]*S6160[1][3];
a61[2]=-(state[33].thd*v61[1]) + a60[2]*S6160[2][2] + a60[3]*S6160[2][3];
a61[3]=state[33].thdd + a60[1];
a61[4]=state[33].thd*v61[5] + a60[5]*S6160[1][2] - CERVICAL*a60[1]*S6160[1][3] + a60[6]*S6160[1][3];
a61[5]=-(state[33].thd*v61[4]) + a60[5]*S6160[2][2] - CERVICAL*a60[1]*S6160[2][3] + a60[6]*S6160[2][3];
a61[6]=CERVICAL*a60[3] + a60[4];

a62[1]=state[34].thd*v62[2] + a61[1]*S6261[1][1] + a61[3]*S6261[1][3];
a62[2]=-(state[34].thd*v62[1]) + a61[1]*S6261[2][1] + a61[3]*S6261[2][3];
a62[3]=state[34].thdd + a61[2];
a62[4]=state[34].thd*v62[5] + a61[4]*S6261[1][1] + a61[6]*S6261[1][3];
a62[5]=-(state[34].thd*v62[4]) + a61[4]*S6261[2][1] + a61[6]*S6261[2][3];
a62[6]=a61[5];

a63[1]=state[35].thd*v63[2] + a62[1]*S6362[1][1] + a62[2]*S6362[1][2];
a63[2]=-(state[35].thd*v63[1]) + a62[1]*S6362[2][1] + a62[2]*S6362[2][2];
a63[3]=state[35].thdd + a62[3];
a63[4]=state[35].thd*v63[5] - HEAD*a62[2]*S6362[1][1] + a62[4]*S6362[1][1] + HEAD*a62[1]*S6362[1][2] + a62[5]*S6362[1][2] + a62[3]*(EYEYOFF*S6362[1][1] + EYEXOFF*S6362[1][2]);
a63[5]=-(state[35].thd*v63[4]) - HEAD*a62[2]*S6362[2][1] + a62[4]*S6362[2][1] + HEAD*a62[1]*S6362[2][2] + a62[5]*S6362[2][2] + a62[3]*(EYEYOFF*S6362[2][1] + EYEXOFF*S6362[2][2]);
a63[6]=-(EYEYOFF*a62[1]) - EYEXOFF*a62[2] + a62[6];

a64[1]=state[36].thd*v64[2] + a63[2]*S6463[1][2] + a63[3]*S6463[1][3];
a64[2]=-(state[36].thd*v64[1]) + a63[2]*S6463[2][2] + a63[3]*S6463[2][3];
a64[3]=state[36].thdd - a63[1];
a64[4]=state[36].thd*v64[5] + a63[5]*S6463[1][2] + a63[6]*S6463[1][3];
a64[5]=-(state[36].thd*v64[4]) + a63[5]*S6463[2][2] + a63[6]*S6463[2][3];
a64[6]=-a63[4];

a65[1]=-a64[3];
a65[2]=-a64[2];
a65[3]=-a64[1];
a65[4]=EYE*a64[1] - a64[6];
a65[5]=-a64[5];
a65[6]=-(EYE*a64[3]) - a64[4];

a66[1]=state[37].thd*v66[2] + a62[1]*S6662[1][1] + a62[2]*S6662[1][2];
a66[2]=-(state[37].thd*v66[1]) + a62[1]*S6662[2][1] + a62[2]*S6662[2][2];
a66[3]=state[37].thdd + a62[3];
a66[4]=state[37].thd*v66[5] - HEAD*a62[2]*S6662[1][1] + a62[4]*S6662[1][1] + HEAD*a62[1]*S6662[1][2] + a62[5]*S6662[1][2] + a62[3]*(EYEYOFF*S6662[1][1] - EYEXOFF*S6662[1][2]);
a66[5]=-(state[37].thd*v66[4]) - HEAD*a62[2]*S6662[2][1] + a62[4]*S6662[2][1] + HEAD*a62[1]*S6662[2][2] + a62[5]*S6662[2][2] + a62[3]*(EYEYOFF*S6662[2][1] - EYEXOFF*S6662[2][2]);
a66[6]=-(EYEYOFF*a62[1]) + EYEXOFF*a62[2] + a62[6];

a67[1]=state[38].thd*v67[2] + a66[2]*S6766[1][2] + a66[3]*S6766[1][3];
a67[2]=-(state[38].thd*v67[1]) + a66[2]*S6766[2][2] + a66[3]*S6766[2][3];
a67[3]=state[38].thdd - a66[1];
a67[4]=state[38].thd*v67[5] + a66[5]*S6766[1][2] + a66[6]*S6766[1][3];
a67[5]=-(state[38].thd*v67[4]) + a66[5]*S6766[2][2] + a66[6]*S6766[2][3];
a67[6]=-a66[4];

a68[1]=-a67[3];
a68[2]=-a67[2];
a68[3]=-a67[1];
a68[4]=EYE*a67[1] - a67[6];
a68[5]=-a67[5];
a68[6]=-(EYE*a67[3]) - a67[4];

a69[1]=a62[1];
a69[2]=-a62[2];
a69[3]=-a62[3];
a69[4]=-(TOPofHEAD*a62[2]) + a62[4];
a69[5]=-(TOPofHEAD*a62[1]) - a62[5];
a69[6]=-a62[6];

a70[1]=state[23].thd*v70[2] + a0[1]*S700[1][1] + a0[3]*S700[1][3];
a70[2]=-(state[23].thd*v70[1]) + a0[1]*S700[2][1] + a0[3]*S700[2][3];
a70[3]=state[23].thdd + a0[2];
a70[4]=state[23].thd*v70[5] + a0[4]*S700[1][1] - XHIP*a0[2]*S700[1][3] + a0[6]*S700[1][3];
a70[5]=-(state[23].thd*v70[4]) + a0[4]*S700[2][1] - XHIP*a0[2]*S700[2][3] + a0[6]*S700[2][3];
a70[6]=XHIP*a0[3] + a0[5];

a71[1]=state[22].thd*v71[2] + a70[1]*S7170[1][1] + a70[3]*S7170[1][3];
a71[2]=-(state[22].thd*v71[1]) + a70[1]*S7170[2][1] + a70[3]*S7170[2][3];
a71[3]=state[22].thdd - a70[2];
a71[4]=state[22].thd*v71[5] + a70[4]*S7170[1][1] + a70[6]*S7170[1][3];
a71[5]=-(state[22].thd*v71[4]) + a70[4]*S7170[2][1] + a70[6]*S7170[2][3];
a71[6]=-a70[5];

a72[1]=state[24].thd*v72[2] + a71[1]*S7271[1][1] + a71[2]*S7271[1][2] + a71[3]*S7271[1][3];
a72[2]=-(state[24].thd*v72[1]) + a71[1]*S7271[2][1] + a71[2]*S7271[2][2] + a71[3]*S7271[2][3];
a72[3]=state[24].thdd + a71[1]*S7271[3][1] + a71[2]*S7271[3][2];
a72[4]=state[24].thd*v72[5] + a71[4]*S7271[1][1] + YHIP*a71[3]*S7271[1][2] + a71[5]*S7271[1][2] - YHIP*a71[2]*S7271[1][3] + a71[6]*S7271[1][3];
a72[5]=-(state[24].thd*v72[4]) + a71[4]*S7271[2][1] + YHIP*a71[3]*S7271[2][2] + a71[5]*S7271[2][2] - YHIP*a71[2]*S7271[2][3] + a71[6]*S7271[2][3];
a72[6]=a71[4]*S7271[3][1] + YHIP*a71[3]*S7271[3][2] + a71[5]*S7271[3][2];

a73[1]=state[25].thd*v73[2] + a72[1]*S7372[1][1] + a72[3]*S7372[1][3];
a73[2]=-(state[25].thd*v73[1]) + a72[1]*S7372[2][1] + a72[3]*S7372[2][3];
a73[3]=state[25].thdd + a72[2];
a73[4]=state[25].thd*v73[5] + a72[4]*S7372[1][1] + a72[6]*S7372[1][3] + a72[2]*(UPPERLEGMOD*S7372[1][1] - YKNEE*S7372[1][3]);
a73[5]=-(state[25].thd*v73[4]) + a72[4]*S7372[2][1] + a72[6]*S7372[2][3] + a72[2]*(UPPERLEGMOD*S7372[2][1] - YKNEE*S7372[2][3]);
a73[6]=-(UPPERLEGMOD*a72[1]) + YKNEE*a72[3] + a72[5];

a74[1]=state[26].thd*v74[2] + a73[1]*S7473[1][1] + a73[3]*S7473[1][3];
a74[2]=-(state[26].thd*v74[1]) + a73[1]*S7473[2][1] + a73[3]*S7473[2][3];
a74[3]=state[26].thdd - a73[2];
a74[4]=state[26].thd*v74[5] + a73[4]*S7473[1][1] + a73[6]*S7473[1][3];
a74[5]=-(state[26].thd*v74[4]) + a73[4]*S7473[2][1] + a73[6]*S7473[2][3];
a74[6]=-a73[5];

a75[1]=state[27].thd*v75[2] + a74[1]*S7574[1][1] + a74[3]*S7574[1][3];
a75[2]=-(state[27].thd*v75[1]) + a74[1]*S7574[2][1] + a74[3]*S7574[2][3];
a75[3]=state[27].thdd - a74[2];
a75[4]=state[27].thd*v75[5] + LOWERLEG*a74[2]*S7574[1][1] + a74[4]*S7574[1][1] + a74[6]*S7574[1][3];
a75[5]=-(state[27].thd*v75[4]) + LOWERLEG*a74[2]*S7574[2][1] + a74[4]*S7574[2][1] + a74[6]*S7574[2][3];
a75[6]=LOWERLEG*a74[1] - a74[5];

a76[1]=state[28].thd*v76[2] + a75[1]*S7675[1][1] + a75[3]*S7675[1][3];
a76[2]=-(state[28].thd*v76[1]) + a75[1]*S7675[2][1] + a75[3]*S7675[2][3];
a76[3]=state[28].thdd + a75[2];
a76[4]=state[28].thd*v76[5] + a75[4]*S7675[1][1] + a75[6]*S7675[1][3];
a76[5]=-(state[28].thd*v76[4]) + a75[4]*S7675[2][1] + a75[6]*S7675[2][3];
a76[6]=a75[5];

a77[1]=a76[1];
a77[2]=a76[2];
a77[3]=a76[3];
a77[4]=YTOE*a76[2] + XTOEOUTER*a76[3] + a76[4];
a77[5]=-(YTOE*a76[1]) + ZTOE*a76[3] + a76[5];
a77[6]=-(XTOEOUTER*a76[1]) - ZTOE*a76[2] + a76[6];

a78[1]=a76[1];
a78[2]=a76[2];
a78[3]=a76[3];
a78[4]=YTOE*a76[2] - XTOEINNER*a76[3] + a76[4];
a78[5]=-(YTOE*a76[1]) + ZTOE*a76[3] + a76[5];
a78[6]=XTOEINNER*a76[1] - ZTOE*a76[2] + a76[6];

a79[1]=a76[1];
a79[2]=a76[2];
a79[3]=a76[3];
a79[4]=YMETATARSAL*a76[2] + XMETATARSALOUTER*a76[3] + a76[4];
a79[5]=-(YMETATARSAL*a76[1]) + ZTOE*a76[3] + a76[5];
a79[6]=-(XMETATARSALOUTER*a76[1]) - ZTOE*a76[2] + a76[6];

a80[1]=a76[1];
a80[2]=a76[2];
a80[3]=a76[3];
a80[4]=YMETATARSAL*a76[2] - XMETATARSALINNER*a76[3] + a76[4];
a80[5]=-(YMETATARSAL*a76[1]) + ZTOE*a76[3] + a76[5];
a80[6]=XMETATARSALINNER*a76[1] - ZTOE*a76[2] + a76[6];

a81[1]=a76[1];
a81[2]=a76[2];
a81[3]=a76[3];
a81[4]=-(YHEEL*a76[2]) + XHEELOUTER*a76[3] + a76[4];
a81[5]=YHEEL*a76[1] + ZHEEL*a76[3] + a76[5];
a81[6]=-(XHEELOUTER*a76[1]) - ZHEEL*a76[2] + a76[6];

a82[1]=a76[1];
a82[2]=a76[2];
a82[3]=a76[3];
a82[4]=-(YHEEL*a76[2]) - XHEELINNER*a76[3] + a76[4];
a82[5]=YHEEL*a76[1] + ZHEEL*a76[3] + a76[5];
a82[6]=XHEELINNER*a76[1] - ZHEEL*a76[2] + a76[6];

a83[1]=a76[1]*S8376[1][1] + a76[2]*S8376[1][2] + a76[3]*S8376[1][3];
a83[2]=a76[1]*S8376[2][1] + a76[2]*S8376[2][2] + a76[3]*S8376[2][3];
a83[3]=a76[1]*S8376[3][1] + a76[2]*S8376[3][2] + a76[3]*S8376[3][3];
a83[4]=a76[4]*S8376[1][1] + a76[5]*S8376[1][2] + a76[3]*(-(eff[3].x[2]*S8376[1][1]) + eff[3].x[1]*S8376[1][2]) + a76[6]*S8376[1][3] + a76[2]*(eff[3].x[3]*S8376[1][1] - eff[3].x[1]*S8376[1][3]) + a76[1]*(-(eff[3].x[3]*S8376[1][2]) + eff[3].x[2]*S8376[1][3]);
a83[5]=a76[4]*S8376[2][1] + a76[5]*S8376[2][2] + a76[3]*(-(eff[3].x[2]*S8376[2][1]) + eff[3].x[1]*S8376[2][2]) + a76[6]*S8376[2][3] + a76[2]*(eff[3].x[3]*S8376[2][1] - eff[3].x[1]*S8376[2][3]) + a76[1]*(-(eff[3].x[3]*S8376[2][2]) + eff[3].x[2]*S8376[2][3]);
a83[6]=a76[4]*S8376[3][1] + a76[5]*S8376[3][2] + a76[3]*(-(eff[3].x[2]*S8376[3][1]) + eff[3].x[1]*S8376[3][2]) + a76[6]*S8376[3][3] + a76[2]*(eff[3].x[3]*S8376[3][1] - eff[3].x[1]*S8376[3][3]) + a76[1]*(-(eff[3].x[3]*S8376[3][2]) + eff[3].x[2]*S8376[3][3]);

a84[1]=state[16].thd*v84[2] + a0[1]*S840[1][1] + a0[3]*S840[1][3];
a84[2]=-(state[16].thd*v84[1]) + a0[1]*S840[2][1] + a0[3]*S840[2][3];
a84[3]=state[16].thdd - a0[2];
a84[4]=state[16].thd*v84[5] + a0[4]*S840[1][1] + XHIP*a0[2]*S840[1][3] + a0[6]*S840[1][3];
a84[5]=-(state[16].thd*v84[4]) + a0[4]*S840[2][1] + XHIP*a0[2]*S840[2][3] + a0[6]*S840[2][3];
a84[6]=XHIP*a0[3] - a0[5];

a85[1]=state[15].thd*v85[2] + a84[1]*S8584[1][1] + a84[3]*S8584[1][3];
a85[2]=-(state[15].thd*v85[1]) + a84[1]*S8584[2][1] + a84[3]*S8584[2][3];
a85[3]=state[15].thdd + a84[2];
a85[4]=state[15].thd*v85[5] + a84[4]*S8584[1][1] + a84[6]*S8584[1][3];
a85[5]=-(state[15].thd*v85[4]) + a84[4]*S8584[2][1] + a84[6]*S8584[2][3];
a85[6]=a84[5];

a86[1]=state[17].thd*v86[2] + a85[1]*S8685[1][1] + a85[2]*S8685[1][2] + a85[3]*S8685[1][3];
a86[2]=-(state[17].thd*v86[1]) + a85[1]*S8685[2][1] + a85[2]*S8685[2][2] + a85[3]*S8685[2][3];
a86[3]=state[17].thdd + a85[1]*S8685[3][1] + a85[2]*S8685[3][2];
a86[4]=state[17].thd*v86[5] + a85[4]*S8685[1][1] + YHIP*a85[3]*S8685[1][2] + a85[5]*S8685[1][2] - YHIP*a85[2]*S8685[1][3] + a85[6]*S8685[1][3];
a86[5]=-(state[17].thd*v86[4]) + a85[4]*S8685[2][1] + YHIP*a85[3]*S8685[2][2] + a85[5]*S8685[2][2] - YHIP*a85[2]*S8685[2][3] + a85[6]*S8685[2][3];
a86[6]=a85[4]*S8685[3][1] + YHIP*a85[3]*S8685[3][2] + a85[5]*S8685[3][2];

a87[1]=state[18].thd*v87[2] + a86[1]*S8786[1][1] + a86[3]*S8786[1][3];
a87[2]=-(state[18].thd*v87[1]) + a86[1]*S8786[2][1] + a86[3]*S8786[2][3];
a87[3]=state[18].thdd - a86[2];
a87[4]=state[18].thd*v87[5] + a86[4]*S8786[1][1] + a86[6]*S8786[1][3] + a86[2]*(-(UPPERLEGMOD*S8786[1][1]) - YKNEE*S8786[1][3]);
a87[5]=-(state[18].thd*v87[4]) + a86[4]*S8786[2][1] + a86[6]*S8786[2][3] + a86[2]*(-(UPPERLEGMOD*S8786[2][1]) - YKNEE*S8786[2][3]);
a87[6]=-(UPPERLEGMOD*a86[1]) - YKNEE*a86[3] - a86[5];

a88[1]=state[19].thd*v88[2] + a87[1]*S8887[1][1] + a87[3]*S8887[1][3];
a88[2]=-(state[19].thd*v88[1]) + a87[1]*S8887[2][1] + a87[3]*S8887[2][3];
a88[3]=state[19].thdd + a87[2];
a88[4]=state[19].thd*v88[5] + a87[4]*S8887[1][1] + a87[6]*S8887[1][3];
a88[5]=-(state[19].thd*v88[4]) + a87[4]*S8887[2][1] + a87[6]*S8887[2][3];
a88[6]=a87[5];

a89[1]=state[20].thd*v89[2] + a88[1]*S8988[1][1] + a88[3]*S8988[1][3];
a89[2]=-(state[20].thd*v89[1]) + a88[1]*S8988[2][1] + a88[3]*S8988[2][3];
a89[3]=state[20].thdd + a88[2];
a89[4]=state[20].thd*v89[5] - LOWERLEG*a88[2]*S8988[1][1] + a88[4]*S8988[1][1] + a88[6]*S8988[1][3];
a89[5]=-(state[20].thd*v89[4]) - LOWERLEG*a88[2]*S8988[2][1] + a88[4]*S8988[2][1] + a88[6]*S8988[2][3];
a89[6]=LOWERLEG*a88[1] + a88[5];

a90[1]=state[21].thd*v90[2] + a89[1]*S9089[1][1] + a89[3]*S9089[1][3];
a90[2]=-(state[21].thd*v90[1]) + a89[1]*S9089[2][1] + a89[3]*S9089[2][3];
a90[3]=state[21].thdd - a89[2];
a90[4]=state[21].thd*v90[5] + a89[4]*S9089[1][1] + a89[6]*S9089[1][3];
a90[5]=-(state[21].thd*v90[4]) + a89[4]*S9089[2][1] + a89[6]*S9089[2][3];
a90[6]=-a89[5];

a91[1]=a90[1];
a91[2]=a90[2];
a91[3]=a90[3];
a91[4]=-(YTOE*a90[2]) + XTOEOUTER*a90[3] + a90[4];
a91[5]=YTOE*a90[1] + ZTOE*a90[3] + a90[5];
a91[6]=-(XTOEOUTER*a90[1]) - ZTOE*a90[2] + a90[6];

a92[1]=a90[1];
a92[2]=a90[2];
a92[3]=a90[3];
a92[4]=-(YTOE*a90[2]) - XTOEINNER*a90[3] + a90[4];
a92[5]=YTOE*a90[1] + ZTOE*a90[3] + a90[5];
a92[6]=XTOEINNER*a90[1] - ZTOE*a90[2] + a90[6];

a93[1]=a90[1];
a93[2]=a90[2];
a93[3]=a90[3];
a93[4]=-(YMETATARSAL*a90[2]) + XMETATARSALOUTER*a90[3] + a90[4];
a93[5]=YMETATARSAL*a90[1] + ZTOE*a90[3] + a90[5];
a93[6]=-(XMETATARSALOUTER*a90[1]) - ZTOE*a90[2] + a90[6];

a94[1]=a90[1];
a94[2]=a90[2];
a94[3]=a90[3];
a94[4]=-(YMETATARSAL*a90[2]) - XMETATARSALINNER*a90[3] + a90[4];
a94[5]=YMETATARSAL*a90[1] + ZTOE*a90[3] + a90[5];
a94[6]=XMETATARSALINNER*a90[1] - ZTOE*a90[2] + a90[6];

a95[1]=a90[1];
a95[2]=a90[2];
a95[3]=a90[3];
a95[4]=YHEEL*a90[2] + XHEELOUTER*a90[3] + a90[4];
a95[5]=-(YHEEL*a90[1]) + ZHEEL*a90[3] + a90[5];
a95[6]=-(XHEELOUTER*a90[1]) - ZHEEL*a90[2] + a90[6];

a96[1]=a90[1];
a96[2]=a90[2];
a96[3]=a90[3];
a96[4]=YHEEL*a90[2] - XHEELINNER*a90[3] + a90[4];
a96[5]=-(YHEEL*a90[1]) + ZHEEL*a90[3] + a90[5];
a96[6]=XHEELINNER*a90[1] - ZHEEL*a90[2] + a90[6];

a97[1]=a90[1]*S9790[1][1] + a90[2]*S9790[1][2] + a90[3]*S9790[1][3];
a97[2]=a90[1]*S9790[2][1] + a90[2]*S9790[2][2] + a90[3]*S9790[2][3];
a97[3]=a90[1]*S9790[3][1] + a90[2]*S9790[3][2] + a90[3]*S9790[3][3];
a97[4]=a90[4]*S9790[1][1] + a90[5]*S9790[1][2] + a90[3]*(-(eff[4].x[2]*S9790[1][1]) + eff[4].x[1]*S9790[1][2]) + a90[6]*S9790[1][3] + a90[2]*(eff[4].x[3]*S9790[1][1] - eff[4].x[1]*S9790[1][3]) + a90[1]*(-(eff[4].x[3]*S9790[1][2]) + eff[4].x[2]*S9790[1][3]);
a97[5]=a90[4]*S9790[2][1] + a90[5]*S9790[2][2] + a90[3]*(-(eff[4].x[2]*S9790[2][1]) + eff[4].x[1]*S9790[2][2]) + a90[6]*S9790[2][3] + a90[2]*(eff[4].x[3]*S9790[2][1] - eff[4].x[1]*S9790[2][3]) + a90[1]*(-(eff[4].x[3]*S9790[2][2]) + eff[4].x[2]*S9790[2][3]);
a97[6]=a90[4]*S9790[3][1] + a90[5]*S9790[3][2] + a90[3]*(-(eff[4].x[2]*S9790[3][1]) + eff[4].x[1]*S9790[3][2]) + a90[6]*S9790[3][3] + a90[2]*(eff[4].x[3]*S9790[3][1] - eff[4].x[1]*S9790[3][3]) + a90[1]*(-(eff[4].x[3]*S9790[3][2]) + eff[4].x[2]*S9790[3][3]);



}


void
hermes_InvDynNEfunc6(void)
     {
/* net forces and external forces for each joint */
fnet0[1]=links[0].m*a0[4] - a0[3]*links[0].mcm[2] + a0[2]*links[0].mcm[3] - links[0].mcm[1]*Power(v0[2],2) - links[0].mcm[1]*Power(v0[3],2) + v0[1]*(links[0].mcm[2]*v0[2] + links[0].mcm[3]*v0[3]) - links[0].m*v0[3]*v0[5] + links[0].m*v0[2]*v0[6];
fnet0[2]=links[0].m*a0[5] + a0[3]*links[0].mcm[1] - a0[1]*links[0].mcm[3] - links[0].mcm[2]*Power(v0[1],2) - links[0].mcm[2]*Power(v0[3],2) + v0[2]*(links[0].mcm[1]*v0[1] + links[0].mcm[3]*v0[3]) + links[0].m*v0[3]*v0[4] - links[0].m*v0[1]*v0[6];
fnet0[3]=links[0].m*a0[6] - a0[2]*links[0].mcm[1] + a0[1]*links[0].mcm[2] - links[0].mcm[3]*Power(v0[1],2) - links[0].mcm[3]*Power(v0[2],2) + (links[0].mcm[1]*v0[1] + links[0].mcm[2]*v0[2])*v0[3] - links[0].m*v0[2]*v0[4] + links[0].m*v0[1]*v0[5];
fnet0[4]=a0[6]*links[0].mcm[2] - a0[5]*links[0].mcm[3] + (-(links[0].mcm[2]*v0[2]) - links[0].mcm[3]*v0[3])*v0[4] + (links[0].mcm[1]*v0[3] + links[0].m*v0[5])*v0[6] + v0[5]*(links[0].mcm[1]*v0[2] - links[0].m*v0[6]) + a0[1]*links[0].inertia[1][1] + a0[2]*links[0].inertia[1][2] + a0[3]*links[0].inertia[1][3] + v0[1]*(links[0].mcm[2]*v0[5] + links[0].mcm[3]*v0[6] - v0[3]*links[0].inertia[1][2] + v0[2]*links[0].inertia[1][3]) + v0[2]*(-(links[0].mcm[1]*v0[5]) - v0[3]*links[0].inertia[2][2] + v0[2]*links[0].inertia[2][3]) + v0[3]*(-(links[0].mcm[1]*v0[6]) - v0[3]*links[0].inertia[2][3] + v0[2]*links[0].inertia[3][3]);
fnet0[5]=-(a0[6]*links[0].mcm[1]) + a0[4]*links[0].mcm[3] + (-(links[0].mcm[1]*v0[1]) - links[0].mcm[3]*v0[3])*v0[5] + (links[0].mcm[2]*v0[3] - links[0].m*v0[4])*v0[6] + v0[4]*(links[0].mcm[2]*v0[1] + links[0].m*v0[6]) + a0[1]*links[0].inertia[1][2] + v0[1]*(-(links[0].mcm[2]*v0[4]) + v0[3]*links[0].inertia[1][1] - v0[1]*links[0].inertia[1][3]) + a0[2]*links[0].inertia[2][2] + a0[3]*links[0].inertia[2][3] + v0[2]*(links[0].mcm[1]*v0[4] + links[0].mcm[3]*v0[6] + v0[3]*links[0].inertia[1][2] - v0[1]*links[0].inertia[2][3]) + v0[3]*(-(links[0].mcm[2]*v0[6]) + v0[3]*links[0].inertia[1][3] - v0[1]*links[0].inertia[3][3]);
fnet0[6]=a0[5]*links[0].mcm[1] - a0[4]*links[0].mcm[2] + (links[0].mcm[3]*v0[2] + links[0].m*v0[4])*v0[5] + v0[4]*(links[0].mcm[3]*v0[1] - links[0].m*v0[5]) + (-(links[0].mcm[1]*v0[1]) - links[0].mcm[2]*v0[2])*v0[6] + v0[1]*(-(links[0].mcm[3]*v0[4]) - v0[2]*links[0].inertia[1][1] + v0[1]*links[0].inertia[1][2]) + a0[1]*links[0].inertia[1][3] + v0[2]*(-(links[0].mcm[3]*v0[5]) - v0[2]*links[0].inertia[1][2] + v0[1]*links[0].inertia[2][2]) + a0[2]*links[0].inertia[2][3] + v0[3]*(links[0].mcm[1]*v0[4] + links[0].mcm[2]*v0[5] - v0[2]*links[0].inertia[1][3] + v0[1]*links[0].inertia[2][3]) + a0[3]*links[0].inertia[3][3];

fnet1[1]=links[29].m*a1[4] - a1[3]*links[29].mcm[2] + a1[2]*links[29].mcm[3] - links[29].mcm[1]*Power(v1[2],2) - links[29].mcm[1]*Power(v1[3],2) + v1[1]*(links[29].mcm[2]*v1[2] + links[29].mcm[3]*v1[3]) - links[29].m*v1[3]*v1[5] + links[29].m*v1[2]*v1[6];
fnet1[2]=links[29].m*a1[5] + a1[3]*links[29].mcm[1] - a1[1]*links[29].mcm[3] - links[29].mcm[2]*Power(v1[1],2) - links[29].mcm[2]*Power(v1[3],2) + v1[2]*(links[29].mcm[1]*v1[1] + links[29].mcm[3]*v1[3]) + links[29].m*v1[3]*v1[4] - links[29].m*v1[1]*v1[6];
fnet1[3]=links[29].m*a1[6] - a1[2]*links[29].mcm[1] + a1[1]*links[29].mcm[2] - links[29].mcm[3]*Power(v1[1],2) - links[29].mcm[3]*Power(v1[2],2) + (links[29].mcm[1]*v1[1] + links[29].mcm[2]*v1[2])*v1[3] - links[29].m*v1[2]*v1[4] + links[29].m*v1[1]*v1[5];
fnet1[4]=a1[6]*links[29].mcm[2] - a1[5]*links[29].mcm[3] + (-(links[29].mcm[2]*v1[2]) - links[29].mcm[3]*v1[3])*v1[4] + (links[29].mcm[1]*v1[3] + links[29].m*v1[5])*v1[6] + v1[5]*(links[29].mcm[1]*v1[2] - links[29].m*v1[6]) + a1[1]*links[29].inertia[1][1] + a1[2]*links[29].inertia[1][2] + a1[3]*links[29].inertia[1][3] + v1[1]*(links[29].mcm[2]*v1[5] + links[29].mcm[3]*v1[6] - v1[3]*links[29].inertia[1][2] + v1[2]*links[29].inertia[1][3]) + v1[2]*(-(links[29].mcm[1]*v1[5]) - v1[3]*links[29].inertia[2][2] + v1[2]*links[29].inertia[2][3]) + v1[3]*(-(links[29].mcm[1]*v1[6]) - v1[3]*links[29].inertia[2][3] + v1[2]*links[29].inertia[3][3]);
fnet1[5]=-(a1[6]*links[29].mcm[1]) + a1[4]*links[29].mcm[3] + (-(links[29].mcm[1]*v1[1]) - links[29].mcm[3]*v1[3])*v1[5] + (links[29].mcm[2]*v1[3] - links[29].m*v1[4])*v1[6] + v1[4]*(links[29].mcm[2]*v1[1] + links[29].m*v1[6]) + a1[1]*links[29].inertia[1][2] + v1[1]*(-(links[29].mcm[2]*v1[4]) + v1[3]*links[29].inertia[1][1] - v1[1]*links[29].inertia[1][3]) + a1[2]*links[29].inertia[2][2] + a1[3]*links[29].inertia[2][3] + v1[2]*(links[29].mcm[1]*v1[4] + links[29].mcm[3]*v1[6] + v1[3]*links[29].inertia[1][2] - v1[1]*links[29].inertia[2][3]) + v1[3]*(-(links[29].mcm[2]*v1[6]) + v1[3]*links[29].inertia[1][3] - v1[1]*links[29].inertia[3][3]);
fnet1[6]=a1[5]*links[29].mcm[1] - a1[4]*links[29].mcm[2] + (links[29].mcm[3]*v1[2] + links[29].m*v1[4])*v1[5] + v1[4]*(links[29].mcm[3]*v1[1] - links[29].m*v1[5]) + (-(links[29].mcm[1]*v1[1]) - links[29].mcm[2]*v1[2])*v1[6] + v1[1]*(-(links[29].mcm[3]*v1[4]) - v1[2]*links[29].inertia[1][1] + v1[1]*links[29].inertia[1][2]) + a1[1]*links[29].inertia[1][3] + v1[2]*(-(links[29].mcm[3]*v1[5]) - v1[2]*links[29].inertia[1][2] + v1[1]*links[29].inertia[2][2]) + a1[2]*links[29].inertia[2][3] + v1[3]*(links[29].mcm[1]*v1[4] + links[29].mcm[2]*v1[5] - v1[2]*links[29].inertia[1][3] + v1[1]*links[29].inertia[2][3]) + a1[3]*links[29].inertia[3][3];

fnet2[1]=links[30].m*a2[4] - a2[3]*links[30].mcm[2] + a2[2]*links[30].mcm[3] - links[30].mcm[1]*Power(v2[2],2) - links[30].mcm[1]*Power(v2[3],2) + v2[1]*(links[30].mcm[2]*v2[2] + links[30].mcm[3]*v2[3]) - links[30].m*v2[3]*v2[5] + links[30].m*v2[2]*v2[6];
fnet2[2]=links[30].m*a2[5] + a2[3]*links[30].mcm[1] - a2[1]*links[30].mcm[3] - links[30].mcm[2]*Power(v2[1],2) - links[30].mcm[2]*Power(v2[3],2) + v2[2]*(links[30].mcm[1]*v2[1] + links[30].mcm[3]*v2[3]) + links[30].m*v2[3]*v2[4] - links[30].m*v2[1]*v2[6];
fnet2[3]=links[30].m*a2[6] - a2[2]*links[30].mcm[1] + a2[1]*links[30].mcm[2] - links[30].mcm[3]*Power(v2[1],2) - links[30].mcm[3]*Power(v2[2],2) + (links[30].mcm[1]*v2[1] + links[30].mcm[2]*v2[2])*v2[3] - links[30].m*v2[2]*v2[4] + links[30].m*v2[1]*v2[5];
fnet2[4]=a2[6]*links[30].mcm[2] - a2[5]*links[30].mcm[3] + (-(links[30].mcm[2]*v2[2]) - links[30].mcm[3]*v2[3])*v2[4] + (links[30].mcm[1]*v2[3] + links[30].m*v2[5])*v2[6] + v2[5]*(links[30].mcm[1]*v2[2] - links[30].m*v2[6]) + a2[1]*links[30].inertia[1][1] + a2[2]*links[30].inertia[1][2] + a2[3]*links[30].inertia[1][3] + v2[1]*(links[30].mcm[2]*v2[5] + links[30].mcm[3]*v2[6] - v2[3]*links[30].inertia[1][2] + v2[2]*links[30].inertia[1][3]) + v2[2]*(-(links[30].mcm[1]*v2[5]) - v2[3]*links[30].inertia[2][2] + v2[2]*links[30].inertia[2][3]) + v2[3]*(-(links[30].mcm[1]*v2[6]) - v2[3]*links[30].inertia[2][3] + v2[2]*links[30].inertia[3][3]);
fnet2[5]=-(a2[6]*links[30].mcm[1]) + a2[4]*links[30].mcm[3] + (-(links[30].mcm[1]*v2[1]) - links[30].mcm[3]*v2[3])*v2[5] + (links[30].mcm[2]*v2[3] - links[30].m*v2[4])*v2[6] + v2[4]*(links[30].mcm[2]*v2[1] + links[30].m*v2[6]) + a2[1]*links[30].inertia[1][2] + v2[1]*(-(links[30].mcm[2]*v2[4]) + v2[3]*links[30].inertia[1][1] - v2[1]*links[30].inertia[1][3]) + a2[2]*links[30].inertia[2][2] + a2[3]*links[30].inertia[2][3] + v2[2]*(links[30].mcm[1]*v2[4] + links[30].mcm[3]*v2[6] + v2[3]*links[30].inertia[1][2] - v2[1]*links[30].inertia[2][3]) + v2[3]*(-(links[30].mcm[2]*v2[6]) + v2[3]*links[30].inertia[1][3] - v2[1]*links[30].inertia[3][3]);
fnet2[6]=a2[5]*links[30].mcm[1] - a2[4]*links[30].mcm[2] + (links[30].mcm[3]*v2[2] + links[30].m*v2[4])*v2[5] + v2[4]*(links[30].mcm[3]*v2[1] - links[30].m*v2[5]) + (-(links[30].mcm[1]*v2[1]) - links[30].mcm[2]*v2[2])*v2[6] + v2[1]*(-(links[30].mcm[3]*v2[4]) - v2[2]*links[30].inertia[1][1] + v2[1]*links[30].inertia[1][2]) + a2[1]*links[30].inertia[1][3] + v2[2]*(-(links[30].mcm[3]*v2[5]) - v2[2]*links[30].inertia[1][2] + v2[1]*links[30].inertia[2][2]) + a2[2]*links[30].inertia[2][3] + v2[3]*(links[30].mcm[1]*v2[4] + links[30].mcm[2]*v2[5] - v2[2]*links[30].inertia[1][3] + v2[1]*links[30].inertia[2][3]) + a2[3]*links[30].inertia[3][3];

fnet3[1]=links[31].m*a3[4] - a3[3]*links[31].mcm[2] + a3[2]*links[31].mcm[3] - links[31].mcm[1]*Power(v3[2],2) - links[31].mcm[1]*Power(v3[3],2) + v3[1]*(links[31].mcm[2]*v3[2] + links[31].mcm[3]*v3[3]) - links[31].m*v3[3]*v3[5] + links[31].m*v3[2]*v3[6];
fnet3[2]=links[31].m*a3[5] + a3[3]*links[31].mcm[1] - a3[1]*links[31].mcm[3] - links[31].mcm[2]*Power(v3[1],2) - links[31].mcm[2]*Power(v3[3],2) + v3[2]*(links[31].mcm[1]*v3[1] + links[31].mcm[3]*v3[3]) + links[31].m*v3[3]*v3[4] - links[31].m*v3[1]*v3[6];
fnet3[3]=links[31].m*a3[6] - a3[2]*links[31].mcm[1] + a3[1]*links[31].mcm[2] - links[31].mcm[3]*Power(v3[1],2) - links[31].mcm[3]*Power(v3[2],2) + (links[31].mcm[1]*v3[1] + links[31].mcm[2]*v3[2])*v3[3] - links[31].m*v3[2]*v3[4] + links[31].m*v3[1]*v3[5];
fnet3[4]=a3[6]*links[31].mcm[2] - a3[5]*links[31].mcm[3] + (-(links[31].mcm[2]*v3[2]) - links[31].mcm[3]*v3[3])*v3[4] + (links[31].mcm[1]*v3[3] + links[31].m*v3[5])*v3[6] + v3[5]*(links[31].mcm[1]*v3[2] - links[31].m*v3[6]) + a3[1]*links[31].inertia[1][1] + a3[2]*links[31].inertia[1][2] + a3[3]*links[31].inertia[1][3] + v3[1]*(links[31].mcm[2]*v3[5] + links[31].mcm[3]*v3[6] - v3[3]*links[31].inertia[1][2] + v3[2]*links[31].inertia[1][3]) + v3[2]*(-(links[31].mcm[1]*v3[5]) - v3[3]*links[31].inertia[2][2] + v3[2]*links[31].inertia[2][3]) + v3[3]*(-(links[31].mcm[1]*v3[6]) - v3[3]*links[31].inertia[2][3] + v3[2]*links[31].inertia[3][3]);
fnet3[5]=-(a3[6]*links[31].mcm[1]) + a3[4]*links[31].mcm[3] + (-(links[31].mcm[1]*v3[1]) - links[31].mcm[3]*v3[3])*v3[5] + (links[31].mcm[2]*v3[3] - links[31].m*v3[4])*v3[6] + v3[4]*(links[31].mcm[2]*v3[1] + links[31].m*v3[6]) + a3[1]*links[31].inertia[1][2] + v3[1]*(-(links[31].mcm[2]*v3[4]) + v3[3]*links[31].inertia[1][1] - v3[1]*links[31].inertia[1][3]) + a3[2]*links[31].inertia[2][2] + a3[3]*links[31].inertia[2][3] + v3[2]*(links[31].mcm[1]*v3[4] + links[31].mcm[3]*v3[6] + v3[3]*links[31].inertia[1][2] - v3[1]*links[31].inertia[2][3]) + v3[3]*(-(links[31].mcm[2]*v3[6]) + v3[3]*links[31].inertia[1][3] - v3[1]*links[31].inertia[3][3]);
fnet3[6]=a3[5]*links[31].mcm[1] - a3[4]*links[31].mcm[2] + (links[31].mcm[3]*v3[2] + links[31].m*v3[4])*v3[5] + v3[4]*(links[31].mcm[3]*v3[1] - links[31].m*v3[5]) + (-(links[31].mcm[1]*v3[1]) - links[31].mcm[2]*v3[2])*v3[6] + v3[1]*(-(links[31].mcm[3]*v3[4]) - v3[2]*links[31].inertia[1][1] + v3[1]*links[31].inertia[1][2]) + a3[1]*links[31].inertia[1][3] + v3[2]*(-(links[31].mcm[3]*v3[5]) - v3[2]*links[31].inertia[1][2] + v3[1]*links[31].inertia[2][2]) + a3[2]*links[31].inertia[2][3] + v3[3]*(links[31].mcm[1]*v3[4] + links[31].mcm[2]*v3[5] - v3[2]*links[31].inertia[1][3] + v3[1]*links[31].inertia[2][3]) + a3[3]*links[31].inertia[3][3];

fnet4[1]=links[1].m*a4[4] - a4[3]*links[1].mcm[2] + a4[2]*links[1].mcm[3] - links[1].mcm[1]*Power(v4[2],2) - links[1].mcm[1]*Power(v4[3],2) + v4[1]*(links[1].mcm[2]*v4[2] + links[1].mcm[3]*v4[3]) - links[1].m*v4[3]*v4[5] + links[1].m*v4[2]*v4[6];
fnet4[2]=links[1].m*a4[5] + a4[3]*links[1].mcm[1] - a4[1]*links[1].mcm[3] - links[1].mcm[2]*Power(v4[1],2) - links[1].mcm[2]*Power(v4[3],2) + v4[2]*(links[1].mcm[1]*v4[1] + links[1].mcm[3]*v4[3]) + links[1].m*v4[3]*v4[4] - links[1].m*v4[1]*v4[6];
fnet4[3]=links[1].m*a4[6] - a4[2]*links[1].mcm[1] + a4[1]*links[1].mcm[2] - links[1].mcm[3]*Power(v4[1],2) - links[1].mcm[3]*Power(v4[2],2) + (links[1].mcm[1]*v4[1] + links[1].mcm[2]*v4[2])*v4[3] - links[1].m*v4[2]*v4[4] + links[1].m*v4[1]*v4[5];
fnet4[4]=a4[6]*links[1].mcm[2] - a4[5]*links[1].mcm[3] + (-(links[1].mcm[2]*v4[2]) - links[1].mcm[3]*v4[3])*v4[4] + (links[1].mcm[1]*v4[3] + links[1].m*v4[5])*v4[6] + v4[5]*(links[1].mcm[1]*v4[2] - links[1].m*v4[6]) + a4[1]*links[1].inertia[1][1] + a4[2]*links[1].inertia[1][2] + a4[3]*links[1].inertia[1][3] + v4[1]*(links[1].mcm[2]*v4[5] + links[1].mcm[3]*v4[6] - v4[3]*links[1].inertia[1][2] + v4[2]*links[1].inertia[1][3]) + v4[2]*(-(links[1].mcm[1]*v4[5]) - v4[3]*links[1].inertia[2][2] + v4[2]*links[1].inertia[2][3]) + v4[3]*(-(links[1].mcm[1]*v4[6]) - v4[3]*links[1].inertia[2][3] + v4[2]*links[1].inertia[3][3]);
fnet4[5]=-(a4[6]*links[1].mcm[1]) + a4[4]*links[1].mcm[3] + (-(links[1].mcm[1]*v4[1]) - links[1].mcm[3]*v4[3])*v4[5] + (links[1].mcm[2]*v4[3] - links[1].m*v4[4])*v4[6] + v4[4]*(links[1].mcm[2]*v4[1] + links[1].m*v4[6]) + a4[1]*links[1].inertia[1][2] + v4[1]*(-(links[1].mcm[2]*v4[4]) + v4[3]*links[1].inertia[1][1] - v4[1]*links[1].inertia[1][3]) + a4[2]*links[1].inertia[2][2] + a4[3]*links[1].inertia[2][3] + v4[2]*(links[1].mcm[1]*v4[4] + links[1].mcm[3]*v4[6] + v4[3]*links[1].inertia[1][2] - v4[1]*links[1].inertia[2][3]) + v4[3]*(-(links[1].mcm[2]*v4[6]) + v4[3]*links[1].inertia[1][3] - v4[1]*links[1].inertia[3][3]);
fnet4[6]=a4[5]*links[1].mcm[1] - a4[4]*links[1].mcm[2] + (links[1].mcm[3]*v4[2] + links[1].m*v4[4])*v4[5] + v4[4]*(links[1].mcm[3]*v4[1] - links[1].m*v4[5]) + (-(links[1].mcm[1]*v4[1]) - links[1].mcm[2]*v4[2])*v4[6] + v4[1]*(-(links[1].mcm[3]*v4[4]) - v4[2]*links[1].inertia[1][1] + v4[1]*links[1].inertia[1][2]) + a4[1]*links[1].inertia[1][3] + v4[2]*(-(links[1].mcm[3]*v4[5]) - v4[2]*links[1].inertia[1][2] + v4[1]*links[1].inertia[2][2]) + a4[2]*links[1].inertia[2][3] + v4[3]*(links[1].mcm[1]*v4[4] + links[1].mcm[2]*v4[5] - v4[2]*links[1].inertia[1][3] + v4[1]*links[1].inertia[2][3]) + a4[3]*links[1].inertia[3][3];

fnet5[1]=links[2].m*a5[4] - a5[3]*links[2].mcm[2] + a5[2]*links[2].mcm[3] - links[2].mcm[1]*Power(v5[2],2) - links[2].mcm[1]*Power(v5[3],2) + v5[1]*(links[2].mcm[2]*v5[2] + links[2].mcm[3]*v5[3]) - links[2].m*v5[3]*v5[5] + links[2].m*v5[2]*v5[6];
fnet5[2]=links[2].m*a5[5] + a5[3]*links[2].mcm[1] - a5[1]*links[2].mcm[3] - links[2].mcm[2]*Power(v5[1],2) - links[2].mcm[2]*Power(v5[3],2) + v5[2]*(links[2].mcm[1]*v5[1] + links[2].mcm[3]*v5[3]) + links[2].m*v5[3]*v5[4] - links[2].m*v5[1]*v5[6];
fnet5[3]=links[2].m*a5[6] - a5[2]*links[2].mcm[1] + a5[1]*links[2].mcm[2] - links[2].mcm[3]*Power(v5[1],2) - links[2].mcm[3]*Power(v5[2],2) + (links[2].mcm[1]*v5[1] + links[2].mcm[2]*v5[2])*v5[3] - links[2].m*v5[2]*v5[4] + links[2].m*v5[1]*v5[5];
fnet5[4]=a5[6]*links[2].mcm[2] - a5[5]*links[2].mcm[3] + (-(links[2].mcm[2]*v5[2]) - links[2].mcm[3]*v5[3])*v5[4] + (links[2].mcm[1]*v5[3] + links[2].m*v5[5])*v5[6] + v5[5]*(links[2].mcm[1]*v5[2] - links[2].m*v5[6]) + a5[1]*links[2].inertia[1][1] + a5[2]*links[2].inertia[1][2] + a5[3]*links[2].inertia[1][3] + v5[1]*(links[2].mcm[2]*v5[5] + links[2].mcm[3]*v5[6] - v5[3]*links[2].inertia[1][2] + v5[2]*links[2].inertia[1][3]) + v5[2]*(-(links[2].mcm[1]*v5[5]) - v5[3]*links[2].inertia[2][2] + v5[2]*links[2].inertia[2][3]) + v5[3]*(-(links[2].mcm[1]*v5[6]) - v5[3]*links[2].inertia[2][3] + v5[2]*links[2].inertia[3][3]);
fnet5[5]=-(a5[6]*links[2].mcm[1]) + a5[4]*links[2].mcm[3] + (-(links[2].mcm[1]*v5[1]) - links[2].mcm[3]*v5[3])*v5[5] + (links[2].mcm[2]*v5[3] - links[2].m*v5[4])*v5[6] + v5[4]*(links[2].mcm[2]*v5[1] + links[2].m*v5[6]) + a5[1]*links[2].inertia[1][2] + v5[1]*(-(links[2].mcm[2]*v5[4]) + v5[3]*links[2].inertia[1][1] - v5[1]*links[2].inertia[1][3]) + a5[2]*links[2].inertia[2][2] + a5[3]*links[2].inertia[2][3] + v5[2]*(links[2].mcm[1]*v5[4] + links[2].mcm[3]*v5[6] + v5[3]*links[2].inertia[1][2] - v5[1]*links[2].inertia[2][3]) + v5[3]*(-(links[2].mcm[2]*v5[6]) + v5[3]*links[2].inertia[1][3] - v5[1]*links[2].inertia[3][3]);
fnet5[6]=a5[5]*links[2].mcm[1] - a5[4]*links[2].mcm[2] + (links[2].mcm[3]*v5[2] + links[2].m*v5[4])*v5[5] + v5[4]*(links[2].mcm[3]*v5[1] - links[2].m*v5[5]) + (-(links[2].mcm[1]*v5[1]) - links[2].mcm[2]*v5[2])*v5[6] + v5[1]*(-(links[2].mcm[3]*v5[4]) - v5[2]*links[2].inertia[1][1] + v5[1]*links[2].inertia[1][2]) + a5[1]*links[2].inertia[1][3] + v5[2]*(-(links[2].mcm[3]*v5[5]) - v5[2]*links[2].inertia[1][2] + v5[1]*links[2].inertia[2][2]) + a5[2]*links[2].inertia[2][3] + v5[3]*(links[2].mcm[1]*v5[4] + links[2].mcm[2]*v5[5] - v5[2]*links[2].inertia[1][3] + v5[1]*links[2].inertia[2][3]) + a5[3]*links[2].inertia[3][3];

fnet6[1]=links[3].m*a6[4] - a6[3]*links[3].mcm[2] + a6[2]*links[3].mcm[3] - links[3].mcm[1]*Power(v6[2],2) - links[3].mcm[1]*Power(v6[3],2) + v6[1]*(links[3].mcm[2]*v6[2] + links[3].mcm[3]*v6[3]) - links[3].m*v6[3]*v6[5] + links[3].m*v6[2]*v6[6];
fnet6[2]=links[3].m*a6[5] + a6[3]*links[3].mcm[1] - a6[1]*links[3].mcm[3] - links[3].mcm[2]*Power(v6[1],2) - links[3].mcm[2]*Power(v6[3],2) + v6[2]*(links[3].mcm[1]*v6[1] + links[3].mcm[3]*v6[3]) + links[3].m*v6[3]*v6[4] - links[3].m*v6[1]*v6[6];
fnet6[3]=links[3].m*a6[6] - a6[2]*links[3].mcm[1] + a6[1]*links[3].mcm[2] - links[3].mcm[3]*Power(v6[1],2) - links[3].mcm[3]*Power(v6[2],2) + (links[3].mcm[1]*v6[1] + links[3].mcm[2]*v6[2])*v6[3] - links[3].m*v6[2]*v6[4] + links[3].m*v6[1]*v6[5];
fnet6[4]=a6[6]*links[3].mcm[2] - a6[5]*links[3].mcm[3] + (-(links[3].mcm[2]*v6[2]) - links[3].mcm[3]*v6[3])*v6[4] + (links[3].mcm[1]*v6[3] + links[3].m*v6[5])*v6[6] + v6[5]*(links[3].mcm[1]*v6[2] - links[3].m*v6[6]) + a6[1]*links[3].inertia[1][1] + a6[2]*links[3].inertia[1][2] + a6[3]*links[3].inertia[1][3] + v6[1]*(links[3].mcm[2]*v6[5] + links[3].mcm[3]*v6[6] - v6[3]*links[3].inertia[1][2] + v6[2]*links[3].inertia[1][3]) + v6[2]*(-(links[3].mcm[1]*v6[5]) - v6[3]*links[3].inertia[2][2] + v6[2]*links[3].inertia[2][3]) + v6[3]*(-(links[3].mcm[1]*v6[6]) - v6[3]*links[3].inertia[2][3] + v6[2]*links[3].inertia[3][3]);
fnet6[5]=-(a6[6]*links[3].mcm[1]) + a6[4]*links[3].mcm[3] + (-(links[3].mcm[1]*v6[1]) - links[3].mcm[3]*v6[3])*v6[5] + (links[3].mcm[2]*v6[3] - links[3].m*v6[4])*v6[6] + v6[4]*(links[3].mcm[2]*v6[1] + links[3].m*v6[6]) + a6[1]*links[3].inertia[1][2] + v6[1]*(-(links[3].mcm[2]*v6[4]) + v6[3]*links[3].inertia[1][1] - v6[1]*links[3].inertia[1][3]) + a6[2]*links[3].inertia[2][2] + a6[3]*links[3].inertia[2][3] + v6[2]*(links[3].mcm[1]*v6[4] + links[3].mcm[3]*v6[6] + v6[3]*links[3].inertia[1][2] - v6[1]*links[3].inertia[2][3]) + v6[3]*(-(links[3].mcm[2]*v6[6]) + v6[3]*links[3].inertia[1][3] - v6[1]*links[3].inertia[3][3]);
fnet6[6]=a6[5]*links[3].mcm[1] - a6[4]*links[3].mcm[2] + (links[3].mcm[3]*v6[2] + links[3].m*v6[4])*v6[5] + v6[4]*(links[3].mcm[3]*v6[1] - links[3].m*v6[5]) + (-(links[3].mcm[1]*v6[1]) - links[3].mcm[2]*v6[2])*v6[6] + v6[1]*(-(links[3].mcm[3]*v6[4]) - v6[2]*links[3].inertia[1][1] + v6[1]*links[3].inertia[1][2]) + a6[1]*links[3].inertia[1][3] + v6[2]*(-(links[3].mcm[3]*v6[5]) - v6[2]*links[3].inertia[1][2] + v6[1]*links[3].inertia[2][2]) + a6[2]*links[3].inertia[2][3] + v6[3]*(links[3].mcm[1]*v6[4] + links[3].mcm[2]*v6[5] - v6[2]*links[3].inertia[1][3] + v6[1]*links[3].inertia[2][3]) + a6[3]*links[3].inertia[3][3];

fnet7[1]=links[4].m*a7[4] - a7[3]*links[4].mcm[2] + a7[2]*links[4].mcm[3] - links[4].mcm[1]*Power(v7[2],2) - links[4].mcm[1]*Power(v7[3],2) + v7[1]*(links[4].mcm[2]*v7[2] + links[4].mcm[3]*v7[3]) - links[4].m*v7[3]*v7[5] + links[4].m*v7[2]*v7[6];
fnet7[2]=links[4].m*a7[5] + a7[3]*links[4].mcm[1] - a7[1]*links[4].mcm[3] - links[4].mcm[2]*Power(v7[1],2) - links[4].mcm[2]*Power(v7[3],2) + v7[2]*(links[4].mcm[1]*v7[1] + links[4].mcm[3]*v7[3]) + links[4].m*v7[3]*v7[4] - links[4].m*v7[1]*v7[6];
fnet7[3]=links[4].m*a7[6] - a7[2]*links[4].mcm[1] + a7[1]*links[4].mcm[2] - links[4].mcm[3]*Power(v7[1],2) - links[4].mcm[3]*Power(v7[2],2) + (links[4].mcm[1]*v7[1] + links[4].mcm[2]*v7[2])*v7[3] - links[4].m*v7[2]*v7[4] + links[4].m*v7[1]*v7[5];
fnet7[4]=a7[6]*links[4].mcm[2] - a7[5]*links[4].mcm[3] + (-(links[4].mcm[2]*v7[2]) - links[4].mcm[3]*v7[3])*v7[4] + (links[4].mcm[1]*v7[3] + links[4].m*v7[5])*v7[6] + v7[5]*(links[4].mcm[1]*v7[2] - links[4].m*v7[6]) + a7[1]*links[4].inertia[1][1] + a7[2]*links[4].inertia[1][2] + a7[3]*links[4].inertia[1][3] + v7[1]*(links[4].mcm[2]*v7[5] + links[4].mcm[3]*v7[6] - v7[3]*links[4].inertia[1][2] + v7[2]*links[4].inertia[1][3]) + v7[2]*(-(links[4].mcm[1]*v7[5]) - v7[3]*links[4].inertia[2][2] + v7[2]*links[4].inertia[2][3]) + v7[3]*(-(links[4].mcm[1]*v7[6]) - v7[3]*links[4].inertia[2][3] + v7[2]*links[4].inertia[3][3]);
fnet7[5]=-(a7[6]*links[4].mcm[1]) + a7[4]*links[4].mcm[3] + (-(links[4].mcm[1]*v7[1]) - links[4].mcm[3]*v7[3])*v7[5] + (links[4].mcm[2]*v7[3] - links[4].m*v7[4])*v7[6] + v7[4]*(links[4].mcm[2]*v7[1] + links[4].m*v7[6]) + a7[1]*links[4].inertia[1][2] + v7[1]*(-(links[4].mcm[2]*v7[4]) + v7[3]*links[4].inertia[1][1] - v7[1]*links[4].inertia[1][3]) + a7[2]*links[4].inertia[2][2] + a7[3]*links[4].inertia[2][3] + v7[2]*(links[4].mcm[1]*v7[4] + links[4].mcm[3]*v7[6] + v7[3]*links[4].inertia[1][2] - v7[1]*links[4].inertia[2][3]) + v7[3]*(-(links[4].mcm[2]*v7[6]) + v7[3]*links[4].inertia[1][3] - v7[1]*links[4].inertia[3][3]);
fnet7[6]=a7[5]*links[4].mcm[1] - a7[4]*links[4].mcm[2] + (links[4].mcm[3]*v7[2] + links[4].m*v7[4])*v7[5] + v7[4]*(links[4].mcm[3]*v7[1] - links[4].m*v7[5]) + (-(links[4].mcm[1]*v7[1]) - links[4].mcm[2]*v7[2])*v7[6] + v7[1]*(-(links[4].mcm[3]*v7[4]) - v7[2]*links[4].inertia[1][1] + v7[1]*links[4].inertia[1][2]) + a7[1]*links[4].inertia[1][3] + v7[2]*(-(links[4].mcm[3]*v7[5]) - v7[2]*links[4].inertia[1][2] + v7[1]*links[4].inertia[2][2]) + a7[2]*links[4].inertia[2][3] + v7[3]*(links[4].mcm[1]*v7[4] + links[4].mcm[2]*v7[5] - v7[2]*links[4].inertia[1][3] + v7[1]*links[4].inertia[2][3]) + a7[3]*links[4].inertia[3][3];

fnet8[1]=links[5].m*a8[4] - a8[3]*links[5].mcm[2] + a8[2]*links[5].mcm[3] - links[5].mcm[1]*Power(v8[2],2) - links[5].mcm[1]*Power(v8[3],2) + v8[1]*(links[5].mcm[2]*v8[2] + links[5].mcm[3]*v8[3]) - links[5].m*v8[3]*v8[5] + links[5].m*v8[2]*v8[6];
fnet8[2]=links[5].m*a8[5] + a8[3]*links[5].mcm[1] - a8[1]*links[5].mcm[3] - links[5].mcm[2]*Power(v8[1],2) - links[5].mcm[2]*Power(v8[3],2) + v8[2]*(links[5].mcm[1]*v8[1] + links[5].mcm[3]*v8[3]) + links[5].m*v8[3]*v8[4] - links[5].m*v8[1]*v8[6];
fnet8[3]=links[5].m*a8[6] - a8[2]*links[5].mcm[1] + a8[1]*links[5].mcm[2] - links[5].mcm[3]*Power(v8[1],2) - links[5].mcm[3]*Power(v8[2],2) + (links[5].mcm[1]*v8[1] + links[5].mcm[2]*v8[2])*v8[3] - links[5].m*v8[2]*v8[4] + links[5].m*v8[1]*v8[5];
fnet8[4]=a8[6]*links[5].mcm[2] - a8[5]*links[5].mcm[3] + (-(links[5].mcm[2]*v8[2]) - links[5].mcm[3]*v8[3])*v8[4] + (links[5].mcm[1]*v8[3] + links[5].m*v8[5])*v8[6] + v8[5]*(links[5].mcm[1]*v8[2] - links[5].m*v8[6]) + a8[1]*links[5].inertia[1][1] + a8[2]*links[5].inertia[1][2] + a8[3]*links[5].inertia[1][3] + v8[1]*(links[5].mcm[2]*v8[5] + links[5].mcm[3]*v8[6] - v8[3]*links[5].inertia[1][2] + v8[2]*links[5].inertia[1][3]) + v8[2]*(-(links[5].mcm[1]*v8[5]) - v8[3]*links[5].inertia[2][2] + v8[2]*links[5].inertia[2][3]) + v8[3]*(-(links[5].mcm[1]*v8[6]) - v8[3]*links[5].inertia[2][3] + v8[2]*links[5].inertia[3][3]);
fnet8[5]=-(a8[6]*links[5].mcm[1]) + a8[4]*links[5].mcm[3] + (-(links[5].mcm[1]*v8[1]) - links[5].mcm[3]*v8[3])*v8[5] + (links[5].mcm[2]*v8[3] - links[5].m*v8[4])*v8[6] + v8[4]*(links[5].mcm[2]*v8[1] + links[5].m*v8[6]) + a8[1]*links[5].inertia[1][2] + v8[1]*(-(links[5].mcm[2]*v8[4]) + v8[3]*links[5].inertia[1][1] - v8[1]*links[5].inertia[1][3]) + a8[2]*links[5].inertia[2][2] + a8[3]*links[5].inertia[2][3] + v8[2]*(links[5].mcm[1]*v8[4] + links[5].mcm[3]*v8[6] + v8[3]*links[5].inertia[1][2] - v8[1]*links[5].inertia[2][3]) + v8[3]*(-(links[5].mcm[2]*v8[6]) + v8[3]*links[5].inertia[1][3] - v8[1]*links[5].inertia[3][3]);
fnet8[6]=a8[5]*links[5].mcm[1] - a8[4]*links[5].mcm[2] + (links[5].mcm[3]*v8[2] + links[5].m*v8[4])*v8[5] + v8[4]*(links[5].mcm[3]*v8[1] - links[5].m*v8[5]) + (-(links[5].mcm[1]*v8[1]) - links[5].mcm[2]*v8[2])*v8[6] + v8[1]*(-(links[5].mcm[3]*v8[4]) - v8[2]*links[5].inertia[1][1] + v8[1]*links[5].inertia[1][2]) + a8[1]*links[5].inertia[1][3] + v8[2]*(-(links[5].mcm[3]*v8[5]) - v8[2]*links[5].inertia[1][2] + v8[1]*links[5].inertia[2][2]) + a8[2]*links[5].inertia[2][3] + v8[3]*(links[5].mcm[1]*v8[4] + links[5].mcm[2]*v8[5] - v8[2]*links[5].inertia[1][3] + v8[1]*links[5].inertia[2][3]) + a8[3]*links[5].inertia[3][3];

fnet9[1]=links[6].m*a9[4] - a9[3]*links[6].mcm[2] + a9[2]*links[6].mcm[3] - links[6].mcm[1]*Power(v9[2],2) - links[6].mcm[1]*Power(v9[3],2) + v9[1]*(links[6].mcm[2]*v9[2] + links[6].mcm[3]*v9[3]) - links[6].m*v9[3]*v9[5] + links[6].m*v9[2]*v9[6];
fnet9[2]=links[6].m*a9[5] + a9[3]*links[6].mcm[1] - a9[1]*links[6].mcm[3] - links[6].mcm[2]*Power(v9[1],2) - links[6].mcm[2]*Power(v9[3],2) + v9[2]*(links[6].mcm[1]*v9[1] + links[6].mcm[3]*v9[3]) + links[6].m*v9[3]*v9[4] - links[6].m*v9[1]*v9[6];
fnet9[3]=links[6].m*a9[6] - a9[2]*links[6].mcm[1] + a9[1]*links[6].mcm[2] - links[6].mcm[3]*Power(v9[1],2) - links[6].mcm[3]*Power(v9[2],2) + (links[6].mcm[1]*v9[1] + links[6].mcm[2]*v9[2])*v9[3] - links[6].m*v9[2]*v9[4] + links[6].m*v9[1]*v9[5];
fnet9[4]=a9[6]*links[6].mcm[2] - a9[5]*links[6].mcm[3] + (-(links[6].mcm[2]*v9[2]) - links[6].mcm[3]*v9[3])*v9[4] + (links[6].mcm[1]*v9[3] + links[6].m*v9[5])*v9[6] + v9[5]*(links[6].mcm[1]*v9[2] - links[6].m*v9[6]) + a9[1]*links[6].inertia[1][1] + a9[2]*links[6].inertia[1][2] + a9[3]*links[6].inertia[1][3] + v9[1]*(links[6].mcm[2]*v9[5] + links[6].mcm[3]*v9[6] - v9[3]*links[6].inertia[1][2] + v9[2]*links[6].inertia[1][3]) + v9[2]*(-(links[6].mcm[1]*v9[5]) - v9[3]*links[6].inertia[2][2] + v9[2]*links[6].inertia[2][3]) + v9[3]*(-(links[6].mcm[1]*v9[6]) - v9[3]*links[6].inertia[2][3] + v9[2]*links[6].inertia[3][3]);
fnet9[5]=-(a9[6]*links[6].mcm[1]) + a9[4]*links[6].mcm[3] + (-(links[6].mcm[1]*v9[1]) - links[6].mcm[3]*v9[3])*v9[5] + (links[6].mcm[2]*v9[3] - links[6].m*v9[4])*v9[6] + v9[4]*(links[6].mcm[2]*v9[1] + links[6].m*v9[6]) + a9[1]*links[6].inertia[1][2] + v9[1]*(-(links[6].mcm[2]*v9[4]) + v9[3]*links[6].inertia[1][1] - v9[1]*links[6].inertia[1][3]) + a9[2]*links[6].inertia[2][2] + a9[3]*links[6].inertia[2][3] + v9[2]*(links[6].mcm[1]*v9[4] + links[6].mcm[3]*v9[6] + v9[3]*links[6].inertia[1][2] - v9[1]*links[6].inertia[2][3]) + v9[3]*(-(links[6].mcm[2]*v9[6]) + v9[3]*links[6].inertia[1][3] - v9[1]*links[6].inertia[3][3]);
fnet9[6]=a9[5]*links[6].mcm[1] - a9[4]*links[6].mcm[2] + (links[6].mcm[3]*v9[2] + links[6].m*v9[4])*v9[5] + v9[4]*(links[6].mcm[3]*v9[1] - links[6].m*v9[5]) + (-(links[6].mcm[1]*v9[1]) - links[6].mcm[2]*v9[2])*v9[6] + v9[1]*(-(links[6].mcm[3]*v9[4]) - v9[2]*links[6].inertia[1][1] + v9[1]*links[6].inertia[1][2]) + a9[1]*links[6].inertia[1][3] + v9[2]*(-(links[6].mcm[3]*v9[5]) - v9[2]*links[6].inertia[1][2] + v9[1]*links[6].inertia[2][2]) + a9[2]*links[6].inertia[2][3] + v9[3]*(links[6].mcm[1]*v9[4] + links[6].mcm[2]*v9[5] - v9[2]*links[6].inertia[1][3] + v9[1]*links[6].inertia[2][3]) + a9[3]*links[6].inertia[3][3];

fnet10[1]=links[7].m*a10[4] - a10[3]*links[7].mcm[2] + a10[2]*links[7].mcm[3] - links[7].mcm[1]*Power(v10[2],2) - links[7].mcm[1]*Power(v10[3],2) + v10[1]*(links[7].mcm[2]*v10[2] + links[7].mcm[3]*v10[3]) - links[7].m*v10[3]*v10[5] + links[7].m*v10[2]*v10[6];
fnet10[2]=links[7].m*a10[5] + a10[3]*links[7].mcm[1] - a10[1]*links[7].mcm[3] - links[7].mcm[2]*Power(v10[1],2) - links[7].mcm[2]*Power(v10[3],2) + v10[2]*(links[7].mcm[1]*v10[1] + links[7].mcm[3]*v10[3]) + links[7].m*v10[3]*v10[4] - links[7].m*v10[1]*v10[6];
fnet10[3]=links[7].m*a10[6] - a10[2]*links[7].mcm[1] + a10[1]*links[7].mcm[2] - links[7].mcm[3]*Power(v10[1],2) - links[7].mcm[3]*Power(v10[2],2) + (links[7].mcm[1]*v10[1] + links[7].mcm[2]*v10[2])*v10[3] - links[7].m*v10[2]*v10[4] + links[7].m*v10[1]*v10[5];
fnet10[4]=a10[6]*links[7].mcm[2] - a10[5]*links[7].mcm[3] + (-(links[7].mcm[2]*v10[2]) - links[7].mcm[3]*v10[3])*v10[4] + (links[7].mcm[1]*v10[3] + links[7].m*v10[5])*v10[6] + v10[5]*(links[7].mcm[1]*v10[2] - links[7].m*v10[6]) + a10[1]*links[7].inertia[1][1] + a10[2]*links[7].inertia[1][2] + a10[3]*links[7].inertia[1][3] + v10[1]*(links[7].mcm[2]*v10[5] + links[7].mcm[3]*v10[6] - v10[3]*links[7].inertia[1][2] + v10[2]*links[7].inertia[1][3]) + v10[2]*(-(links[7].mcm[1]*v10[5]) - v10[3]*links[7].inertia[2][2] + v10[2]*links[7].inertia[2][3]) + v10[3]*(-(links[7].mcm[1]*v10[6]) - v10[3]*links[7].inertia[2][3] + v10[2]*links[7].inertia[3][3]);
fnet10[5]=-(a10[6]*links[7].mcm[1]) + a10[4]*links[7].mcm[3] + (-(links[7].mcm[1]*v10[1]) - links[7].mcm[3]*v10[3])*v10[5] + (links[7].mcm[2]*v10[3] - links[7].m*v10[4])*v10[6] + v10[4]*(links[7].mcm[2]*v10[1] + links[7].m*v10[6]) + a10[1]*links[7].inertia[1][2] + v10[1]*(-(links[7].mcm[2]*v10[4]) + v10[3]*links[7].inertia[1][1] - v10[1]*links[7].inertia[1][3]) + a10[2]*links[7].inertia[2][2] + a10[3]*links[7].inertia[2][3] + v10[2]*(links[7].mcm[1]*v10[4] + links[7].mcm[3]*v10[6] + v10[3]*links[7].inertia[1][2] - v10[1]*links[7].inertia[2][3]) + v10[3]*(-(links[7].mcm[2]*v10[6]) + v10[3]*links[7].inertia[1][3] - v10[1]*links[7].inertia[3][3]);
fnet10[6]=a10[5]*links[7].mcm[1] - a10[4]*links[7].mcm[2] + (links[7].mcm[3]*v10[2] + links[7].m*v10[4])*v10[5] + v10[4]*(links[7].mcm[3]*v10[1] - links[7].m*v10[5]) + (-(links[7].mcm[1]*v10[1]) - links[7].mcm[2]*v10[2])*v10[6] + v10[1]*(-(links[7].mcm[3]*v10[4]) - v10[2]*links[7].inertia[1][1] + v10[1]*links[7].inertia[1][2]) + a10[1]*links[7].inertia[1][3] + v10[2]*(-(links[7].mcm[3]*v10[5]) - v10[2]*links[7].inertia[1][2] + v10[1]*links[7].inertia[2][2]) + a10[2]*links[7].inertia[2][3] + v10[3]*(links[7].mcm[1]*v10[4] + links[7].mcm[2]*v10[5] - v10[2]*links[7].inertia[1][3] + v10[1]*links[7].inertia[2][3]) + a10[3]*links[7].inertia[3][3];

fnet11[1]=eff[2].m*a11[4] - a11[3]*eff[2].mcm[2] + a11[2]*eff[2].mcm[3] - eff[2].mcm[1]*Power(v11[2],2) - eff[2].mcm[1]*Power(v11[3],2) + v11[1]*(eff[2].mcm[2]*v11[2] + eff[2].mcm[3]*v11[3]) - eff[2].m*v11[3]*v11[5] + eff[2].m*v11[2]*v11[6];
fnet11[2]=eff[2].m*a11[5] + a11[3]*eff[2].mcm[1] - a11[1]*eff[2].mcm[3] - eff[2].mcm[2]*Power(v11[1],2) - eff[2].mcm[2]*Power(v11[3],2) + v11[2]*(eff[2].mcm[1]*v11[1] + eff[2].mcm[3]*v11[3]) + eff[2].m*v11[3]*v11[4] - eff[2].m*v11[1]*v11[6];
fnet11[3]=eff[2].m*a11[6] - a11[2]*eff[2].mcm[1] + a11[1]*eff[2].mcm[2] - eff[2].mcm[3]*Power(v11[1],2) - eff[2].mcm[3]*Power(v11[2],2) + (eff[2].mcm[1]*v11[1] + eff[2].mcm[2]*v11[2])*v11[3] - eff[2].m*v11[2]*v11[4] + eff[2].m*v11[1]*v11[5];
fnet11[4]=a11[6]*eff[2].mcm[2] - a11[5]*eff[2].mcm[3] + (-(eff[2].mcm[2]*v11[2]) - eff[2].mcm[3]*v11[3])*v11[4] - eff[2].mcm[1]*v11[2]*v11[5] - eff[2].mcm[1]*v11[3]*v11[6] + (eff[2].mcm[1]*v11[3] + eff[2].m*v11[5])*v11[6] + v11[5]*(eff[2].mcm[1]*v11[2] - eff[2].m*v11[6]) + v11[1]*(eff[2].mcm[2]*v11[5] + eff[2].mcm[3]*v11[6]);
fnet11[5]=-(a11[6]*eff[2].mcm[1]) + a11[4]*eff[2].mcm[3] - eff[2].mcm[2]*v11[1]*v11[4] + (-(eff[2].mcm[1]*v11[1]) - eff[2].mcm[3]*v11[3])*v11[5] - eff[2].mcm[2]*v11[3]*v11[6] + (eff[2].mcm[2]*v11[3] - eff[2].m*v11[4])*v11[6] + v11[4]*(eff[2].mcm[2]*v11[1] + eff[2].m*v11[6]) + v11[2]*(eff[2].mcm[1]*v11[4] + eff[2].mcm[3]*v11[6]);
fnet11[6]=a11[5]*eff[2].mcm[1] - a11[4]*eff[2].mcm[2] - eff[2].mcm[3]*v11[1]*v11[4] - eff[2].mcm[3]*v11[2]*v11[5] + (eff[2].mcm[3]*v11[2] + eff[2].m*v11[4])*v11[5] + v11[4]*(eff[2].mcm[3]*v11[1] - eff[2].m*v11[5]) + v11[3]*(eff[2].mcm[1]*v11[4] + eff[2].mcm[2]*v11[5]) + (-(eff[2].mcm[1]*v11[1]) - eff[2].mcm[2]*v11[2])*v11[6];

fnet12[1]=links[39].m*a12[4] - a12[3]*links[39].mcm[2] + a12[2]*links[39].mcm[3] - links[39].mcm[1]*Power(v12[2],2) - links[39].mcm[1]*Power(v12[3],2) + v12[1]*(links[39].mcm[2]*v12[2] + links[39].mcm[3]*v12[3]) - links[39].m*v12[3]*v12[5] + links[39].m*v12[2]*v12[6];
fnet12[2]=links[39].m*a12[5] + a12[3]*links[39].mcm[1] - a12[1]*links[39].mcm[3] - links[39].mcm[2]*Power(v12[1],2) - links[39].mcm[2]*Power(v12[3],2) + v12[2]*(links[39].mcm[1]*v12[1] + links[39].mcm[3]*v12[3]) + links[39].m*v12[3]*v12[4] - links[39].m*v12[1]*v12[6];
fnet12[3]=links[39].m*a12[6] - a12[2]*links[39].mcm[1] + a12[1]*links[39].mcm[2] - links[39].mcm[3]*Power(v12[1],2) - links[39].mcm[3]*Power(v12[2],2) + (links[39].mcm[1]*v12[1] + links[39].mcm[2]*v12[2])*v12[3] - links[39].m*v12[2]*v12[4] + links[39].m*v12[1]*v12[5];
fnet12[4]=a12[6]*links[39].mcm[2] - a12[5]*links[39].mcm[3] + (-(links[39].mcm[2]*v12[2]) - links[39].mcm[3]*v12[3])*v12[4] + (links[39].mcm[1]*v12[3] + links[39].m*v12[5])*v12[6] + v12[5]*(links[39].mcm[1]*v12[2] - links[39].m*v12[6]) + a12[1]*links[39].inertia[1][1] + a12[2]*links[39].inertia[1][2] + a12[3]*links[39].inertia[1][3] + v12[1]*(links[39].mcm[2]*v12[5] + links[39].mcm[3]*v12[6] - v12[3]*links[39].inertia[1][2] + v12[2]*links[39].inertia[1][3]) + v12[2]*(-(links[39].mcm[1]*v12[5]) - v12[3]*links[39].inertia[2][2] + v12[2]*links[39].inertia[2][3]) + v12[3]*(-(links[39].mcm[1]*v12[6]) - v12[3]*links[39].inertia[2][3] + v12[2]*links[39].inertia[3][3]);
fnet12[5]=-(a12[6]*links[39].mcm[1]) + a12[4]*links[39].mcm[3] + (-(links[39].mcm[1]*v12[1]) - links[39].mcm[3]*v12[3])*v12[5] + (links[39].mcm[2]*v12[3] - links[39].m*v12[4])*v12[6] + v12[4]*(links[39].mcm[2]*v12[1] + links[39].m*v12[6]) + a12[1]*links[39].inertia[1][2] + v12[1]*(-(links[39].mcm[2]*v12[4]) + v12[3]*links[39].inertia[1][1] - v12[1]*links[39].inertia[1][3]) + a12[2]*links[39].inertia[2][2] + a12[3]*links[39].inertia[2][3] + v12[2]*(links[39].mcm[1]*v12[4] + links[39].mcm[3]*v12[6] + v12[3]*links[39].inertia[1][2] - v12[1]*links[39].inertia[2][3]) + v12[3]*(-(links[39].mcm[2]*v12[6]) + v12[3]*links[39].inertia[1][3] - v12[1]*links[39].inertia[3][3]);
fnet12[6]=a12[5]*links[39].mcm[1] - a12[4]*links[39].mcm[2] + (links[39].mcm[3]*v12[2] + links[39].m*v12[4])*v12[5] + v12[4]*(links[39].mcm[3]*v12[1] - links[39].m*v12[5]) + (-(links[39].mcm[1]*v12[1]) - links[39].mcm[2]*v12[2])*v12[6] + v12[1]*(-(links[39].mcm[3]*v12[4]) - v12[2]*links[39].inertia[1][1] + v12[1]*links[39].inertia[1][2]) + a12[1]*links[39].inertia[1][3] + v12[2]*(-(links[39].mcm[3]*v12[5]) - v12[2]*links[39].inertia[1][2] + v12[1]*links[39].inertia[2][2]) + a12[2]*links[39].inertia[2][3] + v12[3]*(links[39].mcm[1]*v12[4] + links[39].mcm[2]*v12[5] - v12[2]*links[39].inertia[1][3] + v12[1]*links[39].inertia[2][3]) + a12[3]*links[39].inertia[3][3];

fnet13[1]=links[40].m*a13[4] - a13[3]*links[40].mcm[2] + a13[2]*links[40].mcm[3] - links[40].mcm[1]*Power(v13[2],2) - links[40].mcm[1]*Power(v13[3],2) + v13[1]*(links[40].mcm[2]*v13[2] + links[40].mcm[3]*v13[3]) - links[40].m*v13[3]*v13[5] + links[40].m*v13[2]*v13[6];
fnet13[2]=links[40].m*a13[5] + a13[3]*links[40].mcm[1] - a13[1]*links[40].mcm[3] - links[40].mcm[2]*Power(v13[1],2) - links[40].mcm[2]*Power(v13[3],2) + v13[2]*(links[40].mcm[1]*v13[1] + links[40].mcm[3]*v13[3]) + links[40].m*v13[3]*v13[4] - links[40].m*v13[1]*v13[6];
fnet13[3]=links[40].m*a13[6] - a13[2]*links[40].mcm[1] + a13[1]*links[40].mcm[2] - links[40].mcm[3]*Power(v13[1],2) - links[40].mcm[3]*Power(v13[2],2) + (links[40].mcm[1]*v13[1] + links[40].mcm[2]*v13[2])*v13[3] - links[40].m*v13[2]*v13[4] + links[40].m*v13[1]*v13[5];
fnet13[4]=a13[6]*links[40].mcm[2] - a13[5]*links[40].mcm[3] + (-(links[40].mcm[2]*v13[2]) - links[40].mcm[3]*v13[3])*v13[4] + (links[40].mcm[1]*v13[3] + links[40].m*v13[5])*v13[6] + v13[5]*(links[40].mcm[1]*v13[2] - links[40].m*v13[6]) + a13[1]*links[40].inertia[1][1] + a13[2]*links[40].inertia[1][2] + a13[3]*links[40].inertia[1][3] + v13[1]*(links[40].mcm[2]*v13[5] + links[40].mcm[3]*v13[6] - v13[3]*links[40].inertia[1][2] + v13[2]*links[40].inertia[1][3]) + v13[2]*(-(links[40].mcm[1]*v13[5]) - v13[3]*links[40].inertia[2][2] + v13[2]*links[40].inertia[2][3]) + v13[3]*(-(links[40].mcm[1]*v13[6]) - v13[3]*links[40].inertia[2][3] + v13[2]*links[40].inertia[3][3]);
fnet13[5]=-(a13[6]*links[40].mcm[1]) + a13[4]*links[40].mcm[3] + (-(links[40].mcm[1]*v13[1]) - links[40].mcm[3]*v13[3])*v13[5] + (links[40].mcm[2]*v13[3] - links[40].m*v13[4])*v13[6] + v13[4]*(links[40].mcm[2]*v13[1] + links[40].m*v13[6]) + a13[1]*links[40].inertia[1][2] + v13[1]*(-(links[40].mcm[2]*v13[4]) + v13[3]*links[40].inertia[1][1] - v13[1]*links[40].inertia[1][3]) + a13[2]*links[40].inertia[2][2] + a13[3]*links[40].inertia[2][3] + v13[2]*(links[40].mcm[1]*v13[4] + links[40].mcm[3]*v13[6] + v13[3]*links[40].inertia[1][2] - v13[1]*links[40].inertia[2][3]) + v13[3]*(-(links[40].mcm[2]*v13[6]) + v13[3]*links[40].inertia[1][3] - v13[1]*links[40].inertia[3][3]);
fnet13[6]=a13[5]*links[40].mcm[1] - a13[4]*links[40].mcm[2] + (links[40].mcm[3]*v13[2] + links[40].m*v13[4])*v13[5] + v13[4]*(links[40].mcm[3]*v13[1] - links[40].m*v13[5]) + (-(links[40].mcm[1]*v13[1]) - links[40].mcm[2]*v13[2])*v13[6] + v13[1]*(-(links[40].mcm[3]*v13[4]) - v13[2]*links[40].inertia[1][1] + v13[1]*links[40].inertia[1][2]) + a13[1]*links[40].inertia[1][3] + v13[2]*(-(links[40].mcm[3]*v13[5]) - v13[2]*links[40].inertia[1][2] + v13[1]*links[40].inertia[2][2]) + a13[2]*links[40].inertia[2][3] + v13[3]*(links[40].mcm[1]*v13[4] + links[40].mcm[2]*v13[5] - v13[2]*links[40].inertia[1][3] + v13[1]*links[40].inertia[2][3]) + a13[3]*links[40].inertia[3][3];

fnet16[1]=links[41].m*a16[4] - a16[3]*links[41].mcm[2] + a16[2]*links[41].mcm[3] - links[41].mcm[1]*Power(v16[2],2) - links[41].mcm[1]*Power(v16[3],2) + v16[1]*(links[41].mcm[2]*v16[2] + links[41].mcm[3]*v16[3]) - links[41].m*v16[3]*v16[5] + links[41].m*v16[2]*v16[6];
fnet16[2]=links[41].m*a16[5] + a16[3]*links[41].mcm[1] - a16[1]*links[41].mcm[3] - links[41].mcm[2]*Power(v16[1],2) - links[41].mcm[2]*Power(v16[3],2) + v16[2]*(links[41].mcm[1]*v16[1] + links[41].mcm[3]*v16[3]) + links[41].m*v16[3]*v16[4] - links[41].m*v16[1]*v16[6];
fnet16[3]=links[41].m*a16[6] - a16[2]*links[41].mcm[1] + a16[1]*links[41].mcm[2] - links[41].mcm[3]*Power(v16[1],2) - links[41].mcm[3]*Power(v16[2],2) + (links[41].mcm[1]*v16[1] + links[41].mcm[2]*v16[2])*v16[3] - links[41].m*v16[2]*v16[4] + links[41].m*v16[1]*v16[5];
fnet16[4]=a16[6]*links[41].mcm[2] - a16[5]*links[41].mcm[3] + (-(links[41].mcm[2]*v16[2]) - links[41].mcm[3]*v16[3])*v16[4] + (links[41].mcm[1]*v16[3] + links[41].m*v16[5])*v16[6] + v16[5]*(links[41].mcm[1]*v16[2] - links[41].m*v16[6]) + a16[1]*links[41].inertia[1][1] + a16[2]*links[41].inertia[1][2] + a16[3]*links[41].inertia[1][3] + v16[1]*(links[41].mcm[2]*v16[5] + links[41].mcm[3]*v16[6] - v16[3]*links[41].inertia[1][2] + v16[2]*links[41].inertia[1][3]) + v16[2]*(-(links[41].mcm[1]*v16[5]) - v16[3]*links[41].inertia[2][2] + v16[2]*links[41].inertia[2][3]) + v16[3]*(-(links[41].mcm[1]*v16[6]) - v16[3]*links[41].inertia[2][3] + v16[2]*links[41].inertia[3][3]);
fnet16[5]=-(a16[6]*links[41].mcm[1]) + a16[4]*links[41].mcm[3] + (-(links[41].mcm[1]*v16[1]) - links[41].mcm[3]*v16[3])*v16[5] + (links[41].mcm[2]*v16[3] - links[41].m*v16[4])*v16[6] + v16[4]*(links[41].mcm[2]*v16[1] + links[41].m*v16[6]) + a16[1]*links[41].inertia[1][2] + v16[1]*(-(links[41].mcm[2]*v16[4]) + v16[3]*links[41].inertia[1][1] - v16[1]*links[41].inertia[1][3]) + a16[2]*links[41].inertia[2][2] + a16[3]*links[41].inertia[2][3] + v16[2]*(links[41].mcm[1]*v16[4] + links[41].mcm[3]*v16[6] + v16[3]*links[41].inertia[1][2] - v16[1]*links[41].inertia[2][3]) + v16[3]*(-(links[41].mcm[2]*v16[6]) + v16[3]*links[41].inertia[1][3] - v16[1]*links[41].inertia[3][3]);
fnet16[6]=a16[5]*links[41].mcm[1] - a16[4]*links[41].mcm[2] + (links[41].mcm[3]*v16[2] + links[41].m*v16[4])*v16[5] + v16[4]*(links[41].mcm[3]*v16[1] - links[41].m*v16[5]) + (-(links[41].mcm[1]*v16[1]) - links[41].mcm[2]*v16[2])*v16[6] + v16[1]*(-(links[41].mcm[3]*v16[4]) - v16[2]*links[41].inertia[1][1] + v16[1]*links[41].inertia[1][2]) + a16[1]*links[41].inertia[1][3] + v16[2]*(-(links[41].mcm[3]*v16[5]) - v16[2]*links[41].inertia[1][2] + v16[1]*links[41].inertia[2][2]) + a16[2]*links[41].inertia[2][3] + v16[3]*(links[41].mcm[1]*v16[4] + links[41].mcm[2]*v16[5] - v16[2]*links[41].inertia[1][3] + v16[1]*links[41].inertia[2][3]) + a16[3]*links[41].inertia[3][3];

fnet20[1]=links[42].m*a20[4] - a20[3]*links[42].mcm[2] + a20[2]*links[42].mcm[3] - links[42].mcm[1]*Power(v20[2],2) - links[42].mcm[1]*Power(v20[3],2) + v20[1]*(links[42].mcm[2]*v20[2] + links[42].mcm[3]*v20[3]) - links[42].m*v20[3]*v20[5] + links[42].m*v20[2]*v20[6];
fnet20[2]=links[42].m*a20[5] + a20[3]*links[42].mcm[1] - a20[1]*links[42].mcm[3] - links[42].mcm[2]*Power(v20[1],2) - links[42].mcm[2]*Power(v20[3],2) + v20[2]*(links[42].mcm[1]*v20[1] + links[42].mcm[3]*v20[3]) + links[42].m*v20[3]*v20[4] - links[42].m*v20[1]*v20[6];
fnet20[3]=links[42].m*a20[6] - a20[2]*links[42].mcm[1] + a20[1]*links[42].mcm[2] - links[42].mcm[3]*Power(v20[1],2) - links[42].mcm[3]*Power(v20[2],2) + (links[42].mcm[1]*v20[1] + links[42].mcm[2]*v20[2])*v20[3] - links[42].m*v20[2]*v20[4] + links[42].m*v20[1]*v20[5];
fnet20[4]=a20[6]*links[42].mcm[2] - a20[5]*links[42].mcm[3] + (-(links[42].mcm[2]*v20[2]) - links[42].mcm[3]*v20[3])*v20[4] + (links[42].mcm[1]*v20[3] + links[42].m*v20[5])*v20[6] + v20[5]*(links[42].mcm[1]*v20[2] - links[42].m*v20[6]) + a20[1]*links[42].inertia[1][1] + a20[2]*links[42].inertia[1][2] + a20[3]*links[42].inertia[1][3] + v20[1]*(links[42].mcm[2]*v20[5] + links[42].mcm[3]*v20[6] - v20[3]*links[42].inertia[1][2] + v20[2]*links[42].inertia[1][3]) + v20[2]*(-(links[42].mcm[1]*v20[5]) - v20[3]*links[42].inertia[2][2] + v20[2]*links[42].inertia[2][3]) + v20[3]*(-(links[42].mcm[1]*v20[6]) - v20[3]*links[42].inertia[2][3] + v20[2]*links[42].inertia[3][3]);
fnet20[5]=-(a20[6]*links[42].mcm[1]) + a20[4]*links[42].mcm[3] + (-(links[42].mcm[1]*v20[1]) - links[42].mcm[3]*v20[3])*v20[5] + (links[42].mcm[2]*v20[3] - links[42].m*v20[4])*v20[6] + v20[4]*(links[42].mcm[2]*v20[1] + links[42].m*v20[6]) + a20[1]*links[42].inertia[1][2] + v20[1]*(-(links[42].mcm[2]*v20[4]) + v20[3]*links[42].inertia[1][1] - v20[1]*links[42].inertia[1][3]) + a20[2]*links[42].inertia[2][2] + a20[3]*links[42].inertia[2][3] + v20[2]*(links[42].mcm[1]*v20[4] + links[42].mcm[3]*v20[6] + v20[3]*links[42].inertia[1][2] - v20[1]*links[42].inertia[2][3]) + v20[3]*(-(links[42].mcm[2]*v20[6]) + v20[3]*links[42].inertia[1][3] - v20[1]*links[42].inertia[3][3]);
fnet20[6]=a20[5]*links[42].mcm[1] - a20[4]*links[42].mcm[2] + (links[42].mcm[3]*v20[2] + links[42].m*v20[4])*v20[5] + v20[4]*(links[42].mcm[3]*v20[1] - links[42].m*v20[5]) + (-(links[42].mcm[1]*v20[1]) - links[42].mcm[2]*v20[2])*v20[6] + v20[1]*(-(links[42].mcm[3]*v20[4]) - v20[2]*links[42].inertia[1][1] + v20[1]*links[42].inertia[1][2]) + a20[1]*links[42].inertia[1][3] + v20[2]*(-(links[42].mcm[3]*v20[5]) - v20[2]*links[42].inertia[1][2] + v20[1]*links[42].inertia[2][2]) + a20[2]*links[42].inertia[2][3] + v20[3]*(links[42].mcm[1]*v20[4] + links[42].mcm[2]*v20[5] - v20[2]*links[42].inertia[1][3] + v20[1]*links[42].inertia[2][3]) + a20[3]*links[42].inertia[3][3];

fnet24[1]=links[43].m*a24[4] - a24[3]*links[43].mcm[2] + a24[2]*links[43].mcm[3] - links[43].mcm[1]*Power(v24[2],2) - links[43].mcm[1]*Power(v24[3],2) + v24[1]*(links[43].mcm[2]*v24[2] + links[43].mcm[3]*v24[3]) - links[43].m*v24[3]*v24[5] + links[43].m*v24[2]*v24[6];
fnet24[2]=links[43].m*a24[5] + a24[3]*links[43].mcm[1] - a24[1]*links[43].mcm[3] - links[43].mcm[2]*Power(v24[1],2) - links[43].mcm[2]*Power(v24[3],2) + v24[2]*(links[43].mcm[1]*v24[1] + links[43].mcm[3]*v24[3]) + links[43].m*v24[3]*v24[4] - links[43].m*v24[1]*v24[6];
fnet24[3]=links[43].m*a24[6] - a24[2]*links[43].mcm[1] + a24[1]*links[43].mcm[2] - links[43].mcm[3]*Power(v24[1],2) - links[43].mcm[3]*Power(v24[2],2) + (links[43].mcm[1]*v24[1] + links[43].mcm[2]*v24[2])*v24[3] - links[43].m*v24[2]*v24[4] + links[43].m*v24[1]*v24[5];
fnet24[4]=a24[6]*links[43].mcm[2] - a24[5]*links[43].mcm[3] + (-(links[43].mcm[2]*v24[2]) - links[43].mcm[3]*v24[3])*v24[4] + (links[43].mcm[1]*v24[3] + links[43].m*v24[5])*v24[6] + v24[5]*(links[43].mcm[1]*v24[2] - links[43].m*v24[6]) + a24[1]*links[43].inertia[1][1] + a24[2]*links[43].inertia[1][2] + a24[3]*links[43].inertia[1][3] + v24[1]*(links[43].mcm[2]*v24[5] + links[43].mcm[3]*v24[6] - v24[3]*links[43].inertia[1][2] + v24[2]*links[43].inertia[1][3]) + v24[2]*(-(links[43].mcm[1]*v24[5]) - v24[3]*links[43].inertia[2][2] + v24[2]*links[43].inertia[2][3]) + v24[3]*(-(links[43].mcm[1]*v24[6]) - v24[3]*links[43].inertia[2][3] + v24[2]*links[43].inertia[3][3]);
fnet24[5]=-(a24[6]*links[43].mcm[1]) + a24[4]*links[43].mcm[3] + (-(links[43].mcm[1]*v24[1]) - links[43].mcm[3]*v24[3])*v24[5] + (links[43].mcm[2]*v24[3] - links[43].m*v24[4])*v24[6] + v24[4]*(links[43].mcm[2]*v24[1] + links[43].m*v24[6]) + a24[1]*links[43].inertia[1][2] + v24[1]*(-(links[43].mcm[2]*v24[4]) + v24[3]*links[43].inertia[1][1] - v24[1]*links[43].inertia[1][3]) + a24[2]*links[43].inertia[2][2] + a24[3]*links[43].inertia[2][3] + v24[2]*(links[43].mcm[1]*v24[4] + links[43].mcm[3]*v24[6] + v24[3]*links[43].inertia[1][2] - v24[1]*links[43].inertia[2][3]) + v24[3]*(-(links[43].mcm[2]*v24[6]) + v24[3]*links[43].inertia[1][3] - v24[1]*links[43].inertia[3][3]);
fnet24[6]=a24[5]*links[43].mcm[1] - a24[4]*links[43].mcm[2] + (links[43].mcm[3]*v24[2] + links[43].m*v24[4])*v24[5] + v24[4]*(links[43].mcm[3]*v24[1] - links[43].m*v24[5]) + (-(links[43].mcm[1]*v24[1]) - links[43].mcm[2]*v24[2])*v24[6] + v24[1]*(-(links[43].mcm[3]*v24[4]) - v24[2]*links[43].inertia[1][1] + v24[1]*links[43].inertia[1][2]) + a24[1]*links[43].inertia[1][3] + v24[2]*(-(links[43].mcm[3]*v24[5]) - v24[2]*links[43].inertia[1][2] + v24[1]*links[43].inertia[2][2]) + a24[2]*links[43].inertia[2][3] + v24[3]*(links[43].mcm[1]*v24[4] + links[43].mcm[2]*v24[5] - v24[2]*links[43].inertia[1][3] + v24[1]*links[43].inertia[2][3]) + a24[3]*links[43].inertia[3][3];

fnet28[1]=links[44].m*a28[4] - a28[3]*links[44].mcm[2] + a28[2]*links[44].mcm[3] - links[44].mcm[1]*Power(v28[2],2) - links[44].mcm[1]*Power(v28[3],2) + v28[1]*(links[44].mcm[2]*v28[2] + links[44].mcm[3]*v28[3]) - links[44].m*v28[3]*v28[5] + links[44].m*v28[2]*v28[6];
fnet28[2]=links[44].m*a28[5] + a28[3]*links[44].mcm[1] - a28[1]*links[44].mcm[3] - links[44].mcm[2]*Power(v28[1],2) - links[44].mcm[2]*Power(v28[3],2) + v28[2]*(links[44].mcm[1]*v28[1] + links[44].mcm[3]*v28[3]) + links[44].m*v28[3]*v28[4] - links[44].m*v28[1]*v28[6];
fnet28[3]=links[44].m*a28[6] - a28[2]*links[44].mcm[1] + a28[1]*links[44].mcm[2] - links[44].mcm[3]*Power(v28[1],2) - links[44].mcm[3]*Power(v28[2],2) + (links[44].mcm[1]*v28[1] + links[44].mcm[2]*v28[2])*v28[3] - links[44].m*v28[2]*v28[4] + links[44].m*v28[1]*v28[5];
fnet28[4]=a28[6]*links[44].mcm[2] - a28[5]*links[44].mcm[3] + (-(links[44].mcm[2]*v28[2]) - links[44].mcm[3]*v28[3])*v28[4] + (links[44].mcm[1]*v28[3] + links[44].m*v28[5])*v28[6] + v28[5]*(links[44].mcm[1]*v28[2] - links[44].m*v28[6]) + a28[1]*links[44].inertia[1][1] + a28[2]*links[44].inertia[1][2] + a28[3]*links[44].inertia[1][3] + v28[1]*(links[44].mcm[2]*v28[5] + links[44].mcm[3]*v28[6] - v28[3]*links[44].inertia[1][2] + v28[2]*links[44].inertia[1][3]) + v28[2]*(-(links[44].mcm[1]*v28[5]) - v28[3]*links[44].inertia[2][2] + v28[2]*links[44].inertia[2][3]) + v28[3]*(-(links[44].mcm[1]*v28[6]) - v28[3]*links[44].inertia[2][3] + v28[2]*links[44].inertia[3][3]);
fnet28[5]=-(a28[6]*links[44].mcm[1]) + a28[4]*links[44].mcm[3] + (-(links[44].mcm[1]*v28[1]) - links[44].mcm[3]*v28[3])*v28[5] + (links[44].mcm[2]*v28[3] - links[44].m*v28[4])*v28[6] + v28[4]*(links[44].mcm[2]*v28[1] + links[44].m*v28[6]) + a28[1]*links[44].inertia[1][2] + v28[1]*(-(links[44].mcm[2]*v28[4]) + v28[3]*links[44].inertia[1][1] - v28[1]*links[44].inertia[1][3]) + a28[2]*links[44].inertia[2][2] + a28[3]*links[44].inertia[2][3] + v28[2]*(links[44].mcm[1]*v28[4] + links[44].mcm[3]*v28[6] + v28[3]*links[44].inertia[1][2] - v28[1]*links[44].inertia[2][3]) + v28[3]*(-(links[44].mcm[2]*v28[6]) + v28[3]*links[44].inertia[1][3] - v28[1]*links[44].inertia[3][3]);
fnet28[6]=a28[5]*links[44].mcm[1] - a28[4]*links[44].mcm[2] + (links[44].mcm[3]*v28[2] + links[44].m*v28[4])*v28[5] + v28[4]*(links[44].mcm[3]*v28[1] - links[44].m*v28[5]) + (-(links[44].mcm[1]*v28[1]) - links[44].mcm[2]*v28[2])*v28[6] + v28[1]*(-(links[44].mcm[3]*v28[4]) - v28[2]*links[44].inertia[1][1] + v28[1]*links[44].inertia[1][2]) + a28[1]*links[44].inertia[1][3] + v28[2]*(-(links[44].mcm[3]*v28[5]) - v28[2]*links[44].inertia[1][2] + v28[1]*links[44].inertia[2][2]) + a28[2]*links[44].inertia[2][3] + v28[3]*(links[44].mcm[1]*v28[4] + links[44].mcm[2]*v28[5] - v28[2]*links[44].inertia[1][3] + v28[1]*links[44].inertia[2][3]) + a28[3]*links[44].inertia[3][3];

fnet32[1]=links[8].m*a32[4] - a32[3]*links[8].mcm[2] + a32[2]*links[8].mcm[3] - links[8].mcm[1]*Power(v32[2],2) - links[8].mcm[1]*Power(v32[3],2) + v32[1]*(links[8].mcm[2]*v32[2] + links[8].mcm[3]*v32[3]) - links[8].m*v32[3]*v32[5] + links[8].m*v32[2]*v32[6];
fnet32[2]=links[8].m*a32[5] + a32[3]*links[8].mcm[1] - a32[1]*links[8].mcm[3] - links[8].mcm[2]*Power(v32[1],2) - links[8].mcm[2]*Power(v32[3],2) + v32[2]*(links[8].mcm[1]*v32[1] + links[8].mcm[3]*v32[3]) + links[8].m*v32[3]*v32[4] - links[8].m*v32[1]*v32[6];
fnet32[3]=links[8].m*a32[6] - a32[2]*links[8].mcm[1] + a32[1]*links[8].mcm[2] - links[8].mcm[3]*Power(v32[1],2) - links[8].mcm[3]*Power(v32[2],2) + (links[8].mcm[1]*v32[1] + links[8].mcm[2]*v32[2])*v32[3] - links[8].m*v32[2]*v32[4] + links[8].m*v32[1]*v32[5];
fnet32[4]=a32[6]*links[8].mcm[2] - a32[5]*links[8].mcm[3] + (-(links[8].mcm[2]*v32[2]) - links[8].mcm[3]*v32[3])*v32[4] + (links[8].mcm[1]*v32[3] + links[8].m*v32[5])*v32[6] + v32[5]*(links[8].mcm[1]*v32[2] - links[8].m*v32[6]) + a32[1]*links[8].inertia[1][1] + a32[2]*links[8].inertia[1][2] + a32[3]*links[8].inertia[1][3] + v32[1]*(links[8].mcm[2]*v32[5] + links[8].mcm[3]*v32[6] - v32[3]*links[8].inertia[1][2] + v32[2]*links[8].inertia[1][3]) + v32[2]*(-(links[8].mcm[1]*v32[5]) - v32[3]*links[8].inertia[2][2] + v32[2]*links[8].inertia[2][3]) + v32[3]*(-(links[8].mcm[1]*v32[6]) - v32[3]*links[8].inertia[2][3] + v32[2]*links[8].inertia[3][3]);
fnet32[5]=-(a32[6]*links[8].mcm[1]) + a32[4]*links[8].mcm[3] + (-(links[8].mcm[1]*v32[1]) - links[8].mcm[3]*v32[3])*v32[5] + (links[8].mcm[2]*v32[3] - links[8].m*v32[4])*v32[6] + v32[4]*(links[8].mcm[2]*v32[1] + links[8].m*v32[6]) + a32[1]*links[8].inertia[1][2] + v32[1]*(-(links[8].mcm[2]*v32[4]) + v32[3]*links[8].inertia[1][1] - v32[1]*links[8].inertia[1][3]) + a32[2]*links[8].inertia[2][2] + a32[3]*links[8].inertia[2][3] + v32[2]*(links[8].mcm[1]*v32[4] + links[8].mcm[3]*v32[6] + v32[3]*links[8].inertia[1][2] - v32[1]*links[8].inertia[2][3]) + v32[3]*(-(links[8].mcm[2]*v32[6]) + v32[3]*links[8].inertia[1][3] - v32[1]*links[8].inertia[3][3]);
fnet32[6]=a32[5]*links[8].mcm[1] - a32[4]*links[8].mcm[2] + (links[8].mcm[3]*v32[2] + links[8].m*v32[4])*v32[5] + v32[4]*(links[8].mcm[3]*v32[1] - links[8].m*v32[5]) + (-(links[8].mcm[1]*v32[1]) - links[8].mcm[2]*v32[2])*v32[6] + v32[1]*(-(links[8].mcm[3]*v32[4]) - v32[2]*links[8].inertia[1][1] + v32[1]*links[8].inertia[1][2]) + a32[1]*links[8].inertia[1][3] + v32[2]*(-(links[8].mcm[3]*v32[5]) - v32[2]*links[8].inertia[1][2] + v32[1]*links[8].inertia[2][2]) + a32[2]*links[8].inertia[2][3] + v32[3]*(links[8].mcm[1]*v32[4] + links[8].mcm[2]*v32[5] - v32[2]*links[8].inertia[1][3] + v32[1]*links[8].inertia[2][3]) + a32[3]*links[8].inertia[3][3];

fnet33[1]=links[9].m*a33[4] - a33[3]*links[9].mcm[2] + a33[2]*links[9].mcm[3] - links[9].mcm[1]*Power(v33[2],2) - links[9].mcm[1]*Power(v33[3],2) + v33[1]*(links[9].mcm[2]*v33[2] + links[9].mcm[3]*v33[3]) - links[9].m*v33[3]*v33[5] + links[9].m*v33[2]*v33[6];
fnet33[2]=links[9].m*a33[5] + a33[3]*links[9].mcm[1] - a33[1]*links[9].mcm[3] - links[9].mcm[2]*Power(v33[1],2) - links[9].mcm[2]*Power(v33[3],2) + v33[2]*(links[9].mcm[1]*v33[1] + links[9].mcm[3]*v33[3]) + links[9].m*v33[3]*v33[4] - links[9].m*v33[1]*v33[6];
fnet33[3]=links[9].m*a33[6] - a33[2]*links[9].mcm[1] + a33[1]*links[9].mcm[2] - links[9].mcm[3]*Power(v33[1],2) - links[9].mcm[3]*Power(v33[2],2) + (links[9].mcm[1]*v33[1] + links[9].mcm[2]*v33[2])*v33[3] - links[9].m*v33[2]*v33[4] + links[9].m*v33[1]*v33[5];
fnet33[4]=a33[6]*links[9].mcm[2] - a33[5]*links[9].mcm[3] + (-(links[9].mcm[2]*v33[2]) - links[9].mcm[3]*v33[3])*v33[4] + (links[9].mcm[1]*v33[3] + links[9].m*v33[5])*v33[6] + v33[5]*(links[9].mcm[1]*v33[2] - links[9].m*v33[6]) + a33[1]*links[9].inertia[1][1] + a33[2]*links[9].inertia[1][2] + a33[3]*links[9].inertia[1][3] + v33[1]*(links[9].mcm[2]*v33[5] + links[9].mcm[3]*v33[6] - v33[3]*links[9].inertia[1][2] + v33[2]*links[9].inertia[1][3]) + v33[2]*(-(links[9].mcm[1]*v33[5]) - v33[3]*links[9].inertia[2][2] + v33[2]*links[9].inertia[2][3]) + v33[3]*(-(links[9].mcm[1]*v33[6]) - v33[3]*links[9].inertia[2][3] + v33[2]*links[9].inertia[3][3]);
fnet33[5]=-(a33[6]*links[9].mcm[1]) + a33[4]*links[9].mcm[3] + (-(links[9].mcm[1]*v33[1]) - links[9].mcm[3]*v33[3])*v33[5] + (links[9].mcm[2]*v33[3] - links[9].m*v33[4])*v33[6] + v33[4]*(links[9].mcm[2]*v33[1] + links[9].m*v33[6]) + a33[1]*links[9].inertia[1][2] + v33[1]*(-(links[9].mcm[2]*v33[4]) + v33[3]*links[9].inertia[1][1] - v33[1]*links[9].inertia[1][3]) + a33[2]*links[9].inertia[2][2] + a33[3]*links[9].inertia[2][3] + v33[2]*(links[9].mcm[1]*v33[4] + links[9].mcm[3]*v33[6] + v33[3]*links[9].inertia[1][2] - v33[1]*links[9].inertia[2][3]) + v33[3]*(-(links[9].mcm[2]*v33[6]) + v33[3]*links[9].inertia[1][3] - v33[1]*links[9].inertia[3][3]);
fnet33[6]=a33[5]*links[9].mcm[1] - a33[4]*links[9].mcm[2] + (links[9].mcm[3]*v33[2] + links[9].m*v33[4])*v33[5] + v33[4]*(links[9].mcm[3]*v33[1] - links[9].m*v33[5]) + (-(links[9].mcm[1]*v33[1]) - links[9].mcm[2]*v33[2])*v33[6] + v33[1]*(-(links[9].mcm[3]*v33[4]) - v33[2]*links[9].inertia[1][1] + v33[1]*links[9].inertia[1][2]) + a33[1]*links[9].inertia[1][3] + v33[2]*(-(links[9].mcm[3]*v33[5]) - v33[2]*links[9].inertia[1][2] + v33[1]*links[9].inertia[2][2]) + a33[2]*links[9].inertia[2][3] + v33[3]*(links[9].mcm[1]*v33[4] + links[9].mcm[2]*v33[5] - v33[2]*links[9].inertia[1][3] + v33[1]*links[9].inertia[2][3]) + a33[3]*links[9].inertia[3][3];

fnet34[1]=links[10].m*a34[4] - a34[3]*links[10].mcm[2] + a34[2]*links[10].mcm[3] - links[10].mcm[1]*Power(v34[2],2) - links[10].mcm[1]*Power(v34[3],2) + v34[1]*(links[10].mcm[2]*v34[2] + links[10].mcm[3]*v34[3]) - links[10].m*v34[3]*v34[5] + links[10].m*v34[2]*v34[6];
fnet34[2]=links[10].m*a34[5] + a34[3]*links[10].mcm[1] - a34[1]*links[10].mcm[3] - links[10].mcm[2]*Power(v34[1],2) - links[10].mcm[2]*Power(v34[3],2) + v34[2]*(links[10].mcm[1]*v34[1] + links[10].mcm[3]*v34[3]) + links[10].m*v34[3]*v34[4] - links[10].m*v34[1]*v34[6];
fnet34[3]=links[10].m*a34[6] - a34[2]*links[10].mcm[1] + a34[1]*links[10].mcm[2] - links[10].mcm[3]*Power(v34[1],2) - links[10].mcm[3]*Power(v34[2],2) + (links[10].mcm[1]*v34[1] + links[10].mcm[2]*v34[2])*v34[3] - links[10].m*v34[2]*v34[4] + links[10].m*v34[1]*v34[5];
fnet34[4]=a34[6]*links[10].mcm[2] - a34[5]*links[10].mcm[3] + (-(links[10].mcm[2]*v34[2]) - links[10].mcm[3]*v34[3])*v34[4] + (links[10].mcm[1]*v34[3] + links[10].m*v34[5])*v34[6] + v34[5]*(links[10].mcm[1]*v34[2] - links[10].m*v34[6]) + a34[1]*links[10].inertia[1][1] + a34[2]*links[10].inertia[1][2] + a34[3]*links[10].inertia[1][3] + v34[1]*(links[10].mcm[2]*v34[5] + links[10].mcm[3]*v34[6] - v34[3]*links[10].inertia[1][2] + v34[2]*links[10].inertia[1][3]) + v34[2]*(-(links[10].mcm[1]*v34[5]) - v34[3]*links[10].inertia[2][2] + v34[2]*links[10].inertia[2][3]) + v34[3]*(-(links[10].mcm[1]*v34[6]) - v34[3]*links[10].inertia[2][3] + v34[2]*links[10].inertia[3][3]);
fnet34[5]=-(a34[6]*links[10].mcm[1]) + a34[4]*links[10].mcm[3] + (-(links[10].mcm[1]*v34[1]) - links[10].mcm[3]*v34[3])*v34[5] + (links[10].mcm[2]*v34[3] - links[10].m*v34[4])*v34[6] + v34[4]*(links[10].mcm[2]*v34[1] + links[10].m*v34[6]) + a34[1]*links[10].inertia[1][2] + v34[1]*(-(links[10].mcm[2]*v34[4]) + v34[3]*links[10].inertia[1][1] - v34[1]*links[10].inertia[1][3]) + a34[2]*links[10].inertia[2][2] + a34[3]*links[10].inertia[2][3] + v34[2]*(links[10].mcm[1]*v34[4] + links[10].mcm[3]*v34[6] + v34[3]*links[10].inertia[1][2] - v34[1]*links[10].inertia[2][3]) + v34[3]*(-(links[10].mcm[2]*v34[6]) + v34[3]*links[10].inertia[1][3] - v34[1]*links[10].inertia[3][3]);
fnet34[6]=a34[5]*links[10].mcm[1] - a34[4]*links[10].mcm[2] + (links[10].mcm[3]*v34[2] + links[10].m*v34[4])*v34[5] + v34[4]*(links[10].mcm[3]*v34[1] - links[10].m*v34[5]) + (-(links[10].mcm[1]*v34[1]) - links[10].mcm[2]*v34[2])*v34[6] + v34[1]*(-(links[10].mcm[3]*v34[4]) - v34[2]*links[10].inertia[1][1] + v34[1]*links[10].inertia[1][2]) + a34[1]*links[10].inertia[1][3] + v34[2]*(-(links[10].mcm[3]*v34[5]) - v34[2]*links[10].inertia[1][2] + v34[1]*links[10].inertia[2][2]) + a34[2]*links[10].inertia[2][3] + v34[3]*(links[10].mcm[1]*v34[4] + links[10].mcm[2]*v34[5] - v34[2]*links[10].inertia[1][3] + v34[1]*links[10].inertia[2][3]) + a34[3]*links[10].inertia[3][3];

fnet35[1]=links[11].m*a35[4] - a35[3]*links[11].mcm[2] + a35[2]*links[11].mcm[3] - links[11].mcm[1]*Power(v35[2],2) - links[11].mcm[1]*Power(v35[3],2) + v35[1]*(links[11].mcm[2]*v35[2] + links[11].mcm[3]*v35[3]) - links[11].m*v35[3]*v35[5] + links[11].m*v35[2]*v35[6];
fnet35[2]=links[11].m*a35[5] + a35[3]*links[11].mcm[1] - a35[1]*links[11].mcm[3] - links[11].mcm[2]*Power(v35[1],2) - links[11].mcm[2]*Power(v35[3],2) + v35[2]*(links[11].mcm[1]*v35[1] + links[11].mcm[3]*v35[3]) + links[11].m*v35[3]*v35[4] - links[11].m*v35[1]*v35[6];
fnet35[3]=links[11].m*a35[6] - a35[2]*links[11].mcm[1] + a35[1]*links[11].mcm[2] - links[11].mcm[3]*Power(v35[1],2) - links[11].mcm[3]*Power(v35[2],2) + (links[11].mcm[1]*v35[1] + links[11].mcm[2]*v35[2])*v35[3] - links[11].m*v35[2]*v35[4] + links[11].m*v35[1]*v35[5];
fnet35[4]=a35[6]*links[11].mcm[2] - a35[5]*links[11].mcm[3] + (-(links[11].mcm[2]*v35[2]) - links[11].mcm[3]*v35[3])*v35[4] + (links[11].mcm[1]*v35[3] + links[11].m*v35[5])*v35[6] + v35[5]*(links[11].mcm[1]*v35[2] - links[11].m*v35[6]) + a35[1]*links[11].inertia[1][1] + a35[2]*links[11].inertia[1][2] + a35[3]*links[11].inertia[1][3] + v35[1]*(links[11].mcm[2]*v35[5] + links[11].mcm[3]*v35[6] - v35[3]*links[11].inertia[1][2] + v35[2]*links[11].inertia[1][3]) + v35[2]*(-(links[11].mcm[1]*v35[5]) - v35[3]*links[11].inertia[2][2] + v35[2]*links[11].inertia[2][3]) + v35[3]*(-(links[11].mcm[1]*v35[6]) - v35[3]*links[11].inertia[2][3] + v35[2]*links[11].inertia[3][3]);
fnet35[5]=-(a35[6]*links[11].mcm[1]) + a35[4]*links[11].mcm[3] + (-(links[11].mcm[1]*v35[1]) - links[11].mcm[3]*v35[3])*v35[5] + (links[11].mcm[2]*v35[3] - links[11].m*v35[4])*v35[6] + v35[4]*(links[11].mcm[2]*v35[1] + links[11].m*v35[6]) + a35[1]*links[11].inertia[1][2] + v35[1]*(-(links[11].mcm[2]*v35[4]) + v35[3]*links[11].inertia[1][1] - v35[1]*links[11].inertia[1][3]) + a35[2]*links[11].inertia[2][2] + a35[3]*links[11].inertia[2][3] + v35[2]*(links[11].mcm[1]*v35[4] + links[11].mcm[3]*v35[6] + v35[3]*links[11].inertia[1][2] - v35[1]*links[11].inertia[2][3]) + v35[3]*(-(links[11].mcm[2]*v35[6]) + v35[3]*links[11].inertia[1][3] - v35[1]*links[11].inertia[3][3]);
fnet35[6]=a35[5]*links[11].mcm[1] - a35[4]*links[11].mcm[2] + (links[11].mcm[3]*v35[2] + links[11].m*v35[4])*v35[5] + v35[4]*(links[11].mcm[3]*v35[1] - links[11].m*v35[5]) + (-(links[11].mcm[1]*v35[1]) - links[11].mcm[2]*v35[2])*v35[6] + v35[1]*(-(links[11].mcm[3]*v35[4]) - v35[2]*links[11].inertia[1][1] + v35[1]*links[11].inertia[1][2]) + a35[1]*links[11].inertia[1][3] + v35[2]*(-(links[11].mcm[3]*v35[5]) - v35[2]*links[11].inertia[1][2] + v35[1]*links[11].inertia[2][2]) + a35[2]*links[11].inertia[2][3] + v35[3]*(links[11].mcm[1]*v35[4] + links[11].mcm[2]*v35[5] - v35[2]*links[11].inertia[1][3] + v35[1]*links[11].inertia[2][3]) + a35[3]*links[11].inertia[3][3];

fnet36[1]=links[12].m*a36[4] - a36[3]*links[12].mcm[2] + a36[2]*links[12].mcm[3] - links[12].mcm[1]*Power(v36[2],2) - links[12].mcm[1]*Power(v36[3],2) + v36[1]*(links[12].mcm[2]*v36[2] + links[12].mcm[3]*v36[3]) - links[12].m*v36[3]*v36[5] + links[12].m*v36[2]*v36[6];
fnet36[2]=links[12].m*a36[5] + a36[3]*links[12].mcm[1] - a36[1]*links[12].mcm[3] - links[12].mcm[2]*Power(v36[1],2) - links[12].mcm[2]*Power(v36[3],2) + v36[2]*(links[12].mcm[1]*v36[1] + links[12].mcm[3]*v36[3]) + links[12].m*v36[3]*v36[4] - links[12].m*v36[1]*v36[6];
fnet36[3]=links[12].m*a36[6] - a36[2]*links[12].mcm[1] + a36[1]*links[12].mcm[2] - links[12].mcm[3]*Power(v36[1],2) - links[12].mcm[3]*Power(v36[2],2) + (links[12].mcm[1]*v36[1] + links[12].mcm[2]*v36[2])*v36[3] - links[12].m*v36[2]*v36[4] + links[12].m*v36[1]*v36[5];
fnet36[4]=a36[6]*links[12].mcm[2] - a36[5]*links[12].mcm[3] + (-(links[12].mcm[2]*v36[2]) - links[12].mcm[3]*v36[3])*v36[4] + (links[12].mcm[1]*v36[3] + links[12].m*v36[5])*v36[6] + v36[5]*(links[12].mcm[1]*v36[2] - links[12].m*v36[6]) + a36[1]*links[12].inertia[1][1] + a36[2]*links[12].inertia[1][2] + a36[3]*links[12].inertia[1][3] + v36[1]*(links[12].mcm[2]*v36[5] + links[12].mcm[3]*v36[6] - v36[3]*links[12].inertia[1][2] + v36[2]*links[12].inertia[1][3]) + v36[2]*(-(links[12].mcm[1]*v36[5]) - v36[3]*links[12].inertia[2][2] + v36[2]*links[12].inertia[2][3]) + v36[3]*(-(links[12].mcm[1]*v36[6]) - v36[3]*links[12].inertia[2][3] + v36[2]*links[12].inertia[3][3]);
fnet36[5]=-(a36[6]*links[12].mcm[1]) + a36[4]*links[12].mcm[3] + (-(links[12].mcm[1]*v36[1]) - links[12].mcm[3]*v36[3])*v36[5] + (links[12].mcm[2]*v36[3] - links[12].m*v36[4])*v36[6] + v36[4]*(links[12].mcm[2]*v36[1] + links[12].m*v36[6]) + a36[1]*links[12].inertia[1][2] + v36[1]*(-(links[12].mcm[2]*v36[4]) + v36[3]*links[12].inertia[1][1] - v36[1]*links[12].inertia[1][3]) + a36[2]*links[12].inertia[2][2] + a36[3]*links[12].inertia[2][3] + v36[2]*(links[12].mcm[1]*v36[4] + links[12].mcm[3]*v36[6] + v36[3]*links[12].inertia[1][2] - v36[1]*links[12].inertia[2][3]) + v36[3]*(-(links[12].mcm[2]*v36[6]) + v36[3]*links[12].inertia[1][3] - v36[1]*links[12].inertia[3][3]);
fnet36[6]=a36[5]*links[12].mcm[1] - a36[4]*links[12].mcm[2] + (links[12].mcm[3]*v36[2] + links[12].m*v36[4])*v36[5] + v36[4]*(links[12].mcm[3]*v36[1] - links[12].m*v36[5]) + (-(links[12].mcm[1]*v36[1]) - links[12].mcm[2]*v36[2])*v36[6] + v36[1]*(-(links[12].mcm[3]*v36[4]) - v36[2]*links[12].inertia[1][1] + v36[1]*links[12].inertia[1][2]) + a36[1]*links[12].inertia[1][3] + v36[2]*(-(links[12].mcm[3]*v36[5]) - v36[2]*links[12].inertia[1][2] + v36[1]*links[12].inertia[2][2]) + a36[2]*links[12].inertia[2][3] + v36[3]*(links[12].mcm[1]*v36[4] + links[12].mcm[2]*v36[5] - v36[2]*links[12].inertia[1][3] + v36[1]*links[12].inertia[2][3]) + a36[3]*links[12].inertia[3][3];

fnet37[1]=links[13].m*a37[4] - a37[3]*links[13].mcm[2] + a37[2]*links[13].mcm[3] - links[13].mcm[1]*Power(v37[2],2) - links[13].mcm[1]*Power(v37[3],2) + v37[1]*(links[13].mcm[2]*v37[2] + links[13].mcm[3]*v37[3]) - links[13].m*v37[3]*v37[5] + links[13].m*v37[2]*v37[6];
fnet37[2]=links[13].m*a37[5] + a37[3]*links[13].mcm[1] - a37[1]*links[13].mcm[3] - links[13].mcm[2]*Power(v37[1],2) - links[13].mcm[2]*Power(v37[3],2) + v37[2]*(links[13].mcm[1]*v37[1] + links[13].mcm[3]*v37[3]) + links[13].m*v37[3]*v37[4] - links[13].m*v37[1]*v37[6];
fnet37[3]=links[13].m*a37[6] - a37[2]*links[13].mcm[1] + a37[1]*links[13].mcm[2] - links[13].mcm[3]*Power(v37[1],2) - links[13].mcm[3]*Power(v37[2],2) + (links[13].mcm[1]*v37[1] + links[13].mcm[2]*v37[2])*v37[3] - links[13].m*v37[2]*v37[4] + links[13].m*v37[1]*v37[5];
fnet37[4]=a37[6]*links[13].mcm[2] - a37[5]*links[13].mcm[3] + (-(links[13].mcm[2]*v37[2]) - links[13].mcm[3]*v37[3])*v37[4] + (links[13].mcm[1]*v37[3] + links[13].m*v37[5])*v37[6] + v37[5]*(links[13].mcm[1]*v37[2] - links[13].m*v37[6]) + a37[1]*links[13].inertia[1][1] + a37[2]*links[13].inertia[1][2] + a37[3]*links[13].inertia[1][3] + v37[1]*(links[13].mcm[2]*v37[5] + links[13].mcm[3]*v37[6] - v37[3]*links[13].inertia[1][2] + v37[2]*links[13].inertia[1][3]) + v37[2]*(-(links[13].mcm[1]*v37[5]) - v37[3]*links[13].inertia[2][2] + v37[2]*links[13].inertia[2][3]) + v37[3]*(-(links[13].mcm[1]*v37[6]) - v37[3]*links[13].inertia[2][3] + v37[2]*links[13].inertia[3][3]);
fnet37[5]=-(a37[6]*links[13].mcm[1]) + a37[4]*links[13].mcm[3] + (-(links[13].mcm[1]*v37[1]) - links[13].mcm[3]*v37[3])*v37[5] + (links[13].mcm[2]*v37[3] - links[13].m*v37[4])*v37[6] + v37[4]*(links[13].mcm[2]*v37[1] + links[13].m*v37[6]) + a37[1]*links[13].inertia[1][2] + v37[1]*(-(links[13].mcm[2]*v37[4]) + v37[3]*links[13].inertia[1][1] - v37[1]*links[13].inertia[1][3]) + a37[2]*links[13].inertia[2][2] + a37[3]*links[13].inertia[2][3] + v37[2]*(links[13].mcm[1]*v37[4] + links[13].mcm[3]*v37[6] + v37[3]*links[13].inertia[1][2] - v37[1]*links[13].inertia[2][3]) + v37[3]*(-(links[13].mcm[2]*v37[6]) + v37[3]*links[13].inertia[1][3] - v37[1]*links[13].inertia[3][3]);
fnet37[6]=a37[5]*links[13].mcm[1] - a37[4]*links[13].mcm[2] + (links[13].mcm[3]*v37[2] + links[13].m*v37[4])*v37[5] + v37[4]*(links[13].mcm[3]*v37[1] - links[13].m*v37[5]) + (-(links[13].mcm[1]*v37[1]) - links[13].mcm[2]*v37[2])*v37[6] + v37[1]*(-(links[13].mcm[3]*v37[4]) - v37[2]*links[13].inertia[1][1] + v37[1]*links[13].inertia[1][2]) + a37[1]*links[13].inertia[1][3] + v37[2]*(-(links[13].mcm[3]*v37[5]) - v37[2]*links[13].inertia[1][2] + v37[1]*links[13].inertia[2][2]) + a37[2]*links[13].inertia[2][3] + v37[3]*(links[13].mcm[1]*v37[4] + links[13].mcm[2]*v37[5] - v37[2]*links[13].inertia[1][3] + v37[1]*links[13].inertia[2][3]) + a37[3]*links[13].inertia[3][3];

fnet38[1]=links[14].m*a38[4] - a38[3]*links[14].mcm[2] + a38[2]*links[14].mcm[3] - links[14].mcm[1]*Power(v38[2],2) - links[14].mcm[1]*Power(v38[3],2) + v38[1]*(links[14].mcm[2]*v38[2] + links[14].mcm[3]*v38[3]) - links[14].m*v38[3]*v38[5] + links[14].m*v38[2]*v38[6];
fnet38[2]=links[14].m*a38[5] + a38[3]*links[14].mcm[1] - a38[1]*links[14].mcm[3] - links[14].mcm[2]*Power(v38[1],2) - links[14].mcm[2]*Power(v38[3],2) + v38[2]*(links[14].mcm[1]*v38[1] + links[14].mcm[3]*v38[3]) + links[14].m*v38[3]*v38[4] - links[14].m*v38[1]*v38[6];
fnet38[3]=links[14].m*a38[6] - a38[2]*links[14].mcm[1] + a38[1]*links[14].mcm[2] - links[14].mcm[3]*Power(v38[1],2) - links[14].mcm[3]*Power(v38[2],2) + (links[14].mcm[1]*v38[1] + links[14].mcm[2]*v38[2])*v38[3] - links[14].m*v38[2]*v38[4] + links[14].m*v38[1]*v38[5];
fnet38[4]=a38[6]*links[14].mcm[2] - a38[5]*links[14].mcm[3] + (-(links[14].mcm[2]*v38[2]) - links[14].mcm[3]*v38[3])*v38[4] + (links[14].mcm[1]*v38[3] + links[14].m*v38[5])*v38[6] + v38[5]*(links[14].mcm[1]*v38[2] - links[14].m*v38[6]) + a38[1]*links[14].inertia[1][1] + a38[2]*links[14].inertia[1][2] + a38[3]*links[14].inertia[1][3] + v38[1]*(links[14].mcm[2]*v38[5] + links[14].mcm[3]*v38[6] - v38[3]*links[14].inertia[1][2] + v38[2]*links[14].inertia[1][3]) + v38[2]*(-(links[14].mcm[1]*v38[5]) - v38[3]*links[14].inertia[2][2] + v38[2]*links[14].inertia[2][3]) + v38[3]*(-(links[14].mcm[1]*v38[6]) - v38[3]*links[14].inertia[2][3] + v38[2]*links[14].inertia[3][3]);
fnet38[5]=-(a38[6]*links[14].mcm[1]) + a38[4]*links[14].mcm[3] + (-(links[14].mcm[1]*v38[1]) - links[14].mcm[3]*v38[3])*v38[5] + (links[14].mcm[2]*v38[3] - links[14].m*v38[4])*v38[6] + v38[4]*(links[14].mcm[2]*v38[1] + links[14].m*v38[6]) + a38[1]*links[14].inertia[1][2] + v38[1]*(-(links[14].mcm[2]*v38[4]) + v38[3]*links[14].inertia[1][1] - v38[1]*links[14].inertia[1][3]) + a38[2]*links[14].inertia[2][2] + a38[3]*links[14].inertia[2][3] + v38[2]*(links[14].mcm[1]*v38[4] + links[14].mcm[3]*v38[6] + v38[3]*links[14].inertia[1][2] - v38[1]*links[14].inertia[2][3]) + v38[3]*(-(links[14].mcm[2]*v38[6]) + v38[3]*links[14].inertia[1][3] - v38[1]*links[14].inertia[3][3]);
fnet38[6]=a38[5]*links[14].mcm[1] - a38[4]*links[14].mcm[2] + (links[14].mcm[3]*v38[2] + links[14].m*v38[4])*v38[5] + v38[4]*(links[14].mcm[3]*v38[1] - links[14].m*v38[5]) + (-(links[14].mcm[1]*v38[1]) - links[14].mcm[2]*v38[2])*v38[6] + v38[1]*(-(links[14].mcm[3]*v38[4]) - v38[2]*links[14].inertia[1][1] + v38[1]*links[14].inertia[1][2]) + a38[1]*links[14].inertia[1][3] + v38[2]*(-(links[14].mcm[3]*v38[5]) - v38[2]*links[14].inertia[1][2] + v38[1]*links[14].inertia[2][2]) + a38[2]*links[14].inertia[2][3] + v38[3]*(links[14].mcm[1]*v38[4] + links[14].mcm[2]*v38[5] - v38[2]*links[14].inertia[1][3] + v38[1]*links[14].inertia[2][3]) + a38[3]*links[14].inertia[3][3];

fnet39[1]=eff[1].m*a39[4] - a39[3]*eff[1].mcm[2] + a39[2]*eff[1].mcm[3] - eff[1].mcm[1]*Power(v39[2],2) - eff[1].mcm[1]*Power(v39[3],2) + v39[1]*(eff[1].mcm[2]*v39[2] + eff[1].mcm[3]*v39[3]) - eff[1].m*v39[3]*v39[5] + eff[1].m*v39[2]*v39[6];
fnet39[2]=eff[1].m*a39[5] + a39[3]*eff[1].mcm[1] - a39[1]*eff[1].mcm[3] - eff[1].mcm[2]*Power(v39[1],2) - eff[1].mcm[2]*Power(v39[3],2) + v39[2]*(eff[1].mcm[1]*v39[1] + eff[1].mcm[3]*v39[3]) + eff[1].m*v39[3]*v39[4] - eff[1].m*v39[1]*v39[6];
fnet39[3]=eff[1].m*a39[6] - a39[2]*eff[1].mcm[1] + a39[1]*eff[1].mcm[2] - eff[1].mcm[3]*Power(v39[1],2) - eff[1].mcm[3]*Power(v39[2],2) + (eff[1].mcm[1]*v39[1] + eff[1].mcm[2]*v39[2])*v39[3] - eff[1].m*v39[2]*v39[4] + eff[1].m*v39[1]*v39[5];
fnet39[4]=a39[6]*eff[1].mcm[2] - a39[5]*eff[1].mcm[3] + (-(eff[1].mcm[2]*v39[2]) - eff[1].mcm[3]*v39[3])*v39[4] - eff[1].mcm[1]*v39[2]*v39[5] - eff[1].mcm[1]*v39[3]*v39[6] + (eff[1].mcm[1]*v39[3] + eff[1].m*v39[5])*v39[6] + v39[5]*(eff[1].mcm[1]*v39[2] - eff[1].m*v39[6]) + v39[1]*(eff[1].mcm[2]*v39[5] + eff[1].mcm[3]*v39[6]);
fnet39[5]=-(a39[6]*eff[1].mcm[1]) + a39[4]*eff[1].mcm[3] - eff[1].mcm[2]*v39[1]*v39[4] + (-(eff[1].mcm[1]*v39[1]) - eff[1].mcm[3]*v39[3])*v39[5] - eff[1].mcm[2]*v39[3]*v39[6] + (eff[1].mcm[2]*v39[3] - eff[1].m*v39[4])*v39[6] + v39[4]*(eff[1].mcm[2]*v39[1] + eff[1].m*v39[6]) + v39[2]*(eff[1].mcm[1]*v39[4] + eff[1].mcm[3]*v39[6]);
fnet39[6]=a39[5]*eff[1].mcm[1] - a39[4]*eff[1].mcm[2] - eff[1].mcm[3]*v39[1]*v39[4] - eff[1].mcm[3]*v39[2]*v39[5] + (eff[1].mcm[3]*v39[2] + eff[1].m*v39[4])*v39[5] + v39[4]*(eff[1].mcm[3]*v39[1] - eff[1].m*v39[5]) + v39[3]*(eff[1].mcm[1]*v39[4] + eff[1].mcm[2]*v39[5]) + (-(eff[1].mcm[1]*v39[1]) - eff[1].mcm[2]*v39[2])*v39[6];

fnet40[1]=links[45].m*a40[4] - a40[3]*links[45].mcm[2] + a40[2]*links[45].mcm[3] - links[45].mcm[1]*Power(v40[2],2) - links[45].mcm[1]*Power(v40[3],2) + v40[1]*(links[45].mcm[2]*v40[2] + links[45].mcm[3]*v40[3]) - links[45].m*v40[3]*v40[5] + links[45].m*v40[2]*v40[6];
fnet40[2]=links[45].m*a40[5] + a40[3]*links[45].mcm[1] - a40[1]*links[45].mcm[3] - links[45].mcm[2]*Power(v40[1],2) - links[45].mcm[2]*Power(v40[3],2) + v40[2]*(links[45].mcm[1]*v40[1] + links[45].mcm[3]*v40[3]) + links[45].m*v40[3]*v40[4] - links[45].m*v40[1]*v40[6];
fnet40[3]=links[45].m*a40[6] - a40[2]*links[45].mcm[1] + a40[1]*links[45].mcm[2] - links[45].mcm[3]*Power(v40[1],2) - links[45].mcm[3]*Power(v40[2],2) + (links[45].mcm[1]*v40[1] + links[45].mcm[2]*v40[2])*v40[3] - links[45].m*v40[2]*v40[4] + links[45].m*v40[1]*v40[5];
fnet40[4]=a40[6]*links[45].mcm[2] - a40[5]*links[45].mcm[3] + (-(links[45].mcm[2]*v40[2]) - links[45].mcm[3]*v40[3])*v40[4] + (links[45].mcm[1]*v40[3] + links[45].m*v40[5])*v40[6] + v40[5]*(links[45].mcm[1]*v40[2] - links[45].m*v40[6]) + a40[1]*links[45].inertia[1][1] + a40[2]*links[45].inertia[1][2] + a40[3]*links[45].inertia[1][3] + v40[1]*(links[45].mcm[2]*v40[5] + links[45].mcm[3]*v40[6] - v40[3]*links[45].inertia[1][2] + v40[2]*links[45].inertia[1][3]) + v40[2]*(-(links[45].mcm[1]*v40[5]) - v40[3]*links[45].inertia[2][2] + v40[2]*links[45].inertia[2][3]) + v40[3]*(-(links[45].mcm[1]*v40[6]) - v40[3]*links[45].inertia[2][3] + v40[2]*links[45].inertia[3][3]);
fnet40[5]=-(a40[6]*links[45].mcm[1]) + a40[4]*links[45].mcm[3] + (-(links[45].mcm[1]*v40[1]) - links[45].mcm[3]*v40[3])*v40[5] + (links[45].mcm[2]*v40[3] - links[45].m*v40[4])*v40[6] + v40[4]*(links[45].mcm[2]*v40[1] + links[45].m*v40[6]) + a40[1]*links[45].inertia[1][2] + v40[1]*(-(links[45].mcm[2]*v40[4]) + v40[3]*links[45].inertia[1][1] - v40[1]*links[45].inertia[1][3]) + a40[2]*links[45].inertia[2][2] + a40[3]*links[45].inertia[2][3] + v40[2]*(links[45].mcm[1]*v40[4] + links[45].mcm[3]*v40[6] + v40[3]*links[45].inertia[1][2] - v40[1]*links[45].inertia[2][3]) + v40[3]*(-(links[45].mcm[2]*v40[6]) + v40[3]*links[45].inertia[1][3] - v40[1]*links[45].inertia[3][3]);
fnet40[6]=a40[5]*links[45].mcm[1] - a40[4]*links[45].mcm[2] + (links[45].mcm[3]*v40[2] + links[45].m*v40[4])*v40[5] + v40[4]*(links[45].mcm[3]*v40[1] - links[45].m*v40[5]) + (-(links[45].mcm[1]*v40[1]) - links[45].mcm[2]*v40[2])*v40[6] + v40[1]*(-(links[45].mcm[3]*v40[4]) - v40[2]*links[45].inertia[1][1] + v40[1]*links[45].inertia[1][2]) + a40[1]*links[45].inertia[1][3] + v40[2]*(-(links[45].mcm[3]*v40[5]) - v40[2]*links[45].inertia[1][2] + v40[1]*links[45].inertia[2][2]) + a40[2]*links[45].inertia[2][3] + v40[3]*(links[45].mcm[1]*v40[4] + links[45].mcm[2]*v40[5] - v40[2]*links[45].inertia[1][3] + v40[1]*links[45].inertia[2][3]) + a40[3]*links[45].inertia[3][3];

fnet41[1]=links[46].m*a41[4] - a41[3]*links[46].mcm[2] + a41[2]*links[46].mcm[3] - links[46].mcm[1]*Power(v41[2],2) - links[46].mcm[1]*Power(v41[3],2) + v41[1]*(links[46].mcm[2]*v41[2] + links[46].mcm[3]*v41[3]) - links[46].m*v41[3]*v41[5] + links[46].m*v41[2]*v41[6];
fnet41[2]=links[46].m*a41[5] + a41[3]*links[46].mcm[1] - a41[1]*links[46].mcm[3] - links[46].mcm[2]*Power(v41[1],2) - links[46].mcm[2]*Power(v41[3],2) + v41[2]*(links[46].mcm[1]*v41[1] + links[46].mcm[3]*v41[3]) + links[46].m*v41[3]*v41[4] - links[46].m*v41[1]*v41[6];
fnet41[3]=links[46].m*a41[6] - a41[2]*links[46].mcm[1] + a41[1]*links[46].mcm[2] - links[46].mcm[3]*Power(v41[1],2) - links[46].mcm[3]*Power(v41[2],2) + (links[46].mcm[1]*v41[1] + links[46].mcm[2]*v41[2])*v41[3] - links[46].m*v41[2]*v41[4] + links[46].m*v41[1]*v41[5];
fnet41[4]=a41[6]*links[46].mcm[2] - a41[5]*links[46].mcm[3] + (-(links[46].mcm[2]*v41[2]) - links[46].mcm[3]*v41[3])*v41[4] + (links[46].mcm[1]*v41[3] + links[46].m*v41[5])*v41[6] + v41[5]*(links[46].mcm[1]*v41[2] - links[46].m*v41[6]) + a41[1]*links[46].inertia[1][1] + a41[2]*links[46].inertia[1][2] + a41[3]*links[46].inertia[1][3] + v41[1]*(links[46].mcm[2]*v41[5] + links[46].mcm[3]*v41[6] - v41[3]*links[46].inertia[1][2] + v41[2]*links[46].inertia[1][3]) + v41[2]*(-(links[46].mcm[1]*v41[5]) - v41[3]*links[46].inertia[2][2] + v41[2]*links[46].inertia[2][3]) + v41[3]*(-(links[46].mcm[1]*v41[6]) - v41[3]*links[46].inertia[2][3] + v41[2]*links[46].inertia[3][3]);
fnet41[5]=-(a41[6]*links[46].mcm[1]) + a41[4]*links[46].mcm[3] + (-(links[46].mcm[1]*v41[1]) - links[46].mcm[3]*v41[3])*v41[5] + (links[46].mcm[2]*v41[3] - links[46].m*v41[4])*v41[6] + v41[4]*(links[46].mcm[2]*v41[1] + links[46].m*v41[6]) + a41[1]*links[46].inertia[1][2] + v41[1]*(-(links[46].mcm[2]*v41[4]) + v41[3]*links[46].inertia[1][1] - v41[1]*links[46].inertia[1][3]) + a41[2]*links[46].inertia[2][2] + a41[3]*links[46].inertia[2][3] + v41[2]*(links[46].mcm[1]*v41[4] + links[46].mcm[3]*v41[6] + v41[3]*links[46].inertia[1][2] - v41[1]*links[46].inertia[2][3]) + v41[3]*(-(links[46].mcm[2]*v41[6]) + v41[3]*links[46].inertia[1][3] - v41[1]*links[46].inertia[3][3]);
fnet41[6]=a41[5]*links[46].mcm[1] - a41[4]*links[46].mcm[2] + (links[46].mcm[3]*v41[2] + links[46].m*v41[4])*v41[5] + v41[4]*(links[46].mcm[3]*v41[1] - links[46].m*v41[5]) + (-(links[46].mcm[1]*v41[1]) - links[46].mcm[2]*v41[2])*v41[6] + v41[1]*(-(links[46].mcm[3]*v41[4]) - v41[2]*links[46].inertia[1][1] + v41[1]*links[46].inertia[1][2]) + a41[1]*links[46].inertia[1][3] + v41[2]*(-(links[46].mcm[3]*v41[5]) - v41[2]*links[46].inertia[1][2] + v41[1]*links[46].inertia[2][2]) + a41[2]*links[46].inertia[2][3] + v41[3]*(links[46].mcm[1]*v41[4] + links[46].mcm[2]*v41[5] - v41[2]*links[46].inertia[1][3] + v41[1]*links[46].inertia[2][3]) + a41[3]*links[46].inertia[3][3];

fnet44[1]=links[47].m*a44[4] - a44[3]*links[47].mcm[2] + a44[2]*links[47].mcm[3] - links[47].mcm[1]*Power(v44[2],2) - links[47].mcm[1]*Power(v44[3],2) + v44[1]*(links[47].mcm[2]*v44[2] + links[47].mcm[3]*v44[3]) - links[47].m*v44[3]*v44[5] + links[47].m*v44[2]*v44[6];
fnet44[2]=links[47].m*a44[5] + a44[3]*links[47].mcm[1] - a44[1]*links[47].mcm[3] - links[47].mcm[2]*Power(v44[1],2) - links[47].mcm[2]*Power(v44[3],2) + v44[2]*(links[47].mcm[1]*v44[1] + links[47].mcm[3]*v44[3]) + links[47].m*v44[3]*v44[4] - links[47].m*v44[1]*v44[6];
fnet44[3]=links[47].m*a44[6] - a44[2]*links[47].mcm[1] + a44[1]*links[47].mcm[2] - links[47].mcm[3]*Power(v44[1],2) - links[47].mcm[3]*Power(v44[2],2) + (links[47].mcm[1]*v44[1] + links[47].mcm[2]*v44[2])*v44[3] - links[47].m*v44[2]*v44[4] + links[47].m*v44[1]*v44[5];
fnet44[4]=a44[6]*links[47].mcm[2] - a44[5]*links[47].mcm[3] + (-(links[47].mcm[2]*v44[2]) - links[47].mcm[3]*v44[3])*v44[4] + (links[47].mcm[1]*v44[3] + links[47].m*v44[5])*v44[6] + v44[5]*(links[47].mcm[1]*v44[2] - links[47].m*v44[6]) + a44[1]*links[47].inertia[1][1] + a44[2]*links[47].inertia[1][2] + a44[3]*links[47].inertia[1][3] + v44[1]*(links[47].mcm[2]*v44[5] + links[47].mcm[3]*v44[6] - v44[3]*links[47].inertia[1][2] + v44[2]*links[47].inertia[1][3]) + v44[2]*(-(links[47].mcm[1]*v44[5]) - v44[3]*links[47].inertia[2][2] + v44[2]*links[47].inertia[2][3]) + v44[3]*(-(links[47].mcm[1]*v44[6]) - v44[3]*links[47].inertia[2][3] + v44[2]*links[47].inertia[3][3]);
fnet44[5]=-(a44[6]*links[47].mcm[1]) + a44[4]*links[47].mcm[3] + (-(links[47].mcm[1]*v44[1]) - links[47].mcm[3]*v44[3])*v44[5] + (links[47].mcm[2]*v44[3] - links[47].m*v44[4])*v44[6] + v44[4]*(links[47].mcm[2]*v44[1] + links[47].m*v44[6]) + a44[1]*links[47].inertia[1][2] + v44[1]*(-(links[47].mcm[2]*v44[4]) + v44[3]*links[47].inertia[1][1] - v44[1]*links[47].inertia[1][3]) + a44[2]*links[47].inertia[2][2] + a44[3]*links[47].inertia[2][3] + v44[2]*(links[47].mcm[1]*v44[4] + links[47].mcm[3]*v44[6] + v44[3]*links[47].inertia[1][2] - v44[1]*links[47].inertia[2][3]) + v44[3]*(-(links[47].mcm[2]*v44[6]) + v44[3]*links[47].inertia[1][3] - v44[1]*links[47].inertia[3][3]);
fnet44[6]=a44[5]*links[47].mcm[1] - a44[4]*links[47].mcm[2] + (links[47].mcm[3]*v44[2] + links[47].m*v44[4])*v44[5] + v44[4]*(links[47].mcm[3]*v44[1] - links[47].m*v44[5]) + (-(links[47].mcm[1]*v44[1]) - links[47].mcm[2]*v44[2])*v44[6] + v44[1]*(-(links[47].mcm[3]*v44[4]) - v44[2]*links[47].inertia[1][1] + v44[1]*links[47].inertia[1][2]) + a44[1]*links[47].inertia[1][3] + v44[2]*(-(links[47].mcm[3]*v44[5]) - v44[2]*links[47].inertia[1][2] + v44[1]*links[47].inertia[2][2]) + a44[2]*links[47].inertia[2][3] + v44[3]*(links[47].mcm[1]*v44[4] + links[47].mcm[2]*v44[5] - v44[2]*links[47].inertia[1][3] + v44[1]*links[47].inertia[2][3]) + a44[3]*links[47].inertia[3][3];

fnet48[1]=links[48].m*a48[4] - a48[3]*links[48].mcm[2] + a48[2]*links[48].mcm[3] - links[48].mcm[1]*Power(v48[2],2) - links[48].mcm[1]*Power(v48[3],2) + v48[1]*(links[48].mcm[2]*v48[2] + links[48].mcm[3]*v48[3]) - links[48].m*v48[3]*v48[5] + links[48].m*v48[2]*v48[6];
fnet48[2]=links[48].m*a48[5] + a48[3]*links[48].mcm[1] - a48[1]*links[48].mcm[3] - links[48].mcm[2]*Power(v48[1],2) - links[48].mcm[2]*Power(v48[3],2) + v48[2]*(links[48].mcm[1]*v48[1] + links[48].mcm[3]*v48[3]) + links[48].m*v48[3]*v48[4] - links[48].m*v48[1]*v48[6];
fnet48[3]=links[48].m*a48[6] - a48[2]*links[48].mcm[1] + a48[1]*links[48].mcm[2] - links[48].mcm[3]*Power(v48[1],2) - links[48].mcm[3]*Power(v48[2],2) + (links[48].mcm[1]*v48[1] + links[48].mcm[2]*v48[2])*v48[3] - links[48].m*v48[2]*v48[4] + links[48].m*v48[1]*v48[5];
fnet48[4]=a48[6]*links[48].mcm[2] - a48[5]*links[48].mcm[3] + (-(links[48].mcm[2]*v48[2]) - links[48].mcm[3]*v48[3])*v48[4] + (links[48].mcm[1]*v48[3] + links[48].m*v48[5])*v48[6] + v48[5]*(links[48].mcm[1]*v48[2] - links[48].m*v48[6]) + a48[1]*links[48].inertia[1][1] + a48[2]*links[48].inertia[1][2] + a48[3]*links[48].inertia[1][3] + v48[1]*(links[48].mcm[2]*v48[5] + links[48].mcm[3]*v48[6] - v48[3]*links[48].inertia[1][2] + v48[2]*links[48].inertia[1][3]) + v48[2]*(-(links[48].mcm[1]*v48[5]) - v48[3]*links[48].inertia[2][2] + v48[2]*links[48].inertia[2][3]) + v48[3]*(-(links[48].mcm[1]*v48[6]) - v48[3]*links[48].inertia[2][3] + v48[2]*links[48].inertia[3][3]);
fnet48[5]=-(a48[6]*links[48].mcm[1]) + a48[4]*links[48].mcm[3] + (-(links[48].mcm[1]*v48[1]) - links[48].mcm[3]*v48[3])*v48[5] + (links[48].mcm[2]*v48[3] - links[48].m*v48[4])*v48[6] + v48[4]*(links[48].mcm[2]*v48[1] + links[48].m*v48[6]) + a48[1]*links[48].inertia[1][2] + v48[1]*(-(links[48].mcm[2]*v48[4]) + v48[3]*links[48].inertia[1][1] - v48[1]*links[48].inertia[1][3]) + a48[2]*links[48].inertia[2][2] + a48[3]*links[48].inertia[2][3] + v48[2]*(links[48].mcm[1]*v48[4] + links[48].mcm[3]*v48[6] + v48[3]*links[48].inertia[1][2] - v48[1]*links[48].inertia[2][3]) + v48[3]*(-(links[48].mcm[2]*v48[6]) + v48[3]*links[48].inertia[1][3] - v48[1]*links[48].inertia[3][3]);
fnet48[6]=a48[5]*links[48].mcm[1] - a48[4]*links[48].mcm[2] + (links[48].mcm[3]*v48[2] + links[48].m*v48[4])*v48[5] + v48[4]*(links[48].mcm[3]*v48[1] - links[48].m*v48[5]) + (-(links[48].mcm[1]*v48[1]) - links[48].mcm[2]*v48[2])*v48[6] + v48[1]*(-(links[48].mcm[3]*v48[4]) - v48[2]*links[48].inertia[1][1] + v48[1]*links[48].inertia[1][2]) + a48[1]*links[48].inertia[1][3] + v48[2]*(-(links[48].mcm[3]*v48[5]) - v48[2]*links[48].inertia[1][2] + v48[1]*links[48].inertia[2][2]) + a48[2]*links[48].inertia[2][3] + v48[3]*(links[48].mcm[1]*v48[4] + links[48].mcm[2]*v48[5] - v48[2]*links[48].inertia[1][3] + v48[1]*links[48].inertia[2][3]) + a48[3]*links[48].inertia[3][3];

fnet52[1]=links[49].m*a52[4] - a52[3]*links[49].mcm[2] + a52[2]*links[49].mcm[3] - links[49].mcm[1]*Power(v52[2],2) - links[49].mcm[1]*Power(v52[3],2) + v52[1]*(links[49].mcm[2]*v52[2] + links[49].mcm[3]*v52[3]) - links[49].m*v52[3]*v52[5] + links[49].m*v52[2]*v52[6];
fnet52[2]=links[49].m*a52[5] + a52[3]*links[49].mcm[1] - a52[1]*links[49].mcm[3] - links[49].mcm[2]*Power(v52[1],2) - links[49].mcm[2]*Power(v52[3],2) + v52[2]*(links[49].mcm[1]*v52[1] + links[49].mcm[3]*v52[3]) + links[49].m*v52[3]*v52[4] - links[49].m*v52[1]*v52[6];
fnet52[3]=links[49].m*a52[6] - a52[2]*links[49].mcm[1] + a52[1]*links[49].mcm[2] - links[49].mcm[3]*Power(v52[1],2) - links[49].mcm[3]*Power(v52[2],2) + (links[49].mcm[1]*v52[1] + links[49].mcm[2]*v52[2])*v52[3] - links[49].m*v52[2]*v52[4] + links[49].m*v52[1]*v52[5];
fnet52[4]=a52[6]*links[49].mcm[2] - a52[5]*links[49].mcm[3] + (-(links[49].mcm[2]*v52[2]) - links[49].mcm[3]*v52[3])*v52[4] + (links[49].mcm[1]*v52[3] + links[49].m*v52[5])*v52[6] + v52[5]*(links[49].mcm[1]*v52[2] - links[49].m*v52[6]) + a52[1]*links[49].inertia[1][1] + a52[2]*links[49].inertia[1][2] + a52[3]*links[49].inertia[1][3] + v52[1]*(links[49].mcm[2]*v52[5] + links[49].mcm[3]*v52[6] - v52[3]*links[49].inertia[1][2] + v52[2]*links[49].inertia[1][3]) + v52[2]*(-(links[49].mcm[1]*v52[5]) - v52[3]*links[49].inertia[2][2] + v52[2]*links[49].inertia[2][3]) + v52[3]*(-(links[49].mcm[1]*v52[6]) - v52[3]*links[49].inertia[2][3] + v52[2]*links[49].inertia[3][3]);
fnet52[5]=-(a52[6]*links[49].mcm[1]) + a52[4]*links[49].mcm[3] + (-(links[49].mcm[1]*v52[1]) - links[49].mcm[3]*v52[3])*v52[5] + (links[49].mcm[2]*v52[3] - links[49].m*v52[4])*v52[6] + v52[4]*(links[49].mcm[2]*v52[1] + links[49].m*v52[6]) + a52[1]*links[49].inertia[1][2] + v52[1]*(-(links[49].mcm[2]*v52[4]) + v52[3]*links[49].inertia[1][1] - v52[1]*links[49].inertia[1][3]) + a52[2]*links[49].inertia[2][2] + a52[3]*links[49].inertia[2][3] + v52[2]*(links[49].mcm[1]*v52[4] + links[49].mcm[3]*v52[6] + v52[3]*links[49].inertia[1][2] - v52[1]*links[49].inertia[2][3]) + v52[3]*(-(links[49].mcm[2]*v52[6]) + v52[3]*links[49].inertia[1][3] - v52[1]*links[49].inertia[3][3]);
fnet52[6]=a52[5]*links[49].mcm[1] - a52[4]*links[49].mcm[2] + (links[49].mcm[3]*v52[2] + links[49].m*v52[4])*v52[5] + v52[4]*(links[49].mcm[3]*v52[1] - links[49].m*v52[5]) + (-(links[49].mcm[1]*v52[1]) - links[49].mcm[2]*v52[2])*v52[6] + v52[1]*(-(links[49].mcm[3]*v52[4]) - v52[2]*links[49].inertia[1][1] + v52[1]*links[49].inertia[1][2]) + a52[1]*links[49].inertia[1][3] + v52[2]*(-(links[49].mcm[3]*v52[5]) - v52[2]*links[49].inertia[1][2] + v52[1]*links[49].inertia[2][2]) + a52[2]*links[49].inertia[2][3] + v52[3]*(links[49].mcm[1]*v52[4] + links[49].mcm[2]*v52[5] - v52[2]*links[49].inertia[1][3] + v52[1]*links[49].inertia[2][3]) + a52[3]*links[49].inertia[3][3];

fnet56[1]=links[50].m*a56[4] - a56[3]*links[50].mcm[2] + a56[2]*links[50].mcm[3] - links[50].mcm[1]*Power(v56[2],2) - links[50].mcm[1]*Power(v56[3],2) + v56[1]*(links[50].mcm[2]*v56[2] + links[50].mcm[3]*v56[3]) - links[50].m*v56[3]*v56[5] + links[50].m*v56[2]*v56[6];
fnet56[2]=links[50].m*a56[5] + a56[3]*links[50].mcm[1] - a56[1]*links[50].mcm[3] - links[50].mcm[2]*Power(v56[1],2) - links[50].mcm[2]*Power(v56[3],2) + v56[2]*(links[50].mcm[1]*v56[1] + links[50].mcm[3]*v56[3]) + links[50].m*v56[3]*v56[4] - links[50].m*v56[1]*v56[6];
fnet56[3]=links[50].m*a56[6] - a56[2]*links[50].mcm[1] + a56[1]*links[50].mcm[2] - links[50].mcm[3]*Power(v56[1],2) - links[50].mcm[3]*Power(v56[2],2) + (links[50].mcm[1]*v56[1] + links[50].mcm[2]*v56[2])*v56[3] - links[50].m*v56[2]*v56[4] + links[50].m*v56[1]*v56[5];
fnet56[4]=a56[6]*links[50].mcm[2] - a56[5]*links[50].mcm[3] + (-(links[50].mcm[2]*v56[2]) - links[50].mcm[3]*v56[3])*v56[4] + (links[50].mcm[1]*v56[3] + links[50].m*v56[5])*v56[6] + v56[5]*(links[50].mcm[1]*v56[2] - links[50].m*v56[6]) + a56[1]*links[50].inertia[1][1] + a56[2]*links[50].inertia[1][2] + a56[3]*links[50].inertia[1][3] + v56[1]*(links[50].mcm[2]*v56[5] + links[50].mcm[3]*v56[6] - v56[3]*links[50].inertia[1][2] + v56[2]*links[50].inertia[1][3]) + v56[2]*(-(links[50].mcm[1]*v56[5]) - v56[3]*links[50].inertia[2][2] + v56[2]*links[50].inertia[2][3]) + v56[3]*(-(links[50].mcm[1]*v56[6]) - v56[3]*links[50].inertia[2][3] + v56[2]*links[50].inertia[3][3]);
fnet56[5]=-(a56[6]*links[50].mcm[1]) + a56[4]*links[50].mcm[3] + (-(links[50].mcm[1]*v56[1]) - links[50].mcm[3]*v56[3])*v56[5] + (links[50].mcm[2]*v56[3] - links[50].m*v56[4])*v56[6] + v56[4]*(links[50].mcm[2]*v56[1] + links[50].m*v56[6]) + a56[1]*links[50].inertia[1][2] + v56[1]*(-(links[50].mcm[2]*v56[4]) + v56[3]*links[50].inertia[1][1] - v56[1]*links[50].inertia[1][3]) + a56[2]*links[50].inertia[2][2] + a56[3]*links[50].inertia[2][3] + v56[2]*(links[50].mcm[1]*v56[4] + links[50].mcm[3]*v56[6] + v56[3]*links[50].inertia[1][2] - v56[1]*links[50].inertia[2][3]) + v56[3]*(-(links[50].mcm[2]*v56[6]) + v56[3]*links[50].inertia[1][3] - v56[1]*links[50].inertia[3][3]);
fnet56[6]=a56[5]*links[50].mcm[1] - a56[4]*links[50].mcm[2] + (links[50].mcm[3]*v56[2] + links[50].m*v56[4])*v56[5] + v56[4]*(links[50].mcm[3]*v56[1] - links[50].m*v56[5]) + (-(links[50].mcm[1]*v56[1]) - links[50].mcm[2]*v56[2])*v56[6] + v56[1]*(-(links[50].mcm[3]*v56[4]) - v56[2]*links[50].inertia[1][1] + v56[1]*links[50].inertia[1][2]) + a56[1]*links[50].inertia[1][3] + v56[2]*(-(links[50].mcm[3]*v56[5]) - v56[2]*links[50].inertia[1][2] + v56[1]*links[50].inertia[2][2]) + a56[2]*links[50].inertia[2][3] + v56[3]*(links[50].mcm[1]*v56[4] + links[50].mcm[2]*v56[5] - v56[2]*links[50].inertia[1][3] + v56[1]*links[50].inertia[2][3]) + a56[3]*links[50].inertia[3][3];

fnet60[1]=links[32].m*a60[4] - a60[3]*links[32].mcm[2] + a60[2]*links[32].mcm[3] - links[32].mcm[1]*Power(v60[2],2) - links[32].mcm[1]*Power(v60[3],2) + v60[1]*(links[32].mcm[2]*v60[2] + links[32].mcm[3]*v60[3]) - links[32].m*v60[3]*v60[5] + links[32].m*v60[2]*v60[6];
fnet60[2]=links[32].m*a60[5] + a60[3]*links[32].mcm[1] - a60[1]*links[32].mcm[3] - links[32].mcm[2]*Power(v60[1],2) - links[32].mcm[2]*Power(v60[3],2) + v60[2]*(links[32].mcm[1]*v60[1] + links[32].mcm[3]*v60[3]) + links[32].m*v60[3]*v60[4] - links[32].m*v60[1]*v60[6];
fnet60[3]=links[32].m*a60[6] - a60[2]*links[32].mcm[1] + a60[1]*links[32].mcm[2] - links[32].mcm[3]*Power(v60[1],2) - links[32].mcm[3]*Power(v60[2],2) + (links[32].mcm[1]*v60[1] + links[32].mcm[2]*v60[2])*v60[3] - links[32].m*v60[2]*v60[4] + links[32].m*v60[1]*v60[5];
fnet60[4]=a60[6]*links[32].mcm[2] - a60[5]*links[32].mcm[3] + (-(links[32].mcm[2]*v60[2]) - links[32].mcm[3]*v60[3])*v60[4] + (links[32].mcm[1]*v60[3] + links[32].m*v60[5])*v60[6] + v60[5]*(links[32].mcm[1]*v60[2] - links[32].m*v60[6]) + a60[1]*links[32].inertia[1][1] + a60[2]*links[32].inertia[1][2] + a60[3]*links[32].inertia[1][3] + v60[1]*(links[32].mcm[2]*v60[5] + links[32].mcm[3]*v60[6] - v60[3]*links[32].inertia[1][2] + v60[2]*links[32].inertia[1][3]) + v60[2]*(-(links[32].mcm[1]*v60[5]) - v60[3]*links[32].inertia[2][2] + v60[2]*links[32].inertia[2][3]) + v60[3]*(-(links[32].mcm[1]*v60[6]) - v60[3]*links[32].inertia[2][3] + v60[2]*links[32].inertia[3][3]);
fnet60[5]=-(a60[6]*links[32].mcm[1]) + a60[4]*links[32].mcm[3] + (-(links[32].mcm[1]*v60[1]) - links[32].mcm[3]*v60[3])*v60[5] + (links[32].mcm[2]*v60[3] - links[32].m*v60[4])*v60[6] + v60[4]*(links[32].mcm[2]*v60[1] + links[32].m*v60[6]) + a60[1]*links[32].inertia[1][2] + v60[1]*(-(links[32].mcm[2]*v60[4]) + v60[3]*links[32].inertia[1][1] - v60[1]*links[32].inertia[1][3]) + a60[2]*links[32].inertia[2][2] + a60[3]*links[32].inertia[2][3] + v60[2]*(links[32].mcm[1]*v60[4] + links[32].mcm[3]*v60[6] + v60[3]*links[32].inertia[1][2] - v60[1]*links[32].inertia[2][3]) + v60[3]*(-(links[32].mcm[2]*v60[6]) + v60[3]*links[32].inertia[1][3] - v60[1]*links[32].inertia[3][3]);
fnet60[6]=a60[5]*links[32].mcm[1] - a60[4]*links[32].mcm[2] + (links[32].mcm[3]*v60[2] + links[32].m*v60[4])*v60[5] + v60[4]*(links[32].mcm[3]*v60[1] - links[32].m*v60[5]) + (-(links[32].mcm[1]*v60[1]) - links[32].mcm[2]*v60[2])*v60[6] + v60[1]*(-(links[32].mcm[3]*v60[4]) - v60[2]*links[32].inertia[1][1] + v60[1]*links[32].inertia[1][2]) + a60[1]*links[32].inertia[1][3] + v60[2]*(-(links[32].mcm[3]*v60[5]) - v60[2]*links[32].inertia[1][2] + v60[1]*links[32].inertia[2][2]) + a60[2]*links[32].inertia[2][3] + v60[3]*(links[32].mcm[1]*v60[4] + links[32].mcm[2]*v60[5] - v60[2]*links[32].inertia[1][3] + v60[1]*links[32].inertia[2][3]) + a60[3]*links[32].inertia[3][3];

fnet61[1]=links[33].m*a61[4] - a61[3]*links[33].mcm[2] + a61[2]*links[33].mcm[3] - links[33].mcm[1]*Power(v61[2],2) - links[33].mcm[1]*Power(v61[3],2) + v61[1]*(links[33].mcm[2]*v61[2] + links[33].mcm[3]*v61[3]) - links[33].m*v61[3]*v61[5] + links[33].m*v61[2]*v61[6];
fnet61[2]=links[33].m*a61[5] + a61[3]*links[33].mcm[1] - a61[1]*links[33].mcm[3] - links[33].mcm[2]*Power(v61[1],2) - links[33].mcm[2]*Power(v61[3],2) + v61[2]*(links[33].mcm[1]*v61[1] + links[33].mcm[3]*v61[3]) + links[33].m*v61[3]*v61[4] - links[33].m*v61[1]*v61[6];
fnet61[3]=links[33].m*a61[6] - a61[2]*links[33].mcm[1] + a61[1]*links[33].mcm[2] - links[33].mcm[3]*Power(v61[1],2) - links[33].mcm[3]*Power(v61[2],2) + (links[33].mcm[1]*v61[1] + links[33].mcm[2]*v61[2])*v61[3] - links[33].m*v61[2]*v61[4] + links[33].m*v61[1]*v61[5];
fnet61[4]=a61[6]*links[33].mcm[2] - a61[5]*links[33].mcm[3] + (-(links[33].mcm[2]*v61[2]) - links[33].mcm[3]*v61[3])*v61[4] + (links[33].mcm[1]*v61[3] + links[33].m*v61[5])*v61[6] + v61[5]*(links[33].mcm[1]*v61[2] - links[33].m*v61[6]) + a61[1]*links[33].inertia[1][1] + a61[2]*links[33].inertia[1][2] + a61[3]*links[33].inertia[1][3] + v61[1]*(links[33].mcm[2]*v61[5] + links[33].mcm[3]*v61[6] - v61[3]*links[33].inertia[1][2] + v61[2]*links[33].inertia[1][3]) + v61[2]*(-(links[33].mcm[1]*v61[5]) - v61[3]*links[33].inertia[2][2] + v61[2]*links[33].inertia[2][3]) + v61[3]*(-(links[33].mcm[1]*v61[6]) - v61[3]*links[33].inertia[2][3] + v61[2]*links[33].inertia[3][3]);
fnet61[5]=-(a61[6]*links[33].mcm[1]) + a61[4]*links[33].mcm[3] + (-(links[33].mcm[1]*v61[1]) - links[33].mcm[3]*v61[3])*v61[5] + (links[33].mcm[2]*v61[3] - links[33].m*v61[4])*v61[6] + v61[4]*(links[33].mcm[2]*v61[1] + links[33].m*v61[6]) + a61[1]*links[33].inertia[1][2] + v61[1]*(-(links[33].mcm[2]*v61[4]) + v61[3]*links[33].inertia[1][1] - v61[1]*links[33].inertia[1][3]) + a61[2]*links[33].inertia[2][2] + a61[3]*links[33].inertia[2][3] + v61[2]*(links[33].mcm[1]*v61[4] + links[33].mcm[3]*v61[6] + v61[3]*links[33].inertia[1][2] - v61[1]*links[33].inertia[2][3]) + v61[3]*(-(links[33].mcm[2]*v61[6]) + v61[3]*links[33].inertia[1][3] - v61[1]*links[33].inertia[3][3]);
fnet61[6]=a61[5]*links[33].mcm[1] - a61[4]*links[33].mcm[2] + (links[33].mcm[3]*v61[2] + links[33].m*v61[4])*v61[5] + v61[4]*(links[33].mcm[3]*v61[1] - links[33].m*v61[5]) + (-(links[33].mcm[1]*v61[1]) - links[33].mcm[2]*v61[2])*v61[6] + v61[1]*(-(links[33].mcm[3]*v61[4]) - v61[2]*links[33].inertia[1][1] + v61[1]*links[33].inertia[1][2]) + a61[1]*links[33].inertia[1][3] + v61[2]*(-(links[33].mcm[3]*v61[5]) - v61[2]*links[33].inertia[1][2] + v61[1]*links[33].inertia[2][2]) + a61[2]*links[33].inertia[2][3] + v61[3]*(links[33].mcm[1]*v61[4] + links[33].mcm[2]*v61[5] - v61[2]*links[33].inertia[1][3] + v61[1]*links[33].inertia[2][3]) + a61[3]*links[33].inertia[3][3];

fnet62[1]=links[34].m*a62[4] - a62[3]*links[34].mcm[2] + a62[2]*links[34].mcm[3] - links[34].mcm[1]*Power(v62[2],2) - links[34].mcm[1]*Power(v62[3],2) + v62[1]*(links[34].mcm[2]*v62[2] + links[34].mcm[3]*v62[3]) - links[34].m*v62[3]*v62[5] + links[34].m*v62[2]*v62[6];
fnet62[2]=links[34].m*a62[5] + a62[3]*links[34].mcm[1] - a62[1]*links[34].mcm[3] - links[34].mcm[2]*Power(v62[1],2) - links[34].mcm[2]*Power(v62[3],2) + v62[2]*(links[34].mcm[1]*v62[1] + links[34].mcm[3]*v62[3]) + links[34].m*v62[3]*v62[4] - links[34].m*v62[1]*v62[6];
fnet62[3]=links[34].m*a62[6] - a62[2]*links[34].mcm[1] + a62[1]*links[34].mcm[2] - links[34].mcm[3]*Power(v62[1],2) - links[34].mcm[3]*Power(v62[2],2) + (links[34].mcm[1]*v62[1] + links[34].mcm[2]*v62[2])*v62[3] - links[34].m*v62[2]*v62[4] + links[34].m*v62[1]*v62[5];
fnet62[4]=a62[6]*links[34].mcm[2] - a62[5]*links[34].mcm[3] + (-(links[34].mcm[2]*v62[2]) - links[34].mcm[3]*v62[3])*v62[4] + (links[34].mcm[1]*v62[3] + links[34].m*v62[5])*v62[6] + v62[5]*(links[34].mcm[1]*v62[2] - links[34].m*v62[6]) + a62[1]*links[34].inertia[1][1] + a62[2]*links[34].inertia[1][2] + a62[3]*links[34].inertia[1][3] + v62[1]*(links[34].mcm[2]*v62[5] + links[34].mcm[3]*v62[6] - v62[3]*links[34].inertia[1][2] + v62[2]*links[34].inertia[1][3]) + v62[2]*(-(links[34].mcm[1]*v62[5]) - v62[3]*links[34].inertia[2][2] + v62[2]*links[34].inertia[2][3]) + v62[3]*(-(links[34].mcm[1]*v62[6]) - v62[3]*links[34].inertia[2][3] + v62[2]*links[34].inertia[3][3]);
fnet62[5]=-(a62[6]*links[34].mcm[1]) + a62[4]*links[34].mcm[3] + (-(links[34].mcm[1]*v62[1]) - links[34].mcm[3]*v62[3])*v62[5] + (links[34].mcm[2]*v62[3] - links[34].m*v62[4])*v62[6] + v62[4]*(links[34].mcm[2]*v62[1] + links[34].m*v62[6]) + a62[1]*links[34].inertia[1][2] + v62[1]*(-(links[34].mcm[2]*v62[4]) + v62[3]*links[34].inertia[1][1] - v62[1]*links[34].inertia[1][3]) + a62[2]*links[34].inertia[2][2] + a62[3]*links[34].inertia[2][3] + v62[2]*(links[34].mcm[1]*v62[4] + links[34].mcm[3]*v62[6] + v62[3]*links[34].inertia[1][2] - v62[1]*links[34].inertia[2][3]) + v62[3]*(-(links[34].mcm[2]*v62[6]) + v62[3]*links[34].inertia[1][3] - v62[1]*links[34].inertia[3][3]);
fnet62[6]=a62[5]*links[34].mcm[1] - a62[4]*links[34].mcm[2] + (links[34].mcm[3]*v62[2] + links[34].m*v62[4])*v62[5] + v62[4]*(links[34].mcm[3]*v62[1] - links[34].m*v62[5]) + (-(links[34].mcm[1]*v62[1]) - links[34].mcm[2]*v62[2])*v62[6] + v62[1]*(-(links[34].mcm[3]*v62[4]) - v62[2]*links[34].inertia[1][1] + v62[1]*links[34].inertia[1][2]) + a62[1]*links[34].inertia[1][3] + v62[2]*(-(links[34].mcm[3]*v62[5]) - v62[2]*links[34].inertia[1][2] + v62[1]*links[34].inertia[2][2]) + a62[2]*links[34].inertia[2][3] + v62[3]*(links[34].mcm[1]*v62[4] + links[34].mcm[2]*v62[5] - v62[2]*links[34].inertia[1][3] + v62[1]*links[34].inertia[2][3]) + a62[3]*links[34].inertia[3][3];

fnet63[1]=links[35].m*a63[4] - a63[3]*links[35].mcm[2] + a63[2]*links[35].mcm[3] - links[35].mcm[1]*Power(v63[2],2) - links[35].mcm[1]*Power(v63[3],2) + v63[1]*(links[35].mcm[2]*v63[2] + links[35].mcm[3]*v63[3]) - links[35].m*v63[3]*v63[5] + links[35].m*v63[2]*v63[6];
fnet63[2]=links[35].m*a63[5] + a63[3]*links[35].mcm[1] - a63[1]*links[35].mcm[3] - links[35].mcm[2]*Power(v63[1],2) - links[35].mcm[2]*Power(v63[3],2) + v63[2]*(links[35].mcm[1]*v63[1] + links[35].mcm[3]*v63[3]) + links[35].m*v63[3]*v63[4] - links[35].m*v63[1]*v63[6];
fnet63[3]=links[35].m*a63[6] - a63[2]*links[35].mcm[1] + a63[1]*links[35].mcm[2] - links[35].mcm[3]*Power(v63[1],2) - links[35].mcm[3]*Power(v63[2],2) + (links[35].mcm[1]*v63[1] + links[35].mcm[2]*v63[2])*v63[3] - links[35].m*v63[2]*v63[4] + links[35].m*v63[1]*v63[5];
fnet63[4]=a63[6]*links[35].mcm[2] - a63[5]*links[35].mcm[3] + (-(links[35].mcm[2]*v63[2]) - links[35].mcm[3]*v63[3])*v63[4] + (links[35].mcm[1]*v63[3] + links[35].m*v63[5])*v63[6] + v63[5]*(links[35].mcm[1]*v63[2] - links[35].m*v63[6]) + a63[1]*links[35].inertia[1][1] + a63[2]*links[35].inertia[1][2] + a63[3]*links[35].inertia[1][3] + v63[1]*(links[35].mcm[2]*v63[5] + links[35].mcm[3]*v63[6] - v63[3]*links[35].inertia[1][2] + v63[2]*links[35].inertia[1][3]) + v63[2]*(-(links[35].mcm[1]*v63[5]) - v63[3]*links[35].inertia[2][2] + v63[2]*links[35].inertia[2][3]) + v63[3]*(-(links[35].mcm[1]*v63[6]) - v63[3]*links[35].inertia[2][3] + v63[2]*links[35].inertia[3][3]);
fnet63[5]=-(a63[6]*links[35].mcm[1]) + a63[4]*links[35].mcm[3] + (-(links[35].mcm[1]*v63[1]) - links[35].mcm[3]*v63[3])*v63[5] + (links[35].mcm[2]*v63[3] - links[35].m*v63[4])*v63[6] + v63[4]*(links[35].mcm[2]*v63[1] + links[35].m*v63[6]) + a63[1]*links[35].inertia[1][2] + v63[1]*(-(links[35].mcm[2]*v63[4]) + v63[3]*links[35].inertia[1][1] - v63[1]*links[35].inertia[1][3]) + a63[2]*links[35].inertia[2][2] + a63[3]*links[35].inertia[2][3] + v63[2]*(links[35].mcm[1]*v63[4] + links[35].mcm[3]*v63[6] + v63[3]*links[35].inertia[1][2] - v63[1]*links[35].inertia[2][3]) + v63[3]*(-(links[35].mcm[2]*v63[6]) + v63[3]*links[35].inertia[1][3] - v63[1]*links[35].inertia[3][3]);
fnet63[6]=a63[5]*links[35].mcm[1] - a63[4]*links[35].mcm[2] + (links[35].mcm[3]*v63[2] + links[35].m*v63[4])*v63[5] + v63[4]*(links[35].mcm[3]*v63[1] - links[35].m*v63[5]) + (-(links[35].mcm[1]*v63[1]) - links[35].mcm[2]*v63[2])*v63[6] + v63[1]*(-(links[35].mcm[3]*v63[4]) - v63[2]*links[35].inertia[1][1] + v63[1]*links[35].inertia[1][2]) + a63[1]*links[35].inertia[1][3] + v63[2]*(-(links[35].mcm[3]*v63[5]) - v63[2]*links[35].inertia[1][2] + v63[1]*links[35].inertia[2][2]) + a63[2]*links[35].inertia[2][3] + v63[3]*(links[35].mcm[1]*v63[4] + links[35].mcm[2]*v63[5] - v63[2]*links[35].inertia[1][3] + v63[1]*links[35].inertia[2][3]) + a63[3]*links[35].inertia[3][3];

fnet64[1]=links[36].m*a64[4] - a64[3]*links[36].mcm[2] + a64[2]*links[36].mcm[3] - links[36].mcm[1]*Power(v64[2],2) - links[36].mcm[1]*Power(v64[3],2) + v64[1]*(links[36].mcm[2]*v64[2] + links[36].mcm[3]*v64[3]) - links[36].m*v64[3]*v64[5] + links[36].m*v64[2]*v64[6];
fnet64[2]=links[36].m*a64[5] + a64[3]*links[36].mcm[1] - a64[1]*links[36].mcm[3] - links[36].mcm[2]*Power(v64[1],2) - links[36].mcm[2]*Power(v64[3],2) + v64[2]*(links[36].mcm[1]*v64[1] + links[36].mcm[3]*v64[3]) + links[36].m*v64[3]*v64[4] - links[36].m*v64[1]*v64[6];
fnet64[3]=links[36].m*a64[6] - a64[2]*links[36].mcm[1] + a64[1]*links[36].mcm[2] - links[36].mcm[3]*Power(v64[1],2) - links[36].mcm[3]*Power(v64[2],2) + (links[36].mcm[1]*v64[1] + links[36].mcm[2]*v64[2])*v64[3] - links[36].m*v64[2]*v64[4] + links[36].m*v64[1]*v64[5];
fnet64[4]=a64[6]*links[36].mcm[2] - a64[5]*links[36].mcm[3] + (-(links[36].mcm[2]*v64[2]) - links[36].mcm[3]*v64[3])*v64[4] + (links[36].mcm[1]*v64[3] + links[36].m*v64[5])*v64[6] + v64[5]*(links[36].mcm[1]*v64[2] - links[36].m*v64[6]) + a64[1]*links[36].inertia[1][1] + a64[2]*links[36].inertia[1][2] + a64[3]*links[36].inertia[1][3] + v64[1]*(links[36].mcm[2]*v64[5] + links[36].mcm[3]*v64[6] - v64[3]*links[36].inertia[1][2] + v64[2]*links[36].inertia[1][3]) + v64[2]*(-(links[36].mcm[1]*v64[5]) - v64[3]*links[36].inertia[2][2] + v64[2]*links[36].inertia[2][3]) + v64[3]*(-(links[36].mcm[1]*v64[6]) - v64[3]*links[36].inertia[2][3] + v64[2]*links[36].inertia[3][3]);
fnet64[5]=-(a64[6]*links[36].mcm[1]) + a64[4]*links[36].mcm[3] + (-(links[36].mcm[1]*v64[1]) - links[36].mcm[3]*v64[3])*v64[5] + (links[36].mcm[2]*v64[3] - links[36].m*v64[4])*v64[6] + v64[4]*(links[36].mcm[2]*v64[1] + links[36].m*v64[6]) + a64[1]*links[36].inertia[1][2] + v64[1]*(-(links[36].mcm[2]*v64[4]) + v64[3]*links[36].inertia[1][1] - v64[1]*links[36].inertia[1][3]) + a64[2]*links[36].inertia[2][2] + a64[3]*links[36].inertia[2][3] + v64[2]*(links[36].mcm[1]*v64[4] + links[36].mcm[3]*v64[6] + v64[3]*links[36].inertia[1][2] - v64[1]*links[36].inertia[2][3]) + v64[3]*(-(links[36].mcm[2]*v64[6]) + v64[3]*links[36].inertia[1][3] - v64[1]*links[36].inertia[3][3]);
fnet64[6]=a64[5]*links[36].mcm[1] - a64[4]*links[36].mcm[2] + (links[36].mcm[3]*v64[2] + links[36].m*v64[4])*v64[5] + v64[4]*(links[36].mcm[3]*v64[1] - links[36].m*v64[5]) + (-(links[36].mcm[1]*v64[1]) - links[36].mcm[2]*v64[2])*v64[6] + v64[1]*(-(links[36].mcm[3]*v64[4]) - v64[2]*links[36].inertia[1][1] + v64[1]*links[36].inertia[1][2]) + a64[1]*links[36].inertia[1][3] + v64[2]*(-(links[36].mcm[3]*v64[5]) - v64[2]*links[36].inertia[1][2] + v64[1]*links[36].inertia[2][2]) + a64[2]*links[36].inertia[2][3] + v64[3]*(links[36].mcm[1]*v64[4] + links[36].mcm[2]*v64[5] - v64[2]*links[36].inertia[1][3] + v64[1]*links[36].inertia[2][3]) + a64[3]*links[36].inertia[3][3];

fnet66[1]=links[37].m*a66[4] - a66[3]*links[37].mcm[2] + a66[2]*links[37].mcm[3] - links[37].mcm[1]*Power(v66[2],2) - links[37].mcm[1]*Power(v66[3],2) + v66[1]*(links[37].mcm[2]*v66[2] + links[37].mcm[3]*v66[3]) - links[37].m*v66[3]*v66[5] + links[37].m*v66[2]*v66[6];
fnet66[2]=links[37].m*a66[5] + a66[3]*links[37].mcm[1] - a66[1]*links[37].mcm[3] - links[37].mcm[2]*Power(v66[1],2) - links[37].mcm[2]*Power(v66[3],2) + v66[2]*(links[37].mcm[1]*v66[1] + links[37].mcm[3]*v66[3]) + links[37].m*v66[3]*v66[4] - links[37].m*v66[1]*v66[6];
fnet66[3]=links[37].m*a66[6] - a66[2]*links[37].mcm[1] + a66[1]*links[37].mcm[2] - links[37].mcm[3]*Power(v66[1],2) - links[37].mcm[3]*Power(v66[2],2) + (links[37].mcm[1]*v66[1] + links[37].mcm[2]*v66[2])*v66[3] - links[37].m*v66[2]*v66[4] + links[37].m*v66[1]*v66[5];
fnet66[4]=a66[6]*links[37].mcm[2] - a66[5]*links[37].mcm[3] + (-(links[37].mcm[2]*v66[2]) - links[37].mcm[3]*v66[3])*v66[4] + (links[37].mcm[1]*v66[3] + links[37].m*v66[5])*v66[6] + v66[5]*(links[37].mcm[1]*v66[2] - links[37].m*v66[6]) + a66[1]*links[37].inertia[1][1] + a66[2]*links[37].inertia[1][2] + a66[3]*links[37].inertia[1][3] + v66[1]*(links[37].mcm[2]*v66[5] + links[37].mcm[3]*v66[6] - v66[3]*links[37].inertia[1][2] + v66[2]*links[37].inertia[1][3]) + v66[2]*(-(links[37].mcm[1]*v66[5]) - v66[3]*links[37].inertia[2][2] + v66[2]*links[37].inertia[2][3]) + v66[3]*(-(links[37].mcm[1]*v66[6]) - v66[3]*links[37].inertia[2][3] + v66[2]*links[37].inertia[3][3]);
fnet66[5]=-(a66[6]*links[37].mcm[1]) + a66[4]*links[37].mcm[3] + (-(links[37].mcm[1]*v66[1]) - links[37].mcm[3]*v66[3])*v66[5] + (links[37].mcm[2]*v66[3] - links[37].m*v66[4])*v66[6] + v66[4]*(links[37].mcm[2]*v66[1] + links[37].m*v66[6]) + a66[1]*links[37].inertia[1][2] + v66[1]*(-(links[37].mcm[2]*v66[4]) + v66[3]*links[37].inertia[1][1] - v66[1]*links[37].inertia[1][3]) + a66[2]*links[37].inertia[2][2] + a66[3]*links[37].inertia[2][3] + v66[2]*(links[37].mcm[1]*v66[4] + links[37].mcm[3]*v66[6] + v66[3]*links[37].inertia[1][2] - v66[1]*links[37].inertia[2][3]) + v66[3]*(-(links[37].mcm[2]*v66[6]) + v66[3]*links[37].inertia[1][3] - v66[1]*links[37].inertia[3][3]);
fnet66[6]=a66[5]*links[37].mcm[1] - a66[4]*links[37].mcm[2] + (links[37].mcm[3]*v66[2] + links[37].m*v66[4])*v66[5] + v66[4]*(links[37].mcm[3]*v66[1] - links[37].m*v66[5]) + (-(links[37].mcm[1]*v66[1]) - links[37].mcm[2]*v66[2])*v66[6] + v66[1]*(-(links[37].mcm[3]*v66[4]) - v66[2]*links[37].inertia[1][1] + v66[1]*links[37].inertia[1][2]) + a66[1]*links[37].inertia[1][3] + v66[2]*(-(links[37].mcm[3]*v66[5]) - v66[2]*links[37].inertia[1][2] + v66[1]*links[37].inertia[2][2]) + a66[2]*links[37].inertia[2][3] + v66[3]*(links[37].mcm[1]*v66[4] + links[37].mcm[2]*v66[5] - v66[2]*links[37].inertia[1][3] + v66[1]*links[37].inertia[2][3]) + a66[3]*links[37].inertia[3][3];

fnet67[1]=links[38].m*a67[4] - a67[3]*links[38].mcm[2] + a67[2]*links[38].mcm[3] - links[38].mcm[1]*Power(v67[2],2) - links[38].mcm[1]*Power(v67[3],2) + v67[1]*(links[38].mcm[2]*v67[2] + links[38].mcm[3]*v67[3]) - links[38].m*v67[3]*v67[5] + links[38].m*v67[2]*v67[6];
fnet67[2]=links[38].m*a67[5] + a67[3]*links[38].mcm[1] - a67[1]*links[38].mcm[3] - links[38].mcm[2]*Power(v67[1],2) - links[38].mcm[2]*Power(v67[3],2) + v67[2]*(links[38].mcm[1]*v67[1] + links[38].mcm[3]*v67[3]) + links[38].m*v67[3]*v67[4] - links[38].m*v67[1]*v67[6];
fnet67[3]=links[38].m*a67[6] - a67[2]*links[38].mcm[1] + a67[1]*links[38].mcm[2] - links[38].mcm[3]*Power(v67[1],2) - links[38].mcm[3]*Power(v67[2],2) + (links[38].mcm[1]*v67[1] + links[38].mcm[2]*v67[2])*v67[3] - links[38].m*v67[2]*v67[4] + links[38].m*v67[1]*v67[5];
fnet67[4]=a67[6]*links[38].mcm[2] - a67[5]*links[38].mcm[3] + (-(links[38].mcm[2]*v67[2]) - links[38].mcm[3]*v67[3])*v67[4] + (links[38].mcm[1]*v67[3] + links[38].m*v67[5])*v67[6] + v67[5]*(links[38].mcm[1]*v67[2] - links[38].m*v67[6]) + a67[1]*links[38].inertia[1][1] + a67[2]*links[38].inertia[1][2] + a67[3]*links[38].inertia[1][3] + v67[1]*(links[38].mcm[2]*v67[5] + links[38].mcm[3]*v67[6] - v67[3]*links[38].inertia[1][2] + v67[2]*links[38].inertia[1][3]) + v67[2]*(-(links[38].mcm[1]*v67[5]) - v67[3]*links[38].inertia[2][2] + v67[2]*links[38].inertia[2][3]) + v67[3]*(-(links[38].mcm[1]*v67[6]) - v67[3]*links[38].inertia[2][3] + v67[2]*links[38].inertia[3][3]);
fnet67[5]=-(a67[6]*links[38].mcm[1]) + a67[4]*links[38].mcm[3] + (-(links[38].mcm[1]*v67[1]) - links[38].mcm[3]*v67[3])*v67[5] + (links[38].mcm[2]*v67[3] - links[38].m*v67[4])*v67[6] + v67[4]*(links[38].mcm[2]*v67[1] + links[38].m*v67[6]) + a67[1]*links[38].inertia[1][2] + v67[1]*(-(links[38].mcm[2]*v67[4]) + v67[3]*links[38].inertia[1][1] - v67[1]*links[38].inertia[1][3]) + a67[2]*links[38].inertia[2][2] + a67[3]*links[38].inertia[2][3] + v67[2]*(links[38].mcm[1]*v67[4] + links[38].mcm[3]*v67[6] + v67[3]*links[38].inertia[1][2] - v67[1]*links[38].inertia[2][3]) + v67[3]*(-(links[38].mcm[2]*v67[6]) + v67[3]*links[38].inertia[1][3] - v67[1]*links[38].inertia[3][3]);
fnet67[6]=a67[5]*links[38].mcm[1] - a67[4]*links[38].mcm[2] + (links[38].mcm[3]*v67[2] + links[38].m*v67[4])*v67[5] + v67[4]*(links[38].mcm[3]*v67[1] - links[38].m*v67[5]) + (-(links[38].mcm[1]*v67[1]) - links[38].mcm[2]*v67[2])*v67[6] + v67[1]*(-(links[38].mcm[3]*v67[4]) - v67[2]*links[38].inertia[1][1] + v67[1]*links[38].inertia[1][2]) + a67[1]*links[38].inertia[1][3] + v67[2]*(-(links[38].mcm[3]*v67[5]) - v67[2]*links[38].inertia[1][2] + v67[1]*links[38].inertia[2][2]) + a67[2]*links[38].inertia[2][3] + v67[3]*(links[38].mcm[1]*v67[4] + links[38].mcm[2]*v67[5] - v67[2]*links[38].inertia[1][3] + v67[1]*links[38].inertia[2][3]) + a67[3]*links[38].inertia[3][3];

fnet70[1]=links[23].m*a70[4] - a70[3]*links[23].mcm[2] + a70[2]*links[23].mcm[3] - links[23].mcm[1]*Power(v70[2],2) - links[23].mcm[1]*Power(v70[3],2) + v70[1]*(links[23].mcm[2]*v70[2] + links[23].mcm[3]*v70[3]) - links[23].m*v70[3]*v70[5] + links[23].m*v70[2]*v70[6];
fnet70[2]=links[23].m*a70[5] + a70[3]*links[23].mcm[1] - a70[1]*links[23].mcm[3] - links[23].mcm[2]*Power(v70[1],2) - links[23].mcm[2]*Power(v70[3],2) + v70[2]*(links[23].mcm[1]*v70[1] + links[23].mcm[3]*v70[3]) + links[23].m*v70[3]*v70[4] - links[23].m*v70[1]*v70[6];
fnet70[3]=links[23].m*a70[6] - a70[2]*links[23].mcm[1] + a70[1]*links[23].mcm[2] - links[23].mcm[3]*Power(v70[1],2) - links[23].mcm[3]*Power(v70[2],2) + (links[23].mcm[1]*v70[1] + links[23].mcm[2]*v70[2])*v70[3] - links[23].m*v70[2]*v70[4] + links[23].m*v70[1]*v70[5];
fnet70[4]=a70[6]*links[23].mcm[2] - a70[5]*links[23].mcm[3] + (-(links[23].mcm[2]*v70[2]) - links[23].mcm[3]*v70[3])*v70[4] + (links[23].mcm[1]*v70[3] + links[23].m*v70[5])*v70[6] + v70[5]*(links[23].mcm[1]*v70[2] - links[23].m*v70[6]) + a70[1]*links[23].inertia[1][1] + a70[2]*links[23].inertia[1][2] + a70[3]*links[23].inertia[1][3] + v70[1]*(links[23].mcm[2]*v70[5] + links[23].mcm[3]*v70[6] - v70[3]*links[23].inertia[1][2] + v70[2]*links[23].inertia[1][3]) + v70[2]*(-(links[23].mcm[1]*v70[5]) - v70[3]*links[23].inertia[2][2] + v70[2]*links[23].inertia[2][3]) + v70[3]*(-(links[23].mcm[1]*v70[6]) - v70[3]*links[23].inertia[2][3] + v70[2]*links[23].inertia[3][3]);
fnet70[5]=-(a70[6]*links[23].mcm[1]) + a70[4]*links[23].mcm[3] + (-(links[23].mcm[1]*v70[1]) - links[23].mcm[3]*v70[3])*v70[5] + (links[23].mcm[2]*v70[3] - links[23].m*v70[4])*v70[6] + v70[4]*(links[23].mcm[2]*v70[1] + links[23].m*v70[6]) + a70[1]*links[23].inertia[1][2] + v70[1]*(-(links[23].mcm[2]*v70[4]) + v70[3]*links[23].inertia[1][1] - v70[1]*links[23].inertia[1][3]) + a70[2]*links[23].inertia[2][2] + a70[3]*links[23].inertia[2][3] + v70[2]*(links[23].mcm[1]*v70[4] + links[23].mcm[3]*v70[6] + v70[3]*links[23].inertia[1][2] - v70[1]*links[23].inertia[2][3]) + v70[3]*(-(links[23].mcm[2]*v70[6]) + v70[3]*links[23].inertia[1][3] - v70[1]*links[23].inertia[3][3]);
fnet70[6]=a70[5]*links[23].mcm[1] - a70[4]*links[23].mcm[2] + (links[23].mcm[3]*v70[2] + links[23].m*v70[4])*v70[5] + v70[4]*(links[23].mcm[3]*v70[1] - links[23].m*v70[5]) + (-(links[23].mcm[1]*v70[1]) - links[23].mcm[2]*v70[2])*v70[6] + v70[1]*(-(links[23].mcm[3]*v70[4]) - v70[2]*links[23].inertia[1][1] + v70[1]*links[23].inertia[1][2]) + a70[1]*links[23].inertia[1][3] + v70[2]*(-(links[23].mcm[3]*v70[5]) - v70[2]*links[23].inertia[1][2] + v70[1]*links[23].inertia[2][2]) + a70[2]*links[23].inertia[2][3] + v70[3]*(links[23].mcm[1]*v70[4] + links[23].mcm[2]*v70[5] - v70[2]*links[23].inertia[1][3] + v70[1]*links[23].inertia[2][3]) + a70[3]*links[23].inertia[3][3];

fnet71[1]=links[22].m*a71[4] - a71[3]*links[22].mcm[2] + a71[2]*links[22].mcm[3] - links[22].mcm[1]*Power(v71[2],2) - links[22].mcm[1]*Power(v71[3],2) + v71[1]*(links[22].mcm[2]*v71[2] + links[22].mcm[3]*v71[3]) - links[22].m*v71[3]*v71[5] + links[22].m*v71[2]*v71[6];
fnet71[2]=links[22].m*a71[5] + a71[3]*links[22].mcm[1] - a71[1]*links[22].mcm[3] - links[22].mcm[2]*Power(v71[1],2) - links[22].mcm[2]*Power(v71[3],2) + v71[2]*(links[22].mcm[1]*v71[1] + links[22].mcm[3]*v71[3]) + links[22].m*v71[3]*v71[4] - links[22].m*v71[1]*v71[6];
fnet71[3]=links[22].m*a71[6] - a71[2]*links[22].mcm[1] + a71[1]*links[22].mcm[2] - links[22].mcm[3]*Power(v71[1],2) - links[22].mcm[3]*Power(v71[2],2) + (links[22].mcm[1]*v71[1] + links[22].mcm[2]*v71[2])*v71[3] - links[22].m*v71[2]*v71[4] + links[22].m*v71[1]*v71[5];
fnet71[4]=a71[6]*links[22].mcm[2] - a71[5]*links[22].mcm[3] + (-(links[22].mcm[2]*v71[2]) - links[22].mcm[3]*v71[3])*v71[4] + (links[22].mcm[1]*v71[3] + links[22].m*v71[5])*v71[6] + v71[5]*(links[22].mcm[1]*v71[2] - links[22].m*v71[6]) + a71[1]*links[22].inertia[1][1] + a71[2]*links[22].inertia[1][2] + a71[3]*links[22].inertia[1][3] + v71[1]*(links[22].mcm[2]*v71[5] + links[22].mcm[3]*v71[6] - v71[3]*links[22].inertia[1][2] + v71[2]*links[22].inertia[1][3]) + v71[2]*(-(links[22].mcm[1]*v71[5]) - v71[3]*links[22].inertia[2][2] + v71[2]*links[22].inertia[2][3]) + v71[3]*(-(links[22].mcm[1]*v71[6]) - v71[3]*links[22].inertia[2][3] + v71[2]*links[22].inertia[3][3]);
fnet71[5]=-(a71[6]*links[22].mcm[1]) + a71[4]*links[22].mcm[3] + (-(links[22].mcm[1]*v71[1]) - links[22].mcm[3]*v71[3])*v71[5] + (links[22].mcm[2]*v71[3] - links[22].m*v71[4])*v71[6] + v71[4]*(links[22].mcm[2]*v71[1] + links[22].m*v71[6]) + a71[1]*links[22].inertia[1][2] + v71[1]*(-(links[22].mcm[2]*v71[4]) + v71[3]*links[22].inertia[1][1] - v71[1]*links[22].inertia[1][3]) + a71[2]*links[22].inertia[2][2] + a71[3]*links[22].inertia[2][3] + v71[2]*(links[22].mcm[1]*v71[4] + links[22].mcm[3]*v71[6] + v71[3]*links[22].inertia[1][2] - v71[1]*links[22].inertia[2][3]) + v71[3]*(-(links[22].mcm[2]*v71[6]) + v71[3]*links[22].inertia[1][3] - v71[1]*links[22].inertia[3][3]);
fnet71[6]=a71[5]*links[22].mcm[1] - a71[4]*links[22].mcm[2] + (links[22].mcm[3]*v71[2] + links[22].m*v71[4])*v71[5] + v71[4]*(links[22].mcm[3]*v71[1] - links[22].m*v71[5]) + (-(links[22].mcm[1]*v71[1]) - links[22].mcm[2]*v71[2])*v71[6] + v71[1]*(-(links[22].mcm[3]*v71[4]) - v71[2]*links[22].inertia[1][1] + v71[1]*links[22].inertia[1][2]) + a71[1]*links[22].inertia[1][3] + v71[2]*(-(links[22].mcm[3]*v71[5]) - v71[2]*links[22].inertia[1][2] + v71[1]*links[22].inertia[2][2]) + a71[2]*links[22].inertia[2][3] + v71[3]*(links[22].mcm[1]*v71[4] + links[22].mcm[2]*v71[5] - v71[2]*links[22].inertia[1][3] + v71[1]*links[22].inertia[2][3]) + a71[3]*links[22].inertia[3][3];

fnet72[1]=links[24].m*a72[4] - a72[3]*links[24].mcm[2] + a72[2]*links[24].mcm[3] - links[24].mcm[1]*Power(v72[2],2) - links[24].mcm[1]*Power(v72[3],2) + v72[1]*(links[24].mcm[2]*v72[2] + links[24].mcm[3]*v72[3]) - links[24].m*v72[3]*v72[5] + links[24].m*v72[2]*v72[6];
fnet72[2]=links[24].m*a72[5] + a72[3]*links[24].mcm[1] - a72[1]*links[24].mcm[3] - links[24].mcm[2]*Power(v72[1],2) - links[24].mcm[2]*Power(v72[3],2) + v72[2]*(links[24].mcm[1]*v72[1] + links[24].mcm[3]*v72[3]) + links[24].m*v72[3]*v72[4] - links[24].m*v72[1]*v72[6];
fnet72[3]=links[24].m*a72[6] - a72[2]*links[24].mcm[1] + a72[1]*links[24].mcm[2] - links[24].mcm[3]*Power(v72[1],2) - links[24].mcm[3]*Power(v72[2],2) + (links[24].mcm[1]*v72[1] + links[24].mcm[2]*v72[2])*v72[3] - links[24].m*v72[2]*v72[4] + links[24].m*v72[1]*v72[5];
fnet72[4]=a72[6]*links[24].mcm[2] - a72[5]*links[24].mcm[3] + (-(links[24].mcm[2]*v72[2]) - links[24].mcm[3]*v72[3])*v72[4] + (links[24].mcm[1]*v72[3] + links[24].m*v72[5])*v72[6] + v72[5]*(links[24].mcm[1]*v72[2] - links[24].m*v72[6]) + a72[1]*links[24].inertia[1][1] + a72[2]*links[24].inertia[1][2] + a72[3]*links[24].inertia[1][3] + v72[1]*(links[24].mcm[2]*v72[5] + links[24].mcm[3]*v72[6] - v72[3]*links[24].inertia[1][2] + v72[2]*links[24].inertia[1][3]) + v72[2]*(-(links[24].mcm[1]*v72[5]) - v72[3]*links[24].inertia[2][2] + v72[2]*links[24].inertia[2][3]) + v72[3]*(-(links[24].mcm[1]*v72[6]) - v72[3]*links[24].inertia[2][3] + v72[2]*links[24].inertia[3][3]);
fnet72[5]=-(a72[6]*links[24].mcm[1]) + a72[4]*links[24].mcm[3] + (-(links[24].mcm[1]*v72[1]) - links[24].mcm[3]*v72[3])*v72[5] + (links[24].mcm[2]*v72[3] - links[24].m*v72[4])*v72[6] + v72[4]*(links[24].mcm[2]*v72[1] + links[24].m*v72[6]) + a72[1]*links[24].inertia[1][2] + v72[1]*(-(links[24].mcm[2]*v72[4]) + v72[3]*links[24].inertia[1][1] - v72[1]*links[24].inertia[1][3]) + a72[2]*links[24].inertia[2][2] + a72[3]*links[24].inertia[2][3] + v72[2]*(links[24].mcm[1]*v72[4] + links[24].mcm[3]*v72[6] + v72[3]*links[24].inertia[1][2] - v72[1]*links[24].inertia[2][3]) + v72[3]*(-(links[24].mcm[2]*v72[6]) + v72[3]*links[24].inertia[1][3] - v72[1]*links[24].inertia[3][3]);
fnet72[6]=a72[5]*links[24].mcm[1] - a72[4]*links[24].mcm[2] + (links[24].mcm[3]*v72[2] + links[24].m*v72[4])*v72[5] + v72[4]*(links[24].mcm[3]*v72[1] - links[24].m*v72[5]) + (-(links[24].mcm[1]*v72[1]) - links[24].mcm[2]*v72[2])*v72[6] + v72[1]*(-(links[24].mcm[3]*v72[4]) - v72[2]*links[24].inertia[1][1] + v72[1]*links[24].inertia[1][2]) + a72[1]*links[24].inertia[1][3] + v72[2]*(-(links[24].mcm[3]*v72[5]) - v72[2]*links[24].inertia[1][2] + v72[1]*links[24].inertia[2][2]) + a72[2]*links[24].inertia[2][3] + v72[3]*(links[24].mcm[1]*v72[4] + links[24].mcm[2]*v72[5] - v72[2]*links[24].inertia[1][3] + v72[1]*links[24].inertia[2][3]) + a72[3]*links[24].inertia[3][3];

fnet73[1]=links[25].m*a73[4] - a73[3]*links[25].mcm[2] + a73[2]*links[25].mcm[3] - links[25].mcm[1]*Power(v73[2],2) - links[25].mcm[1]*Power(v73[3],2) + v73[1]*(links[25].mcm[2]*v73[2] + links[25].mcm[3]*v73[3]) - links[25].m*v73[3]*v73[5] + links[25].m*v73[2]*v73[6];
fnet73[2]=links[25].m*a73[5] + a73[3]*links[25].mcm[1] - a73[1]*links[25].mcm[3] - links[25].mcm[2]*Power(v73[1],2) - links[25].mcm[2]*Power(v73[3],2) + v73[2]*(links[25].mcm[1]*v73[1] + links[25].mcm[3]*v73[3]) + links[25].m*v73[3]*v73[4] - links[25].m*v73[1]*v73[6];
fnet73[3]=links[25].m*a73[6] - a73[2]*links[25].mcm[1] + a73[1]*links[25].mcm[2] - links[25].mcm[3]*Power(v73[1],2) - links[25].mcm[3]*Power(v73[2],2) + (links[25].mcm[1]*v73[1] + links[25].mcm[2]*v73[2])*v73[3] - links[25].m*v73[2]*v73[4] + links[25].m*v73[1]*v73[5];
fnet73[4]=a73[6]*links[25].mcm[2] - a73[5]*links[25].mcm[3] + (-(links[25].mcm[2]*v73[2]) - links[25].mcm[3]*v73[3])*v73[4] + (links[25].mcm[1]*v73[3] + links[25].m*v73[5])*v73[6] + v73[5]*(links[25].mcm[1]*v73[2] - links[25].m*v73[6]) + a73[1]*links[25].inertia[1][1] + a73[2]*links[25].inertia[1][2] + a73[3]*links[25].inertia[1][3] + v73[1]*(links[25].mcm[2]*v73[5] + links[25].mcm[3]*v73[6] - v73[3]*links[25].inertia[1][2] + v73[2]*links[25].inertia[1][3]) + v73[2]*(-(links[25].mcm[1]*v73[5]) - v73[3]*links[25].inertia[2][2] + v73[2]*links[25].inertia[2][3]) + v73[3]*(-(links[25].mcm[1]*v73[6]) - v73[3]*links[25].inertia[2][3] + v73[2]*links[25].inertia[3][3]);
fnet73[5]=-(a73[6]*links[25].mcm[1]) + a73[4]*links[25].mcm[3] + (-(links[25].mcm[1]*v73[1]) - links[25].mcm[3]*v73[3])*v73[5] + (links[25].mcm[2]*v73[3] - links[25].m*v73[4])*v73[6] + v73[4]*(links[25].mcm[2]*v73[1] + links[25].m*v73[6]) + a73[1]*links[25].inertia[1][2] + v73[1]*(-(links[25].mcm[2]*v73[4]) + v73[3]*links[25].inertia[1][1] - v73[1]*links[25].inertia[1][3]) + a73[2]*links[25].inertia[2][2] + a73[3]*links[25].inertia[2][3] + v73[2]*(links[25].mcm[1]*v73[4] + links[25].mcm[3]*v73[6] + v73[3]*links[25].inertia[1][2] - v73[1]*links[25].inertia[2][3]) + v73[3]*(-(links[25].mcm[2]*v73[6]) + v73[3]*links[25].inertia[1][3] - v73[1]*links[25].inertia[3][3]);
fnet73[6]=a73[5]*links[25].mcm[1] - a73[4]*links[25].mcm[2] + (links[25].mcm[3]*v73[2] + links[25].m*v73[4])*v73[5] + v73[4]*(links[25].mcm[3]*v73[1] - links[25].m*v73[5]) + (-(links[25].mcm[1]*v73[1]) - links[25].mcm[2]*v73[2])*v73[6] + v73[1]*(-(links[25].mcm[3]*v73[4]) - v73[2]*links[25].inertia[1][1] + v73[1]*links[25].inertia[1][2]) + a73[1]*links[25].inertia[1][3] + v73[2]*(-(links[25].mcm[3]*v73[5]) - v73[2]*links[25].inertia[1][2] + v73[1]*links[25].inertia[2][2]) + a73[2]*links[25].inertia[2][3] + v73[3]*(links[25].mcm[1]*v73[4] + links[25].mcm[2]*v73[5] - v73[2]*links[25].inertia[1][3] + v73[1]*links[25].inertia[2][3]) + a73[3]*links[25].inertia[3][3];

fnet74[1]=links[26].m*a74[4] - a74[3]*links[26].mcm[2] + a74[2]*links[26].mcm[3] - links[26].mcm[1]*Power(v74[2],2) - links[26].mcm[1]*Power(v74[3],2) + v74[1]*(links[26].mcm[2]*v74[2] + links[26].mcm[3]*v74[3]) - links[26].m*v74[3]*v74[5] + links[26].m*v74[2]*v74[6];
fnet74[2]=links[26].m*a74[5] + a74[3]*links[26].mcm[1] - a74[1]*links[26].mcm[3] - links[26].mcm[2]*Power(v74[1],2) - links[26].mcm[2]*Power(v74[3],2) + v74[2]*(links[26].mcm[1]*v74[1] + links[26].mcm[3]*v74[3]) + links[26].m*v74[3]*v74[4] - links[26].m*v74[1]*v74[6];
fnet74[3]=links[26].m*a74[6] - a74[2]*links[26].mcm[1] + a74[1]*links[26].mcm[2] - links[26].mcm[3]*Power(v74[1],2) - links[26].mcm[3]*Power(v74[2],2) + (links[26].mcm[1]*v74[1] + links[26].mcm[2]*v74[2])*v74[3] - links[26].m*v74[2]*v74[4] + links[26].m*v74[1]*v74[5];
fnet74[4]=a74[6]*links[26].mcm[2] - a74[5]*links[26].mcm[3] + (-(links[26].mcm[2]*v74[2]) - links[26].mcm[3]*v74[3])*v74[4] + (links[26].mcm[1]*v74[3] + links[26].m*v74[5])*v74[6] + v74[5]*(links[26].mcm[1]*v74[2] - links[26].m*v74[6]) + a74[1]*links[26].inertia[1][1] + a74[2]*links[26].inertia[1][2] + a74[3]*links[26].inertia[1][3] + v74[1]*(links[26].mcm[2]*v74[5] + links[26].mcm[3]*v74[6] - v74[3]*links[26].inertia[1][2] + v74[2]*links[26].inertia[1][3]) + v74[2]*(-(links[26].mcm[1]*v74[5]) - v74[3]*links[26].inertia[2][2] + v74[2]*links[26].inertia[2][3]) + v74[3]*(-(links[26].mcm[1]*v74[6]) - v74[3]*links[26].inertia[2][3] + v74[2]*links[26].inertia[3][3]);
fnet74[5]=-(a74[6]*links[26].mcm[1]) + a74[4]*links[26].mcm[3] + (-(links[26].mcm[1]*v74[1]) - links[26].mcm[3]*v74[3])*v74[5] + (links[26].mcm[2]*v74[3] - links[26].m*v74[4])*v74[6] + v74[4]*(links[26].mcm[2]*v74[1] + links[26].m*v74[6]) + a74[1]*links[26].inertia[1][2] + v74[1]*(-(links[26].mcm[2]*v74[4]) + v74[3]*links[26].inertia[1][1] - v74[1]*links[26].inertia[1][3]) + a74[2]*links[26].inertia[2][2] + a74[3]*links[26].inertia[2][3] + v74[2]*(links[26].mcm[1]*v74[4] + links[26].mcm[3]*v74[6] + v74[3]*links[26].inertia[1][2] - v74[1]*links[26].inertia[2][3]) + v74[3]*(-(links[26].mcm[2]*v74[6]) + v74[3]*links[26].inertia[1][3] - v74[1]*links[26].inertia[3][3]);
fnet74[6]=a74[5]*links[26].mcm[1] - a74[4]*links[26].mcm[2] + (links[26].mcm[3]*v74[2] + links[26].m*v74[4])*v74[5] + v74[4]*(links[26].mcm[3]*v74[1] - links[26].m*v74[5]) + (-(links[26].mcm[1]*v74[1]) - links[26].mcm[2]*v74[2])*v74[6] + v74[1]*(-(links[26].mcm[3]*v74[4]) - v74[2]*links[26].inertia[1][1] + v74[1]*links[26].inertia[1][2]) + a74[1]*links[26].inertia[1][3] + v74[2]*(-(links[26].mcm[3]*v74[5]) - v74[2]*links[26].inertia[1][2] + v74[1]*links[26].inertia[2][2]) + a74[2]*links[26].inertia[2][3] + v74[3]*(links[26].mcm[1]*v74[4] + links[26].mcm[2]*v74[5] - v74[2]*links[26].inertia[1][3] + v74[1]*links[26].inertia[2][3]) + a74[3]*links[26].inertia[3][3];

fnet75[1]=links[27].m*a75[4] - a75[3]*links[27].mcm[2] + a75[2]*links[27].mcm[3] - links[27].mcm[1]*Power(v75[2],2) - links[27].mcm[1]*Power(v75[3],2) + v75[1]*(links[27].mcm[2]*v75[2] + links[27].mcm[3]*v75[3]) - links[27].m*v75[3]*v75[5] + links[27].m*v75[2]*v75[6];
fnet75[2]=links[27].m*a75[5] + a75[3]*links[27].mcm[1] - a75[1]*links[27].mcm[3] - links[27].mcm[2]*Power(v75[1],2) - links[27].mcm[2]*Power(v75[3],2) + v75[2]*(links[27].mcm[1]*v75[1] + links[27].mcm[3]*v75[3]) + links[27].m*v75[3]*v75[4] - links[27].m*v75[1]*v75[6];
fnet75[3]=links[27].m*a75[6] - a75[2]*links[27].mcm[1] + a75[1]*links[27].mcm[2] - links[27].mcm[3]*Power(v75[1],2) - links[27].mcm[3]*Power(v75[2],2) + (links[27].mcm[1]*v75[1] + links[27].mcm[2]*v75[2])*v75[3] - links[27].m*v75[2]*v75[4] + links[27].m*v75[1]*v75[5];
fnet75[4]=a75[6]*links[27].mcm[2] - a75[5]*links[27].mcm[3] + (-(links[27].mcm[2]*v75[2]) - links[27].mcm[3]*v75[3])*v75[4] + (links[27].mcm[1]*v75[3] + links[27].m*v75[5])*v75[6] + v75[5]*(links[27].mcm[1]*v75[2] - links[27].m*v75[6]) + a75[1]*links[27].inertia[1][1] + a75[2]*links[27].inertia[1][2] + a75[3]*links[27].inertia[1][3] + v75[1]*(links[27].mcm[2]*v75[5] + links[27].mcm[3]*v75[6] - v75[3]*links[27].inertia[1][2] + v75[2]*links[27].inertia[1][3]) + v75[2]*(-(links[27].mcm[1]*v75[5]) - v75[3]*links[27].inertia[2][2] + v75[2]*links[27].inertia[2][3]) + v75[3]*(-(links[27].mcm[1]*v75[6]) - v75[3]*links[27].inertia[2][3] + v75[2]*links[27].inertia[3][3]);
fnet75[5]=-(a75[6]*links[27].mcm[1]) + a75[4]*links[27].mcm[3] + (-(links[27].mcm[1]*v75[1]) - links[27].mcm[3]*v75[3])*v75[5] + (links[27].mcm[2]*v75[3] - links[27].m*v75[4])*v75[6] + v75[4]*(links[27].mcm[2]*v75[1] + links[27].m*v75[6]) + a75[1]*links[27].inertia[1][2] + v75[1]*(-(links[27].mcm[2]*v75[4]) + v75[3]*links[27].inertia[1][1] - v75[1]*links[27].inertia[1][3]) + a75[2]*links[27].inertia[2][2] + a75[3]*links[27].inertia[2][3] + v75[2]*(links[27].mcm[1]*v75[4] + links[27].mcm[3]*v75[6] + v75[3]*links[27].inertia[1][2] - v75[1]*links[27].inertia[2][3]) + v75[3]*(-(links[27].mcm[2]*v75[6]) + v75[3]*links[27].inertia[1][3] - v75[1]*links[27].inertia[3][3]);
fnet75[6]=a75[5]*links[27].mcm[1] - a75[4]*links[27].mcm[2] + (links[27].mcm[3]*v75[2] + links[27].m*v75[4])*v75[5] + v75[4]*(links[27].mcm[3]*v75[1] - links[27].m*v75[5]) + (-(links[27].mcm[1]*v75[1]) - links[27].mcm[2]*v75[2])*v75[6] + v75[1]*(-(links[27].mcm[3]*v75[4]) - v75[2]*links[27].inertia[1][1] + v75[1]*links[27].inertia[1][2]) + a75[1]*links[27].inertia[1][3] + v75[2]*(-(links[27].mcm[3]*v75[5]) - v75[2]*links[27].inertia[1][2] + v75[1]*links[27].inertia[2][2]) + a75[2]*links[27].inertia[2][3] + v75[3]*(links[27].mcm[1]*v75[4] + links[27].mcm[2]*v75[5] - v75[2]*links[27].inertia[1][3] + v75[1]*links[27].inertia[2][3]) + a75[3]*links[27].inertia[3][3];

fnet76[1]=links[28].m*a76[4] - a76[3]*links[28].mcm[2] + a76[2]*links[28].mcm[3] - links[28].mcm[1]*Power(v76[2],2) - links[28].mcm[1]*Power(v76[3],2) + v76[1]*(links[28].mcm[2]*v76[2] + links[28].mcm[3]*v76[3]) - links[28].m*v76[3]*v76[5] + links[28].m*v76[2]*v76[6];
fnet76[2]=links[28].m*a76[5] + a76[3]*links[28].mcm[1] - a76[1]*links[28].mcm[3] - links[28].mcm[2]*Power(v76[1],2) - links[28].mcm[2]*Power(v76[3],2) + v76[2]*(links[28].mcm[1]*v76[1] + links[28].mcm[3]*v76[3]) + links[28].m*v76[3]*v76[4] - links[28].m*v76[1]*v76[6];
fnet76[3]=links[28].m*a76[6] - a76[2]*links[28].mcm[1] + a76[1]*links[28].mcm[2] - links[28].mcm[3]*Power(v76[1],2) - links[28].mcm[3]*Power(v76[2],2) + (links[28].mcm[1]*v76[1] + links[28].mcm[2]*v76[2])*v76[3] - links[28].m*v76[2]*v76[4] + links[28].m*v76[1]*v76[5];
fnet76[4]=a76[6]*links[28].mcm[2] - a76[5]*links[28].mcm[3] + (-(links[28].mcm[2]*v76[2]) - links[28].mcm[3]*v76[3])*v76[4] + (links[28].mcm[1]*v76[3] + links[28].m*v76[5])*v76[6] + v76[5]*(links[28].mcm[1]*v76[2] - links[28].m*v76[6]) + a76[1]*links[28].inertia[1][1] + a76[2]*links[28].inertia[1][2] + a76[3]*links[28].inertia[1][3] + v76[1]*(links[28].mcm[2]*v76[5] + links[28].mcm[3]*v76[6] - v76[3]*links[28].inertia[1][2] + v76[2]*links[28].inertia[1][3]) + v76[2]*(-(links[28].mcm[1]*v76[5]) - v76[3]*links[28].inertia[2][2] + v76[2]*links[28].inertia[2][3]) + v76[3]*(-(links[28].mcm[1]*v76[6]) - v76[3]*links[28].inertia[2][3] + v76[2]*links[28].inertia[3][3]);
fnet76[5]=-(a76[6]*links[28].mcm[1]) + a76[4]*links[28].mcm[3] + (-(links[28].mcm[1]*v76[1]) - links[28].mcm[3]*v76[3])*v76[5] + (links[28].mcm[2]*v76[3] - links[28].m*v76[4])*v76[6] + v76[4]*(links[28].mcm[2]*v76[1] + links[28].m*v76[6]) + a76[1]*links[28].inertia[1][2] + v76[1]*(-(links[28].mcm[2]*v76[4]) + v76[3]*links[28].inertia[1][1] - v76[1]*links[28].inertia[1][3]) + a76[2]*links[28].inertia[2][2] + a76[3]*links[28].inertia[2][3] + v76[2]*(links[28].mcm[1]*v76[4] + links[28].mcm[3]*v76[6] + v76[3]*links[28].inertia[1][2] - v76[1]*links[28].inertia[2][3]) + v76[3]*(-(links[28].mcm[2]*v76[6]) + v76[3]*links[28].inertia[1][3] - v76[1]*links[28].inertia[3][3]);
fnet76[6]=a76[5]*links[28].mcm[1] - a76[4]*links[28].mcm[2] + (links[28].mcm[3]*v76[2] + links[28].m*v76[4])*v76[5] + v76[4]*(links[28].mcm[3]*v76[1] - links[28].m*v76[5]) + (-(links[28].mcm[1]*v76[1]) - links[28].mcm[2]*v76[2])*v76[6] + v76[1]*(-(links[28].mcm[3]*v76[4]) - v76[2]*links[28].inertia[1][1] + v76[1]*links[28].inertia[1][2]) + a76[1]*links[28].inertia[1][3] + v76[2]*(-(links[28].mcm[3]*v76[5]) - v76[2]*links[28].inertia[1][2] + v76[1]*links[28].inertia[2][2]) + a76[2]*links[28].inertia[2][3] + v76[3]*(links[28].mcm[1]*v76[4] + links[28].mcm[2]*v76[5] - v76[2]*links[28].inertia[1][3] + v76[1]*links[28].inertia[2][3]) + a76[3]*links[28].inertia[3][3];

fnet83[1]=eff[3].m*a83[4] - a83[3]*eff[3].mcm[2] + a83[2]*eff[3].mcm[3] - eff[3].mcm[1]*Power(v83[2],2) - eff[3].mcm[1]*Power(v83[3],2) + v83[1]*(eff[3].mcm[2]*v83[2] + eff[3].mcm[3]*v83[3]) - eff[3].m*v83[3]*v83[5] + eff[3].m*v83[2]*v83[6];
fnet83[2]=eff[3].m*a83[5] + a83[3]*eff[3].mcm[1] - a83[1]*eff[3].mcm[3] - eff[3].mcm[2]*Power(v83[1],2) - eff[3].mcm[2]*Power(v83[3],2) + v83[2]*(eff[3].mcm[1]*v83[1] + eff[3].mcm[3]*v83[3]) + eff[3].m*v83[3]*v83[4] - eff[3].m*v83[1]*v83[6];
fnet83[3]=eff[3].m*a83[6] - a83[2]*eff[3].mcm[1] + a83[1]*eff[3].mcm[2] - eff[3].mcm[3]*Power(v83[1],2) - eff[3].mcm[3]*Power(v83[2],2) + (eff[3].mcm[1]*v83[1] + eff[3].mcm[2]*v83[2])*v83[3] - eff[3].m*v83[2]*v83[4] + eff[3].m*v83[1]*v83[5];
fnet83[4]=a83[6]*eff[3].mcm[2] - a83[5]*eff[3].mcm[3] + (-(eff[3].mcm[2]*v83[2]) - eff[3].mcm[3]*v83[3])*v83[4] - eff[3].mcm[1]*v83[2]*v83[5] - eff[3].mcm[1]*v83[3]*v83[6] + (eff[3].mcm[1]*v83[3] + eff[3].m*v83[5])*v83[6] + v83[5]*(eff[3].mcm[1]*v83[2] - eff[3].m*v83[6]) + v83[1]*(eff[3].mcm[2]*v83[5] + eff[3].mcm[3]*v83[6]);
fnet83[5]=-(a83[6]*eff[3].mcm[1]) + a83[4]*eff[3].mcm[3] - eff[3].mcm[2]*v83[1]*v83[4] + (-(eff[3].mcm[1]*v83[1]) - eff[3].mcm[3]*v83[3])*v83[5] - eff[3].mcm[2]*v83[3]*v83[6] + (eff[3].mcm[2]*v83[3] - eff[3].m*v83[4])*v83[6] + v83[4]*(eff[3].mcm[2]*v83[1] + eff[3].m*v83[6]) + v83[2]*(eff[3].mcm[1]*v83[4] + eff[3].mcm[3]*v83[6]);
fnet83[6]=a83[5]*eff[3].mcm[1] - a83[4]*eff[3].mcm[2] - eff[3].mcm[3]*v83[1]*v83[4] - eff[3].mcm[3]*v83[2]*v83[5] + (eff[3].mcm[3]*v83[2] + eff[3].m*v83[4])*v83[5] + v83[4]*(eff[3].mcm[3]*v83[1] - eff[3].m*v83[5]) + v83[3]*(eff[3].mcm[1]*v83[4] + eff[3].mcm[2]*v83[5]) + (-(eff[3].mcm[1]*v83[1]) - eff[3].mcm[2]*v83[2])*v83[6];

fnet84[1]=links[16].m*a84[4] - a84[3]*links[16].mcm[2] + a84[2]*links[16].mcm[3] - links[16].mcm[1]*Power(v84[2],2) - links[16].mcm[1]*Power(v84[3],2) + v84[1]*(links[16].mcm[2]*v84[2] + links[16].mcm[3]*v84[3]) - links[16].m*v84[3]*v84[5] + links[16].m*v84[2]*v84[6];
fnet84[2]=links[16].m*a84[5] + a84[3]*links[16].mcm[1] - a84[1]*links[16].mcm[3] - links[16].mcm[2]*Power(v84[1],2) - links[16].mcm[2]*Power(v84[3],2) + v84[2]*(links[16].mcm[1]*v84[1] + links[16].mcm[3]*v84[3]) + links[16].m*v84[3]*v84[4] - links[16].m*v84[1]*v84[6];
fnet84[3]=links[16].m*a84[6] - a84[2]*links[16].mcm[1] + a84[1]*links[16].mcm[2] - links[16].mcm[3]*Power(v84[1],2) - links[16].mcm[3]*Power(v84[2],2) + (links[16].mcm[1]*v84[1] + links[16].mcm[2]*v84[2])*v84[3] - links[16].m*v84[2]*v84[4] + links[16].m*v84[1]*v84[5];
fnet84[4]=a84[6]*links[16].mcm[2] - a84[5]*links[16].mcm[3] + (-(links[16].mcm[2]*v84[2]) - links[16].mcm[3]*v84[3])*v84[4] + (links[16].mcm[1]*v84[3] + links[16].m*v84[5])*v84[6] + v84[5]*(links[16].mcm[1]*v84[2] - links[16].m*v84[6]) + a84[1]*links[16].inertia[1][1] + a84[2]*links[16].inertia[1][2] + a84[3]*links[16].inertia[1][3] + v84[1]*(links[16].mcm[2]*v84[5] + links[16].mcm[3]*v84[6] - v84[3]*links[16].inertia[1][2] + v84[2]*links[16].inertia[1][3]) + v84[2]*(-(links[16].mcm[1]*v84[5]) - v84[3]*links[16].inertia[2][2] + v84[2]*links[16].inertia[2][3]) + v84[3]*(-(links[16].mcm[1]*v84[6]) - v84[3]*links[16].inertia[2][3] + v84[2]*links[16].inertia[3][3]);
fnet84[5]=-(a84[6]*links[16].mcm[1]) + a84[4]*links[16].mcm[3] + (-(links[16].mcm[1]*v84[1]) - links[16].mcm[3]*v84[3])*v84[5] + (links[16].mcm[2]*v84[3] - links[16].m*v84[4])*v84[6] + v84[4]*(links[16].mcm[2]*v84[1] + links[16].m*v84[6]) + a84[1]*links[16].inertia[1][2] + v84[1]*(-(links[16].mcm[2]*v84[4]) + v84[3]*links[16].inertia[1][1] - v84[1]*links[16].inertia[1][3]) + a84[2]*links[16].inertia[2][2] + a84[3]*links[16].inertia[2][3] + v84[2]*(links[16].mcm[1]*v84[4] + links[16].mcm[3]*v84[6] + v84[3]*links[16].inertia[1][2] - v84[1]*links[16].inertia[2][3]) + v84[3]*(-(links[16].mcm[2]*v84[6]) + v84[3]*links[16].inertia[1][3] - v84[1]*links[16].inertia[3][3]);
fnet84[6]=a84[5]*links[16].mcm[1] - a84[4]*links[16].mcm[2] + (links[16].mcm[3]*v84[2] + links[16].m*v84[4])*v84[5] + v84[4]*(links[16].mcm[3]*v84[1] - links[16].m*v84[5]) + (-(links[16].mcm[1]*v84[1]) - links[16].mcm[2]*v84[2])*v84[6] + v84[1]*(-(links[16].mcm[3]*v84[4]) - v84[2]*links[16].inertia[1][1] + v84[1]*links[16].inertia[1][2]) + a84[1]*links[16].inertia[1][3] + v84[2]*(-(links[16].mcm[3]*v84[5]) - v84[2]*links[16].inertia[1][2] + v84[1]*links[16].inertia[2][2]) + a84[2]*links[16].inertia[2][3] + v84[3]*(links[16].mcm[1]*v84[4] + links[16].mcm[2]*v84[5] - v84[2]*links[16].inertia[1][3] + v84[1]*links[16].inertia[2][3]) + a84[3]*links[16].inertia[3][3];

fnet85[1]=links[15].m*a85[4] - a85[3]*links[15].mcm[2] + a85[2]*links[15].mcm[3] - links[15].mcm[1]*Power(v85[2],2) - links[15].mcm[1]*Power(v85[3],2) + v85[1]*(links[15].mcm[2]*v85[2] + links[15].mcm[3]*v85[3]) - links[15].m*v85[3]*v85[5] + links[15].m*v85[2]*v85[6];
fnet85[2]=links[15].m*a85[5] + a85[3]*links[15].mcm[1] - a85[1]*links[15].mcm[3] - links[15].mcm[2]*Power(v85[1],2) - links[15].mcm[2]*Power(v85[3],2) + v85[2]*(links[15].mcm[1]*v85[1] + links[15].mcm[3]*v85[3]) + links[15].m*v85[3]*v85[4] - links[15].m*v85[1]*v85[6];
fnet85[3]=links[15].m*a85[6] - a85[2]*links[15].mcm[1] + a85[1]*links[15].mcm[2] - links[15].mcm[3]*Power(v85[1],2) - links[15].mcm[3]*Power(v85[2],2) + (links[15].mcm[1]*v85[1] + links[15].mcm[2]*v85[2])*v85[3] - links[15].m*v85[2]*v85[4] + links[15].m*v85[1]*v85[5];
fnet85[4]=a85[6]*links[15].mcm[2] - a85[5]*links[15].mcm[3] + (-(links[15].mcm[2]*v85[2]) - links[15].mcm[3]*v85[3])*v85[4] + (links[15].mcm[1]*v85[3] + links[15].m*v85[5])*v85[6] + v85[5]*(links[15].mcm[1]*v85[2] - links[15].m*v85[6]) + a85[1]*links[15].inertia[1][1] + a85[2]*links[15].inertia[1][2] + a85[3]*links[15].inertia[1][3] + v85[1]*(links[15].mcm[2]*v85[5] + links[15].mcm[3]*v85[6] - v85[3]*links[15].inertia[1][2] + v85[2]*links[15].inertia[1][3]) + v85[2]*(-(links[15].mcm[1]*v85[5]) - v85[3]*links[15].inertia[2][2] + v85[2]*links[15].inertia[2][3]) + v85[3]*(-(links[15].mcm[1]*v85[6]) - v85[3]*links[15].inertia[2][3] + v85[2]*links[15].inertia[3][3]);
fnet85[5]=-(a85[6]*links[15].mcm[1]) + a85[4]*links[15].mcm[3] + (-(links[15].mcm[1]*v85[1]) - links[15].mcm[3]*v85[3])*v85[5] + (links[15].mcm[2]*v85[3] - links[15].m*v85[4])*v85[6] + v85[4]*(links[15].mcm[2]*v85[1] + links[15].m*v85[6]) + a85[1]*links[15].inertia[1][2] + v85[1]*(-(links[15].mcm[2]*v85[4]) + v85[3]*links[15].inertia[1][1] - v85[1]*links[15].inertia[1][3]) + a85[2]*links[15].inertia[2][2] + a85[3]*links[15].inertia[2][3] + v85[2]*(links[15].mcm[1]*v85[4] + links[15].mcm[3]*v85[6] + v85[3]*links[15].inertia[1][2] - v85[1]*links[15].inertia[2][3]) + v85[3]*(-(links[15].mcm[2]*v85[6]) + v85[3]*links[15].inertia[1][3] - v85[1]*links[15].inertia[3][3]);
fnet85[6]=a85[5]*links[15].mcm[1] - a85[4]*links[15].mcm[2] + (links[15].mcm[3]*v85[2] + links[15].m*v85[4])*v85[5] + v85[4]*(links[15].mcm[3]*v85[1] - links[15].m*v85[5]) + (-(links[15].mcm[1]*v85[1]) - links[15].mcm[2]*v85[2])*v85[6] + v85[1]*(-(links[15].mcm[3]*v85[4]) - v85[2]*links[15].inertia[1][1] + v85[1]*links[15].inertia[1][2]) + a85[1]*links[15].inertia[1][3] + v85[2]*(-(links[15].mcm[3]*v85[5]) - v85[2]*links[15].inertia[1][2] + v85[1]*links[15].inertia[2][2]) + a85[2]*links[15].inertia[2][3] + v85[3]*(links[15].mcm[1]*v85[4] + links[15].mcm[2]*v85[5] - v85[2]*links[15].inertia[1][3] + v85[1]*links[15].inertia[2][3]) + a85[3]*links[15].inertia[3][3];

fnet86[1]=links[17].m*a86[4] - a86[3]*links[17].mcm[2] + a86[2]*links[17].mcm[3] - links[17].mcm[1]*Power(v86[2],2) - links[17].mcm[1]*Power(v86[3],2) + v86[1]*(links[17].mcm[2]*v86[2] + links[17].mcm[3]*v86[3]) - links[17].m*v86[3]*v86[5] + links[17].m*v86[2]*v86[6];
fnet86[2]=links[17].m*a86[5] + a86[3]*links[17].mcm[1] - a86[1]*links[17].mcm[3] - links[17].mcm[2]*Power(v86[1],2) - links[17].mcm[2]*Power(v86[3],2) + v86[2]*(links[17].mcm[1]*v86[1] + links[17].mcm[3]*v86[3]) + links[17].m*v86[3]*v86[4] - links[17].m*v86[1]*v86[6];
fnet86[3]=links[17].m*a86[6] - a86[2]*links[17].mcm[1] + a86[1]*links[17].mcm[2] - links[17].mcm[3]*Power(v86[1],2) - links[17].mcm[3]*Power(v86[2],2) + (links[17].mcm[1]*v86[1] + links[17].mcm[2]*v86[2])*v86[3] - links[17].m*v86[2]*v86[4] + links[17].m*v86[1]*v86[5];
fnet86[4]=a86[6]*links[17].mcm[2] - a86[5]*links[17].mcm[3] + (-(links[17].mcm[2]*v86[2]) - links[17].mcm[3]*v86[3])*v86[4] + (links[17].mcm[1]*v86[3] + links[17].m*v86[5])*v86[6] + v86[5]*(links[17].mcm[1]*v86[2] - links[17].m*v86[6]) + a86[1]*links[17].inertia[1][1] + a86[2]*links[17].inertia[1][2] + a86[3]*links[17].inertia[1][3] + v86[1]*(links[17].mcm[2]*v86[5] + links[17].mcm[3]*v86[6] - v86[3]*links[17].inertia[1][2] + v86[2]*links[17].inertia[1][3]) + v86[2]*(-(links[17].mcm[1]*v86[5]) - v86[3]*links[17].inertia[2][2] + v86[2]*links[17].inertia[2][3]) + v86[3]*(-(links[17].mcm[1]*v86[6]) - v86[3]*links[17].inertia[2][3] + v86[2]*links[17].inertia[3][3]);
fnet86[5]=-(a86[6]*links[17].mcm[1]) + a86[4]*links[17].mcm[3] + (-(links[17].mcm[1]*v86[1]) - links[17].mcm[3]*v86[3])*v86[5] + (links[17].mcm[2]*v86[3] - links[17].m*v86[4])*v86[6] + v86[4]*(links[17].mcm[2]*v86[1] + links[17].m*v86[6]) + a86[1]*links[17].inertia[1][2] + v86[1]*(-(links[17].mcm[2]*v86[4]) + v86[3]*links[17].inertia[1][1] - v86[1]*links[17].inertia[1][3]) + a86[2]*links[17].inertia[2][2] + a86[3]*links[17].inertia[2][3] + v86[2]*(links[17].mcm[1]*v86[4] + links[17].mcm[3]*v86[6] + v86[3]*links[17].inertia[1][2] - v86[1]*links[17].inertia[2][3]) + v86[3]*(-(links[17].mcm[2]*v86[6]) + v86[3]*links[17].inertia[1][3] - v86[1]*links[17].inertia[3][3]);
fnet86[6]=a86[5]*links[17].mcm[1] - a86[4]*links[17].mcm[2] + (links[17].mcm[3]*v86[2] + links[17].m*v86[4])*v86[5] + v86[4]*(links[17].mcm[3]*v86[1] - links[17].m*v86[5]) + (-(links[17].mcm[1]*v86[1]) - links[17].mcm[2]*v86[2])*v86[6] + v86[1]*(-(links[17].mcm[3]*v86[4]) - v86[2]*links[17].inertia[1][1] + v86[1]*links[17].inertia[1][2]) + a86[1]*links[17].inertia[1][3] + v86[2]*(-(links[17].mcm[3]*v86[5]) - v86[2]*links[17].inertia[1][2] + v86[1]*links[17].inertia[2][2]) + a86[2]*links[17].inertia[2][3] + v86[3]*(links[17].mcm[1]*v86[4] + links[17].mcm[2]*v86[5] - v86[2]*links[17].inertia[1][3] + v86[1]*links[17].inertia[2][3]) + a86[3]*links[17].inertia[3][3];

fnet87[1]=links[18].m*a87[4] - a87[3]*links[18].mcm[2] + a87[2]*links[18].mcm[3] - links[18].mcm[1]*Power(v87[2],2) - links[18].mcm[1]*Power(v87[3],2) + v87[1]*(links[18].mcm[2]*v87[2] + links[18].mcm[3]*v87[3]) - links[18].m*v87[3]*v87[5] + links[18].m*v87[2]*v87[6];
fnet87[2]=links[18].m*a87[5] + a87[3]*links[18].mcm[1] - a87[1]*links[18].mcm[3] - links[18].mcm[2]*Power(v87[1],2) - links[18].mcm[2]*Power(v87[3],2) + v87[2]*(links[18].mcm[1]*v87[1] + links[18].mcm[3]*v87[3]) + links[18].m*v87[3]*v87[4] - links[18].m*v87[1]*v87[6];
fnet87[3]=links[18].m*a87[6] - a87[2]*links[18].mcm[1] + a87[1]*links[18].mcm[2] - links[18].mcm[3]*Power(v87[1],2) - links[18].mcm[3]*Power(v87[2],2) + (links[18].mcm[1]*v87[1] + links[18].mcm[2]*v87[2])*v87[3] - links[18].m*v87[2]*v87[4] + links[18].m*v87[1]*v87[5];
fnet87[4]=a87[6]*links[18].mcm[2] - a87[5]*links[18].mcm[3] + (-(links[18].mcm[2]*v87[2]) - links[18].mcm[3]*v87[3])*v87[4] + (links[18].mcm[1]*v87[3] + links[18].m*v87[5])*v87[6] + v87[5]*(links[18].mcm[1]*v87[2] - links[18].m*v87[6]) + a87[1]*links[18].inertia[1][1] + a87[2]*links[18].inertia[1][2] + a87[3]*links[18].inertia[1][3] + v87[1]*(links[18].mcm[2]*v87[5] + links[18].mcm[3]*v87[6] - v87[3]*links[18].inertia[1][2] + v87[2]*links[18].inertia[1][3]) + v87[2]*(-(links[18].mcm[1]*v87[5]) - v87[3]*links[18].inertia[2][2] + v87[2]*links[18].inertia[2][3]) + v87[3]*(-(links[18].mcm[1]*v87[6]) - v87[3]*links[18].inertia[2][3] + v87[2]*links[18].inertia[3][3]);
fnet87[5]=-(a87[6]*links[18].mcm[1]) + a87[4]*links[18].mcm[3] + (-(links[18].mcm[1]*v87[1]) - links[18].mcm[3]*v87[3])*v87[5] + (links[18].mcm[2]*v87[3] - links[18].m*v87[4])*v87[6] + v87[4]*(links[18].mcm[2]*v87[1] + links[18].m*v87[6]) + a87[1]*links[18].inertia[1][2] + v87[1]*(-(links[18].mcm[2]*v87[4]) + v87[3]*links[18].inertia[1][1] - v87[1]*links[18].inertia[1][3]) + a87[2]*links[18].inertia[2][2] + a87[3]*links[18].inertia[2][3] + v87[2]*(links[18].mcm[1]*v87[4] + links[18].mcm[3]*v87[6] + v87[3]*links[18].inertia[1][2] - v87[1]*links[18].inertia[2][3]) + v87[3]*(-(links[18].mcm[2]*v87[6]) + v87[3]*links[18].inertia[1][3] - v87[1]*links[18].inertia[3][3]);
fnet87[6]=a87[5]*links[18].mcm[1] - a87[4]*links[18].mcm[2] + (links[18].mcm[3]*v87[2] + links[18].m*v87[4])*v87[5] + v87[4]*(links[18].mcm[3]*v87[1] - links[18].m*v87[5]) + (-(links[18].mcm[1]*v87[1]) - links[18].mcm[2]*v87[2])*v87[6] + v87[1]*(-(links[18].mcm[3]*v87[4]) - v87[2]*links[18].inertia[1][1] + v87[1]*links[18].inertia[1][2]) + a87[1]*links[18].inertia[1][3] + v87[2]*(-(links[18].mcm[3]*v87[5]) - v87[2]*links[18].inertia[1][2] + v87[1]*links[18].inertia[2][2]) + a87[2]*links[18].inertia[2][3] + v87[3]*(links[18].mcm[1]*v87[4] + links[18].mcm[2]*v87[5] - v87[2]*links[18].inertia[1][3] + v87[1]*links[18].inertia[2][3]) + a87[3]*links[18].inertia[3][3];

fnet88[1]=links[19].m*a88[4] - a88[3]*links[19].mcm[2] + a88[2]*links[19].mcm[3] - links[19].mcm[1]*Power(v88[2],2) - links[19].mcm[1]*Power(v88[3],2) + v88[1]*(links[19].mcm[2]*v88[2] + links[19].mcm[3]*v88[3]) - links[19].m*v88[3]*v88[5] + links[19].m*v88[2]*v88[6];
fnet88[2]=links[19].m*a88[5] + a88[3]*links[19].mcm[1] - a88[1]*links[19].mcm[3] - links[19].mcm[2]*Power(v88[1],2) - links[19].mcm[2]*Power(v88[3],2) + v88[2]*(links[19].mcm[1]*v88[1] + links[19].mcm[3]*v88[3]) + links[19].m*v88[3]*v88[4] - links[19].m*v88[1]*v88[6];
fnet88[3]=links[19].m*a88[6] - a88[2]*links[19].mcm[1] + a88[1]*links[19].mcm[2] - links[19].mcm[3]*Power(v88[1],2) - links[19].mcm[3]*Power(v88[2],2) + (links[19].mcm[1]*v88[1] + links[19].mcm[2]*v88[2])*v88[3] - links[19].m*v88[2]*v88[4] + links[19].m*v88[1]*v88[5];
fnet88[4]=a88[6]*links[19].mcm[2] - a88[5]*links[19].mcm[3] + (-(links[19].mcm[2]*v88[2]) - links[19].mcm[3]*v88[3])*v88[4] + (links[19].mcm[1]*v88[3] + links[19].m*v88[5])*v88[6] + v88[5]*(links[19].mcm[1]*v88[2] - links[19].m*v88[6]) + a88[1]*links[19].inertia[1][1] + a88[2]*links[19].inertia[1][2] + a88[3]*links[19].inertia[1][3] + v88[1]*(links[19].mcm[2]*v88[5] + links[19].mcm[3]*v88[6] - v88[3]*links[19].inertia[1][2] + v88[2]*links[19].inertia[1][3]) + v88[2]*(-(links[19].mcm[1]*v88[5]) - v88[3]*links[19].inertia[2][2] + v88[2]*links[19].inertia[2][3]) + v88[3]*(-(links[19].mcm[1]*v88[6]) - v88[3]*links[19].inertia[2][3] + v88[2]*links[19].inertia[3][3]);
fnet88[5]=-(a88[6]*links[19].mcm[1]) + a88[4]*links[19].mcm[3] + (-(links[19].mcm[1]*v88[1]) - links[19].mcm[3]*v88[3])*v88[5] + (links[19].mcm[2]*v88[3] - links[19].m*v88[4])*v88[6] + v88[4]*(links[19].mcm[2]*v88[1] + links[19].m*v88[6]) + a88[1]*links[19].inertia[1][2] + v88[1]*(-(links[19].mcm[2]*v88[4]) + v88[3]*links[19].inertia[1][1] - v88[1]*links[19].inertia[1][3]) + a88[2]*links[19].inertia[2][2] + a88[3]*links[19].inertia[2][3] + v88[2]*(links[19].mcm[1]*v88[4] + links[19].mcm[3]*v88[6] + v88[3]*links[19].inertia[1][2] - v88[1]*links[19].inertia[2][3]) + v88[3]*(-(links[19].mcm[2]*v88[6]) + v88[3]*links[19].inertia[1][3] - v88[1]*links[19].inertia[3][3]);
fnet88[6]=a88[5]*links[19].mcm[1] - a88[4]*links[19].mcm[2] + (links[19].mcm[3]*v88[2] + links[19].m*v88[4])*v88[5] + v88[4]*(links[19].mcm[3]*v88[1] - links[19].m*v88[5]) + (-(links[19].mcm[1]*v88[1]) - links[19].mcm[2]*v88[2])*v88[6] + v88[1]*(-(links[19].mcm[3]*v88[4]) - v88[2]*links[19].inertia[1][1] + v88[1]*links[19].inertia[1][2]) + a88[1]*links[19].inertia[1][3] + v88[2]*(-(links[19].mcm[3]*v88[5]) - v88[2]*links[19].inertia[1][2] + v88[1]*links[19].inertia[2][2]) + a88[2]*links[19].inertia[2][3] + v88[3]*(links[19].mcm[1]*v88[4] + links[19].mcm[2]*v88[5] - v88[2]*links[19].inertia[1][3] + v88[1]*links[19].inertia[2][3]) + a88[3]*links[19].inertia[3][3];

fnet89[1]=links[20].m*a89[4] - a89[3]*links[20].mcm[2] + a89[2]*links[20].mcm[3] - links[20].mcm[1]*Power(v89[2],2) - links[20].mcm[1]*Power(v89[3],2) + v89[1]*(links[20].mcm[2]*v89[2] + links[20].mcm[3]*v89[3]) - links[20].m*v89[3]*v89[5] + links[20].m*v89[2]*v89[6];
fnet89[2]=links[20].m*a89[5] + a89[3]*links[20].mcm[1] - a89[1]*links[20].mcm[3] - links[20].mcm[2]*Power(v89[1],2) - links[20].mcm[2]*Power(v89[3],2) + v89[2]*(links[20].mcm[1]*v89[1] + links[20].mcm[3]*v89[3]) + links[20].m*v89[3]*v89[4] - links[20].m*v89[1]*v89[6];
fnet89[3]=links[20].m*a89[6] - a89[2]*links[20].mcm[1] + a89[1]*links[20].mcm[2] - links[20].mcm[3]*Power(v89[1],2) - links[20].mcm[3]*Power(v89[2],2) + (links[20].mcm[1]*v89[1] + links[20].mcm[2]*v89[2])*v89[3] - links[20].m*v89[2]*v89[4] + links[20].m*v89[1]*v89[5];
fnet89[4]=a89[6]*links[20].mcm[2] - a89[5]*links[20].mcm[3] + (-(links[20].mcm[2]*v89[2]) - links[20].mcm[3]*v89[3])*v89[4] + (links[20].mcm[1]*v89[3] + links[20].m*v89[5])*v89[6] + v89[5]*(links[20].mcm[1]*v89[2] - links[20].m*v89[6]) + a89[1]*links[20].inertia[1][1] + a89[2]*links[20].inertia[1][2] + a89[3]*links[20].inertia[1][3] + v89[1]*(links[20].mcm[2]*v89[5] + links[20].mcm[3]*v89[6] - v89[3]*links[20].inertia[1][2] + v89[2]*links[20].inertia[1][3]) + v89[2]*(-(links[20].mcm[1]*v89[5]) - v89[3]*links[20].inertia[2][2] + v89[2]*links[20].inertia[2][3]) + v89[3]*(-(links[20].mcm[1]*v89[6]) - v89[3]*links[20].inertia[2][3] + v89[2]*links[20].inertia[3][3]);
fnet89[5]=-(a89[6]*links[20].mcm[1]) + a89[4]*links[20].mcm[3] + (-(links[20].mcm[1]*v89[1]) - links[20].mcm[3]*v89[3])*v89[5] + (links[20].mcm[2]*v89[3] - links[20].m*v89[4])*v89[6] + v89[4]*(links[20].mcm[2]*v89[1] + links[20].m*v89[6]) + a89[1]*links[20].inertia[1][2] + v89[1]*(-(links[20].mcm[2]*v89[4]) + v89[3]*links[20].inertia[1][1] - v89[1]*links[20].inertia[1][3]) + a89[2]*links[20].inertia[2][2] + a89[3]*links[20].inertia[2][3] + v89[2]*(links[20].mcm[1]*v89[4] + links[20].mcm[3]*v89[6] + v89[3]*links[20].inertia[1][2] - v89[1]*links[20].inertia[2][3]) + v89[3]*(-(links[20].mcm[2]*v89[6]) + v89[3]*links[20].inertia[1][3] - v89[1]*links[20].inertia[3][3]);
fnet89[6]=a89[5]*links[20].mcm[1] - a89[4]*links[20].mcm[2] + (links[20].mcm[3]*v89[2] + links[20].m*v89[4])*v89[5] + v89[4]*(links[20].mcm[3]*v89[1] - links[20].m*v89[5]) + (-(links[20].mcm[1]*v89[1]) - links[20].mcm[2]*v89[2])*v89[6] + v89[1]*(-(links[20].mcm[3]*v89[4]) - v89[2]*links[20].inertia[1][1] + v89[1]*links[20].inertia[1][2]) + a89[1]*links[20].inertia[1][3] + v89[2]*(-(links[20].mcm[3]*v89[5]) - v89[2]*links[20].inertia[1][2] + v89[1]*links[20].inertia[2][2]) + a89[2]*links[20].inertia[2][3] + v89[3]*(links[20].mcm[1]*v89[4] + links[20].mcm[2]*v89[5] - v89[2]*links[20].inertia[1][3] + v89[1]*links[20].inertia[2][3]) + a89[3]*links[20].inertia[3][3];

fnet90[1]=links[21].m*a90[4] - a90[3]*links[21].mcm[2] + a90[2]*links[21].mcm[3] - links[21].mcm[1]*Power(v90[2],2) - links[21].mcm[1]*Power(v90[3],2) + v90[1]*(links[21].mcm[2]*v90[2] + links[21].mcm[3]*v90[3]) - links[21].m*v90[3]*v90[5] + links[21].m*v90[2]*v90[6];
fnet90[2]=links[21].m*a90[5] + a90[3]*links[21].mcm[1] - a90[1]*links[21].mcm[3] - links[21].mcm[2]*Power(v90[1],2) - links[21].mcm[2]*Power(v90[3],2) + v90[2]*(links[21].mcm[1]*v90[1] + links[21].mcm[3]*v90[3]) + links[21].m*v90[3]*v90[4] - links[21].m*v90[1]*v90[6];
fnet90[3]=links[21].m*a90[6] - a90[2]*links[21].mcm[1] + a90[1]*links[21].mcm[2] - links[21].mcm[3]*Power(v90[1],2) - links[21].mcm[3]*Power(v90[2],2) + (links[21].mcm[1]*v90[1] + links[21].mcm[2]*v90[2])*v90[3] - links[21].m*v90[2]*v90[4] + links[21].m*v90[1]*v90[5];
fnet90[4]=a90[6]*links[21].mcm[2] - a90[5]*links[21].mcm[3] + (-(links[21].mcm[2]*v90[2]) - links[21].mcm[3]*v90[3])*v90[4] + (links[21].mcm[1]*v90[3] + links[21].m*v90[5])*v90[6] + v90[5]*(links[21].mcm[1]*v90[2] - links[21].m*v90[6]) + a90[1]*links[21].inertia[1][1] + a90[2]*links[21].inertia[1][2] + a90[3]*links[21].inertia[1][3] + v90[1]*(links[21].mcm[2]*v90[5] + links[21].mcm[3]*v90[6] - v90[3]*links[21].inertia[1][2] + v90[2]*links[21].inertia[1][3]) + v90[2]*(-(links[21].mcm[1]*v90[5]) - v90[3]*links[21].inertia[2][2] + v90[2]*links[21].inertia[2][3]) + v90[3]*(-(links[21].mcm[1]*v90[6]) - v90[3]*links[21].inertia[2][3] + v90[2]*links[21].inertia[3][3]);
fnet90[5]=-(a90[6]*links[21].mcm[1]) + a90[4]*links[21].mcm[3] + (-(links[21].mcm[1]*v90[1]) - links[21].mcm[3]*v90[3])*v90[5] + (links[21].mcm[2]*v90[3] - links[21].m*v90[4])*v90[6] + v90[4]*(links[21].mcm[2]*v90[1] + links[21].m*v90[6]) + a90[1]*links[21].inertia[1][2] + v90[1]*(-(links[21].mcm[2]*v90[4]) + v90[3]*links[21].inertia[1][1] - v90[1]*links[21].inertia[1][3]) + a90[2]*links[21].inertia[2][2] + a90[3]*links[21].inertia[2][3] + v90[2]*(links[21].mcm[1]*v90[4] + links[21].mcm[3]*v90[6] + v90[3]*links[21].inertia[1][2] - v90[1]*links[21].inertia[2][3]) + v90[3]*(-(links[21].mcm[2]*v90[6]) + v90[3]*links[21].inertia[1][3] - v90[1]*links[21].inertia[3][3]);
fnet90[6]=a90[5]*links[21].mcm[1] - a90[4]*links[21].mcm[2] + (links[21].mcm[3]*v90[2] + links[21].m*v90[4])*v90[5] + v90[4]*(links[21].mcm[3]*v90[1] - links[21].m*v90[5]) + (-(links[21].mcm[1]*v90[1]) - links[21].mcm[2]*v90[2])*v90[6] + v90[1]*(-(links[21].mcm[3]*v90[4]) - v90[2]*links[21].inertia[1][1] + v90[1]*links[21].inertia[1][2]) + a90[1]*links[21].inertia[1][3] + v90[2]*(-(links[21].mcm[3]*v90[5]) - v90[2]*links[21].inertia[1][2] + v90[1]*links[21].inertia[2][2]) + a90[2]*links[21].inertia[2][3] + v90[3]*(links[21].mcm[1]*v90[4] + links[21].mcm[2]*v90[5] - v90[2]*links[21].inertia[1][3] + v90[1]*links[21].inertia[2][3]) + a90[3]*links[21].inertia[3][3];

fnet97[1]=eff[4].m*a97[4] - a97[3]*eff[4].mcm[2] + a97[2]*eff[4].mcm[3] - eff[4].mcm[1]*Power(v97[2],2) - eff[4].mcm[1]*Power(v97[3],2) + v97[1]*(eff[4].mcm[2]*v97[2] + eff[4].mcm[3]*v97[3]) - eff[4].m*v97[3]*v97[5] + eff[4].m*v97[2]*v97[6];
fnet97[2]=eff[4].m*a97[5] + a97[3]*eff[4].mcm[1] - a97[1]*eff[4].mcm[3] - eff[4].mcm[2]*Power(v97[1],2) - eff[4].mcm[2]*Power(v97[3],2) + v97[2]*(eff[4].mcm[1]*v97[1] + eff[4].mcm[3]*v97[3]) + eff[4].m*v97[3]*v97[4] - eff[4].m*v97[1]*v97[6];
fnet97[3]=eff[4].m*a97[6] - a97[2]*eff[4].mcm[1] + a97[1]*eff[4].mcm[2] - eff[4].mcm[3]*Power(v97[1],2) - eff[4].mcm[3]*Power(v97[2],2) + (eff[4].mcm[1]*v97[1] + eff[4].mcm[2]*v97[2])*v97[3] - eff[4].m*v97[2]*v97[4] + eff[4].m*v97[1]*v97[5];
fnet97[4]=a97[6]*eff[4].mcm[2] - a97[5]*eff[4].mcm[3] + (-(eff[4].mcm[2]*v97[2]) - eff[4].mcm[3]*v97[3])*v97[4] - eff[4].mcm[1]*v97[2]*v97[5] - eff[4].mcm[1]*v97[3]*v97[6] + (eff[4].mcm[1]*v97[3] + eff[4].m*v97[5])*v97[6] + v97[5]*(eff[4].mcm[1]*v97[2] - eff[4].m*v97[6]) + v97[1]*(eff[4].mcm[2]*v97[5] + eff[4].mcm[3]*v97[6]);
fnet97[5]=-(a97[6]*eff[4].mcm[1]) + a97[4]*eff[4].mcm[3] - eff[4].mcm[2]*v97[1]*v97[4] + (-(eff[4].mcm[1]*v97[1]) - eff[4].mcm[3]*v97[3])*v97[5] - eff[4].mcm[2]*v97[3]*v97[6] + (eff[4].mcm[2]*v97[3] - eff[4].m*v97[4])*v97[6] + v97[4]*(eff[4].mcm[2]*v97[1] + eff[4].m*v97[6]) + v97[2]*(eff[4].mcm[1]*v97[4] + eff[4].mcm[3]*v97[6]);
fnet97[6]=a97[5]*eff[4].mcm[1] - a97[4]*eff[4].mcm[2] - eff[4].mcm[3]*v97[1]*v97[4] - eff[4].mcm[3]*v97[2]*v97[5] + (eff[4].mcm[3]*v97[2] + eff[4].m*v97[4])*v97[5] + v97[4]*(eff[4].mcm[3]*v97[1] - eff[4].m*v97[5]) + v97[3]*(eff[4].mcm[1]*v97[4] + eff[4].mcm[2]*v97[5]) + (-(eff[4].mcm[1]*v97[1]) - eff[4].mcm[2]*v97[2])*v97[6];


fex0[1]=-(uex[0].f[1]*S00[1][1]) - uex[0].f[2]*S00[1][2] - uex[0].f[3]*S00[1][3];
fex0[2]=-(uex[0].f[1]*S00[2][1]) - uex[0].f[2]*S00[2][2] - uex[0].f[3]*S00[2][3];
fex0[3]=-(uex[0].f[1]*S00[3][1]) - uex[0].f[2]*S00[3][2] - uex[0].f[3]*S00[3][3];
fex0[4]=-(uex[0].t[1]*S00[1][1]) - uex[0].t[2]*S00[1][2] - uex[0].t[3]*S00[1][3];
fex0[5]=-(uex[0].t[1]*S00[2][1]) - uex[0].t[2]*S00[2][2] - uex[0].t[3]*S00[2][3];
fex0[6]=-(uex[0].t[1]*S00[3][1]) - uex[0].t[2]*S00[3][2] - uex[0].t[3]*S00[3][3];

fex1[1]=-(uex[29].f[1]*SG10[1][1]) - uex[29].f[2]*SG10[1][2] - uex[29].f[3]*SG10[1][3];
fex1[2]=-(uex[29].f[1]*SG10[2][1]) - uex[29].f[2]*SG10[2][2] - uex[29].f[3]*SG10[2][3];
fex1[3]=-(uex[29].f[1]*SG10[3][1]) - uex[29].f[2]*SG10[3][2] - uex[29].f[3]*SG10[3][3];
fex1[4]=-(uex[29].t[1]*SG10[1][1]) - uex[29].t[2]*SG10[1][2] - uex[29].t[3]*SG10[1][3];
fex1[5]=-(uex[29].t[1]*SG10[2][1]) - uex[29].t[2]*SG10[2][2] - uex[29].t[3]*SG10[2][3];
fex1[6]=-(uex[29].t[1]*SG10[3][1]) - uex[29].t[2]*SG10[3][2] - uex[29].t[3]*SG10[3][3];

fex2[1]=-(uex[30].f[1]*SG20[1][1]) - uex[30].f[2]*SG20[1][2] - uex[30].f[3]*SG20[1][3];
fex2[2]=-(uex[30].f[1]*SG20[2][1]) - uex[30].f[2]*SG20[2][2] - uex[30].f[3]*SG20[2][3];
fex2[3]=-(uex[30].f[1]*SG20[3][1]) - uex[30].f[2]*SG20[3][2] - uex[30].f[3]*SG20[3][3];
fex2[4]=-(uex[30].t[1]*SG20[1][1]) - uex[30].t[2]*SG20[1][2] - uex[30].t[3]*SG20[1][3];
fex2[5]=-(uex[30].t[1]*SG20[2][1]) - uex[30].t[2]*SG20[2][2] - uex[30].t[3]*SG20[2][3];
fex2[6]=-(uex[30].t[1]*SG20[3][1]) - uex[30].t[2]*SG20[3][2] - uex[30].t[3]*SG20[3][3];

fex3[1]=-(uex[31].f[1]*SG30[1][1]) - uex[31].f[2]*SG30[1][2] - uex[31].f[3]*SG30[1][3];
fex3[2]=-(uex[31].f[1]*SG30[2][1]) - uex[31].f[2]*SG30[2][2] - uex[31].f[3]*SG30[2][3];
fex3[3]=-(uex[31].f[1]*SG30[3][1]) - uex[31].f[2]*SG30[3][2] - uex[31].f[3]*SG30[3][3];
fex3[4]=-(uex[31].t[1]*SG30[1][1]) - uex[31].t[2]*SG30[1][2] - uex[31].t[3]*SG30[1][3];
fex3[5]=-(uex[31].t[1]*SG30[2][1]) - uex[31].t[2]*SG30[2][2] - uex[31].t[3]*SG30[2][3];
fex3[6]=-(uex[31].t[1]*SG30[3][1]) - uex[31].t[2]*SG30[3][2] - uex[31].t[3]*SG30[3][3];

fex5[1]=-(uex[2].f[1]*SG50[1][1]) - uex[2].f[2]*SG50[1][2] - uex[2].f[3]*SG50[1][3];
fex5[2]=-(uex[2].f[1]*SG50[2][1]) - uex[2].f[2]*SG50[2][2] - uex[2].f[3]*SG50[2][3];
fex5[3]=-(uex[2].f[1]*SG50[3][1]) - uex[2].f[2]*SG50[3][2] - uex[2].f[3]*SG50[3][3];
fex5[4]=-(uex[2].t[1]*SG50[1][1]) - uex[2].t[2]*SG50[1][2] - uex[2].t[3]*SG50[1][3];
fex5[5]=-(uex[2].t[1]*SG50[2][1]) - uex[2].t[2]*SG50[2][2] - uex[2].t[3]*SG50[2][3];
fex5[6]=-(uex[2].t[1]*SG50[3][1]) - uex[2].t[2]*SG50[3][2] - uex[2].t[3]*SG50[3][3];

fex6[1]=-(uex[3].f[1]*SG60[1][1]) - uex[3].f[2]*SG60[1][2] - uex[3].f[3]*SG60[1][3];
fex6[2]=-(uex[3].f[1]*SG60[2][1]) - uex[3].f[2]*SG60[2][2] - uex[3].f[3]*SG60[2][3];
fex6[3]=-(uex[3].f[1]*SG60[3][1]) - uex[3].f[2]*SG60[3][2] - uex[3].f[3]*SG60[3][3];
fex6[4]=-(uex[3].t[1]*SG60[1][1]) - uex[3].t[2]*SG60[1][2] - uex[3].t[3]*SG60[1][3];
fex6[5]=-(uex[3].t[1]*SG60[2][1]) - uex[3].t[2]*SG60[2][2] - uex[3].t[3]*SG60[2][3];
fex6[6]=-(uex[3].t[1]*SG60[3][1]) - uex[3].t[2]*SG60[3][2] - uex[3].t[3]*SG60[3][3];

fex7[1]=-(uex[4].f[1]*SG70[1][1]) - uex[4].f[2]*SG70[1][2] - uex[4].f[3]*SG70[1][3];
fex7[2]=-(uex[4].f[1]*SG70[2][1]) - uex[4].f[2]*SG70[2][2] - uex[4].f[3]*SG70[2][3];
fex7[3]=-(uex[4].f[1]*SG70[3][1]) - uex[4].f[2]*SG70[3][2] - uex[4].f[3]*SG70[3][3];
fex7[4]=-(uex[4].t[1]*SG70[1][1]) - uex[4].t[2]*SG70[1][2] - uex[4].t[3]*SG70[1][3];
fex7[5]=-(uex[4].t[1]*SG70[2][1]) - uex[4].t[2]*SG70[2][2] - uex[4].t[3]*SG70[2][3];
fex7[6]=-(uex[4].t[1]*SG70[3][1]) - uex[4].t[2]*SG70[3][2] - uex[4].t[3]*SG70[3][3];

fex8[1]=-(uex[5].f[1]*SG80[1][1]) - uex[5].f[2]*SG80[1][2] - uex[5].f[3]*SG80[1][3];
fex8[2]=-(uex[5].f[1]*SG80[2][1]) - uex[5].f[2]*SG80[2][2] - uex[5].f[3]*SG80[2][3];
fex8[3]=-(uex[5].f[1]*SG80[3][1]) - uex[5].f[2]*SG80[3][2] - uex[5].f[3]*SG80[3][3];
fex8[4]=-(uex[5].t[1]*SG80[1][1]) - uex[5].t[2]*SG80[1][2] - uex[5].t[3]*SG80[1][3];
fex8[5]=-(uex[5].t[1]*SG80[2][1]) - uex[5].t[2]*SG80[2][2] - uex[5].t[3]*SG80[2][3];
fex8[6]=-(uex[5].t[1]*SG80[3][1]) - uex[5].t[2]*SG80[3][2] - uex[5].t[3]*SG80[3][3];

fex9[1]=-(uex[6].f[1]*SG90[1][1]) - uex[6].f[2]*SG90[1][2] - uex[6].f[3]*SG90[1][3];
fex9[2]=-(uex[6].f[1]*SG90[2][1]) - uex[6].f[2]*SG90[2][2] - uex[6].f[3]*SG90[2][3];
fex9[3]=-(uex[6].f[1]*SG90[3][1]) - uex[6].f[2]*SG90[3][2] - uex[6].f[3]*SG90[3][3];
fex9[4]=-(uex[6].t[1]*SG90[1][1]) - uex[6].t[2]*SG90[1][2] - uex[6].t[3]*SG90[1][3];
fex9[5]=-(uex[6].t[1]*SG90[2][1]) - uex[6].t[2]*SG90[2][2] - uex[6].t[3]*SG90[2][3];
fex9[6]=-(uex[6].t[1]*SG90[3][1]) - uex[6].t[2]*SG90[3][2] - uex[6].t[3]*SG90[3][3];

fex10[1]=-(uex[7].f[1]*SG100[1][1]) - uex[7].f[2]*SG100[1][2] - uex[7].f[3]*SG100[1][3];
fex10[2]=-(uex[7].f[1]*SG100[2][1]) - uex[7].f[2]*SG100[2][2] - uex[7].f[3]*SG100[2][3];
fex10[3]=-(uex[7].f[1]*SG100[3][1]) - uex[7].f[2]*SG100[3][2] - uex[7].f[3]*SG100[3][3];
fex10[4]=-(uex[7].t[1]*SG100[1][1]) - uex[7].t[2]*SG100[1][2] - uex[7].t[3]*SG100[1][3];
fex10[5]=-(uex[7].t[1]*SG100[2][1]) - uex[7].t[2]*SG100[2][2] - uex[7].t[3]*SG100[2][3];
fex10[6]=-(uex[7].t[1]*SG100[3][1]) - uex[7].t[2]*SG100[3][2] - uex[7].t[3]*SG100[3][3];

fex12[1]=-(uex[39].f[1]*SG120[1][1]) - uex[39].f[2]*SG120[1][2] - uex[39].f[3]*SG120[1][3];
fex12[2]=-(uex[39].f[1]*SG120[2][1]) - uex[39].f[2]*SG120[2][2] - uex[39].f[3]*SG120[2][3];
fex12[3]=-(uex[39].f[1]*SG120[3][1]) - uex[39].f[2]*SG120[3][2] - uex[39].f[3]*SG120[3][3];
fex12[4]=-(uex[39].t[1]*SG120[1][1]) - uex[39].t[2]*SG120[1][2] - uex[39].t[3]*SG120[1][3];
fex12[5]=-(uex[39].t[1]*SG120[2][1]) - uex[39].t[2]*SG120[2][2] - uex[39].t[3]*SG120[2][3];
fex12[6]=-(uex[39].t[1]*SG120[3][1]) - uex[39].t[2]*SG120[3][2] - uex[39].t[3]*SG120[3][3];

fex13[1]=-(uex[40].f[1]*SG130[1][1]) - uex[40].f[2]*SG130[1][2] - uex[40].f[3]*SG130[1][3];
fex13[2]=-(uex[40].f[1]*SG130[2][1]) - uex[40].f[2]*SG130[2][2] - uex[40].f[3]*SG130[2][3];
fex13[3]=-(uex[40].f[1]*SG130[3][1]) - uex[40].f[2]*SG130[3][2] - uex[40].f[3]*SG130[3][3];
fex13[4]=-(uex[40].t[1]*SG130[1][1]) - uex[40].t[2]*SG130[1][2] - uex[40].t[3]*SG130[1][3];
fex13[5]=-(uex[40].t[1]*SG130[2][1]) - uex[40].t[2]*SG130[2][2] - uex[40].t[3]*SG130[2][3];
fex13[6]=-(uex[40].t[1]*SG130[3][1]) - uex[40].t[2]*SG130[3][2] - uex[40].t[3]*SG130[3][3];

fex16[1]=-(uex[41].f[1]*SG160[1][1]) - uex[41].f[2]*SG160[1][2] - uex[41].f[3]*SG160[1][3];
fex16[2]=-(uex[41].f[1]*SG160[2][1]) - uex[41].f[2]*SG160[2][2] - uex[41].f[3]*SG160[2][3];
fex16[3]=-(uex[41].f[1]*SG160[3][1]) - uex[41].f[2]*SG160[3][2] - uex[41].f[3]*SG160[3][3];
fex16[4]=-(uex[41].t[1]*SG160[1][1]) - uex[41].t[2]*SG160[1][2] - uex[41].t[3]*SG160[1][3];
fex16[5]=-(uex[41].t[1]*SG160[2][1]) - uex[41].t[2]*SG160[2][2] - uex[41].t[3]*SG160[2][3];
fex16[6]=-(uex[41].t[1]*SG160[3][1]) - uex[41].t[2]*SG160[3][2] - uex[41].t[3]*SG160[3][3];

fex20[1]=-(uex[42].f[1]*SG200[1][1]) - uex[42].f[2]*SG200[1][2] - uex[42].f[3]*SG200[1][3];
fex20[2]=-(uex[42].f[1]*SG200[2][1]) - uex[42].f[2]*SG200[2][2] - uex[42].f[3]*SG200[2][3];
fex20[3]=-(uex[42].f[1]*SG200[3][1]) - uex[42].f[2]*SG200[3][2] - uex[42].f[3]*SG200[3][3];
fex20[4]=-(uex[42].t[1]*SG200[1][1]) - uex[42].t[2]*SG200[1][2] - uex[42].t[3]*SG200[1][3];
fex20[5]=-(uex[42].t[1]*SG200[2][1]) - uex[42].t[2]*SG200[2][2] - uex[42].t[3]*SG200[2][3];
fex20[6]=-(uex[42].t[1]*SG200[3][1]) - uex[42].t[2]*SG200[3][2] - uex[42].t[3]*SG200[3][3];

fex24[1]=-(uex[43].f[1]*SG240[1][1]) - uex[43].f[2]*SG240[1][2] - uex[43].f[3]*SG240[1][3];
fex24[2]=-(uex[43].f[1]*SG240[2][1]) - uex[43].f[2]*SG240[2][2] - uex[43].f[3]*SG240[2][3];
fex24[3]=-(uex[43].f[1]*SG240[3][1]) - uex[43].f[2]*SG240[3][2] - uex[43].f[3]*SG240[3][3];
fex24[4]=-(uex[43].t[1]*SG240[1][1]) - uex[43].t[2]*SG240[1][2] - uex[43].t[3]*SG240[1][3];
fex24[5]=-(uex[43].t[1]*SG240[2][1]) - uex[43].t[2]*SG240[2][2] - uex[43].t[3]*SG240[2][3];
fex24[6]=-(uex[43].t[1]*SG240[3][1]) - uex[43].t[2]*SG240[3][2] - uex[43].t[3]*SG240[3][3];

fex28[1]=-(uex[44].f[1]*SG280[1][1]) - uex[44].f[2]*SG280[1][2] - uex[44].f[3]*SG280[1][3];
fex28[2]=-(uex[44].f[1]*SG280[2][1]) - uex[44].f[2]*SG280[2][2] - uex[44].f[3]*SG280[2][3];
fex28[3]=-(uex[44].f[1]*SG280[3][1]) - uex[44].f[2]*SG280[3][2] - uex[44].f[3]*SG280[3][3];
fex28[4]=-(uex[44].t[1]*SG280[1][1]) - uex[44].t[2]*SG280[1][2] - uex[44].t[3]*SG280[1][3];
fex28[5]=-(uex[44].t[1]*SG280[2][1]) - uex[44].t[2]*SG280[2][2] - uex[44].t[3]*SG280[2][3];
fex28[6]=-(uex[44].t[1]*SG280[3][1]) - uex[44].t[2]*SG280[3][2] - uex[44].t[3]*SG280[3][3];

fex33[1]=-(uex[9].f[1]*SG330[1][1]) - uex[9].f[2]*SG330[1][2] - uex[9].f[3]*SG330[1][3];
fex33[2]=-(uex[9].f[1]*SG330[2][1]) - uex[9].f[2]*SG330[2][2] - uex[9].f[3]*SG330[2][3];
fex33[3]=-(uex[9].f[1]*SG330[3][1]) - uex[9].f[2]*SG330[3][2] - uex[9].f[3]*SG330[3][3];
fex33[4]=-(uex[9].t[1]*SG330[1][1]) - uex[9].t[2]*SG330[1][2] - uex[9].t[3]*SG330[1][3];
fex33[5]=-(uex[9].t[1]*SG330[2][1]) - uex[9].t[2]*SG330[2][2] - uex[9].t[3]*SG330[2][3];
fex33[6]=-(uex[9].t[1]*SG330[3][1]) - uex[9].t[2]*SG330[3][2] - uex[9].t[3]*SG330[3][3];

fex34[1]=-(uex[10].f[1]*SG340[1][1]) - uex[10].f[2]*SG340[1][2] - uex[10].f[3]*SG340[1][3];
fex34[2]=-(uex[10].f[1]*SG340[2][1]) - uex[10].f[2]*SG340[2][2] - uex[10].f[3]*SG340[2][3];
fex34[3]=-(uex[10].f[1]*SG340[3][1]) - uex[10].f[2]*SG340[3][2] - uex[10].f[3]*SG340[3][3];
fex34[4]=-(uex[10].t[1]*SG340[1][1]) - uex[10].t[2]*SG340[1][2] - uex[10].t[3]*SG340[1][3];
fex34[5]=-(uex[10].t[1]*SG340[2][1]) - uex[10].t[2]*SG340[2][2] - uex[10].t[3]*SG340[2][3];
fex34[6]=-(uex[10].t[1]*SG340[3][1]) - uex[10].t[2]*SG340[3][2] - uex[10].t[3]*SG340[3][3];

fex35[1]=-(uex[11].f[1]*SG350[1][1]) - uex[11].f[2]*SG350[1][2] - uex[11].f[3]*SG350[1][3];
fex35[2]=-(uex[11].f[1]*SG350[2][1]) - uex[11].f[2]*SG350[2][2] - uex[11].f[3]*SG350[2][3];
fex35[3]=-(uex[11].f[1]*SG350[3][1]) - uex[11].f[2]*SG350[3][2] - uex[11].f[3]*SG350[3][3];
fex35[4]=-(uex[11].t[1]*SG350[1][1]) - uex[11].t[2]*SG350[1][2] - uex[11].t[3]*SG350[1][3];
fex35[5]=-(uex[11].t[1]*SG350[2][1]) - uex[11].t[2]*SG350[2][2] - uex[11].t[3]*SG350[2][3];
fex35[6]=-(uex[11].t[1]*SG350[3][1]) - uex[11].t[2]*SG350[3][2] - uex[11].t[3]*SG350[3][3];

fex36[1]=-(uex[12].f[1]*SG360[1][1]) - uex[12].f[2]*SG360[1][2] - uex[12].f[3]*SG360[1][3];
fex36[2]=-(uex[12].f[1]*SG360[2][1]) - uex[12].f[2]*SG360[2][2] - uex[12].f[3]*SG360[2][3];
fex36[3]=-(uex[12].f[1]*SG360[3][1]) - uex[12].f[2]*SG360[3][2] - uex[12].f[3]*SG360[3][3];
fex36[4]=-(uex[12].t[1]*SG360[1][1]) - uex[12].t[2]*SG360[1][2] - uex[12].t[3]*SG360[1][3];
fex36[5]=-(uex[12].t[1]*SG360[2][1]) - uex[12].t[2]*SG360[2][2] - uex[12].t[3]*SG360[2][3];
fex36[6]=-(uex[12].t[1]*SG360[3][1]) - uex[12].t[2]*SG360[3][2] - uex[12].t[3]*SG360[3][3];

fex37[1]=-(uex[13].f[1]*SG370[1][1]) - uex[13].f[2]*SG370[1][2] - uex[13].f[3]*SG370[1][3];
fex37[2]=-(uex[13].f[1]*SG370[2][1]) - uex[13].f[2]*SG370[2][2] - uex[13].f[3]*SG370[2][3];
fex37[3]=-(uex[13].f[1]*SG370[3][1]) - uex[13].f[2]*SG370[3][2] - uex[13].f[3]*SG370[3][3];
fex37[4]=-(uex[13].t[1]*SG370[1][1]) - uex[13].t[2]*SG370[1][2] - uex[13].t[3]*SG370[1][3];
fex37[5]=-(uex[13].t[1]*SG370[2][1]) - uex[13].t[2]*SG370[2][2] - uex[13].t[3]*SG370[2][3];
fex37[6]=-(uex[13].t[1]*SG370[3][1]) - uex[13].t[2]*SG370[3][2] - uex[13].t[3]*SG370[3][3];

fex38[1]=-(uex[14].f[1]*SG380[1][1]) - uex[14].f[2]*SG380[1][2] - uex[14].f[3]*SG380[1][3];
fex38[2]=-(uex[14].f[1]*SG380[2][1]) - uex[14].f[2]*SG380[2][2] - uex[14].f[3]*SG380[2][3];
fex38[3]=-(uex[14].f[1]*SG380[3][1]) - uex[14].f[2]*SG380[3][2] - uex[14].f[3]*SG380[3][3];
fex38[4]=-(uex[14].t[1]*SG380[1][1]) - uex[14].t[2]*SG380[1][2] - uex[14].t[3]*SG380[1][3];
fex38[5]=-(uex[14].t[1]*SG380[2][1]) - uex[14].t[2]*SG380[2][2] - uex[14].t[3]*SG380[2][3];
fex38[6]=-(uex[14].t[1]*SG380[3][1]) - uex[14].t[2]*SG380[3][2] - uex[14].t[3]*SG380[3][3];

fex40[1]=-(uex[45].f[1]*SG400[1][1]) - uex[45].f[2]*SG400[1][2] - uex[45].f[3]*SG400[1][3];
fex40[2]=-(uex[45].f[1]*SG400[2][1]) - uex[45].f[2]*SG400[2][2] - uex[45].f[3]*SG400[2][3];
fex40[3]=-(uex[45].f[1]*SG400[3][1]) - uex[45].f[2]*SG400[3][2] - uex[45].f[3]*SG400[3][3];
fex40[4]=-(uex[45].t[1]*SG400[1][1]) - uex[45].t[2]*SG400[1][2] - uex[45].t[3]*SG400[1][3];
fex40[5]=-(uex[45].t[1]*SG400[2][1]) - uex[45].t[2]*SG400[2][2] - uex[45].t[3]*SG400[2][3];
fex40[6]=-(uex[45].t[1]*SG400[3][1]) - uex[45].t[2]*SG400[3][2] - uex[45].t[3]*SG400[3][3];

fex41[1]=-(uex[46].f[1]*SG410[1][1]) - uex[46].f[2]*SG410[1][2] - uex[46].f[3]*SG410[1][3];
fex41[2]=-(uex[46].f[1]*SG410[2][1]) - uex[46].f[2]*SG410[2][2] - uex[46].f[3]*SG410[2][3];
fex41[3]=-(uex[46].f[1]*SG410[3][1]) - uex[46].f[2]*SG410[3][2] - uex[46].f[3]*SG410[3][3];
fex41[4]=-(uex[46].t[1]*SG410[1][1]) - uex[46].t[2]*SG410[1][2] - uex[46].t[3]*SG410[1][3];
fex41[5]=-(uex[46].t[1]*SG410[2][1]) - uex[46].t[2]*SG410[2][2] - uex[46].t[3]*SG410[2][3];
fex41[6]=-(uex[46].t[1]*SG410[3][1]) - uex[46].t[2]*SG410[3][2] - uex[46].t[3]*SG410[3][3];

fex44[1]=-(uex[47].f[1]*SG440[1][1]) - uex[47].f[2]*SG440[1][2] - uex[47].f[3]*SG440[1][3];
fex44[2]=-(uex[47].f[1]*SG440[2][1]) - uex[47].f[2]*SG440[2][2] - uex[47].f[3]*SG440[2][3];
fex44[3]=-(uex[47].f[1]*SG440[3][1]) - uex[47].f[2]*SG440[3][2] - uex[47].f[3]*SG440[3][3];
fex44[4]=-(uex[47].t[1]*SG440[1][1]) - uex[47].t[2]*SG440[1][2] - uex[47].t[3]*SG440[1][3];
fex44[5]=-(uex[47].t[1]*SG440[2][1]) - uex[47].t[2]*SG440[2][2] - uex[47].t[3]*SG440[2][3];
fex44[6]=-(uex[47].t[1]*SG440[3][1]) - uex[47].t[2]*SG440[3][2] - uex[47].t[3]*SG440[3][3];

fex48[1]=-(uex[48].f[1]*SG480[1][1]) - uex[48].f[2]*SG480[1][2] - uex[48].f[3]*SG480[1][3];
fex48[2]=-(uex[48].f[1]*SG480[2][1]) - uex[48].f[2]*SG480[2][2] - uex[48].f[3]*SG480[2][3];
fex48[3]=-(uex[48].f[1]*SG480[3][1]) - uex[48].f[2]*SG480[3][2] - uex[48].f[3]*SG480[3][3];
fex48[4]=-(uex[48].t[1]*SG480[1][1]) - uex[48].t[2]*SG480[1][2] - uex[48].t[3]*SG480[1][3];
fex48[5]=-(uex[48].t[1]*SG480[2][1]) - uex[48].t[2]*SG480[2][2] - uex[48].t[3]*SG480[2][3];
fex48[6]=-(uex[48].t[1]*SG480[3][1]) - uex[48].t[2]*SG480[3][2] - uex[48].t[3]*SG480[3][3];

fex52[1]=-(uex[49].f[1]*SG520[1][1]) - uex[49].f[2]*SG520[1][2] - uex[49].f[3]*SG520[1][3];
fex52[2]=-(uex[49].f[1]*SG520[2][1]) - uex[49].f[2]*SG520[2][2] - uex[49].f[3]*SG520[2][3];
fex52[3]=-(uex[49].f[1]*SG520[3][1]) - uex[49].f[2]*SG520[3][2] - uex[49].f[3]*SG520[3][3];
fex52[4]=-(uex[49].t[1]*SG520[1][1]) - uex[49].t[2]*SG520[1][2] - uex[49].t[3]*SG520[1][3];
fex52[5]=-(uex[49].t[1]*SG520[2][1]) - uex[49].t[2]*SG520[2][2] - uex[49].t[3]*SG520[2][3];
fex52[6]=-(uex[49].t[1]*SG520[3][1]) - uex[49].t[2]*SG520[3][2] - uex[49].t[3]*SG520[3][3];

fex56[1]=-(uex[50].f[1]*SG560[1][1]) - uex[50].f[2]*SG560[1][2] - uex[50].f[3]*SG560[1][3];
fex56[2]=-(uex[50].f[1]*SG560[2][1]) - uex[50].f[2]*SG560[2][2] - uex[50].f[3]*SG560[2][3];
fex56[3]=-(uex[50].f[1]*SG560[3][1]) - uex[50].f[2]*SG560[3][2] - uex[50].f[3]*SG560[3][3];
fex56[4]=-(uex[50].t[1]*SG560[1][1]) - uex[50].t[2]*SG560[1][2] - uex[50].t[3]*SG560[1][3];
fex56[5]=-(uex[50].t[1]*SG560[2][1]) - uex[50].t[2]*SG560[2][2] - uex[50].t[3]*SG560[2][3];
fex56[6]=-(uex[50].t[1]*SG560[3][1]) - uex[50].t[2]*SG560[3][2] - uex[50].t[3]*SG560[3][3];

fex62[1]=-(uex[34].f[1]*SG620[1][1]) - uex[34].f[2]*SG620[1][2] - uex[34].f[3]*SG620[1][3];
fex62[2]=-(uex[34].f[1]*SG620[2][1]) - uex[34].f[2]*SG620[2][2] - uex[34].f[3]*SG620[2][3];
fex62[3]=-(uex[34].f[1]*SG620[3][1]) - uex[34].f[2]*SG620[3][2] - uex[34].f[3]*SG620[3][3];
fex62[4]=-(uex[34].t[1]*SG620[1][1]) - uex[34].t[2]*SG620[1][2] - uex[34].t[3]*SG620[1][3];
fex62[5]=-(uex[34].t[1]*SG620[2][1]) - uex[34].t[2]*SG620[2][2] - uex[34].t[3]*SG620[2][3];
fex62[6]=-(uex[34].t[1]*SG620[3][1]) - uex[34].t[2]*SG620[3][2] - uex[34].t[3]*SG620[3][3];

fex70[1]=-(uex[23].f[1]*SG700[1][1]) - uex[23].f[2]*SG700[1][2] - uex[23].f[3]*SG700[1][3];
fex70[2]=-(uex[23].f[1]*SG700[2][1]) - uex[23].f[2]*SG700[2][2] - uex[23].f[3]*SG700[2][3];
fex70[3]=-(uex[23].f[1]*SG700[3][1]) - uex[23].f[2]*SG700[3][2] - uex[23].f[3]*SG700[3][3];
fex70[4]=-(uex[23].t[1]*SG700[1][1]) - uex[23].t[2]*SG700[1][2] - uex[23].t[3]*SG700[1][3];
fex70[5]=-(uex[23].t[1]*SG700[2][1]) - uex[23].t[2]*SG700[2][2] - uex[23].t[3]*SG700[2][3];
fex70[6]=-(uex[23].t[1]*SG700[3][1]) - uex[23].t[2]*SG700[3][2] - uex[23].t[3]*SG700[3][3];

fex73[1]=-(uex[25].f[1]*SG730[1][1]) - uex[25].f[2]*SG730[1][2] - uex[25].f[3]*SG730[1][3];
fex73[2]=-(uex[25].f[1]*SG730[2][1]) - uex[25].f[2]*SG730[2][2] - uex[25].f[3]*SG730[2][3];
fex73[3]=-(uex[25].f[1]*SG730[3][1]) - uex[25].f[2]*SG730[3][2] - uex[25].f[3]*SG730[3][3];
fex73[4]=-(uex[25].t[1]*SG730[1][1]) - uex[25].t[2]*SG730[1][2] - uex[25].t[3]*SG730[1][3];
fex73[5]=-(uex[25].t[1]*SG730[2][1]) - uex[25].t[2]*SG730[2][2] - uex[25].t[3]*SG730[2][3];
fex73[6]=-(uex[25].t[1]*SG730[3][1]) - uex[25].t[2]*SG730[3][2] - uex[25].t[3]*SG730[3][3];

fex75[1]=-(uex[27].f[1]*SG750[1][1]) - uex[27].f[2]*SG750[1][2] - uex[27].f[3]*SG750[1][3];
fex75[2]=-(uex[27].f[1]*SG750[2][1]) - uex[27].f[2]*SG750[2][2] - uex[27].f[3]*SG750[2][3];
fex75[3]=-(uex[27].f[1]*SG750[3][1]) - uex[27].f[2]*SG750[3][2] - uex[27].f[3]*SG750[3][3];
fex75[4]=-(uex[27].t[1]*SG750[1][1]) - uex[27].t[2]*SG750[1][2] - uex[27].t[3]*SG750[1][3];
fex75[5]=-(uex[27].t[1]*SG750[2][1]) - uex[27].t[2]*SG750[2][2] - uex[27].t[3]*SG750[2][3];
fex75[6]=-(uex[27].t[1]*SG750[3][1]) - uex[27].t[2]*SG750[3][2] - uex[27].t[3]*SG750[3][3];

fex76[1]=-(uex[28].f[1]*SG760[1][1]) - uex[28].f[2]*SG760[1][2] - uex[28].f[3]*SG760[1][3];
fex76[2]=-(uex[28].f[1]*SG760[2][1]) - uex[28].f[2]*SG760[2][2] - uex[28].f[3]*SG760[2][3];
fex76[3]=-(uex[28].f[1]*SG760[3][1]) - uex[28].f[2]*SG760[3][2] - uex[28].f[3]*SG760[3][3];
fex76[4]=-(uex[28].t[1]*SG760[1][1]) - uex[28].t[2]*SG760[1][2] - uex[28].t[3]*SG760[1][3];
fex76[5]=-(uex[28].t[1]*SG760[2][1]) - uex[28].t[2]*SG760[2][2] - uex[28].t[3]*SG760[2][3];
fex76[6]=-(uex[28].t[1]*SG760[3][1]) - uex[28].t[2]*SG760[3][2] - uex[28].t[3]*SG760[3][3];

fex84[1]=-(uex[16].f[1]*SG840[1][1]) - uex[16].f[2]*SG840[1][2] - uex[16].f[3]*SG840[1][3];
fex84[2]=-(uex[16].f[1]*SG840[2][1]) - uex[16].f[2]*SG840[2][2] - uex[16].f[3]*SG840[2][3];
fex84[3]=-(uex[16].f[1]*SG840[3][1]) - uex[16].f[2]*SG840[3][2] - uex[16].f[3]*SG840[3][3];
fex84[4]=-(uex[16].t[1]*SG840[1][1]) - uex[16].t[2]*SG840[1][2] - uex[16].t[3]*SG840[1][3];
fex84[5]=-(uex[16].t[1]*SG840[2][1]) - uex[16].t[2]*SG840[2][2] - uex[16].t[3]*SG840[2][3];
fex84[6]=-(uex[16].t[1]*SG840[3][1]) - uex[16].t[2]*SG840[3][2] - uex[16].t[3]*SG840[3][3];

fex87[1]=-(uex[18].f[1]*SG870[1][1]) - uex[18].f[2]*SG870[1][2] - uex[18].f[3]*SG870[1][3];
fex87[2]=-(uex[18].f[1]*SG870[2][1]) - uex[18].f[2]*SG870[2][2] - uex[18].f[3]*SG870[2][3];
fex87[3]=-(uex[18].f[1]*SG870[3][1]) - uex[18].f[2]*SG870[3][2] - uex[18].f[3]*SG870[3][3];
fex87[4]=-(uex[18].t[1]*SG870[1][1]) - uex[18].t[2]*SG870[1][2] - uex[18].t[3]*SG870[1][3];
fex87[5]=-(uex[18].t[1]*SG870[2][1]) - uex[18].t[2]*SG870[2][2] - uex[18].t[3]*SG870[2][3];
fex87[6]=-(uex[18].t[1]*SG870[3][1]) - uex[18].t[2]*SG870[3][2] - uex[18].t[3]*SG870[3][3];

fex89[1]=-(uex[20].f[1]*SG890[1][1]) - uex[20].f[2]*SG890[1][2] - uex[20].f[3]*SG890[1][3];
fex89[2]=-(uex[20].f[1]*SG890[2][1]) - uex[20].f[2]*SG890[2][2] - uex[20].f[3]*SG890[2][3];
fex89[3]=-(uex[20].f[1]*SG890[3][1]) - uex[20].f[2]*SG890[3][2] - uex[20].f[3]*SG890[3][3];
fex89[4]=-(uex[20].t[1]*SG890[1][1]) - uex[20].t[2]*SG890[1][2] - uex[20].t[3]*SG890[1][3];
fex89[5]=-(uex[20].t[1]*SG890[2][1]) - uex[20].t[2]*SG890[2][2] - uex[20].t[3]*SG890[2][3];
fex89[6]=-(uex[20].t[1]*SG890[3][1]) - uex[20].t[2]*SG890[3][2] - uex[20].t[3]*SG890[3][3];

fex90[1]=-(uex[21].f[1]*SG900[1][1]) - uex[21].f[2]*SG900[1][2] - uex[21].f[3]*SG900[1][3];
fex90[2]=-(uex[21].f[1]*SG900[2][1]) - uex[21].f[2]*SG900[2][2] - uex[21].f[3]*SG900[2][3];
fex90[3]=-(uex[21].f[1]*SG900[3][1]) - uex[21].f[2]*SG900[3][2] - uex[21].f[3]*SG900[3][3];
fex90[4]=-(uex[21].t[1]*SG900[1][1]) - uex[21].t[2]*SG900[1][2] - uex[21].t[3]*SG900[1][3];
fex90[5]=-(uex[21].t[1]*SG900[2][1]) - uex[21].t[2]*SG900[2][2] - uex[21].t[3]*SG900[2][3];
fex90[6]=-(uex[21].t[1]*SG900[3][1]) - uex[21].t[2]*SG900[3][2] - uex[21].t[3]*SG900[3][3];



}


void
hermes_InvDynNEfunc7(void)
     {
/* total forces and external forces for each joint */
f97[1]=fnet97[1];
f97[2]=fnet97[2];
f97[3]=fnet97[3];
f97[4]=fnet97[4];
f97[5]=fnet97[5];
f97[6]=fnet97[6];

f90[1]=fnet90[1] + f97[1]*Si9097[1][1] + f97[2]*Si9097[1][2] + f97[3]*Si9097[1][3];
f90[2]=fnet90[2] + f97[1]*Si9097[2][1] + f97[2]*Si9097[2][2] + f97[3]*Si9097[2][3];
f90[3]=fnet90[3] + f97[1]*Si9097[3][1] + f97[2]*Si9097[3][2] + f97[3]*Si9097[3][3];
f90[4]=fnet90[4] + f97[4]*Si9097[1][1] + f97[5]*Si9097[1][2] + f97[6]*Si9097[1][3] + f97[1]*(-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1]) + f97[2]*(-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2]) + f97[3]*(-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3]);
f90[5]=fnet90[5] + f97[4]*Si9097[2][1] + f97[5]*Si9097[2][2] + f97[6]*Si9097[2][3] + f97[1]*(eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1]) + f97[2]*(eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2]) + f97[3]*(eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3]);
f90[6]=fnet90[6] + f97[1]*(-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1]) + f97[2]*(-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2]) + f97[3]*(-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3]) + f97[4]*Si9097[3][1] + f97[5]*Si9097[3][2] + f97[6]*Si9097[3][3];

f89[1]=fnet89[1] + f90[1]*Si8990[1][1] + f90[2]*Si8990[1][2];
f89[2]=-f90[3] + fnet89[2];
f89[3]=fnet89[3] + f90[1]*Si8990[3][1] + f90[2]*Si8990[3][2];
f89[4]=fnet89[4] + f90[4]*Si8990[1][1] + f90[5]*Si8990[1][2];
f89[5]=-f90[6] + fnet89[5];
f89[6]=fnet89[6] + f90[4]*Si8990[3][1] + f90[5]*Si8990[3][2];

f88[1]=fnet88[1] + f89[1]*Si8889[1][1] + f89[2]*Si8889[1][2];
f88[2]=f89[3] + fnet88[2];
f88[3]=fnet88[3] + f89[1]*Si8889[3][1] + f89[2]*Si8889[3][2];
f88[4]=LOWERLEG*f89[3] + fnet88[4] + f89[4]*Si8889[1][1] + f89[5]*Si8889[1][2];
f88[5]=f89[6] + fnet88[5] - LOWERLEG*f89[1]*Si8889[1][1] - LOWERLEG*f89[2]*Si8889[1][2];
f88[6]=fnet88[6] + f89[4]*Si8889[3][1] + f89[5]*Si8889[3][2];

f87[1]=fnet87[1] + f88[1]*Si8788[1][1] + f88[2]*Si8788[1][2];
f87[2]=f88[3] + fnet87[2];
f87[3]=fnet87[3] + f88[1]*Si8788[3][1] + f88[2]*Si8788[3][2];
f87[4]=fnet87[4] + f88[4]*Si8788[1][1] + f88[5]*Si8788[1][2];
f87[5]=f88[6] + fnet87[5];
f87[6]=fnet87[6] + f88[4]*Si8788[3][1] + f88[5]*Si8788[3][2];

f86[1]=fnet86[1] + f87[1]*Si8687[1][1] + f87[2]*Si8687[1][2];
f86[2]=-f87[3] + fnet86[2];
f86[3]=fnet86[3] + f87[1]*Si8687[3][1] + f87[2]*Si8687[3][2];
f86[4]=-(UPPERLEGMOD*f87[3]) + fnet86[4] + f87[4]*Si8687[1][1] + f87[5]*Si8687[1][2];
f86[5]=-f87[6] + fnet86[5] + f87[1]*(-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1]) + f87[2]*(-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2]);
f86[6]=-(YKNEE*f87[3]) + fnet86[6] + f87[4]*Si8687[3][1] + f87[5]*Si8687[3][2];

f85[1]=fnet85[1] + f86[1]*Si8586[1][1] + f86[2]*Si8586[1][2] + f86[3]*Si8586[1][3];
f85[2]=fnet85[2] + f86[1]*Si8586[2][1] + f86[2]*Si8586[2][2] + f86[3]*Si8586[2][3];
f85[3]=fnet85[3] + f86[1]*Si8586[3][1] + f86[2]*Si8586[3][2];
f85[4]=fnet85[4] + f86[4]*Si8586[1][1] + f86[5]*Si8586[1][2] + f86[6]*Si8586[1][3];
f85[5]=fnet85[5] + f86[4]*Si8586[2][1] + f86[5]*Si8586[2][2] + f86[6]*Si8586[2][3] - YHIP*f86[1]*Si8586[3][1] - YHIP*f86[2]*Si8586[3][2];
f85[6]=fnet85[6] + YHIP*f86[1]*Si8586[2][1] + YHIP*f86[2]*Si8586[2][2] + YHIP*f86[3]*Si8586[2][3] + f86[4]*Si8586[3][1] + f86[5]*Si8586[3][2];

f84[1]=fnet84[1] + f85[1]*Si8485[1][1] + f85[2]*Si8485[1][2];
f84[2]=f85[3] + fnet84[2];
f84[3]=fnet84[3] + f85[1]*Si8485[3][1] + f85[2]*Si8485[3][2];
f84[4]=fnet84[4] + f85[4]*Si8485[1][1] + f85[5]*Si8485[1][2];
f84[5]=f85[6] + fnet84[5];
f84[6]=fnet84[6] + f85[4]*Si8485[3][1] + f85[5]*Si8485[3][2];

f83[1]=fnet83[1];
f83[2]=fnet83[2];
f83[3]=fnet83[3];
f83[4]=fnet83[4];
f83[5]=fnet83[5];
f83[6]=fnet83[6];

f76[1]=fnet76[1] + f83[1]*Si7683[1][1] + f83[2]*Si7683[1][2] + f83[3]*Si7683[1][3];
f76[2]=fnet76[2] + f83[1]*Si7683[2][1] + f83[2]*Si7683[2][2] + f83[3]*Si7683[2][3];
f76[3]=fnet76[3] + f83[1]*Si7683[3][1] + f83[2]*Si7683[3][2] + f83[3]*Si7683[3][3];
f76[4]=fnet76[4] + f83[4]*Si7683[1][1] + f83[5]*Si7683[1][2] + f83[6]*Si7683[1][3] + f83[1]*(-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1]) + f83[2]*(-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2]) + f83[3]*(-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3]);
f76[5]=fnet76[5] + f83[4]*Si7683[2][1] + f83[5]*Si7683[2][2] + f83[6]*Si7683[2][3] + f83[1]*(eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1]) + f83[2]*(eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2]) + f83[3]*(eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3]);
f76[6]=fnet76[6] + f83[1]*(-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1]) + f83[2]*(-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2]) + f83[3]*(-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3]) + f83[4]*Si7683[3][1] + f83[5]*Si7683[3][2] + f83[6]*Si7683[3][3];

f75[1]=fnet75[1] + f76[1]*Si7576[1][1] + f76[2]*Si7576[1][2];
f75[2]=f76[3] + fnet75[2];
f75[3]=fnet75[3] + f76[1]*Si7576[3][1] + f76[2]*Si7576[3][2];
f75[4]=fnet75[4] + f76[4]*Si7576[1][1] + f76[5]*Si7576[1][2];
f75[5]=f76[6] + fnet75[5];
f75[6]=fnet75[6] + f76[4]*Si7576[3][1] + f76[5]*Si7576[3][2];

f74[1]=fnet74[1] + f75[1]*Si7475[1][1] + f75[2]*Si7475[1][2];
f74[2]=-f75[3] + fnet74[2];
f74[3]=fnet74[3] + f75[1]*Si7475[3][1] + f75[2]*Si7475[3][2];
f74[4]=LOWERLEG*f75[3] + fnet74[4] + f75[4]*Si7475[1][1] + f75[5]*Si7475[1][2];
f74[5]=-f75[6] + fnet74[5] + LOWERLEG*f75[1]*Si7475[1][1] + LOWERLEG*f75[2]*Si7475[1][2];
f74[6]=fnet74[6] + f75[4]*Si7475[3][1] + f75[5]*Si7475[3][2];

f73[1]=fnet73[1] + f74[1]*Si7374[1][1] + f74[2]*Si7374[1][2];
f73[2]=-f74[3] + fnet73[2];
f73[3]=fnet73[3] + f74[1]*Si7374[3][1] + f74[2]*Si7374[3][2];
f73[4]=fnet73[4] + f74[4]*Si7374[1][1] + f74[5]*Si7374[1][2];
f73[5]=-f74[6] + fnet73[5];
f73[6]=fnet73[6] + f74[4]*Si7374[3][1] + f74[5]*Si7374[3][2];

f72[1]=fnet72[1] + f73[1]*Si7273[1][1] + f73[2]*Si7273[1][2];
f72[2]=f73[3] + fnet72[2];
f72[3]=fnet72[3] + f73[1]*Si7273[3][1] + f73[2]*Si7273[3][2];
f72[4]=-(UPPERLEGMOD*f73[3]) + fnet72[4] + f73[4]*Si7273[1][1] + f73[5]*Si7273[1][2];
f72[5]=f73[6] + fnet72[5] + f73[1]*(UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1]) + f73[2]*(UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2]);
f72[6]=YKNEE*f73[3] + fnet72[6] + f73[4]*Si7273[3][1] + f73[5]*Si7273[3][2];

f71[1]=fnet71[1] + f72[1]*Si7172[1][1] + f72[2]*Si7172[1][2] + f72[3]*Si7172[1][3];
f71[2]=fnet71[2] + f72[1]*Si7172[2][1] + f72[2]*Si7172[2][2] + f72[3]*Si7172[2][3];
f71[3]=fnet71[3] + f72[1]*Si7172[3][1] + f72[2]*Si7172[3][2];
f71[4]=fnet71[4] + f72[4]*Si7172[1][1] + f72[5]*Si7172[1][2] + f72[6]*Si7172[1][3];
f71[5]=fnet71[5] + f72[4]*Si7172[2][1] + f72[5]*Si7172[2][2] + f72[6]*Si7172[2][3] - YHIP*f72[1]*Si7172[3][1] - YHIP*f72[2]*Si7172[3][2];
f71[6]=fnet71[6] + YHIP*f72[1]*Si7172[2][1] + YHIP*f72[2]*Si7172[2][2] + YHIP*f72[3]*Si7172[2][3] + f72[4]*Si7172[3][1] + f72[5]*Si7172[3][2];

f70[1]=fnet70[1] + f71[1]*Si7071[1][1] + f71[2]*Si7071[1][2];
f70[2]=-f71[3] + fnet70[2];
f70[3]=fnet70[3] + f71[1]*Si7071[3][1] + f71[2]*Si7071[3][2];
f70[4]=fnet70[4] + f71[4]*Si7071[1][1] + f71[5]*Si7071[1][2];
f70[5]=-f71[6] + fnet70[5];
f70[6]=fnet70[6] + f71[4]*Si7071[3][1] + f71[5]*Si7071[3][2];

f67[1]=fnet67[1];
f67[2]=fnet67[2];
f67[3]=fnet67[3];
f67[4]=fnet67[4];
f67[5]=fnet67[5];
f67[6]=fnet67[6];

f66[1]=-f67[3] + fnet66[1];
f66[2]=fnet66[2] + f67[1]*Si6667[2][1] + f67[2]*Si6667[2][2];
f66[3]=fnet66[3] + f67[1]*Si6667[3][1] + f67[2]*Si6667[3][2];
f66[4]=-f67[6] + fnet66[4];
f66[5]=fnet66[5] + f67[4]*Si6667[2][1] + f67[5]*Si6667[2][2];
f66[6]=fnet66[6] + f67[4]*Si6667[3][1] + f67[5]*Si6667[3][2];

f64[1]=fnet64[1];
f64[2]=fnet64[2];
f64[3]=fnet64[3];
f64[4]=fnet64[4];
f64[5]=fnet64[5];
f64[6]=fnet64[6];

f63[1]=-f64[3] + fnet63[1];
f63[2]=fnet63[2] + f64[1]*Si6364[2][1] + f64[2]*Si6364[2][2];
f63[3]=fnet63[3] + f64[1]*Si6364[3][1] + f64[2]*Si6364[3][2];
f63[4]=-f64[6] + fnet63[4];
f63[5]=fnet63[5] + f64[4]*Si6364[2][1] + f64[5]*Si6364[2][2];
f63[6]=fnet63[6] + f64[4]*Si6364[3][1] + f64[5]*Si6364[3][2];

f62[1]=fnet62[1] + f63[1]*Si6263[1][1] + f63[2]*Si6263[1][2] + f66[1]*Si6266[1][1] + f66[2]*Si6266[1][2];
f62[2]=fnet62[2] + f63[1]*Si6263[2][1] + f63[2]*Si6263[2][2] + f66[1]*Si6266[2][1] + f66[2]*Si6266[2][2];
f62[3]=f63[3] + f66[3] + fnet62[3];
f62[4]=-(EYEYOFF*f63[3]) - EYEYOFF*f66[3] + fnet62[4] + f63[4]*Si6263[1][1] + f63[5]*Si6263[1][2] + HEAD*f63[1]*Si6263[2][1] + HEAD*f63[2]*Si6263[2][2] + f66[4]*Si6266[1][1] + f66[5]*Si6266[1][2] + HEAD*f66[1]*Si6266[2][1] + HEAD*f66[2]*Si6266[2][2];
f62[5]=-(EYEXOFF*f63[3]) + EYEXOFF*f66[3] + fnet62[5] - HEAD*f63[1]*Si6263[1][1] - HEAD*f63[2]*Si6263[1][2] + f63[4]*Si6263[2][1] + f63[5]*Si6263[2][2] - HEAD*f66[1]*Si6266[1][1] - HEAD*f66[2]*Si6266[1][2] + f66[4]*Si6266[2][1] + f66[5]*Si6266[2][2];
f62[6]=f63[6] + f66[6] + fnet62[6] + f63[1]*(EYEYOFF*Si6263[1][1] + EYEXOFF*Si6263[2][1]) + f63[2]*(EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2]) + f66[1]*(EYEYOFF*Si6266[1][1] - EYEXOFF*Si6266[2][1]) + f66[2]*(EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2]);

f61[1]=fnet61[1] + f62[1]*Si6162[1][1] + f62[2]*Si6162[1][2];
f61[2]=f62[3] + fnet61[2];
f61[3]=fnet61[3] + f62[1]*Si6162[3][1] + f62[2]*Si6162[3][2];
f61[4]=fnet61[4] + f62[4]*Si6162[1][1] + f62[5]*Si6162[1][2];
f61[5]=f62[6] + fnet61[5];
f61[6]=fnet61[6] + f62[4]*Si6162[3][1] + f62[5]*Si6162[3][2];

f60[1]=f61[3] + fnet60[1];
f60[2]=fnet60[2] + f61[1]*Si6061[2][1] + f61[2]*Si6061[2][2];
f60[3]=fnet60[3] + f61[1]*Si6061[3][1] + f61[2]*Si6061[3][2];
f60[4]=f61[6] + fnet60[4] - CERVICAL*f61[1]*Si6061[3][1] - CERVICAL*f61[2]*Si6061[3][2];
f60[5]=fnet60[5] + f61[4]*Si6061[2][1] + f61[5]*Si6061[2][2];
f60[6]=CERVICAL*f61[3] + fnet60[6] + f61[4]*Si6061[3][1] + f61[5]*Si6061[3][2];

f56[1]=fnet56[1];
f56[2]=fnet56[2];
f56[3]=fnet56[3];
f56[4]=fnet56[4];
f56[5]=fnet56[5];
f56[6]=fnet56[6];

f52[1]=fnet52[1];
f52[2]=fnet52[2];
f52[3]=fnet52[3];
f52[4]=fnet52[4];
f52[5]=fnet52[5];
f52[6]=fnet52[6];

f48[1]=fnet48[1];
f48[2]=fnet48[2];
f48[3]=fnet48[3];
f48[4]=fnet48[4];
f48[5]=fnet48[5];
f48[6]=fnet48[6];

f44[1]=fnet44[1];
f44[2]=fnet44[2];
f44[3]=fnet44[3];
f44[4]=fnet44[4];
f44[5]=fnet44[5];
f44[6]=fnet44[6];

f41[1]=fnet41[1];
f41[2]=fnet41[2];
f41[3]=fnet41[3];
f41[4]=fnet41[4];
f41[5]=fnet41[5];
f41[6]=fnet41[6];

f40[1]=fnet40[1] + f41[1]*Si4041[1][1] + f41[2]*Si4041[1][2];
f40[2]=fnet40[2] + f41[1]*Si4041[2][1] + f41[2]*Si4041[2][2];
f40[3]=f41[3] + fnet40[3];
f40[4]=YTHUMBFLEX*f41[3] + fnet40[4] + f41[4]*Si4041[1][1] + f41[5]*Si4041[1][2];
f40[5]=-(XTHUMBFLEX*f41[3]) + fnet40[5] + f41[4]*Si4041[2][1] + f41[5]*Si4041[2][2];
f40[6]=f41[6] + fnet40[6] + f41[1]*(-(YTHUMBFLEX*Si4041[1][1]) + XTHUMBFLEX*Si4041[2][1]) + f41[2]*(-(YTHUMBFLEX*Si4041[1][2]) + XTHUMBFLEX*Si4041[2][2]);

f39[1]=fnet39[1];
f39[2]=fnet39[2];
f39[3]=fnet39[3];
f39[4]=fnet39[4];
f39[5]=fnet39[5];
f39[6]=fnet39[6];

f38[1]=fnet38[1] + f39[1]*Si3839[1][1] + f39[2]*Si3839[1][2] + f39[3]*Si3839[1][3] + f40[1]*Si3840[1][1] + f40[2]*Si3840[1][2] + f40[3]*Si3840[1][3] + f44[1]*Si3844[1][1] + f44[2]*Si3844[1][2] + f48[1]*Si3848[1][1] + f48[2]*Si3848[1][2] + f52[1]*Si3852[1][1] + f52[2]*Si3852[1][2] + f56[1]*Si3856[1][1] + f56[2]*Si3856[1][2];
f38[2]=fnet38[2] + f39[1]*Si3839[2][1] + f39[2]*Si3839[2][2] + f39[3]*Si3839[2][3] + f40[1]*Si3840[2][1] + f40[2]*Si3840[2][2] + f40[3]*Si3840[2][3] + f44[1]*Si3844[2][1] + f44[2]*Si3844[2][2] + f44[3]*Si3844[2][3] + f48[1]*Si3848[2][1] + f48[2]*Si3848[2][2] + f48[3]*Si3848[2][3] + f52[1]*Si3852[2][1] + f52[2]*Si3852[2][2] + f52[3]*Si3852[2][3] + f56[1]*Si3856[2][1] + f56[2]*Si3856[2][2] + f56[3]*Si3856[2][3];
f38[3]=fnet38[3] + f39[1]*Si3839[3][1] + f39[2]*Si3839[3][2] + f39[3]*Si3839[3][3] + f40[1]*Si3840[3][1] + f40[2]*Si3840[3][2] + f40[3]*Si3840[3][3] + f44[1]*Si3844[3][1] + f44[2]*Si3844[3][2] + f44[3]*Si3844[3][3] + f48[1]*Si3848[3][1] + f48[2]*Si3848[3][2] + f48[3]*Si3848[3][3] + f52[1]*Si3852[3][1] + f52[2]*Si3852[3][2] + f52[3]*Si3852[3][3] + f56[1]*Si3856[3][1] + f56[2]*Si3856[3][2] + f56[3]*Si3856[3][3];
f38[4]=fnet38[4] + f39[4]*Si3839[1][1] + f39[5]*Si3839[1][2] + f39[6]*Si3839[1][3] + f39[1]*(-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1]) + f39[2]*(-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2]) + f39[3]*(-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3]) + f40[4]*Si3840[1][1] + f40[5]*Si3840[1][2] + f40[6]*Si3840[1][3] + f40[1]*(-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1]) + f40[2]*(-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2]) + f40[3]*(-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3]) + f44[4]*Si3844[1][1] + f44[5]*Si3844[1][2] + f44[1]*(-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1]) + f44[2]*(-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2]) + f44[3]*(-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3]) + f48[4]*Si3848[1][1] + f48[5]*Si3848[1][2] + f48[1]*(-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1]) + f48[2]*(-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2]) + f48[3]*(-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3]) + f52[4]*Si3852[1][1] + f52[5]*Si3852[1][2] + f52[1]*(-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1]) + f52[2]*(-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2]) + f52[3]*(-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3]) + f56[4]*Si3856[1][1] + f56[5]*Si3856[1][2] + f56[1]*(-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1]) + f56[2]*(-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2]) + f56[3]*(-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3]);
f38[5]=fnet38[5] + f39[4]*Si3839[2][1] + f39[5]*Si3839[2][2] + f39[6]*Si3839[2][3] + f39[1]*(eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1]) + f39[2]*(eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2]) + f39[3]*(eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3]) + f40[4]*Si3840[2][1] + f40[5]*Si3840[2][2] + f40[6]*Si3840[2][3] + f40[1]*(ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1]) + f40[2]*(ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2]) + f40[3]*(ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3]) + f44[4]*Si3844[2][1] + f44[5]*Si3844[2][2] + f44[6]*Si3844[2][3] + f44[1]*(ZIF*Si3844[1][1] - XIF*Si3844[3][1]) + f44[2]*(ZIF*Si3844[1][2] - XIF*Si3844[3][2]) - XIF*f44[3]*Si3844[3][3] + f48[4]*Si3848[2][1] + f48[5]*Si3848[2][2] + f48[6]*Si3848[2][3] + f48[1]*(ZMF*Si3848[1][1] - XMF*Si3848[3][1]) + f48[2]*(ZMF*Si3848[1][2] - XMF*Si3848[3][2]) - XMF*f48[3]*Si3848[3][3] + f52[4]*Si3852[2][1] + f52[5]*Si3852[2][2] + f52[6]*Si3852[2][3] + f52[1]*(ZRF*Si3852[1][1] - XRF*Si3852[3][1]) + f52[2]*(ZRF*Si3852[1][2] - XRF*Si3852[3][2]) - XRF*f52[3]*Si3852[3][3] + f56[4]*Si3856[2][1] + f56[5]*Si3856[2][2] + f56[6]*Si3856[2][3] + f56[1]*(ZLF*Si3856[1][1] - XLF*Si3856[3][1]) + f56[2]*(ZLF*Si3856[1][2] - XLF*Si3856[3][2]) - XLF*f56[3]*Si3856[3][3];
f38[6]=fnet38[6] + f39[1]*(-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1]) + f39[2]*(-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2]) + f39[3]*(-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3]) + f39[4]*Si3839[3][1] + f39[5]*Si3839[3][2] + f39[6]*Si3839[3][3] + f40[1]*(-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1]) + f40[2]*(-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2]) + f40[3]*(-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3]) + f40[4]*Si3840[3][1] + f40[5]*Si3840[3][2] + f40[6]*Si3840[3][3] + f44[1]*(-(YIF*Si3844[1][1]) + XIF*Si3844[2][1]) + f44[2]*(-(YIF*Si3844[1][2]) + XIF*Si3844[2][2]) + XIF*f44[3]*Si3844[2][3] + f44[4]*Si3844[3][1] + f44[5]*Si3844[3][2] + f44[6]*Si3844[3][3] + f48[1]*(-(YMF*Si3848[1][1]) + XMF*Si3848[2][1]) + f48[2]*(-(YMF*Si3848[1][2]) + XMF*Si3848[2][2]) + XMF*f48[3]*Si3848[2][3] + f48[4]*Si3848[3][1] + f48[5]*Si3848[3][2] + f48[6]*Si3848[3][3] + f52[1]*(-(YRF*Si3852[1][1]) + XRF*Si3852[2][1]) + f52[2]*(-(YRF*Si3852[1][2]) + XRF*Si3852[2][2]) + XRF*f52[3]*Si3852[2][3] + f52[4]*Si3852[3][1] + f52[5]*Si3852[3][2] + f52[6]*Si3852[3][3] + f56[1]*(-(YLF*Si3856[1][1]) + XLF*Si3856[2][1]) + f56[2]*(-(YLF*Si3856[1][2]) + XLF*Si3856[2][2]) + XLF*f56[3]*Si3856[2][3] + f56[4]*Si3856[3][1] + f56[5]*Si3856[3][2] + f56[6]*Si3856[3][3];

f37[1]=fnet37[1] + f38[1]*Si3738[1][1] + f38[2]*Si3738[1][2];
f37[2]=f38[3] + fnet37[2];
f37[3]=fnet37[3] + f38[1]*Si3738[3][1] + f38[2]*Si3738[3][2];
f37[4]=fnet37[4] + f38[4]*Si3738[1][1] + f38[5]*Si3738[1][2];
f37[5]=f38[6] + fnet37[5];
f37[6]=fnet37[6] + f38[4]*Si3738[3][1] + f38[5]*Si3738[3][2];

f36[1]=-f37[3] + fnet36[1];
f36[2]=fnet36[2] + f37[1]*Si3637[2][1] + f37[2]*Si3637[2][2];
f36[3]=fnet36[3] + f37[1]*Si3637[3][1] + f37[2]*Si3637[3][2];
f36[4]=-f37[6] + fnet36[4] + f37[1]*(-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1]) + f37[2]*(-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2]);
f36[5]=-(LOWERARM*f37[3]) + fnet36[5] + f37[4]*Si3637[2][1] + f37[5]*Si3637[2][2];
f36[6]=WRISTY*f37[3] + fnet36[6] + f37[4]*Si3637[3][1] + f37[5]*Si3637[3][2];

f35[1]=fnet35[1] + f36[1]*Si3536[1][1] + f36[2]*Si3536[1][2];
f35[2]=-f36[3] + fnet35[2];
f35[3]=fnet35[3] + f36[1]*Si3536[3][1] + f36[2]*Si3536[3][2];
f35[4]=fnet35[4] + f36[4]*Si3536[1][1] + f36[5]*Si3536[1][2];
f35[5]=-f36[6] + fnet35[5];
f35[6]=fnet35[6] + f36[4]*Si3536[3][1] + f36[5]*Si3536[3][2];

f34[1]=-f35[3] + fnet34[1];
f34[2]=fnet34[2] + f35[1]*Si3435[2][1] + f35[2]*Si3435[2][2];
f34[3]=fnet34[3] + f35[1]*Si3435[3][1] + f35[2]*Si3435[3][2];
f34[4]=-f35[6] + fnet34[4] - UPPERARM*f35[1]*Si3435[2][1] - UPPERARM*f35[2]*Si3435[2][2];
f34[5]=-(UPPERARM*f35[3]) + fnet34[5] + f35[4]*Si3435[2][1] + f35[5]*Si3435[2][2];
f34[6]=fnet34[6] + f35[4]*Si3435[3][1] + f35[5]*Si3435[3][2];

f33[1]=fnet33[1] + f34[1]*Si3334[1][1] + f34[2]*Si3334[1][2];
f33[2]=-f34[3] + fnet33[2];
f33[3]=fnet33[3] + f34[1]*Si3334[3][1] + f34[2]*Si3334[3][2];
f33[4]=fnet33[4] + f34[4]*Si3334[1][1] + f34[5]*Si3334[1][2];
f33[5]=-f34[6] + fnet33[5] + SHOULDERY*f34[1]*Si3334[3][1] + SHOULDERY*f34[2]*Si3334[3][2];
f33[6]=SHOULDERY*f34[3] + fnet33[6] + f34[4]*Si3334[3][1] + f34[5]*Si3334[3][2];

f32[1]=fnet32[1] + f33[1]*Si3233[1][1] + f33[2]*Si3233[1][2];
f32[2]=f33[3] + fnet32[2];
f32[3]=fnet32[3] + f33[1]*Si3233[3][1] + f33[2]*Si3233[3][2];
f32[4]=-(SHOULDERX*f33[3]) + fnet32[4] + f33[4]*Si3233[1][1] + f33[5]*Si3233[1][2];
f32[5]=f33[6] + fnet32[5] + SHOULDERX*f33[1]*Si3233[1][1] + SHOULDERX*f33[2]*Si3233[1][2];
f32[6]=fnet32[6] + f33[4]*Si3233[3][1] + f33[5]*Si3233[3][2];

f28[1]=fnet28[1];
f28[2]=fnet28[2];
f28[3]=fnet28[3];
f28[4]=fnet28[4];
f28[5]=fnet28[5];
f28[6]=fnet28[6];

f24[1]=fnet24[1];
f24[2]=fnet24[2];
f24[3]=fnet24[3];
f24[4]=fnet24[4];
f24[5]=fnet24[5];
f24[6]=fnet24[6];

f20[1]=fnet20[1];
f20[2]=fnet20[2];
f20[3]=fnet20[3];
f20[4]=fnet20[4];
f20[5]=fnet20[5];
f20[6]=fnet20[6];

f16[1]=fnet16[1];
f16[2]=fnet16[2];
f16[3]=fnet16[3];
f16[4]=fnet16[4];
f16[5]=fnet16[5];
f16[6]=fnet16[6];

f13[1]=fnet13[1];
f13[2]=fnet13[2];
f13[3]=fnet13[3];
f13[4]=fnet13[4];
f13[5]=fnet13[5];
f13[6]=fnet13[6];

f12[1]=fnet12[1] + f13[1]*Si1213[1][1] + f13[2]*Si1213[1][2];
f12[2]=fnet12[2] + f13[1]*Si1213[2][1] + f13[2]*Si1213[2][2];
f12[3]=f13[3] + fnet12[3];
f12[4]=YTHUMBFLEX*f13[3] + fnet12[4] + f13[4]*Si1213[1][1] + f13[5]*Si1213[1][2];
f12[5]=-(XTHUMBFLEX*f13[3]) + fnet12[5] + f13[4]*Si1213[2][1] + f13[5]*Si1213[2][2];
f12[6]=f13[6] + fnet12[6] + f13[1]*(-(YTHUMBFLEX*Si1213[1][1]) + XTHUMBFLEX*Si1213[2][1]) + f13[2]*(-(YTHUMBFLEX*Si1213[1][2]) + XTHUMBFLEX*Si1213[2][2]);

f11[1]=fnet11[1];
f11[2]=fnet11[2];
f11[3]=fnet11[3];
f11[4]=fnet11[4];
f11[5]=fnet11[5];
f11[6]=fnet11[6];

f10[1]=fnet10[1] + f11[1]*Si1011[1][1] + f11[2]*Si1011[1][2] + f11[3]*Si1011[1][3] + f12[1]*Si1012[1][1] + f12[2]*Si1012[1][2] + f12[3]*Si1012[1][3] + f16[1]*Si1016[1][1] + f16[2]*Si1016[1][2] + f20[1]*Si1020[1][1] + f20[2]*Si1020[1][2] + f24[1]*Si1024[1][1] + f24[2]*Si1024[1][2] + f28[1]*Si1028[1][1] + f28[2]*Si1028[1][2];
f10[2]=fnet10[2] + f11[1]*Si1011[2][1] + f11[2]*Si1011[2][2] + f11[3]*Si1011[2][3] + f12[1]*Si1012[2][1] + f12[2]*Si1012[2][2] + f12[3]*Si1012[2][3] + f16[1]*Si1016[2][1] + f16[2]*Si1016[2][2] + f16[3]*Si1016[2][3] + f20[1]*Si1020[2][1] + f20[2]*Si1020[2][2] + f20[3]*Si1020[2][3] + f24[1]*Si1024[2][1] + f24[2]*Si1024[2][2] + f24[3]*Si1024[2][3] + f28[1]*Si1028[2][1] + f28[2]*Si1028[2][2] + f28[3]*Si1028[2][3];
f10[3]=fnet10[3] + f11[1]*Si1011[3][1] + f11[2]*Si1011[3][2] + f11[3]*Si1011[3][3] + f12[1]*Si1012[3][1] + f12[2]*Si1012[3][2] + f12[3]*Si1012[3][3] + f16[1]*Si1016[3][1] + f16[2]*Si1016[3][2] + f16[3]*Si1016[3][3] + f20[1]*Si1020[3][1] + f20[2]*Si1020[3][2] + f20[3]*Si1020[3][3] + f24[1]*Si1024[3][1] + f24[2]*Si1024[3][2] + f24[3]*Si1024[3][3] + f28[1]*Si1028[3][1] + f28[2]*Si1028[3][2] + f28[3]*Si1028[3][3];
f10[4]=fnet10[4] + f11[4]*Si1011[1][1] + f11[5]*Si1011[1][2] + f11[6]*Si1011[1][3] + f11[1]*(-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1]) + f11[2]*(-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2]) + f11[3]*(-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3]) + f12[4]*Si1012[1][1] + f12[5]*Si1012[1][2] + f12[6]*Si1012[1][3] + f12[1]*(ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1]) + f12[2]*(ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2]) + f12[3]*(ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3]) + f16[4]*Si1016[1][1] + f16[5]*Si1016[1][2] + f16[1]*(ZIF*Si1016[2][1] + YIF*Si1016[3][1]) + f16[2]*(ZIF*Si1016[2][2] + YIF*Si1016[3][2]) + f16[3]*(ZIF*Si1016[2][3] + YIF*Si1016[3][3]) + f20[4]*Si1020[1][1] + f20[5]*Si1020[1][2] + f20[1]*(ZMF*Si1020[2][1] + YMF*Si1020[3][1]) + f20[2]*(ZMF*Si1020[2][2] + YMF*Si1020[3][2]) + f20[3]*(ZMF*Si1020[2][3] + YMF*Si1020[3][3]) + f24[4]*Si1024[1][1] + f24[5]*Si1024[1][2] + f24[1]*(ZRF*Si1024[2][1] + YRF*Si1024[3][1]) + f24[2]*(ZRF*Si1024[2][2] + YRF*Si1024[3][2]) + f24[3]*(ZRF*Si1024[2][3] + YRF*Si1024[3][3]) + f28[4]*Si1028[1][1] + f28[5]*Si1028[1][2] + f28[1]*(ZLF*Si1028[2][1] + YLF*Si1028[3][1]) + f28[2]*(ZLF*Si1028[2][2] + YLF*Si1028[3][2]) + f28[3]*(ZLF*Si1028[2][3] + YLF*Si1028[3][3]);
f10[5]=fnet10[5] + f11[4]*Si1011[2][1] + f11[5]*Si1011[2][2] + f11[6]*Si1011[2][3] + f11[1]*(eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1]) + f11[2]*(eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2]) + f11[3]*(eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3]) + f12[4]*Si1012[2][1] + f12[5]*Si1012[2][2] + f12[6]*Si1012[2][3] + f12[1]*(-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1]) + f12[2]*(-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2]) + f12[3]*(-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3]) + f16[4]*Si1016[2][1] + f16[5]*Si1016[2][2] + f16[6]*Si1016[2][3] + f16[1]*(-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1]) + f16[2]*(-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2]) - XIF*f16[3]*Si1016[3][3] + f20[4]*Si1020[2][1] + f20[5]*Si1020[2][2] + f20[6]*Si1020[2][3] + f20[1]*(-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1]) + f20[2]*(-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2]) - XMF*f20[3]*Si1020[3][3] + f24[4]*Si1024[2][1] + f24[5]*Si1024[2][2] + f24[6]*Si1024[2][3] + f24[1]*(-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1]) + f24[2]*(-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2]) - XRF*f24[3]*Si1024[3][3] + f28[4]*Si1028[2][1] + f28[5]*Si1028[2][2] + f28[6]*Si1028[2][3] + f28[1]*(-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1]) + f28[2]*(-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2]) - XLF*f28[3]*Si1028[3][3];
f10[6]=fnet10[6] + f11[1]*(-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1]) + f11[2]*(-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2]) + f11[3]*(-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3]) + f11[4]*Si1011[3][1] + f11[5]*Si1011[3][2] + f11[6]*Si1011[3][3] + f12[1]*(-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1]) + f12[2]*(-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2]) + f12[3]*(-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3]) + f12[4]*Si1012[3][1] + f12[5]*Si1012[3][2] + f12[6]*Si1012[3][3] + f16[1]*(-(YIF*Si1016[1][1]) + XIF*Si1016[2][1]) + f16[2]*(-(YIF*Si1016[1][2]) + XIF*Si1016[2][2]) + XIF*f16[3]*Si1016[2][3] + f16[4]*Si1016[3][1] + f16[5]*Si1016[3][2] + f16[6]*Si1016[3][3] + f20[1]*(-(YMF*Si1020[1][1]) + XMF*Si1020[2][1]) + f20[2]*(-(YMF*Si1020[1][2]) + XMF*Si1020[2][2]) + XMF*f20[3]*Si1020[2][3] + f20[4]*Si1020[3][1] + f20[5]*Si1020[3][2] + f20[6]*Si1020[3][3] + f24[1]*(-(YRF*Si1024[1][1]) + XRF*Si1024[2][1]) + f24[2]*(-(YRF*Si1024[1][2]) + XRF*Si1024[2][2]) + XRF*f24[3]*Si1024[2][3] + f24[4]*Si1024[3][1] + f24[5]*Si1024[3][2] + f24[6]*Si1024[3][3] + f28[1]*(-(YLF*Si1028[1][1]) + XLF*Si1028[2][1]) + f28[2]*(-(YLF*Si1028[1][2]) + XLF*Si1028[2][2]) + XLF*f28[3]*Si1028[2][3] + f28[4]*Si1028[3][1] + f28[5]*Si1028[3][2] + f28[6]*Si1028[3][3];

f9[1]=fnet9[1] + f10[1]*Si910[1][1] + f10[2]*Si910[1][2];
f9[2]=-f10[3] + fnet9[2];
f9[3]=fnet9[3] + f10[1]*Si910[3][1] + f10[2]*Si910[3][2];
f9[4]=fnet9[4] + f10[4]*Si910[1][1] + f10[5]*Si910[1][2];
f9[5]=-f10[6] + fnet9[5];
f9[6]=fnet9[6] + f10[4]*Si910[3][1] + f10[5]*Si910[3][2];

f8[1]=f9[3] + fnet8[1];
f8[2]=fnet8[2] + f9[1]*Si89[2][1] + f9[2]*Si89[2][2];
f8[3]=fnet8[3] + f9[1]*Si89[3][1] + f9[2]*Si89[3][2];
f8[4]=f9[6] + fnet8[4] + f9[1]*(LOWERARM*Si89[2][1] + WRISTY*Si89[3][1]) + f9[2]*(LOWERARM*Si89[2][2] + WRISTY*Si89[3][2]);
f8[5]=-(LOWERARM*f9[3]) + fnet8[5] + f9[4]*Si89[2][1] + f9[5]*Si89[2][2];
f8[6]=-(WRISTY*f9[3]) + fnet8[6] + f9[4]*Si89[3][1] + f9[5]*Si89[3][2];

f7[1]=fnet7[1] + f8[1]*Si78[1][1] + f8[2]*Si78[1][2];
f7[2]=f8[3] + fnet7[2];
f7[3]=fnet7[3] + f8[1]*Si78[3][1] + f8[2]*Si78[3][2];
f7[4]=fnet7[4] + f8[4]*Si78[1][1] + f8[5]*Si78[1][2];
f7[5]=f8[6] + fnet7[5];
f7[6]=fnet7[6] + f8[4]*Si78[3][1] + f8[5]*Si78[3][2];

f6[1]=f7[3] + fnet6[1];
f6[2]=fnet6[2] + f7[1]*Si67[2][1] + f7[2]*Si67[2][2];
f6[3]=fnet6[3] + f7[1]*Si67[3][1] + f7[2]*Si67[3][2];
f6[4]=f7[6] + fnet6[4] + UPPERARM*f7[1]*Si67[2][1] + UPPERARM*f7[2]*Si67[2][2];
f6[5]=-(UPPERARM*f7[3]) + fnet6[5] + f7[4]*Si67[2][1] + f7[5]*Si67[2][2];
f6[6]=fnet6[6] + f7[4]*Si67[3][1] + f7[5]*Si67[3][2];

f5[1]=fnet5[1] + f6[1]*Si56[1][1] + f6[2]*Si56[1][2];
f5[2]=f6[3] + fnet5[2];
f5[3]=fnet5[3] + f6[1]*Si56[3][1] + f6[2]*Si56[3][2];
f5[4]=fnet5[4] + f6[4]*Si56[1][1] + f6[5]*Si56[1][2];
f5[5]=f6[6] + fnet5[5] + SHOULDERY*f6[1]*Si56[3][1] + SHOULDERY*f6[2]*Si56[3][2];
f5[6]=-(SHOULDERY*f6[3]) + fnet5[6] + f6[4]*Si56[3][1] + f6[5]*Si56[3][2];

f4[1]=fnet4[1] + f5[1]*Si45[1][1] + f5[2]*Si45[1][2];
f4[2]=-f5[3] + fnet4[2];
f4[3]=fnet4[3] + f5[1]*Si45[3][1] + f5[2]*Si45[3][2];
f4[4]=-(SHOULDERX*f5[3]) + fnet4[4] + f5[4]*Si45[1][1] + f5[5]*Si45[1][2];
f4[5]=-f5[6] + fnet4[5] - SHOULDERX*f5[1]*Si45[1][1] - SHOULDERX*f5[2]*Si45[1][2];
f4[6]=fnet4[6] + f5[4]*Si45[3][1] + f5[5]*Si45[3][2];

f3[1]=-0.7071067811865475*f32[3] + 0.7071067811865475*f4[3] + fnet3[1] + f32[1]*Si332[1][1] + f32[2]*Si332[1][2] + f4[1]*Si34[1][1] + f4[2]*Si34[1][2] + f60[1]*Si360[1][1] + f60[2]*Si360[1][2];
f3[2]=fnet3[2] + f32[1]*Si332[2][1] + f32[2]*Si332[2][2] + f4[1]*Si34[2][1] + f4[2]*Si34[2][2] + f60[1]*Si360[2][1] + f60[2]*Si360[2][2];
f3[3]=-0.7071067811865475*f32[3] - 0.7071067811865475*f4[3] + f60[3] + fnet3[3] + f32[1]*Si332[3][1] + f32[2]*Si332[3][2] + f4[1]*Si34[3][1] + f4[2]*Si34[3][2];
f3[4]=-0.7071067811865475*f32[6] + 0.7071067811865475*f4[6] + fnet3[4] + f32[4]*Si332[1][1] + f32[5]*Si332[1][2] + f4[4]*Si34[1][1] + f4[5]*Si34[1][2] + f60[4]*Si360[1][1] + f60[5]*Si360[1][2];
f3[5]=-0.7071067811865475*THORAX2SHOULDER*f32[3] - 0.7071067811865475*THORAX2SHOULDER*f4[3] + THORAX2NECK*f60[3] + fnet3[5] + f32[4]*Si332[2][1] + f32[5]*Si332[2][2] + THORAX2SHOULDER*f32[1]*Si332[3][1] + THORAX2SHOULDER*f32[2]*Si332[3][2] + f4[4]*Si34[2][1] + f4[5]*Si34[2][2] + THORAX2SHOULDER*f4[1]*Si34[3][1] + THORAX2SHOULDER*f4[2]*Si34[3][2] + f60[4]*Si360[2][1] + f60[5]*Si360[2][2];
f3[6]=-0.7071067811865475*f32[6] - 0.7071067811865475*f4[6] + f60[6] + fnet3[6] - THORAX2SHOULDER*f32[1]*Si332[2][1] - THORAX2SHOULDER*f32[2]*Si332[2][2] + f32[4]*Si332[3][1] + f32[5]*Si332[3][2] - THORAX2SHOULDER*f4[1]*Si34[2][1] - THORAX2SHOULDER*f4[2]*Si34[2][2] + f4[4]*Si34[3][1] + f4[5]*Si34[3][2] - THORAX2NECK*f60[1]*Si360[2][1] - THORAX2NECK*f60[2]*Si360[2][2];

f2[1]=fnet2[1] + f3[1]*Si23[1][1] + f3[2]*Si23[1][2];
f2[2]=f3[3] + fnet2[2];
f2[3]=fnet2[3] + f3[1]*Si23[3][1] + f3[2]*Si23[3][2];
f2[4]=fnet2[4] + f3[4]*Si23[1][1] + f3[5]*Si23[1][2];
f2[5]=f3[6] + fnet2[5];
f2[6]=fnet2[6] + f3[4]*Si23[3][1] + f3[5]*Si23[3][2];

f1[1]=fnet1[1] + f2[1]*Si12[1][1] + f2[2]*Si12[1][2];
f1[2]=-f2[3] + fnet1[2];
f1[3]=fnet1[3] + f2[1]*Si12[3][1] + f2[2]*Si12[3][2];
f1[4]=fnet1[4] + f2[4]*Si12[1][1] + f2[5]*Si12[1][2];
f1[5]=-f2[6] + fnet1[5];
f1[6]=fnet1[6] + f2[4]*Si12[3][1] + f2[5]*Si12[3][2];

f0[1]=fnet0[1] + f1[1]*Si01[1][1] + f1[2]*Si01[1][2] + f70[1]*Si070[1][1] + f70[2]*Si070[1][2] + f84[1]*Si084[1][1] + f84[2]*Si084[1][2];
f0[2]=f70[3] - f84[3] + fnet0[2] + f1[1]*Si01[2][1] + f1[2]*Si01[2][2];
f0[3]=-f1[3] + fnet0[3] + f70[1]*Si070[3][1] + f70[2]*Si070[3][2] + f84[1]*Si084[3][1] + f84[2]*Si084[3][2];
f0[4]=PELVISOFFSET*f1[3] + fnet0[4] + f1[4]*Si01[1][1] + f1[5]*Si01[1][2] - PELVIS2THORAX*f1[1]*Si01[2][1] - PELVIS2THORAX*f1[2]*Si01[2][2] + f70[4]*Si070[1][1] + f70[5]*Si070[1][2] + f84[4]*Si084[1][1] + f84[5]*Si084[1][2];
f0[5]=f70[6] - f84[6] + fnet0[5] + PELVIS2THORAX*f1[1]*Si01[1][1] + PELVIS2THORAX*f1[2]*Si01[1][2] + f1[4]*Si01[2][1] + f1[5]*Si01[2][2] - XHIP*f70[1]*Si070[3][1] - XHIP*f70[2]*Si070[3][2] + XHIP*f84[1]*Si084[3][1] + XHIP*f84[2]*Si084[3][2];
f0[6]=-f1[6] + XHIP*f70[3] + XHIP*f84[3] + fnet0[6] + PELVISOFFSET*f1[1]*Si01[1][1] + PELVISOFFSET*f1[2]*Si01[1][2] + f70[4]*Si070[3][1] + f70[5]*Si070[3][2] + f84[4]*Si084[3][1] + f84[5]*Si084[3][2];


fext90[1]=fex90[1];
fext90[2]=fex90[2];
fext90[3]=fex90[3];
fext90[4]=fex90[4];
fext90[5]=fex90[5];
fext90[6]=fex90[6];

fext89[1]=fex89[1] + fext90[1]*Si8990[1][1] + fext90[2]*Si8990[1][2];
fext89[2]=fex89[2] - fext90[3];
fext89[3]=fex89[3] + fext90[1]*Si8990[3][1] + fext90[2]*Si8990[3][2];
fext89[4]=fex89[4] + fext90[4]*Si8990[1][1] + fext90[5]*Si8990[1][2];
fext89[5]=fex89[5] - fext90[6];
fext89[6]=fex89[6] + fext90[4]*Si8990[3][1] + fext90[5]*Si8990[3][2];

fext88[1]=fext89[1]*Si8889[1][1] + fext89[2]*Si8889[1][2];
fext88[2]=fext89[3];
fext88[3]=fext89[1]*Si8889[3][1] + fext89[2]*Si8889[3][2];
fext88[4]=LOWERLEG*fext89[3] + fext89[4]*Si8889[1][1] + fext89[5]*Si8889[1][2];
fext88[5]=fext89[6] - LOWERLEG*fext89[1]*Si8889[1][1] - LOWERLEG*fext89[2]*Si8889[1][2];
fext88[6]=fext89[4]*Si8889[3][1] + fext89[5]*Si8889[3][2];

fext87[1]=fex87[1] + fext88[1]*Si8788[1][1] + fext88[2]*Si8788[1][2];
fext87[2]=fex87[2] + fext88[3];
fext87[3]=fex87[3] + fext88[1]*Si8788[3][1] + fext88[2]*Si8788[3][2];
fext87[4]=fex87[4] + fext88[4]*Si8788[1][1] + fext88[5]*Si8788[1][2];
fext87[5]=fex87[5] + fext88[6];
fext87[6]=fex87[6] + fext88[4]*Si8788[3][1] + fext88[5]*Si8788[3][2];

fext86[1]=fext87[1]*Si8687[1][1] + fext87[2]*Si8687[1][2];
fext86[2]=-fext87[3];
fext86[3]=fext87[1]*Si8687[3][1] + fext87[2]*Si8687[3][2];
fext86[4]=-(UPPERLEGMOD*fext87[3]) + fext87[4]*Si8687[1][1] + fext87[5]*Si8687[1][2];
fext86[5]=-fext87[6] + fext87[1]*(-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1]) + fext87[2]*(-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2]);
fext86[6]=-(YKNEE*fext87[3]) + fext87[4]*Si8687[3][1] + fext87[5]*Si8687[3][2];

fext85[1]=fext86[1]*Si8586[1][1] + fext86[2]*Si8586[1][2] + fext86[3]*Si8586[1][3];
fext85[2]=fext86[1]*Si8586[2][1] + fext86[2]*Si8586[2][2] + fext86[3]*Si8586[2][3];
fext85[3]=fext86[1]*Si8586[3][1] + fext86[2]*Si8586[3][2];
fext85[4]=fext86[4]*Si8586[1][1] + fext86[5]*Si8586[1][2] + fext86[6]*Si8586[1][3];
fext85[5]=fext86[4]*Si8586[2][1] + fext86[5]*Si8586[2][2] + fext86[6]*Si8586[2][3] - YHIP*fext86[1]*Si8586[3][1] - YHIP*fext86[2]*Si8586[3][2];
fext85[6]=YHIP*fext86[1]*Si8586[2][1] + YHIP*fext86[2]*Si8586[2][2] + YHIP*fext86[3]*Si8586[2][3] + fext86[4]*Si8586[3][1] + fext86[5]*Si8586[3][2];

fext84[1]=fex84[1] + fext85[1]*Si8485[1][1] + fext85[2]*Si8485[1][2];
fext84[2]=fex84[2] + fext85[3];
fext84[3]=fex84[3] + fext85[1]*Si8485[3][1] + fext85[2]*Si8485[3][2];
fext84[4]=fex84[4] + fext85[4]*Si8485[1][1] + fext85[5]*Si8485[1][2];
fext84[5]=fex84[5] + fext85[6];
fext84[6]=fex84[6] + fext85[4]*Si8485[3][1] + fext85[5]*Si8485[3][2];

fext76[1]=fex76[1];
fext76[2]=fex76[2];
fext76[3]=fex76[3];
fext76[4]=fex76[4];
fext76[5]=fex76[5];
fext76[6]=fex76[6];

fext75[1]=fex75[1] + fext76[1]*Si7576[1][1] + fext76[2]*Si7576[1][2];
fext75[2]=fex75[2] + fext76[3];
fext75[3]=fex75[3] + fext76[1]*Si7576[3][1] + fext76[2]*Si7576[3][2];
fext75[4]=fex75[4] + fext76[4]*Si7576[1][1] + fext76[5]*Si7576[1][2];
fext75[5]=fex75[5] + fext76[6];
fext75[6]=fex75[6] + fext76[4]*Si7576[3][1] + fext76[5]*Si7576[3][2];

fext74[1]=fext75[1]*Si7475[1][1] + fext75[2]*Si7475[1][2];
fext74[2]=-fext75[3];
fext74[3]=fext75[1]*Si7475[3][1] + fext75[2]*Si7475[3][2];
fext74[4]=LOWERLEG*fext75[3] + fext75[4]*Si7475[1][1] + fext75[5]*Si7475[1][2];
fext74[5]=-fext75[6] + LOWERLEG*fext75[1]*Si7475[1][1] + LOWERLEG*fext75[2]*Si7475[1][2];
fext74[6]=fext75[4]*Si7475[3][1] + fext75[5]*Si7475[3][2];

fext73[1]=fex73[1] + fext74[1]*Si7374[1][1] + fext74[2]*Si7374[1][2];
fext73[2]=fex73[2] - fext74[3];
fext73[3]=fex73[3] + fext74[1]*Si7374[3][1] + fext74[2]*Si7374[3][2];
fext73[4]=fex73[4] + fext74[4]*Si7374[1][1] + fext74[5]*Si7374[1][2];
fext73[5]=fex73[5] - fext74[6];
fext73[6]=fex73[6] + fext74[4]*Si7374[3][1] + fext74[5]*Si7374[3][2];

fext72[1]=fext73[1]*Si7273[1][1] + fext73[2]*Si7273[1][2];
fext72[2]=fext73[3];
fext72[3]=fext73[1]*Si7273[3][1] + fext73[2]*Si7273[3][2];
fext72[4]=-(UPPERLEGMOD*fext73[3]) + fext73[4]*Si7273[1][1] + fext73[5]*Si7273[1][2];
fext72[5]=fext73[6] + fext73[1]*(UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1]) + fext73[2]*(UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2]);
fext72[6]=YKNEE*fext73[3] + fext73[4]*Si7273[3][1] + fext73[5]*Si7273[3][2];

fext71[1]=fext72[1]*Si7172[1][1] + fext72[2]*Si7172[1][2] + fext72[3]*Si7172[1][3];
fext71[2]=fext72[1]*Si7172[2][1] + fext72[2]*Si7172[2][2] + fext72[3]*Si7172[2][3];
fext71[3]=fext72[1]*Si7172[3][1] + fext72[2]*Si7172[3][2];
fext71[4]=fext72[4]*Si7172[1][1] + fext72[5]*Si7172[1][2] + fext72[6]*Si7172[1][3];
fext71[5]=fext72[4]*Si7172[2][1] + fext72[5]*Si7172[2][2] + fext72[6]*Si7172[2][3] - YHIP*fext72[1]*Si7172[3][1] - YHIP*fext72[2]*Si7172[3][2];
fext71[6]=YHIP*fext72[1]*Si7172[2][1] + YHIP*fext72[2]*Si7172[2][2] + YHIP*fext72[3]*Si7172[2][3] + fext72[4]*Si7172[3][1] + fext72[5]*Si7172[3][2];

fext70[1]=fex70[1] + fext71[1]*Si7071[1][1] + fext71[2]*Si7071[1][2];
fext70[2]=fex70[2] - fext71[3];
fext70[3]=fex70[3] + fext71[1]*Si7071[3][1] + fext71[2]*Si7071[3][2];
fext70[4]=fex70[4] + fext71[4]*Si7071[1][1] + fext71[5]*Si7071[1][2];
fext70[5]=fex70[5] - fext71[6];
fext70[6]=fex70[6] + fext71[4]*Si7071[3][1] + fext71[5]*Si7071[3][2];

fext62[1]=fex62[1];
fext62[2]=fex62[2];
fext62[3]=fex62[3];
fext62[4]=fex62[4];
fext62[5]=fex62[5];
fext62[6]=fex62[6];

fext61[1]=fext62[1]*Si6162[1][1] + fext62[2]*Si6162[1][2];
fext61[2]=fext62[3];
fext61[3]=fext62[1]*Si6162[3][1] + fext62[2]*Si6162[3][2];
fext61[4]=fext62[4]*Si6162[1][1] + fext62[5]*Si6162[1][2];
fext61[5]=fext62[6];
fext61[6]=fext62[4]*Si6162[3][1] + fext62[5]*Si6162[3][2];

fext60[1]=fext61[3];
fext60[2]=fext61[1]*Si6061[2][1] + fext61[2]*Si6061[2][2];
fext60[3]=fext61[1]*Si6061[3][1] + fext61[2]*Si6061[3][2];
fext60[4]=fext61[6] - CERVICAL*fext61[1]*Si6061[3][1] - CERVICAL*fext61[2]*Si6061[3][2];
fext60[5]=fext61[4]*Si6061[2][1] + fext61[5]*Si6061[2][2];
fext60[6]=CERVICAL*fext61[3] + fext61[4]*Si6061[3][1] + fext61[5]*Si6061[3][2];

fext56[1]=fex56[1];
fext56[2]=fex56[2];
fext56[3]=fex56[3];
fext56[4]=fex56[4];
fext56[5]=fex56[5];
fext56[6]=fex56[6];

fext52[1]=fex52[1];
fext52[2]=fex52[2];
fext52[3]=fex52[3];
fext52[4]=fex52[4];
fext52[5]=fex52[5];
fext52[6]=fex52[6];

fext48[1]=fex48[1];
fext48[2]=fex48[2];
fext48[3]=fex48[3];
fext48[4]=fex48[4];
fext48[5]=fex48[5];
fext48[6]=fex48[6];

fext44[1]=fex44[1];
fext44[2]=fex44[2];
fext44[3]=fex44[3];
fext44[4]=fex44[4];
fext44[5]=fex44[5];
fext44[6]=fex44[6];

fext41[1]=fex41[1];
fext41[2]=fex41[2];
fext41[3]=fex41[3];
fext41[4]=fex41[4];
fext41[5]=fex41[5];
fext41[6]=fex41[6];

fext40[1]=fex40[1] + fext41[1]*Si4041[1][1] + fext41[2]*Si4041[1][2];
fext40[2]=fex40[2] + fext41[1]*Si4041[2][1] + fext41[2]*Si4041[2][2];
fext40[3]=fex40[3] + fext41[3];
fext40[4]=fex40[4] + YTHUMBFLEX*fext41[3] + fext41[4]*Si4041[1][1] + fext41[5]*Si4041[1][2];
fext40[5]=fex40[5] - XTHUMBFLEX*fext41[3] + fext41[4]*Si4041[2][1] + fext41[5]*Si4041[2][2];
fext40[6]=fex40[6] + fext41[6] + fext41[1]*(-(YTHUMBFLEX*Si4041[1][1]) + XTHUMBFLEX*Si4041[2][1]) + fext41[2]*(-(YTHUMBFLEX*Si4041[1][2]) + XTHUMBFLEX*Si4041[2][2]);

fext38[1]=fex38[1] + fext40[1]*Si3840[1][1] + fext40[2]*Si3840[1][2] + fext40[3]*Si3840[1][3] + fext44[1]*Si3844[1][1] + fext44[2]*Si3844[1][2] + fext48[1]*Si3848[1][1] + fext48[2]*Si3848[1][2] + fext52[1]*Si3852[1][1] + fext52[2]*Si3852[1][2] + fext56[1]*Si3856[1][1] + fext56[2]*Si3856[1][2];
fext38[2]=fex38[2] + fext40[1]*Si3840[2][1] + fext40[2]*Si3840[2][2] + fext40[3]*Si3840[2][3] + fext44[1]*Si3844[2][1] + fext44[2]*Si3844[2][2] + fext44[3]*Si3844[2][3] + fext48[1]*Si3848[2][1] + fext48[2]*Si3848[2][2] + fext48[3]*Si3848[2][3] + fext52[1]*Si3852[2][1] + fext52[2]*Si3852[2][2] + fext52[3]*Si3852[2][3] + fext56[1]*Si3856[2][1] + fext56[2]*Si3856[2][2] + fext56[3]*Si3856[2][3];
fext38[3]=fex38[3] + fext40[1]*Si3840[3][1] + fext40[2]*Si3840[3][2] + fext40[3]*Si3840[3][3] + fext44[1]*Si3844[3][1] + fext44[2]*Si3844[3][2] + fext44[3]*Si3844[3][3] + fext48[1]*Si3848[3][1] + fext48[2]*Si3848[3][2] + fext48[3]*Si3848[3][3] + fext52[1]*Si3852[3][1] + fext52[2]*Si3852[3][2] + fext52[3]*Si3852[3][3] + fext56[1]*Si3856[3][1] + fext56[2]*Si3856[3][2] + fext56[3]*Si3856[3][3];
fext38[4]=fex38[4] + fext40[4]*Si3840[1][1] + fext40[5]*Si3840[1][2] + fext40[6]*Si3840[1][3] + fext40[1]*(-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1]) + fext40[2]*(-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2]) + fext40[3]*(-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3]) + fext44[4]*Si3844[1][1] + fext44[5]*Si3844[1][2] + fext44[1]*(-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1]) + fext44[2]*(-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2]) + fext44[3]*(-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3]) + fext48[4]*Si3848[1][1] + fext48[5]*Si3848[1][2] + fext48[1]*(-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1]) + fext48[2]*(-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2]) + fext48[3]*(-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3]) + fext52[4]*Si3852[1][1] + fext52[5]*Si3852[1][2] + fext52[1]*(-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1]) + fext52[2]*(-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2]) + fext52[3]*(-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3]) + fext56[4]*Si3856[1][1] + fext56[5]*Si3856[1][2] + fext56[1]*(-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1]) + fext56[2]*(-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2]) + fext56[3]*(-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3]);
fext38[5]=fex38[5] + fext40[4]*Si3840[2][1] + fext40[5]*Si3840[2][2] + fext40[6]*Si3840[2][3] + fext40[1]*(ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1]) + fext40[2]*(ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2]) + fext40[3]*(ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3]) + fext44[4]*Si3844[2][1] + fext44[5]*Si3844[2][2] + fext44[6]*Si3844[2][3] + fext44[1]*(ZIF*Si3844[1][1] - XIF*Si3844[3][1]) + fext44[2]*(ZIF*Si3844[1][2] - XIF*Si3844[3][2]) - XIF*fext44[3]*Si3844[3][3] + fext48[4]*Si3848[2][1] + fext48[5]*Si3848[2][2] + fext48[6]*Si3848[2][3] + fext48[1]*(ZMF*Si3848[1][1] - XMF*Si3848[3][1]) + fext48[2]*(ZMF*Si3848[1][2] - XMF*Si3848[3][2]) - XMF*fext48[3]*Si3848[3][3] + fext52[4]*Si3852[2][1] + fext52[5]*Si3852[2][2] + fext52[6]*Si3852[2][3] + fext52[1]*(ZRF*Si3852[1][1] - XRF*Si3852[3][1]) + fext52[2]*(ZRF*Si3852[1][2] - XRF*Si3852[3][2]) - XRF*fext52[3]*Si3852[3][3] + fext56[4]*Si3856[2][1] + fext56[5]*Si3856[2][2] + fext56[6]*Si3856[2][3] + fext56[1]*(ZLF*Si3856[1][1] - XLF*Si3856[3][1]) + fext56[2]*(ZLF*Si3856[1][2] - XLF*Si3856[3][2]) - XLF*fext56[3]*Si3856[3][3];
fext38[6]=fex38[6] + fext40[1]*(-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1]) + fext40[2]*(-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2]) + fext40[3]*(-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3]) + fext40[4]*Si3840[3][1] + fext40[5]*Si3840[3][2] + fext40[6]*Si3840[3][3] + fext44[1]*(-(YIF*Si3844[1][1]) + XIF*Si3844[2][1]) + fext44[2]*(-(YIF*Si3844[1][2]) + XIF*Si3844[2][2]) + XIF*fext44[3]*Si3844[2][3] + fext44[4]*Si3844[3][1] + fext44[5]*Si3844[3][2] + fext44[6]*Si3844[3][3] + fext48[1]*(-(YMF*Si3848[1][1]) + XMF*Si3848[2][1]) + fext48[2]*(-(YMF*Si3848[1][2]) + XMF*Si3848[2][2]) + XMF*fext48[3]*Si3848[2][3] + fext48[4]*Si3848[3][1] + fext48[5]*Si3848[3][2] + fext48[6]*Si3848[3][3] + fext52[1]*(-(YRF*Si3852[1][1]) + XRF*Si3852[2][1]) + fext52[2]*(-(YRF*Si3852[1][2]) + XRF*Si3852[2][2]) + XRF*fext52[3]*Si3852[2][3] + fext52[4]*Si3852[3][1] + fext52[5]*Si3852[3][2] + fext52[6]*Si3852[3][3] + fext56[1]*(-(YLF*Si3856[1][1]) + XLF*Si3856[2][1]) + fext56[2]*(-(YLF*Si3856[1][2]) + XLF*Si3856[2][2]) + XLF*fext56[3]*Si3856[2][3] + fext56[4]*Si3856[3][1] + fext56[5]*Si3856[3][2] + fext56[6]*Si3856[3][3];

fext37[1]=fex37[1] + fext38[1]*Si3738[1][1] + fext38[2]*Si3738[1][2];
fext37[2]=fex37[2] + fext38[3];
fext37[3]=fex37[3] + fext38[1]*Si3738[3][1] + fext38[2]*Si3738[3][2];
fext37[4]=fex37[4] + fext38[4]*Si3738[1][1] + fext38[5]*Si3738[1][2];
fext37[5]=fex37[5] + fext38[6];
fext37[6]=fex37[6] + fext38[4]*Si3738[3][1] + fext38[5]*Si3738[3][2];

fext36[1]=fex36[1] - fext37[3];
fext36[2]=fex36[2] + fext37[1]*Si3637[2][1] + fext37[2]*Si3637[2][2];
fext36[3]=fex36[3] + fext37[1]*Si3637[3][1] + fext37[2]*Si3637[3][2];
fext36[4]=fex36[4] - fext37[6] + fext37[1]*(-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1]) + fext37[2]*(-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2]);
fext36[5]=fex36[5] - LOWERARM*fext37[3] + fext37[4]*Si3637[2][1] + fext37[5]*Si3637[2][2];
fext36[6]=fex36[6] + WRISTY*fext37[3] + fext37[4]*Si3637[3][1] + fext37[5]*Si3637[3][2];

fext35[1]=fex35[1] + fext36[1]*Si3536[1][1] + fext36[2]*Si3536[1][2];
fext35[2]=fex35[2] - fext36[3];
fext35[3]=fex35[3] + fext36[1]*Si3536[3][1] + fext36[2]*Si3536[3][2];
fext35[4]=fex35[4] + fext36[4]*Si3536[1][1] + fext36[5]*Si3536[1][2];
fext35[5]=fex35[5] - fext36[6];
fext35[6]=fex35[6] + fext36[4]*Si3536[3][1] + fext36[5]*Si3536[3][2];

fext34[1]=fex34[1] - fext35[3];
fext34[2]=fex34[2] + fext35[1]*Si3435[2][1] + fext35[2]*Si3435[2][2];
fext34[3]=fex34[3] + fext35[1]*Si3435[3][1] + fext35[2]*Si3435[3][2];
fext34[4]=fex34[4] - fext35[6] - UPPERARM*fext35[1]*Si3435[2][1] - UPPERARM*fext35[2]*Si3435[2][2];
fext34[5]=fex34[5] - UPPERARM*fext35[3] + fext35[4]*Si3435[2][1] + fext35[5]*Si3435[2][2];
fext34[6]=fex34[6] + fext35[4]*Si3435[3][1] + fext35[5]*Si3435[3][2];

fext33[1]=fex33[1] + fext34[1]*Si3334[1][1] + fext34[2]*Si3334[1][2];
fext33[2]=fex33[2] - fext34[3];
fext33[3]=fex33[3] + fext34[1]*Si3334[3][1] + fext34[2]*Si3334[3][2];
fext33[4]=fex33[4] + fext34[4]*Si3334[1][1] + fext34[5]*Si3334[1][2];
fext33[5]=fex33[5] - fext34[6] + SHOULDERY*fext34[1]*Si3334[3][1] + SHOULDERY*fext34[2]*Si3334[3][2];
fext33[6]=fex33[6] + SHOULDERY*fext34[3] + fext34[4]*Si3334[3][1] + fext34[5]*Si3334[3][2];

fext32[1]=fext33[1]*Si3233[1][1] + fext33[2]*Si3233[1][2];
fext32[2]=fext33[3];
fext32[3]=fext33[1]*Si3233[3][1] + fext33[2]*Si3233[3][2];
fext32[4]=-(SHOULDERX*fext33[3]) + fext33[4]*Si3233[1][1] + fext33[5]*Si3233[1][2];
fext32[5]=fext33[6] + SHOULDERX*fext33[1]*Si3233[1][1] + SHOULDERX*fext33[2]*Si3233[1][2];
fext32[6]=fext33[4]*Si3233[3][1] + fext33[5]*Si3233[3][2];

fext28[1]=fex28[1];
fext28[2]=fex28[2];
fext28[3]=fex28[3];
fext28[4]=fex28[4];
fext28[5]=fex28[5];
fext28[6]=fex28[6];

fext24[1]=fex24[1];
fext24[2]=fex24[2];
fext24[3]=fex24[3];
fext24[4]=fex24[4];
fext24[5]=fex24[5];
fext24[6]=fex24[6];

fext20[1]=fex20[1];
fext20[2]=fex20[2];
fext20[3]=fex20[3];
fext20[4]=fex20[4];
fext20[5]=fex20[5];
fext20[6]=fex20[6];

fext16[1]=fex16[1];
fext16[2]=fex16[2];
fext16[3]=fex16[3];
fext16[4]=fex16[4];
fext16[5]=fex16[5];
fext16[6]=fex16[6];

fext13[1]=fex13[1];
fext13[2]=fex13[2];
fext13[3]=fex13[3];
fext13[4]=fex13[4];
fext13[5]=fex13[5];
fext13[6]=fex13[6];

fext12[1]=fex12[1] + fext13[1]*Si1213[1][1] + fext13[2]*Si1213[1][2];
fext12[2]=fex12[2] + fext13[1]*Si1213[2][1] + fext13[2]*Si1213[2][2];
fext12[3]=fex12[3] + fext13[3];
fext12[4]=fex12[4] + YTHUMBFLEX*fext13[3] + fext13[4]*Si1213[1][1] + fext13[5]*Si1213[1][2];
fext12[5]=fex12[5] - XTHUMBFLEX*fext13[3] + fext13[4]*Si1213[2][1] + fext13[5]*Si1213[2][2];
fext12[6]=fex12[6] + fext13[6] + fext13[1]*(-(YTHUMBFLEX*Si1213[1][1]) + XTHUMBFLEX*Si1213[2][1]) + fext13[2]*(-(YTHUMBFLEX*Si1213[1][2]) + XTHUMBFLEX*Si1213[2][2]);

fext10[1]=fex10[1] + fext12[1]*Si1012[1][1] + fext12[2]*Si1012[1][2] + fext12[3]*Si1012[1][3] + fext16[1]*Si1016[1][1] + fext16[2]*Si1016[1][2] + fext20[1]*Si1020[1][1] + fext20[2]*Si1020[1][2] + fext24[1]*Si1024[1][1] + fext24[2]*Si1024[1][2] + fext28[1]*Si1028[1][1] + fext28[2]*Si1028[1][2];
fext10[2]=fex10[2] + fext12[1]*Si1012[2][1] + fext12[2]*Si1012[2][2] + fext12[3]*Si1012[2][3] + fext16[1]*Si1016[2][1] + fext16[2]*Si1016[2][2] + fext16[3]*Si1016[2][3] + fext20[1]*Si1020[2][1] + fext20[2]*Si1020[2][2] + fext20[3]*Si1020[2][3] + fext24[1]*Si1024[2][1] + fext24[2]*Si1024[2][2] + fext24[3]*Si1024[2][3] + fext28[1]*Si1028[2][1] + fext28[2]*Si1028[2][2] + fext28[3]*Si1028[2][3];
fext10[3]=fex10[3] + fext12[1]*Si1012[3][1] + fext12[2]*Si1012[3][2] + fext12[3]*Si1012[3][3] + fext16[1]*Si1016[3][1] + fext16[2]*Si1016[3][2] + fext16[3]*Si1016[3][3] + fext20[1]*Si1020[3][1] + fext20[2]*Si1020[3][2] + fext20[3]*Si1020[3][3] + fext24[1]*Si1024[3][1] + fext24[2]*Si1024[3][2] + fext24[3]*Si1024[3][3] + fext28[1]*Si1028[3][1] + fext28[2]*Si1028[3][2] + fext28[3]*Si1028[3][3];
fext10[4]=fex10[4] + fext12[4]*Si1012[1][1] + fext12[5]*Si1012[1][2] + fext12[6]*Si1012[1][3] + fext12[1]*(ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1]) + fext12[2]*(ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2]) + fext12[3]*(ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3]) + fext16[4]*Si1016[1][1] + fext16[5]*Si1016[1][2] + fext16[1]*(ZIF*Si1016[2][1] + YIF*Si1016[3][1]) + fext16[2]*(ZIF*Si1016[2][2] + YIF*Si1016[3][2]) + fext16[3]*(ZIF*Si1016[2][3] + YIF*Si1016[3][3]) + fext20[4]*Si1020[1][1] + fext20[5]*Si1020[1][2] + fext20[1]*(ZMF*Si1020[2][1] + YMF*Si1020[3][1]) + fext20[2]*(ZMF*Si1020[2][2] + YMF*Si1020[3][2]) + fext20[3]*(ZMF*Si1020[2][3] + YMF*Si1020[3][3]) + fext24[4]*Si1024[1][1] + fext24[5]*Si1024[1][2] + fext24[1]*(ZRF*Si1024[2][1] + YRF*Si1024[3][1]) + fext24[2]*(ZRF*Si1024[2][2] + YRF*Si1024[3][2]) + fext24[3]*(ZRF*Si1024[2][3] + YRF*Si1024[3][3]) + fext28[4]*Si1028[1][1] + fext28[5]*Si1028[1][2] + fext28[1]*(ZLF*Si1028[2][1] + YLF*Si1028[3][1]) + fext28[2]*(ZLF*Si1028[2][2] + YLF*Si1028[3][2]) + fext28[3]*(ZLF*Si1028[2][3] + YLF*Si1028[3][3]);
fext10[5]=fex10[5] + fext12[4]*Si1012[2][1] + fext12[5]*Si1012[2][2] + fext12[6]*Si1012[2][3] + fext12[1]*(-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1]) + fext12[2]*(-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2]) + fext12[3]*(-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3]) + fext16[4]*Si1016[2][1] + fext16[5]*Si1016[2][2] + fext16[6]*Si1016[2][3] + fext16[1]*(-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1]) + fext16[2]*(-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2]) - XIF*fext16[3]*Si1016[3][3] + fext20[4]*Si1020[2][1] + fext20[5]*Si1020[2][2] + fext20[6]*Si1020[2][3] + fext20[1]*(-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1]) + fext20[2]*(-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2]) - XMF*fext20[3]*Si1020[3][3] + fext24[4]*Si1024[2][1] + fext24[5]*Si1024[2][2] + fext24[6]*Si1024[2][3] + fext24[1]*(-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1]) + fext24[2]*(-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2]) - XRF*fext24[3]*Si1024[3][3] + fext28[4]*Si1028[2][1] + fext28[5]*Si1028[2][2] + fext28[6]*Si1028[2][3] + fext28[1]*(-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1]) + fext28[2]*(-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2]) - XLF*fext28[3]*Si1028[3][3];
fext10[6]=fex10[6] + fext12[1]*(-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1]) + fext12[2]*(-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2]) + fext12[3]*(-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3]) + fext12[4]*Si1012[3][1] + fext12[5]*Si1012[3][2] + fext12[6]*Si1012[3][3] + fext16[1]*(-(YIF*Si1016[1][1]) + XIF*Si1016[2][1]) + fext16[2]*(-(YIF*Si1016[1][2]) + XIF*Si1016[2][2]) + XIF*fext16[3]*Si1016[2][3] + fext16[4]*Si1016[3][1] + fext16[5]*Si1016[3][2] + fext16[6]*Si1016[3][3] + fext20[1]*(-(YMF*Si1020[1][1]) + XMF*Si1020[2][1]) + fext20[2]*(-(YMF*Si1020[1][2]) + XMF*Si1020[2][2]) + XMF*fext20[3]*Si1020[2][3] + fext20[4]*Si1020[3][1] + fext20[5]*Si1020[3][2] + fext20[6]*Si1020[3][3] + fext24[1]*(-(YRF*Si1024[1][1]) + XRF*Si1024[2][1]) + fext24[2]*(-(YRF*Si1024[1][2]) + XRF*Si1024[2][2]) + XRF*fext24[3]*Si1024[2][3] + fext24[4]*Si1024[3][1] + fext24[5]*Si1024[3][2] + fext24[6]*Si1024[3][3] + fext28[1]*(-(YLF*Si1028[1][1]) + XLF*Si1028[2][1]) + fext28[2]*(-(YLF*Si1028[1][2]) + XLF*Si1028[2][2]) + XLF*fext28[3]*Si1028[2][3] + fext28[4]*Si1028[3][1] + fext28[5]*Si1028[3][2] + fext28[6]*Si1028[3][3];

fext9[1]=fex9[1] + fext10[1]*Si910[1][1] + fext10[2]*Si910[1][2];
fext9[2]=fex9[2] - fext10[3];
fext9[3]=fex9[3] + fext10[1]*Si910[3][1] + fext10[2]*Si910[3][2];
fext9[4]=fex9[4] + fext10[4]*Si910[1][1] + fext10[5]*Si910[1][2];
fext9[5]=fex9[5] - fext10[6];
fext9[6]=fex9[6] + fext10[4]*Si910[3][1] + fext10[5]*Si910[3][2];

fext8[1]=fex8[1] + fext9[3];
fext8[2]=fex8[2] + fext9[1]*Si89[2][1] + fext9[2]*Si89[2][2];
fext8[3]=fex8[3] + fext9[1]*Si89[3][1] + fext9[2]*Si89[3][2];
fext8[4]=fex8[4] + fext9[6] + fext9[1]*(LOWERARM*Si89[2][1] + WRISTY*Si89[3][1]) + fext9[2]*(LOWERARM*Si89[2][2] + WRISTY*Si89[3][2]);
fext8[5]=fex8[5] - LOWERARM*fext9[3] + fext9[4]*Si89[2][1] + fext9[5]*Si89[2][2];
fext8[6]=fex8[6] - WRISTY*fext9[3] + fext9[4]*Si89[3][1] + fext9[5]*Si89[3][2];

fext7[1]=fex7[1] + fext8[1]*Si78[1][1] + fext8[2]*Si78[1][2];
fext7[2]=fex7[2] + fext8[3];
fext7[3]=fex7[3] + fext8[1]*Si78[3][1] + fext8[2]*Si78[3][2];
fext7[4]=fex7[4] + fext8[4]*Si78[1][1] + fext8[5]*Si78[1][2];
fext7[5]=fex7[5] + fext8[6];
fext7[6]=fex7[6] + fext8[4]*Si78[3][1] + fext8[5]*Si78[3][2];

fext6[1]=fex6[1] + fext7[3];
fext6[2]=fex6[2] + fext7[1]*Si67[2][1] + fext7[2]*Si67[2][2];
fext6[3]=fex6[3] + fext7[1]*Si67[3][1] + fext7[2]*Si67[3][2];
fext6[4]=fex6[4] + fext7[6] + UPPERARM*fext7[1]*Si67[2][1] + UPPERARM*fext7[2]*Si67[2][2];
fext6[5]=fex6[5] - UPPERARM*fext7[3] + fext7[4]*Si67[2][1] + fext7[5]*Si67[2][2];
fext6[6]=fex6[6] + fext7[4]*Si67[3][1] + fext7[5]*Si67[3][2];

fext5[1]=fex5[1] + fext6[1]*Si56[1][1] + fext6[2]*Si56[1][2];
fext5[2]=fex5[2] + fext6[3];
fext5[3]=fex5[3] + fext6[1]*Si56[3][1] + fext6[2]*Si56[3][2];
fext5[4]=fex5[4] + fext6[4]*Si56[1][1] + fext6[5]*Si56[1][2];
fext5[5]=fex5[5] + fext6[6] + SHOULDERY*fext6[1]*Si56[3][1] + SHOULDERY*fext6[2]*Si56[3][2];
fext5[6]=fex5[6] - SHOULDERY*fext6[3] + fext6[4]*Si56[3][1] + fext6[5]*Si56[3][2];

fext4[1]=fext5[1]*Si45[1][1] + fext5[2]*Si45[1][2];
fext4[2]=-fext5[3];
fext4[3]=fext5[1]*Si45[3][1] + fext5[2]*Si45[3][2];
fext4[4]=-(SHOULDERX*fext5[3]) + fext5[4]*Si45[1][1] + fext5[5]*Si45[1][2];
fext4[5]=-fext5[6] - SHOULDERX*fext5[1]*Si45[1][1] - SHOULDERX*fext5[2]*Si45[1][2];
fext4[6]=fext5[4]*Si45[3][1] + fext5[5]*Si45[3][2];

fext3[1]=fex3[1] - 0.7071067811865475*fext32[3] + 0.7071067811865475*fext4[3] + fext32[1]*Si332[1][1] + fext32[2]*Si332[1][2] + fext4[1]*Si34[1][1] + fext4[2]*Si34[1][2] + fext60[1]*Si360[1][1] + fext60[2]*Si360[1][2];
fext3[2]=fex3[2] + fext32[1]*Si332[2][1] + fext32[2]*Si332[2][2] + fext4[1]*Si34[2][1] + fext4[2]*Si34[2][2] + fext60[1]*Si360[2][1] + fext60[2]*Si360[2][2];
fext3[3]=fex3[3] - 0.7071067811865475*fext32[3] - 0.7071067811865475*fext4[3] + fext60[3] + fext32[1]*Si332[3][1] + fext32[2]*Si332[3][2] + fext4[1]*Si34[3][1] + fext4[2]*Si34[3][2];
fext3[4]=fex3[4] - 0.7071067811865475*fext32[6] + 0.7071067811865475*fext4[6] + fext32[4]*Si332[1][1] + fext32[5]*Si332[1][2] + fext4[4]*Si34[1][1] + fext4[5]*Si34[1][2] + fext60[4]*Si360[1][1] + fext60[5]*Si360[1][2];
fext3[5]=fex3[5] - 0.7071067811865475*THORAX2SHOULDER*fext32[3] - 0.7071067811865475*THORAX2SHOULDER*fext4[3] + THORAX2NECK*fext60[3] + fext32[4]*Si332[2][1] + fext32[5]*Si332[2][2] + THORAX2SHOULDER*fext32[1]*Si332[3][1] + THORAX2SHOULDER*fext32[2]*Si332[3][2] + fext4[4]*Si34[2][1] + fext4[5]*Si34[2][2] + THORAX2SHOULDER*fext4[1]*Si34[3][1] + THORAX2SHOULDER*fext4[2]*Si34[3][2] + fext60[4]*Si360[2][1] + fext60[5]*Si360[2][2];
fext3[6]=fex3[6] - 0.7071067811865475*fext32[6] - 0.7071067811865475*fext4[6] + fext60[6] - THORAX2SHOULDER*fext32[1]*Si332[2][1] - THORAX2SHOULDER*fext32[2]*Si332[2][2] + fext32[4]*Si332[3][1] + fext32[5]*Si332[3][2] - THORAX2SHOULDER*fext4[1]*Si34[2][1] - THORAX2SHOULDER*fext4[2]*Si34[2][2] + fext4[4]*Si34[3][1] + fext4[5]*Si34[3][2] - THORAX2NECK*fext60[1]*Si360[2][1] - THORAX2NECK*fext60[2]*Si360[2][2];

fext2[1]=fex2[1] + fext3[1]*Si23[1][1] + fext3[2]*Si23[1][2];
fext2[2]=fex2[2] + fext3[3];
fext2[3]=fex2[3] + fext3[1]*Si23[3][1] + fext3[2]*Si23[3][2];
fext2[4]=fex2[4] + fext3[4]*Si23[1][1] + fext3[5]*Si23[1][2];
fext2[5]=fex2[5] + fext3[6];
fext2[6]=fex2[6] + fext3[4]*Si23[3][1] + fext3[5]*Si23[3][2];

fext1[1]=fex1[1] + fext2[1]*Si12[1][1] + fext2[2]*Si12[1][2];
fext1[2]=fex1[2] - fext2[3];
fext1[3]=fex1[3] + fext2[1]*Si12[3][1] + fext2[2]*Si12[3][2];
fext1[4]=fex1[4] + fext2[4]*Si12[1][1] + fext2[5]*Si12[1][2];
fext1[5]=fex1[5] - fext2[6];
fext1[6]=fex1[6] + fext2[4]*Si12[3][1] + fext2[5]*Si12[3][2];

fext0[1]=fex0[1] + fext1[1]*Si01[1][1] + fext1[2]*Si01[1][2] + fext70[1]*Si070[1][1] + fext70[2]*Si070[1][2] + fext84[1]*Si084[1][1] + fext84[2]*Si084[1][2];
fext0[2]=fex0[2] + fext70[3] - fext84[3] + fext1[1]*Si01[2][1] + fext1[2]*Si01[2][2];
fext0[3]=fex0[3] - fext1[3] + fext70[1]*Si070[3][1] + fext70[2]*Si070[3][2] + fext84[1]*Si084[3][1] + fext84[2]*Si084[3][2];
fext0[4]=fex0[4] + PELVISOFFSET*fext1[3] + fext1[4]*Si01[1][1] + fext1[5]*Si01[1][2] - PELVIS2THORAX*fext1[1]*Si01[2][1] - PELVIS2THORAX*fext1[2]*Si01[2][2] + fext70[4]*Si070[1][1] + fext70[5]*Si070[1][2] + fext84[4]*Si084[1][1] + fext84[5]*Si084[1][2];
fext0[5]=fex0[5] + fext70[6] - fext84[6] + PELVIS2THORAX*fext1[1]*Si01[1][1] + PELVIS2THORAX*fext1[2]*Si01[1][2] + fext1[4]*Si01[2][1] + fext1[5]*Si01[2][2] - XHIP*fext70[1]*Si070[3][1] - XHIP*fext70[2]*Si070[3][2] + XHIP*fext84[1]*Si084[3][1] + XHIP*fext84[2]*Si084[3][2];
fext0[6]=fex0[6] - fext1[6] + XHIP*fext70[3] + XHIP*fext84[3] + PELVISOFFSET*fext1[1]*Si01[1][1] + PELVISOFFSET*fext1[2]*Si01[1][2] + fext70[4]*Si070[3][1] + fext70[5]*Si070[3][2] + fext84[4]*Si084[3][1] + fext84[5]*Si084[3][2];



}

