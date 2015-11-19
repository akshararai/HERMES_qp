function links = getLinkInfo

  %link enumration
  BASE=0+1;

  L_SFE=1+1;
  L_SAA=2+1;
  L_HR=3+1;
  L_EB=4+1;
  L_WR=5+1;
  L_WFE=6+1;
  L_WAA=7+1;
  
  R_SFE=8+1;
  R_SAA=9+1;
  R_HR=10+1;
  R_EB=11+1;
  R_WR=12+1;
  R_WFE=13+1;
  R_WAA=14+1;

  L_HFE=15+1;
  L_HAA=16+1;
  L_HFR=17+1;
  L_KFE=18+1;
  L_AR=19+1;
  L_AFE=20+1;
  L_AAA=21+1;

  R_HFE=22+1;
  R_HAA=23+1;
  R_HFR=24+1;
  R_KFE=25+1;
  R_AR=26+1;
  R_AFE=27+1;
  R_AAA=28+1;

  B_TR=29+1;
  B_TAA=30+1;

  N_DOFS=B_TAA;

  
  Rx = @(x) [1 0 0; 0 cos(x) -sin(x); 0 sin(x) cos(x)];
  Ry = @(x) [cos(x) 0 sin(x); 0 1 0; -sin(x) 0 cos(x)];
  Rz = @(x) [cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1];
  
  R = @(x,y,z) Rx(x) * Ry(y) * Rz(z);


  %% base + torso %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  inches_to_meters = 0.0254;
  pounds_to_kilo = 0.45359237;
  poundsSqIn_to_kgSqm = pounds_to_kilo*inches_to_meters*inches_to_meters


  links = cell(N_DOFS,1);
  
  %%BASE
  links{BASE}.name = 'BASE';
  links{BASE}.mass = 20.497 * pounds_to_kilo;
  links{BASE}.com = [0.857; 0.008; 2.871] * inches_to_meters;
  links{BASE}.Iprinc = [356.134; 404.424; 449.429] * poundsSqIn_to_kgSqm;
  links{BASE}.Iaxes = [0.703 -0.015 -0.711;
                      0.711 0.002 0.703;
  		              -0.009 -1.0 0.012]';
  
  links{BASE}.rotation = R(0,-pi/2,pi/2);
  
  
  %%B_TR aka torso rotate
  links{B_TR}.name = 'B_TR';
  links{B_TR}.mass = 1.053 * pounds_to_kilo;
  links{B_TR}.com = [-0.031; -0.068; -1.428] * inches_to_meters;
  links{B_TR}.Iprinc = [0.345; 1.821; 1.950] * poundsSqIn_to_kgSqm;
  links{B_TR}.Iaxes = [-0.012 -0.040 -0.999;
                      1.0 0.003 -0.012;
                       0.003 -0.999 0.040]';
  
  links{B_TR}.rotation = R(0,0,0);
  
  
  
  %%B_TAA aka torso side to side
  links{B_TAA}.name = 'B_TAA';
  links{B_TAA}.mass = 37.429 * pounds_to_kilo;
links{B_TAA}.com = [-11.466; -0.020; 0.796] * inches_to_meters;
links{B_TAA}.Iprinc = [1071.937; 2101.997; 2743.514] * poundsSqIn_to_kgSqm;
  links{B_TAA}.Iaxes = [-0.999 0.007 -0.040;
                    0.007 1.0 0.002;
                   0.040 0.002 -0.999]';
  
  links{B_TAA}.rotation = R(0,0,0);
  

  %% right leg %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  %%R_AAA
  links{R_AAA}.name = 'R_AAA';
  links{R_AAA}.mass = 0.0;
  links{R_AAA}.com = [0.0; 0.0; 0.0];
links{R_AAA}.Iprinc = [0.0; 0.0; 0.0];
  links{R_AAA}.Iaxes = [1.0 0.0 0.0;
                    0.0 1.0 0.0;
                   0.0 0.0 1.0]';
  
  links{R_AAA}.rotation = R(0,0,0);
  
  %%R_AFE
  links{R_AFE}.name = 'R_AFE';
  links{R_AFE}.mass = 0.0 ;
links{R_AFE}.com = [0.0; 0.0; 0.0];
  links{R_AFE}.Iprinc = [0.0; 0.0; 0.0];
  links{R_AFE}.Iaxes = [1.0 0.0 0.0;
                    0.0 1.0 0.0;
                   0.0 0.0 1.0]';
  
  links{R_AFE}.rotation = R(0,0,0);
  
  
  %%R_AR
  links{R_AR}.name = 'R_AR';
  links{R_AR}.mass = 2.013 * pounds_to_kilo;
links{R_AR}.com = [5.946; 0.0; -0.451] * inches_to_meters;
links{R_AR}.Iprinc = [5.86; 71.838; 74.920] * poundsSqIn_to_kgSqm;
  links{R_AR}.Iaxes = [-0.992 0.0 0.125;
                    -0.125 0.0 -0.992;
                   0.0 -1.0 0.0]';
  
  links{R_AR}.rotation = R(0,0,0);
  
  %%R_KFE
  links{R_KFE}.name = 'R_KFE';
  links{R_KFE}.mass = 3.926 * pounds_to_kilo;
links{R_KFE}.com = [1.436; 0.137; -0.337] * inches_to_meters;
links{R_KFE}.Iprinc = [12.236; 14.401; 19.287] * poundsSqIn_to_kgSqm;
  links{R_KFE}.Iaxes = [-0.916 0.024 0.4;
                      0.4 -0.019 0.916;
                     0.03 1.0 0.008]';
  
  links{R_KFE}.rotation = R(0,0,0);
  
  
  %%R_HFR
  links{R_HFR}.name = 'R_HFR';
  links{R_HFR}.mass = 5.838 * pounds_to_kilo;
  links{R_HFR}.com = [0.331; 0.277; 3.720] * pounds_to_kilo;
  links{R_HFR}.Iprinc = [11.864; 44.939; 45.486] * poundsSqIn_to_kgSqm;
  links{R_HFR}.Iaxes = [0.145 0.029 -0.989;
                    0.326 0.942 0.075;
                   0.934 -0.333 0.127]';
  
  links{R_HFR}.rotation = R(0,0,0);
 
  
  %%R_HFE
  links{R_HFE}.name = 'R_HFE';
  links{R_HFE}.mass = 1.896*pounds_to_kilo;
  links{R_HFE}.com = [-0.035; -0.022; 0.601] * inches_to_meters;
links{R_HFE}.Iprinc = [0.822; 6.310; 6.433] * poundsSqIn_to_kgSqm;
  links{R_HFE}.Iaxes = [-0.005 0.027 1.0;
                    0.046 0.999 -0.026;
                   -0.999 0.045 -0.007]';
  
  links{R_HFE}.rotation = R(0,0,0);
  
  %%R_HAA
  links{R_HAA}.name = 'R_HAA';
  links{R_HAA}.mass = 9.058 * pounds_to_kilo;
links{R_HAA}.com = [6.603; 2.127; 1.215] * inches_to_meters;
links{R_HAA}.Iprinc = [43.571; 199.552; 214.932] * poundsSqIn_to_kgSqm;
  links{R_HAA}.Iaxes = [-0.993 0.036 0.115;
                    -0.113 -0.607 -0.787;
                   0.041 -0.794 0.607]';
  
  links{R_HAA}.rotation = R(0,0,0);

  %% left leg %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %L_HFE
  links{L_HFE}.name = 'L_HFE';
  links{L_HFE}.mass = 1.896*pounds_to_kilo;
  links{L_HFE}.com = [-0.035; -0.022; -0.601] * inches_to_meters;
  links{L_HFE}.Iprinc = [0.822; 6.310; 6.433] * poundsSqIn_to_kgSqm;
  links{L_HFE}.Iaxes = [-0.005 0.027 -1.0;
                      -0.046 -0.999 -0.026;
                      -0.999 0.046 0.007]';

  links{L_HFE}.rotation = R(0,0,0);

  %%L_HAA
  links{L_HAA}.name = 'L_HAA';
  links{L_HAA}.mass = 9.058 * pounds_to_kilo;
  links{L_HAA}.com = [6.606; 2.133; -1.211] * inches_to_meters;
  links{L_HAA}.Iprinc = [43.898; 199.901; 215.599] * poundsSqIn_to_kgSqm;
  links{L_HAA}.Iaxes = [-0.993 0.036 -0.115;
                      -0.113 -0.618 0.778;
                      -0.043 0.785 0.618]';

  links{L_HAA}.rotation = R(0,0,0);

  %%L_AR
  links{L_AR}.name = 'L_AR';
  links{L_AR}.mass = 2.013 * pounds_to_kilo;
  links{L_AR}.com = [5.946; 0.0; -0.451] * inches_to_meters;
  links{L_AR}.Iprinc = [5.86; 71.838; 74.920] * poundsSqIn_to_kgSqm;
  links{L_AR}.Iaxes = [-0.992 0.0 0.125;
                     -0.125 0.0 -0.992;
                     0.0 -1.0 0.0]';

  links{L_AR}.rotation = R(0,0,0);

  %%L_KFE
  links{L_KFE}.name = 'L_KFE';
  links{L_KFE}.mass = 3.926 * pounds_to_kilo;
  links{L_KFE}.com = [1.436; 0.138; 0.337] * inches_to_meters;
  links{L_KFE}.Iprinc = [12.228; 14.395; 19.296] * poundsSqIn_to_kgSqm;
  links{L_KFE}.Iaxes = [-0.918 0.024 -0.395;
                      -0.395 0.020 0.918;
                      0.03 1.0 -0.009]';

  links{L_KFE}.rotation = R(0,0,0);


  %%L_HFR
  links{L_HFR}.name = 'L_HFR';
  links{L_HFR}.mass = 5.838 * pounds_to_kilo;
  links{L_HFR}.com = [0.328; 0.272; -3.683] * pounds_to_kilo;
  links{L_HFR}.Iprinc = [11.885; 45.850; 46.374] * poundsSqIn_to_kgSqm;
  links{L_HFR}.Iaxes = [0.139 0.026 0.99;
                      0.413 0.907 -0.082;
                      -0.9 0.420 0.115]';

  links{L_HFR}.rotation = R(0,0,0);

  links{L_AAA} = links{R_AAA};
  links{L_AAA}.name = 'L_AAA';

  links{L_AFE} = links{R_AFE};
  links{L_AFE}.name = 'L_AFE';


%% right arm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%R_SFE aka Rarm forward
links{R_SFE}.name = 'R_SFE';
links{R_SFE}.mass = 1.717;
links{R_SFE}.com = [-0.021781; -0.00053451; -0.16508];
links{R_SFE}.Iprinc = [0.0019687; 0.0062238; 0.0076629];
links{R_SFE}.Iaxes = [0.065974 -0.011843 0.99775;
                      0.99782 0.0025389 -0.065948;
                      -0.0017522 0.99993 0.011984]';

links{R_SFE}.rotation = R(pi,0,0);

%%R_SAA aka Rarm out
links{R_SAA}.name = 'R_SAA';
links{R_SAA}.mass = 0.28695;
links{R_SAA}.com = [-0.01168; 0.00886; 0.00134];
links{R_SAA}.Iprinc = [0.00012; 0.00015; 0.00019];
links{R_SAA}.Iaxes = [0.57527 -0.78268 0.23763;
                      -0.12734 0.20127 0.97122;
                      -0.80799 -0.58898 0.01612]';

links{R_SAA}.rotation = R(pi,0,0);

%%R_HR
links{R_HR}.name = 'R_HR';
links{R_HR}.mass = 3.0791;
links{R_HR}.com = [-0.001955; 0.0058258; -0.14027];
links{R_HR}.Iprinc = [0.0030301; 0.012141; 0.01232];
links{R_HR}.Iaxes = [0.039977 0.03763 0.99849;
                     -0.049541 0.99814 -0.035633;
                     -0.99797 -0.048042 0.041767]';

links{R_HR}.rotation = R(pi,0,0);

%%R_EB
links{R_EB}.name = 'R_EB';
links{R_EB}.mass = 0.34623;
links{R_EB}.com = [-0.00948; 0.01798; -0.00095];
links{R_EB}.Iprinc = [0.00019; 0.00029; 0.00032];
links{R_EB}.Iaxes = [-0.05366 -0.99709 -0.05413;
                     -0.11389 -0.04774 0.99235;
                     -0.99204 0.05941 -0.11100]';

links{R_EB}.rotation = R(pi,0,0);


%%R_WR
links{R_WR}.name = 'R_WR';
links{R_WR}.mass = 1.0582;
links{R_WR}.com = [-0.0185; 0.00062671; -0.13967];
links{R_WR}.Iprinc = [0.00073062; 0.0029283; 0.0030352];
links{R_WR}.Iaxes = [0.28004 0.025693 0.95964;
                     -0.9576 -0.063002 0.28113;
                     0.067682 -0.99768 0.006961]';

links{R_WR}.rotation = R(pi,0,0);

%%R_WFE
links{R_WFE}.name = 'R_WFE';
links{R_WFE}.mass = 0.05659;
links{R_WFE}.com = [-0.00058; 0.00782; -0.00875];
links{R_WFE}.Iprinc = [0.00002; 0.00002; 0.00004];
links{R_WFE}.Iaxes = [0.00731 -0.99296 -0.11825;
                      0.08759 -0.11717 0.98924;
                      -0.99613 -0.01759 0.08612]';

links{R_WFE}.rotation = R(0,0,0);

%%R_WAA
links{R_WAA}.name = 'R_WAA';
links{R_WAA}.mass = 0.27336;
links{R_WAA}.com = [0.08565; 0.00857; -0.00413];
links{R_WAA}.Iprinc = [0.00017; 0.00081; 0.00092];
links{R_WAA}.Iaxes = [0.98956 0.04878 0.13561;
                      -0.09612 0.92451 0.36885;
                      -0.10738 -0.37803 0.91954]';

links{R_WAA}.rotation = R(pi,0,0);
%% left arm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%L_SFE
links{L_SFE} = mirror(links{R_SFE},3);
links{L_SFE}.name = 'L_SFE';
links{L_SFE}.rotation = R(pi,0,0);

%%L_SAA
links{L_SAA} = mirror(links{R_SAA},1);
links{L_SAA}.name = 'L_SAA';
links{L_SAA}.rotation = R(0,0,pi);

%%L_HR
links{L_HR} = mirror(links{R_HR},1);
links{L_HR}.name = 'L_HR';
links{L_HR}.rotation = R(0,0,pi);

%%L_EB
links{L_EB} = mirror(links{R_EB},3);
links{L_EB}.name = 'L_EB';
links{L_EB}.rotation = R(pi,0,0);

%%L_WR
links{L_WR} = mirror(links{R_WR},2);
links{L_WR}.name = 'L_WR';
links{L_WR}.rotation = R(0,0,0);

%%L_WFE
links{L_WFE} = mirror(links{R_WFE},2);
links{L_WFE}.name = 'L_WFE';
links{L_WFE}.rotation = R(pi,0,0);

%%L_WAA
links{L_WAA} = mirror(links{R_WAA},3);
links{L_WAA}.name = 'L_WAA';
links{L_WAA}.rotation = R(pi,0,0);






  
function link_des = mirror(link, coord)
  
  link_des = link;
  
  link_des.com(coord) = -link.com(coord);
  link_des.Iaxes(coord,:) = -link.Iaxes(coord,:);
  
