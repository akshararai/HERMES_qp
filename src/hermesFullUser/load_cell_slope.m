function slope = load_cell_slope(load, load_m, th)
close all
% load = without mass
%load_m = with mass

%   double a = joint_geometry_[dof_num][LOAD_CELL_LENGTH];                                                      
%   double b = joint_geometry_[dof_num][MAIN_LINK_LENGTH];                                                      
%   double c = joint_geometry_[dof_num][MOUNTPOINT];                                                            
%   double d = joint_geometry_[dof_num][MOMENTARM];  

%LOAD_CELL_LENGTH = 10
%MAIN_LINK_LENGTH = 
%MOUNTPOINT =
%MOMENTARM =
%sensorCalibration = [ 'L_HFE',	0,	1,	1,	0.0497,	0.0512,	1,	1,		0.0826,	0.1143,	4.2369,	-1.0,	0.272, 	0.0813, 	1.654]

%a = sensorCalibration(LOAD_CELL_LENGTH); % 0.1143;
a = 0.0826;
b = 0.1143;
c = 0.0512;
d = 0.0497;

theta0 = 4.2369;
for i = 1:length(th)
    % double alpha = joint_geometry_[dof_num][THETA0] + joint_geometry_[dof_num][OFFSET_DIRECTION] *  theta;//get the angle between c and d
    alpha = theta0 + th(i);

    % double e = sqrt(sqr(d) + sqr(c) - 2*c*d*cos(alpha));
    e = sqrt(d^2 + c^2 - 2*c*d*cos(alpha));

    % double beta = acos( (sqr(e) + sqr(d) - sqr(c)) / (2*e*d)); 
    beta = acos(( e^2 + d^2 -c^2) / (2*e*d));

    % double beta2 = acos ( (sqr(e) + sqr(a) - sqr(b)) / (2*e*a));

    % if(a != 0.0)                                                                                                
    %   {                                                                                                           
    %     // get the angle a-e if the quadrilateral abcd is not degenerate                                          
    %     double beta2 = acos ( (sqr(e) + sqr(a) - sqr(b)) / (2*e*a));//get the angle a-e                           
    %                                                                                                               
    %     //check if the quadrilateral is convex to compute the right angle                                         
    %     if(fmod(alpha,2*M_PI)>M_PI)                                                                               
    %       beta = beta2 - beta;//we need to subtract beta1 from beta2                                              
    %     else                                                                                                      
    %       beta = beta2 + beta;                                                                                    
    %   }                 

    if a ~=0.0
        beta2 = acos (( e^2 + a^2 -b^2) / (2*e*a));
        if mod(alpha, 2*pi) > pi
            beta = beta2 - beta;
        else
            beta = beta2 + beta;
        end
    end


    moment_arm(i) = real(d * sin(beta));  
end
%% Actual load values to torque
count = 100;
slope = 6.537;
% while(norm(ratio1 - ratio2) > 0.1)
    for i=1:length(th)-1
        T(i,1) = load(i)*moment_arm(i)/slope;
        T(i,2) = load_m(i)*moment_arm(i)/slope;
        
        T(i+1,1) = load(i+1)*moment_arm(i+1)/slope;
        T(i+1,2) = load_m(i+1)*moment_arm(i+1)/slope;
        
        ratio1(i) = sin(th(i))/sin(th(i+1));
        ratio2(i) = (T(i,1)-T(i,2))/(T(i+1,1)-T(i+1,2));
    end
%     plot(ratio1,ratio2,'.')
    ratio1-ratio2
    slope = slope+0.001;
% end


%% Calculate slope

l= 0.7874;
m = 2.227+0.53+0.443;
g=9.8;

for i=1:length(th)
    netT= m*g*l*sin(th(i));
    slope(i) = (load(i)*moment_arm(i) - load_m(i)*moment_arm(i))/netT;
end

slop = slope(2:end);
mean_slope = mean(slop)


t1_s = load.*moment_arm./mean_slope;
t2_s = load_m.*moment_arm./mean_slope;
dt_s = t1_s - t2_s;

t1 = [6.6 13.28 20.06 27.03 32.26 39.23 45.7];
t2 = [7.91 18.07 27 35.95 44.76 52.82 60.51];
dt = -t1+t2;

sinTh = m*g*l*sin(th);

figure; hold on
plot(th, sinTh, '-', 'marker','.')
plot(th, dt,'.r')
plot(th, dt_s, '.k')
legend('sinth', 'dt', 'dt with updated slope')
