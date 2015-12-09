function Twolink
% Inverse dynamics for a 2 link arm using Newton Euler approach
% Moment of inertia about center of mass of each link
% zero position has robot along X axis, angle is standard angle
% Standard coordinate system (x right, y up)

% define gravity
syms G real
gravity = [ 0, -G ]' 

% the joint angles
syms a1 a2 real

% Define rotation matrices that allow us to do the forward kinematics.
% R01 rotates from the link 1 frame to the link 0 frame.
r01 = [ cos(a1),-sin(a1)
sin(a1),cos(a1) ]
r10 = r01'
r12 = [ cos(a2),-sin(a2)
sin(a2),cos(a2) ]
r21 = r12'

% masses
syms m1 m2 real
% link lengths
syms l1 l2 real
% location of center of mass along links
syms l1cm l2cm real
% moments of inertia
syms I1 I2 real

% angular velocities
syms da1 da2 real

% angular accelerations
syms d2a1 d2a2 real

% Define link lengths.

s1 = [l1,0]'
s2 = [l2,0]'

% Define the location of the proximal end of each link (in world coordinates)

p1 = [0, 0]'
p2 = r01*s1 + p1

% Define the location of the center of mass with respect
% to the proximal end of each link. 

cm1 = r01*[l1cm,0]'
cm2 = r01*r12*[l2cm,0]'

cm1 = simplify(expand(cm1))
cm2 = simplify(expand(cm2))

% Compute the location of the center of mass of each link.

q1 = p1 + cm1
q2 = p2 + cm2

% Compute the linear velocity and acceleration of each link cm.

qd1 = fulldiff( q1, {a1, a2, da1, da2} ) 
qd2 = fulldiff( q2, {a1, a2, da1, da2} ) 

qdd1 = fulldiff( qd1, {a1, a2, da1, da2} ) 
qdd2 = fulldiff( qd2, {a1, a2, da1, da2} ) 

qdd1 = simplify( expand( qdd1 ) )
qdd2 = simplify( expand( qdd2 ) )

% Compute net link forces.

f1 = m1 * qdd1 - m1 * gravity
f2 = m2 * qdd2 - m2 * gravity

% Compute the angular velocity of each link.

w1 = da1
w2 = w1 + da2

% Compute the angular acceleration of each link.

wd1 = fulldiff( w1, {a1, a2, da1, da2} )
wd2 = fulldiff( w2, {a1, a2, da1, da2} )

% Compute net link torques.
% Let's get the things expressed with I_joint

n1 = I1*wd1
n2 = I2*wd2

%Compute forces at the joints.*)

f12 = f2
f01 = f1 + f12

f12 = simplify( expand( f12 ) )
f01 = simplify( expand( f01 ) )

%Compute torques at the joints.*)

tau2 = n2 + cross2d( cm2, f2 )
tau1 = n1 + tau2 + cross2d( cm1, f1 ) + cross2d( p2-p1, f12 )

tau2 = simplify( expand( tau2 ) )
tau1 = simplify( expand( tau1 ) )

%*************************************************************)
%
% tau2 =
%   I2*d2a1 + I2*d2a2 + d2a1*l2cm^2*m2 + d2a2*l2cm^2*m2
%   + da1^2*l1*l2cm*m2*sin(a2) + d2a1*l1*l2cm*m2*cos(a2)
%   + G*l2cm*m2*cos(a1 + a2)
% 
% tau1 =
%   I1*d2a1 + I2*d2a1 + I2*d2a2 + d2a1*l1cm^2*m1 + d2a1*l1^2*m2
%   + d2a1*l2cm^2*m2 + d2a2*l2cm^2*m2 + G*l2cm*m2*cos(a1 + a2)
%   + G*l1cm*m1*cos(a1) + G*l1*m2*cos(a1) - da2^2*l1*l2cm*m2*sin(a2)
%   + 2*d2a1*l1*l2cm*m2*cos(a2) + d2a2*l1*l2cm*m2*cos(a2)
%   - 2*da1*da2*l1*l2cm*m2*sin(a2)

ccode( tau1 )

ccode( tau2 )

%*************************************************************)
% Do some optimization by generating matlab function.

% matlabFunction( tau1, tau2, 'file', 'matdyn2.m' )


%*************************************************************)
% Compute terms of H in H*acceleration = stuff

% [c,t] = coeffs(tau5,{d2a1 d2a2 d2a3 d2a4 d2a5})

%*************************************************************)
%*************************************************************)
end

function [ axb ] = cross2d( a, b )
%CROSS2(A,B) cross product of 2d vectors
% returns A_1 B_2 - A_2 B_1
assert(numel(a)==2);
assert(numel(b)==2);
axb = a(1)*b(2)-a(2)*b(1);
end