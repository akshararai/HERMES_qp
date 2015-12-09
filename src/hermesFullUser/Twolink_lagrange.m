function Twolink_lagrange
clear all
clc
syms r1 r2 l1 l2 m1 m2 I1 I2 fq1(t) fq2(t) fdq1(t) fdq2(t) fd2q1(t) fd2q2(t) g dq1 dq2 d2q2 d2q1 q1 q2 t

% hip
    x1 = r1*cos(q1);
    y1 = r1*sin(q1);
    
% knee
    x2 = l1*cos(q1) + r2 * cos(q1+q2);
    y2 = l1*sin(q1) + r2 * sin(q1+q2);
    
    x1 = subs(x1, q1, fq1);
    x1 = subs(x1, q2, fq2);
    y1 = subs(y1, q1, fq1);
    y1 = subs(y1, q2, fq2);

    x2 = subs(x2, q1, fq1);
    x2 = subs(x2, q2, fq2);

    y2 = subs(y2, q1, fq1);
    y2 = subs(y2, q2, fq2);


    v1 = sqrt(diff(x1,t).^2 + diff(y1,t).^2);
    v2 = sqrt(diff(x2,t).^2 + diff(y2,t).^2);
    
    keyboard
    
    v1 = subs(v1, diff(fq1(t), t), dq1);
    v1 = subs(v1, diff(fq2(t), t), dq2);
    v1 = subs(v1, fq1(t), q1);
    v1 = subs(v1, fq2(t), q2);

    v2 = subs(v2, diff(fq1(t), t), dq1);
    v2 = subs(v2, diff(fq2(t), t), dq2);
    v2 = subs(v2, fq1(t), q1);
    v2 = subs(v2, fq2(t), q2);

    keyboard
    
    T = 0.5*(m1*v1^2 + m2*v2^2) + 0.5*(I1*dq1^2 + I2*(dq1+dq2)^2);

    V = m1*g*r1*sin(q1) + m2*g*(r2*sin(q1+q2) + l1*sin(q1));
    
    L = T - V;
    
    
    dLq1  = diff(L, q1);
    dLq2  = diff(L, q2);

    dLdq1 = diff(L, dq1);
    dLdq2 = diff(L, dq2);

    fdLdq1 = subs(dLdq1, q1, fq1);
    fdLdq1 = subs(fdLdq1, dq1, fdq1);
    fdLdq1 = subs(fdLdq1, d2q1, fd2q1);
    fdLdq1 = subs(fdLdq1, q2, fq2);
    fdLdq1 = subs(fdLdq1, dq2, fdq2);
    fdLdq1 = subs(fdLdq1, d2q2, fd2q2);

    
    fdLdq2 = subs(dLdq2, q1, fq1);
    fdLdq2 = subs(fdLdq2, dq1, fdq1);
    fdLdq2 = subs(fdLdq2, d2q1, fd2q1);
    fdLdq2 = subs(fdLdq2, q2, fq2);
    fdLdq2 = subs(fdLdq2, dq2, fdq2);
    fdLdq2 = subs(fdLdq2, d2q2, fd2q2);

%     keyboard
    
    tmp1 = diff(fdLdq1, t);
    tmp2 = diff(fdLdq2, t);
    
%     keyboard
    
    tmp1 = subs(tmp1, fq1(t), q1);
    tmp1 = subs(tmp1, fq2(t), q2);
    tmp1 = subs(tmp1, diff(fq1(t), t), dq1);
    tmp1 = subs(tmp1, diff(fdq1(t), t), d2q1);
    tmp1 = subs(tmp1, diff(fq2(t), t), dq2);
    dtdLdq1 = subs(tmp1, diff(fdq2(t), t), d2q2);

    tmp2 = subs(tmp2, fq1(t), q1);
    tmp2 = subs(tmp2, fq2(t), q2);
    tmp2 = subs(tmp2, diff(fq1(t), t), dq1);
    tmp2 = subs(tmp2, diff(fdq1(t), t), d2q1);
    tmp2 = subs(tmp2, diff(fq2(t), t), dq2);
    dtdLdq2 = subs(tmp2, diff(fdq2(t), t), d2q2);
    
%     keyboard
    
    tau1 = dtdLdq1 - dLq1;
    tau2 = dtdLdq2 - dLq2;
    
    t1 = simplify(expand(tau1))
    t2 = simplify(expand(tau2))
end