function dynamics_parameters
global freq
freq = 1000;
close all
%t2 = w11 * (d2a1 + d2a2) + w12 *(d1a1^2 sin(z2) + d2a1*cos(a2)) + w13 * cos(a1+a2) + w14 

% t1 = w21* d2a1 + w22 * d2a2 + w23 * cos(a1+a2) + w24* cos(a1) + w25 *
% (d1a2*sin(a2) + 2*d1a1*d1a2*sin(a2)) + w26

file = {'d00895' ; 'd00896' }%; 'd00897';'d00894';'d00898';'d00901'};% 'd00898'; 'd00899'}; %'d00900'; 'd00901'; 'd00902'};{'d00892'; 'd00893'; 'd00894' ; 
% 
regress_ind(file)
% 
[W1_all, W2_all] = regress_together(file)

load Wh.mat
load Wk.mat

file_test = {'d00893' ; 'd00899'; 'd00900'; 'd00902'};
reconstruct(file_test, Wk, Wh);
end

function regress_ind(file)
global freq
  for i=1:length(file)
    q = clmcplot_convert(file{i});

    a1 = q(:,2);
    d1a1 = q(:,3);
    [b,a] = butter(2,0.01,'low');           % IIR filter design
    d1a1 = filtfilt(b,a,d1a1); 

    d2a1 = diff(d1a1).*freq;
    d2a1 = [d2a1 ; d2a1(end)];

    t1 = q(:,6);
    t1 = filtfilt(b,a,t1); 

    a2 = q(:,18);
    d1a2 = q(:,19);
    [b,a] = butter(2,0.01,'low');           % IIR filter design
    d1a2 = filtfilt(b,a,d1a2); 
    
    d2a2 = diff(d1a2).*freq;
    d2a2= [d2a2 ; d2a2(end)];

    t2 = q(:,22);
    t2 = filtfilt(b,a,t2);

    lambda = 0.01;
    


    Xk = [d2a1 d2a2 d2a1.*cos(a2) cos(a1+a2) sin(a2).*(d1a1.^2)];
    Xh = [d2a1 d2a2 cos(a1+a2) cos(a1) (-sin(a2).*d1a1.^2 + 2.*d2a1.*cos(a2)+d2a2.*cos(a2) - 2*d1a1.*d1a2.*sin(a2))];
    Tk = t1;
    Th = t2;

    
    
%     Xk = bsxfun(@rdivide, bsxfun(@minus, Xk, mean(Xk)), std(Xk));
    Xk = [Xk ones(size(Tk))];
%     Tk = bsxfun(@rdivide, bsxfun(@minus, Tk, mean(Tk)), std(Tk));
    Wk = Tk'*Xk*pinv(Xk'*Xk + lambda.*eye(size(Xk'*Xk)))
  
%     Xh = bsxfun(@rdivide, bsxfun(@minus, Xh, mean(Xh)), std(Xh));
    Xh = [Xh ones(size(Th))];
%     Th =  bsxfun(@rdivide, bsxfun(@minus, Th, mean(Th)), std(Th));
  
    Wh = Th'*Xh*pinv(Xh'*Xh + lambda.*eye(size(Xh'*Xh)))

    W1 = Wk;
    W2 = Wh;
    t1_r = W1*Xk';

    figure
    plot(t1_r)
    hold on
    plot(Tk,'r')
    legend('reconstructed', 'original')

    t2_r = W2*Xh';
    figure
    plot(t2_r)
    hold on
    plot(Th,'r')
    legend('reconstructed', 'original')
    
  end
end

function [Wk, Wh] = regress_together(file)
global freq
Xk = [];
Wk = [];
Tk = [];

Xh = [];
Wh = [];
Th = [];

  for i=1:length(file)
    q = clmcplot_convert(file{i});

    a1 = q(:,2);
    d1a1 = q(:,3);
    [b,a] = butter(2,0.01,'low');           % IIR filter design
    d1a1 = filtfilt(b,a,d1a1); 

    d2a1 = diff(d1a1).*freq;
    d2a1 = [d2a1 ; d2a1(end)];

    t1 = q(:,6);
    t1 = filtfilt(b,a,t1); 

    a2 = q(:,18);
    d1a2 = q(:,19);
    [b,a] = butter(2,0.01,'low');           % IIR filter design
    d1a2 = filtfilt(b,a,d1a2); 

    d2a2 = diff(d1a2).*freq;
    d2a2= [d2a2 ; d2a2(end)];

    t2 = q(:,22);
    t2 = filtfilt(b,a,t2);

    lambda = 0;

    X1 = [d2a1 d2a2 d2a1.*cos(a2) cos(a1+a2) sin(a2).*(d1a1.^2)];
    
    Xk =[Xk; X1];
    Tk = [Tk; t1];
    
    X2 = [d2a1 d2a2 cos(a1+a2) cos(a1) (-sin(a2).*d1a1.^2 + 2.*d2a1.*cos(a2)+d2a2.*cos(a2) - 2*d1a1.*d1a2.*sin(a2))];
    Xh = [Xh; X2];
    Th = [Th; t2];

  end
  
%   Xk = bsxfun(@rdivide, bsxfun(@minus, Xk, mean(Xk)), std(Xk));
  Xk = [Xk ones(size(Tk))];
%   Tk = bsxfun(@rdivide, bsxfun(@minus, Tk, mean(Tk)), std(Tk));
  Wk = Tk'*Xk*pinv(Xk'*Xk + lambda.*eye(size(Xk'*Xk)))
  
%   Xh = bsxfun(@rdivide, bsxfun(@minus, Xh, mean(Xh)), std(Xh));
  Xh = [Xh ones(size(Th))];
%   Th =  bsxfun(@rdivide, bsxfun(@minus, Th, mean(Th)), std(Th));
  
  Wh = Th'*Xh*pinv(Xh'*Xh + lambda.*eye(size(Xh'*Xh)))

    t1_r = Wk*Xk';
    figure
    plot(t1_r)
    hold on
    plot(Tk,'r')
    legend('reconstructed', 'original')

    t2_r = Wh*Xh';
    figure
    plot(t2_r)
    hold on
    plot(Th,'r')
    legend('reconstructed', 'original')

    save('Wk.mat', 'Wk')
    save('Wh.mat', 'Wh')

end

function reconstruct(file, Wk, Wh)
global freq
for i = 1: length(file)
    q = clmcplot_convert(file{i});
    a1 = q(:,2);
    d1a1 = q(:,3);
    [b,a] = butter(2,0.01,'low');           % IIR filter design
    d1a1 = filtfilt(b,a,d1a1); 

    d2a1 = diff(d1a1).*freq;
    d2a1 = [d2a1 ; d2a1(end)];

    t1 = q(:,6);
    t1 = filtfilt(b,a,t1); 

    a2 = q(:,18);
    d1a2 = q(:,19);
    [b,a] = butter(2,0.01,'low');           % IIR filter design
    d1a2 = filtfilt(b,a,d1a2); 

    d2a2 = diff(d1a2).*freq;
    d2a2= [d2a2 ; d2a2(end)];

    t2 = q(:,22);
    t2 = filtfilt(b,a,t2);

    lambda = 0.01;

    Xk = [d2a1 d2a2 d2a1.*cos(a2) cos(a1+a2) sin(a2).*(d1a1.^2)];
    
    Xh = [d2a1 d2a2 cos(a1+a2) cos(a1) (-sin(a2).*d1a1.^2 + 2.*d2a1.*cos(a2)+d2a2.*cos(a2) - 2*d1a1.*d1a2.*sin(a2))];

    Tk = t1;
    Th = t2;
%   Xk = bsxfun(@rdivide, bsxfun(@minus, Xk, mean(Xk)), std(Xk));
  Xk = [Xk ones(size(Tk))];
%   Tk = bsxfun(@rdivide, bsxfun(@minus, Tk, mean(Tk)), std(Tk));
  
%   Xh = bsxfun(@rdivide, bsxfun(@minus, Xh, mean(Xh)), std(Xh));
  Xh = [Xh ones(size(Th))];
%   Th =  bsxfun(@rdivide, bsxfun(@minus, Th, mean(Th)), std(Th));
  
    t1_r = Wk*Xk';
    figure
    plot(t1_r)
    hold on
    plot(Tk,'r')
    legend('reconstructed', 'original')

    t2_r = Wh*Xh';
    figure
    plot(t2_r)
    hold on
    plot(Th,'r')
    legend('reconstructed', 'original')
end

end