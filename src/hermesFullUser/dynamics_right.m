function dynamics_right

close all
global W_knee
% 962 - unstable
files = { 'd00973', 'd00974', 'd00975', 'd00976', 'd00977','d00978'};
% 
% for i=1:length(files)
%     regress_individual_files(files{i});
% end
regress_together(files);
% 
% for i=1:length(files)
%     regress_individual_files_hip(files{i});
% end

regress_together_hip(files);

end

function regress_individual_files(file)

    [b,a] = butter(2, 0.005, 'low');
    [b1,a1] = butter(2,0.01,'low');           % IIR filter design

    global W_knee
    clmcplot_convert(file);
    q = ans(:,17);
    
    q1 = ans(:,2);
    
    qd = -ans(:,18);
    qd1 = qd;


    tau = ans(:,21);
    tau1 = tau;

    uv = ans(:,31);
    uv1 = uv;

   
    clear xpd xpdd f fd

    qdf = filtfilt(b,a,qd1); 

    tauf = filtfilt(b1,a1,tau1);                    % zero-phase filtering       

    qdd = [diff(qdf); 0].*1000;
    qdd = filtfilt(b,a,qdd);

    X = [-cos(q+q1) qdd ones(size(qdf))];
    X = X';

    lambda = 0.001;
    W = tauf'*X'*pinv(X*X' + lambda.*eye(size(X*X')))
    W_knee = W;
    
    tau_recons = W * X;

    figure(1), clf, hold on
    plot(tauf)
    plot(tau_recons,'r')
    keyboard
end

function regress_individual_files_hip(file)
global W_knee
[b,a] = butter(2, 0.005, 'low');
[b1,a1] = butter(2,0.01,'low');           % IIR filter design

    clmcplot_convert(file);
    %hip
    q1 = ans(:,2);
    

    qd1 = -ans(:,3);
    

    tau1 = ans(:,6);
    
    %%knee
    q2 = ans(:,17);
    
    qd2 = -ans(:,18);
    
    tau2 = ans(:,21);
    
    qdf1 = filtfilt(b,a,qd1); 

    qdf2 = filtfilt(b,a,qd2);
    
    tauf1 = filtfilt(b1,a1,tau1);                    % zero-phase filtering       
    tauf2 = filtfilt(b1,a1,tau2);
    
    qdd1 = [diff(qdf1); 0].*1000;
    qddf1 = filtfilt(b,a,qdd1);
    qdd2 = [diff(qdf2); 0].*1000;
    qddf2 = filtfilt(b,a,qdd2);
  


    
    consts = W_knee(2).*qddf2 - W_knee(1).*cos(q2+q1);
    
    u = tauf1;% -consts;
    
%     X = [sin(q) qdd ones(size(qdf))];
    X = [qddf2 -cos(q1+q2) cos(q1) qddf2.*cos(q2) ones(size(q2))];
    X = X';

    lambda = 0.1;
    W = u'*X'*pinv(X*X' + lambda.*eye(size(X*X')))
%     W = W+W1;

%     save(['X' file '.mat'],'X1')
%     save(['W' file '.mat'],'W1')
%     save(['U' file '.mat'],'uv1')
    tau_recons = W * X;

    figure(1), clf, hold on
    plot(u)
    plot(tau_recons,'r')
    keyboard
end

function regress_together(files)

global W_knee

    [b,a] = butter(2, 0.005, 'low');
    [b1,a1] = butter(2,0.01,'low');           % IIR filter design
    X=[];
    W=[];
    U=[];
    
    for file_no = 1:length(files)
        clmcplot_convert(files{file_no});
        q = ans(:,17);

        q1 = ans(:,2);

        qd = -ans(:,18);
        qd1 = qd;


        tau = ans(:,21);
        tau1 = tau;

        uv = ans(:,31);
        uv1 = uv;

 

        clear xpd xpdd f fd

        qdf = filtfilt(b,a,qd1); 
        
        tauf = filtfilt(b1,a1,tau1);                    % zero-phase filtering       
        
        qdd = [diff(qdf); 0].*1000;
        qdd = filtfilt(b,a,qdd);
        
        X1 = [cos(q+q1) qdd ones(size(qdf))];
       
        X= [X; X1];
        U = [U; tauf];
    end
    lambda = 0.01;
    X = X';
    
    W = U'*X'*pinv(X*X' + lambda.*eye(size(X*X')))

    W_knee = W;
    tau_recons = W * X;

    figure(1), clf, hold on
    plot(U)
    plot(tau_recons,'r')

end


function regress_together_hip(files)
global W_knee

    [b,a] = butter(2, 0.005, 'low');
    [b1,a1] = butter(2,0.01,'low');           % IIR filter design
    X=[];
    W=[];
    U=[];
    
    for file_no = 1:length(files)
        clmcplot_convert(files{file_no});

        q1 = ans(:,2);
    

        qd1 = -ans(:,3);


        tau1 = ans(:,6);

        %%knee
        q2 = ans(:,17);

        qd2 = -ans(:,18);

        tau2 = ans(:,21);

        qdf1 = filtfilt(b,a,qd1); 

        qdf2 = filtfilt(b,a,qd2);

        tauf1 = filtfilt(b1,a1,tau1);                    % zero-phase filtering       
        tauf2 = filtfilt(b1,a1,tau2);

        qdd1 = [diff(qdf1); 0].*1000;
        qddf1 = filtfilt(b,a,qdd1);
        qdd2 = [diff(qdf2); 0].*1000;
        qddf2 = filtfilt(b,a,qdd2);

        consts = W_knee(2).*qddf2 - W_knee(1).*cos(q2+q1);
    
        tauf1 = tauf1;% - consts;

%       X = [sin(q) qdd ones(size(qdf))];
        X1 = [qddf2 cos(q1+q2) qddf2.*cos(q2) ones(size(q2))];
        X = [X; X1];
        U = [U; tauf1];

    end
    lambda = 0.1;
    X = X';
    
    W = U'*X'*pinv(X*X' + lambda.*eye(size(X*X')))

%     W_knee = W;

    tau_recons = W * X;

    figure(1), clf, hold on
    plot(U)
    plot(tau_recons,'r')
end

