function dynamics_right_2

close all
global W_knee
% 962 - unstable
files = {'d01021', 'd01022', 'd01023', 'd01024', 'd01025', 'd01026', 'd01027', 'd01028', 'd01029', 'd01032', 'd01033', 'd01034', 'd01035', 'd01036', 'd01037', 'd01038', 'd01039', 'd01040', 'd01041','d01042','d01043','d01044'};
% % % 
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
    global W_knee
    clmcplot_convert(file);
    q1 = ans(:,2);
    

    qd1 = -ans(:,3);
    

    tau1 = ans(:,6);
    
    %%knee
    q2 = ans(:,18);
    
    qd2 = -ans(:,19);
    
    tau2 = ans(:,22);
    

    
    [b,a] = butter(2, 0.005, 'low');
    [b1,a1] = butter(2,0.01,'low');           % IIR filter design
    qdf1 = filtfilt(b,a,qd1); 

    qdf2 = filtfilt(b,a,qd2);
    
    tauf1 = filtfilt(b1,a1,tau1);                    % zero-phase filtering       
    tauf2 = filtfilt(b1,a1,tau2);
    
    qdd1 = [diff(qdf1); 0].*1000;
    qddf1 = filtfilt(b,a,qdd1);
    qdd2 = [diff(qdf2); 0].*1000;
    qddf2 = filtfilt(b,a,qdd2);

    consts = 0.97.*qddf1 - 27.53.*cos(q2+q1); 
    u = tauf2 - consts; 
    
    X = [qddf1.*cos(q2) sin(q2).*(qdf1.^2) ones(size(q1))];
    X = X';

    lambda = 0.001;
    
    W = u'*X'*pinv(X*X' + lambda.*eye(size(X*X')))
    W_knee = W;
    
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

function regress_individual_files_hip(file)
global W_knee
[b,a] = butter(2, 0.005, 'low');
[b1,a1] = butter(2,0.001,'low');           % IIR filter design

    clmcplot_convert(file);
    %hip
    q1 = ans(:,2);
    

    qd1 = -ans(:,3);
    

    tau1 = ans(:,6);
    
    %%knee
    q2 = ans(:,18);
    
    qd2 = -ans(:,19);
    
    tau2 = ans(:,22);
    
    qdf1 = filtfilt(b,a,qd1); 

    qdf2 = filtfilt(b,a,qd2);
    
    tauf1 = filtfilt(b1,a1,tau1);                    % zero-phase filtering       
    tauf2 = filtfilt(b1,a1,tau2);
    
    qdd1 = [diff(qdf1); 0].*1000;
    qddf1 = filtfilt(b,a,qdd1);
    qdd2 = [diff(qdf2); 0].*1000;
    qddf2 = filtfilt(b,a,qdd2);
  


    
    consts = 27.*cos(q2+q1) - 2*0.97.*qddf1.*cos(q2);
    
    u = tauf1 - consts;
    
%     X = [sin(q) qdd ones(size(qdf))];
    X = [qddf1 ones(size(q2)) cos(q1)];
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
    q1 = ans(:,2);
    

    qd1 = -ans(:,3);
    

    tau1 = ans(:,6);
    
    %%knee
    q2 = ans(:,18);
    
    qd2 = -ans(:,19);
    
    tau2 = ans(:,22);
    
    qdf1 = filtfilt(b,a,qd1); 

    qdf2 = filtfilt(b,a,qd2);
    
    tauf1 = filtfilt(b1,a1,tau1);                    % zero-phase filtering       
    tauf2 = filtfilt(b1,a1,tau2);
    
    qdd1 = [diff(qdf1); 0].*1000;
    qddf1 = filtfilt(b,a,qdd1);
    qdd2 = [diff(qdf2); 0].*1000;
    qddf2 = filtfilt(b,a,qdd2);
    
    consts = 0.97.*qddf1 - 27.53.*cos(q1+q2);
    tauf2 = -tauf2; %-consts;
    
    X1 = [qddf1 sin(q2).*(qdf1.^2) -cos(q1+q2) ones(size(q1))];
    
    X= [X; X1];
    U = [U; tauf2];
    end
    lambda = 0.1;
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

    [b,a] = butter(2, 0.001, 'low');
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
    q2 = ans(:,18);
    
    qd2 = -ans(:,19);
    
    tau2 = ans(:,22);

    qdf1 = filtfilt(b,a,qd1); 

        qdf2 = filtfilt(b,a,qd2);

        tauf1 = filtfilt(b1,a1,tau1);                    % zero-phase filtering       
        tauf2 = filtfilt(b1,a1,tau2);

        qdd1 = [diff(qdf1); 0].*1000;
        qddf1 = filtfilt(b,a,qdd1);
        qdd2 = [diff(qdf2); 0].*1000;
        qddf2 = filtfilt(b,a,qdd2);

        consts = 27.*cos(q2+q1) - 2*0.97.*qddf1.*cos(q2);
    
        tauf1 = tauf1 - consts;    
%         tauf1 = -tauf1; %-consts;

%       X = [sin(q) qdd ones(size(qdf))];
        X1 = [qddf1 ones(size(q2)) -cos(q1)];
        
        X = [X; X1];
        U = [U; -tauf1];

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



