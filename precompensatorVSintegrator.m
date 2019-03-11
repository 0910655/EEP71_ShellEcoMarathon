%% Parameters
load('parameters_sys_id_2param_50V_20-01-19.mat');

%% State Space model

A = [-(bm*(n^2) + bg)/(Jm*(n^2) + Jg) Kt*n/(Jm*(n^2) + Jg); 
    -(n*Ke)/La -Ra/La];
B = [0; 
    1/La];
C = eye(2);
D = [0];

sysc = ss(A,B,C,D);

%% State space model met integrator

Aa = [-(bm*(n^2) + bg)/(Jm*(n^2) + Jg) Kt*n/(Jm*(n^2) + Jg) 0; 
    -(n*Ke)/La -Ra/La 0;
    1 0 0];
Ba = [0; 
    1/La;
    0];
Br = [0 ; 0; -1];
Ca = [1 0 0;
      0 1 0];
Da = [0;0];

sysci = ss(Aa,Ba,Ca,Da);

%% Feedback constante berekenen met LQR integrator

co = ctrb(sysci);
controllability = rank(co)

Q = Ca'*Ca;
Q(1,1) = 1;
Q(2,2) = 1;
Q(3,3) = 1;
R = 1;
Kci = lqr(Aa,Ba,Q,R)

%% Feedback constante berekenen met LQR precompensator

co = ctrb(sysc);
controllability = rank(co)

Q = C'*C;
Q(1,1) = 1;
Q(2,2) = 1;
R = 1;
Kc = lqr(sysc,Q,R)

%% Pre-compensation 
Cn = [1 0];
sys_ss = ss(A,B,Cn,0);
Nbar = rscale(sys_ss,Kc)
sys_cl_c = ss((A-B*Kc),B*Nbar,C,D);

%% Plot Open en Closed-loop stap responsie precompensator
close all
step(sysc);
hold on
title('precompensator');
step(sys_cl_c)
legend('Open-loop','Closed-loop');
hold off

%% Plot Open en Closed-loop stap responsie integrator
figure
sys_clc = ss(Aa-Ba*Kdi,Br,Ca,Da);
step(sysc);
hold on
title('integrator');
step(sys_clc)
legend('Open-loop','Closed-loop');
hold off
