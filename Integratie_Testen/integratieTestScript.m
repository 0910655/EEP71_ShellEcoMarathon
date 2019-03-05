%% Parameters
n = 16.5; % Tandwielverhouding
load('parameters_sys_id_2param_50V_20-01-19.mat');

%% State Space model

A = [-(bm*(n^2) + bg)/(Jm*(n^2) + Jg) Kt*n/(Jm*(n^2) + Jg); 
    -(n*Ke)/La -Ra/La];
B = [0; 
    1/La];
C = [1 0];
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
Ca = [1 0 0];
Da = [0];

sysci = ss(Aa,Ba,Ca,Da);

%% C2D

Tsc = 1/20000;

sysdi = c2d(sysci, Tsc);

Ad = sysdi.A;
Bd = sysdi.B;
Cd = sysdi.C;
Dd = sysdi.D;

%% Feedback constante berekenen met LQR

co = ctrb(sysci);
controllability = rank(co)

Q = Ca'*Ca;
Q(1,1) = 10000;
Q(2,2) = 1;
Q(3,3) = 10000;%2.32;
R = 1;
Kci = lqr(Aa,Ba,Q,R);
Kdi = dlqr(Ad,Bd,Q,R);

%% Integral
close all
sys_cl = ss(Aa-Ba*Kci,Br,Ca,Da);
step(sys_cl)

sys_cldd = c2d(sys_cl,Tsc);
eigD = eig(sys_cldd)

figure

sys_cld = ss(Ad-Bd*Kdi,Br,Cd,Dd,Tsc);
step(sys_cld)
eig(sys_cld)

%% Observer constante L
sys_clL = ss(A-B*Kci(1:2),B,C,D);
eigenVal = eig(sys_clL);
eigenVal = eigenVal*0.52;