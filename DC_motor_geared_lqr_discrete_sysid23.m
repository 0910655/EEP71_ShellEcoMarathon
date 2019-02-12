%% Parameters

n = 17 % Tandwielverhouding 

Jm = 0.00013932; %Rotor Inertia (J) kg.m^2
Jg = 0.002098; %Rotor Inertia (J) kg.m^2

bm = 0.00077676;  %Viscous Friction Coefficient (b) 
bg = 0.053853;  %Viscous Friction Coefficient (b)

Kt = 0.083796; %Torque Constant (Kt) N.m/A
La = 0.00017073; %Motor Inductance (La) H
Ra = 2.1262; %Motor Resistance (Ra) ohms 

Ke = 0.069585; %Back EMF Constant (Ke)  V/rpm
n = 16.5

%% State Space model

A = [-(bm*(n^2) + bg)/(Jm*(n^2) + Jg) Kt*n/(Jm*(n^2) + Jg); 
    -(n*Ke)/La -Ra/La];

B = [0; 
    1/La];
C = eye(2);
D = [0;0];

sysc = ss(A,B,C,D);

step(sysc);

%% C2D

Tsc = 1/100000;

sysd = c2d(sysc, Tsc);

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

%% Feedback constante berekenen met LQR

co = ctrb(sysd);
controllability = rank(co)

Q = C'*C;
Q(1,1) = 10;
Q(2,2) = 1;
R = 1;
Kc = lqr(sysd,Q,R)

%% Pre-compensation 
Cn = [1 0];
sys_ss = ss(A,B,Cn,0);
Nbar = rscale(sys_ss,Kc);

sys_cl_d = ss((Ad-Bd*Kc),Bd*Nbar,Cd,Dd,Tsc);
step(sys_cl_d)


%% Observer constante L
sys_cl_c = ss((A-B*Kc),B*Nbar,Cn,0);
eig(c2d(sys_cl_c,Tsc))

eigenVal = eig(sys_cl_c);
eigenVal = eigenVal*5;
Kp = place(A,B,eigenVal);
eig(sys_cl_c);
sys_cl_c = ss((A-B*Kp),B*Nbar,Cn,0);

sys_cl_dL = c2d(sys_cl_c,Tsc);
eigenValD = eig(sys_cl_dL)
L = place(Ad',(Cn)',eigenValD)';