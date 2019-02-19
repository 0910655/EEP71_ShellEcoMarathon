%% Parameters

Jm = 0.00013932; %Rotor Inertia (J) kg.m^2
Jg = 0.002098; %Rotor Inertia (J) kg.m^2

bm = 0.00077676;  %Viscous Friction Coefficient (b) 
bg = 0.053853;  %Viscous Friction Coefficient (b)

Kt = 0.083796; %Torque Constant (Kt) N.m/A
La = 0.00017073; %Motor Inductance (La) H
Ra = 2.1262; %Motor Resistance (Ra) ohms 

Ke = 0.069585; %Back EMF Constant (Ke)  V/rpm
n = 16.5; % Tandwielverhouding 

%% State Space model

A = [-(bm*(n^2) + bg)/(Jm*(n^2) + Jg) Kt*n/(Jm*(n^2) + Jg); 
    -(n*Ke)/La -Ra/La];
B = [0; 
    1/La];
C = [1 0];
D = [0];

sysc = ss(A,B,C,D);

%% C2D

Tsc = 1/20000;

sysd = c2d(sysc, Tsc);

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

%% Feedback constante berekenen met LQR

co = ctrb(sysd);
controllability = rank(co);

Q = C'*C;
Q(1,1) = 1;
Q(2,2) = 3900;
R = 1;
Kc = lqr(sysc,Q,R);
Kc2 = dlqr(Ad,Bd,Q,R);
Kc3 = lqrd(A,B,Q,R,Tsc);

%% Integral

sys_cl_1 = ss((A-B*Kc),B,C,D);
eigVal = eig(sys_cl_1);
integralPole = min(eigVal)*27

sys_cl_d1 = ss((Ad-Bd*Kc2),Bd,Cd,Dd,Tsc);
eigValD = eig(sys_cl_d1);
integralPoleD = max(abs(eigValD)) + ((max(abs(eigValD)))*0.60 - 1) %0.891
eigDD = [eigValD' integralPoleD];


Aa = [-(bm*(n^2) + bg)/(Jm*(n^2) + Jg) Kt*n/(Jm*(n^2) + Jg) 0; 
    -(n*Ke)/La -Ra/La 0;
    1 0 0];
Ba = [0; 
    1/La;
    0];
Br = [0 ; 0; -1];
Ca = [1 0 0];
Da = [0];
Ka = place(Aa,Ba,[eigVal' integralPole])

sys_cl = ss(Aa-Ba*Ka,Br,Ca,Da);
step(sys_cl)

%% C2D

sysd = c2d(sys_cl, Tsc);
eigD = eig(sysd)

sys_c2 = ss(Aa, Ba, Ca, Da);
sysd = c2d(sys_c2, Tsc);

Adi = sysd.A;
Bdi = sysd.B;
Cdi = sysd.C;
Ddi = sysd.D;

Kd = place(Adi,Bdi,eigD)
Kd2 = place(Adi,Bdi,eigDD)

sys_cld = ss(Adi-Bdi*Kd2,Br,Cdi,Ddi,Tsc);
figure
step(sys_cld)

%% Observer constante L

eigenVal = eig(sys_cl_1);
eigenVal = eigenVal*100;
Kp = place(A,B,eigenVal);
sys_cl_c = ss((A-B*Kp),B,C,D);

sys_cl_dL = c2d(sys_cl_c,Tsc);

st1 = stepinfo(c2d(sys_cl_1,Tsc));
st2 = stepinfo(sys_cl_dL);
diff = (st1.SettlingTime)/(st2.SettlingTime);

eigenValD = eig(sys_cl_dL);
L = place(Ad',(C)',eigenValD)';