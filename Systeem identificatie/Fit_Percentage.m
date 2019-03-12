%% State Space model

A = [-(bm*(n^2) + bg)/(Jm*(n^2) + Jg) Kt*n/(Jm*(n^2) + Jg); 
    -(n*Ke)/La -Ra/La];
B = [0; 
    1/La];
C = [1 0];
D = [0];

sysc = ss(A,B,C,D);

Sim = lsim(sysc,Voltage,Time);
goodnessOfFit(Speed,Sim, 'MSE')