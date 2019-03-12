%% Parameters

Jm = 0.0000542; %Rotor Inertia (J) kg.m^2 Motor
Jg = 0.0000542; %Rotor Inertia (J) kg.m^2 Gears
Kt = 93.4e-3; %Torque Constant (Kt) N.m/A
Ke = (1/102)*(60/(2*pi)); %Back EMF Constant (Ke)   V/rad/s (1/(102) V/rpm)
La = 0.423e-3; %Motor Inductance (La) H
Ra = 0.608; %Motor Resistance (Ra) ohms 
bm = 18.2e-6;  %Viscous Friction Coefficient (b) Motor
bg = 18.2e-6;  %Viscous Friction Coefficient (b) Gears
n = 16.5;    %Tandwielverhouding
%% Data conversie

Fs = 50000;
Ts = 1/Fs;
Time = (0:Ts:((size(Speed,1)-1)/Fs))';
%Time = (0:Ts:0.315)';

voltageTime = struct('time',Time,'signals',struct('values',Voltage,'dimensions', 1, 'label',''),'blockName', 'struct_motor_voltage');

%Speed = Speed((2*Fs):(2.315*Fs));
%Voltage = Voltage((2*Fs):(2.315*Fs));

%subplot(2,1,1);
plot(Time,Speed);
title('Speed (rad/s)');
hold on
%subplot(2,1,2)
plot(Time,Voltage);
title('Voltage (V)')
hold off
%save meting_v1_50V_20-2-19 Voltage Speed Time voltageTime % verander de naam data per meting

