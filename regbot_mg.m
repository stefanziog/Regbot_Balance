%% Simscape multibody model og Regbot in balance
% initial setup with motor velocity controller 
% this is intended as simulation base for balance control.
%
close all
clear

%% Simulink model name
model='regbot_1mg';

%% parameters for REGBOT
% motor
RA = 3.3/2;    % ohm (2 motors)
JA = 1.3e-6*2; % motor inertia
LA = 6.6e-3/2; % rotor inductor (2 motors)
BA = 3e-6*2;   % rotor friction
Kemf = 0.0105; % motor constant
Km = Kemf;
% vehicle
NG = 9.69; % gear
WR = 0.03; % wheel radius
Bw = 0.155; % wheel distance
% 
% model parts used in Simulink
mmotor = 0.193;   % total mass of motor and gear [kg]
mframe = 0.32;    % total mass of frame and base print [kg]
mtopextra = 0.97 - mframe - mmotor; % extra mass on top (charger and battery) [kg]
mpdist =  0.10;   % distance to lit [m]
% disturbance position (Z)
pushDist = 0.1; % relative to motor axle [m]

%% wheel velocity controller (no balance) PI-regulator
% sample (usable) controller values
Kpwv = 15;     % Kp
tiwv = 0.05;   % Tau_i
Kffwv = 0;     % feed forward constant
startAngle = 10;  % tilt in degrees at time zero
twvlp = 0.005;    % velocity noise low pass filter time constant (recommended)
%%
% From previously designed position controller
Td_wv = 0.264; % tau_d
%% Estimate transfer function for base system using LINEARIZE
% Motor volatge to wheel velocity (wv)
load_system(model);
open_system(model);
% define points in model
ios(1) = linio(strcat(model,'/Limit9v'),1,'openinput');
ios(2) = linio(strcat(model, '/wheel_vel_filter'),1,'openoutput');
% attach to model
setlinio(model,ios);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys = linearize(model,ios,op);
% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gwv = minreal(tf(num, den));
%%
figure();
hold on
pzmap(Gwv);
hold on
pzmap(Gwv_B);
title('Pole Zero map of original and balance systems')
%% Bodeplot
h = figure(100)
bode(Gwv)
grid on
title('Transfer function from motor voltage to velocity')
%saveas(h, 'motor to velocity.png');
%% DESIGN BALANCE REGULATOR (PI-LEAD)
ww = logspace(-3,4,1000);
figure(4);
hold on;
bode(-Gwv,ww);
grid on;
title('Transfer function from motor voltage to velocity')
%%
% From the bodeplot we can deduct tau_i as gain cross-over frequency
Ti_b = 0.35;
bode(-Gwv * (1 + tf(1,[Ti_b 0])),ww);

% a lead joint might improve the phase margin
w_cdb = 60;
alpha_b = 0.15;
Td_b = 1/(sqrt(alpha_b)*w_cdb);

% Determine Kp. Select w_c with 60 degree phase margin
bode(-Gwv * (1 + tf(1,[Ti_b 0])) * tf([Td_b 1],[alpha_b*Td_b 1]),ww);
Kp_b = -10^(28/20);

% Control loop
C_i_b = (1 + tf(1,[Ti_b 0]));
C_d_b = tf([Td_b 1],[alpha_b*Td_b 1]);
C_b = Kp_b * C_i_b * C_d_b;

bode(C_b,ww); % Controller open-loop

bode(C_b * Gwv,ww); % Controlled system

G_cl = (Kp_b * C_i_b * Gwv)/(1 + C_b * Gwv); % Closed-loop TF

bode(G_cl,ww);
hold off;


%% DESIGN BALANCE PI-LEAD
% Linearize simulink model and get transfer function
model1 ='regbot_balance_1mg'
% Motor volatge to wheel velocity (wv)
load_system(model1);
open_system(model1);
% Define points in model
ios(1) = linio(strcat(model1,'/vel_ref'),1,'openinput');
ios(2) = linio(strcat(model1, '/gain1'),1,'openoutput');
% Attach to model
setlinio(model1,ios);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys = linearize(model1,ios,op);
% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gwv_B = minreal(tf(num, den));
pzmap(Gwv_B);
%% StepResponse:
figure;
step(Gwv_B);
grid on;
title('Step Response');
xlabel('Time');
ylabel('Output');

%% Nyquist and bode plots
figure(5);
bode(Gwv_B,ww);
grid on
title('Bodeplot from motor vel\_ref to tilt\_angle plus gyro')

figure(6);
nyquist(Gwv_B);
w_cibv = 0.03;
Ti_bv = 1/w_cibv;
C_i_bv = 1 + tf([1],[Ti_bv 0]);

T_lag_bv = 1/390;
beta = 30;
C_lag_bv = tf([T_lag_bv 1],[beta*T_lag_bv 1]);

Ni = 10;
w_cdbv = 71.9;
alpha = 0.6;
Td_bv = 1/(sqrt(alpha)*w_cdbv);
C_d_bv = tf([Td_bv 1],[alpha*Td_bv 1]);
figure;
bode(Gwv_B * C_lag_bv);

w_cpbv = 2;
Kp_bv = 10^(-9.52/20);

figure(5);
hold on;
bode(Kp_bv * C_lag_bv * Gwv_B,ww);
hold off;

figure(10);
nyquist(Kp_bv * C_i_bv * Gwv_B * C_d_bv);
grid on;

% Closed-loop
G_cl_bv = (Kp_bv * C_i_bv * Gwv_B)/(1 + Kp_bv * C_i_bv * Gwv_B);
figure(11);
bode(G_cl_bv);
grid on;
%% StepResponse:

figure;
step(G_cl_bv);
grid on;
title('Step Response');
xlabel('Time');
ylabel('Output');
%%
%REGBOT BALANCE
figure(2);
hold off;
bode(Gwv_B)

figure(3);
hold on;
nyquist(Gwv_B);
grid on;
set(gcf(),'PaperUnits','centimeters','PaperPosition',[0 0 12 10]);

%% DESIGN BALANCE-VELOCITY 
% Linearize simulink model and get transfer function
model2 ='regbot_balance_velocity_1mg'
% Motor volatge to wheel velocity (wv)
load_system(model2);
open_system(model2);
% define points in model
ios(1) = linio(strcat(model2,'/ref'),1,'openinput');
ios(2) = linio(strcat(model2, '/tiltangle'),1,'openoutput');
% attach to model
setlinio(model2,ios);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys = linearize(model2,ios,op);
% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gwv_BV = minreal(tf(num, den));
%pzmap(Gwv_BV);
bode(Gwv_BV)
%% StepResponse:
figure;
step(Gwv_BV);
grid on;
title('Step Response');
xlabel('Time');
ylabel('Output');
%%
figure;
hold on;
grid on;
title('Transfer function from motor voltage to velocity')
bode(Gwv_BV, ww);
%%
alpha=0.2;
w_dbvp=14;
Td_bvp = 1/(sqrt(alpha)*w_dbvp);
C_d = tf([Td_bvp 1],[alpha*Td_bvp 1]);

bode(C_d*Gwv_BV, ww);

Kp_bvp = 10^(4/20);

bode(Kp_bvp * C_d * Gwv_BV, ww);
%% DESIGN BALANCE-VELOCITY-POSITION
% Linearize simulink model and get transfer function
model3 ='regbot_balance_velocity_position_1mg'
% Motor volatge to wheel velocity (wv)
load_system(model3);
open_system(model3);
% define points in model
ios(1) = linio(strcat(model3,'/vel_ref'),1,'openinput');
ios(2) = linio(strcat(model3, '/Integrator5'),1,'openoutput');
% attach to model
setlinio(model3,ios);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys = linearize(model3,ios,op);
% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gwv_BVP = minreal(tf(num, den));
%pzmap(Gwv_BVP);
bode(Gwv_BVP)
%% StepResponse:

figure;
step(Gwv_BVP);
grid on;
title('Step Response');
xlabel('Time');
ylabel('Output');

%%
figure;
hold on;
grid on;
bode(Gwv_BVP, ww);
