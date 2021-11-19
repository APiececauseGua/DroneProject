%% Non-zero Setpoint Control
% Dr. Sankalp Bhan

clear; clc; close all;

%% Example of non-zero setpoint control with feedforward
% consider the throttle subsystem. We'd like to regulate this system to a
% non-zero setpoint. For example, suppose we want to regulate the system to 1.5 meters
z0 = 1.5;

% Suppose we want the feedback gains to cause the UAV to 
% track the 2nd order models specified by 
zeta_vec =[.4 .707 2]; % damping of 1
w = 3*2*pi; % natural frequency of 3 hz

% Final time for step commands
Tf = 1.5;


for i = 1 : length(zeta_vec)
%%
zeta = zeta_vec(i);


%% form a double integrator system
A = [0 1 ; ...
    0 0 ];
% the control, acceleration influences the 2nd state
B = [0 ; ...
    1];
% output the first state
C = [1 0]; 

%% pole-placement
Kz = w^2;
Kw = 2*zeta*w;
K = [Kz Kw];

%% Form the closed-loop transfer function
s = tf('s');
H_c= C*(s*eye(2)-A+B*K)^-1*B

%% Invert it
H_c_inv = H_c^-1

%% Take the DC gain
F = dcgain(H_c_inv);



%% form a command
t=linspace(0,Tf,1000);
command = t;
command(mod(t,Tf)<Tf/2)= z0;
command(mod(t,Tf)>=Tf/2) = 0;
zcmd = command;


%% Simulate the control law
% the system has the control law
% u = -Kx + F*z0

input = F*zcmd;  % F*z0
ic = [0;0];
 
zout = lsim(H_c,input,t, ic);

%% make plot
figure
plot(t,zout,'ro',t,zcmd,'b-');

%% Plot settings
legend({'$Z$','$Z_{cmd}$'},'Interpreter','Latex','Location','EastOutside');
plot_tit = ['Model: $\omega_n$ =' num2str(w,3) ', $\zeta$=' num2str(zeta,3)];
title(plot_tit,'Interpreter','latex');
xlabel('$t$ (s)','Interpreter','latex');
ylabel('$z$ (m)', 'Interpreter','Latex');
ylim([-.5 2]);
grid minor
end


