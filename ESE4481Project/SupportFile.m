% support file for decouple system %
s = tf('s',0);
%~~~~~~~~~~~~~~~~~~~~~~~~controller for decouple~~~~~~~~~~~~~~~~~~~~~~~
A_T = f_At1;        %ss system of Throttle
B_T = f_Bt1;
C_T = eye(2);
C_T_r = [1 0];
D_T = zeros(2,1);
K_T = place(A_T, B_T, [-5 -2]);  %pole placement for Throttle
H_c_T = C_T_r*(s*eye(2) - A_T + B_T * K_T)^(-1) * B_T;
F_T = dcgain(H_c_T)^-1;


A_A = f_AA3;        %ss system of Aileron
B_A = f_BA1;
C_A = eye(3);
C_A_r = [1 0 0];
D_A = zeros(3,1);
K_A = place(A_A, B_A, [-5 -2 -1]);  %pole placement for Aileron
H_c_A = C_A_r*(s*eye(3) - A_A + B_A * K_A)^(-1) * B_A;
F_A = dcgain(H_c_A)^-1;

A_E = F_AE3;        %ss system of Elevator
B_E = f_BE1;
C_E = eye(3);
C_E_r = [1 0 0];
D_E = zeros(3,1);
K_E = place(A_E, B_E, [-5 -2 -1]);  %pole placement for Elevator
H_c_E = C_E_r*(s*eye(3) - A_E + B_E * K_E)^(-1) * B_E;
F_E = dcgain(H_c_E)^-1;

A_R = f_AR2;        %ss system of Rudder
B_R = f_BR1;
C_R = eye(2);
C_R_r = [1 0];
D_R = zeros(2,1);
K_R = place(A_R, B_R, [-5 -2]);  %pole placement for Rudder
H_c_R = C_R_r*(s*eye(2) - A_R + B_R * K_R)^(-1) * B_R;
F_R = dcgain(H_c_R)^-1;

% function F = NonZeroSet_F(A, B, C, K)
% s = tf('s',0);
% state_size = size(A);
% H_c = C*(s*eye(state_size) - A + B * K)^(-1) * B;
% H_c_inv1 = H_c(1)^(-1);
% H_c_inv2 = H_c(2)^(-1);
% H_c_inv = [H_c_inv1;H_c_inv2];
% F = dcgain(H_c_inv); 
% end
