% support file for decouple system %
A_T = f_At1;        %ss system of Throttle
B_T = f_Bt1;
C_T = eye(2);
D_T = zeros(2,1);
K_T = place(A_T, B_T, [-5 -2])  %pole placement for Throttle

A_A = f_AA3;        %ss system of Aileron
B_A = f_BA1;
C_A = eye(3);
D_A = zeros(3,1);
K_A = place(A_A, B_A, [-5 -2 -1])  %pole placement for Aileron

A_E = F_AE3;        %ss system of Elevator
B_E = f_BE1;
C_E = eye(3);
D_E = zeros(3,1);
K_E = place(A_E, B_E, [-5 -2 -1])  %pole placement for Elevator

A_R = f_AR2;        %ss system of Rudder
B_R = f_BR1;
C_R = eye(2);
D_R = zeros(2,1);
K_R = place(A_R, B_R, [-5 -2])  %pole placement for Rudder