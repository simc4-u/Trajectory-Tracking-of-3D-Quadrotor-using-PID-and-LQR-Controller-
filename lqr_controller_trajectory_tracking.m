%LQR
function [F, M, trpy, drpy] = lqr_controller(qd, t, qn, params, trajhandle)
% Implementation of LQR for finite horizon
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================
persistent gd;
persistent icnt;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;

if ~isempty(t)
desired_state = trajhandle(t, qn);
end

M = [0;0;0];

% Parameter Initialisation
m = params.mass;
g = params.grav;
I = params.I;

% Current state
pos = qd{qn}.pos;
euler = qd{qn}.euler;
vel = qd{qn}.vel;
omega= qd{qn}.omega;

% Current State Vector:
X_current = [pos; euler; vel; omega];

% Desired states
pos_des = desired_state.pos; %position
vel_des = desired_state.vel; %velocity
euler_des(3) = desired_state.yaw; %yaw
yaw_des = desired_state.yaw;

acc_des = desired_state.acc; %desired accelaration
euler_des(1) = (1/g)*(acc_des(1)*sin(yaw_des)-acc_des(2)*cos(yaw_des)); %desired roll
euler_des(2)= (1/g)*(acc_des(1)*cos(yaw_des)+acc_des(2)*sin(yaw_des)); %desired pitch


%euler_des(1)= max(min(phi_des, params.maxangle), -params.maxangle); 
%euler_des(2) = max(min(theta_des, params.maxangle), -params.maxangle);

euler_des = [euler_des(1); euler_des(2); euler_des(3)];

w3_des = desired_state.yawdot;
w_des = [0; 0; w3_des];

% Desired X
X_des = [pos_des; euler_des; vel_des;  w_des];
u_des = [m*g; M];
%disp("X_Des");
%disp(X_des);

p =0;
r = desired_state.yawdot;
theta = euler_des(2);
phi  = euler_des(1);
psi = yaw_des;

% On linearisation A matrix
A = zeros(12, 12);
A(4, 5) = -p * sin(theta) + r * cos(theta);
A(4,10) = cos(theta);
A(4,12) = sin(theta);
A(5, 4) = sec(phi)^2 * (p * sin(theta) - r * cos(theta));
A(5,5) = tan(phi) * (p * cos(theta) + r * sin(theta));
A(5,10) = sin(theta) * tan(phi);
A(5,11) = 1;
A(5, 12) = -cos(theta) * tan(phi);
A(6, 4) = tan(phi)*sec(phi) * (r * cos(theta) - p * sin(theta));
A(6,5) = sec(phi) * (-p * cos(theta) - r * sin(theta));
A(6, 10)= -sin(theta) / cos(phi);
A(6, 12) = cos(theta) / cos(phi);
A(7, 4) = g * sin(psi);
A(7, 5)= g * cos(psi);
A(8, 4) = -g * cos(psi);
A(8, 5)= g * sin(psi);
A(1,7)= 1;
A(2,8) =1;
A(3, 9) =1;

% Display A Matrix
disp("A Matrix:")
disp(A)
I_xx = I (1,1);
I_yy = I(2,2);
I_zz = I(3,3);


% On linearisation B Matrix
B = [
    0,      0,      0,      0;   
    0,      0,      0,      0;   
    0,      0,      0,      0;   
    0,      0,      0,      0;   
    0,      0,      0,      0;   
    0,      0,      0,      0;
    0,      0,      0,      0;   
    0,      0,      0,      0;
    1/m,    0,      0,      0;   
    0,   1/I_xx,    0,      0;  
    0,      0,   1/I_yy,    0; 
    0,      0,      0,   1/I_zz; 
   
];

% Display B Matrix
disp("B Matrix:")
disp(B);


%final Q matrix
Q = zeros(12);
Q(1,1) = 30;
Q(2,2) = 30;
Q(3,3) = 10; 
Q(4,4) = 0.1;  
Q(5,5) = 0.1;
Q(6,6) = 10;   
Q(7,7) = 20;  
Q(8,8) = 20; 
Q(9,9) = 1;  
Q(10,10) = 0.1; 
Q(11,11) = 0.1; 
Q(12,12) = 0.1; 


% Display the Q matrix - final
% disp('Q matrix:');
% disp(Q);
R = zeros*(4);
R(1,1) = 0.1;
%R(1,1) = 0.05;
R(2,2) = 1;
R(3,3) = 1;
R(4,4) = 1;



%LQR CONTROLLER

[K , S] = lqr(A, B, Q, R);
disp("K_Matrix");
disp(K);

% Output of LQR Controller
u = K*(X_des- X_current) + u_des;

F = u(1); % thrust
M = [u(2); u(3); u(4)]; % Moments

roll = euler_des(1);
pitch = euler_des(2);
yaw = yaw_des;

%Output trpy and drpy 
trpy = [0, 0, 0, 0];
drpy = [0, 0,       0,         0];

end
