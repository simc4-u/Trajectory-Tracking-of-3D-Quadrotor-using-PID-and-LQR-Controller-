function [F, M, trpy, drpy] = mpc_controller(qd, t, qn, params, trajhandle)
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

%% Parameter Initialization

dt = 0.05; % Time increment
N = 10; % Number of desired states to compute
desired_states = cell(1, N); % Preallocate for storing results    
if ~isempty(t)
    for i = 1:N
        t_current = t + (i - 1) * dt; % Calculate the current time step
        desired_states{i} = trajhandle(t_current, qn); % Store the result in the cell array
    end
end
% disp(t);
% disp(qn);
disp(desired_states{1}.vel);
disp(desired_states{2}.vel);
disp(desired_states{3}.vel);
M = [0;0;0];

m = params.mass;
g = params.grav;
I = params.I;

%Current state
r_val = qd{qn}.pos;
E = qd{qn}.euler;
r_dot = qd{qn}.vel;
w = qd{qn}.omega;

X_current = [r_val; E; r_dot; w];

%A_0
p_val = w(1);
q_val = w(2);
r_val = w(3);

phi_val = E(1);
theta_val = E(2);
psi_val = E(3);

A = zeros(12);
% disp(A);
A(1,7) = 1;
A(2,8) = 1;
A(3,9) = 1;
A(4,5) = -p_val*sin(theta_val)+r_val*cos(theta_val);
A(4,10) = cos(theta_val);
A(4,12) = sin(theta_val);
A(5,4) =  (1 / (cos(theta_val)^2))*(p_val*sin(theta_val)-r_val*cos(theta_val));
A(5,5) = tan(phi_val)*(p_val*cos(theta_val)+r_val*sin(theta_val));
A(5,10) = sin(theta_val)*tan(phi_val);
A(5,11) = 1;
A(5,12) = -cos(theta_val)*tan(phi_val);
A(6,4) = tan(phi_val)*(1 / (cos(phi_val)))*(r_val*cos(theta_val)-p_val*sin(theta_val));
A(6,5) = (1 / (cos(phi_val)))*(-p_val*cos(theta_val)-r_val*sin(theta_val));
A(6,10) = -sin(theta_val)/cos(phi_val);
A(6,12) = cos(theta_val)/cos(phi_val);
A(7,4) = g*sin(psi_val);
A(7,5) = g*cos(psi_val);
A(8,4) = -g*cos(psi_val);
A(8,5) = g*sin(psi_val);

% A_0 = zeros(12);
A_0 = A;

% Q = 10 * eye(12);
% R = 0.001 * eye(4);

Q = diag([100, 100, 100, 100, 100, 100, 100, 100, 100, 0.01, 0.01, 0.01]);
R = diag([0.001, 0.01, 0.01, 0.01]);

Q_cells = repmat({Q}, 1, N);
Q_tilda = blkdiag(Q_cells{:});
R_cells = repmat({R}, 1, N);
R_tilda = blkdiag(R_cells{:});
Q_bar = blkdiag(Q_tilda, R_tilda);
A_all = cell(1, N-1);
qs = cell(2*(N),1);

for i=1:N
    r_des = desired_states{i}.pos;
    r_dot_des = desired_states{i}.vel;
    E3_des = desired_states{i}.yaw;
    
    r_acc_des = desired_states{i}.acc;
    E1_des = (1/g)*(r_acc_des(1)*sin(E3_des)-r_acc_des(2)*cos(E3_des));
    E2_des = (1/g)*(r_acc_des(1)*cos(E3_des)+r_acc_des(2)*sin(E3_des));
    E_des = [E1_des; E2_des; E3_des];
    
    w3_des = desired_states{i}.yawdot;
    w_des = [0; 0; w3_des];
    
    % epsilon = 1e-6;
    
    X_des = [r_des; E_des; r_dot_des; w_des];
    u_des = [m*g; M];
    
    % X_des = epsilon * ones(12, 1);
    % u_des = epsilon * ones(4, 1);
    
    %disp(X)
    % disp(X_des)
    % disp(u_des)
    
    p_val = w_des(1);
    q_val = w_des(2);
    r_val = w_des(3);
    
    phi_val = E_des(1);
    theta_val = E_des(2);
    psi_val = E_des(3);
    

    %Calculating A_i
    A = zeros(12);
    % disp(A);
    A(1,7) = 1;
    A(2,8) = 1;
    A(3,9) = 1;
    A(4,5) = -p_val*sin(theta_val)+r_val*cos(theta_val);
    A(4,10) = cos(theta_val);
    A(4,12) = sin(theta_val);
    A(5,4) =  (sec(phi_val)^2)*(p_val*sin(theta_val)-r_val*cos(theta_val));
    A(5,5) = tan(phi_val)*(p_val*cos(theta_val)+r_val*sin(theta_val));
    A(5,10) = sin(theta_val)*tan(phi_val);
    A(5,11) = 1;
    A(5,12) = -cos(theta_val)*tan(phi_val);
    A(6,4) = tan(phi_val)*sec(phi_val)*(r_val*cos(theta_val)-p_val*sin(theta_val));
    A(6,5) = sec(phi_val)*(-p_val*cos(theta_val)-r_val*sin(theta_val));
    A(6,10) = -sin(theta_val)/cos(phi_val);
    A(6,12) = cos(theta_val)/cos(phi_val);
    A(7,4) = g*sin(psi_val);
    A(7,5) = g*cos(psi_val);
    A(8,4) = -g*cos(psi_val);
    A(8,5) = g*sin(psi_val);
    
    A_all{i} = A;
    
    q_i = -2*(X_des.')*Q;
    qs{i} = q_i';
end

for i=N+1:2*(N)
    r_i = -2*(u_des.')*R;
    qs{i} = r_i'; 
end
q = cell2mat(qs);
% disp(q);
% disp(size(q));

As = cell(N, N);
for i=1:N
    for j=1:N
        if i==j
            As{i,i} = eye(12);
        else
            As{i,j} = zeros(12);
        end
    end
end

for i=1:N-1
    As{i+1,i} = -A_all{i};
end

As_mat = cell2mat(As);% final A block matrix
% disp(As_mat);

B = zeros(12,4);
B(9,1) = 1/m;
B(10,2) = 1/I(1,1);
B(11,3) = 1/I(2,2);
B(12,4) = 1/I(3,3);
Bs = cell(N, N);
for i=1:N
    for j=1:N
        if i==j
            Bs{i,i} = -B;
        else
            Bs{i,j} = zeros(12,4);
        end
    end
end
% disp(Bs);
Bs_mat = cell2mat(Bs); % converting the block structure to a matrix
% disp(Bs_mat);
% disp(size(Bs_mat));

A_bar = [As_mat, Bs_mat];
% disp(size(A_bar));
% disp(A_bar);

Bs_bar = cell(N, 1);

Bs_bar{1} = A_0*X_current;
for k=2:N
    Bs_bar{k} = zeros(12,1);
end
B_bar = cell2mat(Bs_bar);
% disp(B_bar);

% H Matrix

Hs = cell(4*N, 2*N);
l=1;
for i=1:N
    Hs{l,i}=-eye(12);
    l=l+1;
    Hs{l,i}=eye(12);
    l=l+1;
end
for row = 1:2*N
    for col = 1:N
        if isempty(Hs{row, col})  
            Hs{row, col} = zeros(12);  
        end
    end
end

for i=N+1:2*N
    Hs{l,i}=-eye(4);
    l=l+1;
    Hs{l,i}=eye(4);
    l=l+1;
end
for row = 1:2*N
    for col = N+1:2*N
        if isempty(Hs{row, col})  
            Hs{row, col} = zeros(12,4);  
        end
    end
end
for row = 2*N+1:4*N
    for col = 1:N
        if isempty(Hs{row, col})  
            Hs{row, col} = zeros(4,12);  
        end
    end
end
for row = 2*N+1:4*N
    for col = N+1:2*N
        if isempty(Hs{row, col})  
            Hs{row, col} = zeros(4);  
        end
    end
end
% disp(Hs);
H_bar = cell2mat(Hs);
% disp(H_bar);
% disp(size(H_bar));

% x_lim_min = -1e100*ones(12,1);
% x_lim_max = 1e100*ones(12,1);

x_lim_min = [-10; -10; 0; -pi/6; -pi/6; -1e4; -5; -5; -5; -1e4; -1e4; -1e4];
x_lim_max = [10; 10; 10; pi/6; pi/6; 1e4; 5; 5; 5; 1e4; 1e4; 1e4];

% u_lim_min = [-1e100;-1e100;-1e100;-1e100];
% u_lim_max = [1e100;1e100;1e100;1e100];

u_lim_min = [0.05*m*g; 1e4; -1e4; -1e4]; 
u_lim_max = [ 2.5*m*g; 1e4; 1e4; 1e4];  
h_limit_s = cell(4*N, 1);
for i=1:2:2*N
    h_limit_s{i}=x_lim_min;
    h_limit_s{i+1}=x_lim_max;
end
for i=2*N+1:2:4*N
    h_limit_s{i}=u_lim_min;
    h_limit_s{i+1}=u_lim_max;
end
h_limit = cell2mat(h_limit_s);
% disp(h_limit);
% disp(size(h_limit));
% disp(size(Q_bar));
% disp(size(q));
 %X_obtained = quadprog(Q_bar, q, H_bar, h_limit, A_bar, B_bar);
 X_obtained = quadprog(Q_bar, q, H_bar, h_limit, A_bar, B_bar);
% X_obtained = quadprog(Q_bar, q, [], [], A_bar, B_bar);
disp("X_obtained")
disp(size(X_obtained));

F = X_obtained(12*N+1);
M = [X_obtained(12*N+2);X_obtained(12*N+3);X_obtained(12*N+4)];
disp(F);
disp(M);
%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0,       0,         0];

end