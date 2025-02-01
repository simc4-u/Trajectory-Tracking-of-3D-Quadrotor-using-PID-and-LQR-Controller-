function [F, M, trpy, drpy] = pid_controller(qd, t, qn, params)
    % CONTROLLER quadrotor controller
    % The current states are:
    % qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
    % The desired states are:
    % qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
    % Using these current and desired states, you have to compute the desired controls

    % Quadrotor states
    pos = qd{qn}.pos;        
    vel = qd{qn}.vel;        
    euler = qd{qn}.euler;     
    omega = qd{qn}.omega;     

    % Desired states
    pos_des = qd{qn}.pos_des; 
    vel_des = qd{qn}.vel_des; 
    yaw_des = qd{qn}.yaw_des; 

    % parameters
    mass = params.mass;
    g = params.grav;
    I = params.I;

    % Gains for position controller, Kd and Kp
    kp_pos = [7.5; 7.5; 12];
    kd_pos = [6.3; 6.3;10];
    
   % Gains for atitude controller, Kd and Kp
    kp_att = [40;40;20];
    kd_att = [21;21;18];

    % Calculate position errors & velocity errors
    pos_error = pos_des - pos;
    vel_error = vel_des - vel;  

    
    % Desired acceleration
    acc_des = kp_pos .* pos_error + kd_pos .* vel_error;
    F = mass * (g + acc_des(3)); % Thrust
   
    

    % Desired roll and pitch
    phi_des = (1/g) * (acc_des(1) * sin(yaw_des) - acc_des(2) * cos(yaw_des));
    theta_des = (1/g) * (acc_des(1) * cos(yaw_des) + acc_des(2) * sin(yaw_des));

    %phi_des = max(min(phi_des, params.maxangle), -params.maxangle);
    %theta_des = max(min(theta_des, params.maxangle), -params.maxangle);
   

    % orientation errors
    att_error = [phi_des; theta_des; yaw_des] - euler;
    
    pqr_des = [0;0; qd{qn}.yawdot_des];
    
    %angular velocity errors    
    ang_vel_error = pqr_des -omega;  

    % Moments with PD control for attitude
    M = I * (kp_att .* att_error + kd_att .* ang_vel_error);

  
    trpy = [0,0,0,0];  
    drpy = [0,0,0,0];  

    return;
end

