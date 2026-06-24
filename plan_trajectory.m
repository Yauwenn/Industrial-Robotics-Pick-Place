function [q, qdot, qddot, t] = plan_trajectory(start_angles, target_angles, duration, steps)
    % PLAN_TRAJECTORY Generates a smooth joint-space cubic trajectory
    %
    % Inputs:
    %   start_angles  - 3x1 vector of starting joint angles [rad]
    %   target_angles - 3x1 vector of target joint angles [rad]
    %   duration      - Total time for the movement [seconds]
    %   steps         - Number of waypoints to generate
    %
    % Outputs:
    %   q     - 3xsteps matrix of joint angles over time
    %   qdot  - 3xsteps matrix of joint velocities over time
    %   qddot - 3xsteps matrix of joint accelerations over time
    %   t     - 1xsteps time vector
    
    %% =======================================================
    %  SETUP TIME VECTOR
    %% =======================================================
    t = linspace(0, duration, steps);
    
    % Initialize output matrices to store values for each joint over time
    num_joints = length(start_angles);
    q = zeros(num_joints, steps);
    qdot = zeros(num_joints, steps);
    qddot = zeros(num_joints, steps);
    
    %% =======================================================
    %  ANALYTICAL CUBIC POLYNOMIAL TRAJECTORY GENERATION
    %% =======================================================
    % Formula according to Dr. Abdo's Lecture:
    %   theta(t) = a0 + a1*t + a2*t^2 + a3*t^3
    %
    % Boundary conditions for point-to-point motion:
    %   theta(0)        = theta_i       (start_angles)
    %   theta(duration) = theta_f       (target_angles)
    %   theta_dot(0)    = 0             (starts from rest)
    %   theta_dot(dur)  = 0             (comes to a stop)
    %
    % Solving these equations yields the following analytical coefficients:
    %   a0 = theta_i
    %   a1 = 0
    %   a2 = 3 * (theta_f - theta_i) / (duration^2)
    %   a3 = -2 * (theta_f - theta_i) / (duration^3)
    
    tf = duration; % Final time abbreviation
    
    for i = 1:num_joints
        theta_i = start_angles(i);
        theta_f = target_angles(i);
        
        % Calculate coefficients for the current joint
        a0 = theta_i;
        a1 = 0;
        a2 = (3 * (theta_f - theta_i)) / (tf^2);
        a3 = (-2 * (theta_f - theta_i)) / (tf^3);
        
        % Compute position, velocity, and acceleration profiles across time vector t
        q(i, :)     = a0 + a1.*t + a2.*t.^2 + a3.*t.^3;
        qdot(i, :)  = a1 + 2*a2.*t + 3*a3.*t.^2;
        qddot(i, :) = 2*a2 + 6*a3.*t;
    end
end
