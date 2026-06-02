function [q, qdot, qddot, t] = plan_trajectory(start_angles, target_angles, duration, steps)
    % PLAN_TRAJECTORY Generates a smooth joint-space trajectory
    %
    % Inputs:
    %   start_angles  - 3x1 vector of starting joint angles [rad]
    %   target_angles - 3x1 vector of target joint angles [rad]
    %   duration      - Total time for the movement [seconds]
    %   steps         - Number of waypoints to generate
    %
    % Outputs:
    %   q     - Matrix of joint angles over time
    %   qdot  - Matrix of joint velocities over time
    %   qddot - Matrix of joint accelerations over time
    %   t     - Time vector
    
    %% =======================================================
    %  TODO 1: SETUP TIME VECTOR AND WAYPOINTS
    %% =======================================================
    % Create an array of time points from 0 to 'duration'
    t = linspace(0, duration, steps);
    
    % The start and end times for the waypoints
    tPoints = [0, duration];
    
    % Combine the start and target angles into a single matrix
    waypoints = [start_angles, target_angles];
    
    %% =======================================================
    %  TODO 2: GENERATE THE CUBIC POLYNOMIAL TRAJECTORY
    %% =======================================================
    % This matches Dr. Abdo's Week 9 lecture on Cubic Trajectories!
    % It ensures velocity and acceleration start and end at 0.
    
    [q, qdot, qddot] = cubicpolytraj(waypoints, tPoints, t);
    
    %% =======================================================
    %  REPORT REQUIREMENT REMINDER
    %% =======================================================
    % NOTE FOR REPORT: Dr. Abdo wants to see the math behind this!
    % Make sure you write out the formula: 
    % theta(t) = a0 + a1*t + a2*t^2 + a3*t^3 
    % and show how the boundary conditions (velocity = 0 at start/end) 
    % are solved in your section of the report document.
    "Testing pang...."

end


