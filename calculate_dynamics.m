function [torques] = calculate_dynamics(robot, q, qdot, qddot)
    % CALCULATE_DYNAMICS Computes the required motor torques
    %
    % Inputs:
    %   robot - The 3DOF rigidBodyTree object (from Member 1)
    %   q     - Current joint angles [theta1; theta2; theta3]
    %   qdot  - Current joint velocities [vel1; vel2; vel3]
    %   qddot - Current joint accelerations [accel1; accel2; accel3]
    %
    % Output:
    %   torques - 3x1 vector of required torques for each motor
    
    %% =======================================================
    %  TODO 1: SET THE ENVIRONMENT
    %% =======================================================
    % We need to tell the robot which way gravity is pulling so it 
    % knows how much weight it is fighting against.
    % Assuming the Z-axis points UP, gravity pulls DOWN at 9.81 m/s^2.
    robot.Gravity = [0, 0, -9.81];
    
    %% =======================================================
    %  TODO 2: CALCULATE INVERSE DYNAMICS
    %% =======================================================
    % This function calculates the exact torques required for the 
    % motors to achieve the requested positions (q), velocities (qdot), 
    % and accelerations (qddot), taking into account the masses Member 1 set.
    
    % Ensure inputs are column vectors for the toolbox
    q = q(:);
    qdot = qdot(:);
    qddot = qddot(:);
    
    torques = inverseDynamics(robot, q, qdot, qddot);
    
    %% =======================================================
    %  REPORT REQUIREMENT REMINDER
    %% =======================================================
    % NOTE FOR REPORT: While this one line of code perfectly simulates 
    % the dynamics, Dr. Abdo will want to see the mathematical derivation 
    % in your report. Make sure you write out the Euler-Lagrange equations 
    % (Week 7) on paper to include in your documentation!

end