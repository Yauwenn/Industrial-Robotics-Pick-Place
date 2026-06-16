function [torques] = calculate_dynamics(robot, q, qdot, qddot)
% CALCULATE_DYNAMICS Computes the required motor torques for the 3DOF robot.
%
% Inputs:
%   robot - 3DOF rigidBodyTree object from build_robot.m
%   q     - Joint angles [theta1; theta2; theta3] in radians
%   qdot  - Joint velocities [theta1_dot; theta2_dot; theta3_dot] in rad/s
%   qddot - Joint accelerations [theta1_ddot; theta2_ddot; theta3_ddot] in rad/s^2
%
% Output:
%   torques - 3x1 vector of required motor torques [tau1; tau2; tau3] in N.m

%% 1. Set Gravity
% Z-axis is assumed to point upward, so gravity acts downward.

robot.Gravity = [0 0 -9.81];

%% 2. Ensure inputs are column vectors
% Member 1 built the robot using DataFormat = 'column'.

q     = q(:);
qdot  = qdot(:);
qddot = qddot(:);

%% 3. Calculate inverse dynamics torque
% General robot dynamics form:
%
%   tau = M(q)qddot + C(q,qdot)qdot + G(q)
%
% inverseDynamics calculates the required torque at each joint.

% If no external force is provided (e.g., robot is empty-handed), default to zero forces
    if nargin < 5
        external_force = zeros(6, robot.NumBodies); 
    end

% Calculate the exact torque required at each joint
torques = inverseDynamics(robot, q, qdot, qddot);

end
