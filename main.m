% main.m - Master Integration Script
clear; clc; close all;

%% 1. INITIALIZE ROBOT (Waiting on Member 1)
disp('Building Robot...');
robot = build_robot(); 
show(robot); % Show the initial robot
hold on;

%% 2. DEFINE THE ENVIRONMENT (Your Job!)
% The video shows picking up colored blocks. You need to define where 
% these blocks are in the 3D space (X, Y, Z coordinates).
block_start_xyz = [0.2, 0.1, 0.0];  % Example: 20cm forward, 10cm left, on the floor
square_target_xyz = [0.0, 0.3, 0.0]; % Example: 30cm left, on the floor

% (Optional: You can write code here to plot simple colored 3D cubes 
% at these locations so it looks like the video!)

%% 3. KINEMATICS & ANGLES (Waiting on Member 2)
% We need to know what joint angles will reach the block.
% You will use MATLAB's inverseKinematics tool (or Member 2's math) to 
% find the exact [theta1, theta2, theta3] to reach block_start_xyz.
home_angles = [0; 0; 0];
pick_angles = [pi/4; pi/6; -pi/6]; % DUMMY ANGLES: Replace later
place_angles = [pi/2; pi/4; -pi/4]; % DUMMY ANGLES: Replace later

%% 4. TRAJECTORY PLANNING (Waiting on Member 4)
disp('Calculating Trajectory...');
duration = 3; % 3 seconds to move
steps = 50;   % 50 animation frames

% Path 1: Home to the Block
[q_path1, qdot_path1, qddot_path1, t1] = plan_trajectory(home_angles, pick_angles, duration, steps);

%% 5. SIMULATION & DYNAMICS LOOP (Waiting on Member 3)
disp('Running Simulation...');
% We loop through every single waypoint Member 4 generated
for i = 1:steps
    % Get the current angles, velocity, and acceleration for this exact millisecond
    current_q = q_path1(:, i);
    current_qdot = qdot_path1(:, i);
    current_qddot = qddot_path1(:, i);

    % Ask Member 3's code how much torque the motors need right now
    torques = calculate_dynamics(robot, current_q, current_qdot, current_qddot);

    % Update the 3D plot to show the robot moving
    show(robot, current_q, 'PreservePlot', false, 'Frames', 'off');
    title(sprintf('Time: %.2f s | Motor 1 Torque: %.2f Nm', t1(i), torques(1)));
    drawnow;
end
disp('Pick and Place Complete!');