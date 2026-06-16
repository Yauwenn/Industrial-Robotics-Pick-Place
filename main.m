% main.m - Master Integration Script
clear; clc; close all;

%% =======================================================
%  1. INITIALIZE ROBOT & ENVIRONMENT
%% =======================================================
disp('Building Robot...');
robot = build_robot(); 

% --- NEW: ADD TOOL CENTER POINT (TCP) ---
% We dynamically add an invisible point at the exact TIPS of the gripper
% so the IK solver knows to stop 100mm forward of the wrist!
tcp = rigidBody('tcp');
jnt_tcp = rigidBodyJoint('jnt_tcp', 'fixed');
setFixedTransform(jnt_tcp, trvec2tform([0.100, 0, 0])); % 100mm Gripper Length
tcp.Joint = jnt_tcp;
addBody(robot, tcp, 'gripper'); 
% ----------------------------------------

% Setup the 3D figure
figure('Name', 'FYP Pick and Place Simulation', 'NumberTitle', 'off');
show(robot); 
hold on; grid on;
axis([-0.2 0.8 -0.5 0.5 0 0.6]); 
view(45, 30); 

% Block coordinates
block_xyz = [0.35, -0.20, 0.05];  
target_xyz = [0.35,  0.30, 0.05]; 

% Draw the blocks AND SAVE THEM to variables so we can move them later!
h_target = plot3(target_xyz(1), target_xyz(2), target_xyz(3), 'gs', 'MarkerSize', 30, 'MarkerFaceColor', 'g');
h_block = plot3(block_xyz(1), block_xyz(2), block_xyz(3), 'rs', 'MarkerSize', 20, 'MarkerFaceColor', 'r');

%% =======================================================
%  2. INVERSE KINEMATICS (Calculating Waypoints)
%% =======================================================
disp('Calculating Inverse Kinematics...');
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 1]; 

% --- NEW: FORCE "ELBOW UP" POSTURE ---
% Base at 0 (straight)
% Shoulder at -pi/3 (-60 degrees, aiming high up in the air)
% Elbow at pi/2 (+90 degrees, aiming straight back down at the floor)
home_guess = [0; -pi/3; pi/2];

% Target the 'tcp' (fingertips) instead of the 'gripper' (wrist)
[configPick, ~] = ik('tcp', trvec2tform(block_xyz), weights, home_guess);
pick_angles = configPick;

% Lift up 20cm
lift_block_xyz = block_xyz + [0, 0, 0.20]; 
[configLiftPick, ~] = ik('tcp', trvec2tform(lift_block_xyz), weights, configPick);
lift_pick_angles = configLiftPick;

% Target Green Square
[configPlace, ~] = ik('tcp', trvec2tform(target_xyz), weights, configLiftPick);
place_angles = configPlace;

%% =======================================================
%  3. TRAJECTORY PLANNING (4 Phases of Movement)
%% =======================================================
disp('Calculating Trajectories...');
duration = 2; % 2 seconds per phase
steps = 40;   % 40 frames of animation per phase

% Phase 1: Home to Red Block
[q1, qdot1, qddot1, t1] = plan_trajectory(home_guess, pick_angles, duration, steps);
% Phase 2: Lift Block UP
[q2, qdot2, qddot2, t2] = plan_trajectory(pick_angles, lift_pick_angles, duration, steps);
% Phase 3: Move Block ACROSS and DOWN to Green Square
[q3, qdot3, qddot3, t3] = plan_trajectory(lift_pick_angles, place_angles, duration, steps);
% Phase 4: Return Home
[q4, qdot4, qddot4, t4] = plan_trajectory(place_angles, home_guess, duration, steps);

% Combine all phases
q_total     = [q1, q2, q3, q4];
qdot_total  = [qdot1, qdot2, qdot3, qdot4];
qddot_total = [qddot1, qddot2, qddot3, qddot4];

%% =======================================================
%  4. SIMULATION & DYNAMICS LOOP
%% =======================================================
disp('Running Simulation...');
total_frames = size(q_total, 2);

for i = 1:total_frames
    current_q = q_total(:, i);
    current_qdot = qdot_total(:, i);
    current_qddot = qddot_total(:, i);
    
    % Get the exact XYZ location of the fingertips at this current frame
    T_current = getTransform(robot, current_q, 'tcp');
    tip_xyz = T_current(1:3, 4);
    
    % Check which Phase we are in
    if i > steps && i <= (3*steps)
        % PHASE 2 & 3: GRIPPING AND CARRYING THE BLOCK
        status = 'CARRYING BLOCK';
        
        % 1. Apply Payload Force for Dynamics (Member 3's code)
        payload = zeros(6, robot.NumBodies);
        payload(6, end) = -1.96; % Weight of block pushing down
        torques = calculate_dynamics(robot, current_q, current_qdot, current_qddot, payload);
        
        % 2. Visually attach the red block to the fingertips!
        h_block.XData = tip_xyz(1);
        h_block.YData = tip_xyz(2);
        h_block.ZData = tip_xyz(3) - 0.02; % Hang slightly below the fingertips
        
    elseif i > (3*steps)
        % PHASE 4: RETURNING HOME
        status = 'RETURNING HOME';
        torques = calculate_dynamics(robot, current_q, current_qdot, current_qddot);
        
        % Snap the block to the final green target position
        h_block.XData = target_xyz(1);
        h_block.YData = target_xyz(2);
        h_block.ZData = target_xyz(3);
        
    else
        % PHASE 1: REACHING FOR BLOCK
        status = 'REACHING';
        torques = calculate_dynamics(robot, current_q, current_qdot, current_qddot);
    end
    
    % Update the 3D plot
    show(robot, current_q, 'PreservePlot', false, 'Frames', 'off');
    title(sprintf('Phase: %s | Sldr Torque: %.2f Nm | Elbw Torque: %.2f Nm', status, torques(2), torques(3)));
    drawnow;
end

disp('Simulation Complete!');