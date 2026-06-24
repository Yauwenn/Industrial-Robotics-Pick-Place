% main.m - Master Integration Script (3-Block Sorting)
clear; clc; close all;

%% =======================================================
%  1. INITIALIZE ROBOT & ENVIRONMENT
%% =======================================================
disp('Building Robot...');
robot = build_robot(); 

% --- ADD TOOL CENTER POINT (TCP) ---
tcp = rigidBody('tcp');
jnt_tcp = rigidBodyJoint('jnt_tcp', 'fixed');
setFixedTransform(jnt_tcp, trvec2tform([0.100, 0, 0])); % 100mm Gripper Length
tcp.Joint = jnt_tcp;
addBody(robot, tcp, 'gripper'); 
% ----------------------------------------

% Setup the 3D figure
figure('Name', 'FYP Automated Sorting Simulation', 'NumberTitle', 'off');
show(robot); 
hold on; grid on;
axis([-0.2 0.8 -0.5 0.5 0 0.6]); 
view(45, 30); 

% --- DEFINE 3 BLOCKS AND 3 TARGETS ---
% The robot's Y-axis controls left/right. 
% Positive Y (+0.30) is the Far Left side of the table.
% Negative Y (-0.30) is the Far Right side of the table.
% We stagger them along the X-axis (0.25m, 0.35m, 0.45m) so they sit in a neat row!

blocks_xyz = [
    0.4,  -0.30, 0.05;  % Green Block (Far Left, Furthest away)
    0.2,  -0.30, 0.05;  % Red Block   (Far Left, Middle)
    0.0,  -0.30, 0.05   % Blue Block  (Far Left, Closest)
];

targets_xyz = [
    0.4, 0.30, 0.05;  % Green Target (Far Right, Furthest away)
    0.2, 0.30, 0.05;  % Red Target   (Far Right, Middle)
    0.0, 0.30, 0.05   % Blue Target  (Far Right, Closest)
];

colors = ['g', 'r', 'b']; % Colors array matching the rows above
block_names = {'Green', 'Red', 'Blue'};

% Arrays to store the visual plots so we can move them
h_blocks = gobjects(3, 1);
h_targets = gobjects(3, 1);

for i = 1:3
    % Draw Target Outlines (MarkerFaceColor = 'none' makes them outlines)
    h_targets(i) = plot3(targets_xyz(i,1), targets_xyz(i,2), targets_xyz(i,3), ...
        's', 'MarkerSize', 40, 'MarkerEdgeColor', colors(i), 'MarkerFaceColor', 'none', 'LineWidth', 2);
    
    % Draw Solid Blocks
    h_blocks(i) = plot3(blocks_xyz(i,1), blocks_xyz(i,2), blocks_xyz(i,3), ...
        's', 'MarkerSize', 20, 'MarkerFaceColor', colors(i), 'MarkerEdgeColor', 'k');
end

%% =======================================================
%  2. MASTER AUTOMATION LOOP
%% =======================================================
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 1]; 
home_angles = [0; -pi/3; pi/2]; % "Elbow Up" starting posture

duration = 2; % 2 seconds per phase
steps = 30;   % 30 frames of animation per phase

% Start the loop! It will run 3 times (once for each block)
for b = 1:3
    disp(['--- Starting sequence for ', block_names{b}, ' Block ---']);
    
    % --- A. INVERSE KINEMATICS FOR CURRENT BLOCK ---
    current_block = blocks_xyz(b, :);
    current_target = targets_xyz(b, :);
    
    [configPick, ~] = ik('tcp', trvec2tform(current_block), weights, home_angles);
    pick_angles = configPick;
    
    lift_block_xyz = current_block + [0, 0, 0.20]; % Lift 20cm up
    [configLiftPick, ~] = ik('tcp', trvec2tform(lift_block_xyz), weights, configPick);
    lift_pick_angles = configLiftPick;
    
    [configPlace, ~] = ik('tcp', trvec2tform(current_target), weights, configLiftPick);
    place_angles = configPlace;

    % --- B. TRAJECTORY PLANNING ---
    [q1, qdot1, qddot1, t1] = plan_trajectory(home_angles, pick_angles, duration, steps);
    [q2, qdot2, qddot2, t2] = plan_trajectory(pick_angles, lift_pick_angles, duration, steps);
    [q3, qdot3, qddot3, t3] = plan_trajectory(lift_pick_angles, place_angles, duration, steps);
    [q4, qdot4, qddot4, t4] = plan_trajectory(place_angles, home_angles, duration, steps);

    q_total     = [q1, q2, q3, q4];
    qdot_total  = [qdot1, qdot2, qdot3, qdot4];
    qddot_total = [qddot1, qddot2, qddot3, qddot4];

    % --- C. SIMULATION & DYNAMICS LOOP ---
    total_frames = size(q_total, 2);
    
    for i = 1:total_frames
        current_q = q_total(:, i);
        current_qdot = qdot_total(:, i);
        current_qddot = qddot_total(:, i);
        
        % Get exact TCP location
        T_current = getTransform(robot, current_q, 'tcp');
        tip_xyz = T_current(1:3, 4);
        
        if i > steps && i <= (3*steps)
            % GRIPPING AND CARRYING THE BLOCK
            status = ['CARRYING ', upper(block_names{b})];
            
            payload = zeros(6, robot.NumBodies);
            payload(6, end) = -1.96; % Weight of block pushing down
            torques = calculate_dynamics(robot, current_q, current_qdot, current_qddot, payload);
            
            % Visually attach the current block to the fingertips
            h_blocks(b).XData = tip_xyz(1);
            h_blocks(b).YData = tip_xyz(2);
            h_blocks(b).ZData = tip_xyz(3) - 0.02;
            
        elseif i > (3*steps)
            % RETURNING HOME
            status = 'RETURNING HOME';
            torques = calculate_dynamics(robot, current_q, current_qdot, current_qddot);
            
            % Snap the block to the target position permanently
            h_blocks(b).XData = current_target(1);
            h_blocks(b).YData = current_target(2);
            h_blocks(b).ZData = current_target(3);
            
        else
            % REACHING FOR BLOCK
            status = ['REACHING FOR ', upper(block_names{b})];
            torques = calculate_dynamics(robot, current_q, current_qdot, current_qddot);
        end
        
        % Update plot
        show(robot, current_q, 'PreservePlot', false, 'Frames', 'off');
        title(sprintf('Phase: %s | Base: %.1fNm | Sldr: %.1fNm | Elbw: %.1fNm', ...
            status, torques(1), torques(2), torques(3)));
        drawnow;
    end
end

disp('All 3 Blocks Sorted Successfully! Simulation Complete.');
