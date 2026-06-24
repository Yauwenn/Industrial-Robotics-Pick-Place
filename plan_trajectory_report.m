% generate_report_graphs.m
% Standalone script to generate report visuals.
clear; clc; close all;

%% 1. INITIALIZE ROBOT & ENVIRONMENT
disp('Building Robot...');
robot = build_robot(); 

% --- ADD TOOL CENTER POINT (TCP) ---
tcp = rigidBody('tcp');
jnt_tcp = rigidBodyJoint('jnt_tcp', 'fixed');
setFixedTransform(jnt_tcp, trvec2tform([0.100, 0, 0])); % 100mm Gripper Length
tcp.Joint = jnt_tcp;
addBody(robot, tcp, 'gripper'); 

% --- DEFINE 3 BLOCKS AND 3 TARGETS ---
blocks_xyz = [
    0.4,  -0.30, 0.05;  % Green Block
    0.2,  -0.30, 0.05;  % Red Block  
    0.0,  -0.30, 0.05   % Blue Block 
    ];

targets_xyz = [
    0.4,   0.30, 0.05;  % Green Target
    0.2,   0.30, 0.05;  % Red Target  
    0.0,   0.30, 0.05   % Blue Target 
    ];

block_names = {'Green', 'Red', 'Blue'};

%% 2. MASTER AUTOMATION LOOP
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 1]; 
home_angles = [0; -pi/3; pi/2]; % "Elbow Up" starting posture
duration = 2; % 2 seconds per phase
steps = 30;   % 30 frames of animation per phase

for b = 1:3
    disp(['--- Starting sequence for ', block_names{b}, ' Block ---']);
    
    % --- A. INVERSE KINEMATICS ---
    current_block = blocks_xyz(b, :);
    current_target = targets_xyz(b, :);
    lift_block_xyz = current_block + [0, 0, 0.20]; % Lift 20cm up
    
    [configPick, ~]     = ik('tcp', trvec2tform(current_block), weights, home_angles);
    [configLiftPick, ~] = ik('tcp', trvec2tform(lift_block_xyz), weights, configPick);
    [configPlace, ~]    = ik('tcp', trvec2tform(current_target), weights, configLiftPick);
    
    % --- B. TRAJECTORY PLANNING ---
    [q1, qdot1, qddot1, ~] = plan_trajectory(home_angles, configPick, duration, steps);
    [q2, qdot2, qddot2, ~] = plan_trajectory(configPick, configLiftPick, duration, steps);
    [q3, qdot3, qddot3, ~] = plan_trajectory(configLiftPick, configPlace, duration, steps);
    [q4, qdot4, qddot4, ~] = plan_trajectory(configPlace, home_angles, duration, steps);
    
    % Concatenate profiles
    q_total     = [q1, q2, q3, q4];
    qdot_total  = [qdot1, qdot2, qdot3, qdot4];
    qddot_total = [qddot1, qddot2, qddot3, qddot4];
    t_total     = linspace(0, duration * 4, size(q_total, 2));
    
    num_joints   = size(q_total, 1);
    plot_colors  = lines(num_joints); 
    
    %% 3. PLOT MOTION PROFILE
    figure('Name', ['Trajectory Profile - ' block_names{b} ' Block'], 'Color', [0.15 0.15 0.15], 'WindowStyle', 'docked');
 
    % --- SUBPLOT 1: POSITION ---
    subplot(3, 1, 1); hold on;
    for i = 1:num_joints
        plot(t_total, q_total(i, :), 'Color', plot_colors(i, :), 'LineWidth', 2, 'DisplayName', ['Joint ' num2str(i)]);
    end
    title('Positions', 'Color', 'w'); ylabel('Angle (rad)', 'Color', 'w'); grid on;
    set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);
    legend('Location', 'best', 'TextColor', 'w', 'Color', [0.2 0.2 0.2]);
 
    % --- SUBPLOT 2: VELOCITY ---
    subplot(3, 1, 2); hold on;
    for i = 1:num_joints
        plot(t_total, qdot_total(i, :), 'Color', plot_colors(i, :), 'LineWidth', 2);
    end
    title('Velocities', 'Color', 'w'); ylabel('Velocity (rad/s)', 'Color', 'w'); grid on;
    set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);
 
    % --- SUBPLOT 3: ACCELERATION ---
    subplot(3, 1, 3); hold on;
    for i = 1:num_joints
        plot(t_total, qddot_total(i, :), 'Color', plot_colors(i, :), 'LineWidth', 2);
    end
    title('Accelerations', 'Color', 'w'); xlabel('Total Time (seconds)', 'Color', 'w'); ylabel('Accel (rad/s^2)', 'Color', 'w'); grid on;
    set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);
 
    sgtitle([block_names{b} ' Block: Trajectory Motion Profile'], 'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');
end