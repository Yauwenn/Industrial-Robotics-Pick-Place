function [q, qdot, qddot, t] = plan_trajectory(start_angles, target_angles, duration, steps)
    %% PLAN_TRAJECTORY Generates and plots a smooth joint-space quintic trajectory

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

    %% SETUP TIME VECTORS AND WAYPOINTS
    timePoints = [0, duration]; 
    wayPoints = [start_angles, target_angles];
    t = linspace(0, duration, steps);
   
    %%  USE QUINTIC TRAJECTORY FUNCTION
    [q, qdot, qddot] = quinticpolytraj(wayPoints, timePoints, t);
    
    %% PLOT THE MOTION PROFILE
    figure('Name', 'Robot Arm Motion Profile', 'Color', [0.15 0.15 0.15]);
    
    colors = ['r', 'g', 'b'];
    
    % --- SUBPLOT 1: POSITION ---
    subplot(3, 1, 1);
    hold on;
    for i = 1:size(q, 1)
        plot(t, q(i, :), 'Color', colors(i), 'LineWidth', 2, 'DisplayName', ['Joint ' num2str(i)]);
    end
    title('Joint Positions (Trajectory Path)', 'Color', 'w');
    ylabel('Angle (rad)', 'Color', 'w');
    grid on;
    set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);
    legend('Location', 'best', 'TextColor', 'w', 'Color', [0.2 0.2 0.2]);
    
    % --- SUBPLOT 2: VELOCITY ---
    subplot(3, 1, 2);
    hold on;
    for i = 1:size(qdot, 1)
        plot(t, qdot(i, :), 'Color', colors(i), 'LineWidth', 2);
    end
    title('Joint Velocities', 'Color', 'w');
    ylabel('Velocity (rad/s)', 'Color', 'w');
    grid on;
    set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);
    
    % --- SUBPLOT 3: ACCELERATION ---
    subplot(3, 1, 3);
    hold on;
    for i = 1:size(qddot, 1)
        plot(t, qddot(i, :), 'Color', colors(i), 'LineWidth', 2);
    end
    title('Joint Accelerations (Starts and Ends at 0)', 'Color', 'w');
    xlabel('Time (seconds)', 'Color', 'w');
    ylabel('Accel (rad/s^2)', 'Color', 'w');
    grid on;
    set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);
    
    % Main overarching title styled for dark theme
    sgtitle('Quintic Polynomial Trajectory Profile', 'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');
end

%% Test (Replace plan_trajectory with other name to prevent error)
% start_pos = [0; 0; 0];       % Starting angles for Joint 1, 2, 3
% target_pos = [1.5; -0.8; 1.2]; % Target angles
% total_time = 3.0;             % Move takes 3 seconds
% num_points = 100;             % Resolution
% [q, qdot, qddot, t] = plan_trajectory(start_pos, target_pos, total_time, num_points);