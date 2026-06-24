% generate_report_graphs.m
% Standalone script for Yin Kang to generate report visuals.
clear; clc; close all;

% Dummy angles moving from 0 to 90 degrees over 3 seconds
home_angles = [0; 0; 0];
target_angles = [pi/2; pi/4; -pi/4];
duration = 3; 
steps = 100;

% Call Yin Kang's ORIGINAL cubic trajectory function
[q, qdot, qddot, t] = plan_trajectory(home_angles, target_angles, duration, steps);

% Create a nice figure for the report
figure('Name', 'Trajectory Profiles', 'Color', 'w');

% Plot 1: Position (S-Curve)
subplot(2,1,1);
plot(t, q(1,:), 'b', 'LineWidth', 2);
title('Joint 1 Position (Cubic S-Curve)');
xlabel('Time (s)'); ylabel('Angle (rad)');
grid on;

% Plot 2: Velocity (Parabolic Bell Curve)
subplot(2,1,2);
plot(t, qdot(1,:), 'r', 'LineWidth', 2);
title('Joint 1 Velocity (Smooth Start/Stop)');
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
grid on;

sgtitle('Member 4: Trajectory Planning Results');