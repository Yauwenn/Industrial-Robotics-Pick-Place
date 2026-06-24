function [q_total, qdot_total, qddot_total, t_total] = plan_trajectory(angles, duration, steps, stopAtWaypoint)
    %% PLAN_TRAJECTORY Generates a multi-segment joint-space quintic trajectory
    %
    % Inputs:
    %   angles         - NxM matrix of joint waypoints (e.g., [home, pick, lift, place, home])
    %                    N = number of joints, M = number of waypoints
    %   duration       - Time allocated *per segment* [seconds]
    %   steps          - Number of waypoints to generate *per segment*
    %   stopAtWaypoint - (optional) 1xM logical vector. true  = robot comes to a
    %                    full stop (zero velocity) at that waypoint.
    %                    false = robot passes through with continuous,
    %                    non-zero velocity (no stop).
    %                    Default: true for all waypoints (original behavior).
    %                    The first and last waypoints are ALWAYS forced to
    %                    true, since the robot must start and end at rest.
    %
    %                    Example: for angles = [home, pick, lift, place, home],
    %                    to stop at pick/place (grasp/release) but glide
    %                    through "lift":
    %                       stopAtWaypoint = [true, true, false, true, true];
    %
    % Outputs:
    %   q_total     - Nx(steps*segments) concatenated matrix of joint angles
    %   qdot_total  - Nx(steps*segments) concatenated matrix of joint velocities
    %   qddot_total - Nx(steps*segments) concatenated matrix of joint accelerations
    %   t_total     - 1x(steps*segments) continuous time vector

    % Determine the number of target waypoints, segments, and joints
    num_waypoints = size(angles, 2);
    num_segments  = num_waypoints - 1;
    num_joints    = size(angles, 1);

    % Default: full stop at every waypoint (matches original behavior)
    if nargin < 4 || isempty(stopAtWaypoint)
        stopAtWaypoint = true(1, num_waypoints);
    end
    if numel(stopAtWaypoint) ~= num_waypoints
        error('stopAtWaypoint must have one entry per waypoint (%d entries expected).', num_waypoints);
    end
    % The very first and last waypoints must always be full stops
    stopAtWaypoint(1)   = true;
    stopAtWaypoint(end) = true;

    % Pre-compute the velocity the robot should have AT each waypoint.
    % Stays zero for "stop" waypoints; gets a heuristic non-zero value for "pass-through" waypoints.
    viaVelocities = zeros(num_joints, num_waypoints);
    for i = 2:num_waypoints-1
        if ~stopAtWaypoint(i)
            viaVelocities(:, i) = (angles(:, i+1) - angles(:, i-1)) / (2*duration);
        end
    end

    % Preallocate global accumulators (avoids growing arrays in the loop)
    total_elements = steps * num_segments;
    q_total     = zeros(num_joints, total_elements);
    qdot_total  = zeros(num_joints, total_elements);
    qddot_total = zeros(num_joints, total_elements);
    t_total     = zeros(1, total_elements);

    %%  LOOP THROUGH AND FILL EACH SEGMENT
    for s = 1:num_segments
        start_angles  = angles(:, s);
        target_angles = angles(:, s+1);

        % Calculate local time array for this specific segment
        timePoints = [0, duration];
        wayPoints  = [start_angles, target_angles];
        t_segment  = linspace(0, duration, steps);

        % Velocity boundary conditions for this segment
        v_start = viaVelocities(:, s);
        v_end   = viaVelocities(:, s+1);

        % Run built-in quintic profile solver with explicit boundary conditions
        [q_seg, qdot_seg, qddot_seg] = quinticpolytraj(wayPoints, timePoints, t_segment, ...
            'VelocityBoundaryCondition', [v_start, v_end], ...
            'AccelerationBoundaryCondition', zeros(num_joints, 2));

        % Write this segment into its slice of the global arrays.
        % Each segment's *local* time starts at 0, so need to offset by
        % (s-1)*duration to place it correctly on the global timeline
        idx = (s-1)*steps + 1 : s*steps;
        q_total(:, idx)     = q_seg;
        qdot_total(:, idx)  = qdot_seg;
        qddot_total(:, idx) = qddot_seg;
        t_total(idx)        = (s-1)*duration + t_segment;
    end

    %% PLOT THE PROFILE
    figure('Name', 'Robot Motion Profile', 'Color', [0.15 0.15 0.15], 'WindowState', 'minimized');
    colors = lines(num_joints);

    % --- SUBPLOT 1: POSITION ---
    subplot(3, 1, 1); hold on;
    for i = 1:size(q_total, 1)
        plot(t_total, q_total(i, :), 'Color', colors(i, :), 'LineWidth', 2, 'DisplayName', ['Joint ' num2str(i)]);
    end
    title('Positions', 'Color', 'w'); ylabel('Angle (rad)', 'Color', 'w'); grid on;
    set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);
    legend('Location', 'best', 'TextColor', 'w', 'Color', [0.2 0.2 0.2]);

    % --- SUBPLOT 2: VELOCITY ---
    subplot(3, 1, 2); hold on;
    for i = 1:size(qdot_total, 1)
        plot(t_total, qdot_total(i, :), 'Color', colors(i, :), 'LineWidth', 2);
    end
    title('Velocities', 'Color', 'w'); ylabel('Velocity (rad/s)', 'Color', 'w'); grid on;
    set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);

    % --- SUBPLOT 3: ACCELERATION ---
    subplot(3, 1, 3); hold on;
    for i = 1:size(qddot_total, 1)
        plot(t_total, qddot_total(i, :), 'Color', colors(i, :), 'LineWidth', 2);
    end
    title('Accelerations', 'Color', 'w'); xlabel('Total Time (seconds)', 'Color', 'w'); ylabel('Accel (rad/s^2)', 'Color', 'w'); grid on;
    set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);

    sgtitle('Trajectory Motion Profile', 'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');
    fig.WindowState = 'minimized'; 
    fig.Visible = 'on';
end
