%% TEST_PLAN_TRAJECTORY
% Test script for plan_trajectory.

clear; clc; close all;

%% Test 1: Default behavior - full stop at every waypoint
fprintf('--- Test 1: Default stop at every waypoint ---\n');

% 2 joints, 5 waypoints: home -> pick -> lift -> place -> home
angles = [0,    0.5,  0.8,  0.3,  0;    % Joint 1
          0,    0.3,  0.6,  0.2,  0];   % Joint 2

duration = 2;     % seconds per segment
steps    = 50;    % points per segment

[q1, qd1, qdd1, t1] = plan_trajectory(angles, duration, steps);

fprintf('q_total size: %s (expected [%d %d])\n', ...
    mat2str(size(q1)), size(angles,1), steps*(size(angles,2)-1));

waypoint_idx = [1, steps, 2*steps, 3*steps, 4*steps];
fprintf('Velocity at each waypoint (should all be ~0):\n');
disp(qd1(:, waypoint_idx));

assert(max(abs(qd1(:, waypoint_idx)), [], 'all') < 1e-6, ...
    'Test 1 FAILED: expected zero velocity at all waypoints');
fprintf('Test 1 PASSED\n\n');

%% Test 2: Pass-through (non-zero velocity) at "lift" waypoint (#3)
fprintf('--- Test 2: Pass-through at waypoint 3 (lift) ---\n');

stopAtWaypoint = [true, true, false, true, true];

[q2, qd2, qdd2, t2] = plan_trajectory(angles, duration, steps, stopAtWaypoint);

v_expected = (angles(:,4) - angles(:,2)) / (2*duration);  % heuristic from the function
v_actual   = qd2(:, 2*steps);

fprintf('Expected via-velocity at waypoint 3: %s\n', mat2str(v_expected, 4));
fprintf('Actual velocity at waypoint 3:       %s\n', mat2str(v_actual, 4));

assert(all(abs(v_actual - v_expected) < 1e-6), ...
    'Test 2 FAILED: pass-through velocity does not match heuristic');
fprintf('Test 2 PASSED\n\n');

%% Test 3: Error handling - wrong-length stopAtWaypoint vector
fprintf('--- Test 3: Error handling for bad stopAtWaypoint size ---\n');

try
    badStop = [true, false, true];   % wrong length (3 instead of 5)
    plan_trajectory(angles, duration, steps, badStop);
    fprintf('Test 3 FAILED: expected an error but none was thrown\n\n');
catch ME
    fprintf('Correctly caught error: %s\n', ME.message);
    fprintf('Test 3 PASSED\n\n');
end

%% Test 4: Single joint, simple 2-waypoint trajectory (no via points)
fprintf('--- Test 4: Single joint, 2 waypoints ---\n');

angles_simple = [0, pi/2];
[q3, qd3, qdd3, t3] = plan_trajectory(angles_simple, 1, 100);

fprintf('Start angle: %.4f, End angle: %.4f (expected 0, %.4f)\n', ...
    q3(1,1), q3(1,end), pi/2);
fprintf('Start vel: %.6f, End vel: %.6f (both should be ~0)\n', ...
    qd3(1,1), qd3(1,end));

assert(abs(q3(1,1)) < 1e-6 && abs(q3(1,end) - pi/2) < 1e-6, ...
    'Test 4 FAILED: endpoint positions incorrect');
assert(abs(qd3(1,1)) < 1e-6 && abs(qd3(1,end)) < 1e-6, ...
    'Test 4 FAILED: endpoint velocities should be zero');
fprintf('Test 4 PASSED\n\n');

%% Test 5: First/last waypoint stop is always forced to true
fprintf('--- Test 5: First/last waypoint forced to full stop ---\n');

% Deliberately tell it NOT to stop at first/last; function should override this
stopBad = [false, true, false, true, false];
[q4, qd4, qdd4, t4] = plan_trajectory(angles, duration, steps, stopBad);

fprintf('Velocity at first waypoint (forced 0): %s\n', mat2str(qd4(:,1), 4));
fprintf('Velocity at last waypoint  (forced 0): %s\n', mat2str(qd4(:,end), 4));

assert(all(abs(qd4(:,1)) < 1e-6) && all(abs(qd4(:,end)) < 1e-6), ...
    'Test 5 FAILED: first/last waypoint velocity should be forced to zero');
fprintf('Test 5 PASSED\n\n');

%% Summary
fprintf('=====================================\n');
fprintf('All tests completed.\n');
fprintf('=====================================\n');