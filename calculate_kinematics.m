function [T_end_effector] = calculate_kinematics(theta1, theta2, theta3)
    % CALCULATE_KINEMATICS Computes the forward kinematics of the 3DOF arm
    % Inputs: theta1, theta2, theta3 (Joint angles in radians)
    % Output: 4x4 Transformation Matrix of the end-effector
    
    %% =======================================================
    %  TODO 1: ENTER LINK LENGTHS (From Project Manager)
    %% =======================================================
    L1 = 0.15; % Height of base/link1
    L2 = 0.20; % Length of link 2 (shoulder to elbow)
    L3 = 0.15; % Length of link 3 (elbow to gripper)
    
    %% =======================================================
    %  TODO 2: FILL IN THE DH PARAMETER TABLE
    %  Based on Week 4/5 lectures. 
    %  Format: [ theta,  d (z-offset),  a (x-offset),  alpha (x-twist) ]
    %% =======================================================
    % NOTE: This is an assumed standard Articulated RRR setup. 
    % Verify these alphas and offsets with your hand drawings!
    
    %         |   Theta  |   d   |   a   |   Alpha  |
    dh_row1 = [   theta1,    L1,     0,     pi/2    ]; % Base to Shoulder
    dh_row2 = [   theta2,     0,    L2,        0    ]; % Shoulder to Elbow
    dh_row3 = [   theta3,     0,    L3,        0    ]; % Elbow to Gripper
    
    %% =======================================================
    %  CALCULATE TRANSFORMATION MATRICES
    %% =======================================================
    
    % Get individual transformation matrices using our helper function
    T_01 = build_dh_matrix(dh_row1(1), dh_row1(2), dh_row1(3), dh_row1(4));
    T_12 = build_dh_matrix(dh_row2(1), dh_row2(2), dh_row2(3), dh_row2(4));
    T_23 = build_dh_matrix(dh_row3(1), dh_row3(2), dh_row3(3), dh_row3(4));
    
    % Multiply them all together to get the final position!
    T_end_effector = T_01 * T_12 * T_23;
    
    % Optional: Print the final X, Y, Z position to the console for testing
    % X = T_end_effector(1,4);
    % Y = T_end_effector(2,4);
    % Z = T_end_effector(3,4);
    % fprintf('Gripper Position: X=%.3f, Y=%.3f, Z=%.3f\n', X, Y, Z);

end

%% =======================================================
% HELPER FUNCTION (Do not change this)
% This applies the standard DH transformation matrix formula
% from Dr. Abdo's Week 4/5 Lecture slides.
%% =======================================================
function T = build_dh_matrix(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                  0,             sin(alpha),             cos(alpha),            d;
                  0,                      0,                      0,            1];
end