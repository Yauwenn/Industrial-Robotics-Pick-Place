%% Task Brief for Member 1: Robot Modeler
% Your Goal: Build the digital 3D model of our physical 3 DOF FYP robot
% using MATLAB's Robotics System Toolbox.
%
% This code creates:
% 1. Base / Link 1
% 2. Link 2
% 3. Link 3
% 4. Gripper
%
% It also includes:
% 1. Link lengths
% 2. Link masses
% 3. Centre of mass
% 4. Inertia
% 5. Joint limits
%
% The completed robot model can be used later for dynamics calculation,
% such as torque analysis.

function robot = build_robot()

% BUILD_ROBOT Creates a 3DOF rigid body tree for the Pick and Place arm
    
    %% =======================================================
    %  1. Initialize the robot tree structure
    %% =======================================================
    
    % 'column' format is required for later dynamics calculations
    robot = rigidBodyTree('DataFormat','column');

    %% =======================================================
    %  2. ENTER PHYSICAL MEASUREMENTS HERE
    %% =======================================================

    % --------------------------------------------------------
    % Link Lengths (metres)
    % --------------------------------------------------------
    L1 = 0.199;   % Height of Link 1 / base column, 199 mm
    L2 = 0.300;   % Length of Link 2, 300 mm
    L3 = 0.240;   % Length of Link 3, 240 mm
    Lg = 0.100;   % Length of gripper, 100 mm

    % --------------------------------------------------------
    % Link Dimensions (metres)
    % --------------------------------------------------------
    baseDiameter = 0.114;   % Diameter of cylindrical Link 1, 114 mm

    linkWidth  = 0.050;    % Width of Link 2 and Link 3 aluminium rod, 50 mm
    linkHeight = 0.025;    % Height/thickness of Link 2 and Link 3 aluminium rod, 25 mm

    gripperWidth  = 0.030; % Estimated gripper width
    gripperHeight = 0.020; % Estimated gripper height/thickness

    % --------------------------------------------------------
    % Link/Motor Masses (kg)
    % --------------------------------------------------------
    mass1 = 1.500;   % Mass of Link 1 including motor/parts, 1500 g
    mass2 = 1.028;   % Mass of Link 2 including motor/parts, 1028 g
    mass3 = 0.145;   % Mass of Link 3 including motor/parts, 145 g
    massG = 0.020;   % Mass of gripper, 20 g

    % --------------------------------------------------------
    % Joint Limits (degrees)
    % --------------------------------------------------------
    % MATLAB uses radians for joint limits.
    % Therefore, we first enter the values in degrees, then convert to radians.

    jnt1_limits_deg = [-180, 180];   % Joint 1 can rotate 360 degrees
    jnt2_limits_deg = [0, 90];       % Joint 2 rotation limit
    jnt3_limits_deg = [0, 90];       % Joint 3 rotation limit

    % Convert joint limits from degrees to radians
    jnt1_limits = deg2rad(jnt1_limits_deg);
    jnt2_limits = deg2rad(jnt2_limits_deg);
    jnt3_limits = deg2rad(jnt3_limits_deg);

    %% =======================================================
    %  3. BUILDING THE LINKS & JOINTS
    %% =======================================================

    % --------------------------------------------------------
    % LINK 1: Rotating Base / Cylindrical Vertical Link
    % --------------------------------------------------------

    body1 = rigidBody('link1');
    jnt1 = rigidBodyJoint('jnt1', 'revolute');

    % Joint 1 rotates about the Z-axis.
    % This represents the left-right rotation of the base.
    jnt1.JointAxis = [0 0 1];

    % Set Joint 1 rotation limits
    jnt1.PositionLimits = jnt1_limits;

    % Joint 1 is located at the robot base
    setFixedTransform(jnt1, trvec2tform([0, 0, 0]));

    % Attach the joint to Link 1
    body1.Joint = jnt1;

    % Add mass of Link 1
    body1.Mass = mass1;

    % Centre of mass of Link 1
    % Since Link 1 is a vertical cylinder, the COM is assumed at the middle.
    body1.CenterOfMass = [0, 0, L1/2];

    % Add inertia of Link 1
    % Link 1 is approximated as a solid cylinder.
    body1.Inertia = cylinderInertia(mass1, baseDiameter/2, L1);

    % Add simple visual shape for Link 1
    % MATLAB cylinder visual uses [radius length].
    addVisual(body1, "Cylinder", [baseDiameter/2, L1], trvec2tform([0, 0, L1/2]));

    % Add Link 1 to the robot tree
    addBody(robot, body1, 'base');

    % --------------------------------------------------------
    % LINK 2: Shoulder Link / Aluminium Rectangular Rod
    % --------------------------------------------------------

    body2 = rigidBody('link2');
    jnt2 = rigidBodyJoint('jnt2', 'revolute');

    % Joint 2 rotates about the X-axis.
    % This follows your provided joint axis value [1, 0, 0].
    jnt2.JointAxis = [1 0 0];

    % Set Joint 2 rotation limits
    jnt2.PositionLimits = jnt2_limits;

    % Joint 2 is located at the top of Link 1
    setFixedTransform(jnt2, trvec2tform([0, 0, L1]));

    % Attach the joint to Link 2
    body2.Joint = jnt2;

    % Add mass of Link 2
    body2.Mass = mass2;

    % Centre of mass of Link 2
    % Link 2 is assumed to be a uniform rectangular rod.
    body2.CenterOfMass = [L2/2, 0, 0];

    % Add inertia of Link 2
    % Link 2 is approximated as a rectangular box.
    body2.Inertia = boxInertia(mass2, L2, linkWidth, linkHeight);

    % Add simple visual shape for Link 2
    addVisual(body2, "Box", [L2, linkWidth, linkHeight], trvec2tform([L2/2, 0, 0]));

    % Add Link 2 to the robot tree
    addBody(robot, body2, 'link1');

    % --------------------------------------------------------
    % LINK 3: Elbow Link / Aluminium Rectangular Rod
    % --------------------------------------------------------

    body3 = rigidBody('link3');
    jnt3 = rigidBodyJoint('jnt3', 'revolute');

    % Joint 3 rotates about the X-axis.
    % This follows your provided joint axis value [1, 0, 0].
    jnt3.JointAxis = [1 0 0];

    % Set Joint 3 rotation limits
    jnt3.PositionLimits = jnt3_limits;

    % Joint 3 is located at the end of Link 2
    setFixedTransform(jnt3, trvec2tform([L2, 0, 0]));

    % Attach the joint to Link 3
    body3.Joint = jnt3;

    % Add mass of Link 3
    body3.Mass = mass3;

    % Centre of mass of Link 3
    % Link 3 is assumed to be a uniform rectangular rod.
    body3.CenterOfMass = [L3/2, 0, 0];

    % Add inertia of Link 3
    % Link 3 is approximated as a rectangular box.
    body3.Inertia = boxInertia(mass3, L3, linkWidth, linkHeight);

    % Add simple visual shape for Link 3
    addVisual(body3, "Box", [L3, linkWidth, linkHeight], trvec2tform([L3/2, 0, 0]));

    % Add Link 3 to the robot tree
    addBody(robot, body3, 'link2');

    % --------------------------------------------------------
    % END-EFFECTOR: Gripper
    % --------------------------------------------------------

    gripper = rigidBody('gripper');
    jnt_grip = rigidBodyJoint('jnt_grip', 'fixed');

    % The gripper is fixed at the end of Link 3.
    % It does not add another degree of freedom.
    setFixedTransform(jnt_grip, trvec2tform([L3, 0, 0]));

    % Attach the fixed joint to the gripper
    gripper.Joint = jnt_grip;

    % Add gripper mass
    gripper.Mass = massG;

    % Centre of mass of gripper
    % Gripper COM is assumed to be at the middle of its length.
    gripper.CenterOfMass = [Lg/2, 0, 0];

    % Add inertia of gripper
    % Gripper is approximated as a small rectangular box.
    gripper.Inertia = boxInertia(massG, Lg, gripperWidth, gripperHeight);

    % Add simple visual shape for gripper
    addVisual(gripper, "Box", [Lg, gripperWidth, gripperHeight], trvec2tform([Lg/2, 0, 0]));

    % Add gripper to the robot tree
    addBody(robot, gripper, 'link3');

    %% =======================================================
    %  4. VISUALIZATION TEST
    %% =======================================================
    % To test this function, run the following commands in MATLAB Command Window:
    %
    % robot = build_robot;
    % showdetails(robot);
    % show(robot);
    %
    % Do not write these commands inside the function unless your lecturer asks.

end


%% =======================================================
%  Helper Function 1: Box Inertia
%  This is used for Link 2, Link 3, and the gripper.
%  Do not edit this part.
%% =======================================================
function inertiaVector = boxInertia(m, x, y, z)
% boxInertia calculates the approximate inertia of a rectangular box.
%
% m = mass of the body
% x = length of the box
% y = width of the box
% z = height/thickness of the box
%
% MATLAB rigidBody inertia format:
% [Ixx Iyy Izz Iyz Ixz Ixy]

    Ixx = (1/12) * m * (y^2 + z^2);
    Iyy = (1/12) * m * (x^2 + z^2);
    Izz = (1/12) * m * (x^2 + y^2);

    inertiaVector = [Ixx, Iyy, Izz, 0, 0, 0];

end


%% =======================================================
%  Helper Function 2: Cylinder Inertia
%  This is used for Link 1 because Link 1 is cylindrical.
%  Do not edit this part.
%% =======================================================
function inertiaVector = cylinderInertia(m, r, h)
% cylinderInertia calculates the approximate inertia of a solid cylinder.
%
% m = mass of cylinder
% r = radius of cylinder
% h = height of cylinder
%
% The cylinder is assumed to stand vertically along the Z-axis.
%
% MATLAB rigidBody inertia format:
% [Ixx Iyy Izz Iyz Ixz Ixy]

    Ixx = (1/12) * m * (3*r^2 + h^2);
    Iyy = (1/12) * m * (3*r^2 + h^2);
    Izz = (1/2)  * m * r^2;

    inertiaVector = [Ixx, Iyy, Izz, 0, 0, 0];

end
