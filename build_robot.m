%%Task Brief for Member 1: Robot Modeler
%%Your Goal: Build the digital 3D model of our physical 3 DOF FYP robot using MATLAB's Robotics System Toolbox. Your code needs to create the base, the 3 moving links, and the gripper. Most importantly, you must add the mass and center of mass to each link so Member 3 can calculate the dynamics (torques) later.

%%Data You Need (I will provide these to you from my FYP robot):

%%Lengths: The physical length of Link 1, Link 2, and Link 3 (in meters).

%%Masses: The weight of the IG42 motors, the links, and the gripper (in kg).

%%Joint Limits: The maximum and minimum angles our physical robot can rotate without breaking.   



function robot = build_robot()



% BUILD_ROBOT Creates a 3DOF rigid body tree for the Pick and Place arm
    
    % 1. Initialize the robot tree structure
    % 'column' format is required for later dynamics calculations
    robot = rigidBodyTree('DataFormat','column');
    
    %% =======================================================
    %  TODO 1: ENTER PHYSICAL MEASUREMENTS HERE
    %  (Ask the Project Manager for these exact numbers)
    %% =======================================================
    % Link Lengths (meters)
    L1 = 0.15;  % Height of base/link1
    L2 = 0.20;  % Length of link 2
    L3 = 0.15;  % Length of link 3
    
    % Link/Motor Masses (kg)
    mass1 = 1.5; % Mass of Link 1 + Motor
    mass2 = 1.0; % Mass of Link 2 + Motor
    mass3 = 0.8; % Mass of Link 3 + Gripper + Servo
    
    % Joint Limits (Radians) - e.g., [-pi/2 to pi/2]
    jnt1_limits = [-pi, pi];       % Base rotation
    jnt2_limits = [-pi/2, pi/2];   % Shoulder rotation
    jnt3_limits = [-pi/2, pi/2];   % Elbow rotation

    %% =======================================================
    %  BUILDING THE LINKS & JOINTS
    %% =======================================================
    
    % --------------------------------------------------------
    % LINK 1 (Rotating Base)
    % --------------------------------------------------------
    body1 = rigidBody('link1');
    jnt1 = rigidBodyJoint('jnt1', 'revolute'); % Revolute = rotating joint
    
    % Set joint rotation limits
    jnt1.PositionLimits = jnt1_limits;
    
    % Attach joint to body, and define its translation from the base
    % (Moving it up by L1 along the Z-axis)
    setFixedTransform(jnt1, trvec2tform([0, 0, L1]));
    body1.Joint = jnt1;
    
    % Add Mass and Center of Mass (Assumed to be in the middle of the link)
    body1.Mass = mass1;
    body1.CenterOfMass = [0, 0, -L1/2]; 
    
    % Add to the tree (Attached to the world base)
    addBody(robot, body1, 'base');

    % --------------------------------------------------------
    % LINK 2 (Shoulder)
    % --------------------------------------------------------
    body2 = rigidBody('link2');
    jnt2 = rigidBodyJoint('jnt2', 'revolute');
    jnt2.PositionLimits = jnt2_limits;
    
    % The physical joint rotates around a different axis (like an elbow)
    % We rotate the joint frame so it bends correctly (Pitch instead of Yaw)
    jnt2_transform = trvec2tform([0, 0, 0]) * eul2tform([0, pi/2, 0]); 
    setFixedTransform(jnt2, jnt2_transform);
    
    body2.Joint = jnt2;
    body2.Mass = mass2;
    body2.CenterOfMass = [L2/2, 0, 0]; 
    
    % Add to the tree (Attached to link1)
    addBody(robot, body2, 'link1');

    % --------------------------------------------------------
    % LINK 3 (Elbow)
    % --------------------------------------------------------
    body3 = rigidBody('link3');
    jnt3 = rigidBodyJoint('jnt3', 'revolute');
    jnt3.PositionLimits = jnt3_limits;
    
    % Displaced by length of Link 2 along the X-axis
    setFixedTransform(jnt3, trvec2tform([L2, 0, 0])); 
    
    body3.Joint = jnt3;
    body3.Mass = mass3;
    body3.CenterOfMass = [L3/2, 0, 0];
    
    % Add to the tree (Attached to link2)
    addBody(robot, body3, 'link2');

    % --------------------------------------------------------
    % END-EFFECTOR (The Gripper)
    % --------------------------------------------------------
    gripper = rigidBody('gripper');
    % The gripper is "fixed" to the end of link 3 (it doesn't add a new DOF)
    jnt_grip = rigidBodyJoint('jnt_grip', 'fixed'); 
    
    % Displaced by length of Link 3 along the X-axis
    setFixedTransform(jnt_grip, trvec2tform([L3, 0, 0]));
    gripper.Joint = jnt_grip;
    
    % Add to the tree
    addBody(robot, gripper, 'link3');
    
    %% =======================================================
    %  VISUALIZATION TEST (You can run this to check your work!)
    %% =======================================================
    % If you want to see if it looks right, highlight these 
    % two lines, right-click, and press "Evaluate Selection":
    %
    % showdetails(robot);
    % show(robot);

end