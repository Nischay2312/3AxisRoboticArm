close all
clear all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       BUILDING THE ROBOT          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Import required packages
import robotics.RigidBodyTree;
import robotics.RigidBody;

% Define robot arm
num_links = 5;
link_lengths = [0.047855 0.117, 0.095, 0.0276, 0.0381];

% Create a RigidBodyTree object
robot = RigidBodyTree;

% Joint rotation axes (change these as needed)
joint_axes = [
    0 0 1;
    1 0 0;
    1 0 0;
    0 1 0;
    1 0 0
];
link_axes = [
    0 0 1;
    0 0 1;
    0 1 0;
    0 1 0;
    0 1 0
];

% Create links and attach them to the robot
for i = 1:num_links
    if i == 1
        body = RigidBody("RotatingBase");
        joint = robotics.Joint(['BaseJoint', num2str(i)], 'revolute');
        joint.JointAxis = joint_axes(i, :);
        setFixedTransform(joint, trvec2tform(link_lengths(i)*link_axes(i,:)));
        body.Joint = joint;
        addBody(robot, body, 'base');
    else
        body = RigidBody(['Link', num2str(i-1)]);
        joint = robotics.Joint(['Joint', num2str(i)], 'revolute');
        joint.JointAxis = joint_axes(i, :);
        setFixedTransform(joint, trvec2tform(link_lengths(i-1)*link_axes(i,:)));
        body.Joint = joint;
        addBody(robot, body, robot.BodyNames{end});
    end
end

% Add end-effector
end_effector_length = 0.05; % Change this to set the length of the end-effector
end_effector = RigidBody('EndEffector');
setFixedTransform(end_effector.Joint, trvec2tform([0, end_effector_length, 0.005]));
addBody(robot, end_effector, robot.BodyNames{end});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       VERIFYING THE TRANSFORMATION MATRIX          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

joint_angles = pi/180*[90, 90, 90, 0, 00]; % Change this to the desired initial joint angles
Cosines = [ cos(joint_angles(1)), cos(joint_angles(2)), cos(joint_angles(3)), cos(joint_angles(4)), cos(joint_angles(5))];
Sines = [ sin(joint_angles(1)), sin(joint_angles(2)), sin(joint_angles(3)), sin(joint_angles(4)), sin(joint_angles(5))];

% Calculate the transformation matrix using forward kinematics
config_verification = homeConfiguration(robot);
for i = 1:num_links
    config_verification(i).JointPosition = joint_angles(i);
end
figure(1)
ax = show(robot, config_verification);
ax.XLim = [-1, 1]; ax.YLim = [-1, 1]; ax.ZLim = [-1, 1]; % Set axis limits
ax.DataAspectRatio = [1 1 1];
title('Robot Arm Animation');

T = getTransform(robot, config_verification, robot.BodyNames{end});

% Display the transformation matrix
disp('Transformation matrix Toolbox:');
disp(T);

%manual Matrix:
T01 = [ Cosines(1)  -Sines(1)       0       0;
        Sines(1)     Cosines(1)     0       0;
        0            0              1       link_lengths(1);
        0            0              0       1
      ];

T12 = [ 1            0              0       0;
        0     Cosines(2)     -Sines(2)      0;
        0      Sines(2)     Cosines(2)      link_lengths(1);
        0            0              0       1
      ];

T23 = [ 1            0              0       0;
        0    Cosines(3)      -Sines(3)      link_lengths(2);
        0      Sines(3)     Cosines(3)      0;
        0            0              0       1
      ];

T34 = [ Cosines(4)   0       Sines(4)      0;
        0            1              0       link_lengths(3);
        -Sines(4)    0      Cosines(4)      0;
        0            0              0       1
      ];

T45 = [ 1            0              0       0;
        0    Cosines(5)      -Sines(5)      link_lengths(4);
        0      Sines(5)     Cosines(5)      0;
        0            0              0       1
      ];
  
T5ee = [ 1           0              0       0;
        0            1              0      end_effector_length;
        0            0              1       0.005;
        0            0              0       1
      ];

T_manual = T01 * T12 * T23 * T34 * T45 * T5ee;
% Display the transformation matrix
disp('Transformation matrix(manual):');
disp(T_manual);
 
rotm1 = T_manual(1:3, 1:3);
rotm2 = T(1:3, 1:3);

eulXYZ1 = 180/pi*rotm2eul(rotm1,'XYZ');
eulXYZ2 = 180/pi*rotm2eul(rotm2,'XYZ');

disp('Orienataion(manual):');
disp(eulXYZ1);
disp('Orienataion(toolbox):');
disp(eulXYZ2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       ANIMATING THE ROBOT          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2)

% Initial and final joint angles (in radians)
joint_angles_initial = pi/180*[0, 0, 0, 0, 0]; % Change this to the desired initial joint angles
joint_angles_final = joint_angles; % Change this to the desired final joint angles

% Create the configuration objects
config_initial = homeConfiguration(robot);
config_final = homeConfiguration(robot);

for i = 1:num_links
    config_initial(i).JointPosition = joint_angles_initial(i);
    config_final(i).JointPosition = joint_angles_final(i);
end
% Animation settings
n_steps = 20; % Number of steps in the animation
pause_duration = 0.001; % Pause duration between steps (in seconds)

% Visualize the robot arm moving from the initial to the final joint angles
figure(2)
ax = show(robot, config_initial);
ax.XLim = [-1, 1]; ax.YLim = [-1, 1]; ax.ZLim = [-1, 1]; % Set axis limits
ax.DataAspectRatio = [1 1 1];
title('Robot Arm Animation');

% Initialize arrays to store end-effector positions
end_effector_positions = zeros(n_steps, 3);

for step = 1:n_steps
    config_intermediate = homeConfiguration(robot);
    for i = 1:num_links
        config_intermediate(i).JointPosition = joint_angles_initial(i) + (step/n_steps) * (joint_angles_final(i) - joint_angles_initial(i));
    end
    % Store end-effector position
    T_endeffector = getTransform(robot, config_intermediate, robot.BodyNames{end});
    position_endeffector = tform2trvec(T_endeffector);
    end_effector_positions(step, :) = position_endeffector;
    show(robot, config_intermediate, 'Parent', ax);
    pause(pause_duration);
end

% Add labels for each link and the end-effector
hold on;
plot3(end_effector_positions(:,1), end_effector_positions(:,2)+ end_effector_length, end_effector_positions(:,3), 'b.-', 'MarkerSize', 10);
for i = 1:(num_links + 1)
    body = robot.BodyNames{i};
    T = getTransform(robot, config_final, body);
    position = tform2trvec(T);
    if i <= num_links
        text(position(1), position(2), position(3), ['Link', num2str(i)], 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');
    else
        text(position(1), position(2), position(3), 'EndEffector', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');
    end
end
hold off;


