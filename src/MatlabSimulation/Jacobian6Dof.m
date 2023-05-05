close all
clear all
clc
% Import the required libraries
import robotics.RigidBodyTree;
import robotics.RigidBody;
import robotics.Joint;

% Define the symbolic variables for joint angles
syms q1 q2 q3 q4 q5 q6 real;

link_lengths = [0.09, 0.10, 0.150, 0.06, 0.10];
joint_angles = [0, 0 , 0 , 0 , 0 ,0]; % Replace these with your joint angle values

% Define DH parameters
dhParams_sym = [0 pi/2 link_lengths(1) q1; %A1 alpha1 D1 theta1
            link_lengths(2) 0 0 q2;
            link_lengths(3) 0 0 q3;
            0 pi/2 0 q4;
            0 pi/2 link_lengths(4) q5;
            0 0 link_lengths(5) q6];

dhParams = [0 pi/2 link_lengths(1) joint_angles(1); %A1 alpha1 D1 theta1
            link_lengths(2) 0 0 joint_angles(2);
            link_lengths(3) 0 0 joint_angles(3);
            0 pi/2 0 joint_angles(4);
            0 pi/2 link_lengths(4) joint_angles(5);
            0 0 link_lengths(5) joint_angles(6)];

        
% Create the robot model
robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 7);

% Loop through and create rigid bodies and joints for the robot
for i = 1:6
    bodyName = sprintf('link%d', i);
    jointName = sprintf('joint%d', i);
    body = rigidBody(bodyName);
    joint = rigidBodyJoint(jointName, 'revolute');
    
    setFixedTransform(joint, dhParams(i,:), 'dh');
    
    body.Joint = joint;
    if(i == 1)
        addBody(robot, body, 'base');
    else
        addBody(robot, body, robot.BodyNames{end});
    end
end

% Set the joint angles
% jointAngles = [q1_val, q2_val, q3_val, q4_val, q5_val, q6_val]; % Replace these with your joint angle values
config = homeConfiguration(robot);

% Visualize the robot
figure
show(robot, config);
title('Robot Visualization');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;

% Compute the geometric Jacobian symbolically
endEffector = 'link6';
config = homeConfiguration(robot);
JRobot_lib = geometricJacobian(robot, config, endEffector)

%%%MANUAL JACOBIAN
JSym = computeSymbolicJacobian(dhParams)
% Replace the symbolic variables with the actual joint angles when needed
J_numeric = double(subs(JSym, [q1, q2, q3, q4, q5, q6], joint_angles))

function J = computeSymbolicJacobian(dhParams)
    % Calculate transformation matrices for each joint
T = cell(1, 6);
for i = 1:6
    a = dhParams(i, 1);
    alpha = dhParams(i, 2);
    d = dhParams(i, 3);
    theta = dhParams(i, 4);

    T{i} = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
            sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
            0 sin(alpha) cos(alpha) d;
            0 0 0 1];
end

% Calculate overall transformation from base to end-effector
T_total = T{1} * T{2} * T{3} * T{4} * T{5} * T{6};

% Calculate the Jacobian using the transformation matrices
J = (zeros(6, 6));
for i = 1:6
    if i == 1
        T_prev = eye(4);
    else
        T_prev = T{1};
        for j = 2:i-1
            T_prev = T_prev * T{j};
        end
    end
    
    z_prev = T_prev(1:3, 3);
    p_prev = T_prev(1:3, 4);
    p_e = T_total(1:3, 4);
    
    J(1:3, i) = (cross(z_prev, p_e - p_prev));
    J(4:6, i) = z_prev;
end
end
