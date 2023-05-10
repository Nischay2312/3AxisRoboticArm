close all
clear all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       BUILDING THE ROBOT          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Import required packages
import robotics.RigidBodyTree;
import robotics.InverseKinematics;
import robotics.RigidBody;

% Define robot arm
num_links = 7; %(6 moving + 1 endeffector)
link_lengths = [0.03, 0.064, 0.10289, 0.12379, 0.05262, 0.033, 0.1074];

% Create a RigidBodyTree object
robot = RigidBodyTree;

% Joint rotation axes (change these as needed)
joint_axes = [
    0 0 1;
    1 0 0;
    1 0 0;
    1 0 0;
    0 1 0;
    0 0 1
];
link_axes = [
    0 0 1;
    0 0 1;
    0 1 0;
    0 1 0;
    0 1 0;
    0 1 0;
];

% Create links and attach them to the robot
for i = 1:num_links-1
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
        setFixedTransform(joint, trvec2tform(link_lengths(i)*link_axes(i,:)));
        body.Joint = joint;
        addBody(robot, body, robot.BodyNames{end});
    end
end

% Add end-effector
end_effector = RigidBody('EndEffector');
setFixedTransform(end_effector.Joint, trvec2tform([0, 0, -1*link_lengths(7)]));
addBody(robot, end_effector, robot.BodyNames{end});


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       VERIFYING THE TRANSFORMATION MATRIX          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

joint_angles = pi/180*[0, 120, -110, 0, 0, 0]; % Change this to the desired initial joint angles

% Calculate the transformation matrix using forward kinematics
config_verification = homeConfiguration(robot);
for i = 1:num_links-1
    config_verification(i).JointPosition = joint_angles(i);
end
figure(1)
ax = show(robot, config_verification, 'Frames', 'on');
ax.XLim = [-1, 1]; ax.YLim = [-1, 1]; ax.ZLim = [-1, 1]; % Set axis limits
ax.DataAspectRatio = [1 1 1];
title('Robot Cofiguration for Joint Angles Set 1');

T_libray = getTransform(robot, config_verification, robot.BodyNames{end});
J_libary = geometricJacobian(robot, config_verification, 'EndEffector');
T_manual = forward_kinematics(joint_angles, link_lengths);

% % Display the jacobian
% disp('Robot Jacobian:');
% disp(J_libary);
% 
% % Display the library transformation matrix
% disp('Transformation matrix Toolbox:');
% disp(T_libray);

% Display the manual transformation matrix
disp('Transformation matrix(manual):');
disp(T_manual);
 
% get the Euler Angles
rotm1 = T_manual(1:3, 1:3);
rotm2 = T_libray(1:3, 1:3);

eulXYZ1 = 180/pi*rotm2eul(rotm1,'XYZ');
eulXYZ2 = 180/pi*rotm2eul(rotm2,'XYZ');

%display them
disp('Orienataion(manual):');
disp(eulXYZ1);
% disp('Orienataion(toolbox):');
% disp(eulXYZ2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       INVERSE KINEMATICS           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
max_iterations = 10;
tolerance = 1e-2;

J_sym = calculateSymbolicJacobian(link_lengths);

%%%%%%%%%%%%%%%--Multi Point Path Planning--%%%%%%%%%%%%%%%%%
% Define target points as an N x 6 matrix, where N is the number of target points
% Each row contains XYZ position and Euler angles in the format [X Y Z Rx Ry Rz steps]
target_points = [
    0.00 0.1698 0.3059 10 0 0 5;
    0.00 0.1718 0.1945 10 0 0 5;
    0.00 0.1718 0.1945 10 0 0 5;
    0.00 0.2300 0.1945 10 0 0 5;
    0.00 0.1718 0.1945 10 0 0 5;
    0.00 0.1698 0.3059 10 0 0 5;
%     0.00 0.1698 0.2859 10 0 0 10;
    % Add more target points as needed
];

XYZ_desired = target_points(end, 1:3)';
euler_desired = pi/180*target_points(end, 4:6);
RotM_desired = eul2rotm(euler_desired, 'XYZ');
    
Tdesired = [ RotM_desired XYZ_desired;
                0 0 0 1
               ];

% T_input = T_manual;
T_input = Tdesired;

joint_angles_initial = pi/180*[0 130 -93 20 0 0]; % Change this to the desired initial joint angles

% Call the multi_point_path_planning function
JointArray = multi_point_path_planning(joint_angles_initial, target_points, link_lengths, J_sym);

%display the position of the endeffector and the error
FinalJoints = JointArray(end, :);

% Display the manual transformation matrix
disp('Transformation matrix (Target):');
disp(T_input);

T_end = forward_kinematics(FinalJoints, link_lengths);
disp("Final T Matrix:");
disp(T_end);

PathError = compute_pose_error(T_input, T_end);
disp("Solver Error:");
disp(PathError);

disp("Final Joints values:");
disp(FinalJoints*180/pi);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% XYZ_desired = [0.00; 0.15; 0.2];
% euler_desired = pi/180*[0 0 0];
% RotM_desired = eul2rotm(euler_desired, 'XYZ');
%     
% Tdesired = [ RotM_desired XYZ_desired;
%                 0 0 0 1
%                ];
% 
% % T_input = T_manual;
% T_input = Tdesired;
%            
% % IKINTest(robot, link_lengths, num_links, J_sym);
% %plan path from initial joint angles to Tmanual
% joint_angles_initial = pi/180*[0   0   0 0        0         0]; % Change this to the desired initial joint angles
% 
% nSteps = 15;
% JointArray = PathPlanning(joint_angles_initial, T_input, J_sym, nSteps, link_lengths);
% 
% %display the position of the endeffector and the error
% FinalJoints = JointArray(end, :);
% 
% % Display the manual transformation matrix
% disp('Transformation matrix (Target):');
% disp(T_input);
% 
% T_end = forward_kinematics(FinalJoints, link_lengths);
% disp("Final T Matrix:");
% disp(T_end);
% 
% PathError = compute_pose_error(T_input, T_end);
% disp("Solver Error:");
% disp(PathError);
% 
% disp("Final Joints values:");
% disp(FinalJoints*180/pi);






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       ANIMATING THE ROBOT          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function AnimateRobot(robot, initial_joints, final_joints, num_links, link_lengths, o)
 
    figure(o);
    % Initial and final joint angles (in radians)
    joint_angles_initial = initial_joints; % Change this to the desired initial joint angles
    joint_angles_final = final_joints; % Change this to the desired final joint angles

    % Create the configuration objects
    config_initial = homeConfiguration(robot);
    config_final = homeConfiguration(robot);

    for i = 1:num_links-1
        config_initial(i).JointPosition = joint_angles_initial(i);
        config_final(i).JointPosition = joint_angles_final(i);
    end
    % Animation settings
    n_steps = 20; % Number of steps in the animation
    pause_duration = 0.001; % Pause duration between steps (in seconds)

    % Visualize the robot arm moving from the initial to the final joint angles
    figure(o)
    ax = show(robot, config_initial);
    ax.XLim = [-1, 1]; ax.YLim = [-1, 1]; ax.ZLim = [-1, 1]; % Set axis limits
    ax.DataAspectRatio = [1 1 1];
    title('Robot Arm Animation');

    % Initialize arrays to store end-effector positions
    end_effector_positions = zeros(n_steps, 3);

    for step = 1:n_steps
        config_intermediate = homeConfiguration(robot);
        for i = 1:num_links-1
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
    plot3(end_effector_positions(:,1), end_effector_positions(:,2)+ link_lengths(7), end_effector_positions(:,3), 'b.-', 'MarkerSize', 10);
    for i = 1:(num_links)
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
    
end

function AnimmateRobotHistory( robot, AngleHistory, num_links, figureNum)
    % Animation settings
    n_steps = size(AngleHistory,1); % Number of steps in the animation
    pause_duration = 0.05; % Pause duration between steps (in seconds)

    % Visualize the robot arm moving from the initial to the final joint angles
    figure(figureNum)   
    config_initial = homeConfiguration(robot);
    for i = 1:num_links-1
        config_initial(i).JointPosition = AngleHistory(1,i);
    end
    ax = show(robot, config_initial);
    ax.XLim = [-1, 1]; ax.YLim = [-1, 1]; ax.ZLim = [-1, 1]; % Set axis limits
    ax.DataAspectRatio = [1 1 1];
    title('Robot Arm Animation');

    % Initialize arrays to store end-effector positions 
    end_effector_positions = zeros(n_steps, 3);

    for step = 1:n_steps
        config_intermediate = homeConfiguration(robot);
        for i = 1:num_links-1
            config_intermediate(i).JointPosition = AngleHistory(step,i);
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
    plot3(end_effector_positions(:,1), end_effector_positions(:,2), end_effector_positions(:,3), 'b.-', 'MarkerSize', 10);
    for i = 1:(num_links)
        body = robot.BodyNames{i};
        T = getTransform(robot, config_intermediate, body);
        position = tform2trvec(T);
        if i <= num_links
            text(position(1), position(2), position(3), ['Link', num2str(i)], 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');
        else
            text(position(1), position(2), position(3), 'EndEffector', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');
        end
    end
    hold off;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Functions For IKIN          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function error = compute_pose_error(TDesired, TCurrent)
    current_position = TCurrent(1:3, 4);
    current_orientation = TCurrent(1:3, 1:3);
    desired_position = TDesired(1:3, 4);
    desired_orientation = TDesired(1:3, 1:3);
    position_error = (desired_position - current_position);
    orientation_err =(rotm2eul(desired_orientation,'XYZ') - rotm2eul(current_orientation,'XYZ'));
    error = [orientation_err*0 , position_error']';
end

function [joint_solution, AngleHistory1, ErrorHistory1, Jacobian] = inverse_kinematics_Jacobian(TDesired, Jacobian_Sym, initial_guess, link_lengths, max_iterations, tolerance)
    disp('entered ikin func')
    joint_solution = initial_guess;
    Tcurrent = forward_kinematics(joint_solution, link_lengths);
    Error = compute_pose_error(TDesired, Tcurrent);
    AngleHistory = zeros(max_iterations, numel(joint_solution));
    ErrorHistory = zeros(max_iterations);
    %define the robots joint limits
    jointLimMax = pi/180*[95 145 40 100 98 97];
    jointLimMin = pi/180*[-85 -35 -140 -80 -82 -83];
    for i = 1:max_iterations
        Jacobian = evaluateJacobian(Jacobian_Sym, joint_solution);
        disp('jacobian evaluated')
        if norm(Error) < tolerance
            disp('position reached')
            break
        end
        AngleHistory(i,:) = joint_solution;
        ErrorHistory(i) = norm(Error);
%          angleStep = 1*pinv(Jacobian)*(Error);
%       angleStep = 0.01*(Jacobian)'*(Error);
        %lambda = 0.5; % Damping factor, you can adjust this value
        lambda = 0.001;
        angleStep = ((Jacobian' * Jacobian) + (lambda^2 * eye(size(Jacobian, 1))))\Jacobian' * (Error);
        joint_solution = joint_solution + angleStep';
%       %keep the joint angles between -pi/2 and pi/2
        for j = 1:numel(joint_solution)
%             joint_solution(j) = min(max(joint_solution(j), jointLimMax(j)), jointLimMin(j));
        end
%         joint_solution = mod(joint_solution, 2*pi);
        Tcurrent = forward_kinematics(joint_solution, link_lengths);
        Error = compute_pose_error(TDesired, Tcurrent); 
        disp('one iteration done')
    end
    AngleHistory1 = AngleHistory(1:i-1,:);
    ErrorHistory1 = ErrorHistory(1:i-1);
    disp('ikin complete')
end

function T = forward_kinematics(joint_angles, link_lengths)
    % Unpack the joint angles and link lengths
    q1 = joint_angles(1);
    q2 = joint_angles(2);
    q3 = joint_angles(3);
    q4 = joint_angles(4);
    q5 = joint_angles(5);
    q6 = joint_angles(6);

    b = link_lengths(1);
    l1 = link_lengths(2);
    l2 = link_lengths(3);
    l3 = link_lengths(4);
    l4 = link_lengths(5);
    l5 = link_lengths(6);
    l6 = link_lengths(7);

    % Calculate the transformation matrices using the given joint angles and link lengths
    T01 = [ cos(q1)  -sin(q1)  0  0;
            sin(q1)   cos(q1)  0  0;
                  0        0   1  b;
                  0        0   0  1 ];

    T12 = [ 1  0        0         0;
            0  cos(q2) -sin(q2)  0;
            0  sin(q2)  cos(q2)  l1;
            0  0        0        1 ];

    T23 = [ 1  0        0         0;
            0  cos(q3) -sin(q3)  l2;
            0  sin(q3)  cos(q3)  0;
            0  0        0        1 ];

    T34 = [ 1  0        0         0;
            0  cos(q4) -sin(q4)  l3;
            0  sin(q4)  cos(q4)  0;
            0  0        0        1 ];

    T45 = [ cos(q5)  0  sin(q5)  0;
                  0  1        0  l4;
           -sin(q5)  0  cos(q5)  0;
                  0  0        0  1 ];

    T56 = [ cos(q6)  -sin(q6)  0  0;
            sin(q6)   cos(q6)  0  l5;
                  0        0   1  0;
                  0        0   0  1 ];

    T67 = [ 1  0  0  0;
            0  1  0  0;
            0  0  1  -1*l6;
            0  0  0  1 ];

    % Calculate the end-effector transformation matrix by multiplying the individual transformation matrices
    T = T01 * T12 * T23 * T34 * T45 * T56 * T67;
end

function J_num = evaluateJacobian(J_sym,joint_angles)
    syms q1 q2 q3 q4 q5 q6 real
    J_num = double(subs(J_sym, symvar(J_sym), joint_angles));
end

function J = calculateSymbolicJacobian(link_lengths)
syms q1 q2 q3 q4 q5 q6 b l1 l2 l3 l4 l5 l6 real
Sym_T = TranslationMatrix_Symbolic;
% link_lengths = [0.04, 0.05, 0.10, 0.150, 0.05, 0.01, 0.10];
Sym_T = subs(Sym_T, [b, l1, l2, l3, l4, l5, l6], link_lengths);

X_pos = Sym_T(1,4);
Y_pos = Sym_T(2,4);
Z_pos = Sym_T(3,4);
% Extract the rotation matrix R from the transformation matrix Sym_T
R = Sym_T(1:3, 1:3);
% Compute symbolic Euler angles (XYZ)
% EulerAngles = [atan2(R(3,2), R(3,3));
%                -asin(R(3,1)); 
%                atan2(R(2,1), R(1,1))
%               ] ;

EulerAngles = [atan2(R(3,2), R(3,3));
               atan2(-1*(R(3,1)), sqrt(R(3,2)*R(3,2) + R(3,3)*R(3,3))); 
               atan2(R(2,1), R(1,1))
              ];

%Define the Task Space variables 
TaskSpace = [   EulerAngles(1);
                EulerAngles(2);
                EulerAngles(3);
                X_pos;
                Y_pos;
                Z_pos
            ];

%define the joint space variables
jointSpace = [  q1;
                q2;
                q3;
                q4;
                q5;
                q6;
             ];

%Compute the Symbolic Jacobian
J = sym(zeros(numel(TaskSpace), numel(jointSpace)));
for i = 1:numel(TaskSpace)
    for j = 1:numel(jointSpace)
        J(i,j) = diff(TaskSpace(i), jointSpace(j)); 
    end
end
% J = jacobian(TaskSpace, jointSpace)

joint_angles = pi/180*[0, 0, 0, 0, 0, 0]; % Replace these with your joint angle values
 
X_num = (subs(X_pos , [q1, q2, q3, q4, q5, q6], joint_angles));
Y_num = (subs(Y_pos , [q1, q2, q3, q4, q5, q6], joint_angles));
Z_num = (subs(Z_pos , [q1, q2, q3, q4, q5, q6], joint_angles));
EulerAngles_num = (subs(EulerAngles , [q1, q2, q3, q4, q5, q6], joint_angles));
Sym_T_num = double(subs(Sym_T, symvar(Sym_T), [joint_angles]));

X_pos = Sym_T_num(1,4);
Y_pos = Sym_T_num(2,4);
Z_pos = Sym_T_num(3,4);
eulXYZ = 180/pi*rotm2eul(Sym_T_num(1:3,1:3),'XYZ')';
end

function Sym_T = TranslationMatrix_Symbolic
syms q1 q2 q3 q4 q5 q6 b l1 l2 l3 l4 l5 l6   %Robot joint angle and link lengths

%Declare the variables
Sym_Cosines = [ cos(q1), cos(q2), cos(q3), cos(q4), cos(q5), cos(q6)];
Sym_Sines = [ sin(q1), sin(q2), sin(q3), sin(q4), sin(q5), sin(q6)];
Sym_link_lengths = [ b, l1, l2, l3, l4, l5, l6];

%form the Transformation matrices
Sym_T01 = [ Sym_Cosines(1)  -Sym_Sines(1)       0       0;
            Sym_Sines(1)     Sym_Cosines(1)     0       0;
                    0            0              1       Sym_link_lengths(1);
                    0            0              0       1
          ];

Sym_T12 = [ 1            0              0       0;
            0     Sym_Cosines(2)     -Sym_Sines(2)      0;
            0      Sym_Sines(2)     Sym_Cosines(2)      Sym_link_lengths(2);
            0            0              0       1
          ];

Sym_T23 = [ 1            0              0       0;
            0    Sym_Cosines(3)      -Sym_Sines(3)      Sym_link_lengths(3);
            0      Sym_Sines(3)     Sym_Cosines(3)      0;
            0            0              0       1
          ];
      
Sym_T34 = [ 1            0              0       0;
            0    Sym_Cosines(4)      -Sym_Sines(4)      Sym_link_lengths(4);
            0      Sym_Sines(4)     Sym_Cosines(4)      0;
            0            0              0       1
          ];


Sym_T45 = [ Sym_Cosines(5)   0       Sym_Sines(5)      0;
                0            1              0       Sym_link_lengths(5);
            -Sym_Sines(5)    0      Sym_Cosines(5)      0;
                0            0              0       1
           ];

Sym_T56 = [ Sym_Cosines(6)  -Sym_Sines(6)       0       0;
            Sym_Sines(6)     Sym_Cosines(6)     0       Sym_link_lengths(6);
                    0            0              1       0;
                    0            0              0       1
          ];
  
Sym_T67 = [ 1            0              0       0;
            0            1              0       0;
            0            0              1       -1*Sym_link_lengths(7);
            0            0              0       1
          ];

Sym_T = Sym_T01 * Sym_T12 * Sym_T23 * Sym_T34 * Sym_T45 * Sym_T56 * Sym_T67;
% disp('Tranfomration Matrix Symbolic:');
% disp(Sym_T);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Functions For PathPlanning          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function jointList1 = PathPlanning(initial_joints, TDesired, J_sym, nSteps, link_lengths)
    %allocate the resulting joint list
    jointList = zeros(nSteps, numel(initial_joints));
    
    %define the robots joint limits
    jointLimMax = pi/180*[95 145 40 100 98 97];
    jointLimMin = pi/180*[-85 -35 -140 -80 -82 -83];

    %compute the initial solution
    Tcurrent = forward_kinematics(initial_joints, link_lengths);

    %convert the desired transformation matrix to a vector
    des_EulerAngles_XYZ = rotm2eul(TDesired(1:3,1:3),'XYZ')';
    des_Position_XYZ = TDesired(1:3,4);
    cur_EulerAngles_XYZ = rotm2eul(Tcurrent(1:3,1:3),'XYZ')';
    cur_Position_XYZ = Tcurrent(1:3,4);

    desiredVariables = [des_EulerAngles_XYZ; des_Position_XYZ];
    currentVariables = [cur_EulerAngles_XYZ; cur_Position_XYZ];

    %get the step size for each path step
    stepSize = (desiredVariables - currentVariables)/nSteps;

    %set up for the solver
    max_iterations = 6;
    tolerance = 0.001;
    %compute the path
    singubreak = 0;
    for i = 1:nSteps
        disp('Iteration:')
        disp(i)
        %compute the desired variables for this step
        currentVariables = currentVariables + stepSize;
        disp('Current Variables');
        disp(currentVariables);

        %convert the desired variables to a transformation matrix
        XYZ_desired = currentVariables(4:6);
        euler_desired = currentVariables(1:3)';
        RotM_desired = eul2rotm(euler_desired, 'XYZ');
    
        Tdesired = [ RotM_desired XYZ_desired;
                        0 0 0 1
                    ];
%       Tdesired = eul2tform(currentVariables(1:3)','XYZ') * ([currentVariables(4:6); 1]);
        %do the inverse kinematics 
        [desiredJoints, AngleHistory, ErrorHistory, Jacobian] = inverse_kinematics_Jacobian(Tdesired, J_sym, initial_joints, link_lengths, max_iterations, tolerance);        
%         Jacobian = evaluateJacobian(J_sym, desiredJoints);
        Jacobian_det = det(Jacobian);
        disp('Jacobian');
        disp(Jacobian);
        disp('Jacobian Determinant')
        disp(Jacobian_det);
        if(abs(Jacobian_det) < 0.0001)
            disp('Approaching a singularity. Terminating Path Planner');
            disp('Singularity is near: ');
            disp(Tdesired);
            disp('Current Position Desired');
            disp(currentVariables);
            disp('Joint angles needed to achieve the position');
            disp(180/pi*desiredJoints);
            singubreak = 1;
            break
        end
        %keep the joint angles between -2pi and 2pi
        for j = 1:numel(desiredJoints)
%             if desiredJoints(j) > 2*pi
%                 desiredJoints(j) = mod(desiredJoints(j), - 2*pi);
%             elseif desiredJoints(j) < -2*pi
%                 desiredJoints(j) = mod(desiredJoints(j), 2*pi);
%             end
              %keep the joint angles between -pi/2 and pi/2
              %desiredJoints(j) = max(min(desiredJoints(j), pi), -pi);
              desiredJoints(j) = max(min(desiredJoints(j), jointLimMax(j)), jointLimMin(j));

        end

        %store the desired joint angles
        jointList(i,:) = desiredJoints;

        %update the initial joints for the next step
        initial_joints = desiredJoints;

    end
    if singubreak == 1
        jointList1 = jointList(1:i-1,:);
    else
        jointList1 = jointList(1:i,:);
    end
end

function JointArray1 = multi_point_path_planning(joint_angles_initial, target_points, link_lengths, J_sym)

    % Calculate the total number of rows required for the JointArray
    nSteps = sum(target_points(:, 7));
    total_rows = nSteps * size(target_points, 1);

    % Preallocate memory for the JointArray
    JointArray = zeros(total_rows, numel(joint_angles_initial));

    % Initialize the starting row index for each path segment
    start_idx = 1;

    % Loop through target points
    for i = 1:size(target_points, 1)
        % Extract the desired XYZ and Euler angles for the current target point
        XYZ_desired = target_points(i, 1:3)';
        euler_desired = pi/180 * target_points(i, 4:6);
        RotM_desired = eul2rotm(euler_desired, 'XYZ');
        nSteps = target_points(i, 7);

        Tdesired = [RotM_desired XYZ_desired;
                    0 0 0 1];
                
        disp('calculating trajectory for:')
        disp(Tdesired)
        disp('current angles:')
        disp(joint_angles_initial)
        

        % Perform path planning from the current joint configuration to the target point
        JointPath = PathPlanning(joint_angles_initial, Tdesired, J_sym, nSteps, link_lengths);
        
        % Get the length of the JointPath
        path_length = size(JointPath, 1);

        % Update the end_idx based on the path_length
        end_idx = start_idx + path_length - 1;
        JointArray(start_idx:end_idx, :) = JointPath;

        % Update the initial joint configuration for the next target point
        joint_angles_initial = JointPath(end, :);

        % Update the starting row index for the next path segment
        start_idx = end_idx + 1;
    end
    JointArray1 = JointArray(1:end_idx,:);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Functions For Testing               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function IKINTest(robot, link_lengths, num_links, J_sym)
    % Import required packages
    import robotics.RigidBodyTree;
    import robotics.InverseKinematics;
    import robotics.RigidBody;

    XYZ_desired = [0.000; 0.1973; 0.1618];
    euler_desired = pi/180*[0 0 0];
    RotM_desired = eul2rotm(euler_desired, 'XYZ');
    
    Tdesired = [ RotM_desired XYZ_desired;
                0 0 0 1
               ];
           
    disp("Desired TMatrix");
    disp(Tdesired);
    
    initial_guess = pi/180*[0 0 0 0 0 0]; 
    
    T_test = forward_kinematics(initial_guess, link_lengths);
    disp('Starting Transformation Matrix:');
    disp(T_test);
    
    Test_error = compute_pose_error(Tdesired, T_test);
    
    %now use the inbuilt solver to get the joint angles
    ik = robotics.InverseKinematics('RigidBodyTree', robot);
    weights = [0.25 0.25 0.25 1 1 1];
    desired_pose = Tdesired;
    initial_guess_config = homeConfiguration(robot);
    for i = 1:length(initial_guess)
        initial_guess_config(i).JointPosition = initial_guess(i);
    end
    [jointConfigSol, info] = ik('EndEffector', desired_pose, weights, initial_guess_config);
    joint_angles_Solver = zeros(1, 6);
    
    for i = 1:6
        joint_angles_Solver(i) = jointConfigSol(i).JointPosition;
    end
    T_final_solver = forward_kinematics(joint_angles_Solver, link_lengths);
    disp("Final Transformation matrix(SOLVER):");
    disp(T_final_solver);
    
    disp('Joint angles solution(IK SOLVER):');
    disp(180/pi*joint_angles_Solver);
    
    AnimateRobot(robot, initial_guess, joint_angles_Solver, num_links, link_lengths, 5)
    
    
    [joint_solution, AngleHistory, ErrorHistory] = inverse_kinematics_Jacobian(Tdesired, J_sym, initial_guess, link_lengths, max_iterations, tolerance);
    disp('Joint solution:');
    disp(180/pi*joint_solution);
    
    T_final = forward_kinematics(joint_solution, link_lengths);
    disp("Final Transformation matrix:");
    disp(T_final);
    disp("Final position error:")
    disp(compute_pose_error(Tdesired, T_final))
    
    %Make the robot go to the resulting position
    AnimateRobot(robot, initial_guess, joint_solution, num_links, link_lengths, 2)
    
    AnimmateRobotHistory( robot, AngleHistory, num_links, 4)
    
    figure(3);
    plot(180/pi*AngleHistory);
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
    
    figure(6);
    plot(ErrorHistory);
end