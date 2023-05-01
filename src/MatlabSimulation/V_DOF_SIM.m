close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       BUILDING THE ROBOT          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Import required packages
import robotics.RigidBodyTree;
import robotics.RigidBody;

% Define robot arm
num_links = 5;
link_lengths = [0.047855, 0.047855, 0.117, 0.095, 0.0276, 0.0381, 0.05, 0.005];

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
end_effector = RigidBody('EndEffector');
setFixedTransform(end_effector.Joint, trvec2tform([0, link_lengths(7), link_lengths(8)]));
addBody(robot, end_effector, robot.BodyNames{end});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       VERIFYING THE TRANSFORMATION MATRIX          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

joint_angles = 1*[-0.0384   -1.0160    1.1247   -0.0058    0.9765]; % Change this to the desired initial joint angles
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
        0      Sines(2)     Cosines(2)      link_lengths(2);
        0            0              0       1
      ];

T23 = [ 1            0              0       0;
        0    Cosines(3)      -Sines(3)      link_lengths(3);
        0      Sines(3)     Cosines(3)      0;
        0            0              0       1
      ];

T34 = [ Cosines(4)   0       Sines(4)      0;
        0            1              0       link_lengths(4);
        -Sines(4)    0      Cosines(4)      0;
        0            0              0       1
      ];

T45 = [ 1            0              0       0;
        0    Cosines(5)      -Sines(5)      link_lengths(5);
        0      Sines(5)     Cosines(5)      0;
        0            0              0       1
      ];
  
T5ee = [ 1           0              0       0;
        0            1              0      link_lengths(6) + link_lengths(7);
        0            0              1       link_lengths(8);
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
%       INVERSE KINEMATICS           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
learning_rate = 0.1;
max_iterations = 10000;
tolerance = 1e-4;

Xdesired = [ 1 0 0 0.010;
             0 1 0 0.22;
             0 0 0 0.09;
             0 0 0 1
            ];
desired_position = Xdesired(1:3, 4);
desired_orientation = Xdesired(1:3, 1:3);

T_test = forward_kinematics(joint_angles, link_lengths);
disp('T_test:');
disp(T_test);
joint_angles_solution = inverse_kinematics_gradient_descent(desired_position, desired_orientation, joint_angles, link_lengths, @forward_kinematics, learning_rate, max_iterations, tolerance);
disp('Joint angles solution:');
disp(joint_angles_solution);



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
n_steps = 50; % Number of steps in the animation
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
plot3(end_effector_positions(:,1), end_effector_positions(:,2)+ link_lengths(7), end_effector_positions(:,3), 'b.-', 'MarkerSize', 10);
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



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Functions For IKIN          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function error = compute_pose_error(desired_position, desired_orientation, T)
    current_position = T(1:3, 4);
    current_orientation = T(1:3, 1:3);
%     position_error = sqrt((desired_position*desired_position - current_position*current_position));
    orientation_error = 0.5 * (cross(current_orientation(:, 1), desired_orientation(:, 1)) + ...
                               cross(current_orientation(:, 2), desired_orientation(:, 2)) + ...
                               cross(current_orientation(:, 3), desired_orientation(:, 3)));
%     error = [position_error; orientation_error];
    error = norm(current_position - desired_position);
end

function joint_angles = inverse_kinematics_gradient_descent(desired_position, desired_orientation, joint_angles, link_lengths, forward_kinematics_func, learning_rate, max_iterations, tolerance)
    for i = 1:max_iterations
        for j = 1:numel(joint_angles)
            %Compute the current orientation and position of the robot
            %endeffector
            T = forward_kinematics(joint_angles, link_lengths);
            error = compute_pose_error(desired_position, desired_orientation, T);
            %Break condition
            if (error) < tolerance
                break;
            end

            % Compute the gradient of the error with respect to joint angles
            % using an approximation
            angle_step = 0.1*pi/180;
            joint_angles(j) = joint_angles(j) + angle_step;
            %recompute the updated Endeffector position and position error
            T_step = forward_kinematics(joint_angles, link_lengths);
            error_step = compute_pose_error(desired_position, desired_orientation, T_step);
            
            gradient = ((error_step - error) / angle_step);
            
            %restore the origin joint angle
            joint_angles(j) = joint_angles(j) - angle_step;
            
            % Update joint angles using gradient descent but use the intial
            % guess as the basis for next guess
            if(j == 1)
                joint_angles(j) = joint_angles(j) - learning_rate * gradient;
            else
                joint_angles(j) = joint_angles(j) - learning_rate * gradient;
            end
        end
    end
end

function T = forward_kinematics(joint_angles, link_lengths)
    % Unpack the joint angles and link lengths
    q1 = joint_angles(1);
    q2 = joint_angles(2);
    q3 = joint_angles(3);
    q4 = joint_angles(4);
    q5 = joint_angles(5);

    b = link_lengths(1);
    l1 = link_lengths(2);
    l2 = link_lengths(3);
    l3 = link_lengths(4);
    l4 = link_lengths(5);
    l5 = link_lengths(6);
    l6 = link_lengths(7);
    l7 = link_lengths(8);

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

    T34 = [ cos(q4)  0  sin(q4)  0;
                  0  1        0  l3;
           -sin(q4)  0  cos(q4)  0;
                  0  0        0  1 ];

    T45 = [ 1  0        0         0;
            0  cos(q5) -sin(q5)  l4;
            0  sin(q5)  cos(q5)  0;
            0  0        0        1 ];

    T56 = [ 1  0  0  0;
            0  1  0  l5 + l6;
            0  0  1  l7;
            0  0  0  1 ];

    % Calculate the end-effector transformation matrix by multiplying the individual transformation matrices
    T = T01 * T12 * T23 * T34 * T45 * T56;
end

function Sym_T = TranslationMatrix_Symbolic
syms q1 q2 q3 q4 q5 b l1 l2 l3 l4 l5 l6 l7  %Robot joint angle and link lengths

%Declare the variables
Sym_Cosines = [ cos(q1), cos(q2), cos(q3), cos(q4), cos(q5)];
Sym_Sines = [ sin(q1), sin(q2), sin(q3), sin(q4), sin(q5)];
Sym_link_lengths = [ b, l1, l2, l3, l4, l5, l6, l7];

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

Sym_T34 = [ Sym_Cosines(4)   0       Sym_Sines(4)      0;
                0            1              0       Sym_link_lengths(4);
            -Sym_Sines(4)    0      Sym_Cosines(4)      0;
                0            0              0       1
           ];

Sym_T45 = [ 1            0              0       0;
            0    Sym_Cosines(5)      -Sym_Sines(5)      Sym_link_lengths(5);
            0      Sym_Sines(5)     Sym_Cosines(5)      0;
            0            0              0       1
          ];
  
Sym_T56 = [ 1            0              0       0;
            0            1              0      Sym_link_lengths(5) + Sym_link_lengths(6);
            0            0              1       Sym_link_lengths(7);
            0            0              0       1
          ];

Sym_T = Sym_T01 * Sym_T12 * Sym_T23 * Sym_T34 * Sym_T45 * Sym_T56;
disp('Tranfomration Matrix Symbolic:');
disp(Sym_T);
end

