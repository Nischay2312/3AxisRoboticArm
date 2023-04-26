close all
clear all

% Import required packages
import robotics.RigidBodyTree;
import robotics.RigidBody;

%Creating rigid body elements
robot  = rigidBodyTree("DataFormat", "column");
base = robot.Base;

rotatingBase = rigidBody("rotating_Base");
arm1 = rigidBody("arm1");
arm2 = rigidBody("arm2");
arm3 = rigidBody("arm3");
arm4 = rigidBody("arm4");
endeffector = rigidBody("endeffector");

armWidth = 0.02;
armThickness = 0.01;
baseRadius = 0.05;
baseLength = 0.04;

arm1_l = 0.15;
arm2_l = 0.15;
arm3_l = 0.15;
arm4_l = 0.15;
endeffector_l = 0.15;

%Creating Collision elements
collBase = collisionCylinder(baseRadius, baseLength);
collBase.Pose = trvec2tform([0, 0, baseLength/2]);

coll1 = collisionBox(armThickness, arm1_l ,armWidth);
coll1.Pose = trvec2tform([0, arm1_l/2, 0]);

coll2 = collisionBox(armThickness, arm2_l ,armWidth);
coll2.Pose = trvec2tform([0, arm2_l/2, 0]);

coll3 = collisionBox(armThickness, arm3_l ,armWidth);
coll3.Pose = trvec2tform([0, arm3_l/2, 0]);

coll4 = collisionBox(armThickness, arm4_l ,armWidth);
coll4.Pose = trvec2tform([0, arm4_l/2, 0]);

collee = collisionBox(armThickness, endeffector_l ,armWidth);
collee.Pose = trvec2tform([0, endeffector_l/2, 0]);

addCollision(rotatingBase, collBase)
addCollision(arm1, coll1);
addCollision(arm2, coll2);
addCollision(arm3, coll3);
addCollision(arm4, coll4);
addCollision(endeffector, collee);

%attaching Joints
j_base = rigidBodyJoint("base_joint", "revolute");
j_1 = rigidBodyJoint("Joint1", "revolute");
j_2 = rigidBodyJoint("Joint2", "revolute");
j_3 = rigidBodyJoint("Joint3", "revolute");
j_4 = rigidBodyJoint("Joint4", "revolute");

j_1.JointAxis = [1 0 0];    %[x y z]
j_2.JointAxis = [1 0 0];
j_3.JointAxis = [0 1 0];
j_4.JointAxis = [1 0 0];

setFixedTransform(j_1, trvec2tform([0 0 baseRadius]));
setFixedTransform(j_2, trvec2tform([0 arm1_l 0]));
setFixedTransform(j_3, trvec2tform([0 arm2_l 0]));
setFixedTransform(j_4, trvec2tform([0 arm3_l 0]));
setFixedTransform(endeffector.Joint, trvec2tform([0 endeffector_l 0]));

%Assembling the robot
bodies = {base, rotatingBase, arm1, arm2, arm3, arm4};
joints = {[], j_base, j_1, j_2, j_3, j_4};

figure("Name","Assemble Robot","Visible","on")
for i = 2:length(bodies) % Skip base. Iterate through adding bodies and joints.
            bodies{i}.Joint = joints{i};
            addBody(robot,bodies{i},bodies{i-1}.Name)
end
%add endeffector
addBody(robot, endeffector, robot.BodyNames{end})
show(robot, "Collisions", "on", "Frames", "on");
% Input joint angles (in radians)
joint_angles = pi/180*[90, 60, -90, 90, 45]; % Change this to the desired joint angles

config = homeConfiguration(robot);
% config(3) = joint_angles(3);
% show(robot, "Collisions", "on", "Frames", "on");
gui = interactiveRigidBodyTree(robot,"MarkerScaleFactor",0.25);
