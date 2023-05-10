% This script converts the joint angles from the other script into the format that the robot needs to move

% read the variable called JointArray.

%Then we modify each joint column like so:

% column 1 corresponding to joint 1: Add 95 to all values
% column 2 corresponding to joint 2: substract values from 145
% column 3 corresponding to joint 3: Add 140 to all values
% column 4 corresponding to joint 4: substract values from 100
% column 5 corresponding to joint 5: Add 82 to all values
% column 6 corresponding to joint 6: Add 83 from all values

%convert the to integers

%then we write the new array to a file called JointAngles.csv

JointArray2Export = 180/pi*JointArray;

JointArray2Export(:,1) = JointArray2Export(:,1) + 95;
JointArray2Export(:,2) = 145 - JointArray2Export(:,2);
JointArray2Export(:,3) = JointArray2Export(:,3) + 140;
JointArray2Export(:,4) = 100 - JointArray2Export(:,4);
JointArray2Export(:,5) = JointArray2Export(:,5) + 82;
JointArray2Export(:,6) = JointArray2Export(:,6) + 83;

JointArray2Export = int16(JointArray2Export);

csvwrite('JointAngles.csv',JointArray2Export);

%plot the joint angles
figure(2)
plot(JointArray2Export)
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6')
xlabel('Time (s)')
ylabel('Joint Angle (deg)')



