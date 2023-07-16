# RoboticArm
Repository for the Robotic Arm Controllers

# So far I have worked on three different versions
3 Axis <br>
5 Axis <br>
6 Axis <br>

I refered the 3d models from previously made robotic arms or off the shelf arm kits (I am not a Mechanincal Designer but would like to learn it someday :) )
<br><br>
My major work was developing the electrical + software system used to control these arm, to "give life to them", if you may. <br>
I started developing manual control system using an arduino mega as a Servo controller which takes in Serial input from my computer. It then controls the servos.
<br><bR>
I used Python to read a PS5 Dual sense contorller and mapped its inputs to different axes of the arm. Then it would send the data from the contrller to the Arduino using Serial port.
<br> 
<br>
I used similar techineques to controll all three diferent arms. However I realized that using a controller is a bit harder especially trying to visualize the movement of endeffector by changing the joint angles.
<br><br>
Thus I started looking into Inverse Kinematics and how to solve then for larger DOF arms. I started designing and experimenting with IK Solvers in MATLAB by simulating a model of my robot.
I learnt a lot about Jacobians, Tranformaton Matrices, Rotation Matirices, Euler Angles and solving techniques such as Gradient Descent and Pseudo Inverse Method.
<br><br>
After I was happy with the simualtion, I would then export the planned path in Matlab to CSV format for my python script to read and send it to the arm. I can then see the arm following the planned path.
<br><br>
I also designed an animation engine for Cartoonistic Robot eyes (inspired form Eve in WallE). This was running on an ESP 32 and would add a "cute" personality to the arm. 
<br><br>
Apart from that, I tried to intergrate visual feedback into the arm using an ESP cam (data sent using WiFi (websockets)) and used face detection on python to make the arm follow a person in the frame. There are many more possiblities which I would like to explore futher with this system.
<br><br>
Finally I would like to put some videos (which will be put up here evnetually sorry for delay!)

