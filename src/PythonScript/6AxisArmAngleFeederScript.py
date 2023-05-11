#this script read the joint angles from the csv file and send it to the robot arm via serial output.
#threr are 7 output channels in total, 6 for the 6 joints and 1 for the gripper, the csv file contain the angles for each joint in each row.
#the gripper is always set to 50.
#the serial output format is ANG, angle1, angle2, angle3, angle4, angle5, angle6, gripper; ANG is the header


import csv
import time
import serial

DATASENDRATE = 10  # Hz
DEBUG = True

# CSV file storing angle location
angleFile = r"C:\Users\nisch\OneDrive - UBC\UBC_UAS\2023\RoboticArm\src\MatlabSimulation\JointAngles.csv"

# Serial port settings
port = "COM13"
baudrate = 115200
bytesize = serial.EIGHTBITS
parity = serial.PARITY_NONE
stopbits = serial.STOPBITS_ONE

# Open the serial port
RobotArmSerial = serial.Serial(port, baudrate, bytesize, parity, stopbits, timeout=0.1)
time.sleep(2)
print("Serial port", port, "opened")


# Function to read the angles from the CSV file
def read_csv(file_path):
    angle_list = []
    with open(file_path, "r") as file:
        reader = csv.reader(file)
        for row in reader:
            angle_list.append([float(angle) for angle in row])
    return angle_list


# Read the angles from the CSV file
angleList = read_csv(angleFile)

# Start sending the angles
while True:
    #send every 5th row
    for angle in range(0, len(angleList), 8):
        # Prepare the output string
        output_string = f"ANG,{angleList[angle][0]},{angleList[angle][1]},{angleList[angle][2]},{angleList[angle][3]},{angleList[angle][4]},{angleList[angle][5]},50, END\n"
        # output_string = f"ANG,{angle[0]},{angle[1]},{angle[2]},{angle[3]},{angle[4]},{angle[5]},50, END\n"
        RobotArmSerial.write(bytes(output_string, 'utf-8'))

        # wait until we read the data from the robot arm
        while RobotArmSerial.in_waiting == 0:
            pass

        if RobotArmSerial.in_waiting > 0:
            while RobotArmSerial.in_waiting > 0:
                # Read the data
                data_from_robot_arm = RobotArmSerial.readline().decode().strip()
                # Print the data
                if DEBUG:
                    print("Data From Robot Arm: ", data_from_robot_arm)
        # time.sleep(1 / DATASENDRATE)
