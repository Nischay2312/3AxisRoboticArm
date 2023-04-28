"""
    This Script is the main vision script for the 3Axis Robot Arm.
    There are two main things happening here:
    1. Read and process .jpg images coming from a websocket server
    2. Send the Processed Data to the Robot Screen through serial. This happens once every X Frames. 
"""
"""
    This script is used to view the ESP32-CAM feed in a window.
    Nischay Joshi, 10-04-2023
"""
import asyncio
import websockets
from PIL import Image
import io
import cv2
import numpy as np
from controller import Controller
import helperfunctions
import time
import serial

# This global variable will store the center of the detected face
detected_face_center = None

async def receive_images():
    async with websockets.connect('ws://192.168.4.1:8080') as websocket:
        while True:
            # Receive a binary imagews://192.168.4.1:8080 from the WebSocket server
            img_bytes = await websocket.recv()

            # Convert the binary image to a numpy array
            with io.BytesIO(img_bytes) as buf:
                img = Image.open(buf)
                img_np = np.array(img)
                img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

            #return img_np
            yield img_np

async def Process_image():
    global largest_face, detected_face_center
    async for img in receive_images():
        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect faces in the grayscale image
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        
        #find the largest face
        largest_face_cords = 0
        largest_face_area = 0
        # if there are any faces 
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                area = w*h
                if area > largest_face_area:
                    largest_face_cords = (x, y, w, h)
                    largest_face_area = area
            # draw a rectangle around the largest face
            (x, y, w, h) = largest_face_cords
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        # Update the largest face globalvariable
        largest_face = largest_face_cords
        if largest_face == 0:
            detected_face_center = None
        else:
            detected_face_center = (x + w/2, y + h/2)
        # # Display the image using OpenCV
        cv2.imshow('image', img)
        cv2.waitKey(1)

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

async def Send_data():
    while True:  
        # If we read something from the serial port
        if RobotEye.in_waiting > 0:
            while RobotEye.in_waiting > 0:
                # Read the data
                DataFromRobotArm = RobotEye.readline().decode().strip()
                # Print the data
                print("Data From Robot Arm: ", DataFromRobotArm)
                
        # If largest_face is 0, then no face was detected
        if largest_face == 0:
            print("No Face Detected")
            # Sets the center to centre of the screen
            center_x = ImageWidth/2
            center_y = ImageHeight/2
        else:
            # Get the center of the largest face
            (x, y, w, h) = largest_face
            center_x = x + w/2
            center_y = y + h/2
            print("Center of Face: ", center_x, center_y)
        
            # Map the center to the range 0-255, with 0,0 is 130, 130
            center_x = map_range(center_x, 0, ImageWidth, 255, 0)
            center_y = map_range(center_y, 0, ImageHeight, 0, 255)
            # Send the data to the Robot Eyes
            OutputString = Header + str(center_x) + "," + str(center_y) + "\n"
            RobotEye.write(bytes(OutputString, 'utf-8'))
        await asyncio.sleep(1/DataSendRate)

async def ControlArm():
    RUNRATE =  25#Hz
    DEBUG = True
    DEBUGCONTROLLER = False

    # serial port settings
    port = "COM13"
    baudrate = 115200
    bytesize = serial.EIGHTBITS
    parity = serial.PARITY_NONE
    stopbits = serial.STOPBITS_ONE

    # open the serial port
    RobotArmSerial = serial.Serial(port, baudrate, bytesize, parity, stopbits, timeout=0.1)
    await asyncio.sleep(2)
    print("Serial port", port, "opened")

    InputsWanted = ["LXaxis","LYaxis", "L2", "R2", "L1", "R1", "Triangle", "Circle", "Square", "RXaxis", "RYaxis"]
    DataToSend = {"Header": "FF"}
    OutputString = ""

    #Instantiate a controller object
    PS5Controller = Controller()

    #read the Controller inputs 
    ControllerState = PS5Controller.GetControllerState()

    #convert the dictionaly values to string but only for the inputs we want
    for key in InputsWanted:
        if key in ControllerState:
            DataToSend[key] = str(ControllerState[key])

    #now prepare the output String
    for key in DataToSend:
        OutputString += DataToSend[key] + ","
    OutputString = OutputString[:-1] + "\n"

    #Wait untill we get an "FF" from the Robot Arm
    while True:
        if RobotArmSerial.in_waiting > 0:
            DataFromRobotArm = RobotArmSerial.readline().decode().strip()
            if DataFromRobotArm == "FF":
                print("Robot Arm Ready")
                #send the PreRead data
                RobotArmSerial.write(bytes(OutputString, 'utf-8'))
                break

    #The connection is established, now periodically send the contorller data to the robot arm
    while True:
        #quit the program on Keyboad Interrupt
        try:
            #Check if we read something from the Robot Arm
            if RobotArmSerial.in_waiting > 0:
                while RobotArmSerial.in_waiting > 0:
                    #Read the data
                    DataFromRobotArm = RobotArmSerial.readline().decode().strip()
                    #Print the data
                    if DEBUG:
                        print("Data From Robot Arm: ", DataFromRobotArm)

            #Read the controller
            ControllerState = PS5Controller.GetControllerState()
            # If a face is detected, set the LXaxis value to the detected face's center
            if detected_face_center is not None:
                # Use the x-coordinate of the detected face's center
                # You can adjust the mapping range as needed
                face_center_x = int(map_range(detected_face_center[0], 0, ImageWidth, 20, 235))
                face_center_y = int(map_range(detected_face_center[1], 0, ImageHeight, 0, 255))
                y1 = 0 #to go down
                y2 = 0 #to go up
                if face_center_y > 255/2: #go down
                    y1 = int(map_range(face_center_y, 255/2, 255, 0, 150))
                else:
                    y2 = int(map_range(face_center_y, 0, 255/2, 150, 0))
                ControllerState["LXaxis"] = face_center_x
                ControllerState["L2"] = y1
                ControllerState["R2"] = y2
                ControllerState["RXaxis"] = 0
                ControllerState["RYaxis"] = 0
            #Print the Controller Inputs
            if DEBUGCONTROLLER:
                print(ControllerState)
            #convert the dictionaly values to string but only for the inputs we want
            for key in InputsWanted:
                if key in ControllerState:
                    DataToSend[key] = str(ControllerState[key])
    
            #now prepare the output String
            for key in DataToSend:
                OutputString += DataToSend[key] + ","
            OutputString = OutputString[:-1] + "\n"

            if DEBUG:
                print(OutputString)
        
            # Send the Controller Inputs to the Robot Arm
            RobotArmSerial.write(bytes(OutputString, 'utf-8'))

            # Clear the output string
            OutputString = ""
            await asyncio.sleep(1/RUNRATE)

            #clear the screen
            helperfunctions.ClearScreen()

        except KeyboardInterrupt:
            print("Keyboard Interrupt, Program Terminated")
            break


# SCRIPT STARTS HERE

# IMAGE PROCESSING SETUP
ImageWidth = 640
ImageHeight = 480
cv2.namedWindow('ESP32-CAM', cv2.WINDOW_NORMAL)
cv2.resizeWindow('ESP32-CAM', ImageWidth, ImageHeight)
#this global variable will store the latest largest face
largest_face = 0
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
if face_cascade.empty():
    raise Exception('Error loading face cascade classifier')

# SERIAL COMMUNICATION SETUP
port = "COM20"
baudrate = 115200
bytesize = serial.EIGHTBITS
parity = serial.PARITY_NONE
stopbits = serial.STOPBITS_ONE
DataSendRate = 15 # hz
Header = "FF,"
OutputString = ""
# open the serial port
RobotEye = serial.Serial(port, baudrate, bytesize, parity, stopbits, timeout=0.1)
time.sleep(2)
print("Serial port", port, "opened")


# PROCESS SETUP
loop = asyncio.get_event_loop()
loop.create_task(Process_image())
loop.create_task(Send_data())
loop.create_task(ControlArm())
while True:
    # if KeyboardInterrupt:
    #     break
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        print("KeyBoard Interrupt")
        break