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


async def receive_images():
    async with websockets.connect('ws://192.168.1.6:8080') as websocket:
        while True:
            # Receive a binary image from the WebSocket server
            img_bytes = await websocket.recv()

            # Convert the binary image to a numpy array
            with io.BytesIO(img_bytes) as buf:
                img = Image.open(buf)
                img_np = np.array(img)
                img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

            #return img_np
            yield img_np

cv2.namedWindow('ESP32-CAM', cv2.WINDOW_NORMAL)
cv2.resizeWindow('ESP32-CAM', 640, 480)


async def Display_images():
    async for img in receive_images():
        cv2.imshow('ESP32-CAM', img)
        cv2.waitKey(1)

# Start the event loop to receive images
asyncio.get_event_loop().run_until_complete(Display_images())

#  Print numbers every 1 second
import time
i=1
while i < 20:
    print(i)
    time.sleep(1)
    i+=1

