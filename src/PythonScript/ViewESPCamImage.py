import asyncio
import websockets
from PIL import Image, ImageTk
import tkinter as tk
import io

async def receive_images():
    async with websockets.connect('ws://192.168.1.6:8080') as websocket:
        root = tk.Tk()
        root.title('Image Viewer')
        canvas = tk.Canvas(root, width=800, height=600)
        canvas.pack()

        while True:
            # Receive a binary image from the WebSocket server
            img_bytes = await websocket.recv()

            # Convert the binary image to a PIL image
            img = Image.open(io.BytesIO(img_bytes))

            # Convert the PIL image to a Tkinter-compatible image object
            photo = ImageTk.PhotoImage(img)

            # Update the canvas with the new image
            canvas.create_image(0, 0, anchor='nw', image=photo)
            canvas.photo = photo

            # Force an update of the canvas to display the new image
            canvas.update()

        root.mainloop()

# Start the event loop to receive images
asyncio.get_event_loop().run_until_complete(receive_images())
