# https://websockets.readthedocs.io/en/3.0/intro.html

import asyncio
import websockets

from robot_kinematics import CartesianCoordinates

async def sendCartesian(cartesianCoordinates: CartesianCoordinates):
    async with websockets.connect('ws://172.31.73.76:8765') as websocket:
        x = f"setx {cartesianCoordinates.y}"
        y = f"sety {cartesianCoordinates.x}"
        z = f"setz {cartesianCoordinates.z}"
        
        for i in [x, y, z]:
            await websocket.send(i)
            done = await websocket.recv()
            print("< {}".format(done))
        
