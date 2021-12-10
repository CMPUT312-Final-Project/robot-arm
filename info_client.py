# https://websockets.readthedocs.io/en/3.0/intro.html

import asyncio
import websockets

from robot_kinematics import CartesianCoordinates

async def sendCartesian(cartesianCoordinates: CartesianCoordinates, xr: float, yr: float):
    '''
        Send cartesian coordinates to the server along with rotation values.
    '''
    async with websockets.connect('ws://172.31.73.50:8765') as websocket:
        x = f"setx {cartesianCoordinates.y}"
        y = f"sety {cartesianCoordinates.z}"
        z = f"setz {cartesianCoordinates.x}"
        xr = f"setxr {xr}"
        yr = f"setyr {yr}"
        
        
        for i in [x, y, z, xr, yr]:
            await websocket.send(i)
            # done = await websocket.recv()
            # print("< {}".format(done))
        
