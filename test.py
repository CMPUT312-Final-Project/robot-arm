# https://websockets.readthedocs.io/en/3.0/intro.html

import websockets
import asyncio
async def sendCartesian():
    async with websockets.connect('ws://172.31.56.16:8765') as websocket:
        while True:
            i = input("> ")
            await websocket.send(i)
            done = await websocket.recv()
            print(done)
if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(sendCartesian())