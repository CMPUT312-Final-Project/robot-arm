# https://websockets.readthedocs.io/en/3.0/intro.html

import asyncio
import websockets
import keyboard
import time

server_address = 'ws://172.31.83.56:8765'

async def setStartingValues():
	async with websockets.connect(server_address) as websocket:
		await websocket.send('setx 0')
		await websocket.recv()
		await websocket.send('sety 0')
		await websocket.recv()
		await websocket.send('setz 0')
		await websocket.recv()
		await websocket.send('setxr 0')
		await websocket.recv()
		await websocket.send('setyr 0')
		await websocket.recv()

async def getAndIncrease(label, inc):
	# print("Function call")
	async with websockets.connect(server_address) as websocket:
		send = 'get' + label
		# print("Sending: " + send)
		await websocket.send(send)
		value = await websocket.recv()
		# print("Recieved: " + value)
		value = float(value) + inc
		send = 'set' + label + " " + str(value)
		# print("Sending: " + send)
		await websocket.send(send)
		value = await websocket.recv()
		# print("Recieved: " + value)


def main():
	asyncio.get_event_loop().run_until_complete(setStartingValues())
	while True:
		try:
			if keyboard.is_pressed('w'):
				asyncio.get_event_loop().run_until_complete(getAndIncrease('x', 0.05))
				time.sleep(0.5)
				# break
			elif keyboard.is_pressed('s'):
				asyncio.get_event_loop().run_until_complete(getAndIncrease('x', -0.05))
				time.sleep(0.5)
				# break
			elif keyboard.is_pressed('a'):
				asyncio.get_event_loop().run_until_complete(getAndIncrease('y', 0.05))
				time.sleep(0.5)
				# break
			elif keyboard.is_pressed('d'):
				asyncio.get_event_loop().run_until_complete(getAndIncrease('y', -0.05))
				time.sleep(0.5)
				# break
			elif keyboard.is_pressed('q'):
				asyncio.get_event_loop().run_until_complete(getAndIncrease('z', 0.05))
				time.sleep(0.5)
				# break
			elif keyboard.is_pressed('e'):
				asyncio.get_event_loop().run_until_complete(getAndIncrease('z', -0.05))
				time.sleep(0.5)
				# break
		except:
			# break
			continue

if __name__ == '__main__':
	main()