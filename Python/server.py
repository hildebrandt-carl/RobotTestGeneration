import socket
import time
import json

host, port = "127.0.0.1", 25001

try:
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect((host, port))

	x,y,z = 0,0,0 
	roll, pitch, yaw = 0,0,0

	while True:

		x = -50.0
		y = 25.0
		z = 24.0

		roll += 0.1
		pitch = 0.0
		yaw = 0.0

		data = {}
		data['ID'] = 2
		data['Name'] = "ABC"
		data['Position'] = [y, z, x]
		data['Rotation'] = [roll, yaw, pitch]

		serialized_data = json.dumps(data)

		sock.sendall(serialized_data.encode("utf-8"))
		#data = sock.recv(1024).decode("utf-8")
		print(data)
		time.sleep(0.01)

finally:
	sock.close
