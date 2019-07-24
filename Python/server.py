import socket
import time
import json

host, port = "127.0.0.1", 25002

try:
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect((host, port))

	x,y,z = 0,0,5 
	roll, pitch, yaw = 0,0,0
	ResetFlag = False

	while True:

		x = 0.0
		y = 0.0
		z -= 0.01

		roll += 0.0
		pitch += 0
		yaw += 0.5

		sentData = {}
		sentData['Position'] = [y, z, x]
		sentData['Rotation'] = [roll, yaw, pitch]
		sentData['Reset'] = ResetFlag

		serialized_sentData = json.dumps(sentData)
		sock.sendall(serialized_sentData.encode("utf-8"))
		print(sentData)

		serialized_dataRec = sock.recv(1024).decode("utf-8")
		dataRec = json.loads(serialized_dataRec)
		print(dataRec)

		if(dataRec['Collision'] == True):
			x,y,z = 0,0,5
			ResetFlag = True
		else:
			ResetFlag = False


		time.sleep(0.01)

finally:
	sock.close
