#
#   Hexapod Controller
#   Connects REQ socket to tcp://localhost:5555
#
import zmq
import msgpack
from ctypes import *

NUMLEGS = 6

class HpodCtrlParams(Structure):

	_fields_ = [("kneeAngles", c_float*NUMLEGS),
				("hipAnglesX", c_float*NUMLEGS),
				("hipAnglesY", c_float*NUMLEGS),
				("hipStrength", c_float),
				("kneeStrength", c_float),
				("dtKnee", c_float),
				("dtHip", c_float)
				]

	def toString(self):
		return buffer(self)[:]

def main():
	context = zmq.Context()

	#  Socket to talk to server
	socket = context.socket(zmq.REQ)
	socket.connect ("tcp://localhost:5555")
	params = HpodCtrlParams()

	for i in range(6):
		params.kneeAngles[i] = 0
		params.hipAnglesY[i] = -1
	params.hipAnglesX[0] = 2
	params.hipAnglesX[1] = 0
	params.hipAnglesX[2] = 2
	params.hipAnglesX[3] = 0
	params.hipAnglesX[4] = 2
	params.hipAnglesX[5] = 0


	params.hipStrength = 40
	params.kneeStrength = 40
	params.dtKnee = 4
	params.dtHip = 4

	send_string = params.toString()
	# send_string = format_string(params)

	socket.send(send_string)

	message = socket.recv()
	print "Received reply: " + message

if __name__ == "__main__":
    main()
