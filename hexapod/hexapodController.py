#
#   Hexapod Controller
#   Connects REQ socket to tcp://localhost:5555
#
import zmq
import msgpack
from ctypes import *



#  Do 10 requests, waiting each time for a response
# for request in range (1,10):

# typedef struct HpodCtrlParams {
#     vector<btScalar> kneeAngles;
#     vector<btScalar> hipAnglesX;
#     vector<btScalar> hipAnglesY;
#     btScalar hipStrength;
#     btScalar kneeStrength;
#     btScalar dtKnee;
#     btScalar dtHip;

#     MSGPACK_DEFINE(kneeAngles, hipAnglesX, hipAnglesY, hipStrength, kneeStrength, dtKnee, dtHip);

# } HypodCtrlParams;
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

		# self.kneeAngles = []
		# self.hipAnglesX = []
		# self.hipAnglesY = []
		# self.hipStrength = 0
		# self.kneeStrength = 0
		# self.dtKnee = 0
		# self.dtHip = 0


def format_string(params):
	send_list = []
	send_list.append("setControlParams")
	# send_list.append("kneeAngles")
	send_list.extend(params.kneeAngles)
	# send_list.append("hipAnglesX")
	send_list.extend(params.hipAnglesX)
	# send_list.append("hipAnglesY")
	send_list.extend(params.hipAnglesY)
	# send_list.append("hipStrength")
	send_list.append(params.hipStrength)
	# send_list.append("kneeStrength")
	send_list.append(params.kneeStrength)
	# send_list.append("dtKnee")
	send_list.append(params.dtKnee)
	# send_list.append("dtHip")
	send_list.append(params.dtHip)
	send_string = " ".join(map(str,send_list))
	return send_string


def main():
	context = zmq.Context()

	#  Socket to talk to server
	socket = context.socket(zmq.REQ)
	socket.connect ("tcp://localhost:5555")
	params = HpodCtrlParams()


	# for angle in range(3):
	# 	params.kneeAngles[angle] = 1.0
	# 	params.hipAnglesX[angle] = 1.0
	# 	params.hipAnglesY[angle] = 1.0
	params.kneeAngles[0] = 1
	params.kneeAngles[1] = 4.4
	params.hipAnglesX[0] = 1
	params.hipAnglesY[0] = 1
	# params.hipAnglesX.append(1.0)
	# params.hipAnglesY.append(1.0)

	params.hipStrength = 10
	params.kneeStrength = 1
	params.dtKnee = 1
	params.dtHip = 1

	send_string = params.toString()
	# send_string = format_string(params)

	socket.send(send_string)

	message = socket.recv()
	print "Received reply: " + message


	# while True:

	#     print "Sending request "


	#     socket.send ("Hello")

	#     #  Get the reply.
	#     message = socket.recv()
	#     print "Received reply ", request, "[", message, "]"

if __name__ == "__main__":
    main()
