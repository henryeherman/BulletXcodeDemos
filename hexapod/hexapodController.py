#
#   Hexapod Controller 
#   Connects REQ socket to tcp://localhost:5555
#
import zmq
import msgpack

context = zmq.Context()

#  Socket to talk to server
socket = context.socket(zmq.REQ)
socket.connect ("tcp://localhost:5555")

#  Do 10 requests, waiting each time for a response
# for request in range (1,10):

while True:

    print "Sending request ", request,"…"

    packer = msgpack.Packer()
    serialized = packer.pack()


    socket.send ("Hello")
    
    #  Get the reply.
    message = socket.recv()
    print "Received reply ", request, "[", message, "]"