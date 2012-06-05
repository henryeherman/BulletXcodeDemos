#!/usr/bin/env python
import copy
import zmq
import sys
from ctypes import *
import time
from StringIO import StringIO

NUMLEGS = 6

class HpodReplies(list):

    def __init__(self, arrbuf):
        replies = [HpodSimStep(HpodReply.from_buffer_copy(buf))
                for buf in arrbuf]
        list.__init__(self,replies)

    def getzpos(self):
        return [r.zpos for r in self]

    zpos = property(getzpos)

    def getxpos(self):
        return [r.xpos for r in self]

    xpos = property(getxpos)

    def getypos(self):
        return [r.ypos for r in self]

    ypos = property(getypos)


class HpodSimStep(object):

    def __init__(self, reply):
        self.reply = reply

    def getxpos(self):
        return float(self.reply.xpos)

    def getypos(self):
        return float(self.reply.ypos)

    def getzpos(self):
        return float(self.reply.zpos)

    def getlowerlegforces(self):
        return [float(f) for f in self.reply.lowerlegforce]

    def getupperlegforces(self):
        return [float(f) for f in self.reply.upperlegforce]

    xpos = property(getxpos)
    ypos = property(getypos)
    zpos = property(getzpos)
    upperlegforce = property(getupperlegforces)
    lowerlegforce = property(getlowerlegforces)


class HpodSimCtrlParam(c_uint):
    PAUSE=0
    RESET=1
    CONTINUE=2
    START=3
    LOAD=4
    LOADIMM=5
    RUNEXP=6
    RESETEXP=7
    CHKBUSY=8
    GETREPLY=9
    OPTS = (PAUSE, RESET,
            CONTINUE, START,
            LOAD,LOADIMM,
            RUNEXP, RESETEXP,CHKBUSY,
            GETREPLY)

class HpodReply(Structure):

    _fields_ = [("podid", c_uint),
                ("xpos", c_float),
                ("ypos", c_float),
                ("zpos", c_float),
                ("upperlegforce", c_float*NUMLEGS),
                ("lowerlegforce", c_float*NUMLEGS)]


class HpodCtrlParams(Structure):

    _fields_ = [("kneeAngles", c_float*NUMLEGS),
		("hipAnglesX", c_float*NUMLEGS),
		("hipAnglesY", c_float*NUMLEGS),
		("hipStrength", c_float),
		("kneeStrength", c_float),
		("dtKnee", c_float),
		("dtHip", c_float)]


    def toString(self):
        return buffer(self)[:]




class HexapodException(Exception):
    pass

class HexapodObject(object):
    pass

class Joint(object):
    pass

class Knee(Joint):
    def __init__(self,angle=0):
        self.angle = angle

    def _set_angle(self,angle):
        self._angle = angle

    def _get_angle(self):
        return self._angle
    angle = property(_get_angle, _set_angle)

class Hip(Joint):
    def __init__(self, angles = [0,0]):
        self.angles = angles

    def set_angles(self, xangle, yangle):
        self.xangle = 0
        self.yangle = 0

    def _get_angles(self):
        return [self.xangle, self.yangle]

    def _set_angles(self, angles):
        self.xangle = angles[0]
        self.yangle = angles[1]

    angles = property(_get_angles, _set_angles)


class BodyPart(HexapodObject):
    pass

class Leg(BodyPart):
    FRONTLEFT = "FRONT LEFT"
    FRONTRIGHT = "FRONT RIGHT"
    CENTERLEFT = "CENTER LEFT"
    CENTERRIGHT = "CENTER RIGHT"
    REARLEFT = "REAR LEFT"
    REARRIGHT = "REAR RIGHT"
    POS = (FRONTLEFT,FRONTRIGHT,
            CENTERLEFT,CENTERRIGHT,
            REARLEFT,REARRIGHT)

    def __init__(self, pos):
        self.knee = Knee()
        self.hip = Hip()
        if pos in Leg.POS:
            self.pos = pos
        else:
            raise HexapodException("Invalid Leg Choice")
    def __repr__(self):
        return "Leg('%s')" % self.pos

class Thorax(BodyPart):
    def __init__(self):
        pass
    def __repr__(self):
        return "Thorax(%d)" % id(self)

class HexapodBody:
    def __init__(self):
        self.thorax = Thorax()
        self.leftFrontLeg = Leg(Leg.FRONTLEFT)
        self.rightFrontLeg = Leg(Leg.FRONTRIGHT)
        self.leftCenterLeg = Leg(Leg.CENTERLEFT)
        self.rightCenterLeg = Leg(Leg.CENTERRIGHT)
        self.leftRearLeg = Leg(Leg.REARLEFT)
        self.rightRearLeg = Leg(Leg.REARRIGHT)

        self.legs = ( self.leftFrontLeg,
                    self.rightFrontLeg,
                    self.leftCenterLeg,
                    self.rightCenterLeg,
                    self.leftRearLeg,
                    self.rightRearLeg)

        self.kneeStrength = 0
        self.hipStrength = 0
        self.dtHip = 0
        self.dtKnee = 0
        self.params = HpodCtrlParams()

class Hexapod(HexapodObject, HexapodBody):

    def __init__(self):

        # zmq context server
        self.context = zmq.Context()
        #  Socket to talk to server
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect ("tcp://localhost:5555")

        # Setup Hexapod body parts
        HexapodBody.__init__(self)

        self._paramsArray = []
        self.setControl()

    def setControl(self, ctrl=None):
        if ctrl in HpodSimCtrlParam.OPTS:
            self.msg = [buffer(HpodSimCtrlParam(ctrl))[:]]
            return
        self.msg = [buffer(HpodSimCtrlParam(HpodSimCtrlParam.CONTINUE))[:]]


    def loadParam(self):

        self.params.kneeAngles[0] = self.legs[0].knee.angle
        self.params.kneeAngles[1] = self.legs[1].knee.angle
        self.params.kneeAngles[2] = self.legs[2].knee.angle
        self.params.kneeAngles[3] = self.legs[3].knee.angle
        self.params.kneeAngles[4] = self.legs[4].knee.angle
        self.params.kneeAngles[5] = self.legs[5].knee.angle

        self.params.hipAnglesX[0] = self.legs[0].hip.xangle
        self.params.hipAnglesX[1] = self.legs[1].hip.xangle
        self.params.hipAnglesX[2] = self.legs[2].hip.xangle
        self.params.hipAnglesX[3] = self.legs[3].hip.xangle
        self.params.hipAnglesX[4] = self.legs[4].hip.xangle
        self.params.hipAnglesX[5] = self.legs[5].hip.xangle


        self.params.hipAnglesY[0] = self.legs[0].hip.yangle
        self.params.hipAnglesY[1] = self.legs[1].hip.yangle
        self.params.hipAnglesY[2] = self.legs[2].hip.yangle
        self.params.hipAnglesY[3] = self.legs[3].hip.yangle
        self.params.hipAnglesY[4] = self.legs[4].hip.yangle
        self.params.hipAnglesY[5] = self.legs[5].hip.yangle

        self.params.kneeStrength = self.kneeStrength
        self.params.hipStrength = self.hipStrength

        self.params.dtKnee = self.dtKnee
        self.params.dtHip = self.dtHip

    def clearParamArray(self):
        self._paramsArray =[]

    def getParamString(self):
        self.loadParam()
        return self.params.toString()

    def send(self):
        self.msg.append(self.sendString)
        self.socket.send_multipart(self.msg)
        message = self.socket.recv()
        #print "Received: %s" % str(message)
        self.msg = []
        return message

    def sendArray(self):
        self.msg.append(self.sendStringArray)
        self.socket.send_multipart(self.msg)
        message = self.socket.recv()
        #print "Received: %s" % str(message)
        self.msg = []
        return message

    def getReply(self):
        print "Calling get reply: %d" % HpodSimCtrlParam.GETREPLY
        self.clearParamArray()
        self.setControl( HpodSimCtrlParam.GETREPLY )
        self.socket.send_multipart(self.msg)
        message = self.socket.recv_multipart()
        self.msg = []
        return message

    def checkIsBusy(self):
        time.sleep(0.1)
        self.clearParamArray()
        self.setControl(HpodSimCtrlParam.CHKBUSY)
        s = self.send()
        s = s[:-1]
        if s=="NO":
            return False
        else:
            return True

    def startexp(self):
        self.clearParamArray()
        self.setControl(HpodSimCtrlParam.RUNEXP)
        self.send()
        self.clearParamArray()

    def runexp(self):
        sys.stdout.write("Send %d positions\r\n" % len(self.paramsArray))
        self.startexp()
        while self.checkIsBusy():
            sys.stdout.write("Waiting...\r")
        msg = self.getReply()
        sys.stdout.write("Received %d replies\r\n" % len(msg))
        return HpodReplies(msg)

    def readyLoad(self):
        self.setControl(HpodSimCtrlParam.LOAD)

    def load(self):
        self.readyLoad()
        self.sendArray()

    def resetExp(self):
        self.clearParamArray()
        self.setControl(HpodSimCtrlParam.RESET)
        self.send()

    def cont(self):
        self.clearParamArray()
        self.setControl(HpodSimCtrlParam.CONTINUE)
        self.send()

    def addParam(self):
        self.loadParam()
        tempParam = copy.copy(self.params)
        self._paramsArray.append(tempParam)

    def _getParamsArray(self):
        self.m_HpodCtrlParams = HpodCtrlParams*len(self._paramsArray)
        return self.m_HpodCtrlParams(*self._paramsArray)

    def _getParamsArrayString(self):
        return buffer(self.paramsArray)[:]

    paramsArray = property(_getParamsArray)
    sendStringArray = property(_getParamsArrayString)

    def __repr__(self):
        return "Hexapod(id='%d')" % id(self)

    sendString = property(getParamString)


