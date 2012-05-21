#!/usr/bin/env python
import zmq
import sys
from ctypes import *


NUMLEGS = 6

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

class Hexapod(HexapodObject):
    
    def __init__(self):
        
        # zmq context server
        self.context = zmq.Context()
        #  Socket to talk to server
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect ("tcp://localhost:5555")
        
        # Setup Hexapod body parts
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
        
        
    def getParamString(self):

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

        return self.params.toString()

    def send(self):
        self.socket.send(self.send_string)
        message = self.socket.recv()
        
    def __repr__(self):
        return "Hexapod(%d)" % id(self)

    send_string = property(getParamString)

