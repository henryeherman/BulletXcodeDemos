#!/usr/bin/env python

import zmq
from ctypes import *

class HexapodException(Exception):
    pass

class HexapodObject(object):
    pass

class Joint(object):
    pass

class Knee(Joint):
    def __init__(self,angle):
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
    FRONTLEFT = 0
    FRONTRIGHT = 1
    CENTERLEFT = 2
    CENTERRIGHT = 3
    REARLEFT = 4
    REARRIGHT = 5
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

class Thorax(BodyPart):
    def __init__(self):
        pass

class Hexapod(HexapodObject):
    
    def __init__(self):
        
        # zmq context server
        context = zmq.Context()
        #  Socket to talk to server
        socket = context.socket(zmq.REQ)
        socket.connect ("tcp://localhost:5555")
        
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

