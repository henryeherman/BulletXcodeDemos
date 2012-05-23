#/usr/bin/env python

import time
import hexapod
from hexapod import HpodSimCtrlParam
import numpy as np
import sys

pod = hexapod.Hexapod()

pod.kneeStrength=4
pod.hipStrength=4
pod.dtKnee = 1.0
pod.dtHip = 1.0

t = np.arange(0,10000,0.01)
w = 1
xs = abs(np.sin(2*np.pi*t/2))

time.sleep(2)

pod.setControl(HpodSimCtrlParam.RESETEXP)
pod.addParam()
pod.sendArray()
pod.clearParamArray()

time.sleep(2)

pod.setControl( HpodSimCtrlParam.START)
pod.addParam()
pod.sendArray()
pod.clearParamArray()
time.sleep(2)

pod.setControl( HpodSimCtrlParam.CONTINUE)
pod.addParam()
pod.sendArray()
pod.clearParamArray()
time.sleep(2)

pod.setControl( HpodSimCtrlParam.LOADIMM)
pod.legs[0].knee.angle=2
pod.addParam()
pod.sendArray()
pod.clearParamArray()
time.sleep(2)

pod.setControl( HpodSimCtrlParam.LOAD)
pod.legs[0].knee.angle=0
pod.addParam()
pod.legs[1].hip.xangle=1
pod.addParam()
pod.legs[0].hip.yangle=1
pod.addParam()
pod.sendArray()
pod.clearParamArray()
time.sleep(2)

pod.setControl( HpodSimCtrlParam.RUNEXP)
pod.addParam()
pod.sendArray()
pod.clearParamArray()
time.sleep(5)

pod.setControl( HpodSimCtrlParam.PAUSE)
pod.addParam()
pod.sendArray()
pod.clearParamArray()
time.sleep(5)


pod.setControl( HpodSimCtrlParam.CONTINUE)
pod.addParam()
pod.sendArray()
pod.clearParamArray()

print "Complete"
