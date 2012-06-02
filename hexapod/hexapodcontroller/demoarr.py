#/usr/bin/env python

import time
import hexapod
from hexapod import HpodSimCtrlParam
import numpy as np
import sys

pod = hexapod.Hexapod()

pod.kneeStrength=4
pod.hipStrength=4
pod.dtKnee = 0.1
pod.dtHip = 0.1

t = np.arange(0,5,0.01)
w = 1
xs = abs(np.sin(2*np.pi*t/5))
pod.setControl(HpodSimCtrlParam.RESETEXP)
pod.send()
pod.setControl(HpodSimCtrlParam.CONTINUE)
pod.send()

pod.setControl(HpodSimCtrlParam.LOAD)


print "Begin Exp"
try:
    print "Build Array"
    for x in xs[::1]: 
        for leg in pod.legs:
            leg.knee.angle=x*2
            leg.hip.yangle=2*x-1
            leg.hip.xangle=2*x-1
        pod.addParam()
        print "Add Param %f" % x
    print "Array Built... sending"
    pod.sendArray()
    pod.clearParamArray()
    pod.setControl(HpodSimCtrlParam.RUNEXP)
    pod.send()
    pod.clearParamArray()
    #time.sleep(10)
    s = ""
    time.sleep(1)
    while s!="NO":
        pod.setControl(HpodSimCtrlParam.CHKBUSY)
        s = pod.send()
        s = s[:-1]  
        time.sleep(1)
        print "RECV: %s" % s
    print "Sent %d packets" % len(xs)

except (KeyboardInterrupt,):

    print "Exit Hexapod Client"
    sys.exit(0)
