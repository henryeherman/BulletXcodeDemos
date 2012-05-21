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

for x in xs[::1]: 
    try:
        for i in np.arange(0,2,.1):
            for leg in pod.legs:
                leg.knee.angle=x*2
                leg.hip.yangle=2*x-1
                leg.hip.xangle=2*x-1
            pod.addParam()
            pod.addParam()
            pod.addParam()
            pod.addParam()
            pod.setControl(HpodSimCtrlParam.START)
            pod.sendArray()
            pod.clearParamArray()
    except (KeyboardInterrupt,):
        print "Exit Hexapod Client"
        sys.exit(0)
        break
