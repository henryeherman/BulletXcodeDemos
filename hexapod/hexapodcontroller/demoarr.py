#/usr/bin/env python

import time
import hexapod
from hexapod import HpodSimCtrlParam, HpodReplies
import numpy as np
import sys

from matplotlib import pyplot

pod = hexapod.Hexapod()

pod.kneeStrength=4
pod.hipStrength=4
pod.dtKnee = 0.1
pod.dtHip = 0.1

t = np.arange(0,15,0.01)
w = 1
xs = abs(np.sin(2*np.pi*t/5))

pod.resetExp()
pod.cont()


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
    pod.load()
    results = pod.runexp()

except (KeyboardInterrupt,):

    print "Exit Hexapod Client"
    sys.exit(0)


pyplot.plot(results.zpos)
pyplot.show()
