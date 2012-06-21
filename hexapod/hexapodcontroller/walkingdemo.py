#/usr/bin/env python

import time
import hexapod
from hexapod import HpodSimCtrlParam, HpodReplies
import numpy as np
import sys

import demoarr

from matplotlib import pyplot

pod = hexapod.Hexapod()

pod.kneeStrength=100
pod.hipStrength=30
pod.dtKnee = 0.1
pod.dtHip = 0.1

t = np.arange(0,10,0.01)
xs = np.sin(2*np.pi*t/1)
xc = np.cos(2*np.pi*t/1)

xs_half = -(np.sin(1*np.pi*t/1))
xc_half = -(np.cos(1*np.pi*t/1))

pod.resetExp()
pod.cont()

print "Begin Exp"
try:
    print "Build Array"
    for pos_index, sin_pos in enumerate(xs[::1]):

        cos_pos = xc[pos_index]
        sin_pos_half = xs_half[pos_index]
        cos_pos_half = xc_half[pos_index]

        for leg_index, leg in enumerate(pod.legs):

            # Control FRONT_LEFT, MIDDLE_RIGHT, and BACK_LEFT legs
            if leg_index in [0, 3, 4]:
                leg.knee.angle= np.pi/2
                leg.hip.yangle= -np.pi/6 * sin_pos
                leg.hip.xangle= np.pi/6 * abs(sin_pos_half)

            # Control FRONT_RIGHT, MIDDLE_LEFT, and BACK_RIGHT legs
            else:
                leg.knee.angle= np.pi/2
                leg.hip.yangle= np.pi/6 * cos_pos
                leg.hip.xangle= np.pi/6 * abs(cos_pos_half)

        pod.addParam()
        #exit()

    pod.load()
    results = pod.runexp()


    print "Sent %d packets" % len(xs)

except (KeyboardInterrupt,):

    print "Exit Hexapod Client"
    sys.exit(0)

#pyplot.plot(results.zpos)
#pyplot.show()
demoarr.plotResults(results)
