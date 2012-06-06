#/usr/bin/env python

import time
import hexapod
import numpy as np
import sys

import matplotlib.pyplot as plt

pod = hexapod.Hexapod()

pod.kneeStrength=4
pod.hipStrength=4
pod.dtKnee = 0.5
pod.dtHip = 0.5

<<<<<<< HEAD
t = np.arange(0,10,0.01)
=======
t = np.arange(0,15,0.01)
>>>>>>> 4567152be8d5075f0d77051d37232af915449436
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
    pod.loadDefault()
    results = pod.runexp()

except (KeyboardInterrupt,):

    print "Exit Hexapod Client"
    sys.exit(0)


def plotResults():
    plt.plot(results.xpos, color='b',label='X Position')
    plt.plot(results.ypos, linestyle='--', color='r', label='Y Position')
    plt.plot(results.zpos, linestyle='-', color='y', label='Z Position')
    plt.xlabel('Time')
    plt.ylabel('Distance')
    plt.title('Distance Traveled')
    plt.legend()
    plt.show()


plotResults()
