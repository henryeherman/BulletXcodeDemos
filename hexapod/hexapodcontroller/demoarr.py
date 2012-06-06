#/usr/bin/env python

import time
import hexapod
import numpy as np
import sys

import matplotlib.pyplot as plt


def main():
    pod = hexapod.Hexapod()

    pod.kneeStrength=4
    pod.hipStrength=4
    pod.dtKnee = 0.5
    pod.dtHip = 0.5

    t = np.arange(0,10,0.01)
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

    return results


def plotResults(results):
    plt.interactive(True)
    plt.subplot(131)
    plt.plot(results.xpos, color='b',label='X Position')
    plt.plot(results.ypos, linestyle='--', color='r', label='Y Position')
    plt.plot(results.zpos, linestyle='-', color='y', label='Z Position')
    plt.xlabel('Time')
    plt.ylabel('Distance')
    plt.title('Distance Traveled')
    plt.legend()
    plt.show()

    ts = range(0,len(results),6)
    plt.subplot(131)
    pt = plt.plot(0,results.zpos[0], 'ro', markersize=4)
    for t in ts:
        plt.subplot(131)
        pt[0].set_ydata(results.zpos[t])
        pt[0].set_xdata(t)
        ax1 = plt.subplot(132)
        ax1.clear()
        plt.bar(range(6), results[t].lowerleglinearmag)
        ax2 = plt.subplot(133)
        ax2.clear()
        plt.bar(range(6), results[t].upperleglinearmag)
        plt.draw()
        time.sleep(0.01)

if __name__ == "__main__":
    results = main()
    plotResults(results)

