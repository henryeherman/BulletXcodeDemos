#/usr/bin/env python

import time
import hexapod
import numpy as np

pod = hexapod.Hexapod()
 
pod.kneeStrength=10
pod.hipStrength=10
pod.dtKnee = 0.1
pod.dtHip = 0.1

while(1):
    for i in np.arange(0,3,.1):
        for leg in pod.legs:
            leg.knee.angle=i
        pod.send()
        time.sleep(.1)


