#/usr/bin/env python

import time
import hexapod
from hexapod import HpodSimCtrlParam, HpodReplies
import numpy as np
import sys
import copy

from matplotlib import pyplot


class HexapodSimulator():
    def __init__(self, config):
        """ config is a dictionary containing:
                min/max frequency, stepsize
                min/max amplitude, stepsize
                min/max phase,     stepsize

                respectively called:

                min_freq,  max_freq,  step_freq
                min_ampl,  max_ampl,  step_ampl
                min_phase, max_phase, step_phase
        """
        self.config = config
        self.frequencies = []
        self.amplitudes = []
        self.phases = []
        self.total_states = 0

        self.pod = hexapod.Hexapod()
        self.pod.kneeStrength = 100
        self.pod.hipStrength = 30
        self.pod.dtKnee = 0.1
        self.pod.dtHip = 0.1

        # Includes sinusoids for all configurations
        self.hexapodConfigs = []

        self.createParamVectors()

    #def calculateTotalStates(self):
        #self.total_states = 0

        #total_frequencies = (1+self.config.max_freq - self.config.min_freq)/self.config.step_freq
        #total_amplitudes = (1+self.config.max_ampl - self.config.min_ampl)/self.config.step_ampl
        #total_phases = (1+self.config.max_phase - self.config.min_phase)/self.config.step_phase

        #print total_frequencies
        #print total_amplitudes
        #print total_phases


    def createParamVectors(self):

        self.frequencies = self.createVector(self.config.min_freq, self.config.max_freq, self.config.step_freq)
        self.amplitudes = self.createVector(self.config.min_ampl, self.config.max_ampl, self.config.step_ampl)
        self.phases = self.createVector(self.config.min_phase, self.config.max_phase, self.config.step_phase)


    def createVector(self, minimum, maximum, stepsize):

        paramList = []
        while(minimum <= maximum):
            paramList.append(minimum)
            minimum += stepsize

        return paramList

    def createAllConfigurations(self):

        iteration = 0
        total_states = 0
        total_states = len(self.frequencies) * len(self.amplitudes) * len(self.phases)
        total_states = pow(total_states, 3)
        print "Total States: " + str(total_states)
        total_states = 1

        for state in range(total_states):
            pod = hexapod.HexapodBody()
            for freq in self.frequencies:
                for ampl in self.amplitudes:
                    for phase in self.phases:
                        print "Freq: " + str(freq)
                        print "Ampl: " + str(ampl)
                        print "Phase: " + str(phase)

                        time = np.arange(0, self.config.total_time, 0.01)
                        xs = np.sin(2*np.pi*time/freq)
                        xc = np.cos(2*np.pi*time/freq)
                        xs_half = -(np.sin(1*np.pi*time/freq))
                        xc_half = -(np.cos(1*np.pi*time/freq))

                        #print str(xs)

                        #exit(0)

                        for pos_index, sin_pos in enumerate(xs[::1]):
                            cos_pos = xc[pos_index]
                            sin_pos_half = xs_half[pos_index]
                            cos_pos_half = xc_half[pos_index]

                            for leg_index, leg in enumerate(pod.legs):

                                leg.knee.angle = np.pi/2

                                # Control FRONT_LEFT, MIDDLE_RIGHT, and BACK_LEFT legs
                                if leg_index in [0, 3, 4]:
                                    leg.hip.yangle = -ampl * sin_pos
                                    leg.hip.xangle = ampl* abs(sin_pos_half)

                                # Control FRONT_RIGHT, MIDDLE_LEFT, and BACK_RIGHT legs
                                else:
                                    leg.hip.yangle = -ampl * cos_pos
                                    leg.hip.xangle = ampl* abs(cos_pos_half)

                            copy_pod = copy.copy(pod)
                            self.hexapodConfigs.append(copy_pod)

            print state


                    #for phase in self.phases:

        #print str(self.hexapodConfigs)
        for hexpod in self.hexapodConfigs:
           print hexpod
        print total_states




    def runSimulation(self):
        print "Hex config size: " + str(len(self.hexapodConfigs))
        pod = hexapod.Hexapod()

        pod.kneeStrength = 100
        pod.hipStrength = 30
        pod.dtKnee = 0.1
        pod.dtHip = 0.1

        pod.resetExp()
        pod.cont()

        #pod.copyHexapodBody(self.hexapodConfigs[0])
        for config in self.hexapodConfigs:
            pod.copyHexapodBody(config)
            pod.addParam()

        pod.load()
        results = pod.runexp()

        print "Configuration: "
        print str(self.hexapodConfigs[0])
        print "RESULTS: "
        print str(results)





class HexapodConfiguration:

    min_freq = 0
    max_freq = 0
    step_freq = 0

    min_ampl = 0
    max_ampl = 0
    step_ampl = 0

    min_phase = 0
    max_phase = 0
    step_phase = 0

    total_time = 0




def main():

    # Configure the Hexapod
    config = HexapodConfiguration()

    config.min_freq = 1
    config.max_freq = 1
    config.step_freq = 1

    config.min_ampl = 1
    config.max_ampl = 1
    config.step_ampl = 1

    config.min_phase = np.pi/6
    config.max_phase = np.pi/6
    config.step_phase = np.pi/6

    config.total_time = 10

    # Initialize Simulator with config
    sim = HexapodSimulator(config)

    # Create All Configurations
    sim.createAllConfigurations()

    # Run Simulation
    sim.runSimulation()



if __name__ == "__main__":
    main()
