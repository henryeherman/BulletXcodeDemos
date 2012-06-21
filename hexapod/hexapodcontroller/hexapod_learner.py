#/usr/bin/env python

import time
import hexapod
from hexapod import HpodSimCtrlParam, HpodReplies
import numpy as np
import sys
import copy
import operator

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

        # Frequencies, amplitudes, and phases are all lists of lists
        # They two lists of frequencies, phases, and amplitudes for two 3-leg
        # configurations: one for FRONT_LEFT, MIDDLE_RIGHT, and BACK_LEFT
        #              and one for FRONT_RIGHT, MIDDLE_LEFT, and BACK_RIGHT
        self.frequencies = []
        self.amplitudes = []
        self.phases = []
        self.total_states = 0

        # Holds the results for different configurations
        self.results = []

        self.pod = hexapod.Hexapod()
        self.pod.kneeStrength = 100
        self.pod.hipStrength = 30
        self.pod.dtKnee = 0.1
        self.pod.dtHip = 0.1

        # Includes sinusoids for all configurations
        self.hexapodConfigs = []

        self.createParamVectors()


    def createParamVectors(self):

        for index, config in enumerate(self.config.min_freq):
            max_freq = self.config.max_freq[index]
            step_freq = self.config.step_freq[index]

            min_ampl = self.config.min_ampl[index]
            max_ampl = self.config.max_ampl[index]
            step_ampl = self.config.step_ampl[index]

            min_phase = self.config.min_phase[index]
            max_phase = self.config.max_phase[index]
            step_phase = self.config.step_phase[index]

            frequencies = self.createVector(config, max_freq, step_freq)
            amplitudes = self.createVector(min_ampl, max_ampl, step_ampl)
            phases = self.createVector(min_phase, max_phase, step_phase)

            self.frequencies.append(frequencies)
            self.amplitudes.append(amplitudes)
            self.phases.append(phases)


    def createVector(self, minimum, maximum, stepsize):

        paramList = []
        while(minimum <= maximum):
            paramList.append(minimum)
            minimum += stepsize

        return paramList

    def createAllConfigurations(self):

        iteration = 0

        hexapod_config = []

        pod = hexapod.HexapodBody()

        # For freq, ampl, and phase for FRONT_LEFT, MIDDLE_RIGHT, and BACK_LEFT legs
        for freq in self.frequencies[0]:
            for ampl in self.amplitudes[0]:
                for phase in self.phases[0]:
                    # For freq, ampl, and phase for FRONT_RIGHT, MIDDLE_LEFT, and
                    # BACK_RIGHT legs
                    for freq2 in self.frequencies[1]:
                        for ampl2 in self.amplitudes[1]:
                            for phase2 in self.phases[1]:
                                print "Freq: " + str(freq)
                                print "Ampl: " + str(ampl)
                                print "Phase: " + str(phase)

                                time = np.arange(0, self.config.total_time, 0.01)
                                xs = np.sin(2*np.pi*time/freq + phase)
                                #xc = np.cos(2*np.pi*time/freq)
                                xs_half = -(np.sin(1*np.pi*time/freq + phase))
                                #xc_half = -(np.cos(1*np.pi*time/freq))

                                xs2 = np.sin(2*np.pi*time/freq2 + phase2)
                                xs2_half = -(np.sin(1*np.pi*time/freq2+ phase2))

                                for pos_index, sin_pos in enumerate(xs[::1]):
                                    #cos_pos = xc[pos_index]
                                    sin_pos_half = xs_half[pos_index]
                                    #cos_pos_half = xc_half[pos_index]
                                    sin_pos2 = xs2[pos_index]
                                    sin_pos_half2 = xs2_half[pos_index]

                                    for leg_index, leg in enumerate(pod.legs):

                                        leg.knee.angle = np.pi/2

                                        # Control FRONT_LEFT, MIDDLE_RIGHT, and BACK_LEFT legs
                                        if leg_index in [0, 3, 4]:
                                            leg.hip.yangle = -ampl * sin_pos
                                            leg.hip.xangle = ampl* abs(sin_pos_half)

                                        # Control FRONT_RIGHT, MIDDLE_LEFT, and BACK_RIGHT legs
                                        else:
                                            #leg.hip.yangle = ampl2 * cos_pos
                                            leg.hip.yangle = ampl2 * sin_pos2
                                            #leg.hip.xangle = ampl2* abs(cos_pos_half)
                                            leg.hip.xangle = ampl2* abs(sin_pos_half2)

                                    copy_pod = copy.deepcopy(pod)
                                    hexapod_config.append(copy_pod)

                                self.hexapodConfigs.append(copy.deepcopy(hexapod_config))
                                hexapod_config = []

    def runSimulation(self):
        print "Hex config size: " + str(len(self.hexapodConfigs))
        pod = hexapod.Hexapod()

        z_positions = []

        for index, config in enumerate(self.hexapodConfigs):
            results = []
            pod.resetExp()
            pod.cont()
            pod.clearParamArray()
            for body in config:
                pod.kneeStrength = 100;
                pod.hipStrength = 30
                pod.dtKnee = 0.1
                pod.dtHip = 0.1

                pod.copyHexapodBody(body)

                pod.addParam()
            pod.load()

            print "Running configuration " + str(index)
            results = pod.runexp()


            zpos = self.getLastZpos(results)
            z_positions.append(zpos)

            self.results.append(copy.deepcopy(results))

        print "Z_positions: " + str(z_positions)
        max_index, max_value = max(enumerate(z_positions), key=operator.itemgetter(1))
        print "Best Configuration: " + str(max_index)
        print "With a Z position of: " + str(max_value)

    def getLastZpos(self, results):
        return results[-1].getzpos()



class HexapodConfiguration:

    min_freq = []
    max_freq = []
    step_freq = []

    min_ampl = []
    max_ampl = []
    step_ampl = []

    min_phase = []
    max_phase = []
    step_phase = []


    total_time = 0

def main():

    # Configure the Hexapod
    config = HexapodConfiguration()

    config.min_freq = [1, 1]
    config.max_freq = [2, 2]
    config.step_freq = [1, 1]

    config.min_ampl = [np.pi/7, np.pi/7]
    config.max_ampl = [np.pi/7, np.pi/7]
    config.step_ampl = [np.pi/7, np.pi/7]

    config.min_phase = [np.pi/6, np.pi/6+np.pi/2]
    #config.max_phase = [np.pi/6, np.pi/6+np.pi/2]
    config.max_phase = [np.pi/2, np.pi/2+np.pi/2]
    config.step_phase = [np.pi/6, np.pi/6]

    config.total_time = 3

    # Initialize Simulator with config
    sim = HexapodSimulator(config)

    # Create All Configurations
    sim.createAllConfigurations()

    # Run Simulation
    sim.runSimulation()

if __name__ == "__main__":
    main()
