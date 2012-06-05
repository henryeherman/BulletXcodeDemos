#/usr/bin/env python

import time
import hexapod
from hexapod import HpodSimCtrlParam, HpodReplies
import numpy as np
import sys

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
        self.hexapodConfigurations = []

        self.calculateTotalStates()
        self.createParamVectors()

    def calculateTotalStates(self):
        self.total_states = 0

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

        for freq in self.frequencies:
            for ampl in self.amplitudes:

                pod = hexapod.HexapodBody()
                time = np.arange(0, self.config.total_time, 0.01)
                #xs = ampl*np.sin()

                for leg_index, leg in enumerate(pod.legs):
                    leg.knee.angle = np.pi/2
                    #leg.hip.yangle =

                #for phase in self.phases:



    def runSimulation(self):
        pass

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
    config.max_freq = 4
    config.step_freq = 1

    config.min_ampl = 1
    config.max_ampl = 4
    config.step_ampl = 1

    config.min_phase = 0
    config.max_phase = np.pi/2
    config.step_phase = np.pi/6

    config.total_time = 10

    # Initialize Simulator with config
    sim = HexapodSimulator(config)

    # Run Simulation
    sim.runSimulation()


if __name__ == "__main__":
    main()
