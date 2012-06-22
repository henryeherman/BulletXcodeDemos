#/usr/bin/env python

import time
import hexapod
from hexapod import HpodSimCtrlParam, HpodReplies
import numpy as np
import sys
import copy
import operator
import random

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

    # This function does the same as createAllConfigurations except that the frequencies,
    # amplitudes, and phases are all already created together.
    def createTimeSeriesConfigurations(self, frequencies, amplitudes, phases):
        iteration = 0

        hexapod_config = []

        pod = hexapod.HexapodBody()

        for index, freq in enumerate(frequencies[0]):
            freq2 = frequencies[1][index]

            ampl = amplitudes[0][index]
            ampl2 = amplitudes[1][index]

            phase = phases[0][index]
            phase2 = phases[1][index]

            print "Configuration " + str(iteration)

            print "Freq: " + str(freq) + ", Ampl: " + str(ampl) + ", Phase: " + str(phase)
            print "Freq: " + str(freq2) + ", Ampl: " + str(ampl2) + ", Phase: " + str(phase2)

            pod.frequencies = []
            pod.amplitudes = []
            pod.phases = []
            pod.frequencies.append(freq)
            pod.frequencies.append(freq2)
            pod.amplitudes.append(ampl)
            pod.amplitudes.append(ampl2)
            pod.phases.append(phase)
            pod.phases.append(phase2)

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
            iteration += 1





    def createAllConfigurations(self, frequencies, amplitudes, phases):

        iteration = 0

        hexapod_config = []

        pod = hexapod.HexapodBody()

        print "FREQUENCIES: " + str(frequencies)

        # For freq, ampl, and phase for FRONT_LEFT, MIDDLE_RIGHT, and BACK_LEFT legs
        for freq in frequencies[0]:
            for ampl in amplitudes[0]:
                for phase in phases[0]:
                    # For freq, ampl, and phase for FRONT_RIGHT, MIDDLE_LEFT, and
                    # BACK_RIGHT legs
                    for freq2 in frequencies[1]:
                        for ampl2 in amplitudes[1]:
                            for phase2 in phases[1]:
                                print "Configuration " + str(iteration)

                                print "Freq: " + str(freq) + ", Ampl: " + str(ampl) + ", Phase: " + str(phase)
                                print "Freq: " + str(freq2) + ", Ampl: " + str(ampl2) + ", Phase: " + str(phase2)

                                pod.frequencies = []
                                pod.amplitudes = []
                                pod.phases = []
                                pod.frequencies.append(freq)
                                pod.frequencies.append(freq2)
                                pod.amplitudes.append(ampl)
                                pod.amplitudes.append(ampl2)
                                pod.phases.append(phase)
                                pod.phases.append(phase2)

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
                                iteration += 1

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
            print "Z position: " + str(zpos)
            z_positions.append(zpos)

            self.results.append(copy.deepcopy(results))



        print "Z_positions: " + str(z_positions)
        max_index, max_value = max(enumerate(z_positions), key=operator.itemgetter(1))
        print "Best Configuration: " + str(max_index)
        print "With a Z position of: " + str(max_value)
        hexapodConfig = self.hexapodConfigs[max_index]
        print "Frequencies: " + str(hexapodConfig[0].frequencies)
        print "Amplitudes: " + str(hexapodConfig[0].amplitudes)
        print "Phases: " + str(hexapodConfig[0].phases)

        # Clear the hexapodConfigs list
        self.hexapodConfigs = []

        return (hexapodConfig[0], max_value, max_index)

    def addNoiseN(self, previous_config, n):

        config = copy.deepcopy(previous_config)

        freq = config.frequencies[0]
        freq2 = config.frequencies[1]

        ampl = config.amplitudes[0]
        ampl2 = config.amplitudes[1]

        phase = config.phases[0]
        phase2 = config.phases[1]

        config.frequencies[0] = []
        config.amplitudes[0] = []
        config.phases[0] = []

        config.frequencies[1] = []
        config.amplitudes[1] = []
        config.phases[1] = []

        print "Adding Noise: "
        print "Old freq: " + str(freq)
        print "Old freq2: " + str(freq2)

        for i in range(n):
            rand_num = random.random()
            config.frequencies[0].append(freq + rand_num)
            config.frequencies[1].append(freq2 + rand_num)
            config.amplitudes[0].append(ampl + rand_num)
            config.amplitudes[1].append(ampl2 + rand_num)
            config.phases[0].append(phase + rand_num)
            config.phases[1].append(phase2 + rand_num)

        return config

    #def addNoise(self, config):
        #rand_num = random.random()
        #config.frequencies[0] = config.frequencies[0] + rand_num

        #rand_num = random.random()
        #config.frequencies[1] = config.frequencies[1] + rand_num

        #rand_num = random.random()
        #config.amplitudes[0] = config.amplitudes[0] + rand_num

        #rand_num = random.random()
        #config.amplitudes[1] = config.amplitudes[1] + rand_num

        #rand_num = random.random()
        #config.phases[0] = config.phases[0] + rand_num

        #rand_num = random.random()
        #config.phases[1] = config.phases[1] + rand_num

        #return config

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

    # Configure the Hexapod with initial parameters
    config = HexapodConfiguration()

    config.min_freq = [1, 1]
    config.max_freq = [2, 2]
    config.step_freq = [1, 1]

    config.min_ampl = [np.pi/7, np.pi/7]
    config.max_ampl = [np.pi/7, np.pi/7]
    config.step_ampl = [np.pi/7, np.pi/7]

    config.min_phase = [np.pi/6, np.pi/6+np.pi/2]
    config.max_phase = [np.pi/6, np.pi/6+np.pi/2]
    #config.max_phase = [np.pi/2, np.pi/2+np.pi/2]
    config.step_phase = [np.pi/6, np.pi/6]

    config.total_time = 3

    # Initialize Simulator with config
    sim = HexapodSimulator(config)

    # Create All Configurations for initial parameters and simulate
    sim.createAllConfigurations(sim.frequencies, sim.amplitudes, sim.phases)
    best_sim, dist, max_index = sim.runSimulation()

    # Record these simulation values as initial best values
    max_z_distance = dist
    best_simulation = copy.deepcopy(best_sim)
    max_sim_index = max_index

    prev_best_sim = copy.deepcopy(best_sim)

    num_iterations = 2      # This specifies how many times to iterate through
                            # Higher amount of iterations should give better results
    for iteration in range(num_iterations):

        print "\n\nITERATION " + str(iteration)

        # Create 5 new configurations by adding noise to previous best simulation
        noisy_sim = sim.addNoiseN(prev_best_sim, 5)

        # Create the time series configurations (sine parameters) with these 5 configs
        # and run the simulation
        sim.createTimeSeriesConfigurations(noisy_sim.frequencies,
                                           noisy_sim.amplitudes,
                                           noisy_sim.phases)
        best_sim, dist, max_index = sim.runSimulation()

        # If outcome was better with this iteration, record distance, index, and
        # simulation. Then move on to the next iteration with the best simulation
        if dist > max_z_distance:
            print "New iteration moved farther! Best Simulation is Updated!"
            max_z_distance = dist
            best_simulation = copy.deepcopy(best_sim)
            max_sim_index = max_index
            prev_best_sim = copy.deepcopy(best_sim)

        # if outcome was not better, don't record new variables and redo current
        # simulation
        else:
            print "New iteration moved less distance, reverting back and trying again!"


    print "\n\n---------------------------------------------------------------"
    print "Overall Best"

    # Play best simulation
    sim.createTimeSeriesConfigurations(
            [[best_simulation.frequencies[0]],[best_simulation.frequencies[1]]],
            [[best_simulation.amplitudes[0]], [best_simulation.amplitudes[1]]],
            [[best_simulation.phases[0]], [best_simulation.phases[1]]])
    best_sim, dist, max_index = sim.runSimulation()

    print "Best simulation: " + str(best_sim)






if __name__ == "__main__":
    main()
