#!/usr/bin/env python3

import numpy as np

class Noises(object):
    """docstring for Noises"""
    def __init__(self, noise_mean_list, noise_sigma_list):
        if len(noise_mean_list) != len(noise_sigma_list):
            print('noise_mean_list size not equal to noise_sigma_list size')
        self.mean = np.array(noise_mean_list)
        self.sigma = np.array(noise_sigma_list)
        