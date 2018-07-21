#!/usr/bin/env python3
import numpy as np

class Status(np.ndarray):
    """Store the status of a Car
    Can be used to implement real_status, estimated_system_status
    """
    def __new__(cls, initial_d=0, initial_v=0, initial_a=0):
        ret = np.array([initial_d,
                        initial_v,
                        initial_a])
        return ret.view(cls)
