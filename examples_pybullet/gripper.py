import os
import sys
import time

import numpy as np
import scipy as sp
import pinocchio as pin

import pybullet as p
import pybullet_data

from utils.Toolbox import *

from leap_tracker import LeapTracker

if __name__ == "__main__":

    leap_tracker = LeapTracker()
    leap_tracker.connect('Desktop')

    while True:
        exist, right_tips_pos = leap_tracker.get_right_tips_pos()

        if exist:
            T_right_tips = []
            p_h = []
            for i in range(5):
                right_tip_pos = right_tips_pos[i]
                T_right_tip_pos = np.block([[np.identity(3), right_tip_pos.reshape(-1, 1) * 0.0014], [np.zeros([1, 3]), 1]])
                T_right_tips.append(T_right_tip_pos)
                p_h.append(T_right_tip_pos[0:3, 3])

            p_h = np.array(p_h)

            d = np.linalg.norm(p_h[1] - p_h[0]) + np.linalg.norm(p_h[2] - p_h[0])

            d_min = 0.07
            d_max = 0.20
            d = (np.clip(d, d_min, d_max) - d_min)/(d_max-d_min)
            print(d)

        time.sleep(0.1)
