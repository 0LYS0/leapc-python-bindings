import os
import sys
import time

import numpy as np
import scipy as sp
import pinocchio as pin

from shadow_hand import ShadowHand

import pybullet as p

from utils.Toolbox import *


def my_func(x, p_h):
    # x = [q_r]: 24 = (24,) np.ndarray
    # p_h: 5*3 = (5, 3) np.ndarray
    T_list = shadow_hand.FK(x[0:24])

    p_r1 = T_list[0][0:3, 3]
    p_r2 = T_list[1][0:3, 3]

    p_h1 = p_h[0]
    p_h2 = p_h[1]
    return (np.linalg.norm(p_r1 - p_r2) - np.linalg.norm(p_h1 - p_h2)) ** 2

if __name__ == "__main__":
    CliendId = p.connect(p.GUI)

    p.setTimeStep(1./1000)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    p.resetDebugVisualizerCamera(cameraDistance=0.4, cameraYaw=15, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.2])
    shadow_hand = ShadowHand(CliendId)


    p_h = np.array([
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ])
    f0 = lambda x: my_func(x, p_h=p_h)
    x0 = np.zeros([24])
    res = sp.optimize.minimize(f0, x0, bounds=sp.optimize.Bounds(shadow_hand.q_l, shadow_hand.q_u), method='SLSQP', options={'disp': 1})
    print(res.x)

    while True:
        shadow_hand.move_joint_to(res.x, degree=False)
        time.sleep(0.001)
        p.stepSimulation()