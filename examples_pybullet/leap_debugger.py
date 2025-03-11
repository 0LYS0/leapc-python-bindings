import os
import sys

import serial
import time

import numpy as np

import leap

import pybullet as p
import pybullet_data

from leap_tracker import LeapTracker

if __name__ == "__main__":
    p.connect(p.GUI)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=15, cameraPitch=-60, cameraTargetPosition=[0.0, 0.0, 0.8])

    #load the MuJoCo MJCF hand
    filepath = os.path.dirname(os.path.realpath(__file__))
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF('plane.urdf')
    # p.setRealTimeSimulation(0)

    leap_tracker = LeapTracker()
    leap_tracker.connect('Desktop')
    debug_joint_id_1 = None
    while True:
        exist, thumb_pos, index_pos, middle_pos, ring_pos, little_pos, palm_pos, palm_norm = leap_tracker.test()

        debug_points = np.vstack((thumb_pos, index_pos, middle_pos, ring_pos, little_pos))
        debug_points -= index_pos[0]
        debug_points *= 1/1000
        debug_points = debug_points @ np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        debug_points[:, 2] += 1
        if debug_joint_id_1 is None:
            debug_joint_id_1 = p.addUserDebugPoints(pointPositions=debug_points,
                                                    pointColorsRGB=[[0, 0, 0]]*len(debug_points),
                                                    pointSize=7,
                                                    lifeTime=0)
        else:
            debug_joint_id_1 = p.addUserDebugPoints(pointPositions=debug_points,
                                                    pointColorsRGB=[[0, 0, 0]] * len(debug_points),
                                                    pointSize=7,
                                                    replaceItemUniqueId=debug_joint_id_1,
                                                    lifeTime=0)

        time.sleep(0.0001)
        p.stepSimulation()