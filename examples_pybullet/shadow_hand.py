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

    p.resetDebugVisualizerCamera(cameraDistance=0.4, cameraYaw=15, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.2])

    #load the MuJoCo MJCF hand
    filepath = os.path.dirname(os.path.realpath(__file__))
    handId = p.loadURDF(filepath + "/assets/shadow_hand/model.urdf")

    # p.setRealTimeSimulation(0)

    wrist_joint_list = [0, 1]               # [-30, 10], [-40, 28]
    thumb_joint_list = [2, 3, 4, 5, 6]   # [-60, 60], [0, 70], [-12, 12], [40, 40], [0, 90]
    index_joint_list = [7, 8, 9, 10]        # [-20, 20], [0, 90], [0, 90], [0, 90]
    middle_joint_list = [11, 12, 13, 14]       # [-20, 20], [0, 90], [0, 90], [0, 90]
    ring_joint_list = [15, 16, 17, 18]         # [-20, 20], [0, 90], [0, 90], [0, 90]
    little_joint_list = [19, 20, 21, 22, 23]   # [0, 45], [-20, 20], [0, 90], [0, 90], [0, 90]

    leap_tracker = LeapTracker()
    leap_tracker.connect('Desktop')

    thumb_deg = [0.0] * 5
    index_deg = [0.0] * 4
    middle_deg = [0.0] * 4
    ring_deg = [0.0] * 4
    little_deg = [0.0] * 5

    while True:
        res, left_joints = leap_tracker.get_left_joints()
        if res:

            t = left_joints[0] * 180 / np.pi
            thumb_deg[0] = 60 - t[0]
            thumb_deg[1] = 40
            thumb_deg[2] = 0
            thumb_deg[3] = t[2]
            thumb_deg[4] = t[3]

            t = left_joints[1] * 180/np.pi
            index_deg[0] = -7
            index_deg[1] = t[0]
            index_deg[2] = t[1]
            index_deg[3] = t[2]

            t = left_joints[2] * 180/np.pi
            middle_deg[0] = 0
            middle_deg[1] = t[0]
            middle_deg[2] = t[1]
            middle_deg[3] = t[2]

            t = left_joints[3] * 180 / np.pi
            ring_deg[0] = -7
            ring_deg[1] = t[0]
            ring_deg[2] = t[1]
            ring_deg[3] = t[2]

            t = left_joints[4] * 180 / np.pi
            little_deg[0] = 10
            little_deg[1] = -10
            little_deg[2] = t[0]
            little_deg[3] = t[1]
            little_deg[4] = t[2]

            for i in range(5):
                p.setJointMotorControl2(handId, thumb_joint_list[i], p.POSITION_CONTROL, thumb_deg[i] * np.pi/180)
            for i in range(4):
                p.setJointMotorControl2(handId, index_joint_list[i], p.POSITION_CONTROL, index_deg[i] * np.pi/180)
            for i in range(4):
                p.setJointMotorControl2(handId, middle_joint_list[i], p.POSITION_CONTROL, middle_deg[i] * np.pi/180)
            for i in range(4):
                p.setJointMotorControl2(handId, ring_joint_list[i], p.POSITION_CONTROL, ring_deg[i] * np.pi/180)
            for i in range(5):
                p.setJointMotorControl2(handId, little_joint_list[i], p.POSITION_CONTROL, little_deg[i] * np.pi/180)

        time.sleep(0.0001)
        p.stepSimulation()