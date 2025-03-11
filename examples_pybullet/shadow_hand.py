import os
import sys

import serial
import time

import numpy as np

import leap

import pybullet as p
import pybullet_data


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

#clamp in range 400-600
#minV = 400
#maxV = 600
minVarray = [275, 280, 350, 290]
maxVarray = [450, 550, 500, 400]

pinkId = 0
middleId = 1
indexId = 2
thumbId = 3

p.setRealTimeSimulation(0)

def convertSensor(x, fingerIndex):
    minV = minVarray[fingerIndex]
    maxV = maxVarray[fingerIndex]

    v = minV
    try:
        v = float(x)
    except ValueError:
        v = minV
    if (v < minV):
        v = minV
    if (v > maxV):
        v = maxV
    b = (v - minV) / float(maxV - minV)

    return b

jointState = p.getJointState(
    bodyUniqueId=handId,
    jointIndex=0,
)

# wrist_joint_list = [0, 23]               # [-30, 10], [-40, 28]
# thumb_joint_list = [5, 10, 15, 20, 22]   # [-60, 60], [0, 70], [-12, 12], [40, 40], [0, 90]
# index_joint_list = [1, 6, 11, 16]        # [-20, 20], [0, 90], [0, 90], [0, 90]
# middle_joint_list = [3, 8, 13, 18]       # [-20, 20], [0, 90], [0, 90], [0, 90]
# ring_joint_list = [4, 9, 14, 19]         # [-20, 20], [0, 90], [0, 90], [0, 90]
# little_joint_list = [2, 7, 12, 17, 21]   # [0, 45], [-20, 20], [0, 90], [0, 90], [0, 90]

wrist_joint_list = [0, 1]               # [-30, 10], [-40, 28]
thumb_joint_list = [2, 3, 4, 5, 6]   # [-60, 60], [0, 70], [-12, 12], [40, 40], [0, 90]
index_joint_list = [7, 8, 9, 10]        # [-20, 20], [0, 90], [0, 90], [0, 90]
middle_joint_list = [11, 12, 13, 14]       # [-20, 20], [0, 90], [0, 90], [0, 90]
ring_joint_list = [15, 16, 17, 18]         # [-20, 20], [0, 90], [0, 90], [0, 90]
little_joint_list = [19, 20, 21, 22, 23]   # [0, 45], [-20, 20], [0, 90], [0, 90], [0, 90]

tt = 0
while True:
    # words = line.split(",")
    tt += np.pi/24000
    t = np.sin(tt) ** 2

    index_deg = [0.0]*4
    index_deg[0] = -7
    index_deg[1] = 0 + (70 - 0) * t
    index_deg[2] = 0 + (70 - 0) * t
    index_deg[3] = 0 + (70 - 0) * t

    middle_deg = [0.0] * 4
    middle_deg[0] = 0
    middle_deg[1] = 0 + (70 - 0) * t
    middle_deg[2] = 0 + (70 - 0) * t
    middle_deg[3] = 0 + (70 - 0) * t

    ring_deg = [0.0] * 4
    ring_deg[0] = -7
    ring_deg[1] = 0 + (70 - 0) * t
    ring_deg[2] = 0 + (70 - 0) * t
    ring_deg[3] = 0 + (70 - 0) * t

    little_deg = [0.0] * 5
    little_deg[0] = 10
    little_deg[1] = -10
    little_deg[2] = 0 + (70 - 0) * t
    little_deg[3] = 0 + (70 - 0) * t
    little_deg[4] = 0 + (70 - 0) * t

    for i in range(4):
        p.setJointMotorControl2(handId, index_joint_list[i], p.POSITION_CONTROL, index_deg[i] * np.pi/180)
    for i in range(4):
        p.setJointMotorControl2(handId, middle_joint_list[i], p.POSITION_CONTROL, middle_deg[i] * np.pi/180)
    for i in range(4):
        p.setJointMotorControl2(handId, ring_joint_list[i], p.POSITION_CONTROL, ring_deg[i] * np.pi/180)
    for i in range(5):
        p.setJointMotorControl2(handId, little_joint_list[i], p.POSITION_CONTROL, little_deg[i] * np.pi/180)


    # p.setJointMotorControl2(hand, 7, p.POSITION_CONTROL, np.pi / 4.)
    # p.setJointMotorControl2(hand, 9, p.POSITION_CONTROL, thumb + np.pi / 10)
    # p.setJointMotorControl2(hand, 11, p.POSITION_CONTROL, thumb)
    # p.setJointMotorControl2(hand, 13, p.POSITION_CONTROL, thumb)
    #
    # p.setJointMotorControl2(hand, 17, p.POSITION_CONTROL, index)
    # p.setJointMotorControl2(hand, 19, p.POSITION_CONTROL, index)
    # p.setJointMotorControl2(hand, 21, p.POSITION_CONTROL, index)
    #
    # p.setJointMotorControl2(hand, 24, p.POSITION_CONTROL, middle)
    # p.setJointMotorControl2(hand, 26, p.POSITION_CONTROL, middle)
    # p.setJointMotorControl2(hand, 28, p.POSITION_CONTROL, middle)
    #
    # p.setJointMotorControl2(hand, 40, p.POSITION_CONTROL, pink)
    # p.setJointMotorControl2(hand, 42, p.POSITION_CONTROL, pink)
    # p.setJointMotorControl2(hand, 44, p.POSITION_CONTROL, pink)
    #
    # ringpos = 0.5 * (pink + middle)
    # p.setJointMotorControl2(hand, 32, p.POSITION_CONTROL, ringpos)
    # p.setJointMotorControl2(hand, 34, p.POSITION_CONTROL, ringpos)
    # p.setJointMotorControl2(hand, 36, p.POSITION_CONTROL, ringpos)
    p.stepSimulation()