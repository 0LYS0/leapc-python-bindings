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

class ShadowHand:
    def __init__(self, ClientId):

        self._ClientId = ClientId

        # load the MuJoCo MJCF hand
        filepath = os.path.dirname(os.path.realpath(__file__))
        urdfpath = filepath + "/assets/shadow_hand/model.urdf"
        self._handId = p.loadURDF(urdfpath)
        # p.resetBasePositionAndOrientation(self._handId, [0, 0, 0], [0, 0, 0, 1])

        self._pinModel = pin.buildModelFromUrdf(urdfpath)
        self._pinData = self._pinModel.createData()

        self.num_joint = self._pinModel.nq
        pin.forwardKinematics(self._pinModel, self._pinData, np.zeros([self.num_joint, 1]))
        pin.updateFramePlacements(self._pinModel, self._pinData)

        print("*********************************************************")

        print("*********************************************************")
        wrist_joint_idx = [0, 1]  # [-30, 10], [-40, 28]
        thumb_joint_idx = [2, 3, 4, 5, 6]  # [-60, 60], [0, 70], [-12, 12], [40, 40], [0, 90]
        index_joint_idx = [7, 8, 9, 10]  # [-20, 20], [0, 90], [0, 90], [0, 90]
        middle_joint_idx = [11, 12, 13, 14]  # [-20, 20], [0, 90], [0, 90], [0, 90]
        ring_joint_idx = [15, 16, 17, 18]  # [-20, 20], [0, 90], [0, 90], [0, 90]
        little_joint_idx = [19, 20, 21, 22, 23]  # [0, 45], [-20, 20], [0, 90], [0, 90], [0, 90]

        self.joints_idx = [wrist_joint_idx, thumb_joint_idx, index_joint_idx, middle_joint_idx, ring_joint_idx, little_joint_idx]
        # self.shuffled_joints_idx_list = wrist_joint_idx + index_joint_idx + little_joint_idx + middle_joint_idx + ring_joint_idx + thumb_joint_idx
        self.shuffled_idx_list = [0, 1, 7, 8, 9, 10, 19, 20, 21, 22, 23, 11, 12, 13, 14, 15, 16, 17, 18, 2, 3, 4, 5, 6]
        self.inv_shuffled_idx_list = [0, 1, 19, 20, 21, 22, 23, 2, 3, 4, 5, 11, 12, 13, 14, 15, 16, 17, 18, 6, 7, 8, 9, 10]

        # self.q_l = np.array([-30, -40, -60, 0, -12, -40, 0, -20, 0, 0, 0, -20, 0, 0, 0, -20, 0, 0, 0, 0, -20, 0, 0, 0]) * np.pi/180
        # self.q_u = np.array([10, 28, 60, 70, 12, 40, 90, 20, 90, 90, 90, 20, 90, 90, 90, 20, 90, 90, 90, 45, 20, 90, 90, 90]) * np.pi/180
        self.q_l = np.array([-0, -0,
                             -60, 0, -12, -10, 0,
                             -20, 0, 0, 0,
                             -5, 0, 0, 0,
                             -20, 0, 0, 0,
                             0, -20, 0, 0, 0]) * np.pi / 180
        self.q_u = np.array([ 0, 0,
                              60, 70, 12, 40, 90,
                              0, 90, 90, 90,
                              5, 90, 90, 90,
                              0, 90, 90, 90,
                              45, 0, 90, 90, 90]) * np.pi/180

        self._T_W0 = np.identity(4)
        self._T_EEs = [
            xyzeul2SE3([0, 0, 0.025], [0, 0, 0]),
            xyzeul2SE3([0, 0, 0.025], [0, 0, 0]),
            xyzeul2SE3([0, 0, 0.025], [0, 0, 0]),
            xyzeul2SE3([0, 0, 0.025], [0, 0, 0]),
            xyzeul2SE3([0, 0, 0.025], [0, 0, 0]),
        ]
        self.T_list = np.zeros([5, 4, 4])

        # Debug Frame buffer
        self._debug_frame_buff_list = []
        self._fingertip_frame_buff_list = []

        self._palm_center_SE3 = self._T_W0 @ xyzeul2SE3([0, -0.025, 0.28], [90, 0, 0], degree=True)
        self.add_debug_frames([self._palm_center_SE3])

    # Utils
    def shuffle_to_pin_idx(self, q: np.ndarray):
        return q[self.shuffled_idx_list]

    def shuffle_to_jnt_idx(self, q: np.ndarray):
        return q[self.inv_shuffled_idx_list]

    def np_to_joints_list(self, q):
        return [q[self.joints_idx[i]].tolist() for i in range(6)]

    def joints_list_to_np(self, joints_list):
        return np.array(joints_list[0] + joints_list[1] + joints_list[2] + joints_list[3] + joints_list[4] + joints_list[5])

    # Kinematics
    def forward_kinematics(self, q, degree=True):
        # 2+5+4+4+4+5 = 24

        if degree:
            s = np.pi/180
        else:
            s = 1

        T_list = self.FK(q * s)

        self.add_debug_frames(T_list, self._fingertip_frame_buff_list)

        self.T_list = T_list

    def FK(self, q: np.ndarray):

        q = self.shuffle_to_pin_idx(q)

        pin.forwardKinematics(self._pinModel, self._pinData, q)
        pin.updateFramePlacements(self._pinModel, self._pinData)

        T_list = np.zeros([5, 4, 4])
        T_list[0] = self._T_W0 @ self._pinData.oMi[self.inv_shuffled_idx_list[self.joints_idx[1][-1]] + 1].np @ self._T_EEs[0]
        T_list[1] = self._T_W0 @ self._pinData.oMi[self.inv_shuffled_idx_list[self.joints_idx[2][-1]] + 1].np @ self._T_EEs[1]
        T_list[2] = self._T_W0 @ self._pinData.oMi[self.inv_shuffled_idx_list[self.joints_idx[3][-1]] + 1].np @ self._T_EEs[2]
        T_list[3] = self._T_W0 @ self._pinData.oMi[self.inv_shuffled_idx_list[self.joints_idx[4][-1]] + 1].np @ self._T_EEs[3]
        T_list[4] = self._T_W0 @ self._pinData.oMi[self.inv_shuffled_idx_list[self.joints_idx[5][-1]] + 1].np @ self._T_EEs[4]

        return T_list


    def move_joint_to(self, q, degree=True):

        if degree:
            s = np.pi/180
        else:
            s = 1

        q = q*s

        control_kwargs = {'positionGain': 0.5, 'velocityGain': 0.5}
        for i in range(2):
            p.setJointMotorControl2(self._handId, self.joints_idx[0][i], p.POSITION_CONTROL, q[self.joints_idx[0][i]], **control_kwargs)
        for i in range(5):
            p.setJointMotorControl2(self._handId, self.joints_idx[1][i], p.POSITION_CONTROL, q[self.joints_idx[1][i]], **control_kwargs)
        for i in range(4):
            p.setJointMotorControl2(self._handId, self.joints_idx[2][i], p.POSITION_CONTROL, q[self.joints_idx[2][i]], **control_kwargs)
        for i in range(4):
            p.setJointMotorControl2(self._handId, self.joints_idx[3][i], p.POSITION_CONTROL, q[self.joints_idx[3][i]], **control_kwargs)
        for i in range(4):
            p.setJointMotorControl2(self._handId, self.joints_idx[4][i], p.POSITION_CONTROL, q[self.joints_idx[4][i]], **control_kwargs)
        for i in range(5):
            p.setJointMotorControl2(self._handId, self.joints_idx[5][i], p.POSITION_CONTROL, q[self.joints_idx[5][i]], **control_kwargs)

    # Debug Frame
    def add_debug_frames(self, Tlist, frame_buff_list=None):

        if frame_buff_list is None:
            frame_buff_list = self._debug_frame_buff_list

        if len(Tlist) > len(frame_buff_list):
            for _ in range(len(Tlist) - len(frame_buff_list)):
                frame_buff_list.append(DebugFrame(self._ClientId))
        elif len(Tlist) < len(frame_buff_list):
            for i in range(len(frame_buff_list) - len(Tlist)):
                frame_buff_list[len(Tlist) + i].setPos([0, 0, -1], [0, 0, 0])

        for i, T in enumerate(Tlist):
            frame_buff_list[i].setSE3(T)

    def destroy_debug_frames(self, frame_buff_list=None):

        if frame_buff_list is None:
            frame_buff_list = self._debug_frame_buff_list

        for i in range(len(frame_buff_list)):
            frame_buff_list[0].remove()
            frame_buff_list.pop(0)


class DebugFrame:
    def __init__(self, ClientId, lineWidth=3):

        self.ClientId = ClientId

        visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[0, 0.5, 0.5, 0.45])

        self._endID = p.createMultiBody(baseVisualShapeIndex=visualShapeId, basePosition=[0, 0, -1],
                                        baseOrientation=[0, 0, 0], physicsClientId=self.ClientId)

        self._endID_x = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0.04, 0, 0], lineColorRGB=[1, 0, 0],
                                           lineWidth=lineWidth, parentObjectUniqueId=self._endID,
                                           physicsClientId=self.ClientId)
        self._endID_y = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0, 0.04, 0], lineColorRGB=[0, 1, 0],
                                           lineWidth=lineWidth, parentObjectUniqueId=self._endID,
                                           physicsClientId=self.ClientId)
        self._endID_z = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0, 0, 0.04], lineColorRGB=[0, 0, 1],
                                           lineWidth=lineWidth, parentObjectUniqueId=self._endID,
                                           physicsClientId=self.ClientId)

    def setSE3(self, T):
        p.resetBasePositionAndOrientation(bodyUniqueId=self._endID, posObj=T[0:3, 3],
                                          ornObj=Rot2quat(T[0:3, 0:3]), physicsClientId=self.ClientId)

    def setPos(self, pos, ori):
        self.setSE3(xyzeul2SE3(pos, ori))

    def remove(self):
        p.removeBody(bodyUniqueId=self._endID, physicsClientId=self.ClientId)


def my_func(x, p_h):
    # x = [q_r]: 24 = (24,) np.ndarray
    # p_h: 5*3 = (5, 3) np.ndarray

    J = 0

    T_list = shadow_hand.FK(x[0:24])
    p_palm = shadow_hand._palm_center_SE3[0:3, 3]

    # gamma = 0.05
    # J += gamma * np.linalg.norm(x)

    p_r1 = T_list[0][0:3, 3] - p_palm
    p_r2 = T_list[1][0:3, 3] - p_palm
    p_r3 = T_list[2][0:3, 3] - p_palm
    p_r4 = T_list[3][0:3, 3] - p_palm
    p_r5 = T_list[4][0:3, 3] - p_palm

    p_h1 = p_h[0]
    p_h2 = p_h[1]
    p_h3 = p_h[2]
    p_h4 = p_h[3]
    p_h5 = p_h[4]

    #
    d_r01 = np.linalg.norm(p_r1)
    d_r02 = np.linalg.norm(p_r2)
    d_r03 = np.linalg.norm(p_r3)
    d_r04 = np.linalg.norm(p_r4)
    d_r05 = np.linalg.norm(p_r5)

    r_r12 = p_r1 - p_r2
    r_r13 = p_r1 - p_r3
    r_r14 = p_r1 - p_r4
    r_r15 = p_r1 - p_r5
    r_r23 = p_r2 - p_r3
    r_r24 = p_r2 - p_r4
    r_r25 = p_r2 - p_r5
    r_r34 = p_r3 - p_r4
    r_r35 = p_r3 - p_r5
    r_r45 = p_r4 - p_r5

    d_h01 = np.linalg.norm(p_h1)
    d_h02 = np.linalg.norm(p_h2)
    d_h03 = np.linalg.norm(p_h3)
    d_h04 = np.linalg.norm(p_h4)
    d_h05 = np.linalg.norm(p_h5)

    r_h12 = p_h1 - p_h2
    r_h13 = p_h1 - p_h3
    r_h14 = p_h1 - p_h4
    r_h15 = p_h1 - p_h5
    r_h23 = p_h2 - p_h3
    r_h24 = p_h2 - p_h4
    r_h25 = p_h2 - p_h5
    r_h34 = p_h3 - p_h4
    r_h35 = p_h3 - p_h5
    r_h45 = p_h4 - p_h5

    d_h12 = np.linalg.norm(r_h12)
    d_h13 = np.linalg.norm(r_h13)
    d_h14 = np.linalg.norm(r_h14)
    d_h15 = np.linalg.norm(r_h15)
    d_h23 = np.linalg.norm(r_h23)
    d_h24 = np.linalg.norm(r_h24)
    d_h25 = np.linalg.norm(r_h25)
    d_h34 = np.linalg.norm(r_h34)
    d_h35 = np.linalg.norm(r_h35)
    d_h45 = np.linalg.norm(r_h45)

    epsilon_1 = 0.01
    eta_1 = 0.0001

    if d_h01 < epsilon_1:
        s = 200
        eta = np.linalg.norm(p_r1 - eta_1 * p_h1/d_h01)
    else:
        s = 1
        eta = np.linalg.norm(p_r1 - p_h1)
    J += s * eta ** 2

    if d_h02 < epsilon_1:
        s = 200
        eta = np.linalg.norm(p_r2 - eta_1 * p_h2/d_h02)
    else:
        s = 1
        eta = np.linalg.norm(p_r2 - p_h2)
    J += s * eta ** 2

    if d_h03 < epsilon_1:
        s = 200
        eta = np.linalg.norm(p_r3 - eta_1 * p_h3/d_h03)
    else:
        s = 1
        eta = np.linalg.norm(p_r3 - p_h3)
    J += s * eta ** 2

    if d_h04 < epsilon_1:
        s = 200
        eta = np.linalg.norm(p_r4 - eta_1 * p_h4/d_h04)
    else:
        s = 1
        eta = np.linalg.norm(p_r4 - p_h4)
    J += s * eta ** 2

    if d_h05 < epsilon_1:
        s = 200
        eta = np.linalg.norm(p_r5 - eta_1 * p_h5/d_h05)
    else:
        s = 1
        eta = np.linalg.norm(p_r5 - p_h5)
    J += s * eta ** 2


    epsilon_2 = 0.046
    eta_2 = 0.02
    if d_h12 < epsilon_2:
        s = 400
        eta = np.linalg.norm(r_r12 - eta_2 * r_h12/d_h12)
    else:
        s = 1
        eta = np.linalg.norm(r_r12 - r_h12)
    J += s * eta ** 2

    if d_h13 < epsilon_2:
        s = 400
        eta = np.linalg.norm(r_r13 - eta_2 * r_h13/d_h13)
    else:
        s = 1
        eta = np.linalg.norm(r_r13 - r_h13)
    J += s * eta ** 2

    if d_h14 < epsilon_2:
        s = 400
        eta = np.linalg.norm(r_r14 - eta_2 * r_h14/d_h14)
    else:
        s = 1
        eta = np.linalg.norm(r_r14 - r_h14)
    J += s * eta ** 2

    if d_h15 < epsilon_2:
        s = 400
        eta = np.linalg.norm(r_r15 - eta_2 * r_h15/d_h15)
    else:
        s = 1
        eta = np.linalg.norm(r_r15 - r_h15)
    J += s * eta ** 2

    if d_h23 < epsilon_2:
        s = 400
        eta = np.linalg.norm(r_r23 - eta_2 * r_h23/d_h23)
    else:
        s = 1
        eta = np.linalg.norm(r_r23 - r_h23)
    J += s * eta ** 2

    if d_h24 < epsilon_2:
        s = 400
        eta = np.linalg.norm(r_r24 - eta_2 * r_h24/d_h24)
    else:
        s = 1
        eta = np.linalg.norm(r_r24 - r_h24)
    J += s * eta ** 2

    if d_h25 < epsilon_2:
        s = 400
        eta = np.linalg.norm(r_r25 - eta_2 * r_h25/d_h25)
    else:
        s = 1
        eta = np.linalg.norm(r_r25 - r_h25)
    J += s * eta ** 2

    if d_h34 < epsilon_2:
        s = 400
        eta = np.linalg.norm(r_r34 - eta_2 * r_h34/d_h34)
    else:
        s = 1
        eta = np.linalg.norm(r_r34 - r_h34)
    J += s * eta ** 2

    if d_h35 < epsilon_2:
        s = 400
        eta = np.linalg.norm(r_r35 - eta_2 * r_h35/d_h35)
    else:
        s = 1
        eta = np.linalg.norm(r_r35 - r_h35)
    J += s * eta ** 2

    if d_h45 < epsilon_2:
        s = 400
        eta = np.linalg.norm(r_r45 - eta_2 * r_h45/d_h45)
    else:
        s = 1
        eta = np.linalg.norm(r_r45 - r_h45)
    J += s * eta ** 2

    return J

if __name__ == "__main__":
    ClientId = p.connect(p.GUI)

    p.setTimeStep(1./240)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    # Add world coordinates frame
    p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0.3, 0, 0], lineColorRGB=[1, 0, 0],
                       lineWidth=5, physicsClientId=ClientId)
    p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0, 0.3, 0], lineColorRGB=[0, 1, 0],
                       lineWidth=5, physicsClientId=ClientId)
    p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0, 0, 0.3], lineColorRGB=[0, 0, 1],
                       lineWidth=5, physicsClientId=ClientId)

    # reset camera configuration
    p.resetDebugVisualizerCamera(cameraDistance=0.4, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.2])
    shadow_hand = ShadowHand(ClientId)

    leap_tracker = LeapTracker()
    leap_tracker.connect('Desktop')

    wrist_deg = [0.0] * 2
    thumb_deg = [0.0] * 5
    index_deg = [0.0] * 4
    middle_deg = [0.0] * 4
    ring_deg = [0.0] * 4
    little_deg = [0.0] * 5

    x0 = np.zeros([24])
    x_bound = sp.optimize.Bounds(shadow_hand.q_l, shadow_hand.q_u, keep_feasible=True)

    p_palm = shadow_hand._palm_center_SE3[0:3, 3]
    T_palm = shadow_hand._palm_center_SE3
    beta = [1.0, 1.2, 1.2, 1.3, 1.7]
    while True:
        exist, right_tips_pos = leap_tracker.get_right_tips_pos()

        if exist:
            T_right_tips = []
            p_h = []
            for i in range(5):
                right_tip_pos = right_tips_pos[i] * beta[i]
                T_right_tip_pos = T_palm @ np.block([[np.identity(3), right_tip_pos.reshape(-1, 1) * 0.0014], [np.zeros([1, 3]), 1]])
                T_right_tips.append(T_right_tip_pos)
                p_h.append(T_right_tip_pos[0:3, 3] - p_palm)
            shadow_hand.add_debug_frames(T_right_tips + [T_palm])

            p_h = np.array(p_h)

            # print("*********************************************************")
            # print(l_r12, l_h12)

            f0 = lambda x: my_func(x, p_h=p_h)

            res = sp.optimize.minimize(f0, x0, bounds=x_bound, method='SLSQP', options={'disp': 0})

            q_opt = res.x * 180/np.pi
            x0 = res.x

            shadow_hand.move_joint_to(q_opt, degree=True)
            shadow_hand.forward_kinematics(q_opt, degree=True)

        time.sleep(0.001)
        p.stepSimulation()
