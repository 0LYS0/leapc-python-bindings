from . import triad_openvr
import time

from threading import Thread, Lock

from src.utils.Toolbox import *
from src.utils.Toolbox.print_utils import *

def lock_thread(func):
    def decorated(*args, **kwargs):
        args[0]._lock_thread.acquire()
        func_out = func(*args, **kwargs)
        args[0]._lock_thread.release()
        return func_out

    return decorated

class ViveController():

    def __init__(self, auto_reconnect=True):

        np.set_printoptions(linewidth=500)
        np.set_printoptions(suppress=True)
        np.set_printoptions(precision=4)

        self._dt = 1 / 250

        self._vive_core = None

        self._lock_thread = Lock()
        self._is_running = False

        self._T_zero = [None, None]
        self._T_curr = [None, None]
        self._T_prev = [None, None]
        self._dT_curr = [None, None]

        self._state_trigger_curr = [0, 0]
        self._state_trigger_prev = [0, 0]
        self._state_trackpad_xy = [np.zeros(2), np.zeros(2)]
        self._state_trackpad_curr = [0, 0]
        self._state_trackpad_prev = [0, 0]
        self._state_grip_button_curr = [0, 0]
        self._state_grip_button_prev = [0, 0]
        self._state_grip_button_time = [0, 0]
        self._state_menu_button_curr = [0, 0]
        self._state_menu_button_prev = [0, 0]

        self._auto_reconnect = auto_reconnect

        self._T_offset = np.array([[ 0, -1,  0,  0],
                                   [ 0,  0,  1,  0],
                                   [-1,  0,  0,  0],
                                   [ 0,  0,  0,  1]])

        self._T_offset = self._T_offset @ xyzeul2SE3([0, 0, 0], [0, 60, 0], seq='XYZ', degree=True)

    def connect(self):
        PRINT_BLUE("Connecting to Vive...")
        self._vive_core = triad_openvr.triad_openvr()
        self._vive_core.print_discovered_objects()

        # Run core thread
        self._is_running = True
        self._thread = Thread(target=self._thread_main, daemon=True)
        self._thread.start()

    def disconnect(self):
        PRINT_BLUE("Disconnecting from Vive...")
        self._is_running = False


    def reset(self):
        pass

    def _thread_main(self):

        while self._is_running:
            ts = time.time()

            matrix_1 = self._vive_core.devices["controller_1"].get_pose_matrix()
            matrix_2 = self._vive_core.devices["controller_2"].get_pose_matrix()
            matrix = [matrix_1, matrix_2]

            inputs_1 = self._vive_core.devices["controller_1"].get_controller_inputs()
            inputs_2 = self._vive_core.devices["controller_2"].get_controller_inputs()
            inputs = [inputs_1, inputs_2]

            for i in range(2):
                if matrix[i] is not None:
                    self._process_controller_state(matrix[i], id=i)

                if inputs[i] is not None:
                    self._process_controller_input(inputs[i], id=i)

                if (matrix[i] is not None) and (inputs[i] is not None):
                    if self._state_trigger_curr[i] == 1:
                        self._T_zero[i] = self._T_curr[i].copy()

                    if self._state_trigger_curr[i] == 3:
                        self._T_zero[i] = None

            tf = time.time()
            if tf-ts < self._dt:
                time.sleep(self._dt-tf+ts)

        if self._auto_reconnect:
            self._vive_core = triad_openvr.triad_openvr()
            self._is_running = True

    def _process_controller_state(self, m, id):

        # SE3 = xyzquat2SE3(xyzquat[0:3], [xyzquat[6], xyzquat[3], xyzquat[4], xyzquat[5]]) # wxyz to xyzw
        # SE3 = xyzeul2SE3(xyzeul[0:3], xyzeul[3:6], seq='ZYX', degree=True)
        SE3 = np.array([[m[0][0], m[0][1], m[0][2], m[0][3]],
                        [m[1][0], m[1][1], m[1][2], m[1][3]],
                        [m[2][0], m[2][1], m[2][2], m[2][3]],
                        [0, 0, 0, 1]])


        if self._T_prev[id] is None:
            self._T_curr[id] = SE3.copy()

        self._T_prev[id] = self._T_curr[id].copy()
        self._T_curr[id] = SE3.copy()
        return

    def _process_controller_input(self, inputs, id):

        # trigger button
        # 0: OFF -> OFF, 1: OFF -> ON, 2: ON -> ON, 3: ON -> OFF
        trigger_state = inputs["trigger"]
        trigger_pressed = (trigger_state > 0.99)

        self._state_trigger_prev[id] = self._state_trigger_curr[id]
        if (not trigger_pressed) and (self._state_trigger_prev[id] in [0, 3]):
            self._state_trigger_curr[id] = 0
        elif trigger_pressed and (self._state_trigger_prev[id] in [0, 3]):
            self._state_trigger_curr[id] = 1
        elif trigger_pressed and (self._state_trigger_prev[id] in [1, 2]):
            self._state_trigger_curr[id] = 2
        elif (not trigger_pressed) and (self._state_trigger_prev[id] in [1, 2]):
            self._state_trigger_curr[id] = 3
        else:
            raise Exception("Invalid input (trigger)")

        # trackpad
        # 0: OFF -> OFF, 1: OFF -> ON, 2: ON -> ON, 3: ON -> OFF
        self._state_trackpad_xy[id][0] = inputs["trackpad_x"]
        self._state_trackpad_xy[id][1] = inputs["trackpad_y"]

        trackpad_pressed = inputs["trackpad_pressed"]

        self._state_trackpad_prev[id] = self._state_trackpad_curr[id]
        if (not trackpad_pressed) and (self._state_trackpad_prev[id] in [0, 3]):
            self._state_trackpad_curr[id] = 0
        elif trackpad_pressed and (self._state_trackpad_prev[id] in [0, 3]):
            self._state_trackpad_curr[id] = 1
        elif trackpad_pressed and (self._state_trackpad_prev[id] in [1, 2]):
            self._state_trackpad_curr[id] = 2
        elif (not trackpad_pressed) and (self._state_trackpad_prev[id] in [1, 2]):
            self._state_trackpad_curr[id] = 3
        else:
            raise Exception("Invalid input (trackpad)")

        # grip button
        # 0: OFF -> OFF, 1: OFF -> ON, 2: ON -> ON, 3: ON -> OFF, 4: ON -> ON (> 2sec)
        grip_button_pressed = inputs["grip_button"]

        self._state_grip_button_prev[id] = self._state_grip_button_curr[id]
        if (not grip_button_pressed) and (self._state_grip_button_prev[id] in [0, 3]):
            self._state_grip_button_curr[id] = 0
        elif grip_button_pressed and (self._state_grip_button_prev[id] in [0, 3]):
            self._state_grip_button_curr[id] = 1
        elif grip_button_pressed and (self._state_grip_button_prev[id] in [1, 2, 4]):
            self._state_grip_button_curr[id] = 2
        elif (not grip_button_pressed) and (self._state_grip_button_prev[id] in [1, 2, 4]):
            self._state_grip_button_curr[id] = 3
        else:
            raise Exception("Invalid input (grip button)")

        if self._state_grip_button_curr[id] == 1:
            self._state_grip_button_time[id] = time.time()
        if (self._state_grip_button_curr[id] == 2) and ((time.time() - self._state_grip_button_time[id]) > 2.0):
            self._state_grip_button_curr[id] = 4

        # menu button
        # 0: OFF -> OFF, 1: OFF -> ON, 2: ON -> ON, 3: ON -> OFF
        menu_button_pressed = inputs["menu_button"]

        self._state_menu_button_prev[id] = self._state_menu_button_curr[id]
        if (not menu_button_pressed) and (self._state_menu_button_curr[id] in [0, 3]):
            self._state_menu_button_curr[id] = 0
        elif menu_button_pressed and (self._state_menu_button_curr[id] in [0, 3]):
            self._state_menu_button_curr[id] = 1
        elif menu_button_pressed and (self._state_menu_button_curr[id] in [1, 2]):
            self._state_menu_button_curr[id] = 2
        elif (not menu_button_pressed) and (self._state_menu_button_curr[id] in [1, 2]):
            self._state_menu_button_curr[id] = 3
        else:
            raise Exception("Invalid input (menu button)")

        return

    @lock_thread
    def get_absolute_SE3(self, id):
        if self._T_curr[id] is None:
            PRINT_RED("Cannot get controller's pose. No controller connected.")
            return None

        return self._T_curr[id] @ self._T_offset

    @lock_thread
    def get_relative_SE3(self, id):
        if self._T_curr[id] is None:
            PRINT_RED("Cannot get controller's pose. No controller connected.")
            return None

        if self._T_zero[id] is None:
            PRINT_RED("Please press the trigger button!!!")
            return None

        return TransInv(self._T_zero[id] @ self._T_offset) @ (self._T_curr[id] @ self._T_offset)

    @lock_thread
    def get_vive_inputs(self):

        vive_inputs = {"controller_1":{}, "controller_2":{}}

        for id in range(2):
            vive_inputs["controller_{}".format(id+1)]["trigger"] = self._state_trigger_curr[id]
            vive_inputs["controller_{}".format(id+1)]["trackpad"] = self._state_trackpad_curr[id]
            vive_inputs["controller_{}".format(id+1)]["trackpad_xy"] = self._state_trackpad_xy[id]
            vive_inputs["controller_{}".format(id+1)]["grip_button"] = self._state_grip_button_curr[id]
            vive_inputs["controller_{}".format(id+1)]["menu_button"] = self._state_menu_button_curr[id]

        # PRINT_BLACK("Trigger", self._state_trigger_curr)
        # PRINT_BLACK("Trackpad", self._state_trackpad_xy)
        # PRINT_BLACK("Trackpad", self._state_trackpad_curr)
        # PRINT_BLACK("Grip Button", self._state_grip_button_curr)

        return vive_inputs
