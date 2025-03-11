import leap
from leap.events import Event, TrackingEvent
from leap.datatypes import Hand, HandType

import time
import cv2
import numpy as np

from threading import Thread, Lock


class LeapHand:
    def __init__(self, hand_type:str):
        self.hand_type = hand_type # 'left' or 'right'

        self.exist = False

        self._thumb_pos: list = [np.zeros(3)] * 4
        self._index_pos: list = [np.zeros(3)] * 5
        self._middle_pos: list = [np.zeros(3)] * 5
        self._ring_pos: list = [np.zeros(3)] * 5
        self._little_pos: list = [np.zeros(3)] * 5

    def update_states(self, thumb_pos, index_pos, middle_pos, ring_pos, little_pos):
        for i in range(4):
            self._thumb_pos[i] = thumb_pos[i]
        for i in range(5):
            self._index_pos[i] = index_pos[i]
        for i in range(5):
            self._middle_pos[i] = middle_pos[i]
        for i in range(5):
            self._ring_pos[i] = ring_pos[i]
        for i in range(5):
            self._little_pos[i] = little_pos[i]

    def get_all_states(self):
        thumb_pos = self.thumb_pos
        index_pos = self.index_pos
        middle_pos = self.middle_pos
        ring_pos = self.ring_pos
        little_pos = self.little_pos

        thumb_joint = self.get_thumb_joint()
        index_joint = self.get_index_joint()
        middle_joint = self.get_middle_joint()
        ring_joint = self.get_ring_joint()
        little_joint = self.get_little_joint()

    def get_palm_norm(self):
        palm_joint = np.array([self.thumb_pos[0], self.index_pos[0], self.middle_pos[0], self.ring_pos[0], self.little_pos[0],
                               self.little_pos[1], self.ring_pos[1], self.middle_pos[1], self.index_pos[1]]) * 0.001

        if self.exist:
            palm_norm = np.linalg.solve(palm_joint.T @ palm_joint, palm_joint.T @ np.ones([9, 1]))
            palm_norm = palm_norm / np.linalg.norm(palm_norm)
        else:
            palm_norm = np.zeros(3)

        return palm_norm

    def get_thumb_joint(self) -> np.ndarray:
        thumb_joint: list = [0.0] * 4

        vec1 = self.thumb_pos[1] - self.thumb_pos[0]
        vec2 = self.index_pos[1] - self.index_pos[0]
        thumb_joint[0] = np.arccos(np.sum(vec1 * vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))

        vec1 = self.get_palm_norm()
        vec2 = self.thumb_pos[1] - self.thumb_pos[0]
        thumb_joint[1] = np.arccos(np.sum(vec1 * vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))

        joint_pos = self.thumb_pos
        for joint_idx in range(len(joint_pos) - 2):
            vec1 = joint_pos[joint_idx + 1] - joint_pos[joint_idx + 0]
            vec2 = joint_pos[joint_idx + 2] - joint_pos[joint_idx + 1]

            thumb_joint[joint_idx+2] = np.arccos(np.sum(vec1 * vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))
        return np.array(thumb_joint)

    def get_index_joint(self) -> np.ndarray:
        index_joint: list = [0.0] * 3
        joint_pos = self.index_pos
        for joint_idx in range(len(joint_pos) - 2):
            vec1 = joint_pos[joint_idx + 1] - joint_pos[joint_idx + 0]
            vec2 = joint_pos[joint_idx + 2] - joint_pos[joint_idx + 1]

            index_joint[joint_idx] = np.arccos(np.sum(vec1 * vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))
        return np.array(index_joint)

    def get_middle_joint(self) -> np.ndarray:
        middle_joint: list = [0.0] * 3
        joint_pos = self.middle_pos
        for joint_idx in range(len(joint_pos) - 2):
            vec1 = joint_pos[joint_idx + 1] - joint_pos[joint_idx + 0]
            vec2 = joint_pos[joint_idx + 2] - joint_pos[joint_idx + 1]

            middle_joint[joint_idx] = np.arccos(np.sum(vec1 * vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))
        return np.array(middle_joint)

    def get_ring_joint(self) -> np.ndarray:
        ring_joint: list = [0.0] * 3
        joint_pos = self.ring_pos
        for joint_idx in range(len(joint_pos) - 2):
            vec1 = joint_pos[joint_idx + 1] - joint_pos[joint_idx + 0]
            vec2 = joint_pos[joint_idx + 2] - joint_pos[joint_idx + 1]

            ring_joint[joint_idx] = np.arccos(np.sum(vec1 * vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))
        return np.array(ring_joint)

    def get_little_joint(self) -> np.ndarray:
        little_joint: list = [0.0] * 3
        joint_pos = self.little_pos
        for joint_idx in range(len(joint_pos) - 2):
            vec1 = joint_pos[joint_idx + 1] - joint_pos[joint_idx + 0]
            vec2 = joint_pos[joint_idx + 2] - joint_pos[joint_idx + 1]

            little_joint[joint_idx] = np.arccos(np.sum(vec1 * vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))
        return np.array(little_joint)

    @property
    def thumb_pos(self) -> np.ndarray:
        return np.array(self._thumb_pos) * 10 # cm -> mm

    @property
    def index_pos(self) -> np.ndarray:
        return np.array(self._index_pos) * 10

    @property
    def middle_pos(self) -> np.ndarray:
        return np.array(self._middle_pos) * 10

    @property
    def ring_pos(self) -> np.ndarray:
        return np.array(self._ring_pos) * 10

    @property
    def little_pos(self) -> np.ndarray:
        return np.array(self._little_pos) * 10


class LeapData:
    def __init__(self):
        self.hands = {'left': LeapHand('left'), 'right': LeapHand('right')}

        self._tracking_frame_id: int = 0
        self._framerate: float = 0.0
        self._num_hands: int = 0

    def update_data(self, event:TrackingEvent) -> None:
        self._tracking_frame_id = event.tracking_frame_id
        self._framerate = event.framerate
        self._num_hands = len(event.hands)

        exist = {'left': False, 'right': False}

        for i in range(self._num_hands):
            hand: Hand = event.hands[i]
            hand_type = self._get_hand_type(hand)

            target_hand: LeapHand = self.hands[hand_type]
            exist[hand_type] = True
            # digits: [self.thumb, self.index, self.middle, self.ring, self.pinky]
            # bones: [self.metacarpal, self.proximal, self.intermediate, self.distal]

            # thumb
            digit = hand.digits[0]
            thumb_pos = []
            for joint_idx, bone_idx in enumerate(range(1, 4)): # There are no useful information in metacarpal of thumb
                bone = digit.bones[bone_idx]
                prev_joint = bone.prev_joint
                thumb_pos.append(np.array([prev_joint.x, prev_joint.y, prev_joint.z]))
                if bone_idx == 3:
                    next_joint = bone.next_joint
                    thumb_pos.append(np.array([next_joint.x, next_joint.y, next_joint.z]))

            # index, middle, ring, little
            imrl_pos = []
            for digit_idx in range(1, 5):
                digit = hand.digits[digit_idx]
                joint_pos = []
                for joint_idx, bone_idx in enumerate(range(0, 4)):
                    bone = digit.bones[bone_idx]
                    prev_joint = bone.prev_joint
                    joint_pos.append(np.array([prev_joint.x, prev_joint.y, prev_joint.z]))
                    if bone_idx == 3:
                        next_joint = bone.next_joint
                        joint_pos.append(np.array([next_joint.x, next_joint.y, next_joint.z]))
                imrl_pos.append(joint_pos)

            target_hand.update_states(thumb_pos, *imrl_pos)

        # update existance
        self.hands['left'].exist = exist['left']
        self.hands['right'].exist = exist['right']

    def _get_hand_type(self, hand: Hand) -> str:
        if hand.type == HandType.Left:
            return 'left'
        elif hand.type == HandType.Right:
            return 'right'
        else:
            raise NotImplementedError


class LeapCanvas:
    def __init__(self):
        self.name = "Python Gemini Visualiser"
        self.screen_size = [500, 700]
        self.hands_colour = (255, 255, 255)
        self.font_colour = (0, 255, 44)
        self.output_image = np.zeros((self.screen_size[0], self.screen_size[1], 3), np.uint8)
        self.tracking_mode = None

    def get_joint_position(self, pos):
        return int(pos[0] + (self.screen_size[1] / 2)), int(pos[2] + (self.screen_size[0] / 2))

    def render_hands(self, leap_data:LeapData):
        # Clear the previous image
        self.output_image[:, :] = 0
        for hand_type in ['left', 'right']:
            if leap_data.hands[hand_type].exist:
                digit = leap_data.hands[hand_type].thumb_pos / 10
                for joint_idx in range(len(digit)):
                    cv2.circle(self.output_image, self.get_joint_position(digit[joint_idx]), 2, self.hands_colour, -1)
                digit = leap_data.hands[hand_type].index_pos / 10
                for joint_idx in range(len(digit)):
                    cv2.circle(self.output_image, self.get_joint_position(digit[joint_idx]), 2, self.hands_colour, -1)
                digit = leap_data.hands[hand_type].middle_pos / 10
                for joint_idx in range(len(digit)):
                    cv2.circle(self.output_image, self.get_joint_position(digit[joint_idx]), 2, self.hands_colour, -1)
                digit = leap_data.hands[hand_type].ring_pos / 10
                for joint_idx in range(len(digit)):
                    cv2.circle(self.output_image, self.get_joint_position(digit[joint_idx]), 2, self.hands_colour, -1)
                digit = leap_data.hands[hand_type].little_pos / 10
                for joint_idx in range(len(digit)):
                    cv2.circle(self.output_image, self.get_joint_position(digit[joint_idx]), 2, self.hands_colour, -1)


class TrackingListener(leap.Listener):
    def __init__(self, leap_data:LeapData, canvas:LeapCanvas):
        self.leap_data = leap_data
        self.canvas = canvas

    def on_connection_event(self, event:Event) -> None:
        print("on_connection_event")

    def on_tracking_mode_event(self, event:Event) -> None:
        print("on_tracking_mode_event")

    def on_device_event(self, event:Event) -> None:
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()

        print(f"Found device {info.serial}")

    def on_tracking_event(self, event:TrackingEvent) -> None:
        self.leap_data.update_data(event)
        self.canvas.render_hands(self.leap_data)


class LeapTracker:
    def __init__(self) -> None:
        self._leap_data = LeapData()
        self._leap_canvas = LeapCanvas()

        self._leap_connection = None
        self._leap_tracking_listener = None
        self._leap_tracking_mode = None

    def connect(self, tracking_mode:str='Desktop') -> None:
        self._leap_tracking_listener = TrackingListener(self._leap_data, self._leap_canvas)

        self._leap_connection = leap.Connection()
        self._leap_connection.add_listener(self._leap_tracking_listener)
        self._leap_tracking_mode = self._str_to_leap_tracking_mode(tracking_mode)

        # Run core thread

        time.sleep(1)

        self._leap_run = True
        self._thread = Thread(target=self._thread_func, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._leap_run = False
        print("Disconnect LeapMotion ...")

    def _thread_func(self):
        with self._leap_connection.open():
            self._leap_connection.set_tracking_mode(self._leap_tracking_mode)
            print("Connect LeapMotion ...")

            while self._leap_run:
                cv2.imshow(self._leap_canvas.name, self._leap_canvas.output_image)
                key = cv2.waitKey(1)

    def _str_to_leap_tracking_mode(self, tracking_mode:str) -> leap.TrackingMode:
        if tracking_mode == "Desktop":
            leap_tracking_mode = leap.TrackingMode.Desktop
        elif tracking_mode == "HMD":
            leap_tracking_mode = leap.TrackingMode.HMD
        elif tracking_mode == "ScreenTop":
            leap_tracking_mode = leap.TrackingMode.ScreenTop
        else:
            leap_tracking_mode = leap.TrackingMode.Desktop
        return leap_tracking_mode

    # user functions
    def get_left_joints(self) -> tuple:

        exist = self._leap_data.hands['left'].exist

        thumb_joint  = self._leap_data.hands['left'].get_thumb_joint()
        index_joint  = self._leap_data.hands['left'].get_index_joint()
        middle_joint = self._leap_data.hands['left'].get_middle_joint()
        ring_joint   = self._leap_data.hands['left'].get_ring_joint()
        little_joint = self._leap_data.hands['left'].get_little_joint()

        left_joints = (thumb_joint, index_joint, middle_joint, ring_joint, little_joint)

        return exist, left_joints

    def test(self) -> tuple:

        exist = self._leap_data.hands['left'].exist

        thumb_pos = self._leap_data.hands['left'].thumb_pos
        index_pos = self._leap_data.hands['left'].index_pos
        middle_pos = self._leap_data.hands['left'].middle_pos
        ring_pos = self._leap_data.hands['left'].ring_pos
        little_pos = self._leap_data.hands['left'].little_pos

        palm_joint = np.array([thumb_pos[0], index_pos[0], middle_pos[0], ring_pos[0], little_pos[0],
                               little_pos[1], ring_pos[1], middle_pos[1], index_pos[1]]) * 0.001
        palm_pos = np.mean(palm_joint, axis=0) * 1000

        if exist:
            palm_norm = np.linalg.solve(palm_joint.T @ palm_joint, palm_joint.T @ np.ones([9, 1]))
            palm_norm = palm_norm / np.linalg.norm(palm_norm)
        else:
            palm_norm = np.zeros(3)

        return exist, thumb_pos, index_pos, middle_pos, ring_pos, little_pos, palm_pos, palm_norm




if __name__ == "__main__":
    leap_tracker = LeapTracker()
    leap_tracker.connect('Desktop')

    for i in range(1000):
        res = leap_tracker.get_left_joints()
        time.sleep(0.1)
