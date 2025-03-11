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

        self._joint_pos = [self._thumb_pos, self._index_pos, self._middle_pos, self._ring_pos, self._little_pos]

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
            for joint_idx, bone_idx in enumerate(range(1, 4)): # There are no useful information in metacarpal of thumb
                bone = digit.bones[bone_idx]
                prev_joint = bone.prev_joint
                target_hand._joint_pos[0][joint_idx] = np.array([prev_joint.x, prev_joint.y, prev_joint.z])
                if bone_idx == 3:
                    next_joint = bone.next_joint
                    target_hand._joint_pos[0][joint_idx+1] = np.array([next_joint.x, next_joint.y, next_joint.z])

            # index, middle, ring, little
            for digit_idx in range(1, 5):
                digit = hand.digits[digit_idx]
                for joint_idx, bone_idx in enumerate(range(0, 4)):
                    bone = digit.bones[bone_idx]
                    prev_joint = bone.prev_joint
                    target_hand._joint_pos[digit_idx][joint_idx] = np.array([prev_joint.x, prev_joint.y, prev_joint.z])
                    if bone_idx == 3:
                        next_joint = bone.next_joint
                        target_hand._joint_pos[digit_idx][joint_idx + 1] = np.array([next_joint.x, next_joint.y, next_joint.z])

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

    def test(self):
        test_1 = (self._leap_data.hands['left'].exist, self._leap_data.hands['right'].exist)
        test_2 = self._leap_data.hands['left'].index_pos
        return test_1, test_2

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


if __name__ == "__main__":
    leap_tracker = LeapTracker()
    leap_tracker.connect('Desktop')

    for i in range(10000):
        res = leap_tracker.test()
        time.sleep(0.1)
