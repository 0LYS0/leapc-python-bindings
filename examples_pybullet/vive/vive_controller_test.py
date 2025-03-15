from . import triad_openvr
import time
import sys
import os

sys.path.append(os.getcwd() + "/../../../..")

from src.utils.Toolbox import *

np.set_printoptions(linewidth=500)
np.set_printoptions(suppress=True)
np.set_printoptions(precision=4)

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

if len(sys.argv) == 1:
    interval = 1 / 250
elif len(sys.argv) == 2:
    interval = 1 / float(sys.argv[1])
else:
    print("Invalid number of arguments")
    interval = False

if interval:
    while (True):
        start = time.time()
        txt = ""
        # for each in v.devices["controller_1"].get_pose_euler():
        #     txt += "%.4f" % each
        #     txt += " "
        # print("\r" + txt, end="")

        xyzeul = v.devices["controller_1"].get_pose_quaternion()
        # SE3 = xyzeul2SE3(xyzeul[0:3], xyzeul[3:6], seq='XYZ', degree=True)
        print("\r", xyzeul, end="")
        # inputs = v.devices["controller_1"].get_controller_inputs()
        # print(inputs)

        sleep_time = interval - (time.time() - start)
        if sleep_time > 0:
            time.sleep(sleep_time)