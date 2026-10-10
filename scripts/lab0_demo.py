"""
Date: 08/26/2021

Purpose: This script creates an ArmController and uses it to command the arm's
joint positions and gripper.

Try changing the target position to see what the arm does!

"""

import sys
import rospy
import numpy as np
from math import pi

import _bootstrap  # noqa: F401  (puts src/ on sys.path)

from core.interfaces import ArmController

rospy.init_node('demo')

arm = ArmController()
arm.set_arm_speed(0.2)

arm.close_gripper()

q = arm.neutral_position()
arm.safe_move_to_position(q)
arm.open_gripper()

q = np.array([0.5,-1.6,-0.9,-0.56,1.1,0.75,-1.25]) # TODO: try changing this!
arm.safe_move_to_position(q)
arm.close_gripper()
