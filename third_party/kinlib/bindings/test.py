from kinlib_py import Manipulator, JointType, JointLimits
import numpy as np

manip = Manipulator()

# Create joint limits
limits = JointLimits()
limits.lower_limit_ = -3.14
limits.upper_limit_ = 3.14

# Add joint
manip.addJoint(
    JointType.Revolute,
    "joint_1",
    np.array([0, 0, 1, 0]),   # joint axis
    np.array([0, 0, 0, 1]),   # joint position
    limits,
    np.eye(4)                 # tip pose
)

