import dynamixel_sdk as dxl
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time


def dh_transform(theta, d, a, alpha):
    """Returns the DH transformation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(theta1, theta2, theta3, theta4):
    """Computes T04 and T05 for the given joint angles."""
    # DH parameters
    d1 = 0.05
    a1, a2, a3, a4, a5 = 0, 0.093, 0.093, 0.05, 0.035
    alpha1, alpha2, alpha3, alpha4 = np.pi / 2, 0, 0, 0
    d5_y = 0.045

    # Transformation matrices for each joint
    A1 = dh_transform(theta1, d1, a1, alpha1)
    A2 = dh_transform(theta2, 0, a2, alpha2)
    A3 = dh_transform(theta3, 0, a3, alpha3)
    A4 = dh_transform(theta4, 0, a4, alpha4)
    A5i = dh_transform(theta4, 0, a5, 0)

    Y5 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, d5_y],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    A5 = np.dot(A5i, Y5)

    # Forward kinematics
    T04 = np.dot(np.dot(np.dot(A1, A2), A3), A4)
    T05 = np.dot(np.dot(np.dot(A1, A2), A3), A5)

    return T04, T05


def inverse_kinematics(p_desired):
    """
    Calculate the inverse kinematics for a 4-DOF robotic arm.

    Parameters:
    p_desired: array-like, shape (3,)
        Desired position of the end-effector [x, y, z].
    a2: float
        Length of the second link.
    a3: float
        Length of the third link.
    a4: float
        Length of the fourth link.
    d1: float
        Height offset from the base to the first joint.

    Returns:
    q1, q2, q3, q4: float
        Joint angles in radians.
    """
    d1 = 0.05
    a2, a3, a4 = 0.093, 0.093, 0.05
    # Extract the desired coordinates
    x4, y4, z4 = p_desired

    # Angle in the xy plane
    q1 = np.arctan2(y4, x4)

    x3 = x4 - np.cos(q1) * a4
    y3 = y4 - np.sin(q1) * a4
    z3 = z4

    d = z3
    s = d - d1
    r = np.sqrt(x3**2 + y3**2)

    # Elbow up configuration
    beta = np.arctan2(s, r)
    i = np.sqrt(r**2 + s**2)
    alpha = np.arccos((a3**2 - a2**2 - i**2) / (-2 * i * a2))
    
    q2 = beta - alpha

    t = s - a2 * np.cos(q2)
    u = r - a2 * np.sin(q2)

    q3 = np.arctan2(t, u)

    q4 = -(q2 + q3)

    return [q1, q2, q3, q4]
