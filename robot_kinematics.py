import dynamixel_sdk as dxl
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time
import math
from scipy.optimize import minimize




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
    d1 = 50
    a1, a2, a3, a4, a5 = 0, 93, 93, 50, 35
    alpha1, alpha2, alpha3, alpha4 = np.pi / 2, 0, 0, 0
    d5_y = 45

    # Transformation matrices for each joint
    A1 = dh_transform(theta1, d1, a1, alpha1)
    A2 = dh_transform(np.pi/2+theta2, 0, a2, alpha2) #
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
    d1 = 50
    a2, a3, a4 = 93, 93, 50
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


def inverse_kinematics2(p_desired):
    """
    INPUTS:
    p_desired: list or tuple of size 3 representing desired position of the end-effector [x, y, z]
    a2: length of the second link
    a3: length of the third link
    a4: length of the fourth link
    d1: height offset from the base to the first joint
    
    OUTPUTS:
    q1, q2, q3, q4: Joint angles in radians
    """
    a2, a3, a4, d1 = 93, 93, 50, 50
    # Extract the desired coordinates
    x, y, z = p_desired
    l_desired = math.sqrt(x**2 + y**2 + z**2)

    # Solve for q1 (rotation around the z-axis, in the xy-plane)
    q1 = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2)
    s = z - d1
    # sin_alfa = abs(z) / l_desired
    sin_alfa = 0
    x_prime = r - a4 * (1 - sin_alfa**2)
    y_prime = s - a4 * sin_alfa

    # Solve for q2 and q3 using the geometric approach
    # Law of cosines to solve for q3
    D = (x_prime**2 + y_prime**2 - a2**2 - a3**2) / (2 * a2 * a3)
    q3 = math.atan2(math.sqrt(1 - D**2), D)  # Two solutions, choosing the elbow-up

    # Law of sines or another geometric relation to solve for q2
    phi = math.atan2(y_prime, x_prime)  # angle of the triangle in the z-r plane
    beta = math.atan2(a3 * math.sin(q3), a2 + a3 * math.cos(q3))  # angle at joint 2

    if q3 > 0:
        q2 = phi - beta - np.pi / 2
    else:
        q2 = phi + beta - np.pi / 2

    # For q4, we keep the stylus horizontal (end-effector orientation)
    # Assuming horizontal means q4 = 0
    q4 = math.atan2(s, r) - q2 - q3 - np.pi / 2

    return q1, q2, q3, q4

def numeric_inverse_function(P,current_Q = [0,0,0,0]):
    import numpy as np
    from scipy.optimize import fsolve
    a1, a2, a3 = 93, 93, 50
    d0 = 50
    z = P[2]
    x = np.linalg.norm([P[0],P[1]])
    if P[0] > 1e-13:
        q0 = np.arctan(P[1]/P[0])
    else:
        q0 = 0

    # Define the equations
    def equations(vars):
        q1, q2, q3 = vars
        # t1, t2, t3 = vars
        # q1,q2,q3 = -t1,-t2,-t3
        eq1 = a1 * np.sin(-q1) + a2 * np.sin(-q1 - q2) + a3 * np.sin(-q1 - q2 - q3) - x
        eq2 = d0 + a1 * np.cos(-q1) + a2 * np.cos(-q1 - q2) + a3 * np.cos(-q1  - q2 - q3) - z
        # eq1 = a1 * np.sin(q1) + a2 * np.sin(q1 + q2) + a3 * np.sin(q1 + q2 + q3) + x
        # eq2 = d0 + a1 * np.cos(q1) + a2 * np.cos(q1 + q2) + a3 * np.cos(q1  + q2 + q3) - z
        return np.abs(eq1)+np.abs(eq2)


    # Initial guess for the angles (in radians)

    # Solve the equations
    # [q1,q2,q3] = fsolve(equations, current_Q[1:])
    result = minimize(equations, x0 = np.array(-current_Q[1:]),bounds=[(-np.pi/2,0),(-np.pi/2,1),(-np.pi/2,1)])
    # result = minimize(equations, x0 = np.array([-np.pi/4,-np.pi/4,-np.pi/4]),bounds=[(-np.pi/2,0),(-np.pi/2,1),(-np.pi/2,1)])
    # result = minimize(equations, x0=np.array(current_Q[1:]), bounds=[(0,np.pi / 2), (0,np.pi / 2), (0,np.pi / 2)])
    print(result.fun)
    print(result.success)
    # Output the solution
    return np.append(q0, result.x)
