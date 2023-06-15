from copy import deepcopy

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from ..defs import EULER_CONVENTION, RPY_CONVENTION


def matrix_to_quaternion(rotation):
    try:
        return Rot.from_matrix(rotation).as_quat()
    except:
        return Rot.from_dcm(rotation).as_quat()
    

def quaternion_to_matrix(quat):
    try:
        return Rot.from_quat(quat).as_matrix()
    except:
        return Rot.from_quat(quat).as_dcm()


def quaternion_to_angle_axis(quat):
    rotvec = Rot.from_quat(quat).as_rotvec()
    angle = np.linalg.norm(rotvec)
    axis = np.array(rotvec)/np.linalg.norm(rotvec) if np.linalg.norm(rotvec) != 0 else np.array((0., 0., 1.))
    return angle, axis


def rpy_to_quaternion(rotation):
    return Rot.from_euler(EULER_CONVENTION, rotation).as_quat()


def quaternion_to_rpy(quaternion):
    if type(quaternion) == dict:
        quaternion = [quaternion["x"], quaternion["y"],  quaternion["z"], quaternion["w"]]
    angles = Rot.from_quat(quaternion).as_euler(EULER_CONVENTION)
    return order_angles(angles, EULER_CONVENTION, RPY_CONVENTION)


def rpy_to_matrix(rpy):
    euler = order_angles(rpy, RPY_CONVENTION, EULER_CONVENTION)
    try:
        return Rot.from_euler(EULER_CONVENTION, euler).as_matrix()
    except:
        return Rot.from_euler(EULER_CONVENTION, euler).as_dcm()


def matrix_to_rpy(R):
    try:
        angles = Rot.from_matrix(R).as_euler(EULER_CONVENTION)
    except:
        angles = Rot.from_dcm(R).as_euler(EULER_CONVENTION)
    return order_angles(angles, EULER_CONVENTION, RPY_CONVENTION)


def skew_symmetric(x):
    return np.array([[0, -x[2], -x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])


def angle_between_vectors(a, b, acute=True):
    a = np.array(deepcopy(a))
    b = np.array(deepcopy(b))
    angle = np.arccos(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    if acute:
        return angle
    else:
        return 2 * np.pi - angle


def order_angles(angles, in_order, out_order):
    axes = {"x": angles[in_order.lower().index("x")], "y": angles[in_order.lower().index("y")],
            "z": angles[in_order.lower().index("z")]}
    return [axes[k] for k in out_order.lower()]


def round_array(array, dec=16):
    out = []
    for x in array:
        if 1*10**-(dec+1) > x > -1*10**-(dec+1):
            out.append(0.0)
        else:
            out.append(x)
    return np.around(out, decimals=dec)


def create_transformation(xyz=None, rpy=None):
    if rpy is None:
        rpy = [0, 0, 0]
    if xyz is None:
        xyz = [0, 0, 0]
    R = rpy_to_matrix(rpy)
    p = np.array(xyz)
    T = np.zeros((4, 4))
    T[0:3, 3] = p
    T[0:3, 0:3] = R
    T[3, 3] = 1.0
    return T


def inv(T):
    return np.linalg.inv(T)


def get_adjoint(T: np.ndarray):
    R = T[0:3, 0:3]
    P = skew_symmetric(T[0:3, 3])
    Ad = np.zeros((6, 6))
    Ad[0:3, 0:3] = R
    Ad[3::, 0:3] = np.dot(P, R)
    Ad[3::, 3::] = R
    return Ad
