import numpy as np
from math import atan2, cos, isnan, sin


def zero_on_nan(val):
    """
    :param val: Input value
    :type val:  float
    :return: 0.0 if ``val`` is nan, otherwise ``val``
    :rtype:  float
    """
    if isnan(val):
        return 0.0
    else:
        return val


def any_nan(mat):
    """
    :param mat:
    :return:
    :rtype:  bool
    """
    return np.any(np.isnan(mat))


def rotate_x(angle, dtype=np.float64, output=None):
    """Apply an X rotation to the matrix.

    :param angle:
    :param dtype:

    :param output:
    :type output:  np.ndarray, NoneType

    :return: 3x3 rotation matrix
    :rtype:  np.ndarray
    """
    c = cos(angle)
    s = sin(angle)
    if output is None:
        output = np.empty((3, 3), dtype=dtype)
    output[1, 1] = c
    output[2, 1] = s
    output[1, 2] = -s
    output[2, 2] = c
    output[0, 0] = 1.0
    output[1:3, 0] = output[0, 1:3] = 0.0
    return output


def rotate_y(angle, dtype=np.float64, output=None):
    """Apply a Y rotation to the matrix.

    :param angle:
    :param dtype:

    :param output:
    :type output:  np.ndarray, NoneType

    :return: 3x3 rotation matrix
    :rtype:  np.ndarray
    """
    c = cos(angle)
    s = sin(angle)
    if output is None:
        output = np.empty((3, 3), dtype=dtype)
    output[0, 0] = c
    output[0, 2] = s
    output[2, 0] = -s
    output[2, 2] = c
    output[1, 1] = 1.0
    output[0, 1] = output[1, 0] = output[1, 2] = output[2, 1] = 0.0
    return output


def rotate_z(angle, dtype=np.float64, output=None):
    """Apply a Z rotation to the matrix.

    :param angle:
    :param dtype:

    :param output:
    :type output:  np.ndarray, NoneType

    :return: 3x3 rotation matrix
    :rtype:  np.ndarray
    """
    c = cos(angle)
    s = sin(angle)
    if output is None:
        output = np.empty((3, 3), dtype=dtype)
    output[0, 0] = c
    output[0, 1] = -s
    output[1, 0] = s
    output[1, 1] = c
    output[2, 2] = 1.0
    output[2, 0:2] = output[0:2, 2] = 0.0
    return output


def quat2rot(quaternion, output=None):
    """Retrieve the rotation matrix for the provided rotation quaternion.

    :param quaternion:
    :param output:
    :type output:  np.ndarray, NoneType
    :return:
    """

    if output is None:
        output = np.empty((3, 3), dtype=np.float64)
    X = quaternion[1]
    Y = quaternion[2]
    Z = quaternion[3]
    W = quaternion[0]

    xx = X * X
    xy = X * Y
    xz = X * Z
    xw = X * W

    yy = Y * Y
    yz = Y * Z
    yw = Y * W

    zz = Z * Z
    zw = Z * W

    output[0, 0] = 1.0 - 2.0 * (yy + zz)
    output[0, 1] = 2.0 * (xy - zw)
    output[0, 2] = 2.0 * (xz + yw)

    output[1, 0] = 2.0 * (xy + zw)
    output[1, 1] = 1.0 - 2.0 * (xx + zz)
    output[1, 2] = 2.0 * (yz - xw)

    output[2, 0] = 2.0 * (xz - yw)
    output[2, 1] = 2.0 * (yz + xw)
    output[2, 2] = 1.0 - 2.0 * (xx + yy)

    return output

def rot2ea(R, output=None):
    """Retrieve the Euler angle from the input rotation matrix.

    :param R:
    :param output:
    :type output:  np.ndarray, NoneType

    :return: 3 element array representing the Euler rotation
    :rtype:  np.ndarray
    """
    if output is None:
        output = np.empty(3, dtype=np.float64)

    sy = np.linalg.norm(R[0:2, 0])
    singular = sy < 1e-6

    if not singular:
        x = atan2(R[2, 1], R[2, 2])
        y = atan2(-R[2, 0], sy)
        z = atan2(R[1, 0], R[0, 0])
    else:
        output[0:3] = np.nan
        return output

    pi = np.pi
    n_pi2 = pi * -2.0
    if x > pi:
        x = x + n_pi2
    if y > pi:
        y = y + n_pi2
    if z > pi:
        z = z + n_pi2

    output[0] = x
    output[1] = y
    output[2] = z
    return output
