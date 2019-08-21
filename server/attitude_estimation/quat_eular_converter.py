# -*- coding: utf-8 -*-
import numpy as np
import quaternion


def quat2eular_rad(quat):
    """ convert quaternion to eular angle [rad]

    Parameters
    ----------------
        quat: Quaternion[w, x, y, z]

    Examples
    ----------------
    >>> quat2eular_rad(quaternion.quaternion(0.84462, 0.19134, 0.46194, 0.19134))
    (0.7853916625666502, 0.7853964001765581, 0.7853916625666502)


    """
    w = quat.w
    x = quat.x
    y = quat.y
    z = quat.z
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))

    return roll, pitch, yaw


def quat2eular_deg(quat):
    """ convert quaternion to eular angle [deg]

    Parameters
    ----------------
        quat: Quaternion[w, x, y, z]

    Examples
    ----------------
    >>> quat2eular_deg(quaternion.quaternion(0.84462, 0.19134, 0.46194, 0.19134))
    (44.99962752983194, 44.99989897488464, 44.99962752983194)

    """
    roll, pitch, yaw = quat2eular_rad(quat)
    roll = roll * 180 / np.pi
    pitch = pitch * 180 / np.pi
    yaw = yaw * 180 / np.pi
    return roll, pitch, yaw


if __name__ == '__main__':
    import doctest

    doctest.testmod()
