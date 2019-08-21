# -*- coding: utf-8 -*-
import numpy as np
import quaternion


class MadgwickFilter(object):

    def __init__(self, beta=1):
        """
        Prameters
        ----------------
            beta: float 0 < beta < 1, step size

        """
        self.beta = beta
        self.quaternion = quaternion.quaternion(1, 0, 0, 0)

    def update_attitude(self, gyroscope, accelerometer, magnetometer, dt=0.02):  # TODO dt 調整
        """ Madgwick Filter

        Parameters
        ----------------
            gyroscope: like [x, y, z]
            accelerometer: like [x, y, z]
            magnetometer: like [x, y, z]
            dt: delta t, computing time

        Examples
        ----------------
        >>> mad = MadgwickFilter()
        >>> dt = 0.02
        >>> gyr, acc, mag = [[0.01, 0.01, 1], [0.01, 0.01, 0.01], [1, 1, 30]]
        >>> mad.update_attitude(gyroscope=gyr, accelerometer=acc, magnetometer=mag, dt=dt)
        >>> mad.quaternion.w
        0.999750347680268
        >>> mad.quaternion.x
        0.014793139803496492
        >>> mad.quaternion.y
        -0.01346135929396912
        >>> mad.quaternion.z
        0.009959775795469635

        """

        qw = self.quaternion.w
        qx = self.quaternion.x
        qy = self.quaternion.y
        qz = self.quaternion.z

        wx, wy, wz = gyroscope
        ax, ay, az = accelerometer
        mx, my, mz = magnetometer

        # normalise the accelerometer measurement
        norm = np.sqrt(ax * ax + ay * ay + az * az)
        ax /= norm
        ay /= norm
        az /= norm

        # normalise the magnetometer measurement
        norm = np.sqrt(mx * mx + my * my + mz * mz)
        mx /= norm
        my /= norm
        mz /= norm

        # compute rotation matrix
        a1 = qw * qw - 0.5
        r11 = a1 + qx * qx
        r22 = a1 + qy * qy
        r33 = a1 + qz * qz
        a1 = qx * qy
        a2 = qw * qz
        r21 = a1 + a2
        r12 = a1 - a2
        a1 = qx * qz
        a2 = qw * qy
        r13 = a1 + a2
        r31 = a1 - a2
        a1 = qy * qz
        a2 = qw * qx
        r32 = a1 + a2
        r23 = a1 - a2

        r11 += r11
        r12 += r12
        r13 += r13
        r21 += r21
        r22 += r22
        r23 += r23
        r31 += r31
        r32 += r32
        r33 += r33

        # rotate m to earth frame and compute b
        a1 = r11 * mx + r12 * my + r13 * mz
        a2 = r21 * mx + r22 * my + r23 * mz
        bx = np.sqrt(a1 * a1 + a2 * a2)
        bz = r31 * mx + r32 * my + r33 * mz

        # compute J_g ^ T * f_g to compute qe
        f1 = r31 - ax
        f2 = r32 - ay
        f3 = r33 - az
        a1 = qx * f3
        a2 = qy * f3
        qew = -qy * f1 + qx * f2
        qex = qz * f1 + qw * f2 - a1 - a1
        qey = -qw * f1 + qz * f2 - a2 - a2
        qez = qx * f1 + qy * f2

        # compute J_b ^ T * f_b to compute qe
        f1 = r11 * bx + r31 * bz - mx
        f2 = r12 * bx + r32 * bz - my
        f3 = r13 * bx + r33 * bz - mz
        a1 = qw * bx
        a2 = qx * bx
        a3 = qy * bx
        a4 = qz * bx
        a5 = qw * bz
        a6 = qx * bz
        a7 = qy * bz
        a8 = qz * bz
        qew += -a7 * f1 + (a6 - a4) * f2 + a3 * f3
        qex += a8 * f1 + (a3 + a5) * f2 + (a4 - a6 - a6) * f3
        qey += (-a5 - a3 - a3) * f1 + (a2 + a8) * f2 + (a1 - a7 - a7) * f3
        qez += (a6 - a4 - a4) * f1 + (a7 - a1) * f2 + a2 * f3

        # normalise qe
        norm = np.sqrt(qew * qew + qex * qex + qey * qey + qez * qez)
        qew /= norm
        qex /= norm
        qey /= norm
        qez /= norm

        # compute omega_b
        # ジャイロのバイアスエラーを取り除く場合、ここに処理を記述

        # compute q_dot_omega
        a1 = -qx * wx - qy * wy - qz * wz
        a2 = qw * wx - qz * wy + qy * wz
        a3 = qz * wx + qw * wy - qx * wz
        a4 = -qy * wx + qx * wy + qw * wz
        a1 /= 2
        a2 /= 2
        a3 /= 2
        a4 /= 2

        # compute q_dot
        beta = self.beta
        a1 -= beta * qew
        a2 -= beta * qex
        a3 -= beta * qey
        a4 -= beta * qez

        # compute q
        qw += a1 * dt
        qx += a2 * dt
        qy += a3 * dt
        qz += a4 * dt

        # normalise q
        norm = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        qw /= norm
        qx /= norm
        qy /= norm
        qz /= norm

        # assign quaternion
        self.quaternion.w = qw
        self.quaternion.x = qx
        self.quaternion.y = qy
        self.quaternion.z = qz


if __name__ == '__main__':
    import doctest
    doctest.testmod()
