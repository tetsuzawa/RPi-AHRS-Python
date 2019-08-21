# -*- coding: utf-8 -*-
import numpy as np


class ComplementaryFilter(object):
    def __init__(self):
        self.angle = np.zeros(3)  # [roll, pitch, yaw]

    def update_attitude(self, gyroscope, accelerometer, dt=0.02):
        """ Madgwick Filter

        Parameters
        ----------------
            gyroscope: like [x, y, z]
            accelerometer: like [x, y, z]
            dt: delta t, computing time

        Examples
        ----------------
        >>> mad = ComplementaryFilter()
        >>> dt = 0.02
        >>> gyr, acc = [[0.01, 0.01, 1], [0.01, 0.01, 0.01]]
        >>> mad.update_attitude(gyroscope=gyr, accelerometer=acc, dt=dt)
        >>> mad.angle
        array([0.00069, 0.00069, 0.0195 ])

        """

        gyr = np.array(gyroscope)
        acc = np.array(accelerometer)
        self.angle = 0.95 * (self.angle + gyr * dt) + 0.05 * acc


if __name__ == '__main__':
    import doctest

    doctest.testmod()
