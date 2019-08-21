# -*- coding: utf-8 -*-
import time

import numpy as np

from attitude_estimation.quat2eular import quat2eular_deg
from attitude_estimation import server_thread, madgwick_filter


def main():
    # ---------- quaternion madgwick ------------
    # Initialize
    quat_mad = madgwick_filter.MadgwickFilter()

    roll, pitch, yaw = quat2eular_deg(quat_mad.quaternion)
    print(f'roll: {roll:.2f}  pitch: {pitch:.2f}  yaw: {yaw:.2f}')
    # ---------- quaternion madgwick ------------

    # ---------- eular complementary  ------------
    # Initialize
    # eular_comp = complementary_filter.ComplementaryFilter()
    #
    # rad = eular_comp.angle
    # deg = rad * 180 / np.pi
    # print(f'roll: {deg[0]:.2f}  pitch: {deg[1]:.2f}  yaw: {deg[2]:.2f}')
    # ---------- eular complementary  ------------

    # initial dt
    st = 1e-4
    # initial value
    ini_val = np.array([[0.01, 0.01, 1],
                        [1e-4, 1e-4, 1e-4],
                        [30, 0, 0]])

    # create server thread
    sock = server_thread.ServerThreadUDP(port=50009, ini_val=ini_val)
    sock.setDaemon(True)
    sock.start()

    try:
        while True:
            # ---------- madgwick filter ------------
            gyr = sock.data[0, :]
            acc = sock.data[1, :]
            mag = sock.data[2, :]

            # measure interval time
            dt = time.time() - st
            # update attitude
            quat_mad.update_attitude(gyroscope=gyr,
                                     accelerometer=acc,
                                     magnetometer=mag,
                                     dt=dt)
            st = time.time()
            roll, pitch, yaw = quat2eular_deg(quat_mad.quaternion)
            print(f'roll: {roll:.2f}  pitch: {pitch:.2f}  yaw: {yaw:.2f}')
            # ---------- madgwick filter ------------

            # ---------- complementary filter ------------
            # measure interval time
            # dt = time.time() - st
            # update attitude
            # eular_comp.update_attitude(gyroscope=gyr, accelerometer=acc, dt=dt)
            # st = time.time()
            # rad = eular_comp.angle
            # deg = rad * 180 / np.pi
            # roll, pitch, yaw = deg
            # print(f'roll: {roll:.2f}  pitch: {pitch:.2f}  yaw: {yaw:.2f}')
            # ---------- complementary filter ------------

            time.sleep(0.0005)
    except KeyboardInterrupt:
        sock.join()


if __name__ == '__main__':
    main()
