# -*- coding: utf-8 -*-
import time

import numpy as np
import quaternion
import subprocess

from attitude_estimation.quat_eular_converter import quat2eular_deg
from attitude_estimation import server_thread, madgwick_filter


def main_server():
    # ---------- quaternion madgwick ------------
    # Initialize
    # quat_mad = madgwick_filter.MadgwickFilter()

    # roll, pitch, yaw = quat2eular_deg(quat_mad.quaternion)
    # print(f'roll: {roll: >4.2f}  pitch: {pitch: >4.2f}  yaw: {yaw: >4.2f}')
    # ---------- quaternion madgwick ------------

    # ---------- eular complementary  ------------
    # Initialize
    # eular_comp = complementary_filter.ComplementaryFilter()
    #
    # rad = eular_comp.angle
    # deg = rad * 180 / np.pi
    # print(f'roll: {deg[0]: >.2f}  pitch: {deg[1]: >.2f}  yaw: {deg[2]: >.2f}')
    # ---------- eular complementary  ------------

    # ---------- quaternion madgwick clang ------------
    quat = quaternion.quaternion(1, 0, 0, 0)
    roll, pitch, yaw = quat2eular_deg(quat)
    print(f'roll: {roll: >4.2f}  pitch: {pitch: >4.2f}  yaw: {yaw: >4.2f}')

    # ---------- quaternion madgwick clang ------------

    # initial dt
    st = 1e-4
    # initial value
    ini_val = np.array([[0.01, 0.01, 1],
                        [1e-4, 1e-4, 1e-4],
                        [30, 0, 0]])

    # ---------- madgwick c filter ------------
    # wx, wy, wz = [0.01, 0.01, 1.0]
    # ax, ay, az = [1e-4, 1e-4, 1e-4]
    # mx, my, mz = [30, 0, 0]
    gyr, acc, mag = ini_val
    wx, wy, wz = map(str, gyr)
    ax, ay, az = map(str, acc)
    mx, my, mz = map(str, mag)
    filter_func = './attitude_estimation/madgwick_filter'

    qw = str(quat.w)
    qx = str(quat.x)
    qy = str(quat.y)
    qz = str(quat.z)
    dt = '0.02'
    cmd = [filter_func, wx, wy, wz, ax, ay, az, mx, my, mz, qw, qx, qy, qz, dt]
    # ---------- madgwick c filter ------------

    # create server thread
    sock = server_thread.ServerThreadUDP(port=50009, ini_val=ini_val)
    sock.setDaemon(True)
    sock.start()

    try:
        while True:
            gyr = sock.data[0, :]
            acc = sock.data[1, :]
            mag = sock.data[2, :]

            # ---------- madgwick filter ------------
            # # measure interval time
            # dt = time.time() - st
            # # update attitude
            # quat_mad.update_attitude(gyroscope=gyr,
            #                          accelerometer=acc,
            #                          magnetometer=mag,
            #                          dt=dt)
            # st = time.time()
            # roll, pitch, yaw = quat2eular_deg(quat_mad.quaternion)
            # print(f'roll: {roll: >4.2f}  pitch: {pitch: >4.2f}  yaw: {yaw: >4.2f}')
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
            # print(f'roll: {roll: >4.2f}  pitch: {pitch: >4.2f}  yaw: {yaw: >4.2f}')
            # ---------- complementary filter ------------

            # ---------- madgwick c filter ------------
            # wx, wy, wz = map(str, gyr)
            # ax, ay, az = map(str, acc)
            # mx, my, mz = map(str, mag)

            cmd[1:4] = map(str, gyr)
            cmd[4:7] = map(str, acc)
            cmd[7:10] = map(str, mag)

            dt = time.time() - st
            cmd[-1] = str(dt)
            proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            st = time.time()
            res = proc.stdout.decode("utf8")
            res_list = res.split()
            cmd[10:-1] = res_list
            quat.w, quat.x, quat.y, quat.z = map(float, res_list)
            roll, pitch, yaw = quat2eular_deg(quat)

            print(f'roll: {roll: >4.2f}  pitch: {pitch: >4.2f}  yaw: {yaw: >4.2f}')

            time.sleep(0.0005)
    except KeyboardInterrupt:
        sock.join()


if __name__ == '__main__':
    main_server()
