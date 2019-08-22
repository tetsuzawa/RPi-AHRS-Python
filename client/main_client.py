# -*- coding: utf-8 -*-
import time  # timeライブラリの呼び出し
import numpy as np
import socket
from concurrent.futures import ThreadPoolExecutor

from mpu9250.mpu9250 import MPU9250


def main_client(times, ip_addr):
    ahrs = MPU9250()

    # while True:
    gyr_res = np.zeros(3)
    acc_res = np.zeros(3)

    for _ in range(times):
        with ThreadPoolExecutor(max_workers=10, thread_name_prefix='thread') as executor:
            # with ProcessPoolExecutor() as executor:
            gyr = executor.submit(ahrs.get_gyro)  # ジャイロ値の取得
            acc = executor.submit(ahrs.get_accel)
            mag = executor.submit(ahrs.get_mag)  # 磁気値の取得

        # ------- raw version -------
        # gyr_res = gyr.result()
        # gyr_res = np.array(gyr_res) * np.pi / 180
        # acc_res = acc.result()
        # mag_res = mag.result()
        # ------- raw version -------

        # ------- low, high pass version -------
        w = 0.5  # weight

        # --- high pass filter ---
        gyr_tmp = np.array(gyr.result())
        gyr_tmp = gyr_tmp * np.pi / 180
        gyr_low = gyr_res * w + (1 - w) * gyr_tmp
        gyr_res = gyr_tmp - gyr_low  # high pass

        # --- low pass filter ---
        acc_tmp = np.array(acc.result())
        acc_res = acc_res * w + (1 - w) * acc_tmp  # low pass

        mag_res = mag.result()
        # ------- low, high pass version -------

        # --------------- print sensor data -----------------
        print(f'gyro: {gyr_res[0]: >2.4f} {gyr_res[1]: >2.4f} {gyr_res[2]: >2.4f}  \
        acc: {acc_res[0]: >2.4f} {acc_res[1]: >2.4f} {acc_res[2]: >2.4f}  \
        mag: {mag_res[0]: >4.1f} {mag_res[1]: >4.1f} {mag_res[2]: >4.1f}')
        # --------------- print sensor data -----------------

        ss = ''
        ss += ','.join(map(str, gyr_res))
        ss += ','
        ss += ','.join(map(str, acc_res))
        ss += ','
        ss += ','.join(map(str, mag_res))

        # --------------- serial communication -----------------
        """ UDP """
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            letter = ss.encode('utf-8')
            s.sendto(letter, (ip_addr, 50009))

        # --------------- serial communication -----------------


if __name__ == '__main__':
    times = 5000  # sample times
    ip_addr = '169.254.76.99'  # FILL YOUR IP ADDRESS
    main_client(times=times, ip_addr=ip_addr)
