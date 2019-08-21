#!/usr/bin/python3 -u
# -*- coding: utf-8 -*-
import datetime  # datetimeモジュールの呼び出し
import time  # timeライブラリの呼び出し
import socket
import numpy as np
from concurrent.futures import ThreadPoolExecutor

import wiringpi as wi  # wiringPiモジュールの呼び出し

# データ計測時間は　SAMPLING_TIME x TIMES
SAMPLING_TIME = 0.005  # データ取得の時間間隔[sec]
TIMES = 3000  # データの計測回数

wi.wiringPiSetup()  # wiringPiの初期化
i2c = wi.I2C()  # i2cの初期化

address = 0x68
addrAK8963 = 0x0C  # 磁気センサAK8963 アドレス
mpu9250 = i2c.setup(address)  # i2cアドレス0x68番地をmpu9250として設定(アドレスは$sudo i2cdetect 1で見られる)
AK8963 = i2c.setup(addrAK8963)
gyroRange = 1000  # 250, 500, 1000, 2000　'dps'から選択
accelRange = 8  # +-2, +-4, +-8, +-16 'g'から選択
magRange = 4912  # 'μT'

# センサ定数
REG_PWR_MGMT_1 = 0x6B
REG_INT_PIN_CFG = 0x37
REG_ACCEL_CONFIG1 = 0x1C
REG_ACCEL_CONFIG2 = 0x1D
REG_GYRO_CONFIG = 0x1B

MAG_MODE_POWERDOWN = 0  # 磁気センサpower down
MAG_MODE_SERIAL_1 = 1  # 磁気センサ8Hz連続測定モード
MAG_MODE_SERIAL_2 = 2  # 磁気センサ100Hz連続測定モード
MAG_MODE_SINGLE = 3  # 磁気センサ単発測定モード
MAG_MODE_EX_TRIGER = 4  # 磁気センサ外部トリガ測定モード
MAG_MODE_SELF_TEST = 5  # 磁気センサセルフテストモード
MAG_ACCESS = False  # 磁気センサへのアクセス可否
MAG_MODE = 0  # 磁気センサモード
MAG_BIT = 16  # 磁気センサが出力するbit数

# オフセット用変数
offsetAccelX = 0
offsetAccelY = 0
offsetAccelZ = 0
offsetGyroX = 0
offsetGyroY = 0
offsetGyroZ = 0


# レジスタを初期設定に戻す。
def resetRegister():
    global MAG_ACCESS
    if MAG_ACCESS == True:
        i2c.writeReg8(AK8963, 0x0B, 0x01)
    i2c.writeReg8(mpu9250, 0x6B, 0x80)
    MAG_ACCESS = False
    time.sleep(0.1)


# センシング可能な状態にする。
def powerWakeUp():
    # PWR_MGMT_1をクリア
    i2c.writeReg8(mpu9250, REG_PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    # I2Cで磁気センサ機能(AK8963)へアクセスできるようにする(BYPASS_EN=1)
    i2c.writeReg8(mpu9250, REG_INT_PIN_CFG, 0x02)
    global MAG_ACCESS
    MAG_ACCESS = True
    time.sleep(0.1)


# 加速度の測定レンジを設定
# val = 16, 8, 4, 2(default)
val = 8


def setAccelRange(val, _calibration=False):
    # +-2g (00), +-4g (01), +-8g (10), +-16g (11)
    if val == 16:
        accelRange = 16
        _data = 0x18
    elif val == 8:
        accelRange = 8
        _data = 0x10
    elif val == 4:
        accelRange = 4
        _data = 0x08
    else:
        accelRange = 2
        _data = 0x00
    print("set accelRange=%d [g]" % accelRange)
    i2c.writeReg8(mpu9250, REG_ACCEL_CONFIG1, _data)
    accelCoefficient = accelRange / float(0x8000)
    time.sleep(0.1)

    # オフセット値をリセット
    # offsetAccelX       = 0
    # offsetAccelY       = 0
    # offsetAccelZ       = 0

    # Calibration
    if _calibration == True:
        calibAccel(1000)
    return


# ジャイロの測定レンジを設定します。
# val= 2000, 1000, 500, 250(default)
def setGyroRange(val, _calibration=False):
    if val == 2000:
        gyroRange = 2000
        _data = 0x18
    elif val == 1000:
        gyroRange = 1000
        _data = 0x10
    elif val == 500:
        gyroRange = 500
        _data = 0x08
    else:
        gyroRange = 250
        _data = 0x00
    print("set gyroRange=%d [dps]" % gyroRange)
    i2c.writeReg8(mpu9250, REG_GYRO_CONFIG, _data)
    gyroCoefficient = gyroRange / float(0x8000)
    time.sleep(0.1)

    # Reset offset value (so that the past offset value is not inherited)
    # offsetGyroX        = 0
    # offsetGyroY        = 0
    # offsetGyroZ        = 0

    # Calibration
    if _calibration == True:
        calibGyro(1000)
    return


# 磁気センサのレジスタを設定する
def setMagRegister(_mode, _bit):
    global MAG_ACCESS
    global MAG_MODE
    if MAG_ACCESS == False:
        # 磁気センサへのアクセスが有効になっていない場合は例外
        raise Exception('001 Access to a sensor is invalid.')

    _writeData = 0x00
    # 測定モードの設定
    if _mode == '8Hz':  # Continuous measurement mode 1
        _writeData = 0x02
        MAG_MODE = MAG_MODE_SERIAL_1
    elif _mode == '100Hz':  # Continuous measurement mode 2
        _writeData = 0x06
        MAG_MODE = MAG_MODE_SERIAL_2
    elif _mode == 'POWER_DOWN':  # Power down mode
        _writeData = 0x00
        MAG_MODE = MAG_MODE_POWERDOWN
    elif _mode == 'EX_TRIGER':  # Trigger measurement mode
        _writeData = 0x04
        MAG_MODE = MAG_MODE_EX_TRIGER
    elif _mode == 'SELF_TEST':  # self test mode
        _writeData = 0x08
        MAG_MODE = MAG_MODE_SELF_TEST
    else:  # _mode='SINGLE'    # single measurment mode
        _writeData = 0x01
        MAG_MODE = MAG_MODE_SINGLE

    # 出力するbit数
    if _bit == '14bit':  # output 14bit
        _writeData = _writeData | 0x00
        MAG_BIT = 14
    else:  # _bit='16bit'      # output 16bit
        _writeData = _writeData | 0x10
        MAG_BIT = 16
    print("set MAG_MODE=%s, %d bit" % (_mode, MAG_BIT))
    i2c.writeReg8(AK8963, 0x0A, _writeData)


# センサからのデータはそのまま使おうとするとunsignedとして扱われるため、signedに変換(16ビット限定）
def u2s(unsigneddata):
    if unsigneddata & (0x01 << 15):
        return -1 * ((unsigneddata ^ 0xffff) + 1)
    return unsigneddata


# 加速度値を取得
def getAccel():
    ACCEL_XOUT_H = i2c.readReg8(mpu9250, 0x3B)
    ACCEL_XOUT_L = i2c.readReg8(mpu9250, 0x3C)
    ACCEL_YOUT_H = i2c.readReg8(mpu9250, 0x3D)
    ACCEL_YOUT_L = i2c.readReg8(mpu9250, 0x3E)
    ACCEL_ZOUT_H = i2c.readReg8(mpu9250, 0x3F)
    ACCEL_ZOUT_L = i2c.readReg8(mpu9250, 0x40)
    rawX = accelCoefficient * u2s(ACCEL_XOUT_H << 8 | ACCEL_XOUT_L) + offsetAccelX
    rawY = accelCoefficient * u2s(ACCEL_YOUT_H << 8 | ACCEL_YOUT_L) + offsetAccelY
    rawZ = accelCoefficient * u2s(ACCEL_ZOUT_H << 8 | ACCEL_ZOUT_L) + offsetAccelZ
    # data    = i2c.readReg8(address, 0x3B )
    # print "getaccell data=%d"%data
    # rawX    = accelCoefficient * u2s(data[0] << 8 | data[1]) + offsetAccelX
    # rawY    = accelCoefficient * u2s(data[2] << 8 | data[3]) + offsetAccelY
    # rawZ    = accelCoefficient * u2s(data[4] << 8 | data[5]) + offsetAccelZ
    return rawX, rawY, rawZ


# ジャイロ値を取得
def getGyro():
    GYRO_XOUT_H = i2c.readReg8(mpu9250, 0x43)
    GYRO_XOUT_L = i2c.readReg8(mpu9250, 0x44)
    GYRO_YOUT_H = i2c.readReg8(mpu9250, 0x45)
    GYRO_YOUT_L = i2c.readReg8(mpu9250, 0x46)
    GYRO_ZOUT_H = i2c.readReg8(mpu9250, 0x47)
    GYRO_ZOUT_L = i2c.readReg8(mpu9250, 0x48)
    rawX = gyroCoefficient * u2s(GYRO_XOUT_H << 8 | GYRO_XOUT_L) + offsetGyroX
    rawY = gyroCoefficient * u2s(GYRO_YOUT_H << 8 | GYRO_YOUT_L) + offsetGyroY
    rawZ = gyroCoefficient * u2s(GYRO_ZOUT_H << 8 | GYRO_ZOUT_L) + offsetGyroZ
    # data    =  i2c.readReg8(address, 0x43 )
    # rawX    = gyroCoefficient * u2s(data[0] << 8 | data[1]) + offsetGyroX
    # rawY    = gyroCoefficient * u2s(data[2] << 8 | data[3]) + offsetGyroY
    # rawZ    = gyroCoefficient * u2s(data[4] << 8 | data[5]) + offsetGyroZ
    return rawX, rawY, rawZ


# 磁気値を取得
def getMag():
    global MAG_ACCESS
    if MAG_ACCESS == False:
        # 磁気センサへのアクセスが有効になっていない場合は例外
        raise Exception('002 Access to a sensor is invalid.')

    # 事前処理
    global MAG_MODE
    if MAG_MODE == MAG_MODE_SINGLE:
        # 単発測定モードは測定終了と同時にPower Downになるので、もう一度モードを変更する
        if MAG_BIT == 14:  # output 14bit
            _writeData = 0x01
        else:  # output 16bit
            _writeData = 0x11
        i2c.writeReg8(AK8963, 0x0A, _writeData)
        time.sleep(0.01)

    elif MAG_MODE == MAG_MODE_SERIAL_1 or MAG_MODE == MAG_MODE_SERIAL_2:
        status = i2c.readReg8(AK8963, 0x02)
        if (status & 0x02) == 0x02:
            # if (status[0] & 0x02) == 0x02:
            # データオーバーランがあるので再度センシング
            i2c.readReg8(AK8963, 0x09)

    elif MAG_MODE == MAG_MODE_EX_TRIGER:
        # 未実装
        return

    elif MAG_MODE == MAG_MODE_POWERDOWN:
        raise Exception('003 Mag sensor power down')

    # ST1レジスタを確認してデータ読み出しが可能か確認する
    status = i2c.readReg8(AK8963, 0x02)
    while (status & 0x01) != 0x01:
        # while (status[0] & 0x01) != 0x01:
        # Wait until data ready state.
        # time.sleep(0.01)
        status = i2c.readReg8(AK8963, 0x02)

    # データ読み出し
    MAG_XOUT_L = i2c.readReg8(AK8963, 0x03)
    MAG_XOUT_H = i2c.readReg8(AK8963, 0x04)
    MAG_YOUT_L = i2c.readReg8(AK8963, 0x05)
    MAG_YOUT_H = i2c.readReg8(AK8963, 0x06)
    MAG_ZOUT_L = i2c.readReg8(AK8963, 0x07)
    MAG_ZOUT_H = i2c.readReg8(AK8963, 0x08)
    MAG_OF = i2c.readReg8(AK8963, 0x09)
    rawX = u2s(MAG_XOUT_H << 8 | MAG_XOUT_L)
    rawY = u2s(MAG_YOUT_H << 8 | MAG_YOUT_L)
    rawZ = u2s(MAG_ZOUT_H << 8 | MAG_ZOUT_L)
    st2 = MAG_OF
    # data    = i2c.readReg8(addrAK8963, 0x03 ,7)
    # rawX    = u2s(data[1] << 8 | data[0])  # Lower bit is ahead.
    # rawY    = u2s(data[3] << 8 | data[2])  # Lower bit is ahead.
    # rawZ    = u2s(data[5] << 8 | data[4])  # Lower bit is ahead.
    # st2     = data[6]

    # オーバーフローチェック
    if (st2 & 0x08) == 0x08:
        # オーバーフローのため正しい値が得られていない
        raise Exception('004 Mag sensor over flow')

    # μTへの変換
    if MAG_BIT == 16:  # output 16bit
        rawX = rawX * magCoefficient16
        rawY = rawY * magCoefficient16
        rawZ = rawZ * magCoefficient16
    else:  # output 14bit
        rawX = rawX * magCoefficient14
        rawY = rawY * magCoefficient14
        rawZ = rawZ * magCoefficient14

    # ---- offset by myself ----
    rawX -= 7.49733614
    rawY -= 57.46132325
    rawZ -= -63.09610185

    return rawX, rawY, rawZ


# 加速度センサを較正する
# 本当は緯度、高度、地形なども考慮する必要があるとは思うが、簡略で。
# z軸方向に正しく重力がかかっており、重力以外の加速度が発生していない前提
def calibAccel(_count=1000):
    print("Accel calibration start")
    _sum = [0, 0, 0]

    # データのサンプルを取る
    for _ in range(_count):
        _data = getAccel()
        _sum[0] += _data[0]
        _sum[1] += _data[1]
        _sum[2] += _data[2]

    # 平均値をオフセットにする
    global offsetAccelX, offsetAccelY, offsetAccelZ
    offsetAccelX = -1.0 * _sum[0] / _count
    offsetAccelY = -1.0 * _sum[1] / _count
    # offsetAccelZ = -1.0 * _sum[2] / _count
    offsetAccelZ = -1.0 * ((_sum[2] / _count) - 1.0)  # 重力分を差し引く

    # I want to register an offset value in a register. But I do not know the behavior, so I will put it on hold.
    print("Accel calibration complete")
    print(f'Accel error X: {offsetAccelX:.2f}  Y: {offsetAccelY:.2f}  Z: {offsetAccelZ:.2f}')
    return offsetAccelX, offsetAccelY, offsetAccelZ


# ジャイロセンサを較正する
# 各軸に回転が発生していない前提
def calibGyro(_count=1000):
    print("Gyro calibration start")
    _sum = [0, 0, 0]

    # データのサンプルを取る
    for _i in range(_count):
        _data = getGyro()
        _sum[0] += _data[0]
        _sum[1] += _data[1]
        _sum[2] += _data[2]

    # 平均値をオフセットにする
    global offsetGyroX, offsetGyroY, offsetGyroZ
    offsetGyroX = -1.0 * _sum[0] / _count
    offsetGyroY = -1.0 * _sum[1] / _count
    offsetGyroZ = -1.0 * _sum[2] / _count

    # I want to register an offset value in a register. But I do not know the behavior, so I will put it on hold.
    print("Gyro calibration complete")
    print(f'Gyro error X: {offsetGyroX:.2f}  Y: {offsetGyroY:.2f}  Z: {offsetGyroZ:.2f}')
    time.sleep(0.5)
    return offsetGyroX, offsetGyroY, offsetGyroZ


if __name__ == '__main__':

    # ---------- quaternion madgwick ------------

    qua_mad = madgwickahrs.MadgwickAHRS()

    # ---------- quaternion madgwick ------------

    # bus     = smbus.SMBus(1)
    resetRegister()
    powerWakeUp()
    gyroCoefficient = gyroRange / float(0x8000)  # coefficient : sensed decimal val to dps val.
    accelCoefficient = accelRange / float(0x8000)  # coefficient : sensed decimal val to g val
    magCoefficient16 = magRange / 32760.0  # confficient : sensed decimal val to μT val (16bit)
    magCoefficient14 = magRange / 8190.0  # confficient : sensed decimal val to μT val (14bit)
    setAccelRange(val=accelRange, _calibration=True)
    setGyroRange(val=gyroRange, _calibration=True)
    # setAccelRange(val=accelRange, _calibration=False)
    # setGyroRange(val=gyroRange, _calibration=False)
    setMagRegister('100Hz', '16bit')

    # ファイルへ書出し準備
    now = datetime.datetime.now()
    # 現在時刻を織り込んだファイル名を生成
    # fmt_name = "/home/pi/data/mpu9250wpi_logs_{0:%Y%m%d-%H%M%S}.csv".format(now)
    fmt_name = '/tmp/mpu9250_tizikilog.csv'
    f_mpu9250 = open(fmt_name, 'w')  # 書き込みファイル
    value = "yyyy-mm-dd hh:mm:ss.mmmmmm, x[g],y[g],z[g],x[dps],y[dps],z[dps],x[uT],y[uT],z[uT]"  # header行への書き込み内容
    print(value)
    f_mpu9250.write(value + "\n")  # header行をファイル出力
    # while True:
    analysis_time = time.time()
    gyr_res = np.zeros(3)
    acc_res = np.zeros(3)

    for _ in range(TIMES):
        with ThreadPoolExecutor(max_workers=10, thread_name_prefix='thread') as executor:
            # with ProcessPoolExecutor() as executor:
            gyr = executor.submit(getGyro)  # ジャイロ値の取得
            acc = executor.submit(getAccel)
            mag = executor.submit(getMag)  # 磁気値の取得

        # --- raw version ---

        # gyr_res = gyr.result()
        # gyr_res = np.array(gyr_res) * np.pi / 180
        # acc_res = acc.result()
        # mag_res = mag.result()
        # --- raw version ---

        # --- low, high pass version ---
        w = 0.5  # weight
        gyr_tmp = np.array(gyr.result())
        gyr_tmp = gyr_tmp * np.pi / 180
        gyr_low = gyr_res * w + (1 - w) * gyr_tmp
        gyr_res = gyr_tmp - gyr_low  # high pass

        acc_tmp = np.array(acc.result())
        acc_res = acc_res * w + (1 - w) * acc_tmp  # low pass

        mag_res = mag.result()
        # --- low, high pass version ---

        print(f'gyro: {gyr_res[0]:.4f} {gyr_res[1]:.4f} {gyr_res[2]:.4f}  \
        acc: {acc_res[0]:.4f} {acc_res[1]:.4f} {acc_res[2]:.4f}  \
        mag: {mag_res[0]:.1f} {mag_res[1]:.1f} {mag_res[2]:.1f}')
        # print(acc_res, gyr_res, mag_res)
        # gyr_res = [1e-8, 1e-8, 1e-8]
        # acc_res = [1e-8, 1e-8, 1e-8]
        # mag_res = (14.544078144078144, 36.435164835164834, -28.638339438339436)
        gy1, gy2, gy3 = map(str, gyr_res)
        ac1, ac2, ac3 = map(str, acc_res)
        mg1, mg2, mg3 = map(str, mag_res)

        ss = ''
        ss += ','.join(map(str, gyr_res))
        ss += ','
        ss += ','.join(map(str, acc_res))
        ss += ','
        ss += ','.join(map(str, mag_res))

        # --------------- serial -----------------
        """ UDP """
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            letter = ss.encode('utf-8')
            s.sendto(letter, ('192.168.0.5', 50009))

        # --------------- serial -----------------

    analysis_end_time = time.time() - analysis_time
    print('\nideal time: 5sec', 'executive time:', round(analysis_end_time, 1))
    f_mpu9250.close()  # 書き込みファイルを閉じる
