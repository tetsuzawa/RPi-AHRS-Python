# -*- coding: utf-8 -*-
import time

import wiringpi as wi
import numpy as np
# from scipy.optimize import least_squares


class MPU9250(object):
    wi.wiringPiSetup()
    i2c = wi.I2C()

    _address = 0x68  # addresses of gyroscope and accelerometer
    _addr_AK8963 = 0x0C  # a address of magnetometer (self.AK8963)
    # sensor constant
    _REG_PWR_MGMT_1 = 0x6B
    _REG_INT_PIN_CFG = 0x37
    _REG_ACCEL_CONFIG1 = 0x1C
    _REG_ACCEL_CONFIG2 = 0x1D
    _REG_GYRO_CONFIG = 0x1B

    _MAG_MODE_POWERDOWN = 0  # 磁気センサpower down
    _MAG_MODE_SERIAL_1 = 1  # 磁気センサ8Hz連続測定モード
    _MAG_MODE_SERIAL_2 = 2  # 磁気センサ100Hz連続測定モード
    _MAG_MODE_SINGLE = 3  # 磁気センサ単発測定モード
    _MAG_MODE_EX_TRIGER = 4  # 磁気センサ外部トリガ測定モード
    _MAG_MODE_SELF_TEST = 5  # 磁気センサセルフテストモード
    _MAG_ACCESS = False  # 磁気センサへのアクセス可否
    _MAG_MODE = 0  # 磁気センサモード
    _MAG_BIT = 16  # 磁気センサが出力するbit数
    _gyro_range = 1000  # 250, 500, 1000, 2000　'dps'から選択
    _accel_range = 8  # +-2, +-4, +-8, +-16 'g'から選択
    _mag_range = 4912  # 'μT'

    mpu9250 = i2c.setup(_address)  # i2cアドレス0x68番地をmpu9250として設定(アドレスは$sudo i2cdetect 1で見られる)
    AK8963 = i2c.setup(_addr_AK8963)

    # 加速度の測定レンジを設定
    _val = 8  # val = 16, 8, 4, 2(default)

    def __init__(self):
        # オフセット用変数
        self._offset_gyro_x = 0
        self._offset_gyro_y = 0
        self._offset_gyro_z = 0
        self._offset_accel_x = 0
        self._offset_accel_y = 0
        self._offset_accel_z = 0
        self._offset_mag_x = -7.497336140
        self._offset_mag_y = -57.461323250
        self._offset_mag_z = 63.096101850

        self._reset_register()
        self._power_wakeup()
        self._gyro_coefficient = self._gyro_range / float(0x8000)  # coefficient : sensed decimal val to dps val.
        self._accel_coefficient = self._accel_range / float(0x8000)  # coefficient : sensed decimal val to g val
        self._mag_coefficient_16 = self._mag_range / 32760.0  # coefficient : sensed decimal val to μT val (16bit)
        self._mag_coefficient_14 = self._mag_range / 8190.0  # coefficient : sensed decimal val to μT val (14bit)
        self._set_accel_range(val=self._accel_range, _calibration=True)
        self._set_gyro_range(val=self._gyro_range, _calibration=True)
        # self._set_accel_range(val=self._accel_range, _calibration=False)
        # setself._gyro_range(val=self._gyro_range, _calibration=False)
        self._set_mag_register('100Hz', '16bit')

    # レジスタを初期設定に戻す。
    def _reset_register(self):
        if self._MAG_ACCESS is True:
            self.i2c.writeReg8(self.AK8963, 0x0B, 0x01)
        self.i2c.writeReg8(self.mpu9250, 0x6B, 0x80)
        self._MAG_ACCESS = False
        time.sleep(0.1)

    # センシング可能な状態にする。
    def _power_wakeup(self):
        # PWR_MGMT_1をクリア
        self.i2c.writeReg8(self.mpu9250, self._REG_PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # I2Cで磁気センサ機能(self.AK8963)へアクセスできるようにする(BYPASS_EN=1)
        self.i2c.writeReg8(self.mpu9250, self._REG_INT_PIN_CFG, 0x02)
        self._MAG_ACCESS = True
        time.sleep(0.1)

    def _set_accel_range(self, val, _calibration=False):
        # +-2g (00), +-4g (01), +-8g (10), +-16g (11)
        if val == 16:
            _accel_range = 16
            _data = 0x18
        elif val == 8:
            _accel_range = 8
            _data = 0x10
        elif val == 4:
            _accel_range = 4
            _data = 0x08
        else:
            _accel_range = 2
            _data = 0x00
        print("set _accel_range=%d [g]" % _accel_range)
        self.i2c.writeReg8(self.mpu9250, self._REG_ACCEL_CONFIG1, _data)
        self._accel_coefficient = _accel_range / float(0x8000)
        time.sleep(0.1)

        # Calibration
        if _calibration is True:
            self._calib_accel(1000)
        return

    # ジャイロの測定レンジを設定します。
    # val= 2000, 1000, 500, 250(default)
    def _set_gyro_range(self, val, _calibration=False):
        if val == 2000:
            _gyro_range = 2000
            _data = 0x18
        elif val == 1000:
            _gyro_range = 1000
            _data = 0x10
        elif val == 500:
            _gyro_range = 500
            _data = 0x08
        else:
            _gyro_range = 250
            _data = 0x00
        print("set _gyro_range=%d [dps]" % _gyro_range)
        self.i2c.writeReg8(self.mpu9250, self._REG_GYRO_CONFIG, _data)
        self._gyro_coefficient = _gyro_range / float(0x8000)
        time.sleep(0.1)

        # Calibration
        if _calibration is True:
            self._calib_gyro(1000)
        return

    # 磁気センサのレジスタを設定する
    def _set_mag_register(self, _mode, _bit, _calibration=False):
        if self._MAG_ACCESS is False:
            # 磁気センサへのアクセスが有効になっていない場合は例外
            raise Exception('001 Access to a sensor is invalid.')

        _writeData = 0x00
        # 測定モードの設定
        if _mode == '8Hz':  # Continuous measurement mode 1
            _writeData = 0x02
            self._MAG_MODE = self._MAG_MODE_SERIAL_1
        elif _mode == '100Hz':  # Continuous measurement mode 2
            _writeData = 0x06
            self._MAG_MODE = self._MAG_MODE_SERIAL_2
        elif _mode == 'POWER_DOWN':  # Power down mode
            _writeData = 0x00
            self._MAG_MODE = self._MAG_MODE_POWERDOWN
        elif _mode == 'EX_TRIGER':  # Trigger measurement mode
            _writeData = 0x04
            self._MAG_MODE = self._MAG_MODE_EX_TRIGER
        elif _mode == 'SELF_TEST':  # self test mode
            _writeData = 0x08
            self._MAG_MODE = self._MAG_MODE_SELF_TEST
        else:  # _mode='SINGLE'    # single measurment mode
            _writeData = 0x01
            self._MAG_MODE = self._MAG_MODE_SINGLE

        # 出力するbit数
        if _bit == '14bit':  # output 14bit
            _writeData = _writeData | 0x00
            self._MAG_BIT = 14
        else:  # _bit='16bit'      # output 16bit
            _writeData = _writeData | 0x10
            self._MAG_BIT = 16
        print("set self._MAG_MODE=%s, %d bit" % (_mode, self._MAG_BIT))
        self.i2c.writeReg8(self.AK8963, 0x0A, _writeData)
        time.sleep(0.1)

        # Calibration
        if _calibration is True:
            self._calib_mag(3000)
        return

    # センサからのデータはそのまま使おうとするとunsignedとして扱われるため、signedに変換(16ビット限定）
    def _u2s(self, unsigneddata):
        if unsigneddata & (0x01 << 15):
            return -1 * ((unsigneddata ^ 0xffff) + 1)
        return unsigneddata

    # 加速度値を取得
    def get_accel(self):
        mpu9250 = self.mpu9250
        i2c = self.i2c
        ACCEL_XOUT_H = i2c.readReg8(mpu9250, 0x3B)
        ACCEL_XOUT_L = i2c.readReg8(mpu9250, 0x3C)
        ACCEL_YOUT_H = i2c.readReg8(mpu9250, 0x3D)
        ACCEL_YOUT_L = i2c.readReg8(mpu9250, 0x3E)
        ACCEL_ZOUT_H = i2c.readReg8(mpu9250, 0x3F)
        ACCEL_ZOUT_L = i2c.readReg8(mpu9250, 0x40)
        raw_x = self._accel_coefficient * self._u2s(ACCEL_XOUT_H << 8 | ACCEL_XOUT_L) + self._offset_accel_x
        raw_y = self._accel_coefficient * self._u2s(ACCEL_YOUT_H << 8 | ACCEL_YOUT_L) + self._offset_accel_y
        raw_z = self._accel_coefficient * self._u2s(ACCEL_ZOUT_H << 8 | ACCEL_ZOUT_L) + self._offset_accel_z
        return raw_x, raw_y, raw_z

    # ジャイロ値を取得
    def get_gyro(self):
        mpu9250 = self.mpu9250
        i2c = self.i2c
        GYRO_XOUT_H = i2c.readReg8(mpu9250, 0x43)
        GYRO_XOUT_L = i2c.readReg8(mpu9250, 0x44)
        GYRO_YOUT_H = i2c.readReg8(mpu9250, 0x45)
        GYRO_YOUT_L = i2c.readReg8(mpu9250, 0x46)
        GYRO_ZOUT_H = i2c.readReg8(mpu9250, 0x47)
        GYRO_ZOUT_L = i2c.readReg8(mpu9250, 0x48)
        raw_x = self._gyro_coefficient * self._u2s(GYRO_XOUT_H << 8 | GYRO_XOUT_L) + self._offset_gyro_x
        raw_y = self._gyro_coefficient * self._u2s(GYRO_YOUT_H << 8 | GYRO_YOUT_L) + self._offset_gyro_y
        raw_z = self._gyro_coefficient * self._u2s(GYRO_ZOUT_H << 8 | GYRO_ZOUT_L) + self._offset_gyro_z
        return raw_x, raw_y, raw_z

    # 磁気値を取得
    def get_mag(self):
        AK8963 = self.AK8963
        i2c = self.i2c

        if self._MAG_ACCESS is False:
            # 磁気センサへのアクセスが有効になっていない場合は例外
            raise Exception('002 Access to a sensor is invalid.')

        # 事前処理
        if self._MAG_MODE == self._MAG_MODE_SINGLE:
            # 単発測定モードは測定終了と同時にPower Downになるので、もう一度モードを変更する
            if self._MAG_BIT == 14:  # output 14bit
                _writeData = 0x01
            else:  # output 16bit
                _writeData = 0x11
            self.i2c.writeReg8(self.AK8963, 0x0A, _writeData)
            time.sleep(0.01)

        elif self._MAG_MODE == self._MAG_MODE_SERIAL_1 or self._MAG_MODE == self._MAG_MODE_SERIAL_2:
            status = self.i2c.readReg8(self.AK8963, 0x02)
            if (status & 0x02) == 0x02:
                # if (status[0] & 0x02) == 0x02:
                # データオーバーランがあるので再度センシング
                self.i2c.readReg8(self.AK8963, 0x09)

        elif self._MAG_MODE == self._MAG_MODE_EX_TRIGER:
            # 未実装
            return

        elif self._MAG_MODE == self._MAG_MODE_POWERDOWN:
            raise Exception('003 Mag sensor power down')

        # ST1レジスタを確認してデータ読み出しが可能か確認する
        status = i2c.readReg8(AK8963, 0x02)
        while (status & 0x01) != 0x01:
            # while (status[0] & 0x01) != 0x01:
            # Wait until data ready state.
            # time.sleep(0.01)
            status = self.i2c.readReg8(self.AK8963, 0x02)

        # データ読み出し
        MAG_XOUT_L = i2c.readReg8(AK8963, 0x03)
        MAG_XOUT_H = i2c.readReg8(AK8963, 0x04)
        MAG_YOUT_L = i2c.readReg8(AK8963, 0x05)
        MAG_YOUT_H = i2c.readReg8(AK8963, 0x06)
        MAG_ZOUT_L = i2c.readReg8(AK8963, 0x07)
        MAG_ZOUT_H = i2c.readReg8(AK8963, 0x08)
        MAG_OF = i2c.readReg8(AK8963, 0x09)
        raw_x = self._u2s(MAG_XOUT_H << 8 | MAG_XOUT_L)
        raw_y = self._u2s(MAG_YOUT_H << 8 | MAG_YOUT_L)
        raw_z = self._u2s(MAG_ZOUT_H << 8 | MAG_ZOUT_L)
        st2 = MAG_OF
        # data    = self.i2c.readReg8(addrAK8963, 0x03 ,7)
        # raw_x    = u2s(data[1] << 8 | data[0])  # Lower bit is ahead.
        # raw_y    = u2s(data[3] << 8 | data[2])  # Lower bit is ahead.
        # raw_z    = u2s(data[5] << 8 | data[4])  # Lower bit is ahead.
        # st2     = data[6]

        # オーバーフローチェック
        if (st2 & 0x08) == 0x08:
            # オーバーフローのため正しい値が得られていない
            raise Exception('004 Mag sensor over flow')

        # μTへの変換
        if self._MAG_BIT == 16:  # output 16bit
            raw_x = raw_x * self._mag_coefficient_16
            raw_y = raw_y * self._mag_coefficient_16
            raw_z = raw_z * self._mag_coefficient_16
        else:  # output 14bit
            raw_x = raw_x * self._mag_coefficient_14
            raw_y = raw_y * self._mag_coefficient_14
            raw_z = raw_z * self._mag_coefficient_14

        # ---- offset by myself ----
        # raw_x -= 7.49733614
        # raw_y -= 57.46132325
        # raw_z -= -63.09610185
        # ---- offset by myself ----

        raw_x += self._offset_mag_x
        raw_y += self._offset_mag_y
        raw_z += self._offset_mag_z

        return raw_x, raw_y, raw_z

    # 加速度センサを較正する
    # 本当は緯度、高度、地形なども考慮する必要があるとは思うが、簡略で。
    # z軸方向に正しく重力がかかっており、重力以外の加速度が発生していない前提
    def _calib_accel(self, _count=1000):
        print("Accel calibration start")
        _sum = [0, 0, 0]

        # データのサンプルを取る
        for _ in range(_count):
            _data = self.get_accel()
            _sum[0] += _data[0]
            _sum[1] += _data[1]
            _sum[2] += _data[2]

        # 平均値をオフセットにする
        self._offset_accel_x = -1.0 * _sum[0] / _count
        self._offset_accel_y = -1.0 * _sum[1] / _count
        # self._offset_accel_z = -1.0 * _sum[2] / _count
        self._offset_accel_z = -1.0 * ((_sum[2] / _count) - 1.0)  # 重力分を差し引く

        # I want to register an offset value in a register. But I do not know the behavior, so I will put it on hold.
        print("Accel calibration complete")
        print(
            f'Accel error X: {self._offset_accel_x:.2f}  Y: {self._offset_accel_y:.2f}  Z: {self._offset_accel_z:.2f}')

    # ジャイロセンサを較正する
    # 各軸に回転が発生していない前提
    def _calib_gyro(self, _count=1000):
        print("Gyro calibration start")
        _sum = [0, 0, 0]

        # データのサンプルを取る
        for _i in range(_count):
            _data = self.get_gyro()
            _sum[0] += _data[0]
            _sum[1] += _data[1]
            _sum[2] += _data[2]

        # 平均値をオフセットにする
        self._offset_gyro_x = -1.0 * _sum[0] / _count
        self._offset_gyro_y = -1.0 * _sum[1] / _count
        self._offset_gyro_z = -1.0 * _sum[2] / _count

        # I want to register an offset value in a register. But I do not know the behavior, so I will put it on hold.
        print("Gyro calibration complete")
        print(f'Gyro error X: {self._offset_gyro_x:.2f}  Y: {self._offset_gyro_y:.2f}  Z: {self._offset_gyro_z:.2f}')

#    # 地磁気センサを較正する
#    # def _calib_mag(self, _count=3000):
#
#        def model(param, x):
#            return np.array([((xt[0] - param[0]) / param[3]) ** 2 + ((xt[1] - param[1]) / param[4]) ** 2 + (
#                    (xt[2] - param[2]) / param[5]) ** 2 for xt in x])
#
#        # 誤差関数
#        def residuals(param, x, y):
#            return y - model(param, x)
#
#        print("Mag calibration start")
#        print("Please keep the sensor turning around")
#        log = np.array([])
#
#        # データのサンプルを取る
#        for _ in range(_count):
#            _data = self.get_gyro()
#            np.append(log, [_data])
#
#        x_s = log.T
#        y_s = np.array([1.0] * len(x_s))
#        predicted_offset = np.array([5.0, 55, -65, 40, 40, 40])  # パラメータ初期値
#        res = least_squares(residuals, predicted_offset, args=(x_s, y_s))
#        offset_values = res.x
#
#        # 平均値をオフセットにする
#        self._offset_mag_x = -1.0 * offset_values[0]
#        self._offset_mag_y = -1.0 * offset_values[1]
#        self._offset_mag_z = -1.0 * offset_values[2]
#
#        # I want to register an offset value in a register. But I do not know the behavior, so I will put it on hold.
#        print("Mag calibration complete")
#        print(f'Mag error X: {self._offset_mag_x:.2f}  Y: {self._offset_mag_y:.2f}  Z: {self._offset_mag_z:.2f}')
