import numpy as np
from scipy.optimize import least_squares
import pandas as pd


def model(param, X):
    return np.array([((Xt[0] - param[0]) / param[3]) ** 2 + ((Xt[1] - param[1]) / param[4]) ** 2 + (
            (Xt[2] - param[2]) / param[5]) ** 2 for Xt in X])


# 誤差関数
def residuals(param, x, y):
    return y - model(param, x)


def param_prediction():
    df = pd.read_csv('mpu9250_mag_log.csv', header=None)
    # 観測したx, y, z
    Bx = df[0]
    By = df[1]
    Bz = df[2]
    Xs = np.array((Bx, By, Bz)).T
    ys = np.array([1.0] * len(Xs))

    predicted = np.array([5.0, 55, -65, 38, 38, 40])  # パラメータ初期値
    res = least_squares(residuals, predicted, args=(Xs, ys))
    predicted = res.x  # パラメータ計算結果
    print(predicted.shape)
    res = ','.join(map(str, predicted))
    print(predicted)
    print(res)


if __name__ == '__main__':
    param_prediction()
