# RPi-AHRS-Python
Raspbery Piからソケット通信で送信されたセンサデータからMadgewickフィルタを使用して姿勢推定するclient側プログラムです。
server側([RPI-AHRS-golang](https://github.com/tetsuzawa/RPi-AHRS-golang))で演算を行うことで高速に処理を行うことができます。

Raspberry Pi側の処理はRPi-AHRS-Pythonのclientのプログラムを流用します。
