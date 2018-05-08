# GY-91

---

## Description

加速(3軸)・ジャイロ(3軸)・地磁気(３軸)・温度(1軸)の合計10軸を取得可能，InvenSense社のDMP(Digital Motion Processor)を用いることで安定した角度情報を取得できる．

---

* Chip: MPU-9250 + BMP280
* Power supply: 3-5v (internal low dropout regulator)
* Chip built 16bit AD converter, 16-bit data output
* Gyroscopes range: ± 250 500 1000 2000 ° / s,Acceleration range: ± 2 ± 4 ± 8 ± 16g
* 10 DOF

Base library : [InvenSense](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/)
Shop : [Amazon](https://www.amazon.co.jp/MPU9250-BMP280-Acceleration-Gyroscope-Compass/dp/B01CG26A3A)

---
## 注意
1. arudiunoは小数点第２位以下が切り捨てられるので，1000倍してサンプルプログラムでは１０００で割っています
2. oFのサンプルプログラムのCOMポート選択は，環境によって順番が異なるので手動で調整してください
