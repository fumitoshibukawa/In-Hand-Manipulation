import numpy as np
import math

#初期位置と目標位置
fp = [0, -20]
tp = [0, 10]

#対象球体の半径R，能動球体の半径r
R = 50
r = 11
#カメラ画像内の中心から対象点までの距離L1，L2
L1 = math.sqrt(fp[0]**2 + fp[1]**2)
L2 = math.sqrt(tp[0]**2 + tp[1]**2)

#カメラの2次元(x,y)座標から，3次元(x,y,z)座標へ変換，ベクトル化
cfp = np.array([fp[0], fp[1], math.sqrt(R**2-L1**2)])
ctp = np.array([tp[0], tp[1], math.sqrt(R**2-L2**2)])
print(cfp)
print(ctp)

#外積
cross = np.cross(cfp, ctp)
print(cross)
#外積のノルム
uni_cross = cross/np.linalg.norm(cross)
print(uni_cross)
#内積
dot = np.dot(cfp, ctp)
print(dot)
#2つのベクトルの間の角
theta = math.acos(dot/(np.linalg.norm(cfp)*np.linalg.norm(ctp)))
print(theta)
#点の移動時間
#t = 3

#対象球体の角速度ベクトル
target_omega = uni_cross*(theta)
print(target_omega)

#能動球体W0,W1,W2のx軸からの角度
theta0 = -np.pi / 3
theta1 = np.pi / 3
theta2 = np.pi

theta0_2 = theta2 - theta0 # 4*np.pi / 3
theta1_2 = theta2 - theta1 # 2*np.pi / 3

#行列Mの定義
M = r/R * np.array([
    [np.cos(theta0), np.cos(theta1), 0],
    [np.sin(theta0), np.sin(theta1), 0],
    [0, 0, 1]
])
#逆行列 M_inv
M_inv = np.linalg.inv(M)
print(f"M_inv: {M_inv}\n")

#能動球体の角速度の大きさを求める
active_omega = np.dot(M_inv, target_omega)

# 結果を表示
active_omega_y0, active_omega_y1, active_omega_z0 = active_omega
print(f"Active Omega y0: {active_omega_y0}")
print(f"Active Omega y1: {active_omega_y1}")
print(f"Active Omega z0: {active_omega_z0}\n")

#3つの能動球体の角速度を計算
active_omega_z1 = active_omega_z0
active_omega_z2 = active_omega_z0
active_omega_y2 = np.dot([np.sin(theta0_2), np.sin(theta1_2)], [active_omega_y0, active_omega_y1])

# 結果を表示
print(f"Active Omega z1: {active_omega_z1}")
print(f"Active Omega z2: {active_omega_z2}")
print(f"Active Omega y2: {active_omega_y2}\n")

active_omega_0 = np.array([active_omega_y0, active_omega_z0])
active_omega_1 = np.array([active_omega_y1, active_omega_z1])
active_omega_2 = np.array([active_omega_y2, active_omega_z2])

print(f"Active Omega 0: {active_omega_0}")
print(f"Active Omega 1: {active_omega_1}")
print(f"Active Omega 2: {active_omega_2}")

#ゴムベルトB1,B2のy軸からの角度
active_theta1 = -np.pi / 4
active_theta2 = -3*np.pi / 4
#ローラの半径
r_roll = 5.5

#行列M2の定義
M2 = r_roll/r * np.array([
    [np.cos(active_theta1), np.cos(active_theta2)],
    [np.sin(active_theta1), np.sin(active_theta2)]
])
#逆行列 M2_inv
M2_inv = np.linalg.inv(M2)
print(f"M2_inv: {M2_inv}\n")

#ローラの角速度の大きさを求める
roll_omega_0 = np.dot(M2_inv, active_omega_0)
roll_omega_1 = np.dot(M2_inv, active_omega_1)
roll_omega_2 = np.dot(M2_inv, active_omega_2)

print(f"Roll Omega 0: {roll_omega_0}")
print(f"Roll Omega 1: {roll_omega_1}")
print(f"Roll Omega 2: {roll_omega_2}")

#モータ（プーリ）の半径
r_moter = 7.4
#モータ（プーリ）の角度
moter_theta_0 = r_roll/r_moter * roll_omega_0
moter_theta_1 = r_roll/r_moter * roll_omega_1
moter_theta_2 = r_roll/r_moter * roll_omega_2

print(f"Moter Theta 0: {moter_theta_0}")
print(f"Moter Theta 1: {moter_theta_1}")
print(f"Moter Theta 2: {moter_theta_2}")