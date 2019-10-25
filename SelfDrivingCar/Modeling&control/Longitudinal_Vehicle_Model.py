import sys
from typing import Any, Union

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

"""
Model equation
--------------
𝐽𝑒𝜔˙𝑒=𝑇𝑒−(𝐺𝑅)(𝑟𝑒𝑓𝑓𝐹𝑙𝑜𝑎𝑑)
𝑚𝑥¨=𝐹𝑥−𝐹𝑙𝑜𝑎𝑑
𝑇𝑒=𝑥𝜃(𝑎0+𝑎1𝜔𝑒+𝑎2𝜔2𝑒)
𝐹𝑙𝑜𝑎𝑑=𝐹𝑎𝑒𝑟𝑜+𝑅𝑥+𝐹𝑔
𝐹𝑎𝑒𝑟𝑜=12𝐶𝑎𝜌𝐴𝑥˙2
𝑅𝑥=𝑐𝑎𝑥˙2=𝑁(𝑐̂ 𝑟,0+𝑐̂ 𝑟,1|𝑥˙|+𝑐̂ 𝑟,2𝑥˙2)≈𝑐𝑟,1𝑥˙
𝐹𝑔=𝑚𝑔sin𝛼
𝜔𝑤=(𝐺𝑅)𝜔𝑒
𝑠=𝜔𝑤𝑟𝑒−𝑥˙/𝑥˙
𝐹𝑥={𝑐𝑠,   |𝑠|<1
    𝐹𝑚𝑎𝑥,otherwise}

"""


class Vehicle():
    def __init__(self):
        # ==================================
        #  Parameters
        # ==================================

        # Throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002

        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81

        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01

        # Tire force
        self.c = 10000
        self.F_max = 10000

        # State variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0

        self.sample_time = 0.01

    def reset(self):
        # reset state variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0


class Vehicle(Vehicle):
    def step(self, throttle, alpha):
        # ==================================
        #  Implementation of vehicle model
        # ==================================
        # Aerodynamic force
        F_aero = self.c_a * self.v * self.v
        # Resistance force
        R_x = self.c_r1 * self.v
        # Gravitational force
        F_g = self.m * self.g * np.sin(alpha)
        # Load force on the vehicle
        F_load = F_aero + R_x + F_g
        # Torque generated from the vehicle engine
        T_e = throttle * (self.a_0 + self.a_1 * self.w_e + self.a_2 * self.w_e * self.w_e)
        # Wheels speed
        W_w = self.GR * self.w_e
        r_eff = self.v / W_w
        # Slip ratio
        s = (W_w * self.r_e - self.v) / self.v
        cs = self.c * s

        if abs(s) < 1:
            F_x = cs
        else:
            F_x = self.F_max
        # Motion equations
        self.x = self.x + self.v * self.sample_time
        self.v = self.v + self.a * self.sample_time
        self.a = (F_x - F_load) / self.m
        self.w_e = self.w_e + self.w_e_dot * self.sample_time
        self.w_e_dot = (T_e - self.GR * self.r_e * F_load) / self.J_e

        pass


sample_time = 0.01
time_end = 100
model = Vehicle()

t_data = np.arange(0, time_end, sample_time)
v_data = np.zeros_like(t_data)
x_data = np.zeros_like(t_data)

# throttle percentage between 0 and 1
throttle = 0.2

# incline angle (in radians)
alpha = 0

for i in range(t_data.shape[0]):
    x_data[i] = model.x
    v_data[i] = model.v
    model.step(throttle, alpha)

#plt.plot(t_data, v_data)
# plt.plot(x_data)
#plt.show()
time_end = 20
t_data = np.arange(0, time_end, sample_time)
x_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)
throttle_data = np.zeros_like(t_data)
alpha_data = np.zeros_like(t_data)

# reset the states
model.reset()

for i in range(t_data.shape[0]):
    if 0 <= i < 500:
        throttle = 0.2 + 0.0006 * i
    elif 500 <= i < 1500:
        throttle = 0.5
    elif 1500 <= i < 2000:
        throttle = 2 - 0.001 * i

    if model.x < 60:
        alpha = 0.05
    elif 60 <= model.x <= 150:
        alpha = 0.1
    elif model.x > 150:
        alpha = 0

    alpha_data[i] = alpha
    throttle_data[i] = throttle
    x_data[i] = model.x
    v_data[i] = model.v
    model.step(throttle, alpha)


# Plot x vs t for visualization
# plt.plot(t_data, x_data,label = 'x')
plt.plot(v_data, label='v')
# plt.plot(throttle_data,label = 'throttle')
# plt.plot(alpha_data,label = 'alpha')
plt.legend()
plt.show()
