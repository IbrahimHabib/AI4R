import sys
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
        
        pass
