import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


class Bicycle():
    def __init__(self):
        self.xc = 0  # X coordinate of the CG
        self.yc = 0  # Y coordinate of the CG
        self.theta = 0  # Heading of the car with respect to the inertia frame
        self.delta = 0  # Slip angle
        self.beta = 0  # Steering angle

        self.L = 2  # Distance between the centers of the front and rear wheel
        self.lr = 1.2  # Distance between the CG and the Rear wheel
        self.w_max = 1.22  # Max steering rate

        self.sample_time = 0.01  # Time sample for integration

    # Reset function to reset all values to 0
    def reset(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0


class Bicycle(Bicycle):
    def step(self, v, w):
        # ==================================
        #  Implementation of kinematic model
        # ==================================
        self.xc = v * np.cos(self.beta + self.theta) * self.sample_time + self.xc
        self.yc = v * np.sin(self.theta + self.beta) * self.sample_time + self.yc
        self.theta = ((v * np.cos(self.beta) * np.tan(self.delta)) / self.L) * self.sample_time + self.theta
        self.delta = w * self.sample_time + self.delta
        self.beta = np.arctan((self.lr * np.tan(self.delta)) / self.L)
        pass


# TEST
# MOVING IN A CIRCLE OF RADIUS 10M IN 20SEC
# tan(delta) = L/R; delta = 0.1974
# V = d/t; v = 2*pi*10/20 = pi
sample_time = 0.01
time_end = 20
model = Bicycle()

t_data = np.arange(0, time_end, sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)

for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc
    if model.delta < np.arctan(2 / 10):
        model.step(np.pi, model.w_max)
    else:
        model.step(np.pi, 0)

plt.axis('equal')
plt.plot(x_data, y_data, label='Learner Model')
# plt.plot(x_solution, y_solution,label='Solution Model')
plt.legend()
plt.show()
