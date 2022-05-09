from filterpy.kalman import ExtendedKalmanFilter as EKF
import sympy
from sympy import symbols, Matrix
from sympy.abc import x, y, theta, v, w
import numpy as np
import math
import matplotlib.pyplot as plt

time = symbols('t')
r = symbols('r')
phi = symbols('phi')
d = v*time
fxu = Matrix([[x + d*sympy.cos(theta)],
              [y + d*sympy.sin(theta)],
              [theta + d*w],
              [r],
              [phi]])

F = fxu.jacobian(Matrix([x, y, theta]))
V = fxu.jacobian(Matrix([v, w]))

z = Matrix([[r - (x*sympy.cos(phi) + y*sympy.sin(phi))],
            [phi - theta]])

H_pose = z.jacobian(Matrix([x, y, theta]))
H_landmark = z.jacobian(Matrix([r, phi]))


def Hx(x, line):
    """ takes a state variable and returns the measurement
    that would correspond to that state.
    """
    r = line[0]
    phi = line[1]
    robot_x = x[0,0]
    robot_y = x[1,0]
    robot_theta = x[2,0]
    Hx = np.array([[r - (robot_x*math.cos(phi) + robot_y*math.sin(phi))],
                   [phi - robot_theta]]) 
    return Hx

def residual(a, b):
    """ compute residual (a-b) between measurements containing 
    [range, bearing]. Bearing is normalized to [-pi, pi)"""
    y = a - b
    y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
    if y[1] > np.pi:             # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y

class EKF_SLAM(EKF):
    def __init__(self, dt):
        EKF.__init__(self, 3, 2, 2)
        self.dt = dt
        x, y, theta, v, w, r, phi, time = symbols('x, y, theta, v, w, r, phi, time')
        self.fxu = Matrix([[x + d*sympy.cos(theta)],
              [y + d*sympy.sin(theta)],
              [theta + d*w],
              [r],
              [phi]])
        self.F_j = self.fxu.jacobian(Matrix([x, y, theta]))
        self.V_j = None #TODO
        self.x_x, self.x_y, self.theta, self.v, self.w = x, y, theta, v, w
    
    def move(self, x, u, dt):
        #x_pos = x[0,0]
        #y_pos = x[1,0]
        heading = x[2,0]
        lin_vel = u[0]
        rot_vel = u[1]
        dist = lin_vel*dt
        rot = rot_vel*dt
        dx = np.array([[dist*math.cos(heading)],
                       [dist*math.sin(heading)],
                       [rot]])
        return x + dx

    def predict(self, u):
        self.x = self.move(self.x, u, self.dt)        




if __name__ == "__main__":
    dt = 1.0
    ekf = EKF_SLAM(dt)
    ekf.x = np.array([[0.0], [0.0], [0.0]])
    sim_pos = ekf.x.copy()
    track = []

    for step in range(3):
        u = np.array([0.5, np.pi/6])
        sim_pos = ekf.move(sim_pos, u, dt)
        track.append(sim_pos)

    track = np.array(track)
    
    plt.plot(track[:,0], track[:,1], color='k', lw=2)
    plt.axis('equal')
    plt.show()


