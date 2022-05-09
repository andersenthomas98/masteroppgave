import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from filterpy.stats import plot_covariance_ellipse
from numpy.linalg import inv

def plot_line(r, phi, start, end, color='red'):
    if phi > np.pi/4:
        x = np.linspace(start[0],end[0], 10)
        y = (r - x*np.cos(phi)) / np.sin(phi)
    else:
        y = np.linspace(start[1], end[1], 10)
        x = (r - y*np.sin(phi)) / np.cos(phi)
    plt.plot(x, y, color = color)


nu = np.array([
        [0],    # x
        [0],    # y
        [0],    # theta
        [10.0], # r1
        [0.0]   # phi1
    ]) 

u = np.array([
        1.0, # v
        0.0 # w
        ])

# Initial state covariance
P = np.eye(5,5) * np.array([[0.01, 0.01, 0.01, 0.1, 0.001]])

# Process noise covariance
Q = np.eye(3,3) * np.array([[0.00001, 0.00001, 0.00001]])

# Measurement noise covariance
R = np.eye(2,2)*np.array([[0.1, 0.0001]])

G = np.block([[np.eye(3,3)], 
              [np.zeros((2,3))]])

def f(nu, u, dt):
    x = nu[0,0] + dt*u[0]*np.cos(nu[2,0])
    y = nu[1,0] + dt*u[0]*np.sin(nu[2,0])
    theta = nu[2,0] + dt*u[1]
    r1 = nu[3,0]
    phi1 = nu[4,0]
    return np.array([
        [x],
        [y],
        [theta],
        [r1],
        [phi1]
    ])

def Fx(nu, u, dt):
    matrix = np.eye(3,3)
    matrix[0,2] = -dt*u[0]*np.sin(nu[2, 0])
    matrix[1,2] = dt*u[0]*np.cos(nu[2, 0])
    return matrix

def F(nu, u, dt):
    matrix = np.block([[Fx(nu, u, dt), np.zeros((3,2))],
                       [np.zeros((2,3)), np.eye(2,2)]])
    return matrix


def h(nu, i):
    r = nu[3+i, 0] - (nu[0,0]*np.cos(nu[3+i+1, 0] + nu[1,0]*np.sin(nu[3+i+1, 0])))
    phi = nu[3+i+1, 0] + nu[2,0]
    return np.array([[r], 
                     [phi]])

def transform_measurement_from_global_to_body(z, nu):
    r = z[0,0] - (nu[0,0]*np.cos(z[1,0]) + nu[1,0]*np.sin(z[1,0]))
    phi = z[1,0] + nu[2,0]
    return np.array([[r], 
                     [phi]])

def Hx(nu, i):
    matrix = np.zeros((2,3))
    matrix[0,0] = -np.cos(nu[3 + i, 0])
    matrix[0,1] = -np.sin(nu[3 + i, 0])
    matrix[1,2] = -1
    return matrix

def Hl(nu, i):
    matrix = np.eye(2,2)
    matrix[0,1] = nu[0,0]*np.sin(nu[3+i, 0]) - nu[1,0]*np.sin(nu[3+i, 0]) 
    return matrix

def H(nu, num_landmarks=1):
    matrix = np.block([[Hx(nu, 0), Hl(nu, 0)]])
    return matrix

def residual(a, b):
    """ compute residual (a-b) between measurements containing 
    [range, bearing]. Bearing is normalized to [-pi, pi)"""
    y = a - b
    y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
    if y[1] > np.pi:             # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y

dt = 1.0

# Prediction
'''nu_pred = f(nu, u, dt)
P_pred = F(nu, u, dt) @ P @ F(nu, u, dt).T + G @ Q @ G.T

# This measurement and landmark 0 have been identified as the same feature with a data association scheme
z = np.array([[10.1], [0.001]])

# Update
S = H(nu) @ P_pred @ H(nu).T + R
W = P_pred @ H(nu).T @ inv(S)
v = transform_measurement_from_global_to_body(z, nu) - h(nu, 0)
nu = nu + W @ v 
P = (np.eye(5,5) - W @ H(nu)) @ P_pred'''


r1 = nu[3,0]
phi1 = nu[4,0]
start = np.array([-10, 4])
end = np.array([10, -4])
plot_line(10,0, start, end)


track = []
for step in range(10):
    if (step == 5):
        # Update
        # This measurement and landmark 0 have been identified as the same feature with a data association scheme
        z = np.array([[12.1], [0.1]])
        plot_line(z[0,0], z[1,0], start, end, color='yellow')
        # Update
        S = H(nu) @ P @ H(nu).T + R
        W = P @ H(nu).T @ inv(S)
        v = transform_measurement_from_global_to_body(z, nu) - h(nu, 0)
        nu = nu + W @ v 
        P = (np.eye(5,5) - W @ H(nu)) @ P
    else:
        # Predict
        nu = f(nu, u, dt)
        P = F(nu, u, dt) @ P @ F(nu, u, dt).T + G @ Q @ G.T

    plot_covariance_ellipse(
                    (nu[0,0], nu[1,0]), P[0:2, 0:2], 
                     std=3, facecolor='k', alpha=0.3)
    track.append(nu)

track = np.array(track)

plt.plot(track[:, 0], track[:,1], '--', lw=1, color='black')

RAD2DEG = 180/np.pi
offsetAngle = 30
for i in track:
    plt.plot(i[0,0], i[1,0], marker=(3,1, i[2,0]*RAD2DEG+offsetAngle), markersize=15)

plot_line(nu[3,0], nu[4,0], start, end, color='green')
plt.axis('equal')
plt.show()
