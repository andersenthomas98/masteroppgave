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
        [1],    # x
        [2],    # y
        [0]    # theta
    ]) 

u = np.array([
        1.0, # v
        0.1 # w
        ])


# Initial state covariance
P_ = np.eye(3,3) * np.array([[0, 0, 0]]) # x, y, theta

# Process noise covariance
Q = np.eye(3,3) * np.array([[0.00001, 0.00001, 0.00001]]) # x, y, theta


def R_update_with_new_landmark(R, new_cov):
    num_landmarks = int(np.shape(R)[0] / 2)
    return np.block([[R, np.zeros((2*num_landmarks,2))],
                     [np.zeros((2, num_landmarks*2)), new_cov]])

def P(P_, n):
    return np.block([[P_, np.zeros((3, 2*n))],
                     [np.zeros((2*n, 3 + 2*n))]])

def G(n):
    return np.block([[np.eye(3,3)],
                     [np.zeros((n*2,3))]])

def f(nu, u, dt):
    x = nu[0,0] + dt*u[0]*np.cos(nu[2,0])
    y = nu[1,0] + dt*u[0]*np.sin(nu[2,0])
    theta = nu[2,0] + dt*u[1]
    #r1 = nu[3,0]
    #phi1 = nu[4,0]
    state =  np.array([
        [x],
        [y],
        [theta],
    ])
    if (np.shape(nu)[0] > 3):
        return np.block([[state], 
                         [nu[3:,:]]])
    else:
        return state

def Fx(nu, u, dt):
    matrix = np.eye(3,3)
    matrix[0,2] = -dt*u[0]*np.sin(nu[2, 0])
    matrix[1,2] = dt*u[0]*np.cos(nu[2, 0])
    return matrix


def F(nu, u, dt, n):
    matrix = np.block([[Fx(nu, u, dt), np.zeros((3,n*2))],
                       [np.zeros((n*2,3)), np.eye(n*2,n*2)]])
    return matrix

# Measurement model: Maps state to predicted measurement - robocentric
# This is used since we need to find some relationship between measurements and the robot pose 
def h(nu, i):
    r = nu[3+i*2, 0] - (nu[0,0]*np.cos(nu[3+i*2+1, 0] + nu[1,0]*np.sin(nu[3+i*2+1, 0])))
    phi = nu[3+i*2+1, 0] - nu[2,0]
    return np.array([[r], 
                     [phi]])



def transform_measurement_from_global_to_body(z, nu):
    r = z[0,0] - (nu[0,0]*np.cos(z[1,0]) + nu[1,0]*np.sin(z[1,0]))
    phi = z[1,0] + nu[2,0]
    return np.array([[r], 
                     [phi]])

def Hx(nu, i):
    matrix = np.zeros((2,3))
    landmark_index = 3 + i*2
    matrix[0,0] = -np.cos(nu[landmark_index + 1, 0])
    matrix[0,1] = -np.sin(nu[landmark_index + 1, 0])
    matrix[1,2] = -1
    return matrix

def Hl(nu, i):
    matrix = np.eye(2,2)
    landmark_index = 3 + i*2
    matrix[0,1] = nu[0,0]*np.sin(nu[landmark_index + 1, 0]) - nu[1,0]*np.cos(nu[landmark_index + 1, 0])
    return matrix

def H(nu):
    num_landmarks = int((np.shape(nu)[0]-3) / 2)
    #matrix = np.block([[Hx(nu, 0), Hl(nu, 0)]])
    block_Hx = Hx(nu, 0)
    block_Hl = Hl(nu, 0)
    for i in range(1, num_landmarks):
        _Hx = Hx(nu, i)
        _Hl = Hl(nu, i)
        block_Hx = np.block([[block_Hx], 
                             [_Hx]])
        block_Hl = np.block([[block_Hl, np.zeros((2*i, 2))],
                             [np.zeros((2, 2*i)), _Hl]])
    
    matrix = np.block([[block_Hx, block_Hl]])
    return matrix

def residual(a, b):
    """ compute residual (a-b) between measurements containing 
    [range, bearing]. Bearing is normalized to [-pi, pi)"""
    y = a - b
    y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
    if y[1] > np.pi:             # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y

def add_new_landmark_to_state(nu, l):
    return np.block([[nu], 
                     [l]])


def add_new_landmark(nu, R, new_landmark, new_R):
    updated_state = add_new_landmark_to_state(nu, new_landmark)
    updated_R = R_update_with_new_landmark(R, new_R)
    return updated_state, updated_R

dt = 1.0        # time-step
n = 0           # num landmarks

# Measurement noise covariance
R = np.eye(2,2)*np.array([[0, 0]]) # sigma_r^2, sigma_phi^2

# Prediction
nu = f(nu, u, dt)
P_pred = F(nu, u, dt, n) @ P(P_, n) @ F(nu, u, dt, n).T + G(n) @ Q @ G(n).T
#print(P_pred)


z = np.array([[10.1], [99.001]])
# Measurement noise covariance of landmark
R = np.eye(2,2)*np.array([[99, 99]]) # sigma_r^2, sigma_phi^2

nu = add_new_landmark_to_state(nu, z)
n += 1

# Update
S = H(nu) @ P(P_pred, n) @ H(nu).T + R
W = P(P_pred, n) @ H(nu).T @ inv(S)
v = transform_measurement_from_global_to_body(z, nu) - h(nu, 0)
nu = nu + W @ v 
P_upd = (np.eye(5,5) - W @ H(nu)) @ P(P_pred, n)

print("#################")
print(P_upd)
# OBS! I believe there is an error the covarince predictor, should we not be predicting the covariance for all system states (including line features?)

# Prediction
nu = f(nu, u, dt)
P_pred = F(nu, u, dt, n) @ P(P_upd, n) @ F(nu, u, dt, n).T + G(n) @ Q @ G(n).T

#print(nu)
#print(P_pred)

# Here comes another landmark...
'''z2 = np.array([[69], [96]])
R2 = np.ones((2,2))*np.array([[69, 69]]) # sigma_r^2, sigma_phi^2

nu, R = add_new_landmark(nu, R, z, R2)

n += 1
# Update
S = H(nu) @ P(P_pred, n) @ H(nu).T
W = P(P_, n) @ H(nu).T @ inv(S)
v = transform_measurement_from_global_to_body(z, nu) - h(nu, 0)
nu = nu + W @ v 
P_upd = (np.eye(5,5) - W @ H(nu)) @ P(P_pred, n)

print(nu)
print(R)'''

'''
r1 = nu[3,0]
phi1 = nu[4,0]
start = np.array([-10, 4])
end = np.array([10, -4])
plot_line(10,0, start, end)'''

'''
track = []
for step in range(10):
    if (step == 5):
        # Update
        # This measurement and landmark 0 have been identified as the same feature with a data association scheme
        z = np.array([[10.1], [0.01]])
        plot_line(z[0,0], z[1,0], start, end, color='yellow')
        # Update
        S = H(nu) @ P @ H(nu).T + R
        W = P @ H(nu).T @ inv(S)
        v = residual(transform_measurement_from_global_to_body(z, nu), h(nu, 0))
        nu = nu + W @ v 
        P = (np.eye(5,5) - W @ H(nu)) @ P
    else:
        # Predict
        nu = f(nu, u, dt)
        P = F(nu, u, dt, n) @ P @ F(nu, u, dt, n).T + G @ Q @ G.T

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
'''