import numpy as np
import json
import matplotlib.pyplot as plt

def plot_line(r, phi, start, end, color='red'):
    if (phi > np.pi/4 and phi < (np.pi/4 + np.pi/2)) or (phi > (np.pi + np.pi / 4) and phi < (np.pi + np.pi/2 + np.pi/4)) :
        x = np.linspace(start[0],end[0], 10)
        y = (r - x*np.cos(phi)) / np.sin(phi)
    else:
        y = np.linspace(start[1], end[1], 10)
        x = (r - y*np.sin(phi)) / np.cos(phi)
    plt.plot(x, y, color = color)

def wrap_to_2pi(phi):
    if (phi >= 2*np.pi):
        return phi - 2*np.pi
    elif phi < 0:
        return phi + 2*np.pi    
    return phi

def get_normal_form_parameters(endpoints):
    Px = endpoints[0,0]
    Py = endpoints[0,1]
    Qx = endpoints[1,0]
    Qy = endpoints[1,1]
    m = (Qy - Py) / (Qx - Px)
    phi = np.arctan2((Qy - Py), (Qx - Px))
    r = (Py - m*Px) / np.sqrt(m*m + 1)
    if (r < 0):
        r = np.abs(r)
        phi -= np.pi / 2
    phi = wrap_to_2pi(phi)
    print(r, phi)
    return r, phi

def load_from_json_log(file):
    f = open(file, 'r')
    data = []
    for e in f.readlines():
        data.append(json.loads(e))
    return data




estimator_data = load_from_json_log('estimator_log_third_run.txt')
line_data = load_from_json_log('line_log_third_run.txt')
normal_data = load_from_json_log('normal_log_third_run.txt')

#for i, d in enumerate(normal_data):
#    plot_line(d['r'], d['phi'], np.array([line_data[i]['start']['x'], line_data[i]['start']['y']]), np.array([line_data[i]['end']['x'], line_data[i]['end']['y']]))
#    plt.plot([line_data[i]['start']['x'], line_data[i]['end']['x']], [line_data[i]['start']['y'], line_data[i]['end']['y']], '-', color = 'gray')
    

def get_length(endpoints):
    x1 = endpoints[0][0]
    y1 = endpoints[0][1]
    x2 = endpoints[1][0]
    y2 = endpoints[1][1]
    return np.sqrt(np.power(x1-x2, 2) + np.power(y1-y1, 2))

def max_distance_endpoints(endpoints1, endpoints2):
    points = [endpoints1[0], endpoints1[1], endpoints2[0], endpoints2[1]]
    max_dist = 0
    for i in range(4):
        for j in range(4):
            if get_length([points[i], points[j]]) > max_dist:
                max_dist = get_length([points[i], points[j]])
                curr_endpoints = np.array([points[i], points[j]])

    return curr_endpoints

def mod(x,y):
    return x - np.floor(x / y)*y

def ssa(rad):
    # rad - floorf((rad + M_PI) / (2*M_PI))*(2*M_PI);
    return mod(rad+np.pi, 2*np.pi) - np.pi

def ssa2(rad):
    # mod( angle + pi, 2 * pi ) - pi;
    return (rad + np.pi) % (2*np.pi) - np.pi

def ssa3(rad):
    return (rad+np.pi) - np.floor((rad+np.pi) / (2*np.pi))*(2*np.pi) - np.pi




phi1 = 0
phi2 = np.pi / 4

print(np.abs(ssa(phi1-phi2)))
print(np.abs(ssa3(phi1-phi2)))