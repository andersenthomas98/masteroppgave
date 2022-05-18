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
    #plt.plot(x, y, color = color)

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




estimator_data = load_from_json_log('estimator_log_12.txt')
line_data = load_from_json_log('line_handmade.txt')
normal_data = load_from_json_log('normal_handmade.txt')

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


# [[array([-0.21134221]), array([0.12959791])], [array([2.4811827]), array([0.20778711])]]
# [[1.639, 0.155], [1.328, 0.162]]

def dist_from_point_to_line(point, endpoints):
    Px = endpoints[0][0]
    Py = endpoints[0][1]
    Qx = endpoints[1][0]
    Qy = endpoints[1][1]
    x = point[0]
    y = point[1]
    return np.abs((Qx - Px)*(Py - y) - (Px - x)*(Qy - Py))/(np.sqrt(np.power(Qx - Px, 2) + np.power(Qy - Py, 2)))


def dist_from_point_to_line_segment(point, endpoints):
    Px = endpoints[0][0]
    Py = endpoints[0][1]
    Qx = endpoints[1][0]
    Qy = endpoints[1][1]
    x = point[0]
    y = point[1]
    t = -((Px - x)*(Qx - Px) + (Py - y)*(Qy - Py)) / (np.power(Px - Qx, 2) + np.power(Py - Qy, 2))
    print(t)
    if t >= 0 and t <= 1:
        # Point is perpendicular to the line segment from (Px, Py) to (Qx, Qy)
        print("point is perp")
        return dist_from_point_to_line(point, endpoints)
    d1 = (np.sqrt(np.power(x - Px, 2) + np.power(y - Py, 2)))
    d2 = (np.sqrt(np.power(x - Qx, 2) + np.power(y - Qy, 2)))
    # Shortest distance from point to one of the endpoints
    if (d1 < d2):
        return d1
    return d2

def overlap(endpoints1, endpoints2):
    l1 = get_length(endpoints1)
    l2 = get_length(endpoints2)
    e = max_distance_endpoints(endpoints1, endpoints2)
    len_e = get_length(e)
    L = l1 + l2 - len_e
    if L < 0:
        return 0
    return 1

#P1 = [-0.21134221, 0.12959791]
#Q1 = [2.4811827, 0.20778711]

for i in range(len(line_data)):
    x_s = line_data[i]['start']['x']
    y_s = line_data[i]['start']['y']
    x_e = line_data[i]['end']['x']
    y_e = line_data[i]['end']['y']
    if (line_data[i]['time'] != normal_data[i]['time']):
        print(i, "not aligned")
    if np.sqrt(np.power(x_s-x_e,2) + np.power(y_s - y_e, 2)) > 500 or np.sqrt(np.power(x_s-x_e,2) + np.power(y_s - y_e, 2)) < 100:
        #print(i)
        continue
    plt.plot([x_s, x_e], [y_s, y_e], '-', color='red')
    


plt.axis('equal')
plt.show()

#  [1.5396832] [0.8438354]