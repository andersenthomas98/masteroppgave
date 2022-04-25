import matplotlib.pyplot as plt
import json
from random import random
import numpy as np
from confidence_ellipse import confidence_ellipse2
import signal


line_file = open('line_log.txt', 'r')

point_file = open('point_log.txt', 'r')

dbscan_file = open('dbscan_log.txt', 'r')

iepf_file = open('iepf_log.txt', 'r')

common_point_file = open('common_point_log.txt', 'r')

mse_file = open("mse_log.txt")

n_std = 3.0

fig, ax = plt.subplots(1,1)

for entry in line_file.readlines():
    log = json.loads(entry)
    curr_color = (random(), random(), random())
    #ax.plot([log['start']['x'], log['end']['x']], [log['start']['y'], log['end']['y']], '-', color = curr_color)
    R = np.array([[log['sigma_r2'], log['sigma_rtheta']],
                  [log['sigma_rtheta'], log['sigma_theta2']]]) * 100
    x = np.array([log['start']['x'], log['end']['x']])
    y = np.array([log['start']['y'], log['end']['y']])
    #x = np.array([0, 0])
    #y = np.array([0, 0])
    #R = np.array([[1, 0],[0, 1]])
    #confidence_ellipse2(x, y, R, nstd=3, ax=ax)


for entry in mse_file.readlines():
    log = json.loads(entry)
    curr_color = (random(), random(), random())
    ax.plot([log['start']['x'], log['end']['x']], [log['start']['y'], log['end']['y']], '-', color = curr_color)
    R = np.array([[log['sigma_r2'], log['sigma_rtheta']],
                  [log['sigma_rtheta'], log['sigma_theta2']]]) * 100
    x = np.array([log['start']['x'], log['end']['x']])
    y = np.array([log['start']['y'], log['end']['y']])


colors = ['b', 'r', 'g', 'y']
for entry in point_file.readlines():
    log = json.loads(entry)
    for i in range(1, 5):
        ax.plot(log['ir' + str(i)]['x'], log['ir' + str(i)]['y'], 'o', markersize=1, color='gray')
        pass

'''for entry in common_point_file.readlines()[0:-1]:
    log = json.loads(entry)
    ax.plot(log['x'], log['y'], 'o', markersize=1, color='gray')'''

cluster_id = -99
curr_color = None
for entry in dbscan_file.readlines():
    log = json.loads(entry)
    id = log['id']
    if cluster_id != id:
        curr_color = (random(), random(), random())
        cluster_id = id

   # ax.plot(log['x'], log['y'], 'o', markersize=5, color=curr_color)


cluster_id = -99
for entry in iepf_file.readlines():
    log = json.loads(entry)
    id = log['id']
    if cluster_id != id:
        curr_color = (random(), random(), random())
        cluster_id = id

    #ax.plot(log['x'], log['y'], 'o', markersize=5, color=curr_color)

ax.plot(0, 0, 'gs')
ax.set_aspect('equal')

plt.show()


line_file.close()
point_file.close()
dbscan_file.close()