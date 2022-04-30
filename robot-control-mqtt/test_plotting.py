import matplotlib.pyplot as plt

fig, ax = plt.subplots(1,1)

ax.plot([189, 164],[0, 193], '-', color = 'red')
ax.plot([168, 169],[200, 233], '-', color = 'red')



ax.set_aspect('equal')

plt.show()