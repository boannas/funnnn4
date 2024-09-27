import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

def plot_sphere(ax, radius, center, color, alpha=1.0):
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]

    ax.plot_surface(x, y, z, color=color, alpha=alpha)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

plot_sphere(ax, radius=0.03, center=[0.5, 0.5, 0.2], color='blue', alpha=0.4)
plot_sphere(ax, radius=0.53, center=[0.5, 0.5, 0.2], color='red', alpha=0.2)

ax.plot([0.5, 0.5], [0.5, 0.5], [0.2, 0], color='green', linewidth=2, label="Line to Origin")
ax.text(0.5, 0.5, 0, 'link1', color='green') 

ax.plot([0.5, 0.5], [0.5, 0.5], [0.2, 0.45], color='black', linewidth=2, label="Line to Origin")
ax.text(0.5, 0.5, 0.2, 'link2', color='black') 

ax.plot([0.5, 0.5], [0.5, 0.5], [0.45, 0.73], color='green', linewidth=2, label="Line to Origin")
ax.text(0.5, 0.5, 0.45, 'link3', color='green') 
ax.text(0.5, 0.5, 0.73, 'End effector', color='magenta')  


ax.set_xlim([0, 1])
ax.set_ylim([0, 1])
ax.set_zlim([0, 1])

plt.show()
