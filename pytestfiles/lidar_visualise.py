import matplotlib.pyplot as plt
import numpy as np

# Compute areas and colors
# N = 150
# r = 2 * np.random.rand(N)
# theta = 2 * np.pi * np.random.rand(N)
# area = 200 * r**2
# colors = theta

fig = [0,0,0,0,0,0,0,0,0]
ax = [0,0,0,0,0,0,0,0,0]
for i in range(1,9):
    fig[i] = plt.figure()
    r = np.loadtxt("lidar_doors"+str(i)+".txt")
    print (r.size)
    theta=np.deg2rad(np.arange(0,360,360/r.size))
    ax[i] = fig[i].add_subplot(projection='polar')
    c = ax[i].scatter(theta, r, s=1)

plt.pause(1000)
