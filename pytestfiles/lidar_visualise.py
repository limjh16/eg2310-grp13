import matplotlib.pyplot as plt
import numpy as np

# Compute areas and colors
# N = 150
# r = 2 * np.random.rand(N)
# theta = 2 * np.pi * np.random.rand(N)
# area = 200 * r**2
# colors = theta

r = np.loadtxt("lidar_bucket.txt")
theta=np.deg2rad(np.arange(0,360))

fig = plt.figure()
ax = fig.add_subplot(projection='polar')
c = ax.scatter(theta, r, s=1)
plt.pause(1000)
