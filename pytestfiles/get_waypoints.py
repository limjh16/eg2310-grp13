import cv2 as cv
import numpy as np

def get_waypoints(path_array): # path_array in rviz coord
    """Generate waypoints (vertices) from shortest path

    Args:
        path_array (list): List of each individual point (in (x,y) rviz coordinates) of the shortest path

    Returns:
        waypoints_array (list): List of waypoints (in rviz coordinates format)
    """
    waypoints_array = []
    prev_diff = (path_array[1][0] - path_array[0][0], path_array[1][1] - path_array[0][1])
    for i in range(1,len(path_array)):
        current_diff = (path_array[i][0] - path_array[i-1][0], path_array[i][1] - path_array[i-1][1])
        if prev_diff != current_diff:
            prev_diff = current_diff
            waypoints_array.append(path_array[i-1])
    waypoints_array.append(path_array[-1])
    return waypoints_array

# (14,13) is the start
odata = np.loadtxt('/home/jh/ogm.txt', dtype=np.uint8)
dst = odata.copy()
dst[dst<4]=0
dst[dst==4]=1
path_array = [(9, 13), (10, 14), (11, 15), (12, 16), (13, 17), (14, 18), (14, 19), (14, 20), (14, 21), (14, 22), (14, 23), (14, 24), (14, 25), (14, 26), (14, 27), (14, 28), (14, 29), (14, 30), (14, 31), (14, 32), (14, 33), (14, 34), (14, 35), (14, 36), (14, 37), (14, 38), (14, 39), (14, 40), (14, 41), (14, 42), (14, 43), (14, 44), (15, 45), (16, 46), (17, 47), (18, 47), (19, 48), (20, 49), (21, 50), (22, 51), (23, 52), (24, 53), (25, 54), (25, 55)]
print(get_waypoints(path_array))

'''
cv.imshow("im",odata*60)

try:
    cv.waitKey(0)
except KeyboardInterrupt:
    pass
cv.destroyAllWindows()
'''
