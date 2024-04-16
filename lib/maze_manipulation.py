import cv2 as cv
import numpy as np
def get_waypoints(path_array:list): # path_array in rviz coord
    """Generate waypoints (vertices) from shortest path

    Args:
        path_array (list): List of each individual point (in (x,y) rviz coordinates) of the shortest path

    Returns:
        waypoints_array (list): List of waypoints (in rviz coordinates format)
    """
    waypoints_array = []
    prev_diff = (round((path_array[1][0] - path_array[0][0]),2), round((path_array[1][1] - path_array[0][1]),2))
    for i in range(1,len(path_array)):
        current_diff = (round((path_array[i][0] - path_array[i-1][0]),2), round((path_array[i][1] - path_array[i-1][1]),2))
        if prev_diff != current_diff:
            prev_diff = current_diff
            # print(prev_diff)
            # print(current_diff)
            waypoints_array.append(path_array[i-1])
    waypoints_array.append(path_array[-1])
    return waypoints_array

def dilate123(src, size=1, shape=cv.MORPH_RECT):
    """Dilation for odata array (after binning and reshape)

    Args:
        src (ndarray): input odata array
        size (integer): number of iterations of dilation,
                            this size / resolution is the meters that the image is dilated.
        shape (integer): cv2 MorphShapes

    Returns:
        ndarray: output of dilated array
    """
    array_edited = np.copy(src)
    array_edited[array_edited <= 2] = 0
    array_edited //= 3
    array_dilated = cv.dilate(
        array_edited,
        cv.getStructuringElement(shape, (2 * size + 1, 2 * size + 1)),
    )
    array_dilated *= 3
    return np.maximum(src, array_dilated)

def inflate(src: np.ndarray, dilate = 2, threshold = 40, inflation_radius = 4, inflation_step = 2, erode = 2):
    array_eroded: np.ndarray = src.copy()
    array_eroded[src == -1] = 1
    array_eroded[src != -1] = 0
    array_eroded = cv.morphologyEx(
        np.uint8(array_eroded),
        cv.MORPH_CLOSE,
        cv.getStructuringElement(cv.MORPH_RECT, (2 * erode + 1, 2 * erode + 1))
    ) # Dilate then erode the unknown points, to get rid of stray wall and stray unoccupied points

    array_dilated: np.ndarray = src.copy()
    array_dilated[src >= threshold] = inflation_step
    array_dilated[src < threshold] = 0
    array_dilated = cv.dilate(
        np.uint8(array_dilated),
        cv.getStructuringElement(cv.MORPH_RECT, (2 * dilate + 1, 2 * dilate + 1))
    )

    wall_indexes = np.nonzero(array_dilated)

    array_inflated: np.ndarray = array_dilated.copy()

    for _ in range(inflation_radius):
        array_inflated = array_inflated + cv.dilate(np.uint8(array_dilated), cv.getStructuringElement(cv.MORPH_RECT, (3,3)))
        array_dilated = array_inflated.copy()
        array_dilated[array_dilated != 0] = inflation_step

    array_inflated = np.int8(array_inflated)
    array_inflated[array_eroded == 1] = -1
    array_inflated[wall_indexes] = 100
    return array_inflated
