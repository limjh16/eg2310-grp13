from array import array
import numpy as np
import cv2 as cv

arr = np.array((
    [0, 0, 0, 0, 0,-1], 
    [0, 0, 0, 0, 0,0], 
    [0, 0, 50, -1,0, 0], 
    [0, 0, 50, 0,0, 0],
    [0, 0, 0, 0, 0,0]
    ))
def inflate(src: np.ndarray, dilate = 2, threshold = 50, inflation_radius = 2, inflation_step = 1):
    array_dilated: np.ndarray = src.copy()
    array_dilated[src >= threshold] = inflation_step
    array_dilated[src < threshold] = 0
    array_dilated = cv.dilate(
        np.uint8(array_dilated),
        cv.getStructuringElement(cv.MORPH_RECT, (2 * dilate + 1, 2 * dilate + 1)),
    )

    wall_indexes = np.nonzero(array_dilated)

    array_inflated: np.ndarray = array_dilated.copy()

    for _ in range(inflation_radius):
        array_inflated = array_inflated + cv.dilate(np.uint8(array_dilated), cv.getStructuringElement(cv.MORPH_RECT, (3,3)))
        array_dilated = array_inflated.copy()
        array_dilated[array_dilated != 0] = inflation_step

    array_inflated = np.int8(array_inflated)
    array_inflated[src == -1] = -1
    array_inflated[wall_indexes] = 100
    return array_inflated


print(inflate(arr,dilate=1))
