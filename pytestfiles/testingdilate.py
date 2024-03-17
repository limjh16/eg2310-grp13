import numpy as np
import time
import cv2 as cv

arr = np.array(
    [
        [-1, -1, -1, -1, -1],
        [0, 0, 0, 0, 0],
        [0, -1, 1, -1, 0],
        [-1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1],
    ]
)

arra = np.zeros((5, 5))
arra[2, 2] = 3
arra[2, 1] = 2
arra[0, 0] = 2
arra[4, 4] = 1
# print(arra)


def dilate123(src, size):
    array_edited = np.copy(src)
    array_edited[array_edited <= 2] = 0
    array_dilated = cv.dilate(
        array_edited,
        cv.getStructuringElement(cv.MORPH_CROSS, (2 * size + 1, 2 * size + 1)),
    )
    return np.maximum(src, array_dilated)


time_start = time.perf_counter()
result = dilate123(arra, 1)
time_end = time.perf_counter()
print("cv3: " + str(time_end - time_start))

# after testing, converting all values to 1 and 0 is
# marginally faster than leaving the values as 3 and 0.
time_start = time.perf_counter()
array_edited = np.copy(arra)
array_edited[array_edited <= 2] = 0
array_edited //= 3
array_dilated = cv.dilate(
    array_edited,
    cv.getStructuringElement(cv.MORPH_CROSS, (2 * 1 + 1, 2 * 1 + 1)),
)
array_dilated *= 3
result = np.maximum(arra, array_dilated)
time_end = time.perf_counter()
print(result)
print("cv1: " + str(time_end - time_start))
