import numpy as np
def reject_outliers(in_data, m=2.): # https://stackoverflow.com/a/45399188
    data = np.array(in_data)
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d / (mdev if mdev else 1.)
    return data[s < m]

arr = [2.2,2.1,2.2,2.35,2.3]
print(reject_outliers(arr))
