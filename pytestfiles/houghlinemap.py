import cv2 as cv
import numpy as np


odata = np.loadtxt('/home/jh/ogm.txt', dtype=np.uint8)
dst = odata.copy()
dst[dst<4]=0
dst[dst==4]=255
out = np.zeros(odata.shape)

lsd = cv.createLineSegmentDetector(0)
lines = lsd.detect(image=dst)[0]

for dline in lines:
    print(dline)
    x0 = int(round(dline[0][0]))
    y0 = int(round(dline[0][1]))
    x1 = int(round(dline[0][2]))
    y1 = int(round(dline[0][3]))
    cv.line(out, (x0, y0), (x1,y1), 255, 1, cv.LINE_AA)

cv.imshow("im",odata*60)
cv.imshow("ds",out)
try:
    cv.waitKey(0)
except KeyboardInterrupt:
    pass
cv.destroyAllWindows() 
