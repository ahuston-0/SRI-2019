import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

image = cv.imread("/home/asif/env/cv/wamv/input_images/left0001.jpg",1)

print(image.shape)
cv.namedWindow("buoy", cv.WINDOW_NORMAL)

cv.imshow("buoy",image)



cv.waitKey(0)
cv.destroyWindow("buoy")

image2 = image.copy()
image2[:,:,0] = image[:,:,2]
image2[:,:,2] = image[:,:,0]

plt.imshow(image2, interpolation = 'bicubic')

plt.xticks([]),plt.yticks([])

plt.show()
