import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

def ip(image):
        gray = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
        gray = np.float32(gray)




for x in range(0, 595):
	if x >= 0 and x < 10:
        	img = cv.imread('input_images/left000' + str(x) + '.jpg',1)
        	result = ip(img)
        	cv.imwrite('harris/left000' + str(x) + '.jpg', result)
    	elif x > 9 and x < 100:
	     	img = cv.imread('input_images/left00' + str(x) + '.jpg' ,1)
        	result = ip(img)
        	cv.imwrite('harris/left00' + str(x) + '.jpg', result)
    	else:
        	img = cv.imread('input_images/left0' + str(x) + '.jpg' ,1)
        	result = ip(img)
        	cv.imwrite('harris/left0' + str(x) + '.jpg', result)

