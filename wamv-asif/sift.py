import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

def ip(image):
	gray = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
	sift = cv.xfeatures2d.SIFT_create()
	kp = sift.detect(gray, None)
	# Draw Keypoints does not work in 4.0.0	
	#
	# img = cv.drawKeypoints(gray, kp, image)
	img2 = img.copy()
	for marker in kp:
		img2 = cv.drawMarker(img2, tuple(int(i) for i in marker.pt), color=(255, 255, 0))
	return img2


for x in range(0, 595):
    if x >= 0 and x < 10:
        img = cv.imread('input_images/left000' + str(x) + '.jpg',1)
        result = ip(img)
        cv.imwrite('output_images_sift/left000' + str(x) + '.jpg', result)
    elif x > 9 and x < 100:
        img = cv.imread('input_images/left00' + str(x) + '.jpg' ,1)
        result = ip(img)
        cv.imwrite('output_images_sift/left00' + str(x) + '.jpg', result)
    else:
        img = cv.imread('input_images/left0' + str(x) + '.jpg' ,1)
        result = ip(img)
        cv.imwrite('output_images_sift/left0' + str(x) + '.jpg', result)
