import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

def ip(image):
        # Initiate FAST object with default values
        fast = cv.FastFeatureDetector_create()

        # find and draw the keypoints
        kp = fast.detect(img,None)
        img2 = image.copy()
        for marker in kp:
                img2 = cv.drawMarker(img2, tuple(int(i) for i in marker.pt), color=(0, 0, 255))
        # img2 = cv.drawKeypoints(img, kp, None, color=(255,0,0))

        # Print all default params
        print( "Threshold: {}".format(fast.getThreshold()) )
        print( "nonmaxSuppression:{}".format(fast.getNonmaxSuppression()) )
        print( "neighborhood: {}".format(fast.getType()) )
        print( "Total Keypoints with nonmaxSuppression: {}".format(len(kp)) )
        return img2

def ip2(image):
        # Initiate FAST object with default values
        fast = cv.FastFeatureDetector_create()
        fast.setNonmaxSuppression(False)
        kp = fast.detect(img,None)
        img2 = image.copy()
        for marker in kp:
                img2 = cv.drawMarker(img2, tuple(int(i) for i in marker.pt), color=(0, 255, 0))
        # img2 = cv.drawKeypoints(img, kp, None, color=(255,0,0))

        print( "Total Keypoints without nonmaxSuppression: {}".format(len(kp)) )
        return img2


for x in range(0, 595):
        if x >= 0 and x < 10:
                img = cv.imread('input_images/left000' + str(x) + '.jpg',1)
                result = ip(img)
                result1 = ip2(img)
                cv.imwrite('output_images_fast/left000' + str(x) + '.jpg', result)
                cv.imwrite('output_images_fast2/left000' + str(x) + '.jpg', result1)

        elif x > 9 and x < 100:
                img = cv.imread('input_images/left00' + str(x) + '.jpg',1)
                result = ip(img)
                result1 = ip2(img)
                cv.imwrite('output_images_fast/left00' + str(x) + '.jpg', result)
                cv.imwrite('output_images_fast2/left00' + str(x) + '.jpg', result1)

        else:
                img = cv.imread('input_images/left0' + str(x) + '.jpg',1)
                result = ip(img)
                result1 = ip2(img)
                cv.imwrite('output_images_fast/left0' + str(x) + '.jpg', result)
                cv.imwrite('output_images_fast2/left0' + str(x) + '.jpg', result1)

