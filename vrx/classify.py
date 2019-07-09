import cv2 as cv
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn import metrics
from matplotlib import pyplot as plt
import time
import sys
import multiprocessing
import os
import rospy
from sensor_msgs.msg import Image
import cv_bridge
from stereo_msgs.msg import DisparityImage
import message_filters

# Once a color image has been read, it passes through the 
# transform function. The transform function has two parts:
# pre-processing also known as pp which filters the image based
# of certain values in the hue, saturation, and value (hsv) 
# colorspace and the dbscan which clusters the processed image
# and classifies the clusters as either a buoy, a green buoy,
# a red buoy, or noise. No changes are needed.

def transform(img):
    image = pp(img)
    img2 = dbscan(image, img)
    return img2

# The first function in transform is pp or preprocessing. The 
# first line truncates the lower half of the image. The reason
# this is because the plantoons from the wamv are always 
# blocking the camera vision and become noisy data. The
# function changes the image to the HSV colorspace and 
# filters out the image. No changes are needed. It returns 
# the filtered image in the BGR colorspace.The input is the
# color image.

def pp(image):
    image = np.array(image[0:800][0:515])
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    image[np.logical_not(np.logical_or(np.logical_and(image[:,:,0] > 70, image[:,:,0] < 90),np.logical_or(image[:,:,0] >= 170, image[:,:,0] < 10)))] = [0,0,0]
    image = cv.cvtColor(image,cv.COLOR_HSV2BGR)
    return(image)

# The second function in transform is dbscan. The inputs are 
# the filtered colored image and the original color image
# read from the imread statement. The output is the original
# image.This is the end of this classification algorithm.

###########################################################

def dbscan(image, original): 

# Part 1: Thresholding the image
##########################################################
# The reason we threshold the image before applying DBSCAN to 
# the points is because DBSCAN is a computational demanding 
# function. This part takes the filtered image, makes a 
# copy of it, and converts it into a grayscale image. The gray-
# scale image is thresholded at a value of 10. 0 is black and
# 255 is white. This takes the ~400,000 matrix points and 
# leaves only ~10,000 nonzero matrix points. 

    img = image.copy()
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret,thresh1 = cv.threshold(img,10,255,cv.THRESH_BINARY)

# After thresholding the image, we pull out those nonzero
# points and assign it to the nonzero array variable. This
# array has dimensions of n rows and 2 columns. The first 
# column is the y coordinate and the second column is the x
# coordinate. These two arrays would be combined by 
# np.column_stack which merges the two arrays at their columns.

    nonzero = np.nonzero(thresh1)

    yp = np.array(nonzero[0])
    xp = np.array(nonzero[1])


    X=np.column_stack((xp,yp))
#########################################################
  
# Creating the BDSCAN object and Sorting Array with Labels
#########################################################    
# After thresholding the filtered image and creating an
# array of nonzero coordinates, this algorithm applies a
# clustering algorithm that filters the image based off the
# array X. If X is empty, this function ends and output is 
# the original input. The first line creates the DBSCAN
# object with an epsilon of 3 and minimum samples of 20. 
# Eps refers to the size of the neighborhood and the 
# min_samples refers to the minimum number of samples required
# for that neighborhood to be considered a cluster. The user
# is free to change the two values for eps and min_samples. The
# .fit(X) applies the matrix points to the DBSCAN object.
# One of the parameters of the db scan object are the labels.
# This is invoked by the line labels = db.labels. This creates
# an array of labels that assigns a number to the image index.
# n clusters have labels that start at 0 and end with n-1.For 
# example, if an image has four clusters, the labels would
# include 0,1,2, and 3. -1 is dedicated for noise. 

    if (len(X) > 0):
        db = DBSCAN(eps=3, min_samples=20).fit(X)
        labels = db.labels_

# The array variable combines the x and y coordinates and the 
# labels in a single array aligned by column. The array is
# sorted with the sort variable with the values in the third
# column or the labels. For example, an image with four
# clusters and noise would be labeled in order from (-1,0,1,2,3).
# The while loop iterates from the beginning of the sorted loop
# and deletes every instance of -1 or until the list is empty.
# This while loop is needed because the noise data is
# irrelevant to the cluster algorithm and the user does not 
# know how much noise there is in the image. The image could 
# have no noise or it could all be noise. The output of this
# section is an array with coordinates sorted together in 
# clusters.

        array = zip(xp,yp,labels)
        sort = sorted(list(array), key=lambda x: x[2])
        
        x = 0
        while (len(sort) > 0 and sort[x][2] == -1):
            del sort[x]
#########################################################

# Iterating through and classifying each cluster
#########################################################
# After iterating through sort, the program is either left with
# an empty array or an array of coordinates sorted and grouped
# by each cluster. The user needs the number of clusters. The 
# next three lines in the big if block does this. unique_labels
# sorts the set of labels and removes -1 if present. The next 
# two lines creates a copy of the sort array and removes the 
# labels column from the array.

        if (len(sort) > 0):
            unique_labels = sorted(set(labels))
            if (unique_labels[0] == -1):
                unique_labels.remove(-1)
            points = sort
            points = np.delete(points,2,1)

# The program assigns three variables. The first one i iterates
# through sort and compare its label value to x for each 
# iteration until the very end of the sort array. The second
# x refers to the cluster that the program is currently class-
# ifying. It starts with 0 and continues classifying until it 
# has classified all clusters. The third variable is l and this
# array becomes all the points within a cluster. Every time the 
# variable i iterates through sort and is equal to the value of
# the current cluster x, the coordinates are appended to the 
# array l. Since this array is sorted, the first time the x 
# variable equals a different value- the cluster has been 
# accounted for. This array of l contains all the points within
# the cluster. This array and the filtered image gets passed
# through the color function. The result of the color function
# gets passed through the rectangle function with the array,
# filtered image, and original image. The x variable is 
# incremented for the next cluster and the l array is set equal 
# to zero. Once the while loop has iterated through all 
# clusters, the result of the rectangle function is set to the 
# original image and the original image is returned to the 
# transform function which is returned to the read function 
# which writes the image.
 
            i = 0
            x = 0
            l = []

            while (x < len(unique_labels)):
                if (i < len(sort) and sort[i][2] == x):
                    l.append(points[i])
                    i+=1
                else:
                    x+=1
                    clr = color(np.array(l),image)
                    tb = rectangle(np.array(l),clr,image,original)
                    l = []
            original = tb   
    return original

############################################################

# For the color function, the points of the cluster and the 
# filtered image are passed through. In this function, the 
# average red, green, and blue pixel values are calculated
# from the image inside of the cluster. If the average value
# fits within a certain range, the string 'red' or 'green' is 
# returned. Otherwise, the function returns the string 'none'
# The user is free to change the range of the BGR values that
# determine whether the buoy is red or green.

def color(mat,image):
    sumred = 0
    sumgrn = 0
    sumblu = 0
    for x in range(len(mat)):
        sumred += image [mat[x][1]] [mat[x][0]] [2]
        sumgrn += image [mat[x][1]] [mat[x][0]] [1]
        sumblu += image [mat[x][1]] [mat[x][0]] [0]
    if ((sumred/len(mat)) > 80 and (sumred/len(mat)) < 180 and (sumgrn/len(mat)) > 30 and (sumgrn/len(mat)) < 115  and (sumblu/len(mat)) > 30 and (sumblu/len(mat)) < 110):
        return 'red'
    if ((sumred/len(mat)) > 30 and (sumred/len(mat)) < 80 and (sumgrn/len(mat)) > 60 and (sumgrn/len(mat)) < 180  and (sumblu/len(mat)) > 50 and (sumblu/len(mat)) < 130):
        return 'green'
    else:
        return 'none'
    
# The rectangle function's parameters are the array of coordinates
# of the cluster, the color of the cluster, the filtered image,
# and the original image. The boundingRect function applies
# a non-rotated rectangle to the image points and returns four
# values in an array, the x and y coordinates of the leftmost
# corner and the width and height. These values are converted
# to ints. The font is set to Hershey Simple for the text and
# the tag is set to zero. The tag is useful in the classifica-
# tion of the buoy. The first classification of the buoy is 
# the proportion of the width and height. If the height is
# between 1.5 times the width and 3.5 times the width, the
# cluster is considered a buoy. If the cluster is given a 
# 'red' or 'green' color, it would be classified as a 'red buoy'
# or 'green buoy' respectively. Otherwise, it would simply be
# classified as a buoy. If any of these three options occur, 
# the tag is set to 1. If the height is not in that range
# mentioned above and the color is 'none', the cluster is
# classified as noise. The last function rectangle makes a 
# rectangle over the original image. The original image is sent
# back to the DBSCAN function.

def rectangle(mat,color,image,original):
    rect = cv.boundingRect(mat)
    height = rect[3]
    width = rect[2]
    x = rect[0]
    x = np.int0(x)
    y = rect[1] 
    y = np.int0(y)
    font = cv.FONT_HERSHEY_SIMPLEX
    tag = 0
    if (1.5*width < height and 3.5*width > height):
        if(color =='green'):
            tag = 1
            cv.putText(original,color+' buoy',(np.int0(x-width/2),np.int0(y-height/2.5)), font, 0.5,(255,255,255),2,cv.LINE_AA)
        elif(color =='red'):
            tag = 1
            cv.putText(original,color+' buoy',(np.int0(x-width/2),np.int0(y-height/2.5)), font, 0.5,(255,255,255),2,cv.LINE_AA)
        else:
            tag = 1
            cv.putText(original,'buoy',(np.int0(x-width/2),np.int0(y-height/2.5)), font, 0.5,(255,255,255),2,cv.LINE_AA)     
    if(color == 'none' and tag == 0):
        cv.putText(original,'noise',(np.int0(x-width/2),np.int0(y-height/2.5)), font, 0.5,(255,255,255),2,cv.LINE_AA)
    cv.rectangle(original,(x,y),(x+width,y+height),(255,0,255),2)
    return original


def callback(disp,rect):
   #print("inside")
    bridge = cv_bridge.CvBridge()
   #print("bridge")
    frame = rect.header.seq
    
    image = bridge.imgmsg_to_cv2(rect, desired_encoding="passthrough")
    image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
    image = transform(image)
   #print("imagetransform")
    cv.imshow("left/image_raw", image)
    cv.waitKey(1)

    disp = bridge.imgmsg_to_cv2(disp.image, desired_encoding="passthrough")
    cv.imshow("disparity image",disp)
    cv.waitKey(1)

#new main
if __name__ == '__main__':
    rospy.init_node('liveproc')
   #print("init")
    disp = message_filters.Subscriber("stereo/disparity", DisparityImage)
    rect = message_filters.Subscriber("stereo/left/image_rect_color",Image)
   #print("image")
    ts = message_filters.TimeSynchronizer([disp,rect],10)
    ts.registerCallback(callback)
   #print("callback")
    rospy.spin()
