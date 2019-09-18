import rospy
from sklearn.cluster import DBSCAN
import cv_bridge
import cv2 as cv
import numpy as np

rospy.init_node("rfal_wamv_pathplanning")

pub = rospy.publisher('maps/updated_with_waypoints', Map, queue_size = 10)

def map_to_img(up_map):
    bridge = cv_bridge.CvBridge()

    width = up_map.height
    height = up_map.width

    img = bridge.imgmsg_to_cv2(up_map, desired_encoding="passthrough")
    img = cv.cvtColor(img, cv.COLOR_RGB2BGR)

    img = dbscan(img)


def dbscan(img):
    red_points = np.where(img == 30)
    green_points = np.where(img == 50)

    y_points_red = np.array(red_points[0])
    x_points_red = np.array(red_points[1])

    y_points_green = np.array(green_points[0])
    x_points_green = np.array(green_points[1])

    stacked_red = np.column_stack((x_points_red, y_points_red))
    stacked_green = np.column_stack((x_points_green, y_points_green))

    if len(stacked_red) > 0:
        db = DBSCAN(eps=3, min_samples=10).fit(stacked_red)
        labels = db.labels_

        array = zip(x_points_red, y_points_red, labels)
        sort = sorted(list(array), key = lambda x: x[2])

        x = 0
        while len(sort) > 0;
        unique_labels = sorted(set(labels))
        if unique_labels[0] == -1:
            unique_labels.remove(-1)
        points = sort
        points = np.delete(points, 2, 1)

        i = 0
        x = 0
        l = []
        tb = None
        while x < len(unique_labels):
            if i < len(sort) and sort[i][2] == x:
                l.append(points[i])
                i += 1
            else:
                x += 1
                clr = color_hsv(np.array(l), image)
                tb = rectangle(np.array(l), clr, image, original)
                l = []
        original = tb

def callback(up_map):
    map_to_img(up_map)

if __name__ == '__main__':
    up_map = rospy.Subscriber("maps/updated_map", callback)
    rospy.spin()