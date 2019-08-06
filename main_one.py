# Import Statements
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
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid as Map
import message_filters
import cv_bridge
import tf
import math
from std_msgs.msg import String
import EnvMap
import Buoy_List
from numba import jit


# %%
# Initialization Stuff

rospy.init_node('liveproc')

listener = tf.TransformListener()
pub = rospy.Publisher('updated_map', Map, queue_size=10)

occ_map = None
up_occ_map = None
map_resolution = 0.5
origin = None
robot_x = 0
robot_y = 0
map_width = 0
b_list = []

map_height = 0
camera_range = 85 / map_resolution
overlay = True
display_robot_yaw = False
display_rect = False
display_map = False
display_info = False
tri_width = 5
dist_error = 0.15

np.set_printoptions(threshold=sys.maxsize)


# %%

def draw_background_objects(map_img, map_np):
    start_time = time.time()
    global origin
    global robot_x
    global robot_y

    o_x = (-origin.x) / map_resolution
    o_y = (-origin.y) / map_resolution
    # Color Assignment Section
    map_img[map_np == -1] = 0  # Change to [255,255,255] for color to [0] for grayscale
    map_img[map_np == 0] = 0  # Change to [255,255,255] for color to [0] for grayscale
    map_img[map_np == 100] = 100  # Change to [0,0,0] for color to [100] for grayscale
    origin_color = 100  # Change to [0,0,255] for color to [0] for grayscale
    robot_color = 100  # Change to [0,0,255] for color to [0] for grayscale
    # Origin (Drawn as a cross)

    map_img[int(o_y) + 0][int(o_x) + 0] = origin_color
    map_img[int(o_y) + 0][int(o_x) + 1] = origin_color
    map_img[int(o_y) + 0][int(o_x) + 2] = origin_color
    map_img[int(o_y) + 1][int(o_x) + 0] = origin_color
    map_img[int(o_y) + 2][int(o_x) + 0] = origin_color
    if (-origin.x) != 0:
        map_img[int(o_y) + 0][int(o_x) - 1] = origin_color
        map_img[int(o_y) + 0][int(o_x) - 2] = origin_color
    if (-origin.y) != 0:
        map_img[int(o_y) - 1][int(o_x) + 0] = origin_color
        map_img[int(o_y) - 2][int(o_x) + 0] = origin_color
    # Robot (Drawn as a square)
    map_img[int(robot_y) + 0][int(robot_x) + 0] = robot_color
    map_img[int(robot_y) + 0][int(robot_x) + 2] = robot_color
    map_img[int(robot_y) + 1][int(robot_x) + 2] = robot_color
    map_img[int(robot_y) - 1][int(robot_x) + 2] = robot_color
    map_img[int(robot_y) + 0][int(robot_x) - 2] = robot_color
    map_img[int(robot_y) + 1][int(robot_x) - 2] = robot_color
    map_img[int(robot_y) - 1][int(robot_x) - 2] = robot_color
    map_img[int(robot_y) + 2][int(robot_x) + 0] = robot_color
    map_img[int(robot_y) + 2][int(robot_x) + 1] = robot_color
    map_img[int(robot_y) + 2][int(robot_x) - 1] = robot_color
    map_img[int(robot_y) - 2][int(robot_x) + 0] = robot_color
    map_img[int(robot_y) - 2][int(robot_x) + 1] = robot_color
    map_img[int(robot_y) - 2][int(robot_x) - 1] = robot_color
    #     print time.time()-start_time, "background"
    return map_img


# %%

def draw_ray(map_img, slices):
    start_time = time.time()
    global display_robot_yaw
    global bearings
    global tags
    global areas
    global robot_x
    global robot_y
    global map_resolution
    global areas

    slices_size = len(slices)

    if not display_robot_yaw:
        for i in range(int(slices_size)):
            try:
                arr = slices[i]
                appr_dist_low = (1.0 - 2.5 * dist_error) * map_resolution * 2 * 456.7946397925 * math.pow(areas[i],
                                                                                                          -0.4705300238)
                for ray in arr:
                    if ray[3] > appr_dist_low:
                        if ray[2] == -1 or ray[2] == 0:
                            if False:  # display true if you want to see slice
                                map_img[ray[1]][ray[0]] = [255, 0, 0]  # Change to [255,0,0] for color to [0] for grayscale
                        if ray[2] == 100:
                            if tags[i] == 2:
                                map_img[ray[1]][ray[0]] = 30  # Change to [0,255,0] for color to 0,[180 for grayscale]
                            if tags[i] == 3:
                                map_img[ray[1]][ray[0]] = 50  # Change to [0,0,255] for color to 0,[230] for grayscale
            except Exception as ex:
                print ex
    else:
        for arr in slices[-1]:
            map_img[arr[1]][arr[0]] = [0, 255, 0]
    #     print time.time()-start_time, "draw ray"
    return map_img


# %%

def get_mapnbot_info(trans, robot_yaw):
    start_time = time.time()
    global map_resolution
    global robot_x
    global robot_y
    global map_width
    global map_height
    global origin
    global occ_map
    global map_np

    map_array = occ_map.data

    map_width = occ_map.info.width
    map_height = occ_map.info.height

    map_np = np.reshape(np.array(map_array), (map_height, map_width))

    robot_x = (trans[0] - origin.x) / map_resolution
    robot_y = (trans[1] - origin.y) / map_resolution
    robot_yaw = (90 - robot_yaw)
    while robot_yaw > 360:
        robot_yaw = robot_yaw - 360
    while robot_yaw < 0:
        robot_yaw = robot_yaw + 360
    #     print time.time()-start_time, "map/bot info"
    return map_np, robot_yaw


# %%

def insidetriangle((x1, x2, x3), (y1, y2, y3), map_np):
    start_time = time.time()
    global robot_x
    global robot_y

    xs = np.array((x1, x2, x3), dtype=float)
    ys = np.array((y1, y2, y3), dtype=float)

    # The possible range of coordinates that can be returned
    x_range = np.arange(np.min(xs), np.max(xs) + 1)
    y_range = np.arange(np.min(ys), np.max(ys) + 1)

    # Set the grid of coordinates on which the triangle lies. The centre of the
    # triangle serves as a criterion for what is inside or outside the triangle.
    X, Y = np.meshgrid(x_range, y_range)
    xc = np.mean(xs)
    yc = np.mean(ys)

    # From the array 'triangle', points that lie outside the triangle will be
    # set to 'False'.
    triangle = np.ones(X.shape, dtype=bool)
    for i in range(3):
        ii = (i + 1) % 3
        if xs[i] == xs[ii]:
            include = X * (xc - xs[i]) / abs(xc - xs[i]) > xs[i] * (xc - xs[i]) / abs(xc - xs[i])
        else:
            poly = np.poly1d([(ys[ii] - ys[i]) / (xs[ii] - xs[i]), ys[i] - xs[i] * (ys[ii] - ys[i]) / (xs[ii] - xs[i])])
            include = Y * (yc - poly(xc)) / abs(yc - poly(xc)) > poly(X) * (yc - poly(xc)) / abs(yc - poly(xc))
        triangle *= include

    # Output: 2 arrays with the x- and y- coordinates of the points inside the
    # triangle.

    x_points = X[triangle]
    y_points = Y[triangle]

    points = np.column_stack((x_points.astype(int), y_points.astype(int)))

    itbuffer = np.empty(shape=((points.size / 2), 4), dtype=np.int32)
    itbuffer.fill(np.nan)

    itbuffer[:, 0] = x_points
    itbuffer[:, 1] = y_points
    itbuffer[:, 2] = map_np[points[:, 1], points[:, 0]]

    # Get distance values

    deltax = itbuffer[:, 0] - robot_x
    deltay = itbuffer[:, 1] - robot_y

    deltax = map_resolution * deltax
    deltay = map_resolution * deltay

    deltax = np.square(deltax)
    deltay = np.square(deltay)

    dist_sq = deltax + deltay

    dist_sq = np.sqrt(dist_sq)
    #     dist_sq = map_resolution*dist_sq

    itbuffer[:, 3] = dist_sq
    #     print robot_x,robot_y
    #     print np.column_stack((deltax.astype(int),deltay.astype(int)))
    #     print time.time()-start_time, "inside_triangle"
    return itbuffer


# %%

def get_points(buoy_yaw, index):
    start_time = time.time()
    global robot_x
    global robot_y
    global camera_range
    global tri_width
    global areas
    global dist_error

    appr_dist_up = (1.0 + 2.0 * dist_error) * 2 * 456.7946397925 * math.pow(areas[index], -0.4705300238)
    point_deltax = appr_dist_up * math.sin(math.radians(buoy_yaw))  # appr_dist_up
    point_deltay = appr_dist_up * math.cos(math.radians(buoy_yaw))  # appr_dist_up
    per_slope = -1.0 / (point_deltay / point_deltax)
    point1 = np.array([round(robot_x), round(robot_y)], dtype=int)
    point2 = np.array([round(robot_x + point_deltax), round(robot_y + point_deltay)], dtype=int)

    point3_x = (robot_x + point_deltax) + math.sqrt(math.pow(tri_width, 2) / (math.pow(per_slope, 2) + 1))
    point4_x = (robot_x + point_deltax) - math.sqrt(math.pow(tri_width, 2) / (math.pow(per_slope, 2) + 1))

    point3_y = robot_y + point_deltay + per_slope * (point3_x - (robot_x + point_deltax))
    point4_y = robot_y + point_deltay + per_slope * (point4_x - (robot_x + point_deltax))

    point3 = np.array([round(point3_x), round(point3_y)], dtype=int)
    point4 = np.array([round(point4_x), round(point4_y)], dtype=int)

    #     print time.time()-start_time, "get_points"
    return point1, point2, point3, point4


# %%

def map_filter(trans, robot_yaw):
    start_time = time.time()
    global origin
    global robot_x
    global robot_y
    global camera_range
    global occ_map
    global bearings
    global display_robot_yaw
    global tags

    map_np, robot_yaw = get_mapnbot_info(trans, robot_yaw)

    slices = []
    i = 0
    while i < len(bearings):

        buoy_yaw = robot_yaw + bearings[i]
        if buoy_yaw < 0:
            buoy_yaw = buoy_yaw + 360
        #         print bearings
        ##############################################################################
        # This is for visualization purposes.
        # Set to false if you don't want to display the robot_yaw
        # Delete it if it's irritating to look at
        if display_robot_yaw and occ_map.header.seq % 5 == 0:
            bearings.append(0.0)
            print "Robot_yaw: ", robot_yaw
            print "Buoy yaw: ", buoy_yaw
            print len(slices)
        ##############################################################################

        pt1, pt2, pt3, pt4 = get_points(buoy_yaw, i)

        #         sliced = createLineIterator(pt1.astype(int),pt2.astype(int),map_np)

        sliced = insidetriangle((pt3[0], pt1[0], pt4[0]), (pt3[1], pt1[1], pt4[1]), map_np)
        slices.append(np.array(sliced).astype(int))
        #         print slices
        i += 1
    #     print time.time()-start_time, "map_filter"
    return slices


# %%

def get_map(slices):
    start_time = time.time()
    global camera_range
    global origin
    global robot_x
    global robot_y
    global map_width
    global map_height
    global occ_map

    map_np = np.reshape(np.array(occ_map.data), (map_height, map_width))

    # Set to 1 when publishing, to 3 for visualization

    map_img_z = np.zeros((map_height, map_width))

    map_img_b = draw_background_objects(map_img_z, map_np)

    map_img_r = draw_ray(map_img_b, slices)
    #     print time.time()-start_time, "get_map"
    return map_img_r


# %%

def callback(rect):
    start_time = time.time()
    global occ_map
    global origin
    global bearings
    global listener
    global display_rect
    global display_map
    global tags
    global up_occ_map
    global pub
    global map_height
    global map_width
    global b_list

    origin = occ_map.info.origin.position
    rect = transform(rect)
    post = time.time()
    if not rospy.is_shutdown():
        now = rospy.Time.now()
        listener.waitForTransform("/odom", "/front_left_camera_link", now, rospy.Duration(1))
        (trans, rot) = listener.lookupTransform("/odom", "/front_left_camera_link", now)
        euler = tf.transformations.euler_from_quaternion(rot)
        pixel_lines = map_filter(trans, math.degrees(euler[2]))
        map_image = get_map(pixel_lines)
        map_obj = EnvMap.EnvMap(map_image, [int(origin.y), int(origin.x)])
        buoy_data = map_obj.get_buoys()
        for item in buoy_data:
            b_list.add_buoy(item)
        map_obj.set_buoys(b_list.get_confirmed_buoys())
        map_image = map_obj.get_map()
        if True:  # set to false if you don't want to publish the updated occupancy map
            up_occ_map.data = np.array(map_image).flatten()
            publish = time.time()
            if pub.get_num_connections > 0:
                pub.publish(up_occ_map)
        #                 print time.time()-publish, "publish"
        if display_rect:
            disp_rect(rect)
        if display_map:
            disp_map(map_image)


#     print time.time()-post, "post"
#     print time.time()-start_time, "callback"
#     for i in range(5):
#         print""

# %%

def map_callback(p_map):
    global occ_map
    global up_occ_map

    occ_map = p_map
    up_occ_map = p_map


# %%

if __name__ == '__main__':
    global b_list
    b_list = Buoy_List.BuoyList()
    proj_map = rospy.Subscriber("projected_map", Map, map_callback)
    rect = rospy.Subscriber("stereo/left/image_rect_color", Image, callback)
    rospy.spin()

# %%



