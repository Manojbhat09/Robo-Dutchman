#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import imutils
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#matplotlib.interactive(True)

cwd = os.getcwd()
bridge = CvBridge()
depth1 = np.zeros((480,640,1))
depth2 = np.zeros((480,640,1))
depth3 = np.zeros((480,640,1))
color1 = np.zeros((480,640,3))
color2 = np.zeros((480,640,3))

    
def callback_depth(data):
    global bridge
    global depth1
    global frame

    try:
        # Incoding data is 16bit, check by errors, convert to range of 256
        depth_image = bridge.imgmsg_to_cv2(data, "16UC1")
        depth_image = (depth_image/16).astype(np.uint8)
        
        # Reshape or unsqueeze to add a channel
        shape = np.array(depth_image).shape
        depth_image = np.reshape(depth_image, (shape[0], shape[1], 1))
        
        # apply RAINBOW, can add JET, WINTER, BONE, OCEAN, SUMMER, SPRING, COOL etc.
        depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_RAINBOW)
        
        # Store in shared variable
        depth1 = depth_image
        
        path_save = os.path.join(cwd, "depth_images", "{}".format(frame))
        np.save(depth1, path_save)
        
        # Concatenate with other frames
        concat_full1 = cv2.hconcat([depth1, depth2])
        concat_full1_1 = cv2.hconcat([concat_full1, depth3])
        dummy = np.zeros((480,640,3), dtype=np.uint8)
        concat_full2 = cv2.hconcat([color1, color2])
        concat_full2_1 = cv2.hconcat([concat_full2, dummy])
        concat_full_all = cv2.vconcat([concat_full1_1, concat_full2_1])
        
        # Display
        cv2.imshow("Image window", concat_full_all)
        if cv2.waitKey(3) & 0xFF == ord('q'):
            exit()
            
    except CvBridgeError, e:
        print e
        
def callback_depth_rect(data):
    global bridge
    global depth2
    
    try:
        depth_image = bridge.imgmsg_to_cv2(data, "16UC1")
        depth_image = (depth_image/16).astype(np.uint8)
        shape = np.array(depth_image).shape
        depth_image = np.reshape(depth_image, (shape[0], shape[1], 1))
        depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_RAINBOW)
        depth2 = depth_image
    except CvBridgeError, e:
        print e
        
def callback_depth_infra(data):
    global bridge
    global depth3

    try:
        depth_image = bridge.imgmsg_to_cv2(data, "16UC1")
        depth_image = (depth_image/16).astype(np.uint8)
        shape = np.array(depth_image).shape
        depth_image = np.reshape(depth_image, (shape[0], shape[1], 1))
        depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_RAINBOW)
        depth3 = depth_image
    except CvBridgeError, e:
        print e
        
def callback_color(data):
    global bridge
    global color1
    global frame

    try:
        color_image = bridge.imgmsg_to_cv2(data, "rgb8")
        color_image = ( (color_image - np.min(color_image))\
                       *256/(np.max(color_image) - np.min(color_image)) ).astype(np.uint8)
        color1 = color_image
        
        path_save = os.path.join(cwd, "depth_images", "{}".format(frame))
        np.save(depth1, path_save)
        
    except CvBridgeError, e:
        print(e)
        
def callback_infra_rect(data):
    global bridge
    global color2

    try:
        color_image = bridge.imgmsg_to_cv2(data, "rgb8")
        color_image = ( (color_image - np.min(color_image))\
                       *256/(np.max(color_image) - np.min(color_image)) ).astype(np.uint8)
        color2 = color_image
    except CvBridgeError, e:
        print(e)
    
# def callback_scatter(data):
#     global bridge
#     global color1

#     try:
#         color_image = bridge.imgmsg_to_cv2(data, "rgb8")
#         color_image = ( (color_image - np.min(color_image))\
#                        *256/(np.max(color_image) - np.min(color_image)) ).astype(np.uint8)
#         (r, c, b) = np.shape(color_image)
        
#         # X and Y coordinates of points in the image, spaced by 10.
#         (X, Y) = np.meshgrid(range(0, c, 10), range(0, r, 10))

#         # Display the image
#         plt.imshow(color_image)
#         # Plot points from the image.
#         plt.scatter(X, Y, color_image[Y,X])
#         plt.show()
        
#     except CvBridgeError, e:
#         print(e)
    
    
def listener():    
    rospy.init_node('vision_node', anonymous=True)    
    
    # Display all messages
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback_depth)
    rospy.Subscriber("/camera/color/image_raw", Image, callback_color)
    rospy.Subscriber("/camera/infra1/image_rect_raw", Image, callback_infra_rect)
    rospy.Subscriber("/camera/aligned_depth_to_infra1/image_raw", Image, callback_depth_infra)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback_depth_rect)
    
    # Scatter plot
#     rospy.Subscriber("/camera/color/image_raw", Image, callback_scatter)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
