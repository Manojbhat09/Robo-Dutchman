#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

def Masking(image, color_ranges):
    image = cv2.GaussianBlur(image, (9,9), 80)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    final_mask = np.full(image.shape[:-1],0,dtype=np.uint8)
    for each_range in color_ranges:
        final_mask += cv2.inRange(hsv_image, each_range[0], each_range[1])
    final_image = cv2.bitwise_and(image, image, mask=final_mask)
    blurred_image = cv2.GaussianBlur(final_image, (7,7), 0)
    return blurred_image

def Masking2(image, image2, color_ranges):
    image = cv2.GaussianBlur(image, (9,9), 80)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    image2 = cv2.GaussianBlur(image2, (9,9), 80)
    hsv_image2 = cv2.cvtColor(image2, cv2.COLOR_RGB2HSV)
    final_mask = np.full(image.shape[:-1],0,dtype=np.uint8)
    for each_range in color_ranges:
        final_mask += cv2.inRange(hsv_image, each_range[0], each_range[1])
        final_mask += cv2.inRange(hsv_image2, each_range[0], each_range[1])
    
    final_mask = cv2.dilate(final_mask, None, iterations=2)
    final_mask = cv2.erode(final_mask, None, iterations=2)
        
    final_image = cv2.bitwise_and(image, image, mask=final_mask)
    blurred_image = cv2.GaussianBlur(final_image, (7,7), 0)
    return blurred_image, final_mask

def draw_keypoints(image,                   #-- Input image
                   keypoints,               #-- CV keypoints
                   line_color=(0,0,255),    #-- line's color (b,g,r)
                   imshow=False             #-- show the result
                  ):
    
    #-- Draw detected blobs as red circles.
    #-- cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
    if imshow:
        # Show keypoints
        cv2.imshow("Keypoints", im_with_keypoints)
        
    return(im_with_keypoints)

def callback(data):
    image_orig = bridge.imgmsg_to_cv2(data, "bgr8")
    
    light_white = (100, 100, 105)
    dark_white = (250, 255, 255)
    image = cv2.cvtColor(image_orig, cv2.COLOR_BGR2RGB)
    color_ranges= np.array([[light_white, dark_white]])
    result_image = Masking(image, color_ranges)
    result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
    
    result_image_orange = Masking(image_orig, color_ranges)
    result_image_both , both_mask = Masking2(image, image_orig, color_ranges)
    result_image_both = cv2.cvtColor(result_image_both, cv2.COLOR_RGB2BGR)
    
    blob_params = None
    if blob_params is None:
        # Set up the SimpleBlobdetector with default parameters.
        params = cv2.SimpleBlobDetector_Params()
         
        # Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 100;
         
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 30
        params.maxArea = 20000
         
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
         
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5
         
        # Filter by Inertia
        params.filterByInertia =True
        params.minInertiaRatio = 0.5
         
    else:
        params = blob_params     

    #- Apply blob detection
    detector = cv2.SimpleBlobDetector_create(params)

    # Reverse the mask: blobs are black on white
    reversemask = 255-both_mask
    
    keypoints = detector.detect(reversemask)
    
    
#     import pdb; pdb.set_trace()
    
    result_image_both     = draw_keypoints(result_image_both, keypoints)
    
    
    result_image = cv2.resize(result_image,None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    result_image_orange = cv2.resize(result_image_orange, None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    result_image_both = cv2.resize(result_image_both, None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    image_orig = cv2.resize(image_orig, None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    
    concat_full1 = cv2.hconcat([result_image, image_orig])
    concat_full2 = cv2.hconcat([result_image_orange, result_image_both])
    concat_full = cv2.vconcat([concat_full1, concat_full2])
    cv2.imshow("Image window", concat_full)
    cv2.waitKey(3)
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/device_0/sensor_1/Color_0/image/data", Image, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()