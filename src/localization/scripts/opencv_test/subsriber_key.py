#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import imutils

bridge = CvBridge()
frame = 0
pts = []

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
    
    final_mask = cv2.dilate(final_mask, None, iterations=20)
    final_mask = cv2.erode(final_mask, None, iterations=20)
        
    final_image = cv2.bitwise_and(image, image, mask=final_mask)
    blurred_image = cv2.GaussianBlur(final_image, (7,7), 0)
    return blurred_image, final_mask

def callback(data):
    global frame
    global pts
    image_orig = bridge.imgmsg_to_cv2(data, "bgr8")
    
    light_white = (100, 110, 105)
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
        params.minArea = 50
        params.maxArea = 20000
         
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.01
         
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.01
         
        # Filter by Inertia
        params.filterByInertia =True
        params.minInertiaRatio = 0.01
         
    else:
        params = blob_params     

    #- Apply blob detection
    detector = cv2.SimpleBlobDetector_create(params)

    # result_image_both
    keypoint_image = cv2.resize(image_orig,None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    both_mask = cv2.resize(both_mask,None, fx=0.3, fy=0.3, interpolation = cv2.INTER_CUBIC )
    
    # Reverse the mask: blobs are black on white
    reversemask = 255-both_mask
    
    keypoints = detector.detect(reversemask)
    print("Keypoints in frame {} is {}".format(frame, len(keypoints)) )
    
    
    max_keypoint = max(keypoints, key=lambda p:p.size) if len(keypoints) else keypoints
    
#     import pdb; pdb.set_trace()
    reversemask = cv2.fastNlMeansDenoising(reversemask)
    
    scale_boost = 2
    radius = max_keypoint.size/2*scale_boost + 2
    center = (int(max_keypoint.pt[0]*scale_boost), int(max_keypoint.pt[1]*scale_boost))
    left = (int(center[0] - radius), int(center[1] - radius)) 
    right = (int(center[0] + radius), int(center[1] + radius)) 
    max_keypoint.pt = (max_keypoint.pt[0]*scale_boost, max_keypoint.pt[1]*scale_boost)
    max_keypoint.size *=2
    
    cv2.rectangle(keypoint_image, left, right, (0, 255, 0), 2)
#     cv2.circle(keypoint_image, (int(), int(y)), int(radius), (0, 255, 255), 2)
    line_color=(0,0,255)
    keypoint_image = cv2.drawKeypoints(keypoint_image, [max_keypoint], np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        
#     # find contours in the mask and initialize the current
#     # (x, y) center of the ball
#     cnts = cv2.findContours(reversemask.copy(), cv2.RETR_EXTERNAL,
#         cv2.CHAIN_APPROX_SIMPLE)
#     cnts = imutils.grab_contours(cnts)
#     center = None
#     # only proceed if at least one contour was found
#     if len(cnts) > 0:
#         # find the largest contour in the mask, then use
#         # it to compute the minimum enclosing circle and
#         # centroid
#         c = max(cnts, key=cv2.contourArea)
#         ((x, y), radius) = cv2.minEnclosingCircle(c)
#         M = cv2.moments(c)
#         center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
#         # only proceed if the radius meets a minimum size
#         if radius > 10:
#             # draw the circle and centroid on the frame,
#             # then update the list of tracked points
#             cv2.circle(keypoint_image, (int(x), int(y)), int(radius),
#                 (0, 255, 255), 2)
#             cv2.circle(keypoint_image, center, 5, (0, 0, 255), -1)
#     # update the points queue
#     pts.append(center)
#     print("pts list len {}".format(len(pts)))
    
    
#     if len(pts)>2:
#         for i in range(1, len(pts)):
#             # if either of the tracked points are None, ignore
#             # them
#             if pts[i - 1] is None or pts[i] is None:
#                 continue
#             # otherwise, compute the thickness of the line and
#             # draw the connecting lines
#             thickness = int(np.sqrt(128 / float(i + 1)) * 2.5)
#             cv2.line(keypoint_image, pts[i - 1], pts[i], (0, 0, 255), thickness)
    
#     cv2.circle(keypoint_image, center, int(radius), (100, 100, 200), thickness=1, lineType=8, shift=0)
#     cv2.imshow("Keypoints_blob", keypoint_image)
    
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         exit()
        
    result_image = cv2.resize(result_image,None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    result_image_orange = cv2.resize(result_image_orange, None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    result_image_both = cv2.resize(result_image_both, None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    image_orig = cv2.resize(image_orig, None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    
    concat_full1 = cv2.hconcat([result_image, image_orig])
    concat_full2 = cv2.hconcat([result_image_both, keypoint_image])
    concat_full = cv2.vconcat([concat_full1, concat_full2])
    cv2.imshow("Image window", concat_full)
    if cv2.waitKey(3) & 0xFF == ord('q'):
        exit()
    
#     np.save(reversemask, frame)
    
    frame+=1
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()