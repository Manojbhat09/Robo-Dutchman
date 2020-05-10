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
import time

# ROOT = os.path.dirname(os.path.abspath(__file__))
ROOT = os.getcwd()
bridge = CvBridge()
depth1 = np.zeros((480,640,1))
frame = 0
pts = []

publish_center = "0,0"
publish_radius = "0"
pub = rospy.Publisher('Image_features', String, queue_size = 10)

start_time = time.time()
time_start_flag = 1

def check_make_(save_dir, display=True):
    if not os.path.exists(save_dir):
        if display:
            print("Making folder ", save_dir)
        os.makedirs(save_dir)
    else:
        if display:
            print("Folder present ", save_dir)
        else:
            pass
    
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
    global publish_center
    global publish_radius
    global pub 
    global time_start_flag
    global start_time
    image_orig = bridge.imgmsg_to_cv2(data, "rgb8")
    image = cv2.cvtColor(image_orig, cv2.COLOR_BGR2RGB)
    # Display
    cv2.imshow("Image window", image)
    if cv2.waitKey(3) & 0xFF == ord('q'):
        exit()


def callback_other(data):
    global frame
    global pts
    global publish_center
    global publish_radius
    global pub 
    global time_start_flag
    global start_time
    
    if time_start_flag:
        start_time = time.time()
        time_start_flag = 0
    
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
    max_keypoint = max(keypoints, key=lambda p:p.size) if len(keypoints) else keypoints
    reversemask = cv2.fastNlMeansDenoising(reversemask)
    
    try:
        scale_boost = 2
        radius = max_keypoint.size/2*scale_boost + 2
        center = (int(max_keypoint.pt[0]*scale_boost), int(max_keypoint.pt[1]*scale_boost))
        left = (int(center[0] - radius), int(center[1] - radius)) 
        right = (int(center[0] + radius), int(center[1] + radius)) 
        max_keypoint.pt = (max_keypoint.pt[0]*scale_boost, max_keypoint.pt[1]*scale_boost)
        max_keypoint.size *=2
        
        save_path = os.path.join(ROOT, "keypoint", "keypoint_{}".format(frame))
        check_make_(os.path.dirname(save_path), False)
        dictt = {'radius':radius,
                 'center': center,
                 'left': left,
                 'right': right}
        #np.save(save_path, dictt)        
        

        cv2.rectangle(keypoint_image, left, right, (0, 255, 0), 2)
    #     cv2.circle(keypoint_image, (int(), int(y)), int(radius), (0, 255, 255), 2)
        line_color=(0,0,255)
        # Image with Rectange
        keypoint_image = cv2.drawKeypoints(keypoint_image, [max_keypoint], np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    except Exception as e:
        print(e)
        print("Skipped {}".format(frame))
        return 
    
    
    save_path = os.path.join(ROOT, "keypoint_images", "keypoint_{}".format(frame))
    check_make_(os.path.dirname(save_path), False)
    #np.save(save_path, keypoint_image)
    
    save_path = os.path.join(ROOT, "mask_images", "mask_{}".format(frame))
    check_make_(os.path.dirname(save_path), False)
    #np.save(save_path, result_image_both)
    
    save_path = os.path.join(ROOT, "color_images", "color_{}".format(frame))
    check_make_(os.path.dirname(save_path), False)
    #np.save(save_path, image_orig)
        
    result_image = cv2.resize(result_image,None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    result_image_orange = cv2.resize(result_image_orange, None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    result_image_both = cv2.resize(result_image_both, None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    image_orig = cv2.resize(image_orig, None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    
    result_image = cv2.rotate(result_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    keypoint_image = cv2.rotate(keypoint_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    result_image_both = cv2.rotate(result_image_both, cv2.ROTATE_90_COUNTERCLOCKWISE)
    image_orig = cv2.rotate(image_orig, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
    concat_full1 = cv2.hconcat([result_image, image_orig])
    concat_full2 = cv2.hconcat([result_image_both, keypoint_image])
    concat_full = cv2.vconcat([concat_full1, concat_full2])
#     cv2.imshow("Image window", concat_full)
#     if cv2.waitKey(3) & 0xFF == ord('q'):
#         exit()
        
    # Publish variables: center and radius
    try:
        publish_center = "{},{}".format(center[0], center[1])
        publish_radius = "{:.2f}".format(radius)
        
        imsh = keypoint_image.shape
        publish_str = publish_center+","+publish_radius
        publish_str = publish_str+" shape:{},{},{}".format(imsh[0], imsh[1], imsh[2])
        publish_str = publish_str+" frame:{}".format(frame)
        rospy.loginfo(publish_str)
        pub.publish(publish_str)
        
    except Exception as e:
        print(e)
        print("Skipped {}".format(frame))
    frame+=1
    
    end_time = time.time()
    seconds = start_time - end_time
    print("Time taken : {0} seconds".format(seconds))

    fps = (seconds)/frame
    print("Estimated frames per second : {0}".format(fps));
    
# def depth_callback(data):
#     print("Data found")
    
def callback_depth(data):
    global bridge
    global depth1
    global frame

    try:
        # Incoding data is 16bit, check by errors, convert to range of 256
        depth_image = bridge.imgmsg_to_cv2(data, "16UC1")
        depth_image = (depth_image/16).astype(np.uint8)
        save_path = os.path.join(ROOT, "depth_values", "depth_{}".format(frame))
        check_make_(os.path.dirname(save_path), False)
        np.save(save_path, depth_image)
        
        # Reshape or unsqueeze to add a channel
        shape = np.array(depth_image).shape
        depth_image = np.reshape(depth_image, (shape[0], shape[1], 1))
        
        # apply RAINBOW, can add JET, WINTER, BONE, OCEAN, SUMMER, SPRING, COOL etc.
        depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_RAINBOW)
        
        # Store in shared variable
        depth1 = depth_image
        
        save_path = os.path.join(ROOT, "depth_images", "depth_{}".format(frame))
        check_make_(os.path.dirname(save_path), False)
        np.save(save_path, depth1)
        
        
        # Display
        cv2.imshow("Image window", depth1)
        if cv2.waitKey(3) & 0xFF == ord('q'):
            exit()
            
    except CvBridgeError as e:
        print(e)
    
def listener():
    global publish_center
    global publish_radius
    global pub 
    
    rospy.init_node('vision_node', anonymous=True)    
    rospy.Subscriber("/device_0/sensor_1/Color_0/image/data", Image, callback)
    # rospy.Subscriber("/camera/color/image_raw", Image, callback)

    
    rospy.spin()

if __name__ == '__main__':
    listener()
