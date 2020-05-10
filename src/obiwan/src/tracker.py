#!/usr/bin/env python
import math
import random
import numpy as np
from numpy.linalg import norm
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
import imutils

bridge = CvBridge()
frame = 0
pts = []

mask_center_x = -1
mask_center_y = -1
mask_radius = -1
mask_status = -1
detect_status = -1
pub = rospy.Publisher('Image_features', String, queue_size = 10)

class_dict = {
    0:'A_V2',
    1:'B_V2',
    2:'C_V1',
    3:'D_A_B1',
    4:'D_A_B2',
    5:'D_A_B3',
    6:'E_V3',
    7:'G_V3'
}

def track(img, ret):
    global mask_status
    n_frame = 8
    ref_n_frame_axies = []
    ref_n_frame_label = []
    ref_n_frame_axies_flatten = []
    ref_n_frame_label_flatten = []
    label_cnt = 1
    frm_num = 1
    min_distance = 50
    while(True):
        if detect_status == 1:
            cur_frame_axies = []
            cur_frame_label = []
            cv2.imwrite('test.jpg',img)
            out_data = detect(image)
#             outputs = detect(net, meta, "test.jpg")
            for color,output in zip(colors,out_data):
                class_ = output[0]
                class_text =class_dict[int(class_)] 
                x = int(output[1])
                y = int(output[2])
                fw = int(output[3])
                fh = int(output[4])
                w = int(fw/2)
                h = int(fh/2)
                acc = 100
                left = y - h
                top = x - w
                right = y + h
                bottom = x + w
                lbl = float('nan')
                if text in detected_objects:
                    if(len(ref_n_frame_label_flatten) > 0):
                        b = np.array([(x,y)])
                        a = np.array(ref_n_frame_axies_flatten)
                        distance = norm(a-b,axis=1)
                        min_value = distance.min()
                        if(min_value < min_distance):
                            idx = np.where(distance==min_value)[0][0]
                            lbl = ref_n_frame_label_flatten[idx]
                            # print(idx)
                    if(math.isnan(lbl)):
                        lbl = label_cnt
                        label_cnt += 1
                    cur_frame_label.append(lbl)
                    cur_frame_axies.append((x,y))
                    cv2.rectangle(img,(top,left),(bottom,right),color,2)
                    cv2.putText(img,'{}{}-{}%'.format(text,lbl,acc),(top,left), font, 1,(255,255,255),2)
            
            if(len(ref_n_frame_axies) == n_frame):
                del ref_n_frame_axies[0]
                del ref_n_frame_label[0]
                
            ref_n_frame_label.append(cur_frame_label)
            ref_n_frame_axies.append(cur_frame_axies)
            ref_n_frame_axies_flatten = [a for ref_n_frame_axie in ref_n_frame_axies for a in ref_n_frame_axie]
            ref_n_frame_label_flatten = [b for ref_n_frame_lbl in ref_n_frame_label for b in ref_n_frame_lbl]
            cv2.imshow('image',img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
    cv2.destroyAllWindows()


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

def callback_detector(msg):
    global frame
    global 
    n_detect = int(len(data_list)/5)
    data_list = msg.split(",")
    data_list = data_list[:-1]
    all_out = np.zeros((n_detect, 5))
    for j in range(n_detect):
        for i in range(5):
            idx = i + 5*(j)
            if data_list[idx] == 0:
                break;
            all_out[j][i] = data_list[idx] 
    print(all_out)
    
def callback_original(data):
    global frame
    global pts
    global pub
    global mask_center_x
    global mask_center_y
    global mask_radius
    global mask_status
    
    mask_status = -1
    frame+=1
    image_orig = bridge.imgmsg_to_cv2(data, "bgr8")
    light_white = (100, 110, 105)
    dark_white = (250, 255, 255)
    image = cv2.cvtColor(image_orig, cv2.COLOR_BGR2RGB)
    color_ranges= np.array([[light_white, dark_white]])
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
    keypoint_image = cv2.resize(result_image_both,None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    both_mask = cv2.resize(both_mask,None, fx=0.3, fy=0.3, interpolation = cv2.INTER_CUBIC )
    
    # Reverse the mask: blobs are black on white
    reversemask = 255-both_mask
    reversemask = cv2.fastNlMeansDenoising(reversemask)
    keypoints = detector.detect(reversemask)
    max_keypoint = max(keypoints, key=lambda p:p.size) if len(keypoints) else keypoints
    
    try:
        scale_boost = 2
        radius = max_keypoint.size/2*scale_boost + 2
        center = (int(max_keypoint.pt[0]*scale_boost), int(max_keypoint.pt[1]*scale_boost))
        left = (int(center[0] - radius), int(center[1] - radius)) 
        right = (int(center[0] + radius), int(center[1] + radius)) 
        max_keypoint.pt = (max_keypoint.pt[0]*scale_boost, max_keypoint.pt[1]*scale_boost)
        max_keypoint.size *=2

        cv2.rectangle(keypoint_image, left, right, (0, 255, 0), 2)
        line_color=(0,0,255)
        # Image with Rectange
        keypoint_image = cv2.drawKeypoints(keypoint_image, [max_keypoint], np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
    except Exception as e:
        print(e)
        print("Skipped calculation {}".format(frame))
        return 
    
    mask_center_x = center[0]
    mask_center_y = center[1]
    mask_radius = radius
    mask_status = 1
    
    result_image_both = cv2.resize(reversemask, None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC ) #result_image_both
    # image_orig = cv2.resize(image_orig, None, fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC )
    keypoint_image = cv2.rotate(keypoint_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    result_image_both = cv2.rotate(result_image_both, cv2.ROTATE_90_COUNTERCLOCKWISE)
    # image_orig = cv2.rotate(image_orig, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
    concat_full1 = cv2.hconcat([result_image_both, keypoint_image])
    concat_full = concat_full1
    cv2.imshow("Image window", concat_full)
    if cv2.waitKey(3) & 0xFF == ord('q'):
        exit()
    # Publish variables: center and radius
    try:
        
        publish_center = "{},{}".format(center[0], center[1])
        publish_radius = "{:.2f}".format(radius)
        imsh = keypoint_image.shape
        frame_name = "{}".format(frame)
        
        pose = publish_center+","+publish_radius
        state = "{},{},{}".format(imsh[0], imsh[1], imsh[2])
        class_name = identity
        
        publish_str = frame_name+"_"+class_name+"_"+state+ "_" + pose
        
        rospy.loginfo(publish_str)
        pub.publish(publish_str)
        
    except Exception as e:
        print(e)
        print("Skipped publish {}".format(frame))
    
def callback_see(data):
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
        
def detection_manager():
    global frame
    global mask_center_x
    global mask_center_y
    global mask_radius
    
def listener():
    global publish_center
    global publish_radius
    global pub 
    
    rospy.init_node('vision_node', anonymous=True)    
    rospy.Subscriber("/camera/color/image_raw", Image, callback_see)
    rospy.Subscriber("/localizer/target/detector", String, callback_detector)
    
    rospy.spin()

if __name__ == '__main__':
    listener()