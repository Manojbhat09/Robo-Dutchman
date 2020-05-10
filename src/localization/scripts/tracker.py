#!/usr/bin/env python
import math
import random
import numpy as np
from numpy.linalg import norm
import cv2
import rospy
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
import imutils
from tqdm import tqdm 
import time

bridge = CvBridge()
act_frame = 0
det_frame = 0
pts = []

mask_center_x = -1
mask_center_y = -1
mask_radius = -1
mask_status = -1
detect_status = -1
pub_central = rospy.Publisher('/localizer/target/update_get', String, queue_size = 10)
pub_obiwan = rospy.Publisher('/localizer/target/update_ask_obiwan', String, queue_size = 10)
depth = 0
global_image = None
detected_image = None
keypoint_image= None
depth_data = None
depth_image_data = None
answer_ready = None
publish_str = None 
text = ""
progress_bar = tqdm(total=100, position=det_frame, desc=text)
start_time = None
end_time = None
mask_fails = 0

class_dict = {
    0:'A_V2',
    1:'B_V2',
    2:'C_V1',
    3:'D_A_B1',
    4:'D_A_B2',
    5:'D_A_B3',
    6:'E_V3',
    7:'G_V3_up',
    8:'G_V3_down',
    9:'G_V3_none'
}

# def track(img, ret):
#     global mask_status
#     n_frame = 8
#     ref_n_frame_axies = []
#     ref_n_frame_label = []
#     ref_n_frame_axies_flatten = []
#     ref_n_frame_label_flatten = []
#     label_cnt = 1
#     frm_num = 1
#     min_distance = 50
#     while(True):
#         cur_frame_axies = []
#         cur_frame_label = []
#         for color,output in zip(colors,out_data):
#             class_ = output[0]
#             class_text =class_dict[int(class_)] 
#             x = int(output[1])
#             y = int(output[2])
#             fw = int(output[3])
#             fh = int(output[4])
#             w = int(fw/2)
#             h = int(fh/2)
#             acc = 100
#             left = y - h
#             top = x - w
#             right = y + h
#             bottom = x + w
#             lbl = float('nan')
#             if text in detected_objects:
#                 if(len(ref_n_frame_label_flatten) > 0):
#                     b = np.array([(x,y)])
#                     a = np.array(ref_n_frame_axies_flatten)
#                     distance = norm(a-b,axis=1)
#                     min_value = distance.min()
#                     if(min_value < min_distance):
#                         idx = np.where(distance==min_value)[0][0]
#                         lbl = ref_n_frame_label_flatten[idx]
#                         # print(idx)
#                 if(math.isnan(lbl)):
#                     lbl = label_cnt
#                     label_cnt += 1
#                 cur_frame_label.append(lbl)
#                 cur_frame_axies.append((x,y))
# #                 cv2.rectangle(img,(top,left),(bottom,right),color,2)
# #                 cv2.putText(img,'{}{}-{}%'.format(text,lbl,acc),(top,left), font, 1,(255,255,255),2)

#         if(len(ref_n_frame_axies) == n_frame):
#             del ref_n_frame_axies[0]
#             del ref_n_frame_label[0]

#         ref_n_frame_label.append(cur_frame_label)
#         ref_n_frame_axies.append(cur_frame_axies)
#         ref_n_frame_axies_flatten = [a for ref_n_frame_axie in ref_n_frame_axies for a in ref_n_frame_axie]
#         ref_n_frame_label_flatten = [b for ref_n_frame_lbl in ref_n_frame_label for b in ref_n_frame_lbl]
# #             cv2.imshow('image',img)
# #             if cv2.waitKey(1) & 0xFF == ord('q'):
# #                 break
#         else:
#             break
#     cv2.destroyAllWindows()


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
    global det_frame
    global global_image
    global detected_image
    
    
    print("[Tracker] callback_detector")
    data_list = msg.data.split(",")
    data_list = data_list[:-1]
    
    n_detect = int(len(data_list)/5)
    print("Len : {} {}".format(n_detect,det_frame))
    
    display_image = global_image.copy()
    all_out = np.zeros((n_detect, 5))
    detections = []
    for j in range(n_detect):
        for i in range(5):
            idx = i + 5*(j)
            if data_list[idx] == 0:
                break;
            all_out[j][i] = data_list[idx] 
        class_, x, y, w, h = all_out[j]
        print("[Tracker] Printing a detect")
        print(all_out[j])
        detections.append(all_out[j])
#         boost = (1/0.6)
        boost = 1
#         cv2.rectangle(display_image, (int(x), int(y)), (int(x+w), int(y+h)), (255,0,0), 2)
    detections=np.array(detections)
    print("[Tracker] printing all out")
    print(detections)
    print("[Tracker] Going into detector_manager")
    detection_manager(detections)
#     path = os.path.join("/home/robodutchman/Robo-Dutchman/Images", "detect_{}".format(det_frame)+".jpg")
#     cv2.imwrite(path, display_image)
    detected_image= np.array(display_image)
    
    print("[Tracker] DETECTED:")
#     print(all_out)
    det_frame+=1
#     # Display
#     cv2.imshow("Image window", global_image)
#     if cv2.waitKey(3) & 0xFF == ord('q'):
#         exit()
#     track(image, all_out)
    
def callback_original_old(data):
    global act_frame
    global global_image
    global start_time 
    global det_frame
    
    start_time = time.time()
    # print("[Tracker] callback_original")
    image_orig = bridge.imgmsg_to_cv2(data, "rgb8")
    image_orig_rotated = cv2.rotate(image_orig, cv2.ROTATE_90_COUNTERCLOCKWISE)
    global_image = image_orig_rotated
    act_frame+=1
    
    progress_bar.set_description("Done in {} sec".format(time.time() - start_time))
    progress_bar.update(det_frame)
#     progress_bar.refresh()
    
def show_image(global_image, detect_window=False, depth=False):
    global start_time 
    global progress_bar
    global depth_data 
    global depth_image_data
    
    # Display
    if depth and isinstance(depth_data, np.ndarray ):
        
        merged_ = cv2.addWeighted(global_image,0.4,depth_image_data , 0.5, 0)
        cv2.imshow("Depth window", merged_)
        if cv2.waitKey(3) & 0xFF == ord('q'):
            exit()
        
    cv2.imshow("Image window", global_image)
    if cv2.waitKey(3) & 0xFF == ord('q'):
        exit()
    if detect_window:
        cv2.imshow("Detect window", global_image)
        if cv2.waitKey(3) & 0xFF == ord('q'):
            exit()
    progress_bar.set_description("Done in {} sec".format(time.time() - start_time))
    progress_bar.update(det_frame)
    
def callback_original(data):
    global act_frame
    global global_image
    global start_time 
    global det_frame
    global keypoint_image
    global detected_image
    global image_orig_rotated
    
    start_time = time.time()
    image_orig = bridge.imgmsg_to_cv2(data, "rgb8")
    image = cv2.cvtColor(image_orig, cv2.COLOR_BGR2RGB)
    image_orig_rotated = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    global_image = image_orig_rotated
    show_image(global_image, False, False)
        
def get_mask(image_orig):
    global mask_center_x
    global mask_center_y
    global mask_radius
    global mask_status
    global det_frame
    global mask_fails
    global keypoint_image
    global pub_obiwan
    global global_image
    global pub_central
    global publish_str
    
    
    # publish update to obiwan
    print("[Tracker] Publishing to obiwan")
    pub_obiwan.publish("1")
    
    # Starting masking task 
    light_white = (100, 110, 105)
    dark_white = (250, 255, 255)
    image = cv2.cvtColor(image_orig, cv2.COLOR_BGR2RGB)
#     path = os.path.join("/home/robodutchman/Robo-Dutchman/Images", "frame_{}".format(det_frame)+".jpg")
#     cv2.imwrite(path, image)
    color_ranges= np.array([[light_white, dark_white]])
    result_image_both , both_mask = Masking2(image_orig,image, color_ranges)
#     result_image_both = cv2.cvtColor(result_image_both, cv2.COLOR_RGB2BGR) 
    
    # Keypoint detection
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

    # Apply blob detection
    detector = cv2.SimpleBlobDetector_create(params)
    
    ig_scl = 0.6
    mk_scl = 0.3
    # Scale result image to put bounding box
    keypoint_image = cv2.resize(result_image_both,None, fx=ig_scl, fy=ig_scl, interpolation = cv2.INTER_CUBIC )
    # Scale mask to reduce pixels 
    both_mask = cv2.resize(both_mask,None, fx=mk_scl, fy=mk_scl, interpolation = cv2.INTER_CUBIC )
    
    # Reverse the mask: blobs are black on white
    reversemask = 255-both_mask
    reversemask = cv2.fastNlMeansDenoising(reversemask)
    reversemask = cv2.fastNlMeansDenoising(reversemask)
    keypoints = detector.detect(reversemask)
    
    if keypoints:
        max_keypoint = max(keypoints, key=lambda p:p.size)
    
        # As mask scaled to 0.3, scale up to -> 0.3*2 = 0.6 
        try:
            scale_boost = ig_scl/mk_scl
            select_keypoint_size = (max_keypoint[0].size if type(max_keypoint) == list else max_keypoint.size )
            radius = select_keypoint_size/2*scale_boost + 2
            center = (int(max_keypoint.pt[0]*scale_boost), int(max_keypoint.pt[1]*scale_boost))
            max_keypoint.pt = (max_keypoint.pt[0]*scale_boost, max_keypoint.pt[1]*scale_boost)
            if type(max_keypoint) != list:
                max_keypoint.size *=2
            
            left = (int(center[0] - radius), int(center[1] - radius)) 
            right = (int(center[0] + radius), int(center[1] + radius)) 
            cv2.rectangle(keypoint_image, left, right, (0, 255, 0), 2)
            # Image with Rectange
            keypoint_image = cv2.drawKeypoints(keypoint_image, [max_keypoint], np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
#             path = os.path.join("/home/robodutchman/Robo-Dutchman/Images", "keypoint_{}".format(det_frame)+".jpg")
#             cv2.imwrite(path, keypoint_image)
            
            print("[Tracker] Mask success")
            mask_center_x = center[0]*(1/ig_scl)
            mask_center_y = center[1]*(1/ig_scl)
            mask_radius = radius*(1/ig_scl)
            mask_status = 1
            
            # Display
#             cv2.imshow("Image window", reversemask)
#             if cv2.waitKey(3) & 0xFF == ord('q'):
#                 exit()
#             path = os.path.join("/home/robodutchman/Robo-Dutchman/Images", "mask_{}".format(det_frame)+".jpg")
#             cv2.imwrite(path, reversemask)

        except Exception as e:
            print(e)
            print("Skipped calculation {}".format(det_frame))
            mask_status = 0
            mask_fails += 1
            print("[Tracker][Error] Mask failed, no keypoints") 
            get_mask(global_image)
            if (mask_fails > 5):
                print("[Tracker][Error] No Mask generated, moving on ") 
                pub_central.publish(publish_str)
            return 
    else:
        mask_status = 0
        mask_fails += 1
        print("[Tracker][Error] Mask failed, no keypoints") 
        get_mask(global_image)
        if (mask_fails > 5):
            print("[Tracker][Error] No Mask generated, moving on ") 
            pub_central.publish(publish_str)
        return
        
def callback_see(data):
    global act_frame
    global pts
    global publish_center
    global publish_radius
    global pub 
    global time_start_flag
    global start_time
    image_orig = bridge.imgmsg_to_cv2(data, "rgb8")
    image = cv2.cvtColor(image_orig, cv2.COLOR_BGR2RGB)
    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    # Display
    cv2.imshow("Image window", image)
    if cv2.waitKey(3) & 0xFF == ord('q'):
        exit()
        
def callback_depth(data):
    global act_frame
    global depth
    image_orig = bridge.imgmsg_to_cv2(data, "rgb8")
    image = cv2.cvtColor(image_orig, cv2.COLOR_BGR2RGB)
    # Display
    cv2.imshow("Image window", image)
    if cv2.waitKey(3) & 0xFF == ord('q'):
        exit()
        
def detection_manager(detector_data):
    global det_frame
    global mask_center_x
    global mask_center_y
    global mask_radius
    global pub_central
    global answer_ready
    global publish_str
    global global_image
    
    print("[Tracker] detection_manager")
    publish_str = ""
    for each in  detector_data:
        c,x,y,w,h = each
        print(c,x,y,w,h)
        # Breaker check and modify state
        xc, yc, r = mask_center_x, mask_center_y, mask_radius
        if c == 3 or c == 4 or c == 5:
            bbox_center = np.array([x+(w/2), y+(h/2)])
            mask_center = np.array([xc, yc])
            if ((bbox_center - mask_center)[1] > 0):
                brkr_class = 'U'
            else:
                brkr_class = 'D'
            c_str = str(c) + "_"+ brkr_class
        else:
            c_str = str(c)+"_"+ "N"
        cv2.rectangle(global_image, (int(x), int(y)), (int(x+w), int(y+h)), (255,0,0), 2)
        cv2.circle(global_image, (int(xc), int(yc)), 10, (0,255,0), thickness=2)
        
        publish_str_update = c_str + "_" + str(x) + "_" + str(y) + "_" + str(w) + "_" + str(h)
        publish_str += publish_str_update+"/"
    
    show_image(global_image, True, False)
    answer_ready = 1
    print(publish_str)
    

def callback_update(msg):
    global global_image
    global pub_central
    global pub_obiwan
    global answer_ready
    global mask_status
    global end_time
    global det_frame
    global publish_str
    
    # Ready masking stage
    print("[Tracker] Getting masking stage")
    get_mask(global_image)
#     while mask_status != 1:
#         get_mask(global_image)
#     mask_status = 0
    
    # Wait until data is ready to publish
    print("[Tracker] Making sure ready")
    while answer_ready==None:
        continue
    
    # publish answer to central
    print("[Tracker] Central publisher")
    while publish_str==None:
        continue
        
    print("[Tracker] Publishing")
    pub_central.publish(publish_str)
    publish_str = None
    answer_ready = None
    
#     end_time = time.time()
#     progress_bar.set_description("Done in {} sec".format(end_time - start_time))
#     progress_bar.update(det_frame)
#     progress_bar.refresh()
    
    
def callback_depth(data):
    global depth_data 
    global depth_image_data
    global act_frame
    global det_frame

    try:
        # Incoding data is 16bit, check by errors, convert to range of 256
        depth_image = bridge.imgmsg_to_cv2(data, "16UC1")
        depth_image = (depth_image/16).astype(np.uint8)
        
        # Reshape or unsqueeze to add a channel
        shape = np.array(depth_image).shape
        depth_image = np.reshape(depth_image, (shape[0], shape[1], 1))
        
        # apply RAINBOW, can add JET, WINTER, BONE, OCEAN, SUMMER, SPRING, COOL etc.
        depth_image_data = cv2.applyColorMap(depth_image, cv2.COLORMAP_RAINBOW)
        depth_image_data = cv2.rotate(depth_image_data, cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_image_data = np.array(depth_image_data)
        
        depth_data = cv2.rotate(depth_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_data = np.array(depth_data)
    except CvBridgeError, e:
        print(e)
        
    
def listener():
    global publish_center
    global publish_radius
    global pub 
    
    rospy.init_node('vision_node', anonymous=True)    
#     rospy.Subscriber("/camera/color/image_raw", Image, callback_see)
#     rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback_depth)
    rospy.Subscriber("/camera/color/image_raw", Image, callback_original)
    rospy.Subscriber("/localizer/target/update_ask", String, callback_update)
    rospy.Subscriber("/localizer/target/detect", String, callback_detector)
    # rospy.Subscriber("/camera/color/depth", Image, callback_depth)
    
    rospy.spin()

if __name__ == '__main__':
    listener()