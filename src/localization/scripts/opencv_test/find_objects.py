#!/usr/bin/env python

"""
ON THE RASPI: roslaunch ...
   0------------------> x (cols) Image Frame
   |
   |        c    Camera frame
   |         o---> x
   |         |
   |         V y
   |
   V y (rows)
   
SUBSCRIBES TO:
    /mask_node/image: Source image topic
   
    
PUBLISHES TO:
    /mask_node/image_blob : image with detected blob and search window
    /mask_node/image_mask : masking    
    /mask_node/point_blob : blob position in adimensional values wrt. camera frame
"""


#--- Allow relative importing
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
import sys
import rospy
import cv2
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from include.blob_detector  import *

class ObjectDetector:
    def __init__(self, )


class BlobDetector:

    def __init__(self, thr_min, thr_max, blur=15, blob_params=None, detection_window=None):
    
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window
        
        self._t0 = time.time()
        
        self.blob_point = Point()
    
        print (">> Publishing image to topic image_blob")
        self.image_pub = rospy.Publisher("/mask_node/image_blob",Image,queue_size=1)
        self.mask_pub = rospy.Publisher("/mask_node/image_mask",Image,queue_size=1)
        print (">> Publishing position to topic point_blob")
        self.blob_pub  = rospy.Publisher("/mask_node/point_blob",Point,queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.callback)
        print ("<< Subscribed to topic /raspicam_node/image")
        
    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]
        
    def set_blur(self, blur):
        self._blur = blur
      
    def set_blob_params(self, blob_params):
        self._blob_params = blob_params
        
    def get_blob_relative_position(self, cv_image, keyPoint):
        rows = float(cv_image.shape[0])
        cols = float(cv_image.shape[1])
        # print(rows, cols)
        center_x    = 0.5*cols
        center_y    = 0.5*rows
        # print(center_x)
        x = (keyPoint.pt[0] - center_x)/(center_x)
        y = (keyPoint.pt[1] - center_y)/(center_y)
        return(x,y)
        
    def set_mask(image, color_ranges):
        '''
        Algorithm to mask the image
        
        Args:
            image- The image to mask
            color_ranges- list of color ranges ( list of max min range of color )
        Returns:
            blurred_image- Final image after processing
            final_mask- Mask over image 
        '''
        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        final_mask = np.full(image.shape[:-1],0,dtype=np.uint8)
        for each_range in color_ranges:
            final_mask += cv2.inRange(hsv_image, each_range[0], each_range[1])
        final_image = cv2.bitwise_and(image, image, mask=final_mask)
        blurred_image = cv2.GaussianBlur(final_image, (7,7), 0)
        return blurred_image, final_mask
        
    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            #--- Detect blobs
            keypoints, mask   = blob_detect(cv_image, self._threshold[0], self._threshold[1], self._blur,
                                            blob_params=self._blob_params, search_window=self.detection_window )
            #--- Draw search window and blobs
            cv_image    = blur_outside(cv_image, 10, self.detection_window)

            cv_image    = draw_window(cv_image, self.detection_window, line=1)
            cv_image    = draw_frame(cv_image)
            
            cv_image    = draw_keypoints(cv_image, keypoints) 
            
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))
            except CvBridgeError as e:
                print(e)            
                    
            fps = 1.0/(time.time()-self._t0)
            self._t0 = time.time()
            
def main(args):
    blue_min = (77,40,0)
    blue_max = (101, 255, 255) 
    # blue_min = (82,31,62)
    # blue_max = (106, 116, 193)     
    blue_min = (55,40,0)
    blue_max = (150, 255, 255)     
    
    blur     = 5
    min_size = 10
    max_size = 40
    
    #--- detection window respect to camera frame in [x_min, y_min, x_max, y_max] adimensional (0 to 1)
    x_min   = 0.1
    x_max   = 0.9
    y_min   = 0.4
    y_max   = 0.9
    
    detection_window = [x_min, y_min, x_max, y_max]
    
    params = cv2.SimpleBlobDetector_Params()
         
    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 100;
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 20000
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.2
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.7   

    ic = BlobDetector(blue_min, blue_max, blur, params, detection_window)
    rospy.init_node('blob_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)