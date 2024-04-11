import rclpy                            
from rclpy.node import Node             
from sensor_msgs.msg import CompressedImage       
from std_msgs.msg import String  
from cv_bridge import CvBridge          
import cv2                              
import numpy as np          

lower_red = np.array([49, 9, 55])      
upper_red = np.array([112, 46, 255])  

def change_threshold_V(v):
    lower_red[2] = v

def change_threshold_S(s):
    lower_red[1] = s

def change_threshold_H(h):
    lower_red[0] = h

def hchange_threshold_V(v):
    upper_red[2] = v

def hchange_threshold_S(s):
    upper_red[1] = s

def hchange_threshold_H(h):
    upper_red[0] = h

class ImageSubscriber(Node):
    def init(self):
        super().init('img_node')                                  
        self.sub = self.create_subscription(
            CompressedImage, '/camera/image/compressed', self.listener_callback, 10)     
        self.cv_bridge = CvBridge()   
        cv2.namedWindow('image')
        self.publisher = self.create_publisher(String,'/detected_objects',10)
        
        # create trackbars for color change
        cv2.createTrackbar('V','image',lower_red[2],255,change_threshold_V)
        cv2.createTrackbar('S','image',lower_red[1],255,change_threshold_S)
        cv2.createTrackbar('H','image',lower_red[0],179,change_threshold_H)
        cv2.createTrackbar('hV','image',upper_red[2],255,hchange_threshold_V)
        cv2.createTrackbar('hS','image',upper_red[1],255,hchange_threshold_S)
        cv2.createTrackbar('hH','image',upper_red[0],179,hchange_threshold_H)

    def object_detect(self, image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   
        mask_red = cv2.inRange(image, lower_red, upper_red) 
        contours, hierarchy = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)     

        for cnt in contours:                                    
            if cnt.shape[0] < 150:
                continue

            (x, y, w, h) = cv2.boundingRect(cnt)                
            cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)  
            centre_x,centre_y = int(x+w/2), int(y+h/2)
            cv2.circle(image, (centre_x, centre_y), 5,(0, 255, 0), -1)                         
            
            message = f"({centre_x},{centre_y})"
            self.publisher.publish(String(data=message))

        cv2.imshow('image', image)                             
        cv2.waitKey(10)

    def listener_callback(self, data):    
        image = self.cv_bridge.compressed_imgmsg_to_cv2(data, 'bgr8')     
        self.object_detect(image)                               


def main(args=None):                                        
    rclpy.init(args=args)                                
    node = ImageSubscriber()             
    rclpy.spin(node)                                        
    node.destroy_node()                                     
    rclpy.shutdown()  

if name=='main':
    main()
