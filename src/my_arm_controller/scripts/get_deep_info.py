#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pyzbar import pyzbar
import tf2_ros
import tf.transformations as transformations
from geometry_msgs.msg import PoseStamped, TransformStamped
from my_arm_controller.msg import QRCode


class QRDetector():
    def __init__(self):
        self.bridge = CvBridge()
        
        self.left_img = None
        self.right_img = None
        self.left_qrcenter = {}
        self.right_qrcenter = {}
        self.disparity_dict = {}
        self.frame_id = 'camera_base'
        
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.F = 1108.4
        self.B = 0.2
        self.CENTER_X = 640
        self.CENTER_Y = 360
        
        self.pub = rospy.Publisher("/qr_pose", QRCode, queue_size=10)
        
        rospy.Subscriber("/left_camera/image_raw", Image, self.left_img_callback)
        rospy.Subscriber("/right_camera/image_raw", Image, self.right_img_callback)
        
    def left_img_callback(self, left_msg):
        self.left_img = self.bridge.imgmsg_to_cv2(left_msg, 'rgb8')
        
    def right_img_callback(self, right_msg):
        self.right_img = self.bridge.imgmsg_to_cv2(right_msg, 'rgb8')
        
    def cal_depth(self, delta_pixel):
        if delta_pixel == 0:
            z = float('inf')
        else:
            z = self.F * self.B / delta_pixel
        return z
    
    def pub_qr_position(self, qr_id, x, y, z):
        msg = QRCode()
        msg.id = qr_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        
        self.pub.publish(msg)
        
    def get_camera_pose(self):
        trans = self.tf_buffer.lookup_transform('world', self.frame_id, rospy.Time(0))
        
        try:
            t = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])
            
            q = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,
            ]
            
            r = np.array(transformations.quaternion_matrix(q)[:3, :3])
            return t, r
        except Exception as e:
            rospy.logerr(e)
            return None, None
    
    def get_qr_world_pose(self):
        if self.left_img is None and self.right_img is None:
            return
        
        left_qr_msgs = pyzbar.decode(self.left_img)
        right_qr_msgs = pyzbar.decode(self.right_img)
        
        for left_qr_msg in left_qr_msgs:
            left_qr_tag = left_qr_msg.data.decode('utf-8')
            x, y, w, h = left_qr_msg.rect
            self.left_qrcenter[left_qr_tag] = (x + 0.5 * w, y + 0.5 * h)
                     
        for right_qr_msg in right_qr_msgs:
            right_qr_tag = right_qr_msg.data.decode('utf-8')
            x, y, w, h = right_qr_msg.rect
            self.right_qrcenter[right_qr_tag] = (x + 0.5 * w, y + 0.5 * h)
            
        for key in self.left_qrcenter.keys() & self.right_qrcenter.keys():
            delta_pixel = abs(self.left_qrcenter[key][0] - self.right_qrcenter[key][0])
            z = self.cal_depth(delta_pixel)
            x = (self.left_qrcenter[key][0] - self.CENTER_X) * z / self.F
            y = (self.left_qrcenter[key][1] - self.CENTER_Y) * z / self.F
            
            qr_camera_position = np.array([z, -x, -y])
            t, r = self.get_camera_pose()
            if t is not None and r is not None:
                qr_world_position = r.dot(qr_camera_position) + t
                self.pub_qr_position(key, qr_world_position[0], qr_world_position[1], qr_world_position[2])
            
            
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.get_qr_world_pose()
            rate.sleep()
            
            

if __name__ == "__main__":
    rospy.init_node('get_deep_information')

    qrdetecor = QRDetector()
    qrdetecor.run()

