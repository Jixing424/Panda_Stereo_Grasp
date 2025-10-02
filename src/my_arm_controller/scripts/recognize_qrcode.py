#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class QRDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.window_name = "QR Detector"

    def image_callback(self, msg):
            img_rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            cropped_img = img_rgb[350:550, :]

            qrdecoder = cv2.QRCodeDetector()
            data, bbox, _ = qrdecoder.detectAndDecode(img_rgb)
            
            if data:
                print(f"二维码内容: {data}")
            else:
                print("未检测到二维码")

            if bbox is not None:
                bbox = bbox.astype(int)
                cv2.polylines(img_rgb, [bbox], True, (0, 255, 0), 2)
            cv2.imshow("QR Code Detection", img_rgb)
            key = cv2.waitKey(1)
            if key == ord('q'):
                cv2.destroyAllWindows()
                
                

def main():
    rospy.init_node("qr_detector_node")
    detector = QRDetector()
    rospy.Subscriber("/right_camera/image_raw", Image, detector.image_callback, queue_size=1)
    rospy.loginfo("QR detector node started.")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
