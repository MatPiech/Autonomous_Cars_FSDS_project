#!/usr/bin/env python3

import cv2
import yaml
import rospy as rp
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose

from cone_detector import ConeDetector
from cone_classifier import ConeClassifier


class ImageInference:
    def __init__(self):
        self.detector = ConeDetector()
        self.classifier = ConeClassifier()

        rp.Subscriber("/fsds/camera/cam1", Image, self.right_camera_image_callback) # right camera
        rp.Subscriber("/fsds/camera/cam2", Image, self.left_camera_image_callback) # left camera

        self.inferenced_img_pub = rp.Publisher("/fsds_utils/camera/inferenced_image", Image, queue_size=100)

        self.colors = [(255,0,0), (0,255,255), (0,0,255)] # [blue, yellow, red] in BGR format


    def right_camera_image_callback(self, data):
        self.right_img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)


    def left_camera_image_callback(self, data):
        left_img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)


    def inference(self, left_img: np.ndarray, right_img: np.ndarray):
        raise NotImplementedError("Image inference function not implemented")
        
        #self.publish_inferenced_img(left_img, boxes, cone_colors)


    def publish_inferenced_img(self, img: np.ndarray, boxes: np.ndarray, cone_colors: np.ndarray):
        for box, cone_color in zip(boxes, cone_colors):
            x, y, w, h = box.astype(int)
            cv2.rectangle(img, (x,y), (x+w, y+h), self.colors[cone_color], 2)

        img_msg = Image()
        img_msg.header.stamp = rp.Time.now()
        img_msg.header.frame_id = "/fsds_utils/camera/inferenced_image"
        img_msg.height = img.shape[0]
        img_msg.width = img.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = img.flatten().tostring()
        img_msg.step = len(img_msg.data) // img_msg.height

        self.inferenced_img_pub.publish(img_msg)


if __name__ == '__main__':
    rp.init_node('image_inference', log_level=rp.DEBUG)

    img_infer = ImageInference()

    while not rp.is_shutdown():
        rp.spin()