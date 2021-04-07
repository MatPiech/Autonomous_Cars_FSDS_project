#!/usr/bin/env python3

import cv2
import yaml 
import numpy as np
import rospy as rp


class ConeDetector:
    def __init__(self):
        self.yolo_weights_file = rp.get_param('/sensors/vision/yolo/yolo_weights_file')
        self.yolo_config_file = rp.get_param('/sensors/vision/yolo/yolo_config_file')
        self.confidence_threshold = rp.get_param('/sensors/vision/yolo/confidence_threshold')
        self.nms_threshold = rp.get_param('/sensors/vision/yolo/nms_threshold')
        self.model_size = rp.get_param('/sensors/vision/yolo/model_size')

        self.net = None

        if self.net is None: 
            raise NotImplementedError("Error loading cone detector model")


    def predict(self, img: np.ndarray) -> np.ndarray:
        raise NotImplementedError("Cone detector predict function not implemented")
