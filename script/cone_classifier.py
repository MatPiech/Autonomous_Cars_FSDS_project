#!/usr/bin/env python3

import cv2
import numpy as np
import rospy as rp
import tensorflow as tf


class ConeClassifier:
    def __init__(self):
        self.model = None

        if self.model is None: 
            raise NotImplementedError("Error loading cone classifier model")


    def predict(self, rois: np.ndarray) -> np.ndarray:
        raise NotImplementedError("Cone classifier predict function not implemented")
