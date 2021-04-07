#!/usr/bin/env python3

import math
import yaml
import numpy as np
import rospy as rp

from geometry_msgs.msg import TwistStamped, PoseArray

from fs_msgs.msg import ControlCommand, FinishedSignal


class AutonomousSteering:
    def __init__(self):
        self.max_throttle = rp.get_param('/vehicle/max_throttle')
        self.target_speed = rp.get_param('/vehicle/target_speed')
        self.max_steering = rp.get_param('/vehicle/max_steering')

        self.control_publisher = rp.Publisher('/fsds/control_command', ControlCommand, queue_size=10)
        self.finished_publisher = rp.Publisher('/fsds/signal/finished', FinishedSignal, queue_size=1)

        self.velocity = 0.0
        self.brake = 1 # float64 0..1
        self.go = False
        self.steering = 0.0

    def start(self, *args):
        self.go = True
        self.brake = 0

    def emergency_brake(self):
        self.publish_finish_signal()

    def publish_finish_signal(self, *args):
        self.brake = 1
        while self.velocity > 0:
            self.publish_control_command(throttle=0, steering=0)

        self.finished_publisher.publish(FinishedSignal())

    def go_autonomous(self, *args):
        raise NotImplementedError("Racecar autonomous steering not implemented")


if __name__ == '__main__':
    rp.init_node('steering', log_level=rp.DEBUG)

    AS = AutonomousSteering()

    rp.Timer(rp.Duration(1), AS.start, oneshot=True)

    rp.Timer(rp.Duration(0.1), AS.go_autonomous)

    rp.Timer(rp.Duration(300), AS.publish_finish_signal, oneshot=True)

    while not rp.is_shutdown():
        rp.spin()