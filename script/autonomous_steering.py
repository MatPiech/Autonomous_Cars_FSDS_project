#!/usr/bin/env python3

import math
import yaml
import numpy as np
import rospy as rp

from geometry_msgs.msg import PoseArray

from fs_msgs.msg import ControlCommand, FinishedSignal, GoSignal


class AutonomousSteering:
    def __init__(self):
        self.max_throttle = rp.get_param('/vehicle/max_throttle')
        self.target_speed = rp.get_param('/vehicle/target_speed')
        self.max_steering = rp.get_param('/vehicle/max_steering')

        self.control_publisher = rp.Publisher('/fsds/control_command', ControlCommand, queue_size=10)
        self.finished_publisher = rp.Publisher('/fsds/signal/finished', FinishedSignal, queue_size=1)
        self.go_publisher = rp.Publisher('/fsds/signal/go', GoSignal, queue_size=1)

        self.velocity = 0.0
        self.brake = 1 # float64 0..1
        self.go = False


    def start(self, *args):
        gs = GoSignal()
        gs.mission = 'Trackdrive'
        self.go_publisher.publish(gs)
        
        self.go = True
        self.brake = 0


    def publish_control_command(self, throttle, steering):
        if self.go:
            cc = ControlCommand()

            cc.header.stamp = rp.Time.now()
            cc.throttle = throttle
            cc.steering = steering
            cc.brake = self.brake

            self.control_publisher.publish(cc)


    def emergency_brake(self):
        self.publish_finish_signal()


    def publish_finish_signal(self, *args):
        self.brake = 1
        while self.velocity > 0:
            self.publish_control_command(throttle=0, steering=0)

        self.finished_publisher.publish(FinishedSignal())


    def calculate_steering(self):
        raise NotImplementedError("Racecar autonomous steering not implemented")

        return 0


    def calculate_throttle(self):
        raise NotImplementedError("Racecar autonomous throttle not implemented")

        return 0


    def cones_position_callback(self, data):
        # calculate steering using detected cones position
        # use as data callback for cones poses topics

        steering = self.calculate_steering() # TODO: calculate steering value

        throttle = self.calculate_throttle() # TODO: calculate throttle value

        self.publish_control_command(throttle=throttle, steering=steering)


if __name__ == '__main__':
    rp.init_node('steering', log_level=rp.DEBUG)

    AS = AutonomousSteering()
    
    # One can change sleep time if vision system needs more time for loading
    # Also one can move AS.start to AS __init__ as self.start()
    rp.Timer(rp.Duration(1), AS.start, oneshot=True)

    rp.spin()
