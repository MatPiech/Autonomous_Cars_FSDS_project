#!/usr/bin/env python3

import math
import yaml
import numpy as np
import rospy as rp

from geometry_msgs.msg import TwistStamped, PoseArray

from fs_msgs.msg import ControlCommand, FinishedSignal


class SteeringExample:
    def __init__(self):
        self.max_throttle = rp.get_param('/vehicle/max_throttle')
        self.target_speed = rp.get_param('/vehicle/target_speed')
        self.max_steering = rp.get_param('/vehicle/max_steering')

        rp.Subscriber("/fsds/gss", TwistStamped, self.gss_signal_callback)
        rp.Subscriber("/fsds_utils/cones_position", PoseArray, self.cones_position_callback)

        self.control_publisher = rp.Publisher('/fsds/control_command', ControlCommand, queue_size=10)
        self.finished_publisher = rp.Publisher('/fsds/signal/finished', FinishedSignal, queue_size=1)

        self.steering_time = 0
        self.velocity = 0.0
        self.brake = 1 # float64 0..1
        self.go = False
        self.steering = 0.0

    def start(self, *args):
        self.go = True
        self.brake = 0

    def gss_signal_callback(self, data):
        v_x = data.twist.linear.x
        v_y = data.twist.linear.y

        # Calculate vehicle velocity
        self.velocity = math.sqrt(math.pow(v_x, 2) + math.pow(v_y, 2))

    def cones_position_callback(self, data):
        cones = data.poses      

        self.steering = self.calculate_steering(cones)

    def calculate_steering(self, cones):
        # Calculate steering axis position
        if cones == []:
            self.emergency_brake()

        average_y = sum([cone.position.y for cone in cones]) / len(cones)

        steering = -self.max_steering * average_y

        if steering > self.max_steering:
            steering = self.max_steering
        elif steering < -self.max_steering:
            steering = -self.max_steering

        if self.brake == 1:
            steering = 0
        elif abs(steering) > self.max_steering / 4 * 3:
            self.brake = abs(steering) / 2
        else:
            self.brake = 0 

        return steering

    def calculate_throttle(self):
        # The lower the velocity, the more throttle, up to self.max_throttle
        throttle = self.max_throttle * max(1 - self.velocity / self.target_speed, 0)

        if self.brake == 1:
            throttle = 0

        return throttle

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

    def go_autonomous(self, *args):
        self.publish_control_command(throttle=self.calculate_throttle(), steering=self.steering)

    def go_straight(self, *args):
        self.publish_control_command(throttle=self.calculate_throttle(), steering=0)

    def go_circle(self, *args):
        self.publish_control_command(throttle=self.calculate_throttle(), steering=1)

    def go_sin(self, *args):
        steering_period = 5 # sinus full steering period
        setpoint_frequency = 5 # Hz at what the car setpoints are published

        sin_steering_value = math.sin(self.steering_time * (math.pi / steering_period))
        self.publish_control_command(throttle=self.calculate_throttle(), steering=sin_steering_value)

        self.steering_time += 1.0 / setpoint_frequency


if __name__ == '__main__':
    rp.init_node('steering', log_level=rp.DEBUG)
    steering_mode = rp.get_param('steering_mode')

    SE = SteeringExample()

    rp.Timer(rp.Duration(1), SE.start, oneshot=True)

    # check steering mode according to launch parameter
    if steering_mode == 'autonomous':
        rp.Timer(rp.Duration(0.1), SE.go_autonomous)
    elif steering_mode == 'sin' or steering_mode == 'sinus':
        rp.Timer(rp.Duration(0.1), SE.go_sin)
    elif steering_mode == 'circle':
        rp.Timer(rp.Duration(0.1), SE.go_circle)
    elif steering_mode == 'straight':
        rp.Timer(rp.Duration(0.1), SE.go_straight)
    else:
        raise NotImplementedError('Steering mode not recognized, available options: {autonomous, sinus, circle, straight}')

    rp.Timer(rp.Duration(240), SE.publish_finish_signal, oneshot=True)

    while not rp.is_shutdown():
        rp.spin()