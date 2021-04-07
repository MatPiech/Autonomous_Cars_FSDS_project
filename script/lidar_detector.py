#!/usr/bin/env python3

import math
import yaml
import numpy as np
import rospy as rp

from ros_numpy import point_cloud2
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import PointCloud2


class LidarDetector:
    def __init__(self):
        self.cones_range_cutoff = rp.get_param('/sensors/cones_range_cutoff')

        rp.Subscriber("/fsds/lidar/Lidar", PointCloud2, self.lidar_detection_callback)

        self.cones_position_publisher = rp.Publisher('/fsds_utils/cones_position', PoseArray, queue_size=10)

    def lidar_detection_callback(self, data):
        pose_array = PoseArray()

        # return array of shape (width, 3) with X, Y, Z coordinates 
        points = point_cloud2.pointcloud2_to_xyz_array(data)

        pose_array.header.stamp = rp.Time.now()
        pose_array.header.frame_id = 'fsds_utils/cones_poses'
        pose_array.poses = self.find_cones(points)

        self.cones_position_publisher.publish(pose_array)

    def points_group_to_cone(self, points_group):
        pose = Pose()

        cone_pose = np.sum(points_group, axis=0) / len(points_group)

        pose.position.x = cone_pose[0]
        pose.position.y = cone_pose[1]
        pose.position.z = cone_pose[2] + 0.3

        return pose

    def distance(self, x1, y1, x2, y2):
        return math.sqrt(math.pow(x1-x2, 2) + math.pow(y1-y2, 2))

    def find_cones(self, points):
        current_group = []
        cones = []
        for i in range(1, len(points)):
            # Get the distance from current to previous point
            distance_to_last_point = self.distance(points[i][0], points[i][1], points[i-1][0], points[i-1][1])

            if distance_to_last_point < 0.1:
                # Points closer together then 10 cm are part of the same group
                current_group.append(points[i])
            else:
                # points further away indiate a split between groups
                if len(current_group) > 0:
                    cone = self.points_group_to_cone(current_group)
                    # calculate distance between lidar and cone
                    if self.distance(0, 0, cone.position.x, cone.position.y) < self.cones_range_cutoff:
                        cones.append(cone)
                    current_group = []

        return cones

if __name__ == '__main__':
    rp.init_node('lidar', log_level=rp.DEBUG)

    LD = LidarDetector()

    while not rp.is_shutdown():
        rp.spin()