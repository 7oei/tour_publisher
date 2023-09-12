#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse  
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

def load_pcd(filename):
    pcd = o3d.io.read_point_cloud(filename)
    normals = np.asarray(pcd.normals)
    return np.asarray(pcd.points), normals if normals.size > 0 else None

def create_pointcloud2_message(points, normals):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
    ]

    if normals is not None:
        fields.extend([
            PointField(name='normal_x', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='normal_y', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='normal_z', offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='curvature', offset=28, datatype=PointField.FLOAT32, count=1),
        ])

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map' 

    rgb_values = np.full((len(points), 1), 255<<16 | 255<<8 | 255, dtype=np.uint32)  # rgb

    if normals is not None:
        zero_padding = np.zeros((len(points), 1))  # curvature
        data = np.hstack((points, rgb_values, normals, zero_padding))
    else:
        data = np.hstack((points, rgb_values))

    pc2_msg = pc2.create_cloud(header, fields, data)

    return pc2_msg

def main(args):
    rospy.init_node('pcd_to_path_publisher', anonymous=True)

    path_pub = rospy.Publisher('pcd_path', Path, queue_size=10)
    pc2_pub = rospy.Publisher('pcd_pointcloud', PointCloud2, queue_size=10) 

    rate = rospy.Rate(1)  # 1Hz

    pcd_filename = args.filename  
    points, normals = load_pcd(pcd_filename) 

    path = Path()
    path.header.frame_id = 'map'

    for point in points:
        pose = PoseStamped()
        pose.header.frame_id = path.header.frame_id
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        path.poses.append(pose)

    pc2_msg = create_pointcloud2_message(points, normals)

    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        path_pub.publish(path)
        pc2_pub.publish(pc2_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()  
        parser.add_argument('filename', help='PCD file to load')
        parser.add_argument('rosargs', nargs=argparse.REMAINDER)  # Catch-all for ROS arguments
        args = parser.parse_args()  

        main(args)
    except rospy.ROSInterruptException:
        pass

