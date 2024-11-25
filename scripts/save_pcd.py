#!/usr/bin/env python3
# 自动定时保存ndt_mapping地图到pcd文件
#   python3 save_pcd.py ${topic_name} ${save_path}
#   python3 ~/catkin_ndtg/src/ndtg_mapping/scripts/save_pcd.py /ndtg/map ~/autoware_shared_dir/bag/map-6-park/pcd
#       pcl_viewer ~/autoware_shared_dir/bag/map-6-park/pcd/pcd_1732540520.51428.pcd

import sys
import os
import time
import rospy
import numpy as np
import cv2
import pcl
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge

class SavePcd:
    def __init__(self):
        self.topic = "/ndtg/map"
        self.save_path = None
        self.filter_resolution = 2.0
        self.save_count = 0

    def save_pcd(self, msg):
        filename = self.save_path + '/pcd' + '_' + "{:.5f}".format(time.time()) + '.pcd'
        print("save_pcd start ", filename)

        cloud = pcl.PointCloud(np.array(list(pc2.read_points(msg)), dtype=np.float32)[:, 0:3])
        if self.filter_resolution > 0.0:
            sor = cloud.make_voxel_grid_filter()
            sor.set_leaf_size(self.filter_resolution, self.filter_resolution, self.filter_resolution)
            cloud_filtered = sor.filter()
            cloud_filtered.to_file(filename.encode())
        else:
            cloud.to_file(filename.encode())

        self.save_count += 1
        print("save_pcd done!")

    def run(self):
        try:
            self.topic = sys.argv[1]
            self.save_path = sys.argv[2]
        except Exception as e:
            sys.exit("Example: save_pcd.py topic_name save_path")

        rospy.init_node("save_pcd_node", anonymous=True)
        rospy.Subscriber(self.topic, PointCloud2, self.save_pcd)
        # rospy.spin()
        while True:
            if self.save_count > 0:
                break

if __name__ == '__main__':
    pcd = SavePcd()
    pcd.run()