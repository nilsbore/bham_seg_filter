import pcl
import python_pcd
from sensor_msgs.msg import PointCloud2
import std_msgs
import sensor_msgs.point_cloud2 as pc2
import roslib
import rospy
import math
import numpy as np
import os
import uuid
import python_pcd
from bham_seg_filter.srv import *
from segmentation_srv_definitions.srv import *

class Segmentation():
    def __init__(self):
        rospy.init_node('bham_seg_filter', anonymous = False)
        srv = rospy.Service('/bham_filtered_segmentation',bham_seg,self.segment_cb)
        self.vienna_seg = rospy.ServiceProxy('/object_gestalt_segmentation',segment)


    def segment_cb(self,req):
        rospy.loginfo("received segment request")
        return self.segment(req.cloud)

    def segment(self,point_cloud):
        # preprocess with RANSAC
        rgb_points = []
        col_map = {}
        for p_in in pc2.read_points(point_cloud,field_names=["x","y","z","rgb"]):
            rgb_points.append([p_in[0],p_in[1],p_in[2]])
            col_map[(p_in[0],p_in[1],p_in[2])] = p_in[3]

        rgb_points = np.array(rgb_points,dtype=np.float32)
        working_cloud = pcl.PointCloud()
        working_cloud.from_array(rgb_points)

        # noise filter, clean up?


        # send to vienna segmentation

        #self.seg = vienna_seg(ransac_filtered_cloud)
        # send result back
        #return seg.clusters_indices


if __name__ == '__main__':
    s = Segmentation()

    print("loading pcd")
    pcd = python_pcd.read_pcd("0.pcd")
    s.segment(pcd[0])
