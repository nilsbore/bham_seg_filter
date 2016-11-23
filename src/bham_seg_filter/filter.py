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

def callback(msg):
    print("got msg")
    python_pcd.write_pcd("s.pcd",msg,overwrite=True)

if __name__ == '__main__':
    rospy.init_node('CT_TEST_NODE', anonymous = True)
    rospy.Subscriber("/willthiswork", PointCloud2, callback)

    print("loading pcd")
    pcd = python_pcd.read_pcd("0.pcd")
    print("waiting for service")
    rospy.wait_for_service('/bham_filtered_segmentation/segment')
    df = rospy.ServiceProxy('/bham_filtered_segmentation/segment',bham_seg)
    print("done")
    df(cloud=pcd[0])
    print("done!")



    rospy.spin()
