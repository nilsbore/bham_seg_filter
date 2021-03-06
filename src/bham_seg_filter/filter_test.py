#!/usr/bin/env python
import roslib
import rospy

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
from geometry_msgs.msg import *
from surface_based_object_learning.srv import *


def callback(msg):
    print("got msg")
    python_pcd.write_pcd("s.pcd",msg,overwrite=True)

if __name__ == '__main__':
    rospy.init_node('CT_TEST_NODE', anonymous = True)
    rospy.Subscriber("/bham_filtered_segmentation/ransac_filtered_cloud", PointCloud2, callback)

    print("loading pcd")
    #pcd = python_pcd.read_pcd("2.pcd")

    rospy.wait_for_service('/get_closest_roi_to_robot')
    roicl = rospy.ServiceProxy('/get_closest_roi_to_robot',GetROIClosestToRobot)

    rp = rospy.wait_for_message("/robot_pose",Pose,10)
    roip = roicl(pose=rp.position)

    print("waiting for service")
    rospy.wait_for_service('/bham_filtered_segmentation/segment')
    df = rospy.ServiceProxy('/bham_filtered_segmentation/segment',bham_seg)
    print("done")

    pcd = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)
    print(pcd.header)


    o = df(cloud=pcd,posearray=roip.output)
    print("done!")
    print("got this many clusters: " + str(len(o.clusters_indices)))

    int_data = list(pc2.read_points(pcd))
    filtered_clusters = []
    indices = o.clusters_indices
    for cluster in indices:
        print 'cluster has %f points' %len(cluster.data)
        if(len(cluster.data) > 0 and len(cluster.data) < 10000):
            ic = []
            for point in cluster.data:
                ic.append(int_data[point])
            filtered_clusters.append(ic)


    print 'filtered clusters: %d' %len(filtered_clusters)

    for k in filtered_clusters:
        print("writing file")
        x = 0
        y = 0
        z = 0
        for p in k:
            x+=p[0]
            y+=p[1]
            z+=p[2]
        x/=len(k)
        y/=len(k)
        z/=len(k)
        print("centroid:")
        print(x)
        print(y)
        print(z)
        cld = pc2.create_cloud(pcd.header, pcd.fields, k)
        python_pcd.write_pcd(str("segment_"+str(filtered_clusters.index(k)))+".pcd",cld,overwrite=True)

    rospy.spin()
