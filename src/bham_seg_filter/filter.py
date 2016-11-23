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
    rospy.Subscriber("/bham_filtered_segmentation/ransac_filtered_cloud", PointCloud2, callback)

    print("loading pcd")
    pcd = python_pcd.read_pcd("0.pcd")
    print("waiting for service")
    rospy.wait_for_service('/bham_filtered_segmentation/segment')
    df = rospy.ServiceProxy('/bham_filtered_segmentation/segment',bham_seg)
    print("done")
    o = df(cloud=pcd[0])
    print("done!")
    print("got this many clusters: " + str(len(o.clusters_indices)))

    int_data = list(pc2.read_points(pcd[0]))
    filtered_clusters = []
    indices = o.clusters_indices
    for cluster in indices:
        print 'cluster has %f points' %len(cluster.data)
        if(len(cluster.data) > 500 and len(cluster.data) < 5000):
            ic = []
            for point in cluster.data:
                ic.append(int_data[point])
            filtered_clusters.append(ic)


    print 'filtered clusters: %d' %len(filtered_clusters)

    for k in filtered_clusters:
        print("writing file")
        cld = pc2.create_cloud(pcd[0].header, pcd[0].fields, k)
        python_pcd.write_pcd(str("segment_"+filtered_clusters.index(k))+".pcd",cld,overwrite=True)

    rospy.spin()
