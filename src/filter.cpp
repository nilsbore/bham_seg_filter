#include <cstdlib>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/Image.h>
#include "bham_seg_filter/bham_seg.h"
#include <pcl/io/pcd_io.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/range_image/range_image.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
using namespace std;
using namespace sensor_msgs;
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZ PointT;

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;
#define ANGLE_MAX_DIFF (M_PI / 4)

bool enforceNormals (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  Eigen::Vector3f point_a_normal = point_a.getVector3fMap();
  Eigen::Vector3f point_b_normal = point_b.getVector3fMap();
  float angle = point_a_normal.dot(point_b_normal);
  std::cout << angle << std::endl;
  return true;


  if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  if (fabs (point_a_normal.dot (point_b_normal)) < 0.05)
    return (true);
  return (false);
}


sensor_msgs::PointCloud2 ransac_filter(sensor_msgs::PointCloud2 input_cloud)
{
  ROS_INFO("Doing RANSAC filter");
  pcl::PCDWriter writer;
ros::NodeHandle nh_;
  pcl::PCLPointCloud2 pcl_pc2;

  pcl_conversions::toPCL(input_cloud,pcl_pc2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr col_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  pcl::io::loadPCDFile<pcl::PointXYZ> ("2.pcd", *temp_cloud);

//  pcl::VoxelGrid<pcl::PointXYZ> sor;
//  sor.setInputCloud (temp_cloud);
//  sor.setLeafSize (0.1f, 0.1f, 0.1f);
//  sor.filter (*ffcloud);
//  writer.write<pcl::PointXYZ> ("gridfil.pcd", *ffcloud, false);
//temp_cloud = ffcloud;

//  ROS_INFO("Filtering away everything further than 2m");
//  pcl::PassThrough<pcl::PointXYZ> pass;
//  pass.setInputCloud (temp_cloud);
//  pass.setFilterFieldName ("z");
//  pass.setFilterLimits (0.0, 3.0);
//  pass.filter (*filcld);
//  writer.write<pcl::PointXYZ> ("pass.pcd", *filcld, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setSearchMethod (tree);
  ne.setInputCloud (temp_cloud);
  ne.setKSearch (75);
  ne.compute (*cloud_normals);
/*
  cout << cloud_normals->points.size() << std::endl;
  cout << temp_cloud->points.size() << std::endl;
  Eigen::Vector3f up = Eigen::Vector3f(0.0,0.0,1.0);
  pcl::PointIndices::Ptr up_facing_points (new pcl::PointIndices ());

  for (size_t i = 0; i < cloud_normals->points.size(); ++i)
      {
        Eigen::Vector3f n = Eigen::Vector3f(cloud_normals->points[i].normal[0],cloud_normals->points[i].normal[1],cloud_normals->points[i].normal[2]);
        float normal = n.dot(up);
        if(normal <= ANGLE_MAX_DIFF){
          up_facing_points->indices.push_back(i);
        }
      }


    pcl::PointCloud<pcl::PointXYZ>::Ptr up_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ExtractIndices<pcl::PointXYZ> up_extract;
    up_extract.setInputCloud (temp_cloud);
    up_extract.setIndices (up_facing_points);
    up_extract.setNegative (true);
    up_extract.filter (*up_cloud);


    writer.write<pcl::PointXYZ> ("up.pcd", *up_cloud, false);
*/

  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE );
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.018);
  seg.setInputNormals (cloud_normals);
  seg.setNormalDistanceWeight (0.001);
  seg.setMaxIterations(1000);
//  seg.setEpsAngle(pcl::deg2rad(3.0)); // degrees to radians
//  seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0));
  seg.setInputCloud (temp_cloud);
  seg.segment (*inliers, *coefficients);

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // do ransac stuff
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (temp_cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*filtered_cloud);

  sensor_msgs::Image image_;
  sensor_msgs::PointCloud2 pc;
  writer.write<pcl::PointXYZ> ("filtered.pcd", *filtered_cloud, false);
/*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_col_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract2;
  extract2.setInputCloud (col_cloud);
  extract2.setIndices (inliers);
  extract2.setNegative (true);
  extract2.filter (*filtered_col_cloud);

  pcl::toROSMsg (*filtered_col_cloud, pc);
  pcl::toROSMsg (pc, image_);

  ros::Publisher image_pub_ = nh_.advertise<sensor_msgs::Image> ("/willthiswork", 30);
  ros::Rate r(10); // 10 hz
  int i = 0;
  while(i < 10) {
    i++;
    ROS_INFO("Publishing");
    image_pub_.publish (image_);
    r.sleep();
  }
*/

  return input_cloud;
}

bool segment_cb(
bham_seg_filter::bham_seg::Request &req, // phew! what a mouthful!
bham_seg_filter::bham_seg::Response &res)
{
  ROS_INFO("Received service call");
  sensor_msgs::PointCloud2 filtered_cloud = ransac_filter(req.cloud);



  return true;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "bham_filtered_segmentation");
  //ros::NodeHandle node;

  ROS_INFO("Setting up services");
  //ros::ServiceServer conversion_service = node.advertiseService("/bham_filtered_segmentation/segment", segment_cb);
  ROS_INFO("Done, ready to go");

  sensor_msgs::PointCloud2 input_cloud;
  ransac_filter(input_cloud);

  ROS_INFO("Spinning");
  //ros::spin();
  return 0;
}
