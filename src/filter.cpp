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
#include "segmentation_srv_definitions/segment.h"
//#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/io/pcd_io.h>


#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_box.h>
using namespace std;
using namespace sensor_msgs;
#include <pcl/filters/passthrough.h>


//tf::TransformListener listener;

class SegmentFilter{
  tf::TransformListener listener;

  ros::NodeHandle* node;

  ros::Publisher plane_pub;
  public:
     sensor_msgs::PointCloud2 roi_crop(sensor_msgs::PointCloud2 input_cloud, geometry_msgs::PoseArray posearray);
     sensor_msgs::PointCloud2 ransac_filter(sensor_msgs::PointCloud2 input_cloud);
     bool segment_cb(bham_seg_filter::bham_seg::Request &req,bham_seg_filter::bham_seg::Response &res);
     SegmentFilter();
};

SegmentFilter::SegmentFilter(){
  ROS_INFO("Loaded Filter Class");

  node = new ros::NodeHandle ("~");
  ROS_INFO("letting TF breathe before starting the service");
  ros::Duration(5.0).sleep();

  ROS_INFO("Setting up services");
  ros::ServiceServer conversion_service = node->advertiseService("/bham_filtered_segmentation/segment", &SegmentFilter::segment_cb,this);
  plane_pub =  node->advertise<sensor_msgs::PointCloud2> ("/bham_filtered_segmentation/ransac_plane_estimate", 5);
  ros::Rate r(10);
  ROS_INFO("Done, ready to go");

  ros::spin();
}

sensor_msgs::PointCloud2 SegmentFilter::roi_crop(sensor_msgs::PointCloud2 input_cloud, geometry_msgs::PoseArray posearray) {
  ROS_INFO("Doing ROI Cropping filter");
//  ros::NodeHandle nh;

  pcl::PCLPointCloud2 pcl_pc2;

  pcl_conversions::toPCL(input_cloud,pcl_pc2);
  ROS_INFO("Doing CROP BOX");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  pcl::PointCloud<pcl::PointXYZRGB> xyz_filtered_cloud;
  pcl::CropBox<pcl::PointXYZRGB> crop;
  crop.setInputCloud(temp_cloud);
  ROS_INFO("Almost there");

  std::cout << " MIN X: " << posearray.poses[0].position.x << " MIN Y" << posearray.poses[0].position.y << std::endl;
  std::cout << " MAX X: " << posearray.poses[1].position.x << " MAX Y" << posearray.poses[1].position.y << std::endl;


  Eigen::Vector4f min_point = Eigen::Vector4f(posearray.poses[0].position.x, posearray.poses[0].position.y, 0, 0);
  Eigen::Vector4f max_point = Eigen::Vector4f(posearray.poses[1].position.x, posearray.poses[1].position.y, 10, 0);
  crop.setMin(min_point);
  crop.setMax(max_point);
  crop.setNegative(false);
  crop.setKeepOrganized(true);
  crop.filter(xyz_filtered_cloud);
//  pcl::PCDWriter writer;
//  writer.write<pcl::PointXYZRGB> ("crop.pcd", xyz_filtered_cloud, false);
  ROS_INFO("DONE  CROP BOX");
  sensor_msgs::PointCloud2 output_cloud;
  pcl::toROSMsg(xyz_filtered_cloud,output_cloud);

  return output_cloud;
}

sensor_msgs::PointCloud2 SegmentFilter::ransac_filter(sensor_msgs::PointCloud2 input_cloud)
{
  ROS_INFO("Doing RANSAC filter");
  //pcl::PCDWriter writer;
  pcl::PCLPointCloud2 pcl_pc2;
  //ros::NodeHandle nh;
  pcl_conversions::toPCL(input_cloud,pcl_pc2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr col_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

//  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("2.pcd", *temp_cloud);
//  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("3.pcd", *col_cloud);

//    writer.write<pcl::PointXYZRGB> ("cc.pcd", *col_cloud, false);
//  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
//  sor.setInputCloud (temp_cloud);
//  sor.setLeafSize (0.02f, 0.02f, 0.02f);
//  sor.filter (*temp_cloud);
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
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
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

  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE  );
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.015);
  seg.setInputNormals (cloud_normals);
  seg.setNormalDistanceWeight (0.001);
  seg.setMaxIterations(1000);
  seg.setEpsAngle(pcl::deg2rad(20.0)); // degrees to radians
  seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0));
  seg.setInputCloud (temp_cloud);
  seg.segment (*inliers, *coefficients);

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  if(inliers->indices.size() == 0) {
    ROS_ERROR("No inliers to the plane model found. Quitting!");
    sensor_msgs::PointCloud2 empty_cloud;
    return empty_cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_plane(new pcl::PointCloud<pcl::PointXYZRGB>);

  // do ransac stuff


  // this extracts the plane from the pc
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (temp_cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.setKeepOrganized(true);
  extract.filter(*filtered_cloud);

  float a = coefficients->values[0];
  float b = coefficients->values[1];
  float c = coefficients->values[2];
  float d = coefficients->values[3];


  pcl::PointIndices::Ptr below_plane_indices (new pcl::PointIndices);
  int above_plane = 0;
  int below_plane = 0;
  for (size_t i = 0; i < filtered_cloud->size(); ++i)
  {
    pcl::PointXYZRGB cur_point = filtered_cloud->at(i);

    float co = (a * cur_point.x) + (b * cur_point.y) + (c * cur_point.z) + d;
    if(co > 0.02) {
      above_plane++;
    } else {
      below_plane_indices->indices.push_back(i);
      below_plane++;
    }

  }

  std::cout << "ABOVE PLANE: " << above_plane << std::endl;
  std::cout << "BELOW PLANE: " << below_plane << std::endl;



  pcl::ExtractIndices<pcl::PointXYZRGB> extract_plane;
  extract.setInputCloud (temp_cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter(*filtered_plane);
  sensor_msgs::PointCloud2 pc;
  pcl::toROSMsg (*filtered_plane, pc);
  plane_pub.publish(pc);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract_below_plane;
  extract_below_plane.setInputCloud (filtered_cloud);
  extract_below_plane.setIndices (below_plane_indices);
  extract_below_plane.setNegative (true);
  extract_below_plane.setKeepOrganized(true);
  extract_below_plane.filter(*filtered_cloud);

//  pcl::PCDWriter writer;
//  writer.write<pcl::PointXYZRGB> ("BELOW_PLANE_REMOVED.pcd", *filtered_cloud, false);
//  writer.write<pcl::PointXYZRGB> ("PLANE.pcd", *filtered_plane, false);

//  sensor_msgs::Image image_;
//  sensor_msgs::PointCloud2 pc;
//  writer.write<pcl::PointXYZRGB> ("filtered.pcd", *filtered_cloud, false);
  sensor_msgs::PointCloud2 output_cloud;
  pcl::toROSMsg(*filtered_cloud,output_cloud);


/*
    ros::Publisher image_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/willthiswork", 30);
    ros::Rate r(10); // 10 hz
    int i = 0;
    while(i < 10) {
      i++;
      ROS_INFO("Publishing");
      image_pub_.publish (output_cloud);
      r.sleep();
    }
    */
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

/*
  ros::Publisher image_pub_ = node->advertise<sensor_msgs::PointCloud2> ("/bham_filtered_segmentation/ransac_filtered_cloud", 30);
  ros::Rate r(10); // 10 hz
  int i = 0;
  while(i < 10) {
    i++;
    ROS_INFO("Publishing");
    image_pub_.publish (output_cloud);
    r.sleep();
  }
*/
  return output_cloud;
}



bool SegmentFilter::segment_cb(
bham_seg_filter::bham_seg::Request &req, // phew! what a mouthful!
bham_seg_filter::bham_seg::Response &res)
{
  //ros::NodeHandle nh;
  ROS_INFO("Received service call");

   //sensor_msgs::PointCloud2 pcd = *ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/head_xtion/depth_registered/points", ros::Duration(5));
   //req.cloud = pcd;
   const std::string& target("/map");
   const std::string& root(req.cloud.header.frame_id);
   ros::Time current_transform = ros::Time(0);
   tf::StampedTransform transform;
   pcl::PointCloud<pcl::PointXYZRGB> out_cloud;
   sensor_msgs::PointCloud2 new_out_cloud;
   ROS_INFO("WAITING");
   std::cout<< "TARGET:" << target << std::endl;
   std::cout<< "ORIGIN:" << root << std::endl;
   std::string* err;

   ROS_INFO("Setting up listener");
   ros::Duration(2).sleep();

  bool success = false;

   // some horrible looking stuff from stackoverflow
   int runs = 0;
   while(!success && runs < 30) {
   try {
     runs++;
	ROS_INFO("Waiting for transform topics");
	 listener.waitForTransform(target, root, current_transform, ros::Duration(10.0) );
	ROS_INFO("Looking up transform");
  	 listener.lookupTransform(target, root, current_transform, transform);
	ROS_INFO("Done!");
   success = true;
   } catch (tf::ExtrapolationException e) {
	ROS_INFO("WAITING TO GET TRANSFORM BETWEEN MAP AND CAMERA");
   }
 ros::Duration(0.1).sleep();
}

 ROS_INFO("TRANSFORMING");

   pcl::PCLPointCloud2 pcl_pc2;
   pcl_conversions::toPCL(req.cloud,pcl_pc2);
   pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
   //pcl::PointCloud<pcl::PointXYZRGB>::Ptr col_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::fromPCLPointCloud2(pcl_pc2,temp_cloud);

   //pcl_ros::transformPointCloud (temp_cloud, out_cloud, transform);
  // pcl_ros::transformPointCloud (target, transform, req.cloud, out_cloud);
  Eigen::Vector4f beforecentroid;
  pcl::compute3DCentroid (temp_cloud, beforecentroid);
  std::cout << " CENTROID BEFORE: " << beforecentroid[0] << " : " << beforecentroid[1] << " : " << beforecentroid[2] << " : " << beforecentroid[3] << std::endl;

//  pcl_ros::transformPointCloud (target,temp_cloud,out_cloud, listener);
   pcl_ros::transformPointCloud(temp_cloud,out_cloud,transform);


  Eigen::Vector4f aftercentroid;
  pcl::compute3DCentroid (out_cloud, aftercentroid);
  std::cout << " CENTROID AFTER: " << aftercentroid[0] << " : " << aftercentroid[1] << " : " << aftercentroid[2] << " : " << aftercentroid[3] << std::endl;

  ROS_INFO("done");
  pcl::toROSMsg(out_cloud,new_out_cloud);

  sensor_msgs::PointCloud2 roi_filtered_cloud = roi_crop(new_out_cloud,req.posearray);

  sensor_msgs::PointCloud2 filtered_cloud = ransac_filter(roi_filtered_cloud);

  pcl::PCLPointCloud2 t_pc2;
  pcl_conversions::toPCL(filtered_cloud,t_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(t_pc2,*test_cloud);

  if(test_cloud->points.size() == 0) {
    ROS_ERROR("Unable to find plane, not segmenting any further.");
    return false;
  }

  ros::ServiceClient vienna_seg = node->serviceClient<segmentation_srv_definitions::segment>("/object_gestalt_segmentation");

  segmentation_srv_definitions::segment srv;
  srv.request.cloud = filtered_cloud;
  if (vienna_seg.call(srv)) {
    ROS_INFO("v4r seg called successfully!");
    res.clusters_indices = srv.response.clusters_indices;
  }  else  {
    ROS_ERROR("Failed to call service object_gestalt_segmentation");
  }


  return true;
}

int main (int argc, char** argv)
{

  ros::init (argc, argv, "bham_filtered_segmentation");
  SegmentFilter s;
  return 0;
}
