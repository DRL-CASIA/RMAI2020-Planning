#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_sub");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr coming_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sub_in (new pcl::PointCloud<pcl::PointXYZ>);
  ROS_INFO("Hello world!");

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/drl/bagfiles/pointclouds/map_pointcloud.pcd", *map_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file map_pointcloud.pcd \n");
      return (-1);
    }
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/drl/bagfiles/pointclouds/laser_0.pcd", *coming_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file laser.pcd \n");
      return (-1);
    }
  std::cout<<"map cloud size: "<<map_cloud->size()<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(map_cloud);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*downsample_map);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setTransformationEpsilon(1e-8);
  icp.setInputSource(coming_cloud);
  icp.setInputTarget(map_cloud);
  std::cout<<"down sampling map cloud size: "<<downsample_map->size()<<std::endl;
  pcl::io::savePCDFileASCII ("/home/drl/bagfiles/pointclouds/down_sample_map_pointcloud.pcd", *downsample_map);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  *sub_in = Final;
  pcl::SegmentDifferences<pcl::PointXYZ> seg_sub;
  seg_sub.setInputCloud(sub_in);
  seg_sub.setTargetCloud(map_cloud);
  seg_sub.setDistanceThreshold(0.1);
  pcl::PointCloud<pcl::PointXYZ> sub_pc;
  seg_sub.segment(sub_pc);
//  *coming_cloud += *map_cloud;
  pcl::io::savePCDFileASCII ("/home/drl/bagfiles/pointclouds/sub_pc.pcd", sub_pc);

}
