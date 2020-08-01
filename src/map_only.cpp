// map_only.cpp
// 200731
// mapping based on the ground-truth trajectory

#include <algorithm>
#include <cmath>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>

// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265

#include "rbpf.h"

std::string odom_topic;
std::string odom_reset_topic;
std::string pc_topic;
std::string world_frame_id;
std::string robot_frame_id;

sensor_msgs::PointCloud2 pc_buff;
std::mutex pc_mutex;

std::vector<std::string> range_topics;
std::map<std::string, tf::Pose> range_poses;
std::map<std::string, double> range_buff;
std::mutex range_mutex;
double range_max, range_min;

////////////////////////////////////////////////////////////////////////////////
void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(pc_mutex);
  pc_buff = *msg;
}

////////////////////////////////////////////////////////////////////////////////
void pf_main()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tf::TransformBroadcaster tf_broadcaster;

  std::string octomap_topic;
  pnh.getParam("octomap_topic", octomap_topic);
  ros::Publisher map_pub = nh.advertise<octomap_msgs::Octomap>(octomap_topic, 1);
  ros::Publisher marker_occupied_pub
    = nh.advertise<visualization_msgs::MarkerArray>("map_marker_occupied", 1);
  ros::Publisher vis_poses_pub
    = nh.advertise<geometry_msgs::PoseArray>("loc_vis_poses", 1, true);

  pnh.getParam("world_frame_id", world_frame_id);
  pnh.getParam("robot_frame_id", robot_frame_id);

  double update_freq;
  pnh.getParam("update_freq", update_freq);
  const ros::Duration update_phase(1.0/update_freq);

  int depth_cam_pc_downsample;
  pnh.getParam("depth_cam_pc_downsample", depth_cam_pc_downsample);

  double init_x;
  double init_y;
  double init_z;
  double init_Y;
  double resol;
  double probHit;
  double probMiss;
  double threshMin;
  double threshMax;
  pnh.getParam("init_x", init_x);
  pnh.getParam("init_y", init_y);
  pnh.getParam("init_z", init_z);
  pnh.getParam("init_Y", init_Y);
  pnh.getParam("map_resol", resol);
  pnh.getParam("map_probHit", probHit);
  pnh.getParam("map_probMiss", probMiss);
  pnh.getParam("map_threshMin", threshMin);
  pnh.getParam("map_threshMax", threshMax);

  int mapping_interval;
  pnh.getParam("mapping_interval", mapping_interval);
  int publish_interval;
  pnh.getParam("publish_interval", publish_interval);
  int vismap_interval;
  pnh.getParam("vismap_interval", vismap_interval);
  int compress_interval;
  pnh.getParam("compress_interval", compress_interval);

  const ros::Time initial_update = ros::Time::now();
  ros::Time last_update = initial_update;

  PointCloudT::Ptr depth_cam_pc(new PointCloudT());
  tf::Quaternion rotation;
  rotation.setRPY(-PI/2.0, 0, -PI/2.0);
  //const tf::Pose camera_pose(rotation, tf::Vector3(0, 0, 0));
  const tf::Pose camera_pose(rotation, tf::Vector3(0.03, 0, -0.06));

  int counts_publish = 0;
  int counts_visualize_map = 0;
  int counts_map_update = 0;
  int counts_compress = 0;

  // TODO: create octomap object
  //

  // TODO: get the current pose
  //

  // the main loop
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    if (now > last_update + update_phase)
    {
      // initialize the time step
      last_update = now;

      octomap::Pointcloud octocloud;
      {
        // Depthcam data
        std::lock_guard<std::mutex> lk(pc_mutex);
        if (pc_buff.height * pc_buff.width > 1)
        {
          pcl::fromROSMsg(pc_buff, *depth_cam_pc);

          pcl::VoxelGrid<PointT> downSizeFilter;
          downSizeFilter.setLeafSize(resol, resol, resol);
          downSizeFilter.setInputCloud(depth_cam_pc);
          downSizeFilter.filter(*depth_cam_pc);
          for (unsigned int i = 0; i < depth_cam_pc->points.size(); ++i)
          {
            // if (pcl_isfinite(depth_cam_pc->points[i].x) &&
            //     pcl_isfinite(depth_cam_pc->points[i].y) &&
            //     pcl_isfinite(depth_cam_pc->points[i].z))
            // {
              tf::Vector3 point = camera_pose * tf::Vector3(
                                          depth_cam_pc->points[i].x,
                                          depth_cam_pc->points[i].y,
                                          depth_cam_pc->points[i].z);
              octocloud.push_back(
                octomap::point3d(point.x(), point.y(), point.z()));
            // }
          }
          ROS_DEBUG_STREAM(
            "depth_cam_pc(" << depth_cam_pc->points.size() << ") => " <<
            "octocloud(" << octocloud.size() << ")");
          pc_buff.height = 0;
          pc_buff.width = 0;
        }
      }

      if (octocloud.size() > 0)
      {
        // TODO
        // update map
      }

      // compress maps
      if (counts_compress >= compress_interval)
      {
        // TODO compress
        counts_compress = 0;
      }
      else
      {
        ++counts_compress;
      }

      // publish data
      if (counts_publish >= publish_interval)
      {
        ROS_DEBUG("publish map");
        octomap_msgs::Octomap map;
        map.header.frame_id = world_frame_id;
        map.header.stamp = now;
        // TODO publish 
        // if (octomap_msgs::fullMapToMsg(*particles[index_best]->getMap(), map))
        //   map_pub.publish(map);
        // else
        //   ROS_ERROR("Error serializing OctoMap");
        counts_publish = 0;
      }
      else
      {
        ++counts_publish;
      }

      // visualization
      if (counts_visualize_map >= vismap_interval)
      {
        ROS_DEBUG("visualize map");
        const octomap::OcTree* m;// = particles[index_best]->getMap();
        visualization_msgs::MarkerArray occupiedNodesVis;
        occupiedNodesVis.markers.resize(m->getTreeDepth()+1);
        for (
          octomap::OcTree::iterator it = m->begin(m->getTreeDepth()),
          end = m->end(); it != end; ++it)
        {
          if (m->isNodeAtThreshold(*it))
          {
            double x = it.getX();
            double z = it.getZ();
            double y = it.getY();

            unsigned idx = it.getDepth();
            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            if (m->isNodeOccupied(*it))
            {
              occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

              double cosR = std::cos(PI*z/10.0)*0.8+0.2;
              double cosG = std::cos(PI*(2.0/3.0+z/10.0))*0.8+0.2;
              double cosB = std::cos(PI*(4.0/3.0+z/10.0))*0.8+0.2;
              std_msgs::ColorRGBA clr;
              clr.r = (cosR > 0)? cosR: 0;
              clr.g = (cosG > 0)? cosG: 0;
              clr.b = (cosB > 0)? cosB: 0;
              clr.a = 0.5;
              occupiedNodesVis.markers[idx].colors.push_back(clr);
            }
          }
        }
        // std_msgs::ColorRGBA m_color_occupied;
        // m_color_occupied.r = 1;
        // m_color_occupied.g = 1;
        // m_color_occupied.b = 0.3;
        // m_color_occupied.a = 0.5;
        for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
        {
          double size = m->getNodeSize(i);

          occupiedNodesVis.markers[i].header.frame_id = "world";
          occupiedNodesVis.markers[i].header.stamp = now;
          occupiedNodesVis.markers[i].ns = "iris";
          occupiedNodesVis.markers[i].id = i;
          occupiedNodesVis.markers[i].type
            = visualization_msgs::Marker::CUBE_LIST;
          occupiedNodesVis.markers[i].scale.x = size;
          occupiedNodesVis.markers[i].scale.y = size;
          occupiedNodesVis.markers[i].scale.z = size;

          // without this line, rviz complains orientation is uninitialized.
          occupiedNodesVis.markers[i].pose.orientation.w = 1;

          //occupiedNodesVis.markers[i].color = m_color_occupied;

          if (occupiedNodesVis.markers[i].points.size() > 0)
            occupiedNodesVis.markers[i].action
              = visualization_msgs::Marker::ADD;
          else
            occupiedNodesVis.markers[i].action
              = visualization_msgs::Marker::DELETE;
        }
        marker_occupied_pub.publish(occupiedNodesVis);
        counts_visualize_map = 0;
      }
      else
      {
        ++counts_visualize_map;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rbpf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("pc_topic", pc_topic);
  ros::Subscriber pc_sub = nh.subscribe(pc_topic, 1000, pcCallback);

  std::thread t(pf_main);

  ros::spin();

  t.join();

  return(0);
}
