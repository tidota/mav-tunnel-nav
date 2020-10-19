// rbpf.cpp
// 191028
// Rao-Blackwellized Particle Filter

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
#include <nav_msgs/Odometry.h>

#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mav_tunnel_nav/SrcDstMsg.h>
#include <mav_tunnel_nav/Particles.h>

#include <ros/ros.h>

// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>

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

nav_msgs::Odometry odom_buff;
std::mutex odom_mutex;

sensor_msgs::PointCloud2 pc_buff;
std::mutex pc_mutex;

std::vector<std::string> range_topics;
std::map<std::string, tf::Pose> range_poses;
std::map<std::string, double> range_buff;
std::mutex range_mutex;
double range_max, range_min;

std::mutex beacon_mutex;
std::mutex sync_mutex;
std::mutex data_mutex;

////////////////////////////////////////////////////////////////////////////////
void beaconCallback(const mav_tunnel_nav::SrcDstMsg::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(beacon_mutex);

}

////////////////////////////////////////////////////////////////////////////////
void syncCallback(const mav_tunnel_nav::SrcDstMsg::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(sync_mutex);

}

////////////////////////////////////////////////////////////////////////////////
void dataCallback(const mav_tunnel_nav::Particles::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(data_mutex);

}

////////////////////////////////////////////////////////////////////////////////
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // update odom_pose by the odometry data.
  std::lock_guard<std::mutex> lk(odom_mutex);
  odom_buff = *msg;
}

////////////////////////////////////////////////////////////////////////////////
void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(pc_mutex);
  pc_buff = *msg;
}

////////////////////////////////////////////////////////////////////////////////
void rangeCallback(const sensor_msgs::Range::ConstPtr& new_range)
{
  std::lock_guard<std::mutex> lk(range_mutex);
  // frame_id looks like "ray_xxxx_link". we need "xxxx" part.
  int len = new_range->header.frame_id.length();
  int pos1 = 0;
  while (pos1 < len && new_range->header.frame_id[pos1] != '_'){ ++pos1; }
  ++pos1;
  int pos2 = pos1;
  while (pos2 < len && new_range->header.frame_id[pos2] != '_'){ ++pos2; }

  // then need "range_xxxx" so "range_" is appended
  range_buff["range_" + new_range->header.frame_id.substr(pos1, pos2 - pos1)]
    = new_range->range;

  // ROS_DEBUG_STREAM(
  //   "range " << new_range->header.frame_id
  //            << "("
  //            << new_range->header.frame_id.substr(pos1, pos2 - pos1)
  //            << ")"
  //            << " = " << new_range->range);
}

////////////////////////////////////////////////////////////////////////////////
Particle::Particle(
  const double &init_x, const double &init_y, const double &init_z,
  const double &init_Y, const double &resol,
  const double &probHit, const double &probMiss,
  const double &threshMin, const double &threshMax)
{
  this->map = new octomap::OcTree(resol);
  this->map->setProbHit(probHit);
  this->map->setProbMiss(probMiss);
  this->map->setClampingThresMin(threshMin);
  this->map->setClampingThresMax(threshMax);

  tf::Quaternion rot_buff;
  rot_buff.setRPY(0, 0, init_Y);
  this->pose = tf::Transform(rot_buff, tf::Vector3(init_x, init_y, init_z));
}

////////////////////////////////////////////////////////////////////////////////
Particle::Particle(const Particle &src)
{
  // copy the localization data
  this->pose = src.pose;
//  this->vel_linear = src.vel_linear;

  // copy the mapping data
  this->map = new octomap::OcTree(*src.map);
}

////////////////////////////////////////////////////////////////////////////////
Particle::Particle(): Particle(0.0, 0.0, 0.0, 0.0, 0.25, 0.7, 0.4, 0.12, 0.97){}

////////////////////////////////////////////////////////////////////////////////
Particle::~Particle()
{
  delete this->map;
}

////////////////////////////////////////////////////////////////////////////////
const tf::Pose Particle::getPose()
{
  return this->pose;
}

////////////////////////////////////////////////////////////////////////////////
// const tf::Vector3 Particle::getVel()
// {
//   return this->vel_linear;
// }
////////////////////////////////////////////////////////////////////////////////
const octomap::OcTree* Particle::getMap()
{
  return this->map;
}

////////////////////////////////////////////////////////////////////////////////
void Particle::initOrientation(const tf::Quaternion &orientation)
{
  this->pose.setRotation(orientation);
}

////////////////////////////////////////////////////////////////////////////////
void Particle::predict(
  const tf::Vector3 &delta_pos, const tf::Quaternion &delta_rot,
  //const double &deltaT,
  std::mt19937 &gen)
{
  // TODO:
  // apply the relative pose from the previous step with some noise.
  std::normal_distribution<> motion_noise_lin(0, 0.05);
  // this->vel_linear = tf::Vector3(
  //   vel.x() + motion_noise_lin(gen),
  //   vel.y() + motion_noise_lin(gen),
  //   vel.z() + motion_noise_lin(gen));
  // tf::Vector3 new_pos = this->pose.getOrigin()
  //                     + tf::Vector3(
  //                         this->vel_linear.x() * deltaT,
  //                         this->vel_linear.y() * deltaT,
  //                         this->vel_linear.z() * deltaT);
  tf::Vector3 delta_pos_noise(
    delta_pos.x() + motion_noise_lin(gen),
    delta_pos.y() + motion_noise_lin(gen),
    delta_pos.z() + motion_noise_lin(gen));
  this->pose = this->pose * tf::Transform(delta_rot, delta_pos_noise);
}

////////////////////////////////////////////////////////////////////////////////
double Particle::evaluate(
  const std::map<std::string, double> &range_data,
  const octomap::Pointcloud &scan)
{
  double log_lik = 0;
  // int hits = 0;

  // evaluation by range data
  for (auto range_name: range_topics)
  {
    auto data = range_data.find(range_name);
    if (data == range_data.end())
      continue;
    double dist = data->second;
    if (dist >= range_max || dist <= range_min)
      continue;
    tf::Vector3 tf_pos = range_poses[range_name].getOrigin();
    tf::Vector3 tf_sens(dist, 0, 0);
    tf::Vector3 tf_target = range_poses[range_name] * tf_sens;

    octomap::point3d oct_pos(tf_pos.x(), tf_pos.y(), tf_pos.z());
    octomap::point3d oct_target(tf_target.x(), tf_target.y(), tf_target.z());
    octomap::point3d direction = oct_pos - oct_target;
    octomap::point3d hit;
    if (this->map->castRay(oct_target, direction, hit,
        true, dist + 0.2)) //ignoreUnknownCells = true, maxRange
    {
      // if (this->map->coordToKeyChecked(hit, key) &&
      //  (node = this->map->search(key,0 /*depth*/)))
      // {
        double err = (oct_target - hit).norm();
        double sigma = 0.2; // standard deviation

        log_lik +=
          -std::log(2*3.14159*sigma*sigma)/2.0 - err*err/sigma/sigma/2.0;
      // }
    }
  }

  tf::Pose sens_pose = this->pose;

  // for all point in the point cloud
  octomap::OcTreeKey key;
  //octomap::OcTreeNode *node;
  int offset = scan.size()/30;
  for (unsigned int ip = 0; ip < scan.size(); ip += offset)
  {
    tf::Vector3 tf_pos = sens_pose.getOrigin();
    tf::Vector3 tf_sens = tf::Vector3(scan[ip].x(), scan[ip].y(), scan[ip].z());
    tf::Vector3 tf_target
          = sens_pose * tf_sens;

    double dist = tf_sens.length();
    if (dist < 0.5)
      continue;
    octomap::point3d oct_pos(tf_pos.x(), tf_pos.y(), tf_pos.z());
    octomap::point3d oct_target(tf_target.x(), tf_target.y(), tf_target.z());
    octomap::point3d direction = oct_pos - oct_target;
    octomap::point3d hit;
    if (this->map->castRay(oct_target, direction, hit,
        true, dist + 0.2)) //ignoreUnknownCells = true, maxRange
    {
      // if (this->map->coordToKeyChecked(hit, key) &&
      //  (node = this->map->search(key,0 /*depth*/)))
      // {
        double err = (oct_target - hit).norm();
        double sigma = 0.2; // standard deviation

        log_lik +=
          -std::log(2*3.14159*sigma*sigma)/2.0 - err*err/sigma/sigma/2.0;
      // }
    }
  }

  // for (unsigned int ip = 0; ip < scan.size(); ++ip)
  // {
  //   tf::Vector3 point
  //         = sens_pose * tf::Vector3(scan[ip].x(), scan[ip].y(), scan[ip].z());
  //   if (this->map->coordToKeyChecked(
  //         point.x(), point.y(), point.z(), key)
  //       && (node = this->map->search(key,0)))
  //   {
  //     log_lik += std::log(node->getOccupancy());
  //     ++hits;
  //   }
  // }
  // return 0 if the major part of the point cloud landed on unknown cells.
  //ROS_DEBUG_STREAM("hits: " << hits);
  return std::exp(log_lik);
}

////////////////////////////////////////////////////////////////////////////////
void Particle::update_map(const octomap::Pointcloud &scan)
{
  octomath::Vector3 sensor_org(0, 0, 0);
  tf::Vector3 pose_org = this->pose.getOrigin();
  tf::Quaternion pose_rot = this->pose.getRotation();
  octomath::Pose6D frame_org(
    octomath::Vector3(pose_org.x(), pose_org.y(), pose_org.z()),
    octomath::Quaternion(
      pose_rot.w(), pose_rot.x(), pose_rot.y(), pose_rot.z())
  );
  this->map->insertPointCloud(scan, sensor_org, frame_org);
}

////////////////////////////////////////////////////////////////////////////////
void Particle::compress_map()
{
  this->map->toMaxLikelihood();
  this->map->prune();
}

////////////////////////////////////////////////////////////////////////////////
void pf_main()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // publisher for synchronization of exchange
  std::string sync_up_topic;
  ros::Publisher sync_pub;
  if (nh.getParam("/sync_up_topic", sync_up_topic)) // global param
  {
    sync_pub = nh.advertise<mav_tunnel_nav::SrcDstMsg>(sync_up_topic, 1);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: sync_up_topic");
  }

  // publisher for data exchange
  std::string data_up_topic;
  ros::Publisher data_pub;
  if (nh.getParam("/data_up_topic", data_up_topic)) // global param
  {
    data_pub = nh.advertise<mav_tunnel_nav::Particles>(data_up_topic, 1);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: data_up_topic");
  }

  // random numbers
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::uniform_real_distribution<> dis(0, 1.0);

  tf::TransformBroadcaster tf_broadcaster;

  pnh.getParam("odom_reset_topic", odom_reset_topic);
  ros::Publisher odom_reset_pub
    = nh.advertise<nav_msgs::Odometry>(odom_reset_topic, 1);

  std::string octomap_topic;
  pnh.getParam("octomap_topic", octomap_topic);
  ros::Publisher map_pub = nh.advertise<octomap_msgs::Octomap>(octomap_topic, 1);
  ros::Publisher marker_occupied_pub
    = nh.advertise<visualization_msgs::MarkerArray>("map_marker_occupied", 1);
  ros::Publisher vis_poses_pub
    = nh.advertise<geometry_msgs::PoseArray>("loc_vis_poses", 1, true);

  pnh.getParam("world_frame_id", world_frame_id);
  pnh.getParam("robot_frame_id", robot_frame_id);

  // === Initialize PF ===
  int n_particles;
  double update_freq;
  pnh.getParam("n_particles", n_particles);
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

  double t_pose_adjust;
  pnh.getParam("t_pose_adjust", t_pose_adjust);
  const ros::Duration phase_pose_adjust(t_pose_adjust);
  double t_only_mapping;
  pnh.getParam("t_only_mapping", t_only_mapping);
  const ros::Duration phase_only_mapping(t_only_mapping);
  int mapping_interval;
  pnh.getParam("mapping_interval", mapping_interval);
  int publish_interval;
  pnh.getParam("publish_interval", publish_interval);
  int vismap_interval;
  pnh.getParam("vismap_interval", vismap_interval);
  int visloc_interval;
  pnh.getParam("visloc_interval", visloc_interval);
  int compress_interval;
  pnh.getParam("compress_interval", compress_interval);
  int locdata_interval;
  pnh.getParam("locdata_interval", locdata_interval);

  std::vector< std::shared_ptr<Particle> > particles;
  for (int i = 0; i < n_particles; ++i)
  {
    particles.push_back(
      std::make_shared<Particle>(
        init_x, init_y, init_z, init_Y,
        resol, probHit, probMiss, threshMin, threshMax));
  }
  std::vector<double> weights(n_particles);
  std::vector<double> errors(n_particles);
  const ros::Time initial_update = ros::Time::now();
  ros::Time last_update = initial_update;

  tf::Pose pose_prev;
  tf::Pose pose_curr;
  //tf::Transform vel;

  PointCloudT::Ptr depth_cam_pc(new PointCloudT());
  tf::Quaternion rotation;
  rotation.setRPY(-PI/2.0, 0, -PI/2.0);
  //const tf::Pose camera_pose(rotation, tf::Vector3(0, 0, 0));
  const tf::Pose camera_pose(rotation, tf::Vector3(0.03, 0, -0.06));

  int counts_publish = 0;
  int counts_visualize_map = 0;
  int counts_visualize_loc = 0;
  int counts_map_update = 0;
  int counts_compress = 0;
  int counts_locdata = 0;

  std::uniform_int_distribution<int> dwnsmp_start(0, depth_cam_pc_downsample-1);

  // initialize the estimated pose
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    if (now > last_update + update_phase)
    {
      ROS_DEBUG("rbpf: iteration for pose init");
      // initialize the time step
      last_update = now;
      if (now > initial_update + phase_pose_adjust)
      {
        // Odometry data
        tf::Quaternion orientation;
        //tf::Transform diff_pose;
        {
          std::lock_guard<std::mutex> lk(odom_mutex);
          // tf::Vector3 pos(
          //   odom_buff.pose.pose.position.x,
          //   odom_buff.pose.pose.position.y,
          //   odom_buff.pose.pose.position.z);
          // tf::Quaternion dir(
          //   odom_buff.pose.pose.orientation.x,
          //   odom_buff.pose.pose.orientation.y,
          //   odom_buff.pose.pose.orientation.z,
          //   odom_buff.pose.pose.orientation.w);
          // pose_curr.setOrigin(pos);
          // pose_curr.setRotation(dir);
          orientation = tf::Quaternion(
            odom_buff.pose.pose.orientation.x,
            odom_buff.pose.pose.orientation.y,
            odom_buff.pose.pose.orientation.z,
            odom_buff.pose.pose.orientation.w);
          // diff_pose = pose_prev.inverse() * pose_curr;
          // pose_prev = pose_curr;
        }

        // update the estimated poses with the odometry data.
        for (auto p: particles)
        {
          p->initOrientation(orientation);
        }

        break;
      }
    }
  }

  // the main loop
  while (ros::ok())
  {
    // === Update PF ===
    ros::Time now = ros::Time::now();
    if (now > last_update + update_phase)
    {
      ROS_DEBUG("rbpf: new iteration");

      // { // just for debugging
      //   // it shows the frames on the world frame.
      //   for (auto range_topic: range_topics)
      //   {
      //
      //     tf::StampedTransform tf_stamped(
      //       range_poses[range_topic], now,
      //       world_frame_id, range_topic);
      //     tf_broadcaster.sendTransform(tf_stamped);
      //   }
      // }

      // calculate the delta T
      //const double deltaT = (now - last_update).toSec();
      // initialize the time step
      last_update = now;

      std::map<std::string, double> range_data;
      {
        std::lock_guard<std::mutex> lk(range_mutex);
        for (auto item: range_buff)
        {
          range_data[item.first] = item.second;
        }
      }
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
      // Odometry data
      tf::Transform diff_pose;
      {
        // TODO:
        // get the relative pose from the previous step
        // then, reset it to 0
        std::lock_guard<std::mutex> lk(odom_mutex);
        tf::Vector3 pos(
          odom_buff.pose.pose.position.x,
          odom_buff.pose.pose.position.y,
          odom_buff.pose.pose.position.z);
        tf::Quaternion dir(
          odom_buff.pose.pose.orientation.x,
          odom_buff.pose.pose.orientation.y,
          odom_buff.pose.pose.orientation.z,
          odom_buff.pose.pose.orientation.w);
        pose_curr.setOrigin(pos);
        pose_curr.setRotation(dir);
        diff_pose = pose_prev.inverse() * pose_curr;
        pose_prev = pose_curr;
        // tf::Vector3 vel_lin(
        //   odom_buff.twist.twist.linear.x,
        //   odom_buff.twist.twist.linear.y,
        //   odom_buff.twist.twist.linear.z);
        // tf::Quaternion vel_ang;
        // vel_ang.setRPY(
        //   odom_buff.twist.twist.angular.x,
        //   odom_buff.twist.twist.angular.y,
        //   odom_buff.twist.twist.angular.z);
        // vel.setOrigin(vel_lin);
        // vel.setRotation(vel_ang);
      }

      int index_best = 0;
      double max_weight = 0;
      double weight_sum = 0;
      if (now > initial_update + phase_pose_adjust + phase_only_mapping)
      {
        // initialize weights and errors
        for (int i = 0; i < n_particles; ++i)
        {
          weights[i] = 0;
          errors[i] = 0;
        }

        ROS_DEBUG("rbpf: predict");
        // predict PF (use odometory)
        const tf::Vector3 delta_pos = diff_pose.getOrigin();
        const tf::Quaternion delta_rot = diff_pose.getRotation();
        for (auto p: particles)
        {
          // move the particle
          // TODO:
          // call predict with the relative pose.
          p->predict(delta_pos, delta_rot,
            // tf::Vector3(
            //   odom_buff.twist.twist.linear.x,
            //   odom_buff.twist.twist.linear.y,
            //   odom_buff.twist.twist.linear.z),
            // tf::Quaternion(
            //   odom_buff.pose.pose.orientation.x,
            //   odom_buff.pose.pose.orientation.y,
            //   odom_buff.pose.pose.orientation.z,
            //   odom_buff.pose.pose.orientation.w),
            //deltaT,
            gen);
        }

        ROS_DEBUG("rbpf: evaluate");
        // weight PF (use depth cam)
        for (int i = 0; i < n_particles; ++i)
        {
          // Calculate a probability ranging from 0 to 1.
          weights[i] = particles[i]->evaluate(range_data, octocloud);
          weight_sum += weights[i];
        }
        for (int i = 0; i < n_particles; ++i)
        {
          ROS_DEBUG("eval[%2d]: %7.2f", i, weights[i]/weight_sum);
        }

        // resample PF (and update map)
        if (weight_sum != 0)
        {
          ROS_DEBUG("rbpf: resample start");
          // http://mrpt.ual.es/reference/devel/_c_particle_filter_data_8h_source.html#l00109
          std::vector<int> indx_list(n_particles);
          for (int i = 0; i < n_particles; ++i)
          {
            double rval = dis(gen);
            double weight_buff = 0;
            int index = 0;
            for (; index < n_particles - 1; ++index)
            {
              weight_buff += weights[index]/weight_sum;
              if (rval <= weight_buff)
                break;
            }
            indx_list[i] = index;
            if (i == 0 || max_weight < weights[index]/weight_sum)
            {
              max_weight = weights[index]/weight_sum;
              index_best = i;
            }
          }
          std::vector<int> indx_unused(n_particles, -1);

          std::vector< std::shared_ptr<Particle> > new_generation;
          for (int i = 0; i < n_particles; ++i)
          {
            const int prev_indx = indx_unused[indx_list[i]];
            if (prev_indx == -1)
            {
              new_generation.push_back(std::move(particles[indx_list[i]]));
              indx_unused[indx_list[i]] = i;

              // update the map
              if (counts_map_update >= mapping_interval && octocloud.size() > 0)
              {
                new_generation[i]->update_map(octocloud);
              }
            }
            else
            {
              new_generation.push_back(
                std::make_shared<Particle>(*new_generation[prev_indx]));
            }
          }

          // Copy the children to the parents.
          particles = std::move(new_generation);

          // update the map
          if (counts_map_update >= mapping_interval)
          {
            counts_map_update = 0;
          }
          else
          {
            ++counts_map_update;
          }
          ROS_DEBUG("rbpf: resample done");
        }
        else
        {
          // update the map
          if (counts_map_update >= mapping_interval && octocloud.size() > 0)
          {
            ROS_DEBUG("rbpf: update map");
            for (auto p: particles)
            {
              p->update_map(octocloud);
            }
            counts_map_update = 0;
          }
          else
          {
            ++counts_map_update;
          }
        }
      }
      else if (octocloud.size() > 0)
      {
        // mapping only at the beginning
        ROS_DEBUG("rbpf: mapping only");
        for (auto p: particles)
        {
          p->update_map(octocloud);
        }
      }

      // compress maps
      if (counts_compress >= compress_interval)
      {
        ROS_DEBUG("rbpf: compress");
        for (auto p: particles)
        {
          p->compress_map();
        }
        counts_compress = 0;
      }
      else
      {
        ++counts_compress;
      }

      double x, y, z;
      x = 0;
      y = 0;
      z = 0;
      for (int i = 0; i < n_particles; ++i)
      {
        tf::Vector3 buff = particles[i]->getPose().getOrigin();
        x += buff.x()/n_particles;
        y += buff.y()/n_particles;
        z += buff.z()/n_particles;
      }
      tf::Vector3 average_loc(x, y, z);
      // x = 0;
      // y = 0;
      // z = 0;
      // for (int i = 0; i < n_particles; ++i)
      // {
      //   tf::Vector3 buff = particles[i]->getVel();
      //   x += buff.x()/n_particles;
      //   y += buff.y()/n_particles;
      //   z += buff.z()/n_particles;
      // }
      // tf::Vector3 average_vel(x, y, z);

      // publish data
      if (counts_publish >= publish_interval)
      {
        ROS_DEBUG("rbpf: publish data");
        octomap_msgs::Octomap map;
        map.header.frame_id = world_frame_id;
        map.header.stamp = now;
        if (octomap_msgs::fullMapToMsg(*particles[index_best]->getMap(), map))
          map_pub.publish(map);
        else
          ROS_ERROR("Error serializing OctoMap");

        tf::Quaternion q = particles[index_best]->getPose().getRotation();
        tf::StampedTransform tf_stamped(
          tf::Transform(q, average_loc), now,
          world_frame_id, robot_frame_id);
        tf_broadcaster.sendTransform(tf_stamped);
        counts_publish = 0;
      }
      else
      {
        ++counts_publish;
      }

      if (counts_locdata >= locdata_interval)// && max_weight > 1.0/n_particles * 3.0)
      {
        nav_msgs::Odometry locdata;
        locdata.header.frame_id = world_frame_id;
        locdata.header.stamp = now;
        locdata.child_frame_id = robot_frame_id;
        locdata.pose.pose.position.x = average_loc.x();
        locdata.pose.pose.position.y = average_loc.y();
        locdata.pose.pose.position.z = average_loc.z();
        locdata.twist.twist.linear.x = 0;//average_vel.x();
        locdata.twist.twist.linear.y = 0;//average_vel.y();
        locdata.twist.twist.linear.z = 0;//average_vel.z();
        odom_reset_pub.publish(locdata);
        ROS_DEBUG_STREAM("odom_reset: weight = " << max_weight << " | UPDATE!!!!!!!!!");
        ROS_DEBUG("vx: %7.2f, vy: %7.2f, vz: %7.2f",
          locdata.twist.twist.linear.x,
          locdata.twist.twist.linear.y,
          locdata.twist.twist.linear.z);
        counts_locdata = 0;
      }
      else
      {
        ROS_DEBUG_STREAM("odom_reset: weight = " << max_weight << " | no update");
        // tf::Vector3 buff = particles[index_best]->getVel();
        // ROS_DEBUG("vx: %7.2f, vy: %7.2f, vz: %7.2f", buff.x(), buff.y(), buff.z());
        ++counts_locdata;
      }

      // visualization
      if (counts_visualize_map >= vismap_interval)
      {
        ROS_DEBUG("rbpf: visualize map");
        const octomap::OcTree* m = particles[index_best]->getMap();
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

      if (counts_visualize_loc >= visloc_interval)
      {
        ROS_DEBUG("rbpf: visualize loc");
        // publish poses
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = world_frame_id;
        poseArray.header.stamp = now;
        poseArray.poses.resize(n_particles);
        for (int i = 0; i < n_particles; ++i)
        {
          tf::Pose pose = particles[i]->getPose();
          tf::Vector3 position = pose.getOrigin();
          tf::Quaternion orientation = pose.getRotation();
          poseArray.poses[i].position.x = position.x();
          poseArray.poses[i].position.y = position.y();
          poseArray.poses[i].position.z = position.z();
          poseArray.poses[i].orientation.x = orientation.x();
          poseArray.poses[i].orientation.y = orientation.y();
          poseArray.poses[i].orientation.z = orientation.z();
          poseArray.poses[i].orientation.w = orientation.w();
        }
        vis_poses_pub.publish(poseArray);

        counts_visualize_loc = 0;
      }
      else
      {
        ++counts_visualize_loc;
      }
      ROS_DEBUG("rbpf: end of iteration");
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rbpf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // subscriber for beacon
  std::string beacon_down_topic;
  ros::Subscriber beacon_sub;
  if (nh.getParam("/beacon_down_topic", beacon_down_topic)) // global param
  {
    beacon_sub = nh.subscribe(beacon_down_topic, 1000, beaconCallback);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: beacon_down_topic");
  }
  // subscriber for synchronization of exchange
  std::string sync_down_topic;
  ros::Subscriber sync_sub;
  if (nh.getParam("/sync_down_topic", sync_down_topic)) // global param
  {
    sync_sub = nh.subscribe(sync_down_topic, 1000, syncCallback);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: sync_down_topic");
  }
  // subscriber for data exchange
  std::string data_down_topic;
  ros::Subscriber data_sub;
  if (nh.getParam("/data_down_topic", data_down_topic)) // global param
  {
    data_sub = nh.subscribe(data_down_topic, 1000, dataCallback);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: data_down_topic");
  }

  pnh.getParam("odom_topic", odom_topic);
  pnh.getParam("pc_topic", pc_topic);
  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1000, odomCallback);
  ros::Subscriber pc_sub = nh.subscribe(pc_topic, 1000, pcCallback);

  pnh.getParam("range_max", range_max);
  pnh.getParam("range_min", range_min);
  std::string str_buff;
  pnh.param<std::string>("range_list", str_buff, "");
  std::stringstream ss(str_buff);
  std::string token;
  double x,y,z,P,R,Y;
  std::vector<ros::Subscriber> range_subs;
  while (ss >> token >> x >> y >> z >> R >> P >> Y)
  {
    ROS_DEBUG_STREAM(
      "range: " << token << ", "
                << x << ", " << y << ", " << z << ", "
                << R << ", " << P << ", " << Y);

    range_subs.push_back(
      nh.subscribe(token, 1000, rangeCallback)
    );
    range_topics.push_back(token);
    tf::Vector3 pos(x, y, z);
    tf::Quaternion rot;
    rot.setRPY(R * PI / 180.0, P * PI / 180.0, Y * PI / 180.0);
    tf::Pose range_pose(rot, pos);
    range_poses[token] = range_pose;
  }

  std::thread t(pf_main);

  ros::spin();

  t.join();

  return(0);
}
