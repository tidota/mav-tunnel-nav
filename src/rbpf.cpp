// rbpf.cpp
// 191028
// Rao-Blackwellized Particle Filter

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <string>
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

#include <mav_tunnel_nav/SrcDst.h>
#include <mav_tunnel_nav/Beacon.h>
#include <mav_tunnel_nav/Particles.h>
#include <mav_tunnel_nav/OctomapWithSegId.h>

#include <ros/ros.h>

// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>

#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>

#include <std_srvs/SetBool.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>

#include "rbpf.h"

////////////////////////////////////////////////////////////////////////////////
RBPF::RBPF(ros::NodeHandle& nh, ros::NodeHandle& pnh):
  depth_cam_pc(new PointCloudT())
{
  state = Init;

  // get this robot's name
  if (!pnh.getParam("robot_name", robot_name))
    ROS_ERROR_STREAM("no ros parameter: robot_name");

  // subscriber for beacon
  std::string beacon_down_topic;
  if (nh.getParam("/beacon_down_topic", beacon_down_topic)) // global param
  {
    beacon_sub
      = nh.subscribe(beacon_down_topic, 1000, &RBPF::beaconCallback, this);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: beacon_down_topic");
  }
  // subscriber for synchronization of exchange
  std::string sync_down_topic;
  if (nh.getParam("/sync_down_topic", sync_down_topic)) // global param
  {
    sync_sub
      = nh.subscribe(sync_down_topic, 1000, &RBPF::syncCallback, this);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: sync_down_topic");
  }
  // subscriber for data exchange
  std::string data_down_topic;
  if (nh.getParam("/data_down_topic", data_down_topic)) // global param
  {
    data_sub
      = nh.subscribe(data_down_topic, 1000, &RBPF::dataCallback, this);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: data_down_topic");
  }

  pnh.getParam("odom_topic", odom_topic);
  pnh.getParam("pc_topic", pc_topic);
  odom_sub = nh.subscribe(odom_topic, 1000, &RBPF::odomCallback, this);
  pc_sub = nh.subscribe(pc_topic, 1000, &RBPF::pcCallback, this);

  pnh.getParam("range_max", range_max);
  pnh.getParam("range_min", range_min);
  std::string str_buff;
  pnh.param<std::string>("range_list", str_buff, "");
  std::stringstream ss(str_buff);
  std::string token;
  double x,y,z,P,R,Y;
  while (ss >> token >> x >> y >> z >> R >> P >> Y)
  {
    range_subs.push_back(
      nh.subscribe(token, 1000, &RBPF::rangeCallback, this)
    );
    range_topics.push_back(token);
    tf::Vector3 pos(x, y, z);
    tf::Quaternion rot;
    rot.setRPY(R * M_PI / 180.0, P * M_PI / 180.0, Y * M_PI / 180.0);
    tf::Pose range_pose(rot, pos);
    range_poses[token] = range_pose;
  }


  // ====================


  if (!nh.getParam("/comm_range", comm_range))
    ROS_ERROR_STREAM("no parameter for rbpf: comm_range");

  // publisher for synchronization of exchange
  std::string sync_up_topic;
  if (nh.getParam("/sync_up_topic", sync_up_topic)) // global param
  {
    sync_pub = nh.advertise<mav_tunnel_nav::SrcDst>(sync_up_topic, 1);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: sync_up_topic");
  }

  // publisher for data exchange
  std::string data_up_topic;
  if (nh.getParam("/data_up_topic", data_up_topic)) // global param
  {
    data_pub = nh.advertise<mav_tunnel_nav::Particles>(data_up_topic, 1);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: data_up_topic");
  }

  // random numbers
  //std::random_device rd{};
  //std::mt19937 gen{rd()};
  int seed_indivloc;
  if (!pnh.getParam("seed_indivloc", seed_indivloc))
  ROS_ERROR_STREAM("no param: seed_indivloc");
  gen_indivloc.seed(seed_indivloc);

  //std::uniform_real_distribution<> dis(0, 1.0);


  // if(!pnh.getParam("odom_reset_topic", odom_reset_topic))
  //   ROS_ERROR_STREAM("no param: odom_reset_topic");
  // odom_reset_pub = nh.advertise<nav_msgs::Odometry>(odom_reset_topic, 1);

  std::string octomap_topic;
  if(!pnh.getParam("octomap_topic", octomap_topic))
    ROS_ERROR_STREAM("no param: octomap_topic");
  map_pub = nh.advertise<mav_tunnel_nav::OctomapWithSegId>(octomap_topic, 1);
  marker_occupied_pub
    = nh.advertise<visualization_msgs::MarkerArray>("map_marker_occupied", 1);
  vis_poses_pub
    = nh.advertise<geometry_msgs::PoseArray>("loc_vis_poses", 1, true);

  if(!pnh.getParam("world_frame_id", world_frame_id))
    ROS_ERROR_STREAM("no param: world_frame_id");
  if(!pnh.getParam("robot_frame_id", robot_frame_id))
    ROS_ERROR_STREAM("no param: robot_frame_id");

  // === Initialize PF ===
  double update_freq;
  if(!pnh.getParam("n_particles", n_particles))
    ROS_ERROR_STREAM("no param: n_particles");
  if(!pnh.getParam("update_freq", update_freq))
    ROS_ERROR_STREAM("no param: update_freq");
  update_phase = ros::Duration(1.0/update_freq);

  // int depth_cam_pc_downsample;
  // if(!pnh.getParam("depth_cam_pc_downsample", depth_cam_pc_downsample))
  //   ROS_ERROR_STREAM("no param: depth_cam_pc_downsample");

  if(!pnh.getParam("init_x", init_x))
    ROS_ERROR_STREAM("no param: init_x");
  if(!pnh.getParam("init_y", init_y))
    ROS_ERROR_STREAM("no param: init_y");
  if(!pnh.getParam("init_z", init_z))
    ROS_ERROR_STREAM("no param: init_z");
  if(!pnh.getParam("init_Y", init_Y))
    ROS_ERROR_STREAM("no param: init_Y");
  if(!pnh.getParam("map_resol", resol))
    ROS_ERROR_STREAM("no param: map_resol");
  if(!pnh.getParam("map_probHit", probHit))
    ROS_ERROR_STREAM("no param: map_probHit");
  if(!pnh.getParam("map_probMiss", probMiss))
    ROS_ERROR_STREAM("no param: map_probMiss");
  if(!pnh.getParam("map_threshMin", threshMin))
    ROS_ERROR_STREAM("no param: map_threshMin");
  if(!pnh.getParam("map_threshMax", threshMax))
    ROS_ERROR_STREAM("no param: map_threshMax");
  if(!pnh.getParam("motion_noise_lin_sigma", motion_noise_lin_sigma))
    ROS_ERROR_STREAM("no param: motion_noise_lin_sigma");
  if(!pnh.getParam("motion_noise_rot_sigma", motion_noise_rot_sigma))
    ROS_ERROR_STREAM("no param: motion_noise_rot_sigma");
  if(!pnh.getParam("sensor_noise_range_sigma", sensor_noise_range_sigma))
    ROS_ERROR_STREAM("no param: sensor_noise_range_sigma");
  if(!pnh.getParam("sensor_noise_depth_sigma", sensor_noise_depth_sigma))
    ROS_ERROR_STREAM("no param: sensor_noise_depth_sigma");

  double t_pose_adjust;
  if(!pnh.getParam("t_pose_adjust", t_pose_adjust))
    ROS_ERROR_STREAM("no param: t_pose_adjust");
  phase_pose_adjust = ros::Duration(t_pose_adjust);
  double t_only_mapping;
  if(!pnh.getParam("t_only_mapping", t_only_mapping))
    ROS_ERROR_STREAM("no param: t_only_mapping");
  phase_only_mapping = ros::Duration(t_only_mapping);
  if(!pnh.getParam("mapping_interval", mapping_interval))
    ROS_ERROR_STREAM("no param: mapping_interval");
  if(!pnh.getParam("publish_interval", publish_interval))
    ROS_ERROR_STREAM("no param: publish_interval");
  if(!pnh.getParam("vismap_interval", vismap_interval))
    ROS_ERROR_STREAM("no param: vismap_interval");
  if(!pnh.getParam("compress_interval", compress_interval))
    ROS_ERROR_STREAM("no param: compress_interval");
  if(!pnh.getParam("enable_indivLoc", enable_indivLoc))
    ROS_ERROR_STREAM("no param: enable_indivLoc");
  // int locdata_interval;
  // pnh.getParam("locdata_interval", locdata_interval);
  counts_publish = 0;
  counts_visualize_map = 0;
  counts_visualize_loc = 0;
  counts_map_update = 0;
  counts_compress = 0;

  //       each vector of particles represent a segment.
  nseg = 0;
  segments.resize(1);
  for (int i = 0; i < n_particles; ++i)
  {
    segments.back().push_back(
      std::make_shared<Particle>(
        init_x, init_y, init_z, init_Y,
        resol, probHit, probMiss, threshMin, threshMax,
        motion_noise_lin_sigma, motion_noise_rot_sigma,
        sensor_noise_range_sigma, sensor_noise_depth_sigma));
  }
  segments_index_best.resize(1);
  cumul_weights_slam.resize(n_particles);
  errors.resize(n_particles);

  // === entry detection ===
  // the name of the next robot for map transfer
  {
    std::string robot_name_prefix;
    int robot_num;
    {
      std::stringstream sp;
      std::stringstream sn;
      for (auto c: robot_name)
      {
        if (std::isalpha(c))
          sp << c;
        else
          sn << c;
      }
      sp >> robot_name_prefix;
      sn >> robot_num;
    }
    std::stringstream ss;
    ss << robot_name_prefix << (robot_num + 2);
    next_robot_name = ss.str();
  }

  //tf::Transform vel;

  tf::Quaternion rotation;
  rotation.setRPY(-M_PI/2.0, 0, -M_PI/2.0);
  //const tf::Pose camera_pose(rotation, tf::Vector3(0, 0, 0));
  camera_pose = tf::Pose(rotation, tf::Vector3(0.03, 0, -0.06));

  tf_listener = std::make_shared<tf::TransformListener>();
  save_traj = false;
  if (pnh.getParam("save_traj", save_traj))
  {
    traj_filename
      = "./" + robot_name + "_"
        + std::to_string((int)(ros::WallTime::now().toSec()))
        + "_traj.txt";
  }
  else
  {
    save_traj = false;
  }

  // =================

  // parameters for cooperative localization
  if (!pnh.getParam("Nref", Nref))
    ROS_ERROR_STREAM("no param: Nref");
  if (!pnh.getParam("seed_cooploc", seed_cooploc))
    ROS_ERROR_STREAM("no param: seed_cooploc");
  if (!pnh.getParam("enable_cooploc", enable_cooploc))
    ROS_ERROR_STREAM("no param: enable_cooploc");
  if (!pnh.getParam("enable_conservative", enable_conservative))
    ROS_ERROR_STREAM("no param: enable_conservative");

  if (!pnh.getParam("conserv_omega", conserv_omega))
    ROS_ERROR_STREAM("no param: conserv_omega");
  if (!pnh.getParam("sigma_kde", sigma_kde))
    ROS_ERROR_STREAM("no param: sigma_kde");

  if (!pnh.getParam("sigmaLocR", sigmaLocR))
    ROS_ERROR_STREAM("no param: sigmaLocR");
  if (!pnh.getParam("sigmaLocT", sigmaLocT))
    ROS_ERROR_STREAM("no param: sigmaLocT");
  // parameters for evaluation
  if (!pnh.getParam("gl_eval_cons", gl_eval_cons))
    ROS_ERROR_STREAM("no param: gl_eval_cons");
  if (!pnh.getParam("ml_eval_cons", ml_eval_cons))
    ROS_ERROR_STREAM("no param: ml_eval_cons");

  double beacon_lifetime_buff;
  if (!pnh.getParam("beacon_lifetime", beacon_lifetime_buff))
    ROS_ERROR_STREAM("no param: beacon_lifetime");
  beacon_lifetime = ros::Duration(beacon_lifetime_buff);
  double cooploc_phase_buff;
  if (!pnh.getParam("cooploc_phase", cooploc_phase_buff))
    ROS_ERROR_STREAM("no param: cooploc_phase");
  cooploc_phase = ros::Duration(cooploc_phase_buff);
  double syncinit_timeout_buff;
  if (!pnh.getParam("syncinit_timeout", syncinit_timeout_buff))
    ROS_ERROR_STREAM("no param: syncinit_timeout");
  syncinit_timeout = ros::Duration(syncinit_timeout_buff);

  gen_cooploc.seed(seed_cooploc);

  if (!pnh.getParam("enable_segmentation", enable_segmentation))
    ROS_ERROR_STREAM("no param: enable_segmentation");
  double init_seg_phase_buff;
  if (!pnh.getParam("init_seg_phase", init_seg_phase_buff))
    ROS_ERROR_STREAM("no param: init_seg_phase");
  init_seg_phase = ros::Duration(init_seg_phase_buff);
  if (!pnh.getParam("next_seg_thresh", next_seg_thresh))
    ROS_ERROR_STREAM("no param: next_seg_thresh");
  if (!pnh.getParam("enable_clr4seg", enable_clr4seg))
    ROS_ERROR_STREAM("no param: enable_clr4seg");

  // === For data exchange. ==
  // 95 % of difference should be in approx. 2.7 * sigma_kde
  // https://stats.stackexchange.com/questions/35012/mahalanobis-distance-and-percentage-of-the-distribution-represented
  sigma_kde_squared_x2 = 2 * sigma_kde * sigma_kde;
  data_msg.source = robot_name;
  cumul_weights.resize(n_particles);
  cumul_weights_comp.resize(n_particles);

  // For synchronization
  sync_msg.source = robot_name;


  if (!pnh.getParam("auto_enable_by_slam", auto_enable_by_slam))
    auto_enable_by_slam = false;
  srv_client
    = nh.serviceClient<std_srvs::SetBool>("/" + robot_name + "/enable");
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::beaconCallback(const mav_tunnel_nav::Beacon::ConstPtr& msg)
{
  if (state != Init && msg->destination.size() > 0)
  {
    std::lock_guard<std::mutex> lk(beacon_mutex);
    beacon_buffer[msg->source] = *msg;
    beacon_lasttime[msg->source] = ros::Time::now();
  }
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::syncCallback(const mav_tunnel_nav::SrcDst::ConstPtr& msg)
{
  if (state == IndivSLAM)
  {
    std::lock_guard<std::mutex> lk(sync_mutex);
    sync_msgs_buffer.push_back(*msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::dataCallback(const mav_tunnel_nav::Particles::ConstPtr& msg)
{
  if (state == SyncInit || state == DataWaiting)
  {
    std::lock_guard<std::mutex> lk(data_mutex);
    data_buffer[msg->source] = *msg;
    data_lasttime[msg->source] = ros::Time::now();
    last_data_src = msg->source;

    if (state == SyncInit)
    {
      state = DataSending;
    }
    else if (state == DataWaiting)
    {
      state = Update;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // update odom_pose by the odometry data.
  std::lock_guard<std::mutex> lk(odom_mutex);
  odom_buff = *msg;
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(pc_mutex);
  pc_buff = *msg;
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::rangeCallback(const sensor_msgs::Range::ConstPtr& new_range)
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
  // NOTE: There may be some bug in the simulator generating very small value
  //       intermitently. At the moment, it is replaced with range_max
  if (new_range->range >= 0.09)
    range_buff["range_" + new_range->header.frame_id.substr(pos1, pos2 - pos1)]
      = new_range->range;
  // range_buff["range_" + new_range->header.frame_id.substr(pos1, pos2 - pos1)]
  //   = new_range->range;
}

////////////////////////////////////////////////////////////////////////////////
inline int RBPF::drawIndex(
  const std::vector<double>& cumul_weights, std::mt19937& gen)
{
  std::uniform_real_distribution<>
    dist(0, cumul_weights[cumul_weights.size() - 1]);

  double val = dist(gen);
  int lo = 0;
  int hi = cumul_weights.size() - 1;

  while (lo < hi - 1)
  {
    int mid = (hi + lo)/2;

    if (val > cumul_weights[mid])
    {
      lo = mid;
    }
    else
    {
      hi = mid;
    }
  }
  int indx = hi;
  if (val <= cumul_weights[lo])
    indx = lo;
  return indx;
}

////////////////////////////////////////////////////////////////////////////////
inline void RBPF::prepareDataMsg(
  mav_tunnel_nav::Particles& data_msg, const std::string& destination,
  const std::vector< std::shared_ptr<Particle> >& particles)
{
  const int n_particles = particles.size();

  // set the destination
  data_msg.destination = destination;

  if (enable_conservative)
  {
    // calculate the weights
    for (int i = 0; i < n_particles; ++i)
    {
      cumul_weights[i] = 1.0;
    }
    for (int i = 0; i < n_particles; ++i)
    {
      // KDE: kernel density estimation.
      // estimate the value on the point of the probability distribution
      for (int j = i + 1; j < n_particles; ++j)
      {
        double diff
          = (particles[i]->getPose().getOrigin()
            - particles[j]->getPose().getOrigin()).length();
        double val = exp(-diff*diff/sigma_kde_squared_x2);
        cumul_weights[i] += val;
        cumul_weights[j] += val;
      }
      double buff = pow(cumul_weights[i], conserv_omega);

      // p^omega / p = p^(omega - 1)
      cumul_weights[i] = buff / cumul_weights[i];
      if (i > 0)
      {
        cumul_weights[i] += cumul_weights[i-1];
      }
      // p^(1-omega) / p = p^(-omega) = 1 / p^omega
      cumul_weights_comp[i] = 1.0 / buff;
      if (i > 0)
      {
        cumul_weights_comp[i] += cumul_weights_comp[i-1];
      }
    }
  }
  else
  {
    // without conservative method
    // i.e., the weights are uniform
    for (int i = 0; i < n_particles; ++i)
    {
      cumul_weights[i] = i + 1;
      cumul_weights_comp[i] = i + 1;
    }
  }

  data_msg.particles.resize(Nref);
  data_msg.cumul_weights.resize(Nref);
  for (int i = 0; i < Nref; ++i)
  {
    // get a particle of the other robot by msg.cumul_weights
    int indx = drawIndex(cumul_weights_comp, gen_cooploc);
    // NOTE: set the weight
    // (just in case. Maybe this part is no longer necessary as the particles
    // have already been resampled)
    data_msg.cumul_weights[i] = cumul_weights_comp[indx];
    // set the particle's poses.
    tf::Vector3 pos = particles[indx]->getPose().getOrigin();
    tf::Quaternion rot = particles[indx]->getPose().getRotation();
    data_msg.particles[i].position.x = pos.x();
    data_msg.particles[i].position.y = pos.y();
    data_msg.particles[i].position.z = pos.z();
    data_msg.particles[i].orientation.w = rot.w();
    data_msg.particles[i].orientation.x = rot.x();
    data_msg.particles[i].orientation.y = rot.y();
    data_msg.particles[i].orientation.z = rot.z();
  }
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::updateCurrPoseOfOdom()
{
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
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::getRanges(std::map<std::string, double>& range_data)
{
  std::lock_guard<std::mutex> lk(range_mutex);
  for (auto item: range_buff)
  {
    range_data[item.first] = item.second;
  }
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::getPC(octomap::Pointcloud& octocloud)
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
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*depth_cam_pc, *depth_cam_pc, indices);
    //
    // for (unsigned int i = 0; i < indices.size(); ++i)
    // {
    //   // if (pcl_isfinite(depth_cam_pc->points[i].x) &&
    //   //     pcl_isfinite(depth_cam_pc->points[i].y) &&
    //   //     pcl_isfinite(depth_cam_pc->points[i].z))
    //   // {
    //     tf::Vector3 point = camera_pose * tf::Vector3(
    //                                 depth_cam_pc->points[indices[i]].x,
    //                                 depth_cam_pc->points[indices[i]].y,
    //                                 depth_cam_pc->points[indices[i]].z);
    //     octocloud.push_back(
    //       octomap::point3d(point.x(), point.y(), point.z()));
    //   // }
    // }
    pc_buff.height = 0;
    pc_buff.width = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::indivSlamPredict(const tf::Transform& diff_pose)
{
  const tf::Vector3 delta_pos = diff_pose.getOrigin();
  const tf::Quaternion delta_rot = diff_pose.getRotation();
  for (auto p: segments.back())
  {
    // move the particle
    // call predict with the relative pose.
    p->predict(delta_pos, delta_rot, gen_indivloc);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::indivSlamEvaluate(
  const ros::Time& now,
  const std::map<std::string, double>& range_data,
  const octomap::Pointcloud& octocloud)
{
  for (int i = 0; i < n_particles; ++i)
  {
    cumul_weights_slam[i] = 0;
  }
  int index_best = 0;
  double max_weight = 0;
  double weight_sum = 0;
  for (int i = 0; i < n_particles; ++i)
  {
    // if the current time is not far away from the initial time
    if (nseg != 0 && now <= init_segment_time + init_seg_phase)
    {
      // call evaluate with the flag which is set to true.
      cumul_weights_slam[i]
        = segments.back()[i]->evaluate(
            range_data, range_min, range_max, range_topics, range_poses,
            octocloud, true);
    }
    else // otherwise, just call evaluate in the default way.
    {
      // Calculate a probability ranging from 0 to 1.
      cumul_weights_slam[i]
        = segments.back()[i]->evaluate(
            range_data, range_min, range_max, range_topics, range_poses,
            octocloud);
    }

    if (i == 0 || max_weight < cumul_weights_slam[i])
    {
      max_weight = cumul_weights_slam[i];
      index_best = i;
    }
    if (i > 0)
      cumul_weights_slam[i] += cumul_weights_slam[i-1];
  }
  weight_sum = cumul_weights_slam[n_particles-1];
  if (weight_sum != 0)
    max_weight /= weight_sum;
  segments_index_best.back() = index_best;
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::indivSlamResample()
{
  // resample PF (and update map)
  std::vector<int> indx_list(n_particles);
  for (int i = 0; i < n_particles; ++i)
  {
    indx_list[i] = drawIndex(cumul_weights_slam, gen_indivloc);
  }

  std::vector< std::shared_ptr<Particle> > new_generation;
  std::vector<int> indx_unused(n_particles, -1);
  for (int i = 0; i < n_particles; ++i)
  {
    const int prev_indx = indx_unused[indx_list[i]];
    if (prev_indx == -1)
    {
      new_generation.push_back(
        std::move(segments.back()[indx_list[i]]));
      indx_unused[indx_list[i]] = i;
    }
    else
    {
      new_generation.push_back(
        std::make_shared<Particle>(*new_generation[prev_indx]));
    }
  }
  // Copy the children to the parents.
  segments.back().swap(new_generation);
}

////////////////////////////////////////////////////////////////////////////////
bool RBPF::updateMap(const octomap::Pointcloud& octocloud)
{
  bool success = false;
  if (octocloud.size() > 0)
  {
    for (auto p: segments.back())
    {
      p->update_map(octocloud);
    }
    success = true;
  }
  return success;
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::compressMap()
{
  // compress maps
  for (auto p: segments.back())
  {
    p->compress_map();
  }
}

////////////////////////////////////////////////////////////////////////////////
std::string RBPF::checkSyncReq(const ros::Time& now)
{
  std::string syncreq_src = "";

  std::lock_guard<std::mutex> lk(sync_mutex);
  while(sync_msgs_buffer.size() > 0)
  {
    mav_tunnel_nav::SrcDst msg = sync_msgs_buffer.front();
    sync_msgs_buffer.pop_front();
    if (now - msg.stamp < syncinit_timeout * 0.7)
    {
      syncreq_src = msg.source;
      break;
    }
  }
  return syncreq_src;
}

////////////////////////////////////////////////////////////////////////////////
bool RBPF::initiateSync(const ros::Time& now)
{
  bool initiated = false;

  // decide if it should initiate interactions with a neighbor.

  // list up candidates to sync.
  std::vector<std::string> candidates;
  for (auto p: beacon_lasttime)
  {
    // NOTE: add an candiate if the packet is "fresh" enough and it is
    //       within 90% of comm range.
    if (now <= p.second + beacon_lifetime
      && beacon_buffer[p.first].estimated_distance < 0.9 * comm_range)
    {
      candidates.push_back(p.first);
    }
  }

  if (candidates.size() > 0)
  {
    // randomly select a neighbor to interact.
    std::uniform_int_distribution<int> dist(0, candidates.size() - 1);

    // send a sync packet.
    sync_msg.stamp = ros::Time::now();
    sync_msg.destination = candidates[dist(gen_cooploc_select)];
    sync_pub.publish(sync_msg);

    initiated = true;
  }
  return initiated;
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::exchangeData()
{
  // Odometry data
  tf::Transform diff_pose;
  updateCurrPoseOfOdom();
  diff_pose = pose_prev.inverse() * pose_curr;
  pose_prev = pose_curr;

  // predict PF (use odometory)
  const tf::Vector3 delta_pos = diff_pose.getOrigin();
  const tf::Quaternion delta_rot = diff_pose.getRotation();
  for (auto p: segments.back())
  {
    // move the particle
    // call predict with the relative pose.
    p->predict(delta_pos, delta_rot, gen_indivloc);
  }

  std::string dest;
  if (state == SyncReact)
    dest = last_sync_src;
  else
    dest = last_data_src;

  prepareDataMsg(data_msg, dest, segments.back());

  // send data to the other
  data_pub.publish(data_msg);
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::cooplocUpdate()
{
  mav_tunnel_nav::Particles msg;
  {
    std::lock_guard<std::mutex> lk(data_mutex);
    msg = data_buffer[last_data_src];
    //data_lasttime[last_data_src]
  }
  // === calculate weights for resampling === //
  // so what it has at this point?
  // cumul_weights
  // msg.estimated_distance
  // msg.estimated_orientation
  // msg.cumul_weights
  // msg.particles

  // convert geometry_msgs::Point to tf::Vector3.
  tf::Vector3 msg_estimated_orientation(
    msg.estimated_orientation.x,
    msg.estimated_orientation.y,
    msg.estimated_orientation.z);

  // cumulative weights for resampling.
  std::vector<double> cumul_weights_update(n_particles);

  double weight_max = 0;
  int index_best = -1;

  // for all particles
  for (int ip = 0; ip < n_particles; ++ip)
  {
    // initialize the wegith
    cumul_weights_update[ip] = 0;
    // get the particle's pose
    tf::Pose robot_pose = segments.back()[ip]->getPose();
    // for Nref
    for (unsigned int i = 0; i < msg.particles.size(); ++i)
    {
      // get a particle of the other robot by msg.cumul_weights
      auto neighbor_pose_msg = msg.particles[i];
      tf::Pose neighbor_pose(
        tf::Quaternion(
          neighbor_pose_msg.orientation.x,
          neighbor_pose_msg.orientation.y,
          neighbor_pose_msg.orientation.z,
          neighbor_pose_msg.orientation.w),
        tf::Vector3(
          neighbor_pose_msg.position.x,
          neighbor_pose_msg.position.y,
          neighbor_pose_msg.position.z));
      // simulate a measurement based on the sampled poses.
      tf::Vector3 sampled_loc
        = (neighbor_pose.inverse() * robot_pose).getOrigin();
      double sampled_range = sampled_loc.length();
      tf::Vector3 sampled_orientation = sampled_loc / sampled_range;

      // difference from the actual sensory data
      double diff_range = sampled_range - msg.estimated_distance;
      double diff_rad
        = std::acos(sampled_orientation.dot(msg_estimated_orientation));

      // calculate weight and add it
      if (msg.particles.size() == 1) // in case of global
      {
        cumul_weights_update[ip]
          += std::exp(
              -(diff_range*diff_range)
                /sigmaLocR/sigmaLocR/gl_eval_cons
              -(diff_rad*diff_rad)
                /sigmaLocT/sigmaLocT/gl_eval_cons);
      }
      else
      {
        cumul_weights_update[ip]
          += std::exp(
              -(diff_range*diff_range)
                /sigmaLocR/sigmaLocR/ml_eval_cons
              -(diff_rad*diff_rad)
                /sigmaLocT/sigmaLocT/ml_eval_cons);
      }
    }

    //   multiply with the original weights
    cumul_weights_update[ip]
      *= ((ip > 0)? cumul_weights[ip] - cumul_weights[ip-1]:
                    cumul_weights[0]);

    if (index_best == -1 || cumul_weights_update[ip] > weight_max)
    {
      index_best = ip;
      weight_max = cumul_weights_update[ip];
    }

    // make it cumuluative
    if (ip > 0)
      cumul_weights_update[ip] += cumul_weights_update[ip - 1];
  }

  // resampling
  std::vector<int> indx_list(n_particles);
  for (int ip = 0; ip < n_particles; ++ip)
  {
    indx_list[ip] = drawIndex(cumul_weights_update, gen_cooploc);
  }

  std::vector< std::shared_ptr<Particle> > new_generation;
  std::vector<int> indx_unused(n_particles, -1);
  for (int ip = 0; ip < n_particles; ++ip)
  {
    const int prev_indx = indx_unused[indx_list[ip]];
    if (prev_indx == -1)
    {
      new_generation.push_back(std::move(segments.back()[indx_list[ip]]));
      indx_unused[indx_list[ip]] = ip;
    }
    else
    {
      new_generation.push_back(
        std::make_shared<Particle>(*new_generation[prev_indx]));
    }
  }
  segments.back().swap(new_generation);
  segments_index_best.back() = index_best;
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::doSegment(const ros::Time& now)
{
  // set the initial pose of the segment to that of the best particle.
  init_segment_pose = segments.back()[segments_index_best.back()]->getPose();
  // set the initial time of the segment to the current one.
  init_segment_time = now;

  // create a new segment
  std::vector< std::shared_ptr<Particle> > new_seg(
    n_particles, nullptr);
  segments_index_best.push_back(segments_index_best.back());
  // copy each resampled particle.
  for (int i = 0; i < n_particles; ++i)
  {
    int indx = i;

    new_seg[i]
      = std::make_shared<Particle>(
          segments.back()[indx],
          resol, probHit, probMiss, threshMin, threshMax);
  }
  // add the segment to the list.
  segments.push_back(new_seg);
  // increment nseg
  ++nseg;

  // set counts to the interval to force to build a map anyway
  counts_map_update = mapping_interval;
}

////////////////////////////////////////////////////////////////////////////////
bool RBPF::isTimeToSegment()
{
  bool do_segment = false;
  tf::Pose best_pose;
  if (enable_segmentation)
  {
    best_pose = segments.back()[segments_index_best.back()]->getPose();
    tf::Pose pose_in_seg = init_segment_pose.inverse() * best_pose;
    if (pose_in_seg.getOrigin().length() > next_seg_thresh)
      do_segment = true;
  }
  return do_segment;
}

////////////////////////////////////////////////////////////////////////////////
bool RBPF::checkEntry(const ros::Time& now)
{
  bool entry_detected = false;
  if (segments.size() > 1)
  {
    // get the location of the previous robot
    if (beacon_buffer.count(next_robot_name)
      && now <= beacon_lasttime[next_robot_name] + beacon_lifetime)
    {
      auto beacon_info = beacon_buffer[next_robot_name];
      tf::Vector3 next_robot_loc_wrt_here(
        beacon_info.estimated_orientation.x
          * beacon_info.estimated_distance,
        beacon_info.estimated_orientation.y
          * beacon_info.estimated_distance,
        beacon_info.estimated_orientation.z
          * beacon_info.estimated_distance);
      tf::Vector3 next_robot_loc
        = segments.back()[segments_index_best.back()]->getPose()
          * next_robot_loc_wrt_here;
      double x = next_robot_loc.getX();
      double y = next_robot_loc.getY();
      double z = next_robot_loc.getZ();

      // get the range of the oldest submap
      auto map = segments.front()[segments_index_best.front()]->getMap();
      double min_x, min_y, min_z;
      map->getMetricMin(min_x, min_y, min_z);
      double max_x, max_y, max_z;
      map->getMetricMax(max_x, max_y, max_z);

      // determine if the location is within the range
      if (min_x <= x && x <= max_x
       && min_y <= y && y <= max_y
       && min_z <= z && z <= max_z)
      {
        entry_detected = true;
      }
    }
  }
  return entry_detected;
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::indivSlamMiscProc(const ros::Time& now)
{
  if (counts_compress >= compress_interval)
  {
    compressMap();
    counts_compress = 0;
  }
  else
  {
    ++counts_compress;
  }

  // publish data
  publishVisPoses(now);

  if (counts_publish >= publish_interval)
  {
    publishCurrentSubMap(now);
    publishTF(now);
    if (save_traj)
    {
      saveTraj();
    }
    counts_publish = 0;
  }
  else
  {
    ++counts_publish;
  }

  // visualization
  if (counts_visualize_map >= vismap_interval)
  {
    publishVisMap(now);
    counts_visualize_map = 0;
  }
  else
  {
    ++counts_visualize_map;
  }

}

////////////////////////////////////////////////////////////////////////////////
void RBPF::publishTF(const ros::Time& now)
{
  tf::StampedTransform tf_stamped(
    segments.back()[segments_index_best.back()]->getPose(), now,
    world_frame_id, robot_frame_id);
  tf_broadcaster.sendTransform(tf_stamped);
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::publishCurrentSubMap(const ros::Time& now)
{
  mav_tunnel_nav::OctomapWithSegId map;
  map.header.frame_id = world_frame_id;
  map.header.stamp = now;
  std::stringstream ss;
  ss << robot_name << "-" << std::setw(3) << std::setfill('0') << nseg;
  map.segid = ss.str();
  if (octomap_msgs::fullMapToMsg(
      *segments.back()[segments_index_best.back()]->getMap(), map.octomap))
    map_pub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::saveTraj()
{
  std::fstream
    fin(traj_filename, std::fstream::out | std::fstream::app);
  if (!fin)
  {
    ROS_ERROR_STREAM("FILE NOT OPEN: " << traj_filename);
  }
  else
  {
    // save ground truth x, y, z
    tf::StampedTransform ground_truth_tf;
    try
    {
      tf_listener->waitForTransform(
        world_frame_id, robot_name + "_groundtruth",
        ros::Time(0), ros::Duration(1));
      tf_listener->lookupTransform(
        world_frame_id, robot_name + "_groundtruth",
        ros::Time(0), ground_truth_tf);
      tf::Vector3 loc = ground_truth_tf.getOrigin();
      fin << loc.x() << " "
          << loc.y() << " "
          << loc.z() << " ";
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_STREAM(
        "Transfrom from " << robot_name + "_groundtruth" <<
        " to " << world_frame_id << " is not available yet.");
      fin << "0 0 0 ";
    }

    double x, y, z;
    x = 0;
    y = 0;
    z = 0;
    for (int i = 0; i < n_particles; ++i)
    {
      tf::Vector3 buff = segments.back()[i]->getPose().getOrigin();
      x += buff.x()/n_particles;
      y += buff.y()/n_particles;
      z += buff.z()/n_particles;
    }
    tf::Vector3 average_loc(x, y, z);
    // save estimated loc x, y, z
    fin << average_loc.x() << " "
        << average_loc.y() << " "
        << average_loc.z() << std::endl;
    fin.close();
  }
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::publishVisMap(const ros::Time& now)
{
  const unsigned int index_offset = nseg - segments.size() + 1;
  for (unsigned int isgm = index_offset; isgm <= nseg; ++isgm)
  {
    const unsigned int index = isgm - index_offset;
    const octomap::OcTree* m
      = segments[index][segments_index_best[index]]->getMap();
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

        std_msgs::ColorRGBA clr;
        clr.a = 1.0;
        double cosR;
        double cosG;
        double cosB;

        if (enable_clr4seg)
        {
          double val = 0.8 * isgm;
          cosR = std::cos(M_PI*val);
          cosG = std::cos(M_PI*(2.0/3.0+val));
          cosB = std::cos(M_PI*(4.0/3.0+val));
        }

        if (m->isNodeOccupied(*it))
        {
          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

          if (enable_clr4seg)
          {
            double brightness = -z/20.0;
            while (brightness < 0)
              brightness += 1.0;
            while (brightness > 1.0)
              brightness -= 1.0;
            if (brightness >= 0.5)
              brightness = (brightness - 0.5)*2.0;
            else
              brightness = -(brightness - 0.5)*2.0;
            brightness = brightness * 0.8 + 0.2;
            clr.r = (cosR > 0)? cosR * brightness: 0;
            clr.g = (cosG > 0)? cosG * brightness: 0;
            clr.b = (cosB > 0)? cosB * brightness: 0;
          }
          else
          {
            double brightness = (isgm + 1.0)/(nseg + 1.0);
            cosR = std::cos(M_PI*z/10.0)*0.8+0.2;
            cosG = std::cos(M_PI*(2.0/3.0+z/10.0))*0.8+0.2;
            cosB = std::cos(M_PI*(4.0/3.0+z/10.0))*0.8+0.2;
            clr.r = (cosR > 0)? cosR * brightness: 0;
            clr.g = (cosG > 0)? cosG * brightness: 0;
            clr.b = (cosB > 0)? cosB * brightness: 0;
          }

          occupiedNodesVis.markers[idx].colors.push_back(clr);
        }
      }
    }
    // std_msgs::ColorRGBA m_color_occupied;
    // m_color_occupied.r = 1;
    // m_color_occupied.g = 1;
    // m_color_occupied.b = 0.3;
    // m_color_occupied.a = 0.5;
    for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i)
    {
      double size = m->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = "world";
      occupiedNodesVis.markers[i].header.stamp = now;
      occupiedNodesVis.markers[i].ns
        = robot_name + "-" + std::to_string(isgm);
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
  }
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::publishVisPoses(const ros::Time& now)
{
  // publish poses
  geometry_msgs::PoseArray poseArray;
  poseArray.header.frame_id = world_frame_id;
  poseArray.header.stamp = now;
  poseArray.poses.resize(n_particles);
  for (int i = 0; i < n_particles; ++i)
  {
    tf::Pose pose = segments.back()[i]->getPose();
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
}

////////////////////////////////////////////////////////////////////////////////
void RBPF::pf_main()
{
  last_update = ros::Time::now();

  // wait for the tf of initial ground truth to be published
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    if (now > last_update + update_phase)
    {
      // initialize the time step
      last_update = now;
      tf::StampedTransform ground_truth_tf;
      try
      {
        tf_listener->waitForTransform(
          world_frame_id, robot_name + "_groundtruth",
          ros::Time(0), ros::Duration(1));
        tf_listener->lookupTransform(
          world_frame_id, robot_name + "_groundtruth",
          ros::Time(0), ground_truth_tf);

        break;
      }
      catch (tf::TransformException& ex)
      {
        // NOTE: do nothing. Just wait.
      }
    }
  }

  // initialize the estimated pose
  initial_update = ros::Time::now();

  // wait for the robot's body to get stable after being deployed
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    if (now > initial_update + phase_pose_adjust)
    {
      // initialize the time step
      tf::StampedTransform ground_truth_tf;
      try
      {
        tf_listener->waitForTransform(
          world_frame_id, robot_name + "_groundtruth",
          ros::Time(0), ros::Duration(1));
        tf_listener->lookupTransform(
          world_frame_id, robot_name + "_groundtruth",
          ros::Time(0), ground_truth_tf);

        tf::Vector3 position = ground_truth_tf.getOrigin();
        tf::Quaternion orientation = ground_truth_tf.getRotation();
        for (auto p: segments.back())
        {
          p->initPosition(position);
          p->initOrientation(orientation);
        }
        init_segment_pose = ground_truth_tf;
        break;
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR_STREAM(
          "Transfrom from " << robot_name + "_groundtruth" <<
          " to " << world_frame_id << " is not available yet.");
      }
    }
  }

  // Initialization of prev pose
  // Odometry data
  updateCurrPoseOfOdom();
  pose_prev = pose_curr;

  initial_update = ros::Time::now();
  ros::Time last_cooploc = initial_update;
  ros::Time last_syncinit = initial_update;
  last_update = initial_update;

  // the main loop
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();

    if (state == Init)
    {
      // initialize the time step
      last_update = now;

      std::map<std::string, double> range_data;
      getRanges(range_data);
      octomap::Pointcloud octocloud;
      getPC(octocloud);

      if (now > initial_update + phase_pose_adjust + phase_only_mapping)
      {
        // auto enable: call the ROS service to fly.
        if (auto_enable_by_slam)
        {
          std_srvs::SetBool srv;
          srv.request.data = true;
          srv_client.call(srv);
        }

        state = IndivSLAM;
      }
      else
      {
        // mapping only at the beginning
        if (updateMap(octocloud))
          counts_map_update = 0;
      }

      indivSlamMiscProc(now);
    }
    else if (state == IndivSLAM)
    {
      if (now > last_update + update_phase)
      {
        // initialize the time step
        last_update = now;

        octomap::Pointcloud octocloud;
        getPC(octocloud);

        if (enable_indivLoc)
        {
          std::map<std::string, double> range_data;
          getRanges(range_data);

          // Odometry data
          tf::Transform diff_pose;
          updateCurrPoseOfOdom();
          diff_pose = pose_prev.inverse() * pose_curr;
          pose_prev = pose_curr;

          // predict PF (use odometory)
          indivSlamPredict(diff_pose);

          // weight PF (use depth cam)
          indivSlamEvaluate(now, range_data, octocloud);

          indivSlamResample();
        }

        // if far away from the init position of the segment
        if (isTimeToSegment())
        {
          doSegment(now);
          if (updateMap(octocloud))
            counts_map_update = 0;
        }
        else
        {
          if (counts_map_update >= mapping_interval)
          {
            if (updateMap(octocloud))
              counts_map_update = 0;
          }
          else
          {
            ++counts_map_update;
          }
        }

        indivSlamMiscProc(now);
      }
      else
      {
        // NOTE: check if the previous robot is in the oldest map
        if (checkEntry(now))
        {
          // TODO: initiate map_transfer
          ROS_ERROR("HIT!!!!");
        }

        if (enable_cooploc)
        {
          if (now < last_cooploc + cooploc_phase)
          {
            std::string syncreq_src = checkSyncReq(now);
            if (syncreq_src.size() > 0)
            {
              last_sync_src = syncreq_src;
              state = SyncReact;
              last_syncinit = now;
            }
          }
          else
          {
            if (initiateSync(now))
            {
              last_syncinit = now;
              state = SyncInit;
            }
          }
        }
      }
    }
    else if (state == SyncInit)
    {
      // NOTE: If timed out, it will switch back to IndivSLAM. Otherwise, the
      //       state is switched to DataSending by the callback once the data
      //       from the other robot is received.
      if (now >= last_syncinit + syncinit_timeout)
      {
        state = IndivSLAM;
      }
    }
    else if (state == SyncReact)
    {
      exchangeData();
      state = DataWaiting;
    }
    else if (state == DataSending)
    {
      exchangeData();
      state = Update;
    }
    else if (state == DataWaiting)
    {
      // NOTE: DataWaiting: do nothing in the main loop.
      //       the state is switched to Update by the callback once the
      //       data from the other robot is received.
      //
      //       If it takes too long, there must be some problem.
      //       In such a case, print out an error and returns to the initial
      //       state.
      if (now >= last_syncinit + syncinit_timeout)
      {
        ROS_ERROR_STREAM("" << robot_name << ": timeout in DataWaiting");
        state = IndivSLAM;
      }
    }
    else if (state == Update)
    {
      cooplocUpdate();

      // publish the location
      publishTF(now);
      publishVisPoses(now);

      last_cooploc = now;
      state = IndivSLAM;
    }
    else
    {
      // should not be here
      ROS_ERROR_STREAM("Invalid State: " << state);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rbpf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::shared_ptr<RBPF> rbpf = std::make_shared<RBPF>(nh, pnh);

  std::thread t(&RBPF::pf_main, rbpf.get());

  ros::spin();

  t.join();

  return(0);
}
