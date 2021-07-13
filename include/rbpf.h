#ifndef __RBPF_H__
#define __RBPF_H__

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>

#include <mav_tunnel_nav/SrcDst.h>
#include <mav_tunnel_nav/Beacon.h>
#include <mav_tunnel_nav/Particles.h>
#include <mav_tunnel_nav/OctomapWithSegId.h>

#include <ros/ros.h>

// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "particle.h"

class RBPF
{
private:
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
  std::map<std::string, mav_tunnel_nav::Beacon> beacon_buffer;
  std::map<std::string, ros::Time> beacon_lasttime;

  std::mutex sync_mutex;
  std::string last_sync_src;
  std::deque<mav_tunnel_nav::SrcDst> sync_msgs_buffer;

  std::mutex data_mutex;
  std::map<std::string, mav_tunnel_nav::Particles> data_buffer;
  std::map<std::string, ros::Time> data_lasttime;
  std::string last_data_src;
  enum INTERACT_STATE
    { Init, LocalSLAM, SyncInit, DataSending, SyncReact, DataWaiting, Update };
  INTERACT_STATE state;

  ros::Subscriber beacon_sub;
  ros::Subscriber sync_sub;
  ros::Subscriber data_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber pc_sub;
  std::vector<ros::Subscriber> range_subs;

  // =======

  std::string robot_name;
  double comm_range;

  ros::Publisher sync_pub;
  ros::Publisher data_pub;

  std::mt19937 gen_indivloc;

  tf::TransformBroadcaster tf_broadcaster;

  ros::Publisher odom_reset_pub;
  ros::Publisher map_pub;
  ros::Publisher marker_occupied_pub;
  ros::Publisher vis_poses_pub;

  int n_particles;

  ros::Duration update_phase;

  double init_x;
  double init_y;
  double init_z;
  double init_Y;
  double resol;
  double probHit;
  double probMiss;
  double threshMin;
  double threshMax;
  double motion_noise_lin_sigma;
  double motion_noise_rot_sigma;
  double sensor_noise_range_sigma;
  double sensor_noise_depth_sigma;

  ros::Duration phase_pose_adjust;
  ros::Duration phase_only_mapping;

  int mapping_interval;
  int publish_interval;
  int vismap_interval;
  int visloc_interval;
  int compress_interval;

  int counts_publish;
  int counts_visualize_map;
  int counts_visualize_loc;
  int counts_map_update;
  int counts_compress;
  // int counts_locdata = 0;
  bool enable_indivLoc;

  std::deque< std::vector< std::shared_ptr<Particle> > > segments;
  unsigned int nseg;
  std::deque< int > segments_index_best;
  std::vector<double> cumul_weights_slam;
  std::vector<double> errors;

  tf::Pose init_segment_pose;
  ros::Time init_segment_time;

  std::string next_robot_name;

  tf::Pose pose_prev;
  tf::Pose pose_curr;

  PointCloudT::Ptr depth_cam_pc;

  tf::Pose camera_pose;

  std::shared_ptr<tf::TransformListener> tf_listener;

  bool save_traj;
  std::string traj_filename;

  ros::Time last_update;
  ros::Time initial_update;

  // parameters for cooperative localization
  int Nref;
  int seed_cooploc;
  bool enable_cooploc;
  bool enable_conservative;

  double conserv_omega;
  double sigma_kde;

  double sigmaLocR;
  double sigmaLocT;
  // parameters for evaluation
  double gl_eval_cons;
  double ml_eval_cons;

  ros::Duration beacon_lifetime;
  ros::Duration cooploc_phase;
  ros::Duration syncinit_timeout;


  std::mt19937 gen_cooploc;
  std::default_random_engine gen_cooploc_select;


  bool enable_segmentation;
  ros::Duration init_seg_phase;
  double next_seg_thresh;
  bool enable_clr4seg;

  // === For data exchange. ==
  double sigma_kde_squared_x2;
  mav_tunnel_nav::Particles data_msg;
  std::vector<double> cumul_weights;
  std::vector<double> cumul_weights_comp;

  // For synchronization
  mav_tunnel_nav::SrcDst sync_msg;

  // auto enable: call the ROS service to fly.
  bool auto_enable_by_slam;
  ros::ServiceClient srv_client;

public:
  RBPF(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  void beaconCallback(const mav_tunnel_nav::Beacon::ConstPtr& msg);
  void syncCallback(const mav_tunnel_nav::SrcDst::ConstPtr& msg);
  void dataCallback(const mav_tunnel_nav::Particles::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void rangeCallback(const sensor_msgs::Range::ConstPtr& new_range);

  int drawIndex(
    const std::vector<double>& cumul_weights, std::mt19937& gen);
  void prepareDataMsg(
    mav_tunnel_nav::Particles& data_msg, const std::string& destination,
    const std::vector< std::shared_ptr<Particle> >& particles);

  void pf_main();
};

#endif
