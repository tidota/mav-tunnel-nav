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
  enum INTERACT_STATE
    { Init, IndivSLAM, SyncInit, DataSending, SyncReact, DataWaiting, Update };
  INTERACT_STATE state;

  std::string robot_name;
  double comm_range;

  std::string world_frame_id;
  std::string robot_frame_id;

  ros::Subscriber odom_sub;
  std::mutex odom_mutex;
  std::string odom_topic;
  // std::string odom_reset_topic;
  nav_msgs::Odometry odom_buff;
  tf::Pose pose_prev;
  tf::Pose pose_curr;

  ros::Subscriber pc_sub;
  std::mutex pc_mutex;
  std::string pc_topic;
  sensor_msgs::PointCloud2 pc_buff;
  PointCloudT::Ptr depth_cam_pc;
  tf::Pose camera_pose;

  std::vector<ros::Subscriber> range_subs;
  std::mutex range_mutex;
  std::vector<std::string> range_topics;
  std::map<std::string, tf::Pose> range_poses;
  std::map<std::string, double> range_buff;
  double range_max, range_min;

  ros::Subscriber beacon_sub;
  std::mutex beacon_mutex;
  std::map<std::string, mav_tunnel_nav::Beacon> beacon_buffer;
  std::map<std::string, ros::Time> beacon_lasttime;

  ros::Publisher sync_pub;
  ros::Subscriber sync_sub;
  std::mutex sync_mutex;
  std::deque<mav_tunnel_nav::SrcDst> sync_msgs_buffer;
  std::string last_sync_src;
  mav_tunnel_nav::SrcDst sync_msg;

  ros::Publisher data_pub;
  ros::Subscriber data_sub;
  std::mutex data_mutex;
  std::map<std::string, mav_tunnel_nav::Particles> data_buffer;
  std::map<std::string, ros::Time> data_lasttime;
  std::string last_data_src;
  mav_tunnel_nav::Particles data_msg;

  std::shared_ptr<tf::TransformListener> tf_listener;

  ros::Publisher odom_reset_pub;
  ros::Publisher map_pub;
  ros::Publisher marker_occupied_pub;
  ros::Publisher vis_poses_pub;
  tf::TransformBroadcaster tf_broadcaster;
  bool save_traj;
  std::string traj_filename;

  int n_particles;

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
  int compress_interval;

  int counts_publish;
  int counts_visualize_map;
  int counts_visualize_loc;
  int counts_map_update;
  int counts_compress;
  // int counts_locdata = 0;

  bool enable_indivLoc;
  ros::Duration update_phase;
  std::mt19937 gen_indivloc;

  std::vector<double> cumul_weights_slam;
  std::vector<double> errors;

  // segments
  bool enable_segmentation;
  std::deque< std::vector< std::shared_ptr<Particle> > > segments;
  unsigned int nseg;
  tf::Pose init_segment_pose;
  ros::Time init_segment_time;
  std::string next_robot_name;
  ros::Duration init_seg_phase;
  double next_seg_thresh;
  bool enable_clr4seg;

  // parameters for cooperative localization
  int Nref;
  int seed_cooploc;
  bool enable_cooploc;
  bool enable_conservative;
  double conserv_omega;
  double sigma_kde;
  std::mt19937 gen_cooploc;
  std::default_random_engine gen_cooploc_select;

  // For data exchange.
  double sigma_kde_squared_x2;
  std::vector<double> cumul_weights;
  std::vector<double> cumul_weights_comp;

  ros::Time last_update;
  ros::Time initial_update;
  ros::Duration beacon_lifetime;
  ros::Duration cooploc_phase;
  ros::Duration syncinit_timeout;

  double sigmaLocR;
  double sigmaLocT;
  // parameters for evaluation
  double gl_eval_cons;
  double ml_eval_cons;

  // auto enable: call the ROS service to fly.
  bool auto_enable_by_slam;
  ros::ServiceClient srv_client;

public:
  RBPF(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void pf_main();

private:
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

  void updateCurrPoseOfOdom();
  void getRanges(std::map<std::string, double>& range_data);
  void getPC(octomap::Pointcloud& octocloud);

  void indivSlamPredict(const tf::Transform& diff_pose);
  void indivSlamEvaluate(
    const ros::Time& now,
    const std::map<std::string, double>& range_data,
    const octomap::Pointcloud& octocloud);
  void indivSlamResample();

  bool updateMap(const octomap::Pointcloud& octocloud);
  void compressMap();

  std::string checkSyncReq(const ros::Time& now);
  bool initiateSync(const ros::Time& now);
  void exchangeData();
  void cooplocUpdate();

  void doSegment(const ros::Time& now);
  bool isTimeToSegment();
  bool checkEntry(const ros::Time& now);

  void indivSlamMiscProc(const ros::Time& now);

  void publishTF(const ros::Time& now);
  void publishCurrentSubMap(const ros::Time& now);

  void saveTraj();

  void publishVisMap(const ros::Time& now);
  void publishVisPoses(const ros::Time& now);
};

#endif
