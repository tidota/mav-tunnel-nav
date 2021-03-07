#ifndef __RBPF_H__
#define __RBPF_H__

#include <memory>
#include <random>

#include <octomap/octomap.h>
#include <tf/transform_datatypes.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Particle
{
  // pose
  private: tf::Pose pose;
  public: const tf::Pose getPose();
  // velocities
  // private: tf::Vector3 vel_linear;
  // public: const tf::Vector3 getVel();
  // map
  private: octomap::OcTree *map;
  public: const octomap::OcTree* getMap();

  // sigma for motion noise.
  private: const double motion_noise_lin_sigma;
  private: const double motion_noise_rot_sigma;

  private: std::shared_ptr<Particle> prev;

  public: Particle(
    const double &init_x, const double &init_y, const double &init_z,
    const double &init_Y, const double &resol,
    const double &probHit, const double &probMiss,
    const double &threshMin, const double &threshMax,
    const double &new_motion_noise_lin_sigma,
    const double &new_motion_noise_rot_sigma,
    const std::shared_ptr<Particle>& newPrev = nullptr);
  public: Particle(const Particle &src);
  public: Particle();
  public: ~Particle();
  public: void initPosition(const tf::Vector3 &position);
  public: void initOrientation(const tf::Quaternion &orientation);
  public: void predict(
    const tf::Vector3 &delta_pos, const tf::Quaternion &delta_rot,
    //const double &deltaT,
    std::mt19937 &gen);
  public: double evaluate(
    const std::map<std::string, double> &range_data,
    const octomap::Pointcloud &scan, const bool use_prev = false);
  public: void update_map(const octomap::Pointcloud &scan);
  public: void compress_map();
};

#endif
