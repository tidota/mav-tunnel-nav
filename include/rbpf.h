#ifndef __RBPF_H__
#define __RBPF_H__

#include <memory>

#include <octomap/octomap.h>
#include <tf/transform_datatypes.h>

class Particle
{
  // pose
  private: tf::Pose pose;
  // map
  private: octomap::OcTree *map;

  public: Particle(const double &resol,
    const double &probHit, const double &probMiss,
    const double &threshMin, const double &threshMax);
  public: Particle();
  public: ~Particle();
};

#endif
