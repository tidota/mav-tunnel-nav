#ifndef ADHOCNETPLUGIN_HH_
#define ADHOCNETPLUGIN_HH_

#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <thread>
#include <unordered_set>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>

#include <sim_plugins/CommonTypes.hh>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// NOTE: visualization
#include <visualization_msgs/Marker.h>

#include <mav_tunnel_nav/SrcDst.h>
#include <mav_tunnel_nav/Beacon.h>
#include <mav_tunnel_nav/Particles.h>
#include <mav_tunnel_nav/Submap.h>
#include <mav_tunnel_nav/SubmapAck.h>

namespace adhoc
{
  namespace msgs
  {
    // Forward declarations.
    class Datagram;
  }
}

namespace gazebo
{
  /// \brief A plugin that controls ad hoc communications.
  class AdHocNetPlugin : public WorldPlugin
  {
    /// \breif Constructor.
    public: AdHocNetPlugin()
    {}

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Callback executed when a new beacon is received.
    /// \param[in] msg The beacon message.
    private: void OnBeaconMsg(const mav_tunnel_nav::Beacon::ConstPtr& msg);

    /// \brief Callback executed when a new sync message is received.
    /// \param[in] msg The synchronization message.
    private: void OnSyncMsg(const mav_tunnel_nav::SrcDst::ConstPtr& msg);

    /// \brief Callback executed when new particle data is received.
    /// \param[in] msg The particle message.
    private: void OnDataMsg(const mav_tunnel_nav::Particles::ConstPtr& msg);

    /// \brief Callback executed when new submap data is received.
    /// \param[in] msg The submap message.
    private: void OnSubmapMsg(const mav_tunnel_nav::Submap::ConstPtr& msg);

    /// \brief Callback executed when new submap ack data is received.
    /// \param[in] msg The submap ack message.
    private:
      void OnSubmapAckMsg(const mav_tunnel_nav::SubmapAck::ConstPtr& msg);

    /// \brief Helper function to check the line of sight condition of the given
    ///        two robots.
    /// \param[in] start The pose on which the line starts.
    /// \param[in] end The pose on which the line ends.
    /// \return True if there is no obstacle between the points.
    private: inline bool CheckLineOfSight(
      const ignition::math::Vector3d& start,
      const ignition::math::Vector3d& end);

    /// \brief Helper function to check the two robots are in the communication
    ///        range. (obstacles between them are not checked)
    /// \param[in] robot1 the first robot.
    /// \param[in] robot2 the second robot.
    /// \return The distance between the robots.
    // private: inline double CheckRange(
    //   const physics::ModelPtr& robot1, const physics::ModelPtr& robot2);

    // /// \brief Helper function to check if two robots can perform local
    // ///        communication. It checks the topology information updated by
    // ///        OnUpdate callback.
    // /// \param[in] robot_name1 the first robot's name
    // /// \param[in] robot_name2 the second robot's name
    // /// \return True if they can communicate locally.
    // private: inline bool CheckTopology(
    //   const std::string& robot_name1, const std::string& robot_name2);

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief ROS node handler
    private: ros::NodeHandle nh;

    /// \brief TF broadcaster (for ground truth trajectory)
    private: tf::TransformBroadcaster tf_broadcaster;

    /// \brief subscribers for beacon
    private: std::map<std::string, ros::Subscriber> beacon_subs;
    /// \brief publishers for beacon
    private: std::map<std::string, ros::Publisher> beacon_pubs;
    // /// \brief buffer for beacon
    // private:
    //   std::vector< std::shared_ptr<mav_tunnel_nav::Beacon> > beacon_buff;
    // /// \brief mutex for beacon
    // private: std::mutex beacon_mutex;

    /// \brief subscribers for synchronization of exchange
    private: std::map<std::string, ros::Subscriber> sync_subs;
    /// \brief publishers for synchronization of exchange
    private: std::map<std::string, ros::Publisher> sync_pubs;
    // /// \brief buffer for sync
    // private:
    //   std::vector< std::shared_ptr<mav_tunnel_nav::SrcDst> > sync_buff;
    // /// \brief mutex for sync
    // private: std::mutex beacon_sync;

    /// \brief subscribers for data exchange
    private: std::map<std::string, ros::Subscriber> data_subs;
    /// \brief publishers for data exchange
    private: std::map<std::string, ros::Publisher> data_pubs;
    // /// \brief buffer for data
    // private:
    //   std::vector< std::shared_ptr<mav_tunnel_nav::ParticlesMsg> > data_buff;
    // /// \brief mutex for data
    // private: std::mutex beacon_data;

    /// \brief subscribers for submap
    private: std::map<std::string, ros::Subscriber> submap_subs;
    /// \brief publishers for submap
    private: std::map<std::string, ros::Publisher> submap_pubs;

    /// \brief subscribers for submap_ack
    private: std::map<std::string, ros::Subscriber> submap_ack_subs;
    /// \brief publishers for submap_ack
    private: std::map<std::string, ros::Publisher> submap_ack_pubs;

    /// \brief the type of auto pilot
    private: std::string auto_pilot_type;

    /// \brief list of robot names.
    private: std::vector<std::string> robotList;

    /// \brief list of spawned robots.
    private: std::vector<std::string> spawnedList;

    /// \brief If the last spawned robot reached this distance from the base,
    ///        spawn a new robot.
    private: double spawnDist;

    /// \brief If true, the ROS service has been called to spawn a new robot and
    ///        it is waiting for the robot to be spawned.
    private: bool spawning;

    /// \brief The robot initial location when it is spawn.
    private: tf::Vector3 initLoc;

    /// \brief The robot initial orientation (Yaw) when it is spawn.
    private: double initOri;

    /// \brief ROS service client to spawn a robot.
    private: ros::ServiceClient robot_spawner_client;

    /// \brief communication range
    private: double comm_range;

    /// \brief pseudo random number generator.
    private: std::mt19937 gen;
    private: double sigmaDst;
    private: double sigmaOri;

    /// \brief shape to check line-of-sight condition.
    private: physics::RayShapePtr line_of_sight;

    /// \brief the name of the terrain
    private: std::string terrainName;

    private: ros::Duration topo_interval;
    private: ros::Time current;
    private: ros::Time last_update;

    /// \brief the base lander information
    private: std::string base_name;
    private: ignition::math::Pose3d base_pose;

    /// \brief topology information
    private: std::map<std::string, bool> topoInfo;
    private: inline void setTopoInfo(
      const std::string& str1, const std::string& str2, const bool& flag)
    {
      if (str1 < str2)
        topoInfo[str1 + str2] = flag;
      else
        topoInfo[str2 + str1] = flag;
    }
    private: inline bool getTopoInfo(
      const std::string& str1, const std::string& str2)
    {
      bool flag;
      if (str1 < str2)
        flag = topoInfo[str1 + str2];
      else
        flag = topoInfo[str2 + str1];
      return flag;
    }
    /// \brief relative pose information
    private: std::map<std::string, ignition::math::Pose3d> poseInfo;
    private: inline void setPoseInfo(
      const std::string& str1, const std::string& str2,
      const ignition::math::Pose3d& pose)
    {
      if (str1 < str2)
        poseInfo[str1 + str2] = pose;
      else
        poseInfo[str2 + str1] = pose.Inverse();
    }
    private: inline ignition::math::Pose3d getPoseInfo(
      const std::string& str1, const std::string& str2)
    {
      ignition::math::Pose3d pose;
      if (str1 < str2)
        pose = poseInfo[str1 + str2];
      else
        pose = poseInfo[str2 + str1].Inverse();
      return pose;
    }

    /// \brief marker for visualization
    /// NOTE: visualization
    private: bool enable_vis_cooploc;
    private: ros::Publisher marker_pub;
  };
}
#endif
