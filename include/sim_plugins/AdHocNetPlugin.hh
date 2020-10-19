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

#include <tf/transform_datatypes.h>

#include <mav_tunnel_nav/SrcDstMsg.h>
#include <mav_tunnel_nav/Particles.h>

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
    /// \breif. Constructor.
    public: AdHocNetPlugin()
    {}

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Callback executed when a new beacon is received.
    /// \param[in] msg The beacon message.
    private: void OnBeaconMsg(const mav_tunnel_nav::SrcDstMsg::ConstPtr& msg);

    /// \brief Callback executed when a new sync message is received.
    /// \param[in] msg The synchronization message.
    private: void OnSyncMsg(const mav_tunnel_nav::SrcDstMsg::ConstPtr& msg);

    /// \brief Callback executed when new particle data is received.
    /// \param[in] msg The particle message.
    private: void OnDataMsg(const mav_tunnel_nav::Particles::ConstPtr& msg);

    /// \brief Helper function to check the line of sight condition of the given
    ///        two robots.
    /// \param[in] robot1 the first robot.
    /// \param[in] robot2 the second robot.
    /// \return True if there is no obstacle between the points.
    private: inline bool CheckLineOfSight(
      const physics::ModelPtr& robot1, const physics::ModelPtr& robot2);

    /// \brief Helper function to check the two robots are in the communication
    ///        range. (obstacles between them are not checked)
    /// \param[in] robot1 the first robot.
    /// \param[in] robot2 the second robot.
    /// \return The distance between the robots.
    private: inline double CheckRange(
      const physics::ModelPtr& robot1, const physics::ModelPtr& robot2);

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

    /// \brief subscribers for beacon
    private: std::map<std::string, ros::Subscriber> beacon_subs;
    /// \brief publishers for beacon
    private: std::map<std::string, ros::Publisher> beacon_pubs;
    // /// \brief buffer for beacon
    // private:
    //   std::vector< std::shared_ptr<mav_tunnel_nav::SrcDstMsg> > beacon_buff;
    // /// \brief mutex for beacon
    // private: std::mutex beacon_mutex;

    /// \brief subscribers for synchronization of exchange
    private: std::map<std::string, ros::Subscriber> sync_subs;
    /// \brief publishers for synchronization of exchange
    private: std::map<std::string, ros::Publisher> sync_pubs;
    // /// \brief buffer for sync
    // private:
    //   std::vector< std::shared_ptr<mav_tunnel_nav::SrcDstMsg> > sync_buff;
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

    /// \brief list of robot names.
    private: std::vector<std::string> robotList;

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
    /// \brief distance information
    private: std::map<std::string, double> distInfo;
    private: inline void setDistInfo(
      const std::string& str1, const std::string& str2, const double& dist)
    {
      if (str1 < str2)
        distInfo[str1 + str2] = dist;
      else
        distInfo[str2 + str1] = dist;
    }
    private: inline double getDistInfo(
      const std::string& str1, const std::string& str2)
    {
      double dist = -1;
      if (str1 < str2)
        dist = distInfo[str1 + str2];
      else
        dist = distInfo[str2 + str1];
      return dist;
    }
  };
}
#endif
