#ifndef ADHOCNETPLUGIN_HH_
#define ADHOCNETPLUGIN_HH_

#include <memory>
#include <mutex>
#include <queue>
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

    // /// \brief Checks all robots are ready to fly
    // public: void CheckRobotsReadyTh();
    //
    // /// \brief Callback executed when a new request is received.
    // /// \param[in] _req The datagram/packet contained in the request.
    // private: void OnMessage(
    //   const boost::shared_ptr<adhoc::msgs::Datagram const> &_req);
    //
    // /// \brief Callback to receive a response from a client.
    // /// \param[in] _res A message containing simulation statistics from the
    // ///                 model plugin.
    // public: void OnSimCmdResponse(
    //     const boost::shared_ptr<adhoc::msgs::SimInfo const> &_res);
    //
    // /// \brief Make a hash string based on the message.
    // /// \param[in] _msg A message to calculate a hash value.
    // /// \param[out] _hash a hash value of the message.
    // private: void CalcHash(
    //   const adhoc::msgs::Datagram &_msg, unsigned char *_hash);
    //
    // /// \brief Check if the given hash value is already registered.
    // /// \param[in] _hash A hash value to check.
    // /// \return True if the hash value has already been registered. Otherwise,
    // ///         False.
    // private: bool HasHash(const unsigned char *_hash);
    //
    // /// \brief Register a hash value.
    // /// \param[in] _hash A hash value to register.
    // private: void RegistHash(const unsigned char *_hash);
    //
    // /// \brief Check if the net topology changed.
    // private: int CheckTopoChange();
    //
    // /// \brief Initialize the simulation status.
    // private: void StartNewTrial();

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    // /// \brief Thread object to check if robots are ready to fly.
    // private: std::thread robotCheckThread;

    // /// \brief An Ignition Transport node for communications.
    // private: transport::NodePtr node;

    // /// \brief subscriber map to receive data.
    // private: std::map< std::string, transport::SubscriberPtr > subMap;
    //
    // /// \brief publisher map to send data.
    // private: std::map< std::string, transport::PublisherPtr > pubMap;
    //
    // /// \brief Subscriber for simulation command responses.
    // private: transport::SubscriberPtr simCmdResSub;
    //
    // /// \brief Publisher for simulation command.
    // private: transport::PublisherPtr simCmdPub;
    //
    // /// \brief List of received start responses.
    // private: std::vector<boost::shared_ptr<adhoc::msgs::SimInfo const>>
    //   listStartResponses;
    //
    // /// \brief List of received stop responses.
    // private: std::vector<boost::shared_ptr<adhoc::msgs::SimInfo const>>
    //   listStopResponses;

    /// \brief ROS node handler
    private: ros::NodeHandle nh;

    // subscribers and publishers for beacon
    private: std::map<std::string, ros::Subscriber> beacon_subs;
    private: std::map<std::string, ros::Publisher> beacon_pubs;

    // subscribers and publishers for synchronization of exchange
    private: std::map<std::string, ros::Subscriber> sync_subs;
    private: std::map<std::string, ros::Publisher> sync_pubs;

    // subscribers and publishers for data exchange
    private: std::map<std::string, ros::Subscriber> data_subs;
    private: std::map<std::string, ros::Publisher> data_pubs;

    /// \brief list of robot names.
    private: std::vector<std::string> robotList;

    // /// \brief list of maps from robot's name to its initial pose.
    // private: std::map<std::string, POSE> initPoseList;
    //
    // /// \brief list of simulation settings.
    // private: std::queue<std::string> settingList;
    //
    // /// \brief Simulation start time.
    // private: common::Time startTime;
    //
    // /// \brief Time when the last packet was received.
    // private: common::Time lastRecvTime;
    //
    // /// \brief the last time to print the robot positions on the terminal.
    // private: common::Time lastStatPrintTime;
    //
    // /// \brief delay time currently used.
    // private: double currentDelayTime;
    //
    // /// \brief robot speed currently used.
    // private: double currentRobotSpeed;
    //
    // /// \brief Communciation range.
    // private: double commRange;
    //
    // /// \brief Period to run the communication (in seconds)
    // private: double simPeriod;
    //
    // /// \brief True if the all robots are flying at the specific altitude
    // /// and ready to start netowrking.
    // private: bool robotsReadyToComm;
    //
    // /// \brief True if the communication started.
    // private: bool started;
    //
    // /// \brief True if they stopped to create new messages.
    // private: bool stopping;
    //
    // /// \brief True if the communicaiton finished.
    // private: bool finished;
    //
    // /// \brief list of hash values
    // private: std::unordered_set<std::string> hashSet;
    //
    // /// \brief List of connections
    // private: std::map<std::string, bool> topoList;
    //
    // /// \brief Total number of packets processed.
    // private: int totalRecvPackets;
    //
    // /// \brief Total number of packets submitted.
    // private: int totalSentPackets;
    //
    // /// \brief # of topology changes
    // private: int topoChangeCount;
    //
    // /// \brief Mutex for communication data.
    // /// This mutex for items updated by OnUpdate and OnMessage.
    // private: std::mutex messageMutex;
    //
    // /// \brief Mutex for the simulation information from clients.
    // /// This mutex for items updated by OnUPdate and OnSimCmdResponse.
    // private: std::mutex simInfoMutex;
  };
}
#endif
