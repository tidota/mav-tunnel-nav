#include <algorithm>
#include <fstream>
#include <random>
#include <sstream>
#include <thread>

// #include <dynamic_reconfigure/Reconfigure.h>
// #include <openssl/sha.h>

#include <ignition/math.hh>

#include <ros/master.h>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <sim_plugins/AdHocNetPlugin.hh>

#include <tf/transform_datatypes.h>

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(AdHocNetPlugin)

/////////////////////////////////////////////////
void AdHocNetPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  gzmsg << "Starting Ad Hoc Net server" << std::endl;

  GZ_ASSERT(_world, "AdHocNetPlugin world pointer is NULL");
  this->world = _world;

  this->nh.param("robots", this->robotList, this->robotList);

  this->nh.param("comm_range", this->comm_range, this->comm_range);

  this->nh.param("sigmaDst", this->sigmaDst, this->sigmaDst);
  this->nh.param("sigmaOri", this->sigmaOri, this->sigmaOri);
  this->gen.seed(1000);

  std::string beacon_up_topic;
  this->nh.param(
    "beacon_up_topic", beacon_up_topic, beacon_up_topic);
  std::string beacon_down_topic;
  this->nh.param(
    "beacon_down_topic", beacon_down_topic, beacon_down_topic);
  std::string sync_up_topic;
  this->nh.param(
    "sync_up_topic", sync_up_topic, sync_up_topic);
  std::string sync_down_topic;
  this->nh.param(
    "sync_down_topic", sync_down_topic, sync_down_topic);
  std::string data_up_topic;
  this->nh.param(
    "data_up_topic", data_up_topic, data_up_topic);
  std::string data_down_topic;
  this->nh.param(
    "data_down_topic", data_down_topic, data_down_topic);

  for (auto robot: this->robotList)
  {
    this->beacon_subs[robot]
      = this->nh.subscribe(
          "/" + robot + "/" + beacon_up_topic, 1000,
          &AdHocNetPlugin::OnBeaconMsg, this);
    this->beacon_pubs[robot]
      = this->nh.advertise<mav_tunnel_nav::Beacon>(
          "/" + robot + "/" + beacon_down_topic, 1);

    this->sync_subs[robot]
      = this->nh.subscribe(
          "/" + robot + "/" + sync_up_topic, 1000,
          &AdHocNetPlugin::OnSyncMsg, this);
    this->sync_pubs[robot]
      = this->nh.advertise<mav_tunnel_nav::SrcDst>(
          "/" + robot + "/" + sync_down_topic, 1);

    this->data_subs[robot]
      = this->nh.subscribe(
          "/" + robot + "/" + data_up_topic, 1000,
          &AdHocNetPlugin::OnDataMsg, this);
    this->data_pubs[robot]
      = this->nh.advertise<mav_tunnel_nav::Particles>(
          "/" + robot + "/" + data_down_topic, 1);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AdHocNetPlugin::OnUpdate, this));

  this->line_of_sight = boost::dynamic_pointer_cast<physics::RayShape>(
      this->world->Physics()->CreateShape("ray",
      physics::CollisionPtr()));

  this->terrainName = _sdf->Get<std::string>("terrain_name");
  auto base_elem = _sdf->GetElement("base_station");
  this->base_name = base_elem->Get<std::string>("name");
  this->base_pose = ignition::math::Pose3d(
                      base_elem->Get<double>("x"),
                      base_elem->Get<double>("y"),
                      base_elem->Get<double>("z"),
                      0, 0, 0);

  topo_interval = ros::Duration(0.1);
  last_update = ros::Time::now();

  // initialize topology info
  for (unsigned int i = 0; i < this->robotList.size() - 1; ++i)
  {
    for (unsigned int j = i + 1; j < this->robotList.size(); ++j)
    {
      setTopoInfo(this->robotList[i], this->robotList[j], false);
      setPoseInfo(
        this->robotList[i], this->robotList[j], ignition::math::Pose3d());
    }
  }
  for (auto robot_name: this->robotList)
  {
    setTopoInfo(base_name, robot_name, false);
    setPoseInfo(base_name, robot_name, ignition::math::Pose3d());
  }
}

/////////////////////////////////////////////////
void AdHocNetPlugin::OnUpdate()
{
  ros::Time current = ros::Time::now();
  if (current - last_update > topo_interval)
  {
    for (unsigned int i = 0; i < this->robotList.size() - 1; ++i)
    {
      for (unsigned int j = i + 1; j < this->robotList.size(); ++j)
      {
        auto robot1 = this->world->ModelByName(this->robotList[i]);
        auto robot2 = this->world->ModelByName(this->robotList[j]);
        if (robot1 && robot2)
        {
          auto robot1_pose = robot1->WorldPose();
          auto robot2_pose = robot2->WorldPose();
          auto diff = robot2_pose - robot1_pose;
          setPoseInfo(this->robotList[i], this->robotList[j], diff);
          if (diff.Pos().Length() <= this->comm_range)
          {
            setTopoInfo(
              this->robotList[i], this->robotList[j],
              this->CheckLineOfSight(robot1_pose.Pos(), robot2_pose.Pos()));
          }
          else
          {
            setTopoInfo(
              this->robotList[i], this->robotList[j], false);
          }
        }
      }
    }

    // checking intercations with the base station
    for (auto robot_name: this->robotList)
    {
      auto robot = this->world->ModelByName(robot_name);
      if (robot)
      {
        auto diff = robot->WorldPose() - base_pose;
        setPoseInfo(base_name, robot_name, diff);
        if (diff.Pos().Length() <= this->comm_range)
        {
          setTopoInfo(base_name, robot_name,
            this->CheckLineOfSight(base_pose.Pos(), robot->WorldPose().Pos()));
        }
        else
        {
          setTopoInfo(base_name, robot_name, false);
        }
      }
    }

    last_update = current;
  }
}

//////////////////////////////////////////////////
void AdHocNetPlugin::OnBeaconMsg(const mav_tunnel_nav::Beacon::ConstPtr& msg)
{
  // std::lock_guard<std::mutex> lk(this->beacon_mutex);
  // this->beacon_buff(std::make_shared<mav_tunnel_nav::Beacon>(*msg));

  std::normal_distribution<> dst_noise(0, sigmaDst);
  std::normal_distribution<> ori_noise(0, sigmaOri);

  if (this->beacon_subs.count(msg->source) > 0)
  {
    if (msg->destination.size() == 0)
    {
      for (auto robot: this->robotList)
      {
        if (msg->source != robot)
        {
          // if the destination is in the range
          if (getTopoInfo(robot, msg->source))
          {
            // NOTE: a beacon packet provides the relative location of the
            //       sender with respect to the receiver.
            mav_tunnel_nav::Beacon msg2send = *msg;
            auto pos = getPoseInfo(robot, msg->source).Pos();
            msg2send.estimated_distance = pos.Length() + dst_noise(gen);
            pos.Normalize();
            ignition::math::Quaternion<double>
              rot(0, ori_noise(gen), ori_noise(gen)); // noise on pitch/yaw
            pos = rot * pos;
            msg2send.estimated_orientation.x = pos.X();
            msg2send.estimated_orientation.y = pos.Y();
            msg2send.estimated_orientation.z = pos.Z();
            this->beacon_pubs[robot].publish(msg2send);
          }
        }
      }
      // for interactions with the base station
      if (getTopoInfo(msg->source, base_name))
      {
        // the plugin replies to those broadcasting beacon packets directly.
        mav_tunnel_nav::Beacon msg2send = *msg;
        msg2send.source = base_name;
        msg2send.destination = msg->source;
        auto pos = getPoseInfo(msg->source, base_name).Pos();
        msg2send.estimated_distance = pos.Length() + dst_noise(gen);
        pos.Normalize();
        ignition::math::Quaternion<double>
          rot(0, ori_noise(gen), ori_noise(gen)); // noise on pitch/yaw
        pos = rot * pos;
        msg2send.estimated_orientation.x = pos.X();
        msg2send.estimated_orientation.y = pos.Y();
        msg2send.estimated_orientation.z = pos.Z();
        this->beacon_pubs[msg->source].publish(msg2send);
      }
    }
    else if (this->beacon_pubs.count(msg->destination) > 0)
    {
      // if the destination is in the range
      if (getTopoInfo(msg->source, msg->destination))
      {
        mav_tunnel_nav::Beacon msg2send = *msg;
        auto pos = getPoseInfo(msg->destination, msg->source).Pos();
        msg2send.estimated_distance = pos.Length() + dst_noise(gen);
        pos.Normalize();
        ignition::math::Quaternion<double>
          rot(0, ori_noise(gen), ori_noise(gen)); // noise on pitch/yaw
        pos = rot * pos;
        msg2send.estimated_orientation.x = pos.X();
        msg2send.estimated_orientation.y = pos.Y();
        msg2send.estimated_orientation.z = pos.Z();
        this->beacon_pubs[msg->destination].publish(msg2send);
      }
    }
  }
}

//////////////////////////////////////////////////
void AdHocNetPlugin::OnSyncMsg(const mav_tunnel_nav::SrcDst::ConstPtr& msg)
{
  // std::lock_guard<std::mutex> lk(this->sync_mutex);
  // this->sync_buff(std::make_shared<mav_tunnel_nav::SrcDst>(*msg));

  std::normal_distribution<> dst_noise(0, sigmaDst);

  if (this->sync_subs.count(msg->source) > 0)
  {
    if (this->sync_pubs.count(msg->destination) > 0)
    {
      // if the destination is in the range
      if (getTopoInfo(msg->source, msg->destination))
      {
        mav_tunnel_nav::SrcDst msg2send = *msg;
        msg2send.estimated_distance
          = getPoseInfo(msg->source, msg->destination).Pos().Length()
          + dst_noise(gen);
        this->sync_pubs[msg->destination].publish(msg2send);
      }
    }
  }
}

//////////////////////////////////////////////////
void AdHocNetPlugin::OnDataMsg(const mav_tunnel_nav::Particles::ConstPtr& msg)
{
  // std::lock_guard<std::mutex> lk(this->data_mutex);
  // this->data_buff(std::make_shared<mav_tunnel_nav::ParticlesMsg>(*msg));

  std::normal_distribution<> dst_noise(0, sigmaDst);
  std::normal_distribution<> ori_noise(0, sigmaOri);

  if (this->data_subs.count(msg->source) > 0)
  {
    if (this->data_pubs.count(msg->destination) > 0)
    {
      // if the destination is in the range
      if (getTopoInfo(msg->source, msg->destination))
      {
        // auto robot1 = this->world->ModelByName(msg->source);
        // auto robot2 = this->world->ModelByName(msg->destination);

        // NOTE: A particles packet provides a relative pose of the receiver
        //       with respect to the sender.
        mav_tunnel_nav::Particles msg2send = *msg;
        auto pos = getPoseInfo(msg->source, msg->destination).Pos();

        // NOTE: Simple loc w/o orientation. The reference frame is rotated to
        //       be aligned with the global reference frame so that only
        //       the location can be used without the orientation.
        pos = this->world->ModelByName(msg->source)->WorldPose().Rot().Inverse()
              * pos;

        msg2send.estimated_distance = pos.Length() + dst_noise(gen);
        pos.Normalize();
        ignition::math::Quaternion<double>
          rot(0, ori_noise(gen), ori_noise(gen)); // noise on pitch/yaw
        pos = rot * pos;
        msg2send.estimated_orientation.x = pos.X();
        msg2send.estimated_orientation.y = pos.Y();
        msg2send.estimated_orientation.z = pos.Z();
        this->data_pubs[msg->destination].publish(msg2send);
      }
    }
  }
}

//////////////////////////////////////////////////
bool AdHocNetPlugin::CheckLineOfSight(
  const ignition::math::Vector3d& start, const ignition::math::Vector3d& end)
{
  ignition::math::Vector3d diff = end - start;
  diff.Normalize();

  // move start/end for .5 meter toward the other robot
  this->line_of_sight->SetPoints(
    start + diff * 0.5, end - diff * 0.5);
  // check the intersection
  std::string entityName;
  double dist;
  this->line_of_sight->GetIntersection(dist, entityName);

  // empty string means there was no intersecting object
  bool clear = false;
  if (entityName.size() == 0)
    clear = true;

  return clear;
}

//////////////////////////////////////////////////
// double AdHocNetPlugin::CheckRange(
//   const physics::ModelPtr& robot1, const physics::ModelPtr& robot2)
// {
//   ignition::math::Vector3d start = robot1->WorldPose().Pos();
//   ignition::math::Vector3d end = robot2->WorldPose().Pos();
//
//   return (end - start).Length();
// }

// ////////////////////////////////////////////////
// private: inline bool CheckTopology(
//   const std::string& robot_name1, const std::string& robot_name2)
// {
//   // check the topology info
//   // TODO
//   return false;
// }

// //////////////////////////////////////////////////
// void AdHocNetPlugin::CheckRobotsReadyTh()
// {
//   while (ros::ok())
//   {
//     ros::master::V_TopicInfo master_topics;
//     ros::master::getTopics(master_topics);
//
//     bool allReady = true;
//     for (auto name: this->robotList)
//     {
//       int counts = 0;
//       for (auto it = master_topics.begin(); it != master_topics.end(); it++)
//       {
//         const ros::master::TopicInfo& info = *it;
//         if (
//           info.name == "/" + name + "/controller/velocity/x/state" ||
//           info.name == "/" + name + "/controller/velocity/y/state" ||
//           info.name == "/" + name + "/controller/velocity/z/state" ||
//           info.name == "/" + name + "/controller/attitude/roll/state" ||
//           info.name == "/" + name + "/controller/attitude/pitch/state" ||
//           info.name == "/" + name + "/controller/attitude/yawrate/state")
//         {
//           counts++;
//         }
//       }
//       if (counts < 6)
//       {
//         allReady = false;
//       }
//     }
//     if (allReady)
//     {
//       gzmsg << "ALL READY!" << std::endl;
//       for (auto robotName: this->robotList)
//       {
//         this->initPoseList[robotName]
//           = this->world->ModelByName(robotName)->WorldPose();
//       }
//       break;
//     }
//     ros::Duration(0.1).sleep();
//   }
//
//   if (ros::ok())
//   {
//     ros::Duration(2).sleep();
//     std_msgs::Bool start;
//     start.data = true;
//     this->startFlyingPub.publish(start);
//   }
// }
//
// /////////////////////////////////////////////////
// void AdHocNetPlugin::OnMessage(
//   const boost::shared_ptr<adhoc::msgs::Datagram const> &_req)
// {
//   // Once a packet is received, it is instantaneously processed.
//   std::lock_guard<std::mutex> lk(this->messageMutex);
//   this->lastRecvTime = this->world->SimTime();
//
//   physics::ModelPtr sender = this->world->ModelByName(_req->robot_name());
//   this->totalSentPackets++;
//
//   if (sender)
//   {
//     for (int i = 1; i <= 10; ++i)
//     {
//       std::string name = "robot" + std::to_string(i);
//
//       if (name == sender->GetName())
//         continue;
//
//       physics::ModelPtr robot = this->world->ModelByName(name);
//
//       if (robot)
//       {
//         // forward the message if the robot is within the range of the sender.
//         auto diffVec
//           = robot->WorldPose().CoordPositionSub(sender->WorldPose());
//         double length = fabs(diffVec.Length());
//
//         if (length <= this->commRange)
//         {
//           this->pubMap[robot->GetName()]->Publish(*_req);
//           unsigned char hash[SHA256_DIGEST_LENGTH];
//           this->CalcHash(*_req, hash);
//           if (!this->HasHash(hash))
//             this->RegistHash(hash);
//           this->totalRecvPackets++;
//         }
//       }
//     }
//   }
// }
//
// //////////////////////////////////////////////////
// void AdHocNetPlugin::OnSimCmdResponse(
//   const boost::shared_ptr<adhoc::msgs::SimInfo const> &_res)
// {
//   std::lock_guard<std::mutex> lk(this->simInfoMutex);
//
//   if (_res->state() == "started")
//   {
//     gzmsg << "response (started) from " << _res->robot_name() << std::endl;
//
//     if (std::find(
//         this->listStartResponses.begin(), this->listStartResponses.end(), _res)
//           == this->listStartResponses.end())
//       this->listStartResponses.push_back(_res);
//
//     // if all started.
//     if (this->listStartResponses.size() == this->robotList.size())
//     {
//       this->CheckTopoChange();
//       this->startTime = this->world->SimTime();
//       this->lastStatPrintTime = this->world->SimTime();
//       this->started = true;
//
//       // start recording
//       gzmsg << "Network started" << std::endl;
//     }
//   }
//   else if (_res->state() == "stopped")
//   {
//     gzmsg << "response (stopped) from " << _res->robot_name() << std::endl;
//
//     if (std::find(
//         this->listStopResponses.begin(), this->listStopResponses.end(), _res)
//           == this->listStopResponses.end())
//       this->listStopResponses.push_back(_res);
//
//     // if all are done.
//     if (this->listStopResponses.size() == this->robotList.size())
//     {
//       common::Time current = this->world->SimTime();
//       double elapsed = current.Double() - this->startTime.Double();
//
//       int sentMessageCount = 0;
//       int recvMessageCount = 0;
//       int totalHops = 0;
//       double totalRoundTripTime = 0;
//
//       double totalDistComm = 0;
//       double totalDistMotion = 0;
//
//       for (auto res: this->listStopResponses)
//       {
//         sentMessageCount += res->sent_message_count();
//         recvMessageCount += res->recv_message_count();
//         totalHops += res->total_hops();
//         totalRoundTripTime += res->total_round_trip_time();
//
//         totalDistComm += res->total_dist_comm();
//         totalDistMotion += res->total_dist_motion();
//       }
//
//       // finish recording
//       std::stringstream ss;
//       ss << "--- Network ---" << std::endl;
//       ss << "Time," << elapsed << std::endl;
//       ss << "Total # of Sent Packets," << this->totalSentPackets << std::endl;
//       ss << "Total # of Recv Packets," << this->totalRecvPackets << std::endl;
//       ss << "Total # of Message," << this->hashSet.size() << std::endl;
//       ss << "Avg # of Packets per Sent Message,"
//          << ((double)this->totalSentPackets)/this->hashSet.size() << std::endl;
//       ss << "Avg # of Packets per Recv Message,"
//          << ((double)this->totalRecvPackets)/this->hashSet.size() << std::endl;
//       ss << "Total # of Topology Changes,"
//          << this->topoChangeCount << std::endl;
//       ss << "Frequency of Topology Change,"
//          << this->topoChangeCount / elapsed << std::endl;
//
//       ss << "--- Client ---" << std::endl;
//       ss << "Robot Speed," << this->currentRobotSpeed << std::endl;
//       ss << "Time of hop to delay," << this->currentDelayTime << std::endl;
//       ss << "Total # of Sent Messages," << sentMessageCount << std::endl;
//       ss << "Total # of Received Messages," << recvMessageCount << std::endl;
//       ss << "Total # of Hops," << totalHops << std::endl;
//       ss << "Avg # of hops per Message,"
//          << ((double)totalHops)/recvMessageCount << std::endl;
//       ss << "Total Round Trip Time," << totalRoundTripTime << std::endl;
//       ss << "Avg Round Trip Time per Message,"
//          << totalRoundTripTime/recvMessageCount << std::endl;
//       ss << "Total Distance of Communication," << totalDistComm << std::endl;
//       ss << "Avg Distance of Communicaiton Taken by a Packet,"
//          << totalDistComm/recvMessageCount << std::endl;
//       ss << "Total Distance of Motion," << totalDistMotion << std::endl;
//       ss << "Avg Distance of Motion Taken by a Packet,"
//          << totalDistMotion/recvMessageCount << std::endl;
//
//       gzmsg << ss.str();
//
//       std::stringstream filename;
//       filename << "Simulation_d" << this->currentDelayTime
//                << "_s" << this->currentRobotSpeed
//                << "_" << current.FormattedString() << ".csv";
//       std::fstream fs;
//       fs.open(filename.str(), std::fstream::out);
//       fs << ss.str();
//       fs.close();
//
//       // Reset the robot's pose.
//       for (auto robotName: this->robotList)
//       {
//         auto model = this->world->ModelByName(robotName);
//         for (auto link: model->GetLinks())
//         {
//           link->ResetPhysicsStates();
//         }
//         model->SetWorldPose(this->initPoseList[robotName]);
//         //model->Reset();
//       }
//       // Once the flag is set to false, the plugin starts to check the robot's
//       // pose and start the next simulation trial when they reach the specific
//       // altitude.
//       this->robotsReadyToComm = false;
//     }
//   }
// }
//
// //////////////////////////////////////////////////
// void AdHocNetPlugin::CalcHash(
//   const adhoc::msgs::Datagram &_msg, unsigned char *_hash)
// {
//   std::string input
//     = std::to_string(_msg.src_address()) + std::to_string(_msg.dst_address())
//     + std::to_string(_msg.index()) + _msg.data();
//
//   SHA256((unsigned char*)input.c_str(), input.length(), _hash);
// }
//
// //////////////////////////////////////////////////
// bool AdHocNetPlugin::HasHash(const unsigned char *_hash)
// {
//   std::string buff;
//   buff.assign((const char*)_hash, SHA256_DIGEST_LENGTH);
//
//   if (this->hashSet.count(buff) != 0)
//       return true;
//   else
//       return false;
// }
//
// //////////////////////////////////////////////////
// void AdHocNetPlugin::RegistHash(const unsigned char *_hash)
// {
//   std::string str;
//   str.assign((const char*)_hash, SHA256_DIGEST_LENGTH);
//   this->hashSet.insert(str);
// }
//
// //////////////////////////////////////////////////
// int AdHocNetPlugin::CheckTopoChange()
// {
//   int count = 0;
//   for (int i = 1; i <= 9; ++i)
//   {
//     for (int j = i + 1; j <= 10; ++j)
//     {
//       physics::ModelPtr robot1
//         = this->world->ModelByName("robot" + std::to_string(i));
//       physics::ModelPtr robot2
//         = this->world->ModelByName("robot" + std::to_string(j));
//
//       if (!robot1 || !robot2)
//         continue;
//
//       auto diffVec
//         = robot1->WorldPose().CoordPositionSub(robot2->WorldPose());
//       double length = fabs(diffVec.Length());
//
//       bool inRange = (length <= this->commRange);
//       if (inRange != this->topoList[std::to_string(i)+":"+std::to_string(j)])
//       {
//         count++;
//         this->topoList[std::to_string(i)+":"+std::to_string(j)] = inRange;
//         //if (inRange)
//         //  gzdbg << i << ":" << j << ", connected" << std::endl;
//         //else
//         //  gzdbg << i << ":" << j << ", disconnected" << std::endl;
//       }
//     }
//   }
//
//   return count;
// }
//
// //////////////////////////////////////////////////
// void AdHocNetPlugin::StartNewTrial()
// {
//   if (this->settingList.size() > 0)
//   {
//     gzmsg << "StartNewTrial" << std::endl;
//     std::string settingName = this->settingList.front();
//     this->settingList.pop();
//
//     gzmsg << "getting setting: " << settingName << std::endl;
//     std::map<std::string, double> settingMap;
//     this->n.param(settingName, settingMap, settingMap);
//
//     this->started = false;
//     this->stopping = false;
//     this->finished = false;
//
//     this->totalSentPackets = 0;
//     this->totalRecvPackets = 0;
//     this->topoChangeCount = 0;
//
//     gzmsg << "Net: clearing hashSet" << std::endl;
//     this->hashSet.clear();
//     gzmsg << "Net: done" << std::endl;
//
//     this->n.getParam("simulation_period", this->simPeriod);
//     this->n.getParam("communication_range", this->commRange);
//
//     if (settingMap.count("simulation_period") > 0)
//       this->simPeriod = settingMap["simulation_period"];
//     if (settingMap.count("communication_range") > 0)
//       this->commRange = settingMap["communication_range"];
//
//     gzmsg << "publishing new velocity" << std::endl;
//
//     std_msgs::Float32 vel;
//     this->currentRobotSpeed = settingMap["robot_speed"];
//     vel.data = this->currentRobotSpeed;
//     this->navVelUpdatePub.publish(vel);
//
//     gzmsg << std::endl
//           << "===================================================" << std::endl
//           << "===================================================" << std::endl
//           << "Trial: " << settingName << std::endl
//           << "simulation period: " << this->simPeriod << std::endl
//           << "communicaiton range: " << this->commRange << std::endl
//           << "robot speed: " << vel.data << std::endl
//           << "===================================================" << std::endl
//           << "===================================================" << std::endl;
//
//     for (int i = 1; i <= 9; ++i)
//     {
//       for (int j = i + 1; j <= 10; ++j)
//       {
//         this->topoList[std::to_string(i)+":"+std::to_string(j)] = false;
//       }
//     }
//
//     this->currentDelayTime = settingMap["delay_time"];
//
//     this->listStartResponses.clear();
//     adhoc::msgs::SimInfo start;
//     start.set_state("start");
//     start.set_robot_name("");
//     start.set_delay_time(this->currentDelayTime);
//     this->simCmdPub->Publish(start);
//   }
//   else
//   {
//     gzmsg << "Simulation done: " << std::endl
//           << this->world->SimTime().FormattedString() << " (Sim Time)"
//           << std::endl
//           << this->world->RealTime().FormattedString() << " (Real Time)"
//           << std::endl;
//   }
// }
