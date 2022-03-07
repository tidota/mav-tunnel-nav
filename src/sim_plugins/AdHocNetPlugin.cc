#include <algorithm>
#include <fstream>
#include <random>
#include <sstream>
#include <thread>

// #include <dynamic_reconfigure/Reconfigure.h>
// #include <openssl/sha.h>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math.hh>
#include <ignition/math/config.hh>

#include <mav_tunnel_nav/SpawnRobot.h>

#include <ros/master.h>

#include <sim_plugins/AdHocNetPlugin.hh>

#include <tf/transform_datatypes.h>

// NOTE: visualization
#include <visualization_msgs/Marker.h>

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(AdHocNetPlugin)

/////////////////////////////////////////////////
void AdHocNetPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  gzmsg << "Starting Ad Hoc Net server" << std::endl;

  GZ_ASSERT(_world, "AdHocNetPlugin world pointer is NULL");
  this->world = _world;

  this->nh.param(
    "auto_pilot_type", this->auto_pilot_type, this->auto_pilot_type);
  this->nh.param("robots", this->robotList, this->robotList);

  this->nh.param("comm_range", this->comm_range, this->comm_range);

  this->nh.param("sigmaDst", this->sigmaDst, this->sigmaDst);
  this->nh.param("sigmaOri", this->sigmaOri, this->sigmaOri);
  this->nh.param("biasDst", this->biasDst, this->biasDst);
  this->nh.param("biasOri", this->biasOri, this->biasOri);
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

  std::string submap_up_topic;
  this->nh.param(
    "submap_up_topic", submap_up_topic, submap_up_topic);
  std::string submap_down_topic;
  this->nh.param(
    "submap_down_topic", submap_down_topic, submap_down_topic);
  std::string submap_ack_up_topic;
  this->nh.param(
    "submap_ack_up_topic", submap_ack_up_topic, submap_ack_up_topic);
  std::string submap_ack_down_topic;
  this->nh.param(
    "submap_ack_down_topic", submap_ack_down_topic, submap_ack_down_topic);

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

    this->submap_subs[robot]
      = this->nh.subscribe(
          "/" + robot + "/" + submap_up_topic, 1000,
          &AdHocNetPlugin::OnSubmapMsg, this);
    this->submap_pubs[robot]
      = this->nh.advertise<mav_tunnel_nav::Submap>(
          "/" + robot + "/" + submap_down_topic, 1);

    this->submap_ack_subs[robot]
      = this->nh.subscribe(
          "/" + robot + "/" + submap_ack_up_topic, 1000,
          &AdHocNetPlugin::OnSubmapAckMsg, this);
    this->submap_ack_pubs[robot]
      = this->nh.advertise<mav_tunnel_nav::SubmapAck>(
          "/" + robot + "/" + submap_ack_down_topic, 1);
  }

  double ratio_spawn_comm;
  this->nh.param("ratio_spawn_comm", ratio_spawn_comm, ratio_spawn_comm);
  this->spawnDist = this->comm_range * ratio_spawn_comm;
  this->spawning = false;
  auto init_robot_elem = _sdf->GetElement("init_robot_loc");
  this->initLoc = tf::Vector3(
                    init_robot_elem->Get<double>("x"),
                    init_robot_elem->Get<double>("y"),
                    init_robot_elem->Get<double>("z"));
  this->initOri = init_robot_elem->Get<double>("ori");
  std::string service_name;
  this->nh.param("spawn_service_name", service_name, service_name);
  this->robot_spawner_client
    = this->nh.serviceClient<mav_tunnel_nav::SpawnRobot>(service_name);
  gzmsg << "ROS service name: " << service_name << std::endl;

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
  for (unsigned int i = 0; i < this->robotList.size(); ++i)
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

  // NOTE: visualization
  enable_vis_cooploc = _sdf->Get<bool>("enable_vis_cooploc");
  marker_pub
    = this->nh.advertise<visualization_msgs::Marker>("data_viz", 10);

  this->nh.param("first_robot_autostart",
    this->first_robot_autostart, this->first_robot_autostart);
}

/////////////////////////////////////////////////
void AdHocNetPlugin::OnUpdate()
{
  ros::Time current = ros::Time::now();
  if (current - last_update > topo_interval)
  {
    // check interactions of each pair of robots.
    for (unsigned int i = 0; i < this->spawnedList.size(); ++i)
    {
      for (unsigned int j = i + 1; j < this->spawnedList.size(); ++j)
      {
        auto robot1 = this->world->ModelByName(this->spawnedList[i]);
        auto robot2 = this->world->ModelByName(this->spawnedList[j]);
        if (robot1 && robot2)
        {
          auto robot1_pose = robot1->WorldPose();
          auto robot2_pose = robot2->WorldPose();

          // NOTE: ignition math 4 and math 5 have a different behavior
          // Gazebo 11 uses Ignition math 6
          // Older versions uses Ignition math 4
          // http://gazebosim.org/tutorials?tut=install_dependencies_from_source
          #if GAZEBO_MAJOR_VERSION < 11
          auto diff = robot2_pose * robot1_pose.Inverse();
          #else
          auto diff = robot1_pose.Inverse() * robot2_pose;
          #endif

          setPoseInfo(this->spawnedList[i], this->spawnedList[j], diff);
          if (diff.Pos().Length() <= this->comm_range)
          {
            setTopoInfo(
              this->spawnedList[i], this->spawnedList[j],
              this->CheckLineOfSight(robot1_pose.Pos(), robot2_pose.Pos()));
          }
          else
          {
            setTopoInfo(
              this->spawnedList[i], this->spawnedList[j], false);
          }
        }
      }
    }

    // check intercations with the base station
    for (auto robot_name: this->spawnedList)
    {
      auto robot = this->world->ModelByName(robot_name);
      if (robot)
      {
        #if GAZEBO_MAJOR_VERSION < 11
        auto diff = robot->WorldPose() * base_pose.Inverse();
        #else
        auto diff = base_pose.Inverse() * robot->WorldPose();
        #endif
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

    if (this->spawning)
    {
      if (this->world->ModelByName(this->robotList[this->spawnedList.size()]))
      {
        this->spawnedList.push_back(
          this->robotList[this->spawnedList.size()]);
        this->spawning = false;
      }
    }
    else
    {
      // check if the next robot is to be spawned.
      bool toSpawn = false;
      if (this->spawnedList.size() == 0)
      {
        toSpawn = true;
      }
      else if (this->spawnedList.size() != this->robotList.size())
      {
        // check if the last robot is far away from the base.
        auto robot = this->world->ModelByName(
              this->spawnedList[this->spawnedList.size() - 1]);
        if (robot->WorldPose().Pos().Length() > this->spawnDist)
          toSpawn = true;
      }

      if (toSpawn)
      {
        // call the ROS service.
        mav_tunnel_nav::SpawnRobot srv;
        if (this->auto_pilot_type == "mesh")
        {
          // NOTE: spawn at different places

          tf::Vector3 pos(2.0, 0, 0.2);
          double initY;
          if (this->spawnedList.size() % 2 == 0)
          {
            initY = 0.1;
          }
          else
          {
            initY = 2.0;
          }
          pos = pos.rotate(tf::Vector3(0,0,1), initY);

          srv.request.x = pos.getX();
          srv.request.y = pos.getY();
          srv.request.z = pos.getZ();
          srv.request.Y = initY;
        }
        else
        {
          srv.request.x = this->initLoc.getX();
          srv.request.y = this->initLoc.getY();
          srv.request.z = this->initLoc.getZ();
          srv.request.Y = this->initOri;
        }
        if (!this->first_robot_autostart && this->spawnedList.size() == 0)
          srv.request.auto_enable_by_slam = false;
        else
          srv.request.auto_enable_by_slam = true;
        srv.request.robot = this->robotList[this->spawnedList.size()];
        gzmsg << "calling the ROS service" << std::endl;
        this->robot_spawner_client.call(srv);
        gzmsg << "calling the ROS service: done" << std::endl;

        this->spawning = true;
      }
    }

    // for publishing the ground truth trajectory
    auto now = ros::Time::now();
    for (auto robot_name: this->spawnedList)
    {
      auto robot = this->world->ModelByName(robot_name);
      if (robot)
      {
        auto pose = robot->WorldPose();
        tf::Vector3 pos(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        tf::Quaternion rot(
          pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
        tf::StampedTransform tf_stamped(
          tf::Transform(rot, pos), now,
          "world", robot_name + "_groundtruth");
        this->tf_broadcaster.sendTransform(tf_stamped);
      }
    }

    // for publishing the base station's pose.
    {
      auto pose = this->base_pose;
      tf::Vector3 pos(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
      tf::Quaternion rot(
        pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
      tf::StampedTransform tf_stamped(
        tf::Transform(rot, pos), now,
        "world", this->base_name);
      this->tf_broadcaster.sendTransform(tf_stamped);
    }

    last_update = current;
  }
}

//////////////////////////////////////////////////
void AdHocNetPlugin::OnBeaconMsg(const mav_tunnel_nav::Beacon::ConstPtr& msg)
{
  // std::lock_guard<std::mutex> lk(this->beacon_mutex);
  // this->beacon_buff(std::make_shared<mav_tunnel_nav::Beacon>(*msg));

  std::normal_distribution<> dst_noise(biasDst, sigmaDst);
  std::normal_distribution<> ori_noise(biasOri, sigmaOri);

  if (this->beacon_subs.count(msg->source) > 0)
  {
    if (msg->destination.size() == 0)
    {
      for (auto robot: this->spawnedList)
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

  std::normal_distribution<> dst_noise(biasDst, sigmaDst);
  std::normal_distribution<> ori_noise(biasOri, sigmaOri);

  if (this->sync_subs.count(msg->source) > 0)
  {
    if (msg->destination == this->base_name)
    {
      // if the packet is for the base, just send data to the source.

      if (getTopoInfo(msg->source, this->base_name))
      {
        mav_tunnel_nav::Particles msg2send;
        msg2send.source = this->base_name;
        msg2send.destination = msg->source;
        {
          ignition::math::Vector3d pos = this->base_pose.Pos();
          ignition::math::Quaternion<double> rot = this->base_pose.Rot();
          geometry_msgs::Pose pose_msg;
          pose_msg.position.x = pos.X();
          pose_msg.position.y = pos.Y();
          pose_msg.position.z = pos.Z();
          pose_msg.orientation.w = rot.W();
          pose_msg.orientation.x = rot.X();
          pose_msg.orientation.y = rot.Y();
          pose_msg.orientation.z = rot.Z();
          msg2send.particles.push_back(pose_msg);
        }
        msg2send.cumul_weights.push_back(1.0);
        auto pos = getPoseInfo(this->base_name, msg->source).Pos();

        // assume the base can get absolute bearing infor.
        pos = this->base_pose.Rot() * pos;

        msg2send.estimated_distance = pos.Length() + dst_noise(gen);
        pos.Normalize();
        ignition::math::Quaternion<double>
          rot(0, ori_noise(gen), ori_noise(gen)); // noise on pitch/yaw
        pos = rot * pos;
        msg2send.estimated_orientation.x = pos.X();
        msg2send.estimated_orientation.y = pos.Y();
        msg2send.estimated_orientation.z = pos.Z();

        this->data_pubs[msg->source].publish(msg2send);

        // NOTE: visualization
        if (enable_vis_cooploc)
        {
          visualization_msgs::Marker line_list, line_sens;
          line_list.header.frame_id = "/world";
          line_list.header.stamp = ros::Time::now();
          line_list.ns = base_name;
          line_list.action = visualization_msgs::Marker::ADD;
          line_list.pose.orientation.w = 1.0;
          line_list.type = visualization_msgs::Marker::LINE_LIST;
          line_list.scale.x = 0.1;
          line_sens = line_list;
          line_list.id = 0;
          line_list.color.r = 1.0;
          line_list.color.a = 1.0;
          geometry_msgs::Point p1, p2;
          auto p1_buff = this->base_pose.Pos();
          p1.x = p1_buff.X();
          p1.y = p1_buff.Y();
          p1.z = p1_buff.Z();
          auto p2_buff
            = this->world->ModelByName(msg->source)->WorldPose().Pos();
          p2.x = p2_buff.X();
          p2.y = p2_buff.Y();
          p2.z = p2_buff.Z();
          line_list.points.push_back(p1);
          line_list.points.push_back(p2);
          marker_pub.publish(line_list);
          p2_buff = this->base_pose.Pos();
          p2.x = p2_buff.X()
               + msg2send.estimated_orientation.x * msg2send.estimated_distance;
          p2.y = p2_buff.Y()
               + msg2send.estimated_orientation.y * msg2send.estimated_distance;
          p2.z = p2_buff.Z()
               + msg2send.estimated_orientation.z * msg2send.estimated_distance;
          line_sens.points.push_back(p1);
          line_sens.points.push_back(p2);
          line_sens.color.g = 1.0;
          line_sens.color.a = 1.0;
          line_sens.id = 1;
          marker_pub.publish(line_sens);
        }
      }
    }
    else if (this->sync_pubs.count(msg->destination) > 0)
    {
      // if the packet is for another robot, just send the packe to
      // the destination.

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

  std::normal_distribution<> dst_noise(biasDst, sigmaDst);
  std::normal_distribution<> ori_noise(biasOri, sigmaOri);

  if (this->data_subs.count(msg->source) > 0)
  {
    if (this->data_pubs.count(msg->destination) > 0)
    {
      // NOTE: it used to check the connectivity here, but it no longer does.
      //       This is because the connection is supposed to be established
      //       once the sync packet is received.
      // // if the destination is in the range
      // if (getTopoInfo(msg->source, msg->destination))
      {
        // auto robot1 = this->world->ModelByName(msg->source);
        // auto robot2 = this->world->ModelByName(msg->destination);

        // NOTE: A particles packet provides a relative pose of the receiver
        //       with respect to the sender.
        mav_tunnel_nav::Particles msg2send = *msg;
        auto pos = getPoseInfo(msg->source, msg->destination).Pos();

        msg2send.estimated_distance = pos.Length() + dst_noise(gen);
        pos.Normalize();
        ignition::math::Quaternion<double>
          rot(0, ori_noise(gen), ori_noise(gen)); // noise on pitch/yaw
        pos = rot * pos;
        msg2send.estimated_orientation.x = pos.X();
        msg2send.estimated_orientation.y = pos.Y();
        msg2send.estimated_orientation.z = pos.Z();
        this->data_pubs[msg->destination].publish(msg2send);

        // NOTE: visualization
        if (enable_vis_cooploc)
        {
          visualization_msgs::Marker line_list, line_sens;
          line_list.header.frame_id = "/world";
          line_list.header.stamp = ros::Time::now();
          line_list.ns = base_name;
          line_list.action = visualization_msgs::Marker::ADD;
          line_list.pose.orientation.w = 1.0;
          line_list.type = visualization_msgs::Marker::LINE_LIST;
          line_list.scale.x = 0.1;
          line_sens = line_list;
          line_list.id = 0;
          line_list.color.r = 1.0;
          line_list.color.a = 1.0;
          geometry_msgs::Point p1, p2;
          auto p1_buff
            = this->world->ModelByName(msg->source)->WorldPose().Pos();
          p1.x = p1_buff.X();
          p1.y = p1_buff.Y();
          p1.z = p1_buff.Z();
          auto p2_buff
            = this->world->ModelByName(msg->destination)->WorldPose().Pos();
          p2.x = p2_buff.X();
          p2.y = p2_buff.Y();
          p2.z = p2_buff.Z();
          line_list.points.push_back(p1);
          line_list.points.push_back(p2);
          marker_pub.publish(line_list);
          p2_buff
            = this->world->ModelByName(msg->source)->WorldPose().Pos()
            + this->world->ModelByName(msg->source)->WorldPose().Rot() *
             ignition::math::Vector3d(
              msg2send.estimated_orientation.x * msg2send.estimated_distance,
              msg2send.estimated_orientation.y * msg2send.estimated_distance,
              msg2send.estimated_orientation.z * msg2send.estimated_distance);
          p2.x = p2_buff.X();
          p2.y = p2_buff.Y();
          p2.z = p2_buff.Z();
          line_sens.points.push_back(p1);
          line_sens.points.push_back(p2);
          line_sens.color.g = 1.0;
          line_sens.color.a = 1.0;
          line_sens.id = 1;
          marker_pub.publish(line_sens);
        }

      }
    }
  }
}

//////////////////////////////////////////////////
void AdHocNetPlugin::OnSubmapMsg(const mav_tunnel_nav::Submap::ConstPtr& msg)
{
  if (this->submap_subs.count(msg->source) > 0
   && this->submap_pubs.count(msg->destination) > 0)
  {
    // if the destination is in the range
    if (getTopoInfo(msg->source, msg->destination))
    {
      this->submap_pubs[msg->destination].publish(*msg);
    }
  }
}

//////////////////////////////////////////////////
void AdHocNetPlugin::OnSubmapAckMsg(const mav_tunnel_nav::SubmapAck::ConstPtr& msg)
{
  if (this->submap_ack_subs.count(msg->source) > 0
   && this->submap_ack_pubs.count(msg->destination) > 0)
  {
    // if the destination is in the range
    if (getTopoInfo(msg->source, msg->destination))
    {
      this->submap_ack_pubs[msg->destination].publish(*msg);
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
