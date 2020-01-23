// auto_control.cpp
// 200120
// reactive control

#include <algorithm>
#include <cmath>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

#include <ros/ros.h>

// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// #include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265

std::string world_frame_id;
std::string robot_frame_id;

std::vector<std::string> range_topics;
std::map<std::string, tf::Pose> range_poses;
std::map<std::string, double> range_buff;
std::mutex range_mutex;
double range_max, range_min;

std::string enable_topic;
bool f_enabled;

////////////////////////////////////////////////////////////////////////////////
bool enableCallback(
  std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  f_enabled = request.data;
  response.success = f_enabled;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void rangeCallback(const sensor_msgs::Range::ConstPtr& new_range)
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
  range_buff["range_" + new_range->header.frame_id.substr(pos1, pos2 - pos1)]
    = new_range->range;

  // ROS_DEBUG_STREAM(
  //   "range " << new_range->header.frame_id
  //            << "("
  //            << new_range->header.frame_id.substr(pos1, pos2 - pos1)
  //            << ")"
  //            << " = " << new_range->range);
}

////////////////////////////////////////////////////////////////////////////////
void control_main()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // std::string control_topic;
  // pnh.getParam("control_topic", control_topic);
  ros::Publisher ctrl_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust> (
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 10);

  // random numbers
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::uniform_real_distribution<> dis(0, 1.0);

  tf::TransformBroadcaster tf_broadcaster;
  pnh.getParam("world_frame_id", world_frame_id);
  pnh.getParam("robot_frame_id", robot_frame_id);

  double update_freq;
  pnh.getParam("update_freq", update_freq);
  const ros::Duration update_phase(1.0/update_freq);
  const ros::Time initial_update = ros::Time::now();
  ros::Time last_update = initial_update;

  while (ros::ok())
  {
    ros::Time now = ros::Time::now();

    if (now > last_update + update_phase)
    {
      // calculate the delta T
      //const double deltaT = (now - last_update).toSec();
      // initialize the time step
      last_update = now;

      if (f_enabled)
      {
        ROS_DEBUG("auto control: do something!");

        std::map<std::string, double> range_data;
        {
          std::lock_guard<std::mutex> lk(range_mutex);
          for (auto item: range_buff)
          {
            range_data[item.first] = item.second;
          }
        }

        mav_msgs::RollPitchYawrateThrust control_msg;
        control_msg.roll = 0;
        control_msg.pitch = 0;
        control_msg.yaw_rate = 0;
        control_msg.thrust.x = 0;
        control_msg.thrust.y = 0;
        control_msg.thrust.z = 0;

        ros::Time update_time = ros::Time::now();
        control_msg.header.stamp = update_time;
        control_msg.header.frame_id = "rotors_joy_frame";
        ctrl_pub.publish(control_msg);
      }
      else
      {
        ROS_DEBUG("auto control: disabled");
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "auto_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("enable_topic", enable_topic);
  f_enabled = false;
  ros::ServiceServer srv = nh.advertiseService(enable_topic, enableCallback);

  pnh.getParam("range_max", range_max);
  pnh.getParam("range_min", range_min);
  std::string str_buff;
  pnh.param<std::string>("range_list", str_buff, "");
  std::stringstream ss(str_buff);
  std::string token;
  double x,y,z,P,R,Y;
  std::vector<ros::Subscriber> range_subs;
  while (ss >> token >> x >> y >> z >> R >> P >> Y)
  {
    ROS_DEBUG_STREAM(
      "range: " << token << ", "
                << x << ", " << y << ", " << z << ", "
                << R << ", " << P << ", " << Y);

    range_subs.push_back(
      nh.subscribe(token, 1000, rangeCallback)
    );
    range_topics.push_back(token);
    tf::Vector3 pos(x, y, z);
    tf::Quaternion rot;
    rot.setRPY(R * PI / 180.0, P * PI / 180.0, Y * PI / 180.0);
    tf::Pose range_pose(rot, pos);
    range_poses[token] = range_pose;
  }

  std::thread t(control_main);

  ros::spin();

  t.join();

  return(0);
}
