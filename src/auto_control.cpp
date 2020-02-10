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
#include <stdexcept>
#include <thread>
#include <vector>

#include <geometry_msgs/Twist.h>
// #include <mav_msgs/default_topics.h>
// #include <mav_msgs/RollPitchYawrateThrust.h>

#include <ros/ros.h>

// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
// #include <std_msgs/Bool.h>
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
  ros::Publisher ctrl_pub = nh.advertise<geometry_msgs::Twist> (
      "cmd_vel", 10);

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

  double STEER_RIGHT_THRESH;
  pnh.getParam("STEER_RIGHT_THRESH", STEER_RIGHT_THRESH);
  double STEER_LEFT_THRESH;
  pnh.getParam("STEER_LEFT_THRESH", STEER_LEFT_THRESH);
  double MIDDLE_THRESH;
  pnh.getParam("MIDDLE_THRESH", MIDDLE_THRESH);
  double TURN_THRESH1;
  pnh.getParam("TURN_THRESH1", TURN_THRESH1);
  double TURN_THRESH2;
  pnh.getParam("TURN_THRESH2", TURN_THRESH2);
  double TURN_THRESH3;
  pnh.getParam("TURN_THRESH3", TURN_THRESH3);
  double ALT_THRESH;
  pnh.getParam("ALT_THRESH", ALT_THRESH);
  double OBS_THRESH;
  pnh.getParam("OBS_THRESH", OBS_THRESH);

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
        ROS_DEBUG("auto control: calculate inputs!");

        // get ranging data
        std::map<std::string, double> range_data;
        {
          std::lock_guard<std::mutex> lk(range_mutex);
          for (auto item: range_buff)
          {
            range_data[item.first] = item.second;
          }
        }

        //=== reactive control ===//
        geometry_msgs::Twist control_msg;
        control_msg.linear.x = 0;
        control_msg.linear.y = 0;
        control_msg.linear.z = 0;
        control_msg.angular.x = 0;
        control_msg.angular.y = 0;
        control_msg.angular.z = 0;

        try // for std::out_of_range of range_data
        {
          // ===================== going_straight ============================= //
          // moves the robot forward.
          {
            ROS_DEBUG("STRAIGHT");
            double range = range_data.at("range_front");
            if(range > range_max)
              range = range_max;
            double going_straight_pitch = 0.3; // TODO: parameterize it later?
            // control_msg.pitch = going_straight_pitch * range/range_max;
          }

          // ===================== steering =================================== //
          // adjusts the heading so that the robot's right side faces toward the
          // wall.
          {
            double rate = range_data.at("range_rfront") / range_data.at("range_rrear");

            // if(
            //   std::fmax(range_data.at("range_ufront"), range_data.at("range_dfront"))
            //   / std::sqrt(2) < DIST_MIN_STEER)
            // {
            //   rate = 0;
            // }

            // input check
            double steering_yaw_rate = 3.0; // TODO: parameterize it later?
            if(rate > STEER_RIGHT_THRESH)
            {
              ROS_DEBUG("STEER TO THE RIGHT");
              // calculate the output
              // control_msg.yaw_rate = -steering_yaw_rate * (rate - 1.0);
            }
            else if(rate < STEER_LEFT_THRESH)
            {
              ROS_DEBUG("STEER TO THE LEFT");
              // calculate the output
              // control_msg.yaw_rate = steering_yaw_rate * (1.0 - rate);
            }
          }

          // ===================== staying_on_the_middle_line ================= //
          // adjusts the horizontal position in the tube.
          {
            // if(std::fmax(range_data.at("range_ufront"), range_data.at("range_dfront"))
            // / std::sqrt(2) >= DIST_MIN_MID) // TODO: parameterize it later?
            // {
            const double lengL
              = (range_data.at("range_left") < range_data.at("range_lfront")/sqrt(2))?
                  range_data.at("range_left"): range_data.at("range_lfront")/sqrt(2);
            const double lengR = range_data.at("range_right");
            const double lengF = range_data.at("range_front");

            const double leng2comp = (lengL < lengF)? lengL: lengF;

            // diff_rate is the gap from the mid with respect to y axis (left is
            // positive)
            const double mid_leng = (leng2comp + lengR)/2;
            const double diff_leng = (lengR - leng2comp)/2;
            const double diff_rate = (mid_leng != 0)? diff_leng/mid_leng: 0;

            // input check
            // if the front side is clear and it is out of range from the middle
            // line apply a proportional value
            double middle_line_roll = 0.3; // TODO: parameterize it later?
            if(diff_rate < -MIDDLE_THRESH || MIDDLE_THRESH < diff_rate)
            {
              ROS_DEBUG("STAY ON THE MIDDLE LINE");
              // control_msg.roll = middle_line_roll * diff_rate;
            }
            // }
          }

          // ===================== turning_around ============================= //
          // turns the robot around so that it can avoid the wall in front of it.
          {
            double length_comp
              = std::fmax(
                  range_data.at("range_ufront"),
                  range_data.at("range_dfront")) / std::sqrt(2);

            // input check
            double turn_yaw_rate = 3.0; // TODO: parameterize it later?
            if(length_comp < TURN_THRESH1)
            {
              ROS_DEBUG("TURN LEFT!");
              // control_msg.roll = 0;
              // control_msg.pitch = 0;
              // // calculate the output
              // control_msg.yaw_rate = turn_yaw_rate;
            }
            else if(
              range_data.at("range_ufront")
                <= range_data.at("range_up") * sqrt(2) * TURN_THRESH2 &&
              range_data.at("range_dfront")
                <= range_data.at("range_down") * sqrt(2) * TURN_THRESH2)
            {
              length_comp = fmax(range_data.at("range_front"), length_comp);

              if(
                range_data.at("range_right") > length_comp &&
                range_data.at("range_rfront")
                  > range_data.at("range_right") * sqrt(2) * TURN_THRESH3)
              {
                ROS_DEBUG("TURN RIGHT");
                // control_msg.roll = 0;
                // control_msg.pitch = 0;
                // // calculate the output
                // control_msg.yaw_rate = -turn_yaw_rate;
              }
              else if(
                range_data.at("range_right") > length_comp &&
                range_data.at("range_rfront")
                  <= range_data.at("range_right") * sqrt(2) * TURN_THRESH3)
              {
                ROS_DEBUG("TURN LEFT");
                // control_msg.roll = 0;
                // control_msg.pitch = 0;
                // // calculate the output
                // control_msg.yaw_rate = turn_yaw_rate;
              }
            }
          }

          // ===================== altitude_adjustment ======================== //
          // keeps the altitude in the middle of the vertical line, assumes that
          // there is no impending obstacles.
          // diff_rate is the gap from the mid altitude with respect to z axis
          {
            const double mid_leng
              = (range_data.at("range_up") + range_data.at("range_down"))/2;
            const double diff_leng
              = (range_data.at("range_down") - range_data.at("range_up"))/2;
            const double diff_rate = (mid_leng != 0)? diff_leng/mid_leng: 0;

            double alt_thrust = 30.0; // TODO: parameterize it later?

            // input check
            // if it is out of range defined by DIST_OFF_RATE_ALT, apply a proportional value
            if(diff_rate < -ALT_THRESH || ALT_THRESH < diff_rate)
            {
              ROS_DEBUG("KEEP THE ALTITUDE");
              // control_msg.thrust.z =  alt_thrust * (diff_rate + 1.0) / 2;
            }
          }

          // ===================== obstacle_avoidance ========================= //
          // avoids impending obstacles, assumes nothing.
          {
            tf::Vector3 unit_vec(-1, 0, 0);
            tf::Vector3 mov_dir(0, 0, 0);
            for (auto item: range_poses)
            {
              if (range_data.at(item.first) < OBS_THRESH)
              {
                tf::Quaternion rot = item.second.getRotation();
                mov_dir += tf::Transform(rot, tf::Vector3(0,0,0)) * unit_vec;
              }
            }
            if (mov_dir.length() > 0)
            {
              // initialize the command to override it.
              control_msg.linear.x = 0;
              control_msg.linear.y = 0;
              control_msg.linear.z = 0;
              control_msg.angular.x = 0;
              control_msg.angular.y = 0;
              control_msg.angular.z = 0;
              mov_dir.normalize();
              // x
              control_msg.linear.x = mov_dir.x() * 1.0; // TODO: parameterize it later?
              // y
              control_msg.linear.y = mov_dir.y() * 1.0; // TODO: parameterize it later?
              // z
              control_msg.linear.z = mov_dir.z() * 1.0; // TODO: parameterize it later?
            }
          }
        }
        catch (const std::out_of_range& oor)
        {
          ROS_ERROR_STREAM("possibly problem in range_data: " << oor.what());
        }

        // ros::Time update_time = ros::Time::now();
        // control_msg.header.stamp = update_time;
        // control_msg.header.frame_id = "rotors_joy_frame";
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
