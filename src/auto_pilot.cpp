// auto_pilot.cpp
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

#include <mav_tunnel_nav/SrcDst.h>
#include <mav_tunnel_nav/Beacon.h>
#include <mav_tunnel_nav/Particles.h>

#include <ros/ros.h>

// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
// #include <std_msgs/Bool.h>
#include <std_msgs/String.h>
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

std::mutex beacon_mutex;
std::map<std::string, mav_tunnel_nav::Beacon> beacon_buffer;
std::map<std::string, ros::Time> beacon_lasttime;

double distance_to_neighbor;
ros::Duration rel_pose_expiration;

////////////////////////////////////////////////////////////////////////////////
void beaconCallback(const mav_tunnel_nav::Beacon::ConstPtr& msg)
{
  if (msg->destination.size() > 0)
  {
    std::lock_guard<std::mutex> lk(beacon_mutex);
    beacon_buffer[msg->source] = *msg;
    beacon_lasttime[msg->source] = ros::Time::now();
  }
}

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
  // NOTE: There may be some bug in the simulator generating very small value
  //       intermitently. At the moment, it is replaced with range_max
  if (new_range->range >= 0.09)
    range_buff["range_" + new_range->header.frame_id.substr(pos1, pos2 - pos1)]
      = new_range->range;
  // range_buff["range_" + new_range->header.frame_id.substr(pos1, pos2 - pos1)]
  //   = (new_range->range >= 0.09)? new_range->range: range_max;
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

  double STEER_THRESH;
  pnh.getParam("STEER_THRESH", STEER_THRESH);
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

  double straight_rate;
  pnh.getParam("straight_rate", straight_rate);
  double steering_yaw_rate;
  pnh.getParam("steering_yaw_rate", steering_yaw_rate);
  double mid_open;
  pnh.getParam("mid_open", mid_open);
  double middle_line_rate;
  pnh.getParam("middle_line_rate", middle_line_rate);
  double turn_yaw_rate;
  pnh.getParam("turn_yaw_rate", turn_yaw_rate);
  double alt_rate;
  pnh.getParam("alt_rate", alt_rate);
  double obs_rate;
  pnh.getParam("obs_rate", obs_rate);
  double alt_open;
  pnh.getParam("alt_open", alt_open);

  double max_linear_x;
  pnh.getParam("max_linear_x", max_linear_x);
  double max_linear_y;
  pnh.getParam("max_linear_y", max_linear_y);
  double max_linear_z;
  pnh.getParam("max_linear_z", max_linear_z);

  std::string base_station_name;
  pnh.getParam("base_station_name", base_station_name);

  std::string auto_pilot_type;
  pnh.getParam("auto_pilot_type", auto_pilot_type);

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
        // get ranging data
        std::map<std::string, double> range_data;
        {
          std::lock_guard<std::mutex> lk(range_mutex);
          for (auto item: range_buff)
          {
            range_data[item.first] = item.second;
          }
        }

        // === get relative pose data === //
        bool has_dist_front = false;
        bool has_dist_back = false;
        bool has_base = false;
        double dist_front = 0;
        double dist_back = 0;
        double dist_base_x = 0;
        double dist_base = 0;

        double move_x = 0;
        double move_y = 0;
        bool near_base = false;

        tf::Vector3 force_rel;
        {
          auto now = ros::Time::now();
          std::lock_guard<std::mutex> lk(beacon_mutex);
          for (auto p: beacon_buffer)
          {
            if (now - beacon_lasttime[p.first] < rel_pose_expiration)
            {
              auto msg = p.second;
              if (auto_pilot_type == "line")
              {
                auto ori = msg.estimated_orientation;
                if (msg.source == base_station_name)
                {
                  has_base = true;
                  dist_base = msg.estimated_distance;
                  dist_base_x = ori.x * msg.estimated_distance;
                }
                else if (ori.x > 0) // front
                {
                  if (!has_dist_front || msg.estimated_distance < dist_front)
                  {
                    has_dist_front = true;
                    dist_front = msg.estimated_distance;
                  }
                }
                else if (ori.x < 0) // back
                {
                  if (!has_dist_back || msg.estimated_distance < dist_back)
                  {
                    has_dist_back = true;
                    dist_back = msg.estimated_distance;
                  }
                }
              }
              else if (auto_pilot_type == "mesh")
              {
                auto ori = msg.estimated_orientation;
                const double criteria
                  = (ori.x >= 0 && ori.y >= 0)? distance_to_neighbor * 0.6:
                    (ori.x >= 0 && ori.y < 0)?  distance_to_neighbor * 0.7:
                                                distance_to_neighbor;
                double diff = criteria - msg.estimated_distance;
                if (diff > 0)
                {
                  move_x += -ori.x * diff;
                  move_y += -ori.y * diff;
                }

                if (msg.source == base_station_name
                  && msg.estimated_distance < 20.0)
                {
                  near_base = true;
                }
              }
            }
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
          // ===================== going_straight =========================== //
          // moves the robot forward.
          {
            if (auto_pilot_type == "default")
            {
              control_msg.linear.x = straight_rate;
            }
            else if (auto_pilot_type == "line")
            {
              // basically they can only "push" others
              // if there is another neighbor in the direction to go,
              // move slowly or stop.
              if (has_dist_back)
              {
                // consider moving forward
                if (dist_back < distance_to_neighbor * 0.95)
                {
                  {
                    // too close to the back
                    double rate = (distance_to_neighbor * 0.95 - dist_back)
                                / (distance_to_neighbor * 0.1);
                    if (rate > 1.0)
                      rate = 1.0;
                    else if (rate < 0.0)
                      rate = 0.0;
                    control_msg.linear.x
                      = straight_rate * rate;
                  }

                  if (has_dist_front && dist_front < distance_to_neighbor * 0.7)
                  {
                    // too close to the front
                    double rate = (dist_front - distance_to_neighbor * 0.5)
                                / (distance_to_neighbor * (0.7 - 0.5));
                    if (rate > 1.0)
                      rate = 1.0;
                    else if (rate < 0.0)
                      rate = 0.0;

                    control_msg.linear.x *= rate;
                  }
                }
                else
                {
                  control_msg.linear.x = 0;
                }
              }
              else if (has_base)
              {
                if (
                  dist_base_x < 0 && dist_base > distance_to_neighbor * 0.9)
                {
                  // base is behind and too far
                  // should go backward
                  control_msg.linear.x
                    = -straight_rate
                      * (dist_base - distance_to_neighbor * 0.9)
                      / (distance_to_neighbor * 0.02);
                }
                else if (dist_base_x > 0)
                {
                  // base is ahead
                  // should go forward
                  control_msg.linear.x = straight_rate * 0.3;
                }
                else if (dist_base < distance_to_neighbor * 0.8)
                {
                  // base is ahead or it is so close
                  // should go forward
                  control_msg.linear.x
                    = straight_rate
                      * (distance_to_neighbor * 0.8 - dist_base)
                      / (distance_to_neighbor * 0.03);
                }

                if (has_dist_front && dist_front < distance_to_neighbor * 0.7)
                {
                  // too close to the front
                  double rate = (dist_front - distance_to_neighbor * 0.5)
                              / (distance_to_neighbor * (0.7 - 0.5));
                  if (rate > 1.0)
                    rate = 1.0;
                  else if (rate < 0.0)
                    rate = 0.0;

                  control_msg.linear.x *= rate;
                }
              }
              else
              {
                // the previous robot or the base is lost.
                // so it will just retrieve back to the previous place.
                control_msg.linear.x = -0.1 * straight_rate;
              }

              // if (force_rel.x() > 0.1)
              // {
              //   control_msg.linear.x = straight_rate * force_rel.x();
              // }
              if (control_msg.linear.x > straight_rate)
                control_msg.linear.x = straight_rate;
              if (control_msg.linear.x < -straight_rate)
                control_msg.linear.x = -straight_rate;
            }
            else if (auto_pilot_type == "mesh")
            {
              // too close to the back
              double rate = move_x / distance_to_neighbor * 20;
              if (rate > 1.0)
                rate = 1.0;
              else if (rate < -1.0)
                rate = -1.0;
              control_msg.linear.x
                = straight_rate * rate;

              // NOTE: should it consider the wall?
              // const double leng
              //   = std::min({range_data.at("range_dfront"),
              //               range_data.at("range_ufront"),
              //               range_data.at("range_front")});
              const double leng
                = (range_data.at("range_dfront")
                  + range_data.at("range_ufront")
                  + range_data.at("range_front"))/3.0;
              const double thresh = 2.0;
              if (leng < thresh)
              {
                double rate = 1 - (thresh - leng) / (thresh * 0.1);
                if (rate > 1.0)
                  rate = 1.0;
                else if (rate < 0.0)
                  rate = 0.0;

                control_msg.linear.x *= rate;
              }
              // If it says moving back, just ignore and stay there.
              if (control_msg.linear.x < 0)
                control_msg.linear.x = 0;
            }
            else
            {
              ROS_ERROR_STREAM("invalid auto_pilot_type: " << auto_pilot_type);
            }
          }

          // =================== staying_on_the_middle_line ================= //
          // adjusts the horizontal position in the tube.
          {
            if (auto_pilot_type == "mesh")
            {
              // NOTE: just keep going if it is close to the base station.
              if (near_base)
                move_y = 0;

              // should move away from right?

              // too close to the right
              double rate = move_y / distance_to_neighbor * 20;
              if (rate > 1.0)
                rate = 1.0;
              else if (rate < -1.0)
                rate = -1.0;
              control_msg.linear.y = straight_rate * rate;

              // NOTE: should it consider the wall?
              const double lleng
                = (range_data.at("range_lfront")
                  + range_data.at("range_lrear")
                  + range_data.at("range_left"))/3;
              const double rleng
                = (range_data.at("range_rfront")
                  + range_data.at("range_rrear")
                  + range_data.at("range_right"))/3;
              const double leng = (move_y > 0)? lleng: rleng;
              const double thresh = 2.5;
              if (leng < thresh * 0.9)
              {
                double rate = (thresh * 0.9 - leng) / (thresh * 0.5);
                if (rate > 1.0)
                  rate = 1.0;
                else if (rate < 0.0)
                  rate = 0.0;
                if (move_y > 0)
                  control_msg.linear.y = -straight_rate * rate;
                else
                  control_msg.linear.y = straight_rate * rate;
              }
              else if (leng < thresh)
              {
                double rate = 1 - (thresh - leng) / (thresh * 0.1);
                if (rate > 1.0)
                  rate = 1.0;
                else if (rate < 0.0)
                  rate = 0.0;

                control_msg.linear.y *= rate;
              }
            }
            else
            {
              // pick shorter one as "left" length
              const double lengL
                = std::min({range_data.at("range_left"),
                            range_data.at("range_lfront")/sqrt(2),
                            range_data.at("range_front")});

              const double lengR = range_data.at("range_right");

              // diff_rate is the gap from the mid with respect to y axis (left is
              // positive)
              const double mid_leng = (lengL + lengR)/2;
              const double diff_leng = (lengR - lengL)/2;
              const double diff_rate
                = (lengL > range_max * 0.99)? lengR / range_max - mid_open:
                  (mid_leng != 0)? diff_leng/mid_leng: 0;

              // input check
              // if the front side is clear and it is out of range from the middle
              // line apply a proportional value
              if(diff_rate < -MIDDLE_THRESH || MIDDLE_THRESH < diff_rate)
              {
                control_msg.linear.y = middle_line_rate * -diff_rate;
              }
            }
          }

          // ===================== steering ================================= //
          // adjusts the heading so that the robot's right side faces toward the
          // wall.
          {
            if (auto_pilot_type == "mesh")
            {
              // if there is a wall, face along the wall.
              // otherwise, face along the direction to move.
              const double thresh = 2.0;
              double range_rf = range_data.at("range_rfront");
              double range_rr = range_data.at("range_rrear");
              double range_r = range_data.at("range_right");
              const double rleng
                = std::min({range_rf, range_rr, range_r});
              double range_lf = range_data.at("range_lfront");
              double range_lr = range_data.at("range_lrear");
              double range_l = range_data.at("range_left");
              const double lleng
                = std::min({range_lf, range_lr, range_l});

              if (rleng < thresh && rleng < lleng)
              {
                double diff_rate
                  = (range_rr - range_rf) / (range_rf + range_rr);

                // input check
                if(diff_rate < -STEER_THRESH || STEER_THRESH < diff_rate)
                {
                  // calculate the output
                  control_msg.angular.z = steering_yaw_rate * diff_rate;
                }
              }
              else if (lleng < thresh)
              {
                double diff_rate
                  = (range_lf - range_lr) / (range_lf + range_lr);

                // input check
                if(diff_rate < -STEER_THRESH || STEER_THRESH < diff_rate)
                {
                  // calculate the output
                  control_msg.angular.z = steering_yaw_rate * diff_rate;
                }
              }
              else
              {
                double diff_rate = 0;
                if (control_msg.linear.y != 0)
                {
                  diff_rate
                    = std::atan2(control_msg.linear.y, control_msg.linear.x)
                    / M_PI / 10.0;
                }
                control_msg.angular.z = steering_yaw_rate * diff_rate;
              }
            }
            else
            {
              double range_rf = range_data.at("range_rfront");
              double range_rr = range_data.at("range_rrear");
              double diff_rate = (range_rr - range_rf) / (range_rf + range_rr);

              // input check
              if(diff_rate < -STEER_THRESH || STEER_THRESH < diff_rate)
              {
                // calculate the output
                control_msg.angular.z = steering_yaw_rate * diff_rate;
              }
            }
          }

          // ===================== turning_around =========================== //
          // turns the robot around so that it can avoid the wall in front of it.
          {
            // input check
            std::vector<double> vlist
                = {range_data.at("range_ufront"),
                   range_data.at("range_front"),
                   range_data.at("range_dfront")};
            std::sort(vlist.begin(), vlist.end());
            std::vector<double> hlist
                = {range_data.at("range_front"),
                   range_data.at("range_rfront"),
                   range_data.at("range_right")};
            std::sort(hlist.begin(), hlist.end());
            double front_comp = (vlist[0] + vlist[1])/2;
            double rfront_comp = (hlist[0] + hlist[1])/2;
            if(front_comp < TURN_THRESH1 || rfront_comp < TURN_THRESH1)
            {
              // if both up-front and down-front ranges are too short
              control_msg.linear.x = 0;
              control_msg.linear.y = 0;
              // calculate the output
              control_msg.angular.z = turn_yaw_rate;
            }
            else if(
              range_data.at("range_ufront")
                <= range_data.at("range_up") * sqrt(2) * TURN_THRESH2 &&
              range_data.at("range_dfront")
                <= range_data.at("range_down") * sqrt(2) * TURN_THRESH2)
            {
              // if both up-front and down-front ranges are short relative to
              // up and down ranges respectively

              // max (up-front, front, down-front)
              double length_comp
                = fmax(range_data.at("range_front"), length_comp);

              if(
                range_data.at("range_right") > length_comp &&
                range_data.at("range_rfront")
                  > range_data.at("range_right") * sqrt(2) * TURN_THRESH3)
              {
                // if the length is less than both right and right-front ranges
                control_msg.linear.x = 0;
                control_msg.linear.y = 0;
                // calculate the output
                control_msg.angular.z = -turn_yaw_rate;
              }
              else if(
                range_data.at("range_right") > length_comp &&
                range_data.at("range_rfront")
                  <= range_data.at("range_right") * sqrt(2) * TURN_THRESH3)
              {
                // if the length is more than both right and right-front ranges
                control_msg.linear.x = 0;
                control_msg.linear.y = 0;
                // calculate the output
                control_msg.angular.z = turn_yaw_rate;
              }
            }
          }

          // =================== altitude_adjustment ======================== //
          // keeps the altitude in the middle of the vertical line, assumes that
          // there is no impending obstacles.
          // diff_rate is the gap from the mid altitude with respect to z axis
          {
            const double range_up = range_data.at("range_up");
            const double range_down = range_data.at("range_down");
            const double mid_leng = (range_up + range_down)/2;
            const double diff_leng = (range_down - range_up)/2;
            const double diff_rate
              = (range_down == range_max)? 1:
                (range_up == range_max)? range_down / range_max - alt_open:
                (mid_leng != 0)? diff_leng/mid_leng: 0;

            // input check
            if(diff_rate < -ALT_THRESH || ALT_THRESH < diff_rate)
            {
              // if it is out of range defined by DIST_OFF_RATE_ALT, apply a
              // proportional value
              control_msg.linear.z =  alt_rate * -diff_rate;
            }
          }

          // ===================== obstacle_avoidance ======================= //
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
              control_msg.linear.x = mov_dir.x() * obs_rate;
              // y
              control_msg.linear.y = mov_dir.y() * obs_rate;
              // z
              control_msg.linear.z = mov_dir.z() * obs_rate;
            }
          }

          if (control_msg.linear.x > max_linear_x)
            control_msg.linear.x = max_linear_x;
          else if (control_msg.linear.x < -max_linear_x)
            control_msg.linear.x = -max_linear_x;
          if (control_msg.linear.y > max_linear_y)
            control_msg.linear.y = max_linear_y;
          else if (control_msg.linear.y < -max_linear_y)
            control_msg.linear.y = -max_linear_y;
          if (control_msg.linear.z > max_linear_z)
            control_msg.linear.z = max_linear_z;
          else if (control_msg.linear.z < -max_linear_z)
            control_msg.linear.z = -max_linear_z;
        }
        catch (const std::out_of_range& oor)
        {
          ROS_ERROR_STREAM("possibly a problem exists in range_data: " << oor.what());
        }

        // ros::Time update_time = ros::Time::now();
        // control_msg.header.stamp = update_time;
        // control_msg.header.frame_id = "rotors_joy_frame";
        ctrl_pub.publish(control_msg);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "auto_pilot");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // subscriber for beacon
  std::string beacon_down_topic;
  pnh.getParam("beacon_down_topic", beacon_down_topic);
  ros::Subscriber beacon_sub
    = nh.subscribe(beacon_down_topic, 1000, beaconCallback);

  pnh.getParam("enable_topic", enable_topic);
  f_enabled = false;
  ros::ServiceServer srv = nh.advertiseService(enable_topic, enableCallback);

  pnh.getParam("distance_to_neighbor", distance_to_neighbor);
  double dur;
  pnh.getParam("rel_pose_expiration", dur);
  rel_pose_expiration = ros::Duration(dur);

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
