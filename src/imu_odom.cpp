// imu_odom.cpp
// 190604
// Odometry only by IMU
//

#include <cmath>

#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

std::string imu_topic;
std::string odom_topic;
ros::Publisher odom_pub;
std::string child_frame_id;

const double grav = 9.81;
double x, y, z;
double vx, vy, vz;
ros::Time last_time;
bool initial_imu = true;
bool calibration = true;
tf::Pose current_pose;

tf::Vector3 init_grav_dir;
int sample_num;

////////////////////////////////////////////////////////////////////////////////
void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  ROS_INFO("got imu");

  ros::Time cur_time = ros::Time::now();

  if (initial_imu)
  {
    ROS_INFO("imuCallback: initial step");
    init_grav_dir = tf::Vector3(
                      imu->linear_acceleration.x,
                      imu->linear_acceleration.y,
                      imu->linear_acceleration.z);
    sample_num = 1;
    last_time = cur_time;
    initial_imu = false;
    return;
  }

  double dt = (cur_time - last_time).toSec();

  if (calibration)
  {
    ROS_INFO("imuCallback: calibration step");
    init_grav_dir += tf::Vector3(
                      imu->linear_acceleration.x,
                      imu->linear_acceleration.y,
                      imu->linear_acceleration.z);
    sample_num++;
    if (dt > 3.0)
    {
      init_grav_dir /= sample_num;

      double phi;
      if (init_grav_dir.z() != 0)
      {
        phi = std::atan(init_grav_dir.x()/init_grav_dir.z());
      }
      else if (init_grav_dir.x() > 0)
      {
        phi = M_PI/2;
      }
      else
      {
        phi = -M_PI/2;
      }

      tf::Quaternion rot_y;
      rot_y.setRotation(tf::Vector3(0, 1, 0), -phi);
      init_grav_dir = tf::Transform(rot_y, tf::Vector3(0, 0, 0)) * init_grav_dir;

      double theta;
      if (init_grav_dir.z() != 0)
      {
        theta = -std::atan(init_grav_dir.y()/init_grav_dir.z());
      }
      else if (init_grav_dir.y() > 0)
      {
        theta = -M_PI/2;
      }
      else
      {
        theta = M_PI/2;
      }

      tf::Quaternion q;
      q.setEuler(0, phi, theta);
      current_pose = tf::Pose(q, tf::Vector3(0, 0, 0));

      last_time = cur_time;
      calibration = false;
      return;
    }
  }

  // velocities
  x += vx * dt;
  y += vy * dt;
  z += vz * dt;

  // gravity vector
  tf::Vector3 grav_dir
    = tf::Transform(current_pose.getRotation(), tf::Vector3(0, 0, 0))
      * tf::Vector3(0, 0, grav);

  // acceleration
  //if (std::fabs(imu->linear_acceleration.x) >= 1.0)
    vx += (imu->linear_acceleration.x - grav_dir.x()) * dt;
  //if (std::fabs(imu->linear_acceleration.y) >= 1.0)
    vy += (imu->linear_acceleration.y - grav_dir.y()) * dt;
  //if (std::fabs(imu->linear_acceleration.z - grav) >= 1.0)
    vz += (imu->linear_acceleration.z - grav_dir.z()) * dt;

  // update the current pose
  tf::Quaternion new_rot;
  new_rot.setEuler(
    imu->angular_velocity.z * dt,
    imu->angular_velocity.x * dt,
    imu->angular_velocity.y * dt);
  new_rot = new_rot * current_pose.getRotation();
  new_rot.normalize();
  current_pose = tf::Pose(new_rot, tf::Vector3(0, 0, 0));

  nav_msgs::Odometry odom;
  odom.header = imu->header;
  odom.header.frame_id = "world";
  odom.child_frame_id = child_frame_id;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = imu->orientation;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.linear.z = vz;
  odom.twist.twist.angular = imu->angular_velocity;

  odom_pub.publish(odom);
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_odom");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  x = y = z = 0;
  vx = vy = vz = 0;
  last_time = ros::Time::now();

  pnh.getParam("imu_topic", imu_topic);
  pnh.getParam("odom_topic", odom_topic);
  pnh.getParam("child_frame_id", child_frame_id);

  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);

  ros::Subscriber imu_sub = nh.subscribe(imu_topic, 1000, imuCallback);

  ros::spin();

  return(0);
}
