// imu_odom.cpp
// 190604
// Odometry only by IMU
//

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
    init_grav_dir += tf::Vector3(
                      imu->linear_acceleration.x,
                      imu->linear_acceleration.y,
                      imu->linear_acceleration.z);
    sample_num++;
    if (dt > 3.0)
    {
      init_grav_dir /= sample_num;

      tf::Vector3 z_axis(0.0, 0.0, 1.0);
      tf::Vector3 half = init_grav_dir + z_axis;
      tf::Vector3 rot_axis = z_axis.cross(half);
      tf::Quaternion q(
        rot_axis.x(), rot_axis.y(), rot_axis.z(), z_axis.dot(half));
      q.normalize();
      current_pose = tf::Pose(q, tf::Vector3(0, 0, 0));

      calibration = false;
      return;
    }
  }


  x += vx * dt;
  y += vy * dt;
  z += vz * dt;
  //if (std::fabs(imu->linear_acceleration.x) >= 1.0)
    vx += imu->linear_acceleration.x * dt;
  //if (std::fabs(imu->linear_acceleration.y) >= 1.0)
    vy += imu->linear_acceleration.y * dt;
  //if (std::fabs(imu->linear_acceleration.z - grav) >= 1.0)
    vz += (imu->linear_acceleration.z - grav) * dt;

  nav_msgs::Odometry odom;
  odom.header = imu->header;
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
