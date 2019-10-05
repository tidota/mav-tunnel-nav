// imu_odom.cpp
// 190604
// Odometry only by IMU
//
//
// reference about Quaternion
// https://folk.uio.no/jeanra/Informatics/QuaternionsAndIMUs.html
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

double grav = 9.81;
double x, y, z;
double vx, vy, vz;
ros::Time last_time;
bool initial_imu = true;
bool calibration = true;

tf::Vector3 init_grav_dir;
int sample_num;

////////////////////////////////////////////////////////////////////////////////
void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
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
    if (dt > 3.0 && sample_num >= 100)
    {
      init_grav_dir /= sample_num;
      grav = init_grav_dir.length();

      // double roll, pitch, yaw;
      // tf::Matrix3x3 mat(q);
      // mat.getRPY(roll, pitch, yaw);
      // ROS_INFO_STREAM("roll: " << (roll/M_PI*180) << ", pitch: " << (pitch/M_PI*180) << ", yaw: " << (yaw/M_PI*180));

      last_time = cur_time;
      calibration = false;
    }
    return;
  }

  // velocities
  x += vx * dt;
  y += vy * dt;
  z += vz * dt;

  // gravity vector
  tf::Vector3 grav_dir
    = tf::Transform(
        tf::Quaternion(
          imu->orientation.x, imu->orientation.y,
          imu->orientation.z, imu->orientation.w).inverse(),
          tf::Vector3(0, 0, 0))
      * tf::Vector3(0, 0, grav);

  //ROS_INFO_STREAM("grav_x: " << grav_dir.x() << ", grav_y: " << grav_dir.y() << ", grav_z: " << grav_dir.z());

  // ROS_INFO_STREAM(
  //   "imu_x: " << imu->linear_acceleration.x << ", " <<
  //   "imu_y: " << imu->linear_acceleration.y << ", " <<
  //   "imu_z: " << imu->linear_acceleration.z);

  // acceleration
  double ax = imu->linear_acceleration.x - grav_dir.x();
  double ay = imu->linear_acceleration.y - grav_dir.y();
  double az = imu->linear_acceleration.z - grav_dir.z();
  //if (std::fabs(ax) >= 1.0)
    vx += ax * dt;
  //if (std::fabs(ay) >= 1.0)
    vy += ay * dt;
  //if (std::fabs(az) >= 1.0)
    vz += az * dt;

  // double roll, pitch, yaw;
  // tf::Matrix3x3 mat(current_pose.getRotation());
  // mat.getRPY(roll, pitch, yaw);
  // ROS_INFO_STREAM("roll: " << (roll/M_PI*180) << ", pitch: " << (pitch/M_PI*180) << ", yaw: " << (yaw/M_PI*180));

  ROS_INFO_STREAM("ax: " << ax << ", ay: " << ay << ", az: " << az);

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
