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
tf::Pose current_pose;

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
      current_pose = tf::Pose(q, tf::Vector3(0, 0, 0)).inverse();

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
    = tf::Transform(current_pose.getRotation().inverse(), tf::Vector3(0, 0, 0))
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

  // angular velocity
  double wx = imu->angular_velocity.x;
  double wy = imu->angular_velocity.y;
  double wz = imu->angular_velocity.z;

  double l = std::sqrt(wx*wx + wy*wy + wz*wz);
  double dtlo2 = dt * l / 2;

  tf::Quaternion new_rot;
  if (l != 0)
  {
    new_rot = tf::Quaternion(
      std::sin(dtlo2) * wx / l, std::sin(dtlo2) * wy / l,
      std::sin(dtlo2) * wz / l, std::cos(dtlo2));
  }

  // update the current pose
  new_rot = new_rot * current_pose.getRotation() * new_rot.inverse();
  new_rot.normalize();
  current_pose = tf::Pose(new_rot, tf::Vector3(0, 0, 0));

  nav_msgs::Odometry odom;
  odom.header = imu->header;
  odom.header.frame_id = "world";
  odom.child_frame_id = child_frame_id;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  //odom.pose.pose.orientation = imu->orientation;
  odom.pose.pose.orientation.x = new_rot.x();
  odom.pose.pose.orientation.y = new_rot.y();
  odom.pose.pose.orientation.z = new_rot.z();
  odom.pose.pose.orientation.w = new_rot.w();
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
