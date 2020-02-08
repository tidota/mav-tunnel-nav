// this code is based on OSRF's repository of SubT example team
// https://bitbucket.org/osrf/subt/

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <map>
#include <string>
#include <vector>

/// \brief. Tele-operation node to control a team of robots by joy sticks.
class Teleop
{
  /// \brief Constructor.
  public: Teleop();

  /// \brief Callback function for a joy stick control.
  private: void JoyCallback(const sensor_msgs::Joy::ConstPtr &_joy);

  /// \brief ROS node handler.
  private: ros::NodeHandle nh;

  /// \brief Index for the linear axis of the joy stick.
  private: int linear;

  /// \brief Index for the angular axis of the joy stick.
  private: int angular;

  /// \brief Scale value for the linear axis input.
  private: double linearScale;

  /// \brief Scale value for the angular axis input.
  private: double angularScale;

  /// \brief Index for the vertical axis (z axis) of the joy stick.
  private: int vertical;

  /// \brief Index for the horizontal axis (y axis) of the joy stick.
  private: int horizontal;

  /// \brief Scale value for the vertical axis (z axis) input.
  private: double verticalScale;

  /// \brief Scale value for the horizontal axis (y axis) input.
  private: double horizontalScale;

  /// \brief Index for horizontal axis of arrow keys.
  private: int axisArrowHorizontal;

  /// \brief Index for vertial axis of arrow keys.
  private: int axisArrowVertical;

  /// \brief Index for the left dead man's switch.
  private: int leftDeadMan;

  /// \brief Index for the right dead man's switch.
  private: int rightDeadMan;

  /// \brief Subscriber to get input values from the joy control.
  private: ros::Subscriber joySub;

  /// \brief Map from a button name to an index.
  /// e.g., 'A' -> 1
  /// this mapping should be stored in a yaml file.
  private: std::map<std::string, int> joyButtonIndexMap;

  /// \brief Map from a robot name to a ROS publisher to control velocity.
  private: ros::Publisher velPubMap;
};

/////////////////////////////////////////////////
Teleop::Teleop():
  linear(1), angular(0), linearScale(0), angularScale(0),
  vertical(3), horizontal(2), verticalScale(0),
  horizontalScale(0),
  axisArrowHorizontal(4), axisArrowVertical(5)
{
  // Load joy control settings. Setting values must be loaded by rosparam.
  this->nh.param("axis_linear", this->linear, this->linear);
  this->nh.param("axis_angular", this->angular, this->angular);
  this->nh.param("scale_linear", this->linearScale, this->linearScale);
  this->nh.param("scale_angular", this->angularScale, this->angularScale);

  this->nh.param("axis_vertical", this->vertical, this->vertical);
  this->nh.param("axis_horizontal", this->horizontal, this->horizontal);
  this->nh.param("scale_vertical", this->verticalScale, this->verticalScale);
  this->nh.param(
    "scale_horizontal", this->horizontalScale, this->horizontalScale);

  this->nh.param(
    "dead_man_switch_left", this->leftDeadMan, this->leftDeadMan);
  this->nh.param(
    "dead_man_switch_right", this->rightDeadMan, this->rightDeadMan);

  this->nh.param(
    "axis_arrow_horizontal",
    this->axisArrowHorizontal, this->axisArrowHorizontal);
  this->nh.param(
    "axis_arrow_vertical", this->axisArrowVertical, this->axisArrowVertical);

  this->nh.getParam("button_map", this->joyButtonIndexMap);


  // Create a publisher object to generate a velocity command, and associate
  // it to the corresponding robot's name.
  this->velPubMap
    = this->nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

  // Subscribe "joy" topic to listen to the joy control.
  this->joySub
    = this->nh.subscribe<sensor_msgs::Joy>(
        "joy", 1, &Teleop::JoyCallback, this);
}

/////////////////////////////////////////////////
void Teleop::JoyCallback(const sensor_msgs::Joy::ConstPtr &_joy)
{
  geometry_msgs::Twist twist;
  // If the trigger button or dead man's switch is pressed,
  // calculate control values.
  if (_joy->buttons[this->rightDeadMan] ||
      _joy->buttons[this->leftDeadMan])
  {
    twist.linear.x
      = this->linearScale * _joy->axes[this->linear];
    twist.linear.y
      = this->horizontalScale * _joy->axes[this->horizontal];
    twist.linear.z
      = this->verticalScale * _joy->axes[this->vertical];
    twist.angular.z
      = this->angularScale * _joy->axes[this->angular];

    // Publish the control values.
    this->velPubMap.publish(twist);
  }
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_node");

  Teleop Teleop;

  ros::spin();
}
