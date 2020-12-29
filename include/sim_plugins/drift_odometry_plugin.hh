#ifndef DRIFT_ODOMETRY_PLUGIN_HH_
#define DRIFT_ODOMETRY_PLUGIN_HH_

#include <cmath>
#include <deque>
#include <random>
#include <stdio.h>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <opencv2/core/core.hpp>

#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

#include <Eigen/Dense>
#include <tinyxml.h>

#include <mav_tunnel_nav/protobuf/Odometry.pb.h>


namespace gazebo
{
//===============================================================================================//
//================================== TOPICS FOR ROS INTERFACE ===================================//
//===============================================================================================//

// These should perhaps be defined in an .sdf/.xacro file instead?
static const std::string kConnectGazeboToRosSubtopic = "connect_gazebo_to_ros_subtopic";
// static const std::string kConnectRosToGazeboSubtopic = "connect_ros_to_gazebo_subtopic";

/// \brief    Special-case topic for ROS interface plugin to listen to (if present)
///           and broadcast transforms to the ROS system.
static const std::string kBroadcastTransformSubtopic = "broadcast_transform";


/// \brief      Obtains a parameter from sdf.
/// \param[in]  sdf           Pointer to the sdf object.
/// \param[in]  name          Name of the parameter.
/// \param[out] param         Param Variable to write the parameter to.
/// \param[in]  default_value Default value, if the parameter not available.
/// \param[in]  verbose       If true, gzerror if the parameter is not available.
template<class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value, const bool& verbose =
                     false) {
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else {
    param = default_value;
    if (verbose)
      gzerr << "[rotors_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
  }
  return false;
}

#if SDF_MAJOR_VERSION >= 3
  typedef ignition::math::Vector3d SdfVector3;
#else
  class SdfVector3 : public sdf::Vector3 {
  /*
  A wrapper class for deprecated sdf::Vector3 class to provide the same accessor
  functions as in the newer ignition::math::Vector3 class.
  */

   public:
    using sdf::Vector3::Vector3;
    virtual ~SdfVector3() {}

    /// \brief Get the x value
    /// \return the x value
    double X() { return this->x; }

    /// \brief Get the y value
    /// \return the y value
    double Y() { return this->y; }

    /// \brief Get the z value
    /// \return the z value
    double Z() { return this->z; }
  };
#endif

// Default values
static const std::string kDefaultParentFrameId = "world";
static const std::string kDefaultChildFrameId = "odometry_sensor";
static const std::string kDefaultLinkName = "odometry_sensor_link";

static constexpr int kDefaultMeasurementDelay = 0;
static constexpr int kDefaultMeasurementDivisor = 1;
static constexpr int kDefaultGazeboSequence = 0;
static constexpr int kDefaultOdometrySequence = 0;
static constexpr double kDefaultUnknownDelay = 0.0;
static constexpr double kDefaultCovarianceImageScale = 1.0;

class DriftOdometryPlugin : public ModelPlugin
{
 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;
  typedef std::deque<std::pair<int, gz_geometry_msgs::Odometry> > OdometryQueue;
  typedef boost::array<double, 36> CovarianceMatrix;

  DriftOdometryPlugin()
      : ModelPlugin(),

        pubs_and_subs_created_(false),

        // DEFAULT TOPICS
        pose_pub_topic_(mav_msgs::default_topics::POSE),
        pose_with_covariance_stamped_pub_topic_(mav_msgs::default_topics::POSE_WITH_COVARIANCE),
        position_stamped_pub_topic_(mav_msgs::default_topics::POSITION),
        transform_stamped_pub_topic_(mav_msgs::default_topics::TRANSFORM),
        odometry_pub_topic_(mav_msgs::default_topics::ODOMETRY),
        //---------------
        parent_frame_id_(kDefaultParentFrameId),
        child_frame_id_(kDefaultChildFrameId),
        link_name_(kDefaultLinkName),
        measurement_delay_(kDefaultMeasurementDelay),
        measurement_divisor_(kDefaultMeasurementDivisor),
        gazebo_sequence_(kDefaultGazeboSequence),
        odometry_sequence_(kDefaultOdometrySequence),
        unknown_delay_(kDefaultUnknownDelay),
        covariance_image_scale_(kDefaultCovarianceImageScale),

        random_generator_(random_device_())
  {};
  ~DriftOdometryPlugin() {};

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:

  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
  ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
  bool pubs_and_subs_created_;

  /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
  /// \details  Call this once the first time OnUpdate() is called (can't
  ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
  ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
  void CreatePubsAndSubs();

  OdometryQueue odometry_queue_;

  std::string namespace_;
  std::string pose_pub_topic_;
  std::string pose_with_covariance_stamped_pub_topic_;
  std::string position_stamped_pub_topic_;
  std::string transform_stamped_pub_topic_;
  std::string odometry_pub_topic_;
  std::string parent_frame_id_;
  std::string child_frame_id_;
  std::string link_name_;

  NormalDistribution position_n_[3];
  NormalDistribution attitude_n_[3];
  NormalDistribution linear_velocity_n_[3];
  NormalDistribution angular_velocity_n_[3];
  UniformDistribution position_u_[3];
  UniformDistribution attitude_u_[3];
  UniformDistribution linear_velocity_u_[3];
  UniformDistribution angular_velocity_u_[3];

  CovarianceMatrix pose_covariance_matrix_;
  CovarianceMatrix twist_covariance_matrix_;

  int measurement_delay_;
  int measurement_divisor_;
  int gazebo_sequence_;
  int odometry_sequence_;
  double unknown_delay_;
  double covariance_image_scale_;
  cv::Mat covariance_image_;

  std::random_device random_device_;
  std::mt19937 random_generator_;

  gazebo::transport::NodePtr node_handle_;

  gazebo::transport::PublisherPtr pose_pub_;
  gazebo::transport::PublisherPtr pose_with_covariance_stamped_pub_;
  gazebo::transport::PublisherPtr position_stamped_pub_;
  gazebo::transport::PublisherPtr transform_stamped_pub_;
  gazebo::transport::PublisherPtr odometry_pub_;

  /// \brief    Special-case publisher to publish stamped transforms with
  ///           frame IDs. The ROS interface plugin (if present) will
  ///           listen to this publisher and broadcast the transform
  ///           using transform_broadcast().
  gazebo::transport::PublisherPtr broadcast_transform_pub_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::EntityPtr parent_link_;

  /// \brief    Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
};

} // namespace gazebo

template<class Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(const Eigen::MatrixBase<Derived> & theta) {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
  }
  else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f, theta[2] * f);
  }
}

#endif // DRIFT_ODOMETRY_PLUGIN_HH_