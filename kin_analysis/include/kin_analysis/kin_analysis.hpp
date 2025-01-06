#ifndef KIN_ANALYSIS__KIN_ANALYSIS_HPP_
#define KIN_ANALYSIS__KIN_ANALYSIS_HPP_

#include <kin_analysis/kin_DH_parameters.hpp>

#include <math.h>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues> 

#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class KinAnalysis : public rclcpp::Node
{
public:
  KinAnalysis();

private:
  /**
   * @brief This callback function runs when KA subscribes current joint state.
   *
   * @param joint_state_msg
   */
   void currentJointStateCallback(
        const sensor_msgs::msg::JointState & joint_state);
  /**
   * @brief This function is used to compute gravutational torque, used for feed forward gravity compensation in the control loop
   *
   * @param limb_id
   */
  Eigen::Matrix<double, 6, JOINT_NUM> computeLimbJacobian(int limb_id);
  /**
   * @brief This function is used to initialize markers
   *
   */
  void markerInitialization();
  /**
   * @brief This function is used to visualize the limb ellipsoid in Rviz as a Marker
   *
   */
  void plotEllipsoid();
  /**
   * @brief This function is used to compute the ellipsoid Radi r1,r2,r3
   *
   * @param Jacobian
   * @param type      // 0: velocity translation, 1: velocity rotation 
   */
  std::array<float, 3> computeEllipsoidRadi(Eigen::Matrix<double, 6, JOINT_NUM> Jacobian, int type);

  sensor_msgs::msg::JointState current_joint_state; // store current joint state 

  // Limbs Ellipsoid Markers
  visualization_msgs::msg::Marker LF_ellipsoid;
  visualization_msgs::msg::Marker LH_ellipsoid;
  visualization_msgs::msg::Marker RH_ellipsoid;
  visualization_msgs::msg::Marker RF_ellipsoid;

  // Publisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr LF_ellipsoid_marker_pub_;  // Publish to RViz
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr LH_ellipsoid_marker_pub_;  // Publish to RViz
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr RH_ellipsoid_marker_pub_;  // Publish to RViz
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr RF_ellipsoid_marker_pub_;  // Publish to RViz
  
  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // tf2 tools
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  // Timer
  rclcpp::TimerBase::SharedPtr plot_timer; 
  bool timer_setted;

};

#endif  // KIN_ANALYSIS__KIN_ANALYSIS_HPP_