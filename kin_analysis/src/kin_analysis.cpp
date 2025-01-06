#include <cstdio>
#include "kin_analysis/kin_analysis.hpp"


KinAnalysis::KinAnalysis()
: Node("kin_analysis")
{
  std::cout << "KinAnalysis class is established." << std::endl;
  std::string topic_prefix = "/" + std::string(this->get_name());
  // Publisher
  LF_ellipsoid_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    topic_prefix + "/LF_ellipsoid", 1);
  LH_ellipsoid_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    topic_prefix + "/LH_ellipsoid", 1);
  RH_ellipsoid_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    topic_prefix + "/RH_ellipsoid", 1);
  RF_ellipsoid_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    topic_prefix + "/RF_ellipsoid", 1);
  // Subscriber
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 1, std::bind(&KinAnalysis::currentJointStateCallback, this, std::placeholders::_1));

  // Initialize joint state
  current_joint_state.name.resize(JOINT_NUM*LIMB_NUM);
  current_joint_state.position.resize(JOINT_NUM*LIMB_NUM);
  for(int j=0; j<LIMB_NUM; j++){
    for(int i=0; i<JOINT_NUM; i++){
      current_joint_state.name.at(j*JOINT_NUM + i) = limb_names[j] + joint_names[i];
      current_joint_state.position.at(j*JOINT_NUM + i) = 0.0; 
    }
  }


  // Initialize tf2 related variables
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  timer_setted = false; 

  markerInitialization();
  
}

void KinAnalysis::currentJointStateCallback(const sensor_msgs::msg::JointState & joint_state){
  // Debug
  std::cout << "update joint state" << std::endl; 
  for(int j=0; j<LIMB_NUM; j++){
    // Debug
    std::cout << "update joint state for limb j: " <<  j << std::endl;
    for(int i=0; i<JOINT_NUM; i++){
      auto it = find(joint_state.name.begin(), joint_state.name.end(),current_joint_state.name.at(j*JOINT_NUM + i));
      int index = distance(joint_state.name.begin(), it);
      current_joint_state.position.at(j*JOINT_NUM + i) = joint_state.position.at(index); 
      // Debug
      std::cout << "joint i: " <<  i << " = " << joint_state.position.at(index) << std::endl;
    }
  }

  if(!timer_setted){
    plot_timer = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&KinAnalysis::plotEllipsoid, this));
      timer_setted = true; 
  }
}

Eigen::Matrix<double, 6, JOINT_NUM> KinAnalysis::computeLimbJacobian(int limb_id){
  // define and initialize matrices and vector used for J computation
  Eigen::Matrix<double, 4, 4> A_beforei_i[JOINT_NUM];
  Eigen::Matrix<double, 4, 4> A_0_i[JOINT_NUM];
  Eigen::Vector3d p_i[JOINT_NUM];
  Eigen::Vector3d p_e;
  Eigen::Vector3d z_i[JOINT_NUM];

  Eigen::Matrix<double, 6, JOINT_NUM> J;

  // properly select the model
  double a[JOINT_NUM];
  double d[JOINT_NUM];
  double alpha[JOINT_NUM];
  double offset[JOINT_NUM];

  for(int i=0; i<JOINT_NUM; i++){
      a[i] = A[i];
      d[i] = D[i];
      alpha[i] = ALPHA[i];
      offset[i] = OFFSET[i];
  }

  // compute the homogeneus transformation matrices and z-axis vectors, in base frame
  float q[JOINT_NUM];
  for(int i=0; i<JOINT_NUM; i++){
    q[i] = current_joint_state.position.at(limb_id*JOINT_NUM + i);

    A_beforei_i[i].row(0) << std::cos(q[i]+offset[i]), -std::sin(q[i]+offset[i])*std::cos(alpha[i]), std::sin(q[i]+offset[i])*std::sin(alpha[i]), a[i]*std::cos(q[i]+offset[i]);
    A_beforei_i[i].row(1) << std::sin(q[i]+offset[i]),  std::cos(q[i]+offset[i])*std::cos(alpha[i]), -std::cos(q[i]+offset[i])*std::sin(alpha[i]), a[i]*std::sin(q[i]+offset[i]);
    A_beforei_i[i].row(2) <<    0 , std::sin(alpha[i]), std::cos(alpha[i]), d[i];
    A_beforei_i[i].row(3) <<    0,        0,                 0,               1;
    if(i==0){
       A_0_i[0] = A_beforei_i[0];
    } else if(i>0){
        A_0_i[i] =  A_0_i[i-1]*A_beforei_i[i];
    }
    p_i[i]  << A_0_i[i](0,3),A_0_i[i](1,3),A_0_i[i](2,3);
    z_i[i]    << A_0_i[i](0,2),A_0_i[i](1,2),A_0_i[i](2,2);
  }

  p_e << A_0_i[JOINT_NUM-1](0,3),A_0_i[JOINT_NUM-1](1,3),A_0_i[JOINT_NUM-1](2,3);
  Eigen::Vector3d z_0(0.0, 0.0, 1.0);
  Eigen::Vector3d p_0(0.0, 0.0, 0.0);
  for(int i=0; i<JOINT_NUM; i++){
      if(i==0){
        J.col(i) << z_0.cross(p_e - p_0), z_0;
      }
      else if(i>0){
        J.col(i) << z_i[i-1].cross(p_e-p_i[i-1]), z_i[i-1];
      }
  }
  // Debug
  std::cout << "update jacobian" << std::endl;
  return J; 
}

void KinAnalysis::plotEllipsoid(){
  int limb_id = 0; 

  Eigen::Matrix<double, 6, JOINT_NUM> Jacobian = computeLimbJacobian(limb_id);

  // compute ellipsoid of limb at limb_id using Jacobian matrix 
  std::array<float, 3> ellipsoid_radi; 
  ellipsoid_radi = computeEllipsoidRadi(Jacobian,0);


  // Update Ellipsoid marker 
  geometry_msgs::msg::TransformStamped transform_stamped;
  std::string limb_name;

  if(limb_id == 0){ // LF limb
    // Update tf to compute new Marker pose 
    limb_name = "LF";
    transform_stamped = tf_buffer_->lookupTransform(
        "base_link",                 // Reference frame
        limb_name + "_limb_end",     // Target frame
        tf2::TimePointZero           // Latest available transform
        );
    LF_ellipsoid.header.stamp = this->get_clock()->now();
    LF_ellipsoid.action = visualization_msgs::msg::Marker::MODIFY;
    // Set position from TF translation
    LF_ellipsoid.pose.position.x = transform_stamped.transform.translation.x;
    LF_ellipsoid.pose.position.y = transform_stamped.transform.translation.y;
    LF_ellipsoid.pose.position.z = transform_stamped.transform.translation.z;
    // Set orientation directly from the TF rotation
    // TODO: ROTATE AS IN DH FRAME, for now it is not consitent!
    LF_ellipsoid.pose.orientation = transform_stamped.transform.rotation;

    // Set sphere size (10 cm radius)
    LF_ellipsoid.scale.x = ellipsoid_radi[0];
    LF_ellipsoid.scale.y = ellipsoid_radi[1];
    LF_ellipsoid.scale.z = ellipsoid_radi[2];

    LF_ellipsoid_marker_pub_->publish(LF_ellipsoid);

  }else if(limb_id == 1){ // LH limb
    // Update tf to compute new Marker pose 
    limb_name = "LH";
    transform_stamped = tf_buffer_->lookupTransform(
        "base_link",                 // Reference frame
        limb_name + "_gripper_link",     // Target frame
        tf2::TimePointZero           // Latest available transform
        ); 
    LH_ellipsoid.header.stamp = this->get_clock()->now();
    LH_ellipsoid.action = visualization_msgs::msg::Marker::MODIFY;
    // Set position from TF translation
    LH_ellipsoid.pose.position.x = transform_stamped.transform.translation.x;
    LH_ellipsoid.pose.position.y = transform_stamped.transform.translation.y;
    LH_ellipsoid.pose.position.z = transform_stamped.transform.translation.z; 
    // Set orientation directly from the TF rotation
    // TODO: ROTATE AS IN DH FRAME, for now it is not consitent!
    LH_ellipsoid.pose.orientation = transform_stamped.transform.rotation;

    // Set sphere size (10 cm radius)
    LH_ellipsoid.scale.x = ellipsoid_radi[0];
    LH_ellipsoid.scale.y = ellipsoid_radi[1];
    LH_ellipsoid.scale.z = ellipsoid_radi[2];

    LH_ellipsoid_marker_pub_->publish(LH_ellipsoid);

  }else if(limb_id == 2){ // RH limb
    // Update tf to compute new Marker pose 
    limb_name = "RH";
    transform_stamped = tf_buffer_->lookupTransform(
        "base_link",                 // Reference frame
        limb_name + "_limb_end",     // Target frame
        tf2::TimePointZero           // Latest available transform
        );
    RH_ellipsoid.header.stamp = this->get_clock()->now();
    RH_ellipsoid.action = visualization_msgs::msg::Marker::MODIFY;
    // Set position from TF translation
    RH_ellipsoid.pose.position.x = transform_stamped.transform.translation.x;
    RH_ellipsoid.pose.position.y = transform_stamped.transform.translation.y;
    RH_ellipsoid.pose.position.z = transform_stamped.transform.translation.z; 
    // Set orientation directly from the TF rotation
    // TODO: ROTATE AS IN DH FRAME, for now it is not consitent!
    RH_ellipsoid.pose.orientation = transform_stamped.transform.rotation;

    // Set sphere size (10 cm radius)
    RH_ellipsoid.scale.x = ellipsoid_radi[0];
    RH_ellipsoid.scale.y = ellipsoid_radi[1];
    RH_ellipsoid.scale.z = ellipsoid_radi[2];

    RH_ellipsoid_marker_pub_->publish(RH_ellipsoid);

  }else if(limb_id == 3){ // RF limb
    // Update tf to compute new Marker pose 
    limb_name = "RF";
    transform_stamped = tf_buffer_->lookupTransform(
        "base_link",                 // Reference frame
        limb_name + "_limb_end",     // Target frame
        tf2::TimePointZero           // Latest available transform
        );
    RF_ellipsoid.header.stamp = this->get_clock()->now();
    RF_ellipsoid.action = visualization_msgs::msg::Marker::MODIFY;
    // Set position from TF translation
    RF_ellipsoid.pose.position.x = transform_stamped.transform.translation.x;
    RF_ellipsoid.pose.position.y = transform_stamped.transform.translation.y;
    RF_ellipsoid.pose.position.z = transform_stamped.transform.translation.z;
    // Set orientation directly from the TF rotation
    // TODO: ROTATE AS IN DH FRAME, for now it is not consitent!
    RF_ellipsoid.pose.orientation = transform_stamped.transform.rotation;

    // Set sphere size (10 cm radius)
    RF_ellipsoid.scale.x = ellipsoid_radi[0];
    RF_ellipsoid.scale.y = ellipsoid_radi[1];
    RF_ellipsoid.scale.z = ellipsoid_radi[2];

    RF_ellipsoid_marker_pub_->publish(RF_ellipsoid);
  }

}

std::array<float, 3> KinAnalysis::computeEllipsoidRadi(Eigen::Matrix<double, 6, JOINT_NUM> Jacobian, int type){
  Eigen::Matrix<double, 6, 6> E_core;
  Eigen::Matrix<double, 3, 3> E_core_reduced;

  E_core = Jacobian*Jacobian.transpose();
  if(type == 0){
    E_core_reduced = E_core.block(0,0,3,3); // extract translation part
  } else{
    E_core_reduced = E_core.block(3,3,3,3); // extract rotation part
  }

  Eigen::EigenSolver<Eigen::Matrix<double, 3, 3>> es(E_core_reduced);

  std::array<float, 3> radi; 
  radi[0] = 1/sqrt(es.eigenvalues()[0].real());
  radi[1] = 1/sqrt(es.eigenvalues()[1].real());
  radi[2] = 1/sqrt(es.eigenvalues()[2].real());
  
  return radi; 
}

void KinAnalysis::markerInitialization(){
  LF_ellipsoid.header.frame_id = "base_link";
  LF_ellipsoid.ns = "LF_limb_ellipsoid";
  LF_ellipsoid.header.stamp = this->get_clock()->now();
  LF_ellipsoid.type = visualization_msgs::msg::Marker::SPHERE;
  LF_ellipsoid.action = visualization_msgs::msg::Marker::ADD;
  LF_ellipsoid.id = 0; 
  LF_ellipsoid.pose.position.x = 0;
  LF_ellipsoid.pose.position.y = 0;
  LF_ellipsoid.pose.position.z = 0;
  LF_ellipsoid.pose.orientation.x = 0.0;
  LF_ellipsoid.pose.orientation.y = 0.0;
  LF_ellipsoid.pose.orientation.z = 0.0;
  LF_ellipsoid.pose.orientation.w = 1.0;
  LF_ellipsoid.scale.x = 1.0;
  LF_ellipsoid.scale.y = 1.0;
  LF_ellipsoid.scale.z = 1.0;
  LF_ellipsoid.color.r = 0.0f;
  LF_ellipsoid.color.g = 1.0f;
  LF_ellipsoid.color.b = 0.0f;
  LF_ellipsoid.color.a = 0.5;   
  LF_ellipsoid.frame_locked = true; 

  LH_ellipsoid.header.frame_id = "base_link";
  LH_ellipsoid.ns = "LF_limb_ellipsoid";
  LH_ellipsoid.header.stamp = this->get_clock()->now();
  LH_ellipsoid.type = visualization_msgs::msg::Marker::SPHERE;
  LH_ellipsoid.action = visualization_msgs::msg::Marker::ADD;
  LH_ellipsoid.id = 0; 
  LH_ellipsoid.pose.position.x = 0;
  LH_ellipsoid.pose.position.y = 0;
  LH_ellipsoid.pose.position.z = 0;
  LH_ellipsoid.pose.orientation.x = 0.0;
  LH_ellipsoid.pose.orientation.y = 0.0;
  LH_ellipsoid.pose.orientation.z = 0.0;
  LH_ellipsoid.pose.orientation.w = 1.0;
  LH_ellipsoid.scale.x = 1.0;
  LH_ellipsoid.scale.y = 1.0;
  LH_ellipsoid.scale.z = 1.0;
  LH_ellipsoid.color.r = 0.0f;
  LH_ellipsoid.color.g = 1.0f;
  LH_ellipsoid.color.b = 0.0f;
  LH_ellipsoid.color.a = 0.5; 
  LH_ellipsoid.frame_locked = true; 

  RH_ellipsoid.header.frame_id = "base_link";
  RH_ellipsoid.ns = "RH_limb_ellipsoid";
  RH_ellipsoid.header.stamp = this->get_clock()->now();
  RH_ellipsoid.type = visualization_msgs::msg::Marker::SPHERE;
  RH_ellipsoid.action = visualization_msgs::msg::Marker::ADD;
  RH_ellipsoid.id = 0; 
  RH_ellipsoid.pose.position.x = 0;
  RH_ellipsoid.pose.position.y = 0;
  RH_ellipsoid.pose.position.z = 0;
  RH_ellipsoid.pose.orientation.x = 0.0;
  RH_ellipsoid.pose.orientation.y = 0.0;
  RH_ellipsoid.pose.orientation.z = 0.0;
  RH_ellipsoid.pose.orientation.w = 1.0;
  RH_ellipsoid.scale.x = 1.0;
  RH_ellipsoid.scale.y = 1.0;
  RH_ellipsoid.scale.z = 1.0;
  RH_ellipsoid.color.r = 0.0f;
  RH_ellipsoid.color.g = 1.0f;
  RH_ellipsoid.color.b = 0.0f;
  RH_ellipsoid.color.a = 0.5;    
  RH_ellipsoid.frame_locked = true; 

  RF_ellipsoid.header.frame_id = "base_link";
  RF_ellipsoid.ns = "RF_limb_ellipsoid";
  RF_ellipsoid.header.stamp = this->get_clock()->now();
  RF_ellipsoid.type = visualization_msgs::msg::Marker::SPHERE;
  RF_ellipsoid.action = visualization_msgs::msg::Marker::ADD;
  RF_ellipsoid.id = 0; 
  RF_ellipsoid.pose.position.x = 0;
  RF_ellipsoid.pose.position.y = 0;
  RF_ellipsoid.pose.position.z = 0;
  RF_ellipsoid.pose.orientation.x = 0.0;
  RF_ellipsoid.pose.orientation.y = 0.0;
  RF_ellipsoid.pose.orientation.z = 0.0;
  RF_ellipsoid.pose.orientation.w = 1.0;
  RF_ellipsoid.scale.x = 1.0;
  RF_ellipsoid.scale.y = 1.0;
  RF_ellipsoid.scale.z = 1.0;
  RF_ellipsoid.color.r = 0.0f;
  RF_ellipsoid.color.g = 1.0f;
  RF_ellipsoid.color.b = 0.0f;
  RF_ellipsoid.color.a = 0.5;   
  RF_ellipsoid.frame_locked = true; 
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinAnalysis>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}