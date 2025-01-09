#include <cstdio>
#include "kin_analysis/kin_analysis.hpp"

#define DEBUG_ENABLED true

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
#if DEBUG_ENABLED 
      std::cout << "joint i: " <<  i << " = " << joint_state.position.at(index) << std::endl;
#endif //DEBUG_ENABLED 
    }
  }

  if(!timer_setted){
    plot_timer = this->create_wall_timer(
// TODO Update plotEllipsoid to chose which limb ellipsoid to plot from a selection list  
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
  Eigen::Matrix<double, 6, 6> J_move;
  Eigen::Matrix<double, 6, JOINT_NUM> J_rotated;

  float rotz = std::cos(M_PI_4); 
  J_move.row(0) << rotz, rotz, 0, 0, 0, 0;
  J_move.row(1) << -rotz, rotz, 0, 0, 0, 0;
  J_move.row(2) << 0, 0, 1, 0, 0, 0;
  J_move.row(3) << 0, 0, 0, rotz, rotz, 0;
  J_move.row(4) << 0, 0, 0, -rotz, rotz, 0;
  J_move.row(5) << 0, 0, 0, 0, 0, 1;
 

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
#if DEBUG_ENABLED 
  std::cout << "Forward Kinematic: " << p_e << "\n" << std::endl;
#endif //DEBUG_ENABLED 
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
  J_rotated = J_move.transpose()*J; 
#if DEBUG_ENABLED 
  std::cout << "update jacobian" << std::endl;
  std::cout << "J(q) = \n" << J << "\n---------------------------\n" << std::endl; 
#endif //DEBUG_ENABLED 
  return J; 
}

void KinAnalysis::plotEllipsoid(){
  int limb_id = 0; 

  Eigen::Matrix<double, 6, JOINT_NUM> Jacobian = computeLimbJacobian(limb_id);

  // compute ellipsoid of limb at limb_id using Jacobian matrix 
  Eigen::Matrix<double, 3, 4> ellipsoid_radi_and_direction; 
  ellipsoid_radi_and_direction = computeEllipsoidRadi(Jacobian,0);
  Eigen::Matrix<double, 3, 3> ellipsoid_rotation_matrix = ellipsoid_radi_and_direction.block(0,1,3,3);

  // Update Ellipsoid marker 
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::TransformStamped transform_stamped_base_root;
  std::string limb_name;

  tf2::Quaternion URDFtoDH; 
  URDFtoDH.setRPY(-M_PI, -M_PI_2, 0.0);

  if(limb_id == 0){ // LF limb
    // Update tf to compute new Marker pose 
    limb_name = "LF";
    transform_stamped_base_root = tf_buffer_->lookupTransform(
        "base_link",                 // Reference frame
        limb_name + "_limb_root",     // Target frame
        tf2::TimePointZero           // Latest available transform
        );

    tf2::Quaternion base_root(transform_stamped_base_root.transform.rotation.x, transform_stamped_base_root.transform.rotation.y, transform_stamped_base_root.transform.rotation.z, transform_stamped_base_root.transform.rotation.w); 
    tf2::Matrix3x3 base_root_rotation_matrix(base_root);
    tf2::Matrix3x3 ellipsoid_rotation_matrix_tf2(
        ellipsoid_rotation_matrix(0, 0), ellipsoid_rotation_matrix(0, 1), ellipsoid_rotation_matrix(0, 2),
        ellipsoid_rotation_matrix(1, 0), ellipsoid_rotation_matrix(1, 1), ellipsoid_rotation_matrix(1, 2),
        ellipsoid_rotation_matrix(2, 0), ellipsoid_rotation_matrix(2, 1), ellipsoid_rotation_matrix(2, 2)
    );

    tf2::Matrix3x3 final_rotation_matrix;
    final_rotation_matrix = base_root_rotation_matrix * ellipsoid_rotation_matrix_tf2;  
    tf2::Quaternion final_rotation;
    final_rotation_matrix.getRotation(final_rotation);



    //Eigen::Quaterniond final_rotation = base_root * ellipsoid_rotation;
#if DEBUG_ENABLED 
  std::cout << "Ellipsoid Rotation" << ellipsoid_rotation_matrix << " \n" ;
  std::cout << "\n-----------------------\n * Base Root Rotation \n" << std::endl;
  for (int i = 0; i < 3; ++i) {
      tf2::Vector3 row = base_root_rotation_matrix.getRow(i);
      std::cout << row.x() << " " << row.y() << " " << row.z() << "\n";
    }
  std::cout << "\n-----------------------\n = Final Rotation \n" << std::endl;
  for (int i = 0; i < 3; ++i) {
      tf2::Vector3 row = final_rotation_matrix.getRow(i);
      std::cout << row.x() << " " << row.y() << " " << row.z() << "\n";
    }
#endif //DEBUG_ENABLED 

    transform_stamped = tf_buffer_->lookupTransform(
        "base_link",                 // Reference frame
        limb_name + "_end_effector_DH_Link",     // Target frame
        tf2::TimePointZero           // Latest available transform
        );

    LF_ellipsoid.header.stamp = this->get_clock()->now();
    LF_ellipsoid.action = visualization_msgs::msg::Marker::MODIFY;

    // Set position from TF translation
    LF_ellipsoid.pose.position.x = transform_stamped.transform.translation.x;
    LF_ellipsoid.pose.position.y = transform_stamped.transform.translation.y;
    LF_ellipsoid.pose.position.z = transform_stamped.transform.translation.z;
    // Set orientation 
    LF_ellipsoid.pose.orientation.x = final_rotation.x();
    LF_ellipsoid.pose.orientation.y = final_rotation.y();
    LF_ellipsoid.pose.orientation.z = final_rotation.z();
    LF_ellipsoid.pose.orientation.w = final_rotation.w();
    

    // Set Ellipsoid size 
    LF_ellipsoid.scale.x = ellipsoid_radi_and_direction(0,0);
    LF_ellipsoid.scale.y = ellipsoid_radi_and_direction(1,0);
    LF_ellipsoid.scale.z = ellipsoid_radi_and_direction(2,0);

    LF_ellipsoid_marker_pub_->publish(LF_ellipsoid);

  }else if(limb_id == 1){ // LH limb
    // Update tf to compute new Marker pose 
    limb_name = "LH";
    transform_stamped = tf_buffer_->lookupTransform(
        "base_link",                 // Reference frame
        limb_name + "_gripper_Link",     // Target frame
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
 
    tf2::Quaternion initial_rotation(transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w);
    // Compose rotations
    tf2::Quaternion result_rotation = initial_rotation * URDFtoDH;
    LH_ellipsoid.pose.orientation.x = result_rotation.x();
    LH_ellipsoid.pose.orientation.y = result_rotation.y();
    LH_ellipsoid.pose.orientation.z = result_rotation.z();
    LH_ellipsoid.pose.orientation.w = result_rotation.w();


    // Set Ellipsoid size 
    LH_ellipsoid.scale.x = ellipsoid_radi_and_direction(0,0);
    LH_ellipsoid.scale.y = ellipsoid_radi_and_direction(1,0);
    LH_ellipsoid.scale.z = ellipsoid_radi_and_direction(2,0);

    LH_ellipsoid_marker_pub_->publish(LH_ellipsoid);

  }else if(limb_id == 2){ // RH limb
    // Update tf to compute new Marker pose 
    limb_name = "RH";
    transform_stamped = tf_buffer_->lookupTransform(
        "base_link",                 // Reference frame
        limb_name + "_gripper_Link",     // Target frame
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

    tf2::Quaternion initial_rotation(transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w);
    // Compose rotations
    tf2::Quaternion result_rotation = initial_rotation * URDFtoDH;
    RH_ellipsoid.pose.orientation.x = result_rotation.x();
    RH_ellipsoid.pose.orientation.y = result_rotation.y();
    RH_ellipsoid.pose.orientation.z = result_rotation.z();
    RH_ellipsoid.pose.orientation.w = result_rotation.w();

    // Set Ellipsoid size 
    RH_ellipsoid.scale.x = ellipsoid_radi_and_direction(0,0);
    RH_ellipsoid.scale.y = ellipsoid_radi_and_direction(1,0);
    RH_ellipsoid.scale.z = ellipsoid_radi_and_direction(2,0);


    RH_ellipsoid_marker_pub_->publish(RH_ellipsoid);

  }else if(limb_id == 3){ // RF limb
    // Update tf to compute new Marker pose 
    limb_name = "RF";
    transform_stamped = tf_buffer_->lookupTransform(
        "base_link",                 // Reference frame
        limb_name + "_gripper_Link",     // Target frame
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
    

    tf2::Quaternion initial_rotation(transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w);
    // Compose rotations
    tf2::Quaternion result_rotation = initial_rotation * URDFtoDH;
    RF_ellipsoid.pose.orientation.x = result_rotation.x();
    RF_ellipsoid.pose.orientation.y = result_rotation.y();
    RF_ellipsoid.pose.orientation.z = result_rotation.z();
    RF_ellipsoid.pose.orientation.w = result_rotation.w();


    // Set Ellipsoid size 
    RF_ellipsoid.scale.x = ellipsoid_radi_and_direction(0,0);
    RF_ellipsoid.scale.y = ellipsoid_radi_and_direction(1,0);
    RF_ellipsoid.scale.z = ellipsoid_radi_and_direction(2,0);

    RF_ellipsoid_marker_pub_->publish(RF_ellipsoid);
  }

}

Eigen::Matrix<double, 3, 4> KinAnalysis::computeEllipsoidRadi(Eigen::Matrix<double, 6, JOINT_NUM> Jacobian, int type){
  Eigen::Matrix<double, 6, 6> E_core;
  Eigen::Matrix<double, 3, 3> E_core_reduced;
  Eigen::Matrix<double, 3, 3> E_core_reduced_scaled;
  Eigen::Matrix<double, 3, 3> radi_direction; 
  std::array<double, 3> radi; 
  
  Eigen::Matrix<double, 3,4> radi_and_direction; 

  E_core = Jacobian*Jacobian.transpose();
#if DEBUG_ENABLED 
  std::cout << "Limb " << " E(q) = \n" <<  E_core << "\n---------------------------\n" << std::endl;
#endif //DEBUG_ENABLED 
  if(type == 0){
    E_core_reduced = E_core.block(0,0,3,3); // extract translation part
  } else{
    E_core_reduced = E_core.block(3,3,3,3); // extract rotation part
  }

  float scale_factor = 0.0625; 
  E_core_reduced_scaled = E_core_reduced * scale_factor;
#if DEBUG_ENABLED 
  std::cout << "Limb " << " E_scaled(q) = \n" <<  E_core_reduced_scaled << "\n---------------------------\n" << std::endl;
#endif //DEBUG_ENABLED 

  Eigen::EigenSolver<Eigen::Matrix<double, 3, 3>> es(E_core_reduced_scaled);
#if DEBUG_ENABLED 
  std::cout << "Limb " << " First eigenvector = \n" <<  es.eigenvectors().col(0).real() << "\n---------------------------\n" << std::endl;
#endif //DEBUG_ENABLED 

  radi_direction.row(0) << es.eigenvectors().col(0).real().transpose();
  radi_direction.row(1) << es.eigenvectors().col(1).real().transpose();
  radi_direction.row(2) << es.eigenvectors().col(2).real().transpose();

  radi[0] = sqrt(es.eigenvalues()[0].real());
  radi[1] = sqrt(es.eigenvalues()[1].real());
  radi[2] = sqrt(es.eigenvalues()[2].real());
#if DEBUG_ENABLED 
  std::cout << "Ellipsoid radii: rx = " << radi[0] << ", ry = " << radi[1] << ", rz = " << radi[2] << "\n" <<  std::endl; 
#endif //DEBUG_ENABLED 
  
  radi_and_direction.row(0) << radi[0], radi_direction.row(0);
  radi_and_direction.row(1) << radi[1], radi_direction.row(1);
  radi_and_direction.row(2) << radi[2], radi_direction.row(2);
#if DEBUG_ENABLED 
  std::cout << "Ellipsoid radii and Direction collected: \n " << radi_and_direction << "\n---------------------------\n" <<  std::endl; 
#endif //DEBUG_ENABLED 
  return radi_and_direction;
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