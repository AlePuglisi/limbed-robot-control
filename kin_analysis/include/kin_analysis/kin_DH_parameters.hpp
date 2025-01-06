#ifndef KIN_DH_PARAMETERS__KIN_DH_PARAMETERS_HPP_
#define KIN_DH_PARAMETERS__KIN_DH_PARAMETERS_HPP_

#include <math.h>
#include <string>

// For Legged Robot, Define LIMB and JOINT Number 
constexpr int LIMB_NUM = 4;
constexpr int JOINT_NUM = 7; // Here I'm working with a prototype robot of 7DOF per limb

// Limb Joint Limits (Unit: [rad])
constexpr double JOINT1_UPPER = 2.10;
constexpr double JOINT1_LOWER = -JOINT1_UPPER;
constexpr double JOINT2_UPPER = 1.30;
constexpr double JOINT2_LOWER = -1.95;
constexpr double JOINT3_UPPER = 2.95;
constexpr double JOINT3_LOWER = -0.40;
constexpr double JOINT4_UPPER = M_PI_2;
constexpr double JOINT4_LOWER = -JOINT4_UPPER;
constexpr double JOINT5_UPPER = M_PI;
constexpr double JOINT5_LOWER = -M_PI;
constexpr double JOINT6_UPPER = 0.5*M_PI;
constexpr double JOINT6_LOWER = -0.25*M_PI;
constexpr double JOINT7_UPPER = INFINITY;
constexpr double JOINT7_LOWER = -INFINITY;

// Lims names
std::string limb_names[LIMB_NUM] = {"LF", "LH", "RH", "RF"};
// Joint names list 
std::string joint_names[JOINT_NUM] = {"_B2C", "_C2F", "_F2T", "_T2E", "_wristH", "_wristV", "_driving"};

// We are working with a Legged Robot, and we are interested in the limb usage both for locomotion and manipulation.
// When in locomotion mode, the limb can be considered as a reversed robotic arm, from the contact point (end-effector) to the attchment with the robot base
// When in manipulation mode, the limb can be considered as a robotic arm, from attachment with the base to the end-effector.
// Then, we need two DH models of the same limb, Base->EndEffector (Limb in Manipulation) and EndEffector->Base (Limb in Locmotion)

// Redefine this arrays accoridngly to your robot DH model 

// DH parameters (a,d,alpha,offset)
constexpr double A[JOINT_NUM] = {0.0310, 0.2, 0.2, 0, 0.024, 0.0458, 0.0};
constexpr double D[JOINT_NUM] = {0, 0, 0, 0, 0.0468, 0, 0.1131};
constexpr double ALPHA[JOINT_NUM] = {-M_PI_2, 0, 0, -M_PI_2, M_PI_2, -M_PI_2, 0};
constexpr double OFFSET[JOINT_NUM] = {0, 0, 0, -M_PI_2, 0, 0, 0};

// DH parameters in Contact Model (a,d,alpha,offset)
constexpr double A_CONTACT[JOINT_NUM] = {0.0458, 0.0240, 0, 0.2, 0.2, 0.0310, 0};
constexpr double D_CONTACT[JOINT_NUM] = {0, 0, -0.0468, 0, 0, 0, -0.02};
constexpr double ALPHA_CONTACT[JOINT_NUM] = {-M_PI_2, M_PI_2, -M_PI_2, 0, 0, -M_PI_2, 0};
constexpr double OFFSET_CONTACT[JOINT_NUM] = {0, 0, 0, M_PI_2, -M_PI_2, 0, 0};

#endif  // KIN_DH_PARAMETERS__KIN_DH_PARAMETERS_HPP_