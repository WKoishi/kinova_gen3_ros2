/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

/*
 * This file has been auto-generated and should not be modified.
 */
 
#ifndef _KORTEX_CONTROLCONFIG_ROS_CONVERTER_H_
#define _KORTEX_CONTROLCONFIG_ROS_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ControlConfig.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


#include "kortex_driver/msg/gravity_vector.hpp"
#include "kortex_driver/msg/control_config_position.hpp"
#include "kortex_driver/msg/payload_information.hpp"
#include "kortex_driver/msg/cartesian_transform.hpp"
#include "kortex_driver/msg/tool_configuration.hpp"
#include "kortex_driver/msg/control_configuration_notification.hpp"
#include "kortex_driver/msg/cartesian_reference_frame_info.hpp"
#include "kortex_driver/msg/twist_linear_soft_limit.hpp"
#include "kortex_driver/msg/twist_angular_soft_limit.hpp"
#include "kortex_driver/msg/joint_speed_soft_limits.hpp"
#include "kortex_driver/msg/joint_acceleration_soft_limits.hpp"
#include "kortex_driver/msg/kinematic_limits.hpp"
#include "kortex_driver/msg/kinematic_limits_list.hpp"
#include "kortex_driver/msg/desired_speeds.hpp"
#include "kortex_driver/msg/linear_twist.hpp"
#include "kortex_driver/msg/angular_twist.hpp"
#include "kortex_driver/msg/control_config_joint_speeds.hpp"
#include "kortex_driver/msg/control_config_control_mode_information.hpp"
#include "kortex_driver/msg/control_config_control_mode_notification.hpp"


int ToRosData(Kinova::Api::ControlConfig::GravityVector input, kortex_driver::msg::GravityVector &output);
int ToRosData(Kinova::Api::ControlConfig::Position input, kortex_driver::msg::ControlConfigPosition &output);
int ToRosData(Kinova::Api::ControlConfig::PayloadInformation input, kortex_driver::msg::PayloadInformation &output);
int ToRosData(Kinova::Api::ControlConfig::CartesianTransform input, kortex_driver::msg::CartesianTransform &output);
int ToRosData(Kinova::Api::ControlConfig::ToolConfiguration input, kortex_driver::msg::ToolConfiguration &output);
int ToRosData(Kinova::Api::ControlConfig::ControlConfigurationNotification input, kortex_driver::msg::ControlConfigurationNotification &output);
int ToRosData(Kinova::Api::ControlConfig::CartesianReferenceFrameInfo input, kortex_driver::msg::CartesianReferenceFrameInfo &output);
int ToRosData(Kinova::Api::ControlConfig::TwistLinearSoftLimit input, kortex_driver::msg::TwistLinearSoftLimit &output);
int ToRosData(Kinova::Api::ControlConfig::TwistAngularSoftLimit input, kortex_driver::msg::TwistAngularSoftLimit &output);
int ToRosData(Kinova::Api::ControlConfig::JointSpeedSoftLimits input, kortex_driver::msg::JointSpeedSoftLimits &output);
int ToRosData(Kinova::Api::ControlConfig::JointAccelerationSoftLimits input, kortex_driver::msg::JointAccelerationSoftLimits &output);
int ToRosData(Kinova::Api::ControlConfig::KinematicLimits input, kortex_driver::msg::KinematicLimits &output);
int ToRosData(Kinova::Api::ControlConfig::KinematicLimitsList input, kortex_driver::msg::KinematicLimitsList &output);
int ToRosData(Kinova::Api::ControlConfig::DesiredSpeeds input, kortex_driver::msg::DesiredSpeeds &output);
int ToRosData(Kinova::Api::ControlConfig::LinearTwist input, kortex_driver::msg::LinearTwist &output);
int ToRosData(Kinova::Api::ControlConfig::AngularTwist input, kortex_driver::msg::AngularTwist &output);
int ToRosData(Kinova::Api::ControlConfig::JointSpeeds input, kortex_driver::msg::ControlConfigJointSpeeds &output);
int ToRosData(Kinova::Api::ControlConfig::ControlModeInformation input, kortex_driver::msg::ControlConfigControlModeInformation &output);
int ToRosData(Kinova::Api::ControlConfig::ControlModeNotification input, kortex_driver::msg::ControlConfigControlModeNotification &output);

#endif