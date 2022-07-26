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
 
#ifndef _KORTEX_CONTROLCONFIG_PROTO_CONVERTER_H_
#define _KORTEX_CONTROLCONFIG_PROTO_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ControlConfig.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


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


int ToProtoData(kortex_driver::msg::GravityVector input, Kinova::Api::ControlConfig::GravityVector *output);
int ToProtoData(kortex_driver::msg::ControlConfigPosition input, Kinova::Api::ControlConfig::Position *output);
int ToProtoData(kortex_driver::msg::PayloadInformation input, Kinova::Api::ControlConfig::PayloadInformation *output);
int ToProtoData(kortex_driver::msg::CartesianTransform input, Kinova::Api::ControlConfig::CartesianTransform *output);
int ToProtoData(kortex_driver::msg::ToolConfiguration input, Kinova::Api::ControlConfig::ToolConfiguration *output);
int ToProtoData(kortex_driver::msg::ControlConfigurationNotification input, Kinova::Api::ControlConfig::ControlConfigurationNotification *output);
int ToProtoData(kortex_driver::msg::CartesianReferenceFrameInfo input, Kinova::Api::ControlConfig::CartesianReferenceFrameInfo *output);
int ToProtoData(kortex_driver::msg::TwistLinearSoftLimit input, Kinova::Api::ControlConfig::TwistLinearSoftLimit *output);
int ToProtoData(kortex_driver::msg::TwistAngularSoftLimit input, Kinova::Api::ControlConfig::TwistAngularSoftLimit *output);
int ToProtoData(kortex_driver::msg::JointSpeedSoftLimits input, Kinova::Api::ControlConfig::JointSpeedSoftLimits *output);
int ToProtoData(kortex_driver::msg::JointAccelerationSoftLimits input, Kinova::Api::ControlConfig::JointAccelerationSoftLimits *output);
int ToProtoData(kortex_driver::msg::KinematicLimits input, Kinova::Api::ControlConfig::KinematicLimits *output);
int ToProtoData(kortex_driver::msg::KinematicLimitsList input, Kinova::Api::ControlConfig::KinematicLimitsList *output);
int ToProtoData(kortex_driver::msg::DesiredSpeeds input, Kinova::Api::ControlConfig::DesiredSpeeds *output);
int ToProtoData(kortex_driver::msg::LinearTwist input, Kinova::Api::ControlConfig::LinearTwist *output);
int ToProtoData(kortex_driver::msg::AngularTwist input, Kinova::Api::ControlConfig::AngularTwist *output);
int ToProtoData(kortex_driver::msg::ControlConfigJointSpeeds input, Kinova::Api::ControlConfig::JointSpeeds *output);
int ToProtoData(kortex_driver::msg::ControlConfigControlModeInformation input, Kinova::Api::ControlConfig::ControlModeInformation *output);
int ToProtoData(kortex_driver::msg::ControlConfigControlModeNotification input, Kinova::Api::ControlConfig::ControlModeNotification *output);

#endif