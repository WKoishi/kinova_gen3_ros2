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
 
#ifndef _KORTEX_ACTUATORCONFIG_ROS_CONVERTER_H_
#define _KORTEX_ACTUATORCONFIG_ROS_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ActuatorConfig.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


#include "kortex_driver/msg/axis_position.hpp"
#include "kortex_driver/msg/axis_offsets.hpp"
#include "kortex_driver/msg/torque_calibration.hpp"
#include "kortex_driver/msg/torque_offset.hpp"
#include "kortex_driver/msg/actuator_config_control_mode_information.hpp"
#include "kortex_driver/msg/control_loop.hpp"
#include "kortex_driver/msg/loop_selection.hpp"
#include "kortex_driver/msg/vector_drive_parameters.hpp"
#include "kortex_driver/msg/encoder_derivative_parameters.hpp"
#include "kortex_driver/msg/control_loop_parameters.hpp"
#include "kortex_driver/msg/frequency_response.hpp"
#include "kortex_driver/msg/step_response.hpp"
#include "kortex_driver/msg/ramp_response.hpp"
#include "kortex_driver/msg/custom_data_selection.hpp"
#include "kortex_driver/msg/command_mode_information.hpp"
#include "kortex_driver/msg/servoing.hpp"
#include "kortex_driver/msg/position_command.hpp"
#include "kortex_driver/msg/cogging_feedforward_mode_information.hpp"


int ToRosData(Kinova::Api::ActuatorConfig::AxisPosition input, kortex_driver::msg::AxisPosition &output);
int ToRosData(Kinova::Api::ActuatorConfig::AxisOffsets input, kortex_driver::msg::AxisOffsets &output);
int ToRosData(Kinova::Api::ActuatorConfig::TorqueCalibration input, kortex_driver::msg::TorqueCalibration &output);
int ToRosData(Kinova::Api::ActuatorConfig::TorqueOffset input, kortex_driver::msg::TorqueOffset &output);
int ToRosData(Kinova::Api::ActuatorConfig::ControlModeInformation input, kortex_driver::msg::ActuatorConfigControlModeInformation &output);
int ToRosData(Kinova::Api::ActuatorConfig::ControlLoop input, kortex_driver::msg::ControlLoop &output);
int ToRosData(Kinova::Api::ActuatorConfig::LoopSelection input, kortex_driver::msg::LoopSelection &output);
int ToRosData(Kinova::Api::ActuatorConfig::VectorDriveParameters input, kortex_driver::msg::VectorDriveParameters &output);
int ToRosData(Kinova::Api::ActuatorConfig::EncoderDerivativeParameters input, kortex_driver::msg::EncoderDerivativeParameters &output);
int ToRosData(Kinova::Api::ActuatorConfig::ControlLoopParameters input, kortex_driver::msg::ControlLoopParameters &output);
int ToRosData(Kinova::Api::ActuatorConfig::FrequencyResponse input, kortex_driver::msg::FrequencyResponse &output);
int ToRosData(Kinova::Api::ActuatorConfig::StepResponse input, kortex_driver::msg::StepResponse &output);
int ToRosData(Kinova::Api::ActuatorConfig::RampResponse input, kortex_driver::msg::RampResponse &output);
int ToRosData(Kinova::Api::ActuatorConfig::CustomDataSelection input, kortex_driver::msg::CustomDataSelection &output);
int ToRosData(Kinova::Api::ActuatorConfig::CommandModeInformation input, kortex_driver::msg::CommandModeInformation &output);
int ToRosData(Kinova::Api::ActuatorConfig::Servoing input, kortex_driver::msg::Servoing &output);
int ToRosData(Kinova::Api::ActuatorConfig::PositionCommand input, kortex_driver::msg::PositionCommand &output);
int ToRosData(Kinova::Api::ActuatorConfig::CoggingFeedforwardModeInformation input, kortex_driver::msg::CoggingFeedforwardModeInformation &output);

#endif