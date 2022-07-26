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
 
#ifndef _KORTEX_ACTUATORCONFIG_PROTO_CONVERTER_H_
#define _KORTEX_ACTUATORCONFIG_PROTO_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ActuatorConfig.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


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


int ToProtoData(kortex_driver::msg::AxisPosition input, Kinova::Api::ActuatorConfig::AxisPosition *output);
int ToProtoData(kortex_driver::msg::AxisOffsets input, Kinova::Api::ActuatorConfig::AxisOffsets *output);
int ToProtoData(kortex_driver::msg::TorqueCalibration input, Kinova::Api::ActuatorConfig::TorqueCalibration *output);
int ToProtoData(kortex_driver::msg::TorqueOffset input, Kinova::Api::ActuatorConfig::TorqueOffset *output);
int ToProtoData(kortex_driver::msg::ActuatorConfigControlModeInformation input, Kinova::Api::ActuatorConfig::ControlModeInformation *output);
int ToProtoData(kortex_driver::msg::ControlLoop input, Kinova::Api::ActuatorConfig::ControlLoop *output);
int ToProtoData(kortex_driver::msg::LoopSelection input, Kinova::Api::ActuatorConfig::LoopSelection *output);
int ToProtoData(kortex_driver::msg::VectorDriveParameters input, Kinova::Api::ActuatorConfig::VectorDriveParameters *output);
int ToProtoData(kortex_driver::msg::EncoderDerivativeParameters input, Kinova::Api::ActuatorConfig::EncoderDerivativeParameters *output);
int ToProtoData(kortex_driver::msg::ControlLoopParameters input, Kinova::Api::ActuatorConfig::ControlLoopParameters *output);
int ToProtoData(kortex_driver::msg::FrequencyResponse input, Kinova::Api::ActuatorConfig::FrequencyResponse *output);
int ToProtoData(kortex_driver::msg::StepResponse input, Kinova::Api::ActuatorConfig::StepResponse *output);
int ToProtoData(kortex_driver::msg::RampResponse input, Kinova::Api::ActuatorConfig::RampResponse *output);
int ToProtoData(kortex_driver::msg::CustomDataSelection input, Kinova::Api::ActuatorConfig::CustomDataSelection *output);
int ToProtoData(kortex_driver::msg::CommandModeInformation input, Kinova::Api::ActuatorConfig::CommandModeInformation *output);
int ToProtoData(kortex_driver::msg::Servoing input, Kinova::Api::ActuatorConfig::Servoing *output);
int ToProtoData(kortex_driver::msg::PositionCommand input, Kinova::Api::ActuatorConfig::PositionCommand *output);
int ToProtoData(kortex_driver::msg::CoggingFeedforwardModeInformation input, Kinova::Api::ActuatorConfig::CoggingFeedforwardModeInformation *output);

#endif