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
 
#ifndef _KORTEX_GRIPPERCYCLIC_PROTO_CONVERTER_H_
#define _KORTEX_GRIPPERCYCLIC_PROTO_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <GripperCyclic.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


#include "kortex_driver/msg/gripper_cyclic_message_id.hpp"
#include "kortex_driver/msg/motor_command.hpp"
#include "kortex_driver/msg/gripper_cyclic_command.hpp"
#include "kortex_driver/msg/motor_feedback.hpp"
#include "kortex_driver/msg/gripper_cyclic_feedback.hpp"
#include "kortex_driver/msg/custom_data_unit.hpp"
#include "kortex_driver/msg/gripper_cyclic_custom_data.hpp"


int ToProtoData(kortex_driver::msg::GripperCyclicMessageId input, Kinova::Api::GripperCyclic::MessageId *output);
int ToProtoData(kortex_driver::msg::MotorCommand input, Kinova::Api::GripperCyclic::MotorCommand *output);
int ToProtoData(kortex_driver::msg::GripperCyclicCommand input, Kinova::Api::GripperCyclic::Command *output);
int ToProtoData(kortex_driver::msg::MotorFeedback input, Kinova::Api::GripperCyclic::MotorFeedback *output);
int ToProtoData(kortex_driver::msg::GripperCyclicFeedback input, Kinova::Api::GripperCyclic::Feedback *output);
int ToProtoData(kortex_driver::msg::CustomDataUnit input, Kinova::Api::GripperCyclic::CustomDataUnit *output);
int ToProtoData(kortex_driver::msg::GripperCyclicCustomData input, Kinova::Api::GripperCyclic::CustomData *output);

#endif