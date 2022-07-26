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
 
#ifndef _KORTEX_BASECYCLIC_PROTO_CONVERTER_H_
#define _KORTEX_BASECYCLIC_PROTO_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <BaseCyclic.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


#include "kortex_driver/msg/actuator_command.hpp"
#include "kortex_driver/msg/actuator_feedback.hpp"
#include "kortex_driver/msg/actuator_custom_data.hpp"
#include "kortex_driver/msg/base_feedback.hpp"
#include "kortex_driver/msg/base_cyclic_custom_data.hpp"
#include "kortex_driver/msg/base_cyclic_command.hpp"
#include "kortex_driver/msg/base_cyclic_feedback.hpp"


int ToProtoData(kortex_driver::msg::ActuatorCommand input, Kinova::Api::BaseCyclic::ActuatorCommand *output);
int ToProtoData(kortex_driver::msg::ActuatorFeedback input, Kinova::Api::BaseCyclic::ActuatorFeedback *output);
int ToProtoData(kortex_driver::msg::ActuatorCustomData input, Kinova::Api::BaseCyclic::ActuatorCustomData *output);
int ToProtoData(kortex_driver::msg::BaseFeedback input, Kinova::Api::BaseCyclic::BaseFeedback *output);
int ToProtoData(kortex_driver::msg::BaseCyclicCustomData input, Kinova::Api::BaseCyclic::CustomData *output);
int ToProtoData(kortex_driver::msg::BaseCyclicCommand input, Kinova::Api::BaseCyclic::Command *output);
int ToProtoData(kortex_driver::msg::BaseCyclicFeedback input, Kinova::Api::BaseCyclic::Feedback *output);

#endif