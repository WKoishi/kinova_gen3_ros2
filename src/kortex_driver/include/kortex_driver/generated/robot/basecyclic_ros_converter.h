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
 
#ifndef _KORTEX_BASECYCLIC_ROS_CONVERTER_H_
#define _KORTEX_BASECYCLIC_ROS_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <BaseCyclic.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


#include "kortex_driver/msg/actuator_command.hpp"
#include "kortex_driver/msg/actuator_feedback.hpp"
#include "kortex_driver/msg/actuator_custom_data.hpp"
#include "kortex_driver/msg/base_feedback.hpp"
#include "kortex_driver/msg/base_cyclic_custom_data.hpp"
#include "kortex_driver/msg/base_cyclic_command.hpp"
#include "kortex_driver/msg/base_cyclic_feedback.hpp"


int ToRosData(Kinova::Api::BaseCyclic::ActuatorCommand input, kortex_driver::msg::ActuatorCommand &output);
int ToRosData(Kinova::Api::BaseCyclic::ActuatorFeedback input, kortex_driver::msg::ActuatorFeedback &output);
int ToRosData(Kinova::Api::BaseCyclic::ActuatorCustomData input, kortex_driver::msg::ActuatorCustomData &output);
int ToRosData(Kinova::Api::BaseCyclic::BaseFeedback input, kortex_driver::msg::BaseFeedback &output);
int ToRosData(Kinova::Api::BaseCyclic::CustomData input, kortex_driver::msg::BaseCyclicCustomData &output);
int ToRosData(Kinova::Api::BaseCyclic::Command input, kortex_driver::msg::BaseCyclicCommand &output);
int ToRosData(Kinova::Api::BaseCyclic::Feedback input, kortex_driver::msg::BaseCyclicFeedback &output);

#endif