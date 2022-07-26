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
 
#ifndef _KORTEX_ACTUATORCYCLIC_ROS_CONVERTER_H_
#define _KORTEX_ACTUATORCYCLIC_ROS_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ActuatorCyclic.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
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


#include "kortex_driver/msg/actuator_cyclic_message_id.hpp"
#include "kortex_driver/msg/actuator_cyclic_command.hpp"
#include "kortex_driver/msg/actuator_cyclic_feedback.hpp"
#include "kortex_driver/msg/actuator_cyclic_custom_data.hpp"


int ToRosData(Kinova::Api::ActuatorCyclic::MessageId input, kortex_driver::msg::ActuatorCyclicMessageId &output);
int ToRosData(Kinova::Api::ActuatorCyclic::Command input, kortex_driver::msg::ActuatorCyclicCommand &output);
int ToRosData(Kinova::Api::ActuatorCyclic::Feedback input, kortex_driver::msg::ActuatorCyclicFeedback &output);
int ToRosData(Kinova::Api::ActuatorCyclic::CustomData input, kortex_driver::msg::ActuatorCyclicCustomData &output);

#endif