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
 
#ifndef _KORTEX_COMMON_ROS_CONVERTER_H_
#define _KORTEX_COMMON_ROS_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Common.pb.h>

#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
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


#include "kortex_driver/msg/device_handle.hpp"
#include "kortex_driver/msg/empty.hpp"
#include "kortex_driver/msg/notification_options.hpp"
#include "kortex_driver/msg/safety_handle.hpp"
#include "kortex_driver/msg/notification_handle.hpp"
#include "kortex_driver/msg/safety_notification.hpp"
#include "kortex_driver/msg/timestamp.hpp"
#include "kortex_driver/msg/user_profile_handle.hpp"
#include "kortex_driver/msg/connection.hpp"
#include "kortex_driver/msg/uart_configuration.hpp"
#include "kortex_driver/msg/uart_device_identification.hpp"
#include "kortex_driver/msg/country_code.hpp"


int ToRosData(Kinova::Api::Common::DeviceHandle input, kortex_driver::msg::DeviceHandle &output);
int ToRosData(Kinova::Api::Common::Empty input, kortex_driver::msg::Empty &output);
int ToRosData(Kinova::Api::Common::NotificationOptions input, kortex_driver::msg::NotificationOptions &output);
int ToRosData(Kinova::Api::Common::SafetyHandle input, kortex_driver::msg::SafetyHandle &output);
int ToRosData(Kinova::Api::Common::NotificationHandle input, kortex_driver::msg::NotificationHandle &output);
int ToRosData(Kinova::Api::Common::SafetyNotification input, kortex_driver::msg::SafetyNotification &output);
int ToRosData(Kinova::Api::Common::Timestamp input, kortex_driver::msg::Timestamp &output);
int ToRosData(Kinova::Api::Common::UserProfileHandle input, kortex_driver::msg::UserProfileHandle &output);
int ToRosData(Kinova::Api::Common::Connection input, kortex_driver::msg::Connection &output);
int ToRosData(Kinova::Api::Common::UARTConfiguration input, kortex_driver::msg::UARTConfiguration &output);
int ToRosData(Kinova::Api::Common::UARTDeviceIdentification input, kortex_driver::msg::UARTDeviceIdentification &output);
int ToRosData(Kinova::Api::Common::CountryCode input, kortex_driver::msg::CountryCode &output);

#endif