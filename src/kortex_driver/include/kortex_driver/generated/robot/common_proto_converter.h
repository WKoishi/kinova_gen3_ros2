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
 
#ifndef _KORTEX_COMMON_PROTO_CONVERTER_H_
#define _KORTEX_COMMON_PROTO_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Common.pb.h>

#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
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


int ToProtoData(kortex_driver::msg::DeviceHandle input, Kinova::Api::Common::DeviceHandle *output);
int ToProtoData(kortex_driver::msg::Empty input, Kinova::Api::Common::Empty *output);
int ToProtoData(kortex_driver::msg::NotificationOptions input, Kinova::Api::Common::NotificationOptions *output);
int ToProtoData(kortex_driver::msg::SafetyHandle input, Kinova::Api::Common::SafetyHandle *output);
int ToProtoData(kortex_driver::msg::NotificationHandle input, Kinova::Api::Common::NotificationHandle *output);
int ToProtoData(kortex_driver::msg::SafetyNotification input, Kinova::Api::Common::SafetyNotification *output);
int ToProtoData(kortex_driver::msg::Timestamp input, Kinova::Api::Common::Timestamp *output);
int ToProtoData(kortex_driver::msg::UserProfileHandle input, Kinova::Api::Common::UserProfileHandle *output);
int ToProtoData(kortex_driver::msg::Connection input, Kinova::Api::Common::Connection *output);
int ToProtoData(kortex_driver::msg::UARTConfiguration input, Kinova::Api::Common::UARTConfiguration *output);
int ToProtoData(kortex_driver::msg::UARTDeviceIdentification input, Kinova::Api::Common::UARTDeviceIdentification *output);
int ToProtoData(kortex_driver::msg::CountryCode input, Kinova::Api::Common::CountryCode *output);

#endif