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
 
#ifndef _KORTEX_DEVICECONFIG_PROTO_CONVERTER_H_
#define _KORTEX_DEVICECONFIG_PROTO_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <DeviceConfig.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


#include "kortex_driver/msg/device_type.hpp"
#include "kortex_driver/msg/run_mode.hpp"
#include "kortex_driver/msg/firmware_version.hpp"
#include "kortex_driver/msg/bootloader_version.hpp"
#include "kortex_driver/msg/model_number.hpp"
#include "kortex_driver/msg/part_number.hpp"
#include "kortex_driver/msg/serial_number.hpp"
#include "kortex_driver/msg/mac_address.hpp"
#include "kortex_driver/msg/i_pv4_settings.hpp"
#include "kortex_driver/msg/part_number_revision.hpp"
#include "kortex_driver/msg/power_on_self_test_result.hpp"
#include "kortex_driver/msg/reboot_rqst.hpp"
#include "kortex_driver/msg/safety_information.hpp"
#include "kortex_driver/msg/safety_information_list.hpp"
#include "kortex_driver/msg/safety_enable.hpp"
#include "kortex_driver/msg/safety_threshold.hpp"
#include "kortex_driver/msg/safety_configuration.hpp"
#include "kortex_driver/msg/safety_configuration_list.hpp"
#include "kortex_driver/msg/safety_status.hpp"
#include "kortex_driver/msg/calibration_parameter.hpp"
#include "kortex_driver/msg/calibration.hpp"
#include "kortex_driver/msg/calibration_element.hpp"
#include "kortex_driver/msg/calibration_result.hpp"
#include "kortex_driver/msg/device_config_cap_sense_config.hpp"
#include "kortex_driver/msg/cap_sense_register.hpp"


int ToProtoData(kortex_driver::msg::DeviceType input, Kinova::Api::DeviceConfig::DeviceType *output);
int ToProtoData(kortex_driver::msg::RunMode input, Kinova::Api::DeviceConfig::RunMode *output);
int ToProtoData(kortex_driver::msg::FirmwareVersion input, Kinova::Api::DeviceConfig::FirmwareVersion *output);
int ToProtoData(kortex_driver::msg::BootloaderVersion input, Kinova::Api::DeviceConfig::BootloaderVersion *output);
int ToProtoData(kortex_driver::msg::ModelNumber input, Kinova::Api::DeviceConfig::ModelNumber *output);
int ToProtoData(kortex_driver::msg::PartNumber input, Kinova::Api::DeviceConfig::PartNumber *output);
int ToProtoData(kortex_driver::msg::SerialNumber input, Kinova::Api::DeviceConfig::SerialNumber *output);
int ToProtoData(kortex_driver::msg::MACAddress input, Kinova::Api::DeviceConfig::MACAddress *output);
int ToProtoData(kortex_driver::msg::IPv4Settings input, Kinova::Api::DeviceConfig::IPv4Settings *output);
int ToProtoData(kortex_driver::msg::PartNumberRevision input, Kinova::Api::DeviceConfig::PartNumberRevision *output);
int ToProtoData(kortex_driver::msg::PowerOnSelfTestResult input, Kinova::Api::DeviceConfig::PowerOnSelfTestResult *output);
int ToProtoData(kortex_driver::msg::RebootRqst input, Kinova::Api::DeviceConfig::RebootRqst *output);
int ToProtoData(kortex_driver::msg::SafetyInformation input, Kinova::Api::DeviceConfig::SafetyInformation *output);
int ToProtoData(kortex_driver::msg::SafetyInformationList input, Kinova::Api::DeviceConfig::SafetyInformationList *output);
int ToProtoData(kortex_driver::msg::SafetyEnable input, Kinova::Api::DeviceConfig::SafetyEnable *output);
int ToProtoData(kortex_driver::msg::SafetyThreshold input, Kinova::Api::DeviceConfig::SafetyThreshold *output);
int ToProtoData(kortex_driver::msg::SafetyConfiguration input, Kinova::Api::DeviceConfig::SafetyConfiguration *output);
int ToProtoData(kortex_driver::msg::SafetyConfigurationList input, Kinova::Api::DeviceConfig::SafetyConfigurationList *output);
int ToProtoData(kortex_driver::msg::SafetyStatus input, Kinova::Api::DeviceConfig::SafetyStatus *output);
int ToProtoData(kortex_driver::msg::CalibrationParameter input, Kinova::Api::DeviceConfig::CalibrationParameter *output);
int ToProtoData(kortex_driver::msg::Calibration input, Kinova::Api::DeviceConfig::Calibration *output);
int ToProtoData(kortex_driver::msg::CalibrationElement input, Kinova::Api::DeviceConfig::CalibrationElement *output);
int ToProtoData(kortex_driver::msg::CalibrationResult input, Kinova::Api::DeviceConfig::CalibrationResult *output);
int ToProtoData(kortex_driver::msg::DeviceConfigCapSenseConfig input, Kinova::Api::DeviceConfig::CapSenseConfig *output);
int ToProtoData(kortex_driver::msg::CapSenseRegister input, Kinova::Api::DeviceConfig::CapSenseRegister *output);

#endif