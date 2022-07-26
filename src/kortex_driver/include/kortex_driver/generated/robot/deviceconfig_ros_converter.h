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
 
#ifndef _KORTEX_DEVICECONFIG_ROS_CONVERTER_H_
#define _KORTEX_DEVICECONFIG_ROS_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <DeviceConfig.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


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


int ToRosData(Kinova::Api::DeviceConfig::DeviceType input, kortex_driver::msg::DeviceType &output);
int ToRosData(Kinova::Api::DeviceConfig::RunMode input, kortex_driver::msg::RunMode &output);
int ToRosData(Kinova::Api::DeviceConfig::FirmwareVersion input, kortex_driver::msg::FirmwareVersion &output);
int ToRosData(Kinova::Api::DeviceConfig::BootloaderVersion input, kortex_driver::msg::BootloaderVersion &output);
int ToRosData(Kinova::Api::DeviceConfig::ModelNumber input, kortex_driver::msg::ModelNumber &output);
int ToRosData(Kinova::Api::DeviceConfig::PartNumber input, kortex_driver::msg::PartNumber &output);
int ToRosData(Kinova::Api::DeviceConfig::SerialNumber input, kortex_driver::msg::SerialNumber &output);
int ToRosData(Kinova::Api::DeviceConfig::MACAddress input, kortex_driver::msg::MACAddress &output);
int ToRosData(Kinova::Api::DeviceConfig::IPv4Settings input, kortex_driver::msg::IPv4Settings &output);
int ToRosData(Kinova::Api::DeviceConfig::PartNumberRevision input, kortex_driver::msg::PartNumberRevision &output);
int ToRosData(Kinova::Api::DeviceConfig::PowerOnSelfTestResult input, kortex_driver::msg::PowerOnSelfTestResult &output);
int ToRosData(Kinova::Api::DeviceConfig::RebootRqst input, kortex_driver::msg::RebootRqst &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyInformation input, kortex_driver::msg::SafetyInformation &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyInformationList input, kortex_driver::msg::SafetyInformationList &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyEnable input, kortex_driver::msg::SafetyEnable &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyThreshold input, kortex_driver::msg::SafetyThreshold &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyConfiguration input, kortex_driver::msg::SafetyConfiguration &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyConfigurationList input, kortex_driver::msg::SafetyConfigurationList &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyStatus input, kortex_driver::msg::SafetyStatus &output);
int ToRosData(Kinova::Api::DeviceConfig::CalibrationParameter input, kortex_driver::msg::CalibrationParameter &output);
int ToRosData(Kinova::Api::DeviceConfig::Calibration input, kortex_driver::msg::Calibration &output);
int ToRosData(Kinova::Api::DeviceConfig::CalibrationElement input, kortex_driver::msg::CalibrationElement &output);
int ToRosData(Kinova::Api::DeviceConfig::CalibrationResult input, kortex_driver::msg::CalibrationResult &output);
int ToRosData(Kinova::Api::DeviceConfig::CapSenseConfig input, kortex_driver::msg::DeviceConfigCapSenseConfig &output);
int ToRosData(Kinova::Api::DeviceConfig::CapSenseRegister input, kortex_driver::msg::CapSenseRegister &output);

#endif