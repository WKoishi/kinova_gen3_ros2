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
 
#ifndef _KORTEX_INTERCONNECTCONFIG_ROS_CONVERTER_H_
#define _KORTEX_INTERCONNECTCONFIG_ROS_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <InterconnectConfig.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
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
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


#include "kortex_driver/msg/ethernet_device_identification.hpp"
#include "kortex_driver/msg/ethernet_configuration.hpp"
#include "kortex_driver/msg/gpio_identification.hpp"
#include "kortex_driver/msg/interconnect_config_gpio_configuration.hpp"
#include "kortex_driver/msg/gpio_state.hpp"
#include "kortex_driver/msg/i2_c_device_identification.hpp"
#include "kortex_driver/msg/i2_c_configuration.hpp"
#include "kortex_driver/msg/i2_c_read_parameter.hpp"
#include "kortex_driver/msg/i2_c_read_register_parameter.hpp"
#include "kortex_driver/msg/i2_c_write_parameter.hpp"
#include "kortex_driver/msg/i2_c_write_register_parameter.hpp"
#include "kortex_driver/msg/i2_c_data.hpp"


int ToRosData(Kinova::Api::InterconnectConfig::EthernetDeviceIdentification input, kortex_driver::msg::EthernetDeviceIdentification &output);
int ToRosData(Kinova::Api::InterconnectConfig::EthernetConfiguration input, kortex_driver::msg::EthernetConfiguration &output);
int ToRosData(Kinova::Api::InterconnectConfig::GPIOIdentification input, kortex_driver::msg::GPIOIdentification &output);
int ToRosData(Kinova::Api::InterconnectConfig::GPIOConfiguration input, kortex_driver::msg::InterconnectConfigGPIOConfiguration &output);
int ToRosData(Kinova::Api::InterconnectConfig::GPIOState input, kortex_driver::msg::GPIOState &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CDeviceIdentification input, kortex_driver::msg::I2CDeviceIdentification &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CConfiguration input, kortex_driver::msg::I2CConfiguration &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CReadParameter input, kortex_driver::msg::I2CReadParameter &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CReadRegisterParameter input, kortex_driver::msg::I2CReadRegisterParameter &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CWriteParameter input, kortex_driver::msg::I2CWriteParameter &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CWriteRegisterParameter input, kortex_driver::msg::I2CWriteRegisterParameter &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CData input, kortex_driver::msg::I2CData &output);

#endif