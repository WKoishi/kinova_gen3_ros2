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
 
#ifndef _KORTEX_INTERCONNECTCONFIG_PROTO_CONVERTER_H_
#define _KORTEX_INTERCONNECTCONFIG_PROTO_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <InterconnectConfig.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
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
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


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


int ToProtoData(kortex_driver::msg::EthernetDeviceIdentification input, Kinova::Api::InterconnectConfig::EthernetDeviceIdentification *output);
int ToProtoData(kortex_driver::msg::EthernetConfiguration input, Kinova::Api::InterconnectConfig::EthernetConfiguration *output);
int ToProtoData(kortex_driver::msg::GPIOIdentification input, Kinova::Api::InterconnectConfig::GPIOIdentification *output);
int ToProtoData(kortex_driver::msg::InterconnectConfigGPIOConfiguration input, Kinova::Api::InterconnectConfig::GPIOConfiguration *output);
int ToProtoData(kortex_driver::msg::GPIOState input, Kinova::Api::InterconnectConfig::GPIOState *output);
int ToProtoData(kortex_driver::msg::I2CDeviceIdentification input, Kinova::Api::InterconnectConfig::I2CDeviceIdentification *output);
int ToProtoData(kortex_driver::msg::I2CConfiguration input, Kinova::Api::InterconnectConfig::I2CConfiguration *output);
int ToProtoData(kortex_driver::msg::I2CReadParameter input, Kinova::Api::InterconnectConfig::I2CReadParameter *output);
int ToProtoData(kortex_driver::msg::I2CReadRegisterParameter input, Kinova::Api::InterconnectConfig::I2CReadRegisterParameter *output);
int ToProtoData(kortex_driver::msg::I2CWriteParameter input, Kinova::Api::InterconnectConfig::I2CWriteParameter *output);
int ToProtoData(kortex_driver::msg::I2CWriteRegisterParameter input, Kinova::Api::InterconnectConfig::I2CWriteRegisterParameter *output);
int ToProtoData(kortex_driver::msg::I2CData input, Kinova::Api::InterconnectConfig::I2CData *output);

#endif