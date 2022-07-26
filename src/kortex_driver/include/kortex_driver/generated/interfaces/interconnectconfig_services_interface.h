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
 
#ifndef _KORTEX_INTERCONNECTCONFIG_SERVICES_INTERFACE_H_
#define _KORTEX_INTERCONNECTCONFIG_SERVICES_INTERFACE_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>
#include "kortex_driver/srv/get_uart_configuration.hpp"
#include "kortex_driver/srv/set_uart_configuration.hpp"
#include "kortex_driver/srv/get_ethernet_configuration.hpp"
#include "kortex_driver/srv/set_ethernet_configuration.hpp"
#include "kortex_driver/srv/get_gpio_configuration.hpp"
#include "kortex_driver/srv/set_gpio_configuration.hpp"
#include "kortex_driver/srv/get_gpio_state.hpp"
#include "kortex_driver/srv/set_gpio_state.hpp"
#include "kortex_driver/GetI2CConfiguration.h"
#include "kortex_driver/SetI2CConfiguration.h"
#include "kortex_driver/I2CRead.h"
#include "kortex_driver/I2CReadRegister.h"
#include "kortex_driver/I2CWrite.h"
#include "kortex_driver/I2CWriteRegister.h"

#include "kortex_driver/msg/kortex_error.hpp"
#include "kortex_driver/srv/set_device_id.hpp"
#include "kortex_driver/srv/set_api_options.hpp"
#include "kortex_driver/msg/api_options.hpp"

using namespace std;

class IInterconnectConfigServices
{
    public:
        IInterconnectConfigServices(ros::NodeHandle& node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) = 0;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) = 0;
        virtual bool GetUARTConfiguration(kortex_driver::srv::GetUARTConfiguration::Request  &req, kortex_driver::srv::GetUARTConfiguration::Response &res) = 0;
        virtual bool SetUARTConfiguration(kortex_driver::srv::SetUARTConfiguration::Request  &req, kortex_driver::srv::SetUARTConfiguration::Response &res) = 0;
        virtual bool GetEthernetConfiguration(kortex_driver::srv::GetEthernetConfiguration::Request  &req, kortex_driver::srv::GetEthernetConfiguration::Response &res) = 0;
        virtual bool SetEthernetConfiguration(kortex_driver::srv::SetEthernetConfiguration::Request  &req, kortex_driver::srv::SetEthernetConfiguration::Response &res) = 0;
        virtual bool GetGPIOConfiguration(kortex_driver::srv::GetGPIOConfiguration::Request  &req, kortex_driver::srv::GetGPIOConfiguration::Response &res) = 0;
        virtual bool SetGPIOConfiguration(kortex_driver::srv::SetGPIOConfiguration::Request  &req, kortex_driver::srv::SetGPIOConfiguration::Response &res) = 0;
        virtual bool GetGPIOState(kortex_driver::srv::GetGPIOState::Request  &req, kortex_driver::srv::GetGPIOState::Response &res) = 0;
        virtual bool SetGPIOState(kortex_driver::srv::SetGPIOState::Request  &req, kortex_driver::srv::SetGPIOState::Response &res) = 0;
        virtual bool GetI2CConfiguration(kortex_driver::srv::GetI2CConfiguration::Request  &req, kortex_driver::srv::GetI2CConfiguration::Response &res) = 0;
        virtual bool SetI2CConfiguration(kortex_driver::srv::SetI2CConfiguration::Request  &req, kortex_driver::srv::SetI2CConfiguration::Response &res) = 0;
        virtual bool I2CRead(kortex_driver::srv::I2CRead::Request  &req, kortex_driver::srv::I2CRead::Response &res) = 0;
        virtual bool I2CReadRegister(kortex_driver::srv::I2CReadRegister::Request  &req, kortex_driver::srv::I2CReadRegister::Response &res) = 0;
        virtual bool I2CWrite(kortex_driver::srv::I2CWrite::Request  &req, kortex_driver::srv::I2CWrite::Response &res) = 0;
        virtual bool I2CWriteRegister(kortex_driver::srv::I2CWriteRegister::Request  &req, kortex_driver::srv::I2CWriteRegister::Response &res) = 0;

protected:
        ros::NodeHandle m_node_handle;
        ros::Publisher m_pub_Error;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceGetUARTConfiguration;
	ros::ServiceServer m_serviceSetUARTConfiguration;
	ros::ServiceServer m_serviceGetEthernetConfiguration;
	ros::ServiceServer m_serviceSetEthernetConfiguration;
	ros::ServiceServer m_serviceGetGPIOConfiguration;
	ros::ServiceServer m_serviceSetGPIOConfiguration;
	ros::ServiceServer m_serviceGetGPIOState;
	ros::ServiceServer m_serviceSetGPIOState;
	ros::ServiceServer m_serviceGetI2CConfiguration;
	ros::ServiceServer m_serviceSetI2CConfiguration;
	ros::ServiceServer m_serviceI2CRead;
	ros::ServiceServer m_serviceI2CReadRegister;
	ros::ServiceServer m_serviceI2CWrite;
	ros::ServiceServer m_serviceI2CWriteRegister;
};
#endif
