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
 
#ifndef _KORTEX_DEVICEMANAGER_SERVICES_INTERFACE_H_
#define _KORTEX_DEVICEMANAGER_SERVICES_INTERFACE_H_

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>
#include "kortex_driver/srv/read_all_devices.hpp"

#include "kortex_driver/msg/kortex_error.hpp"
#include "kortex_driver/srv/set_device_id.hpp"
#include "kortex_driver/srv/set_api_options.hpp"
#include "kortex_driver/msg/api_options.hpp"

using namespace std;

class IDeviceManagerServices
{
    public:
        IDeviceManagerServices(rclcpp::Node::SharedPtr node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) = 0;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) = 0;
        virtual bool ReadAllDevices(kortex_driver::srv::ReadAllDevices::Request  &req, kortex_driver::srv::ReadAllDevices::Response &res) = 0;

protected:
        rclcpp::Node::SharedPtr m_node_handle;
        rclcpp::Publisher<kortex_driver::msg::KortexError>::SharedPtr m_pub_Error;

        rclcpp::Service<kortex_driver::srv::SetDeviceID>::SharedPtr m_serviceSetDeviceID;
        rclcpp::Service<kortex_driver::srv::SetApiOptions>::SharedPtr m_serviceSetApiOptions;

	rclcpp::Service<kortex_driver::srv::ReadAllDevices>::SharedPtr m_serviceReadAllDevices;
};
#endif
