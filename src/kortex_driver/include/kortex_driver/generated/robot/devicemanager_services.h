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
 
#ifndef _KORTEX_DEVICEMANAGER_ROBOT_SERVICES_H_
#define _KORTEX_DEVICEMANAGER_ROBOT_SERVICES_H_

#include "kortex_driver/generated/interfaces/devicemanager_services_interface.h"

#include <DeviceManager.pb.h>
#include <DeviceManagerClientRpc.h>

using namespace std;

class DeviceManagerRobotServices : public IDeviceManagerServices
{
    public:
        DeviceManagerRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::DeviceManager::DeviceManagerClient* devicemanager, uint32_t device_id, uint32_t timeout_ms);

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) override;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) override;
        virtual bool ReadAllDevices(const std::shared_ptr<kortex_driver::srv::ReadAllDevices::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllDevices::Response> res) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::DeviceManager::DeviceManagerClient* m_devicemanager;
};
#endif
