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
 
#ifndef _KORTEX_INTERCONNECTCONFIG_ROBOT_SERVICES_H_
#define _KORTEX_INTERCONNECTCONFIG_ROBOT_SERVICES_H_

#include "kortex_driver/generated/interfaces/interconnectconfig_services_interface.h"

#include <InterconnectConfig.pb.h>
#include <InterconnectConfigClientRpc.h>

using namespace std;

class InterconnectConfigRobotServices : public IInterconnectConfigServices
{
    public:
        InterconnectConfigRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::InterconnectConfig::InterconnectConfigClient* interconnectconfig, uint32_t device_id, uint32_t timeout_ms);

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) override;
        virtual bool GetUARTConfiguration(kortex_driver::srv::GetUARTConfiguration::Request  &req, kortex_driver::srv::GetUARTConfiguration::Response &res) override;
        virtual bool SetUARTConfiguration(kortex_driver::srv::SetUARTConfiguration::Request  &req, kortex_driver::srv::SetUARTConfiguration::Response &res) override;
        virtual bool GetEthernetConfiguration(kortex_driver::srv::GetEthernetConfiguration::Request  &req, kortex_driver::srv::GetEthernetConfiguration::Response &res) override;
        virtual bool SetEthernetConfiguration(kortex_driver::srv::SetEthernetConfiguration::Request  &req, kortex_driver::srv::SetEthernetConfiguration::Response &res) override;
        virtual bool GetGPIOConfiguration(kortex_driver::srv::GetGPIOConfiguration::Request  &req, kortex_driver::srv::GetGPIOConfiguration::Response &res) override;
        virtual bool SetGPIOConfiguration(kortex_driver::srv::SetGPIOConfiguration::Request  &req, kortex_driver::srv::SetGPIOConfiguration::Response &res) override;
        virtual bool GetGPIOState(kortex_driver::srv::GetGPIOState::Request  &req, kortex_driver::srv::GetGPIOState::Response &res) override;
        virtual bool SetGPIOState(kortex_driver::srv::SetGPIOState::Request  &req, kortex_driver::srv::SetGPIOState::Response &res) override;
        virtual bool GetI2CConfiguration(kortex_driver::srv::GetI2CConfiguration::Request  &req, kortex_driver::srv::GetI2CConfiguration::Response &res) override;
        virtual bool SetI2CConfiguration(kortex_driver::srv::SetI2CConfiguration::Request  &req, kortex_driver::srv::SetI2CConfiguration::Response &res) override;
        virtual bool I2CRead(kortex_driver::srv::I2CRead::Request  &req, kortex_driver::srv::I2CRead::Response &res) override;
        virtual bool I2CReadRegister(kortex_driver::srv::I2CReadRegister::Request  &req, kortex_driver::srv::I2CReadRegister::Response &res) override;
        virtual bool I2CWrite(kortex_driver::srv::I2CWrite::Request  &req, kortex_driver::srv::I2CWrite::Response &res) override;
        virtual bool I2CWriteRegister(kortex_driver::srv::I2CWriteRegister::Request  &req, kortex_driver::srv::I2CWriteRegister::Response &res) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::InterconnectConfig::InterconnectConfigClient* m_interconnectconfig;
};
#endif
