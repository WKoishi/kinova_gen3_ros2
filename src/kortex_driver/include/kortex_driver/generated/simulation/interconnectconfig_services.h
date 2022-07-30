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
 
#ifndef _KORTEX_INTERCONNECTCONFIG_SIMULATION_SERVICES_H_
#define _KORTEX_INTERCONNECTCONFIG_SIMULATION_SERVICES_H_

#include "kortex_driver/generated/interfaces/interconnectconfig_services_interface.h"

using namespace std;

class InterconnectConfigSimulationServices : public IInterconnectConfigServices
{
    public:
        InterconnectConfigSimulationServices(rclcpp::Node::SharedPtr node_handle);

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) override;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) override;
        std::function<kortex_driver::srv::GetUARTConfiguration::Response(const kortex_driver::srv::GetUARTConfiguration::Request&)> GetUARTConfigurationHandler = nullptr;
        virtual bool GetUARTConfiguration(const std::shared_ptr<kortex_driver::srv::GetUARTConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetUARTConfiguration::Response> res) override;
        std::function<kortex_driver::srv::SetUARTConfiguration::Response(const kortex_driver::srv::SetUARTConfiguration::Request&)> SetUARTConfigurationHandler = nullptr;
        virtual bool SetUARTConfiguration(const std::shared_ptr<kortex_driver::srv::SetUARTConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetUARTConfiguration::Response> res) override;
        std::function<kortex_driver::srv::GetEthernetConfiguration::Response(const kortex_driver::srv::GetEthernetConfiguration::Request&)> GetEthernetConfigurationHandler = nullptr;
        virtual bool GetEthernetConfiguration(const std::shared_ptr<kortex_driver::srv::GetEthernetConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetEthernetConfiguration::Response> res) override;
        std::function<kortex_driver::srv::SetEthernetConfiguration::Response(const kortex_driver::srv::SetEthernetConfiguration::Request&)> SetEthernetConfigurationHandler = nullptr;
        virtual bool SetEthernetConfiguration(const std::shared_ptr<kortex_driver::srv::SetEthernetConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetEthernetConfiguration::Response> res) override;
        std::function<kortex_driver::srv::GetGPIOConfiguration::Response(const kortex_driver::srv::GetGPIOConfiguration::Request&)> GetGPIOConfigurationHandler = nullptr;
        virtual bool GetGPIOConfiguration(const std::shared_ptr<kortex_driver::srv::GetGPIOConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetGPIOConfiguration::Response> res) override;
        std::function<kortex_driver::srv::SetGPIOConfiguration::Response(const kortex_driver::srv::SetGPIOConfiguration::Request&)> SetGPIOConfigurationHandler = nullptr;
        virtual bool SetGPIOConfiguration(const std::shared_ptr<kortex_driver::srv::SetGPIOConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetGPIOConfiguration::Response> res) override;
        std::function<kortex_driver::srv::GetGPIOState::Response(const kortex_driver::srv::GetGPIOState::Request&)> GetGPIOStateHandler = nullptr;
        virtual bool GetGPIOState(const std::shared_ptr<kortex_driver::srv::GetGPIOState::Request> req, std::shared_ptr<kortex_driver::srv::GetGPIOState::Response> res) override;
        std::function<kortex_driver::srv::SetGPIOState::Response(const kortex_driver::srv::SetGPIOState::Request&)> SetGPIOStateHandler = nullptr;
        virtual bool SetGPIOState(const std::shared_ptr<kortex_driver::srv::SetGPIOState::Request> req, std::shared_ptr<kortex_driver::srv::SetGPIOState::Response> res) override;
        std::function<kortex_driver::srv::GetI2CConfiguration::Response(const kortex_driver::srv::GetI2CConfiguration::Request&)> GetI2CConfigurationHandler = nullptr;
        virtual bool GetI2CConfiguration(const std::shared_ptr<kortex_driver::srv::GetI2CConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetI2CConfiguration::Response> res) override;
        std::function<kortex_driver::srv::SetI2CConfiguration::Response(const kortex_driver::srv::SetI2CConfiguration::Request&)> SetI2CConfigurationHandler = nullptr;
        virtual bool SetI2CConfiguration(const std::shared_ptr<kortex_driver::srv::SetI2CConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetI2CConfiguration::Response> res) override;
        std::function<kortex_driver::srv::I2CRead::Response(const kortex_driver::srv::I2CRead::Request&)> I2CReadHandler = nullptr;
        virtual bool I2CRead(const std::shared_ptr<kortex_driver::srv::I2CRead::Request> req, std::shared_ptr<kortex_driver::srv::I2CRead::Response> res) override;
        std::function<kortex_driver::srv::I2CReadRegister::Response(const kortex_driver::srv::I2CReadRegister::Request&)> I2CReadRegisterHandler = nullptr;
        virtual bool I2CReadRegister(const std::shared_ptr<kortex_driver::srv::I2CReadRegister::Request> req, std::shared_ptr<kortex_driver::srv::I2CReadRegister::Response> res) override;
        std::function<kortex_driver::srv::I2CWrite::Response(const kortex_driver::srv::I2CWrite::Request&)> I2CWriteHandler = nullptr;
        virtual bool I2CWrite(const std::shared_ptr<kortex_driver::srv::I2CWrite::Request> req, std::shared_ptr<kortex_driver::srv::I2CWrite::Response> res) override;
        std::function<kortex_driver::srv::I2CWriteRegister::Response(const kortex_driver::srv::I2CWriteRegister::Request&)> I2CWriteRegisterHandler = nullptr;
        virtual bool I2CWriteRegister(const std::shared_ptr<kortex_driver::srv::I2CWriteRegister::Request> req, std::shared_ptr<kortex_driver::srv::I2CWriteRegister::Response> res) override;

};
#endif
