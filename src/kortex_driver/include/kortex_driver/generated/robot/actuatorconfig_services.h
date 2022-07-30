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
 
#ifndef _KORTEX_ACTUATORCONFIG_ROBOT_SERVICES_H_
#define _KORTEX_ACTUATORCONFIG_ROBOT_SERVICES_H_

#include "kortex_driver/generated/interfaces/actuatorconfig_services_interface.h"

#include <ActuatorConfig.pb.h>
#include <ActuatorConfigClientRpc.h>

using namespace std;

class ActuatorConfigRobotServices : public IActuatorConfigServices
{
    public:
        ActuatorConfigRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::ActuatorConfig::ActuatorConfigClient* actuatorconfig, uint32_t device_id, uint32_t timeout_ms);

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) override;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) override;
        virtual bool GetAxisOffsets(const std::shared_ptr<kortex_driver::srv::GetAxisOffsets::Request> req, std::shared_ptr<kortex_driver::srv::GetAxisOffsets::Response> res) override;
        virtual bool SetAxisOffsets(const std::shared_ptr<kortex_driver::srv::SetAxisOffsets::Request> req, std::shared_ptr<kortex_driver::srv::SetAxisOffsets::Response> res) override;
        virtual bool SetTorqueOffset(const std::shared_ptr<kortex_driver::srv::SetTorqueOffset::Request> req, std::shared_ptr<kortex_driver::srv::SetTorqueOffset::Response> res) override;
        virtual bool ActuatorConfig_GetControlMode(const std::shared_ptr<kortex_driver::srv::ActuatorConfigGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::ActuatorConfigGetControlMode::Response> res) override;
        virtual bool SetControlMode(const std::shared_ptr<kortex_driver::srv::SetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::SetControlMode::Response> res) override;
        virtual bool GetActivatedControlLoop(const std::shared_ptr<kortex_driver::srv::GetActivatedControlLoop::Request> req, std::shared_ptr<kortex_driver::srv::GetActivatedControlLoop::Response> res) override;
        virtual bool SetActivatedControlLoop(const std::shared_ptr<kortex_driver::srv::SetActivatedControlLoop::Request> req, std::shared_ptr<kortex_driver::srv::SetActivatedControlLoop::Response> res) override;
        virtual bool GetControlLoopParameters(const std::shared_ptr<kortex_driver::srv::GetControlLoopParameters::Request> req, std::shared_ptr<kortex_driver::srv::GetControlLoopParameters::Response> res) override;
        virtual bool SetControlLoopParameters(const std::shared_ptr<kortex_driver::srv::SetControlLoopParameters::Request> req, std::shared_ptr<kortex_driver::srv::SetControlLoopParameters::Response> res) override;
        virtual bool SelectCustomData(const std::shared_ptr<kortex_driver::srv::SelectCustomData::Request> req, std::shared_ptr<kortex_driver::srv::SelectCustomData::Response> res) override;
        virtual bool GetSelectedCustomData(const std::shared_ptr<kortex_driver::srv::GetSelectedCustomData::Request> req, std::shared_ptr<kortex_driver::srv::GetSelectedCustomData::Response> res) override;
        virtual bool SetCommandMode(const std::shared_ptr<kortex_driver::srv::SetCommandMode::Request> req, std::shared_ptr<kortex_driver::srv::SetCommandMode::Response> res) override;
        virtual bool ActuatorConfig_ClearFaults(const std::shared_ptr<kortex_driver::srv::ActuatorConfigClearFaults::Request> req, std::shared_ptr<kortex_driver::srv::ActuatorConfigClearFaults::Response> res) override;
        virtual bool SetServoing(const std::shared_ptr<kortex_driver::srv::SetServoing::Request> req, std::shared_ptr<kortex_driver::srv::SetServoing::Response> res) override;
        virtual bool MoveToPosition(const std::shared_ptr<kortex_driver::srv::MoveToPosition::Request> req, std::shared_ptr<kortex_driver::srv::MoveToPosition::Response> res) override;
        virtual bool GetCommandMode(const std::shared_ptr<kortex_driver::srv::GetCommandMode::Request> req, std::shared_ptr<kortex_driver::srv::GetCommandMode::Response> res) override;
        virtual bool GetServoing(const std::shared_ptr<kortex_driver::srv::GetServoing::Request> req, std::shared_ptr<kortex_driver::srv::GetServoing::Response> res) override;
        virtual bool GetTorqueOffset(const std::shared_ptr<kortex_driver::srv::GetTorqueOffset::Request> req, std::shared_ptr<kortex_driver::srv::GetTorqueOffset::Response> res) override;
        virtual bool SetCoggingFeedforwardMode(const std::shared_ptr<kortex_driver::srv::SetCoggingFeedforwardMode::Request> req, std::shared_ptr<kortex_driver::srv::SetCoggingFeedforwardMode::Response> res) override;
        virtual bool GetCoggingFeedforwardMode(const std::shared_ptr<kortex_driver::srv::GetCoggingFeedforwardMode::Request> req, std::shared_ptr<kortex_driver::srv::GetCoggingFeedforwardMode::Response> res) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::ActuatorConfig::ActuatorConfigClient* m_actuatorconfig;
};
#endif
