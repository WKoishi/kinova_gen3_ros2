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
 
#ifndef _KORTEX_ACTUATORCONFIG_SIMULATION_SERVICES_H_
#define _KORTEX_ACTUATORCONFIG_SIMULATION_SERVICES_H_

#include "kortex_driver/generated/interfaces/actuatorconfig_services_interface.h"

using namespace std;

class ActuatorConfigSimulationServices : public IActuatorConfigServices
{
    public:
        ActuatorConfigSimulationServices(rclcpp::Node::SharedPtr node_handle);

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) override;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) override;
        std::function<kortex_driver::srv::GetAxisOffsets::Response(const kortex_driver::srv::GetAxisOffsets::Request&)> GetAxisOffsetsHandler = nullptr;
        virtual bool GetAxisOffsets(const std::shared_ptr<kortex_driver::srv::GetAxisOffsets::Request> req, std::shared_ptr<kortex_driver::srv::GetAxisOffsets::Response> res) override;
        std::function<kortex_driver::srv::SetAxisOffsets::Response(const kortex_driver::srv::SetAxisOffsets::Request&)> SetAxisOffsetsHandler = nullptr;
        virtual bool SetAxisOffsets(const std::shared_ptr<kortex_driver::srv::SetAxisOffsets::Request> req, std::shared_ptr<kortex_driver::srv::SetAxisOffsets::Response> res) override;
        std::function<kortex_driver::srv::SetTorqueOffset::Response(const kortex_driver::srv::SetTorqueOffset::Request&)> SetTorqueOffsetHandler = nullptr;
        virtual bool SetTorqueOffset(const std::shared_ptr<kortex_driver::srv::SetTorqueOffset::Request> req, std::shared_ptr<kortex_driver::srv::SetTorqueOffset::Response> res) override;
        std::function<kortex_driver::srv::ActuatorConfigGetControlMode::Response(const kortex_driver::srv::ActuatorConfigGetControlMode::Request&)> ActuatorConfig_GetControlModeHandler = nullptr;
        virtual bool ActuatorConfig_GetControlMode(const std::shared_ptr<kortex_driver::srv::ActuatorConfigGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::ActuatorConfigGetControlMode::Response> res) override;
        std::function<kortex_driver::srv::SetControlMode::Response(const kortex_driver::srv::SetControlMode::Request&)> SetControlModeHandler = nullptr;
        virtual bool SetControlMode(const std::shared_ptr<kortex_driver::srv::SetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::SetControlMode::Response> res) override;
        std::function<kortex_driver::srv::GetActivatedControlLoop::Response(const kortex_driver::srv::GetActivatedControlLoop::Request&)> GetActivatedControlLoopHandler = nullptr;
        virtual bool GetActivatedControlLoop(const std::shared_ptr<kortex_driver::srv::GetActivatedControlLoop::Request> req, std::shared_ptr<kortex_driver::srv::GetActivatedControlLoop::Response> res) override;
        std::function<kortex_driver::srv::SetActivatedControlLoop::Response(const kortex_driver::srv::SetActivatedControlLoop::Request&)> SetActivatedControlLoopHandler = nullptr;
        virtual bool SetActivatedControlLoop(const std::shared_ptr<kortex_driver::srv::SetActivatedControlLoop::Request> req, std::shared_ptr<kortex_driver::srv::SetActivatedControlLoop::Response> res) override;
        std::function<kortex_driver::srv::GetControlLoopParameters::Response(const kortex_driver::srv::GetControlLoopParameters::Request&)> GetControlLoopParametersHandler = nullptr;
        virtual bool GetControlLoopParameters(const std::shared_ptr<kortex_driver::srv::GetControlLoopParameters::Request> req, std::shared_ptr<kortex_driver::srv::GetControlLoopParameters::Response> res) override;
        std::function<kortex_driver::srv::SetControlLoopParameters::Response(const kortex_driver::srv::SetControlLoopParameters::Request&)> SetControlLoopParametersHandler = nullptr;
        virtual bool SetControlLoopParameters(const std::shared_ptr<kortex_driver::srv::SetControlLoopParameters::Request> req, std::shared_ptr<kortex_driver::srv::SetControlLoopParameters::Response> res) override;
        std::function<kortex_driver::srv::SelectCustomData::Response(const kortex_driver::srv::SelectCustomData::Request&)> SelectCustomDataHandler = nullptr;
        virtual bool SelectCustomData(const std::shared_ptr<kortex_driver::srv::SelectCustomData::Request> req, std::shared_ptr<kortex_driver::srv::SelectCustomData::Response> res) override;
        std::function<kortex_driver::srv::GetSelectedCustomData::Response(const kortex_driver::srv::GetSelectedCustomData::Request&)> GetSelectedCustomDataHandler = nullptr;
        virtual bool GetSelectedCustomData(const std::shared_ptr<kortex_driver::srv::GetSelectedCustomData::Request> req, std::shared_ptr<kortex_driver::srv::GetSelectedCustomData::Response> res) override;
        std::function<kortex_driver::srv::SetCommandMode::Response(const kortex_driver::srv::SetCommandMode::Request&)> SetCommandModeHandler = nullptr;
        virtual bool SetCommandMode(const std::shared_ptr<kortex_driver::srv::SetCommandMode::Request> req, std::shared_ptr<kortex_driver::srv::SetCommandMode::Response> res) override;
        std::function<kortex_driver::srv::ActuatorConfigClearFaults::Response(const kortex_driver::srv::ActuatorConfigClearFaults::Request&)> ActuatorConfig_ClearFaultsHandler = nullptr;
        virtual bool ActuatorConfig_ClearFaults(const std::shared_ptr<kortex_driver::srv::ActuatorConfigClearFaults::Request> req, std::shared_ptr<kortex_driver::srv::ActuatorConfigClearFaults::Response> res) override;
        std::function<kortex_driver::srv::SetServoing::Response(const kortex_driver::srv::SetServoing::Request&)> SetServoingHandler = nullptr;
        virtual bool SetServoing(const std::shared_ptr<kortex_driver::srv::SetServoing::Request> req, std::shared_ptr<kortex_driver::srv::SetServoing::Response> res) override;
        std::function<kortex_driver::srv::MoveToPosition::Response(const kortex_driver::srv::MoveToPosition::Request&)> MoveToPositionHandler = nullptr;
        virtual bool MoveToPosition(const std::shared_ptr<kortex_driver::srv::MoveToPosition::Request> req, std::shared_ptr<kortex_driver::srv::MoveToPosition::Response> res) override;
        std::function<kortex_driver::srv::GetCommandMode::Response(const kortex_driver::srv::GetCommandMode::Request&)> GetCommandModeHandler = nullptr;
        virtual bool GetCommandMode(const std::shared_ptr<kortex_driver::srv::GetCommandMode::Request> req, std::shared_ptr<kortex_driver::srv::GetCommandMode::Response> res) override;
        std::function<kortex_driver::srv::GetServoing::Response(const kortex_driver::srv::GetServoing::Request&)> GetServoingHandler = nullptr;
        virtual bool GetServoing(const std::shared_ptr<kortex_driver::srv::GetServoing::Request> req, std::shared_ptr<kortex_driver::srv::GetServoing::Response> res) override;
        std::function<kortex_driver::srv::GetTorqueOffset::Response(const kortex_driver::srv::GetTorqueOffset::Request&)> GetTorqueOffsetHandler = nullptr;
        virtual bool GetTorqueOffset(const std::shared_ptr<kortex_driver::srv::GetTorqueOffset::Request> req, std::shared_ptr<kortex_driver::srv::GetTorqueOffset::Response> res) override;
        std::function<kortex_driver::srv::SetCoggingFeedforwardMode::Response(const kortex_driver::srv::SetCoggingFeedforwardMode::Request&)> SetCoggingFeedforwardModeHandler = nullptr;
        virtual bool SetCoggingFeedforwardMode(const std::shared_ptr<kortex_driver::srv::SetCoggingFeedforwardMode::Request> req, std::shared_ptr<kortex_driver::srv::SetCoggingFeedforwardMode::Response> res) override;
        std::function<kortex_driver::srv::GetCoggingFeedforwardMode::Response(const kortex_driver::srv::GetCoggingFeedforwardMode::Request&)> GetCoggingFeedforwardModeHandler = nullptr;
        virtual bool GetCoggingFeedforwardMode(const std::shared_ptr<kortex_driver::srv::GetCoggingFeedforwardMode::Request> req, std::shared_ptr<kortex_driver::srv::GetCoggingFeedforwardMode::Response> res) override;

};
#endif
