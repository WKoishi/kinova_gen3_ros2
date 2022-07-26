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
        ActuatorConfigSimulationServices(ros::NodeHandle& node_handle);

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) override;
        std::function<kortex_driver::srv::GetAxisOffsets::Response(const kortex_driver::srv::GetAxisOffsets::Request&)> GetAxisOffsetsHandler = nullptr;
        virtual bool GetAxisOffsets(kortex_driver::srv::GetAxisOffsets::Request  &req, kortex_driver::srv::GetAxisOffsets::Response &res) override;
        std::function<kortex_driver::srv::SetAxisOffsets::Response(const kortex_driver::srv::SetAxisOffsets::Request&)> SetAxisOffsetsHandler = nullptr;
        virtual bool SetAxisOffsets(kortex_driver::srv::SetAxisOffsets::Request  &req, kortex_driver::srv::SetAxisOffsets::Response &res) override;
        std::function<kortex_driver::srv::SetTorqueOffset::Response(const kortex_driver::srv::SetTorqueOffset::Request&)> SetTorqueOffsetHandler = nullptr;
        virtual bool SetTorqueOffset(kortex_driver::srv::SetTorqueOffset::Request  &req, kortex_driver::srv::SetTorqueOffset::Response &res) override;
        std::function<kortex_driver::srv::ActuatorConfigGetControlMode::Response(const kortex_driver::srv::ActuatorConfigGetControlMode::Request&)> ActuatorConfig_GetControlModeHandler = nullptr;
        virtual bool ActuatorConfig_GetControlMode(kortex_driver::srv::ActuatorConfigGetControlMode::Request  &req, kortex_driver::srv::ActuatorConfigGetControlMode::Response &res) override;
        std::function<kortex_driver::srv::SetControlMode::Response(const kortex_driver::srv::SetControlMode::Request&)> SetControlModeHandler = nullptr;
        virtual bool SetControlMode(kortex_driver::srv::SetControlMode::Request  &req, kortex_driver::srv::SetControlMode::Response &res) override;
        std::function<kortex_driver::srv::GetActivatedControlLoop::Response(const kortex_driver::srv::GetActivatedControlLoop::Request&)> GetActivatedControlLoopHandler = nullptr;
        virtual bool GetActivatedControlLoop(kortex_driver::srv::GetActivatedControlLoop::Request  &req, kortex_driver::srv::GetActivatedControlLoop::Response &res) override;
        std::function<kortex_driver::srv::SetActivatedControlLoop::Response(const kortex_driver::srv::SetActivatedControlLoop::Request&)> SetActivatedControlLoopHandler = nullptr;
        virtual bool SetActivatedControlLoop(kortex_driver::srv::SetActivatedControlLoop::Request  &req, kortex_driver::srv::SetActivatedControlLoop::Response &res) override;
        std::function<kortex_driver::srv::GetControlLoopParameters::Response(const kortex_driver::srv::GetControlLoopParameters::Request&)> GetControlLoopParametersHandler = nullptr;
        virtual bool GetControlLoopParameters(kortex_driver::srv::GetControlLoopParameters::Request  &req, kortex_driver::srv::GetControlLoopParameters::Response &res) override;
        std::function<kortex_driver::srv::SetControlLoopParameters::Response(const kortex_driver::srv::SetControlLoopParameters::Request&)> SetControlLoopParametersHandler = nullptr;
        virtual bool SetControlLoopParameters(kortex_driver::srv::SetControlLoopParameters::Request  &req, kortex_driver::srv::SetControlLoopParameters::Response &res) override;
        std::function<kortex_driver::srv::SelectCustomData::Response(const kortex_driver::srv::SelectCustomData::Request&)> SelectCustomDataHandler = nullptr;
        virtual bool SelectCustomData(kortex_driver::srv::SelectCustomData::Request  &req, kortex_driver::srv::SelectCustomData::Response &res) override;
        std::function<kortex_driver::srv::GetSelectedCustomData::Response(const kortex_driver::srv::GetSelectedCustomData::Request&)> GetSelectedCustomDataHandler = nullptr;
        virtual bool GetSelectedCustomData(kortex_driver::srv::GetSelectedCustomData::Request  &req, kortex_driver::srv::GetSelectedCustomData::Response &res) override;
        std::function<kortex_driver::srv::SetCommandMode::Response(const kortex_driver::srv::SetCommandMode::Request&)> SetCommandModeHandler = nullptr;
        virtual bool SetCommandMode(kortex_driver::srv::SetCommandMode::Request  &req, kortex_driver::srv::SetCommandMode::Response &res) override;
        std::function<kortex_driver::srv::ActuatorConfigClearFaults::Response(const kortex_driver::srv::ActuatorConfigClearFaults::Request&)> ActuatorConfig_ClearFaultsHandler = nullptr;
        virtual bool ActuatorConfig_ClearFaults(kortex_driver::srv::ActuatorConfigClearFaults::Request  &req, kortex_driver::srv::ActuatorConfigClearFaults::Response &res) override;
        std::function<kortex_driver::srv::SetServoing::Response(const kortex_driver::srv::SetServoing::Request&)> SetServoingHandler = nullptr;
        virtual bool SetServoing(kortex_driver::srv::SetServoing::Request  &req, kortex_driver::srv::SetServoing::Response &res) override;
        std::function<kortex_driver::srv::MoveToPosition::Response(const kortex_driver::srv::MoveToPosition::Request&)> MoveToPositionHandler = nullptr;
        virtual bool MoveToPosition(kortex_driver::srv::MoveToPosition::Request  &req, kortex_driver::srv::MoveToPosition::Response &res) override;
        std::function<kortex_driver::srv::GetCommandMode::Response(const kortex_driver::srv::GetCommandMode::Request&)> GetCommandModeHandler = nullptr;
        virtual bool GetCommandMode(kortex_driver::srv::GetCommandMode::Request  &req, kortex_driver::srv::GetCommandMode::Response &res) override;
        std::function<kortex_driver::srv::GetServoing::Response(const kortex_driver::srv::GetServoing::Request&)> GetServoingHandler = nullptr;
        virtual bool GetServoing(kortex_driver::srv::GetServoing::Request  &req, kortex_driver::srv::GetServoing::Response &res) override;
        std::function<kortex_driver::srv::GetTorqueOffset::Response(const kortex_driver::srv::GetTorqueOffset::Request&)> GetTorqueOffsetHandler = nullptr;
        virtual bool GetTorqueOffset(kortex_driver::srv::GetTorqueOffset::Request  &req, kortex_driver::srv::GetTorqueOffset::Response &res) override;
        std::function<kortex_driver::srv::SetCoggingFeedforwardMode::Response(const kortex_driver::srv::SetCoggingFeedforwardMode::Request&)> SetCoggingFeedforwardModeHandler = nullptr;
        virtual bool SetCoggingFeedforwardMode(kortex_driver::srv::SetCoggingFeedforwardMode::Request  &req, kortex_driver::srv::SetCoggingFeedforwardMode::Response &res) override;
        std::function<kortex_driver::srv::GetCoggingFeedforwardMode::Response(const kortex_driver::srv::GetCoggingFeedforwardMode::Request&)> GetCoggingFeedforwardModeHandler = nullptr;
        virtual bool GetCoggingFeedforwardMode(kortex_driver::srv::GetCoggingFeedforwardMode::Request  &req, kortex_driver::srv::GetCoggingFeedforwardMode::Response &res) override;

};
#endif
