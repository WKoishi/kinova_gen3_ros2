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
 
#ifndef _KORTEX_ACTUATORCONFIG_SERVICES_INTERFACE_H_
#define _KORTEX_ACTUATORCONFIG_SERVICES_INTERFACE_H_

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>
#include "kortex_driver/srv/get_axis_offsets.hpp"
#include "kortex_driver/srv/set_axis_offsets.hpp"
#include "kortex_driver/srv/set_torque_offset.hpp"
#include "kortex_driver/srv/actuator_config_get_control_mode.hpp"
#include "kortex_driver/srv/set_control_mode.hpp"
#include "kortex_driver/srv/get_activated_control_loop.hpp"
#include "kortex_driver/srv/set_activated_control_loop.hpp"
#include "kortex_driver/srv/get_control_loop_parameters.hpp"
#include "kortex_driver/srv/set_control_loop_parameters.hpp"
#include "kortex_driver/srv/select_custom_data.hpp"
#include "kortex_driver/srv/get_selected_custom_data.hpp"
#include "kortex_driver/srv/set_command_mode.hpp"
#include "kortex_driver/srv/actuator_config_clear_faults.hpp"
#include "kortex_driver/srv/set_servoing.hpp"
#include "kortex_driver/srv/move_to_position.hpp"
#include "kortex_driver/srv/get_command_mode.hpp"
#include "kortex_driver/srv/get_servoing.hpp"
#include "kortex_driver/srv/get_torque_offset.hpp"
#include "kortex_driver/srv/set_cogging_feedforward_mode.hpp"
#include "kortex_driver/srv/get_cogging_feedforward_mode.hpp"

#include "kortex_driver/msg/kortex_error.hpp"
#include "kortex_driver/srv/set_device_id.hpp"
#include "kortex_driver/srv/set_api_options.hpp"
#include "kortex_driver/msg/api_options.hpp"

using namespace std;

class IActuatorConfigServices
{
    public:
        IActuatorConfigServices(rclcpp::Node::SharedPtr node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) = 0;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) = 0;
        virtual bool GetAxisOffsets(const std::shared_ptr<kortex_driver::srv::GetAxisOffsets::Request> req, std::shared_ptr<kortex_driver::srv::GetAxisOffsets::Response> res) = 0;
        virtual bool SetAxisOffsets(const std::shared_ptr<kortex_driver::srv::SetAxisOffsets::Request> req, std::shared_ptr<kortex_driver::srv::SetAxisOffsets::Response> res) = 0;
        virtual bool SetTorqueOffset(const std::shared_ptr<kortex_driver::srv::SetTorqueOffset::Request> req, std::shared_ptr<kortex_driver::srv::SetTorqueOffset::Response> res) = 0;
        virtual bool ActuatorConfig_GetControlMode(const std::shared_ptr<kortex_driver::srv::ActuatorConfigGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::ActuatorConfigGetControlMode::Response> res) = 0;
        virtual bool SetControlMode(const std::shared_ptr<kortex_driver::srv::SetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::SetControlMode::Response> res) = 0;
        virtual bool GetActivatedControlLoop(const std::shared_ptr<kortex_driver::srv::GetActivatedControlLoop::Request> req, std::shared_ptr<kortex_driver::srv::GetActivatedControlLoop::Response> res) = 0;
        virtual bool SetActivatedControlLoop(const std::shared_ptr<kortex_driver::srv::SetActivatedControlLoop::Request> req, std::shared_ptr<kortex_driver::srv::SetActivatedControlLoop::Response> res) = 0;
        virtual bool GetControlLoopParameters(const std::shared_ptr<kortex_driver::srv::GetControlLoopParameters::Request> req, std::shared_ptr<kortex_driver::srv::GetControlLoopParameters::Response> res) = 0;
        virtual bool SetControlLoopParameters(const std::shared_ptr<kortex_driver::srv::SetControlLoopParameters::Request> req, std::shared_ptr<kortex_driver::srv::SetControlLoopParameters::Response> res) = 0;
        virtual bool SelectCustomData(const std::shared_ptr<kortex_driver::srv::SelectCustomData::Request> req, std::shared_ptr<kortex_driver::srv::SelectCustomData::Response> res) = 0;
        virtual bool GetSelectedCustomData(const std::shared_ptr<kortex_driver::srv::GetSelectedCustomData::Request> req, std::shared_ptr<kortex_driver::srv::GetSelectedCustomData::Response> res) = 0;
        virtual bool SetCommandMode(const std::shared_ptr<kortex_driver::srv::SetCommandMode::Request> req, std::shared_ptr<kortex_driver::srv::SetCommandMode::Response> res) = 0;
        virtual bool ActuatorConfig_ClearFaults(const std::shared_ptr<kortex_driver::srv::ActuatorConfigClearFaults::Request> req, std::shared_ptr<kortex_driver::srv::ActuatorConfigClearFaults::Response> res) = 0;
        virtual bool SetServoing(const std::shared_ptr<kortex_driver::srv::SetServoing::Request> req, std::shared_ptr<kortex_driver::srv::SetServoing::Response> res) = 0;
        virtual bool MoveToPosition(const std::shared_ptr<kortex_driver::srv::MoveToPosition::Request> req, std::shared_ptr<kortex_driver::srv::MoveToPosition::Response> res) = 0;
        virtual bool GetCommandMode(const std::shared_ptr<kortex_driver::srv::GetCommandMode::Request> req, std::shared_ptr<kortex_driver::srv::GetCommandMode::Response> res) = 0;
        virtual bool GetServoing(const std::shared_ptr<kortex_driver::srv::GetServoing::Request> req, std::shared_ptr<kortex_driver::srv::GetServoing::Response> res) = 0;
        virtual bool GetTorqueOffset(const std::shared_ptr<kortex_driver::srv::GetTorqueOffset::Request> req, std::shared_ptr<kortex_driver::srv::GetTorqueOffset::Response> res) = 0;
        virtual bool SetCoggingFeedforwardMode(const std::shared_ptr<kortex_driver::srv::SetCoggingFeedforwardMode::Request> req, std::shared_ptr<kortex_driver::srv::SetCoggingFeedforwardMode::Response> res) = 0;
        virtual bool GetCoggingFeedforwardMode(const std::shared_ptr<kortex_driver::srv::GetCoggingFeedforwardMode::Request> req, std::shared_ptr<kortex_driver::srv::GetCoggingFeedforwardMode::Response> res) = 0;

protected:
        rclcpp::Node::SharedPtr m_node_handle;
        rclcpp::Publisher<kortex_driver::msg::KortexError>::SharedPtr m_pub_Error;

        rclcpp::Service<kortex_driver::srv::SetDeviceID>::SharedPtr m_serviceSetDeviceID;
        rclcpp::Service<kortex_driver::srv::SetApiOptions>::SharedPtr m_serviceSetApiOptions;

	rclcpp::Service<kortex_driver::srv::GetAxisOffsets>::SharedPtr m_serviceGetAxisOffsets;
	rclcpp::Service<kortex_driver::srv::SetAxisOffsets>::SharedPtr m_serviceSetAxisOffsets;
	rclcpp::Service<kortex_driver::srv::SetTorqueOffset>::SharedPtr m_serviceSetTorqueOffset;
	rclcpp::Service<kortex_driver::srv::ActuatorConfigGetControlMode>::SharedPtr m_serviceActuatorConfig_GetControlMode;
	rclcpp::Service<kortex_driver::srv::SetControlMode>::SharedPtr m_serviceSetControlMode;
	rclcpp::Service<kortex_driver::srv::GetActivatedControlLoop>::SharedPtr m_serviceGetActivatedControlLoop;
	rclcpp::Service<kortex_driver::srv::SetActivatedControlLoop>::SharedPtr m_serviceSetActivatedControlLoop;
	rclcpp::Service<kortex_driver::srv::GetControlLoopParameters>::SharedPtr m_serviceGetControlLoopParameters;
	rclcpp::Service<kortex_driver::srv::SetControlLoopParameters>::SharedPtr m_serviceSetControlLoopParameters;
	rclcpp::Service<kortex_driver::srv::SelectCustomData>::SharedPtr m_serviceSelectCustomData;
	rclcpp::Service<kortex_driver::srv::GetSelectedCustomData>::SharedPtr m_serviceGetSelectedCustomData;
	rclcpp::Service<kortex_driver::srv::SetCommandMode>::SharedPtr m_serviceSetCommandMode;
	rclcpp::Service<kortex_driver::srv::ActuatorConfigClearFaults>::SharedPtr m_serviceActuatorConfig_ClearFaults;
	rclcpp::Service<kortex_driver::srv::SetServoing>::SharedPtr m_serviceSetServoing;
	rclcpp::Service<kortex_driver::srv::MoveToPosition>::SharedPtr m_serviceMoveToPosition;
	rclcpp::Service<kortex_driver::srv::GetCommandMode>::SharedPtr m_serviceGetCommandMode;
	rclcpp::Service<kortex_driver::srv::GetServoing>::SharedPtr m_serviceGetServoing;
	rclcpp::Service<kortex_driver::srv::GetTorqueOffset>::SharedPtr m_serviceGetTorqueOffset;
	rclcpp::Service<kortex_driver::srv::SetCoggingFeedforwardMode>::SharedPtr m_serviceSetCoggingFeedforwardMode;
	rclcpp::Service<kortex_driver::srv::GetCoggingFeedforwardMode>::SharedPtr m_serviceGetCoggingFeedforwardMode;
};
#endif
