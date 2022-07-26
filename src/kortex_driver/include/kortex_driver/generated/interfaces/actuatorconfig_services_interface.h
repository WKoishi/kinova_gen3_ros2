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
        IActuatorConfigServices(ros::NodeHandle& node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) = 0;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) = 0;
        virtual bool GetAxisOffsets(kortex_driver::srv::GetAxisOffsets::Request  &req, kortex_driver::srv::GetAxisOffsets::Response &res) = 0;
        virtual bool SetAxisOffsets(kortex_driver::srv::SetAxisOffsets::Request  &req, kortex_driver::srv::SetAxisOffsets::Response &res) = 0;
        virtual bool SetTorqueOffset(kortex_driver::srv::SetTorqueOffset::Request  &req, kortex_driver::srv::SetTorqueOffset::Response &res) = 0;
        virtual bool ActuatorConfig_GetControlMode(kortex_driver::srv::ActuatorConfigGetControlMode::Request  &req, kortex_driver::srv::ActuatorConfigGetControlMode::Response &res) = 0;
        virtual bool SetControlMode(kortex_driver::srv::SetControlMode::Request  &req, kortex_driver::srv::SetControlMode::Response &res) = 0;
        virtual bool GetActivatedControlLoop(kortex_driver::srv::GetActivatedControlLoop::Request  &req, kortex_driver::srv::GetActivatedControlLoop::Response &res) = 0;
        virtual bool SetActivatedControlLoop(kortex_driver::srv::SetActivatedControlLoop::Request  &req, kortex_driver::srv::SetActivatedControlLoop::Response &res) = 0;
        virtual bool GetControlLoopParameters(kortex_driver::srv::GetControlLoopParameters::Request  &req, kortex_driver::srv::GetControlLoopParameters::Response &res) = 0;
        virtual bool SetControlLoopParameters(kortex_driver::srv::SetControlLoopParameters::Request  &req, kortex_driver::srv::SetControlLoopParameters::Response &res) = 0;
        virtual bool SelectCustomData(kortex_driver::srv::SelectCustomData::Request  &req, kortex_driver::srv::SelectCustomData::Response &res) = 0;
        virtual bool GetSelectedCustomData(kortex_driver::srv::GetSelectedCustomData::Request  &req, kortex_driver::srv::GetSelectedCustomData::Response &res) = 0;
        virtual bool SetCommandMode(kortex_driver::srv::SetCommandMode::Request  &req, kortex_driver::srv::SetCommandMode::Response &res) = 0;
        virtual bool ActuatorConfig_ClearFaults(kortex_driver::srv::ActuatorConfigClearFaults::Request  &req, kortex_driver::srv::ActuatorConfigClearFaults::Response &res) = 0;
        virtual bool SetServoing(kortex_driver::srv::SetServoing::Request  &req, kortex_driver::srv::SetServoing::Response &res) = 0;
        virtual bool MoveToPosition(kortex_driver::srv::MoveToPosition::Request  &req, kortex_driver::srv::MoveToPosition::Response &res) = 0;
        virtual bool GetCommandMode(kortex_driver::srv::GetCommandMode::Request  &req, kortex_driver::srv::GetCommandMode::Response &res) = 0;
        virtual bool GetServoing(kortex_driver::srv::GetServoing::Request  &req, kortex_driver::srv::GetServoing::Response &res) = 0;
        virtual bool GetTorqueOffset(kortex_driver::srv::GetTorqueOffset::Request  &req, kortex_driver::srv::GetTorqueOffset::Response &res) = 0;
        virtual bool SetCoggingFeedforwardMode(kortex_driver::srv::SetCoggingFeedforwardMode::Request  &req, kortex_driver::srv::SetCoggingFeedforwardMode::Response &res) = 0;
        virtual bool GetCoggingFeedforwardMode(kortex_driver::srv::GetCoggingFeedforwardMode::Request  &req, kortex_driver::srv::GetCoggingFeedforwardMode::Response &res) = 0;

protected:
        ros::NodeHandle m_node_handle;
        ros::Publisher m_pub_Error;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceGetAxisOffsets;
	ros::ServiceServer m_serviceSetAxisOffsets;
	ros::ServiceServer m_serviceSetTorqueOffset;
	ros::ServiceServer m_serviceActuatorConfig_GetControlMode;
	ros::ServiceServer m_serviceSetControlMode;
	ros::ServiceServer m_serviceGetActivatedControlLoop;
	ros::ServiceServer m_serviceSetActivatedControlLoop;
	ros::ServiceServer m_serviceGetControlLoopParameters;
	ros::ServiceServer m_serviceSetControlLoopParameters;
	ros::ServiceServer m_serviceSelectCustomData;
	ros::ServiceServer m_serviceGetSelectedCustomData;
	ros::ServiceServer m_serviceSetCommandMode;
	ros::ServiceServer m_serviceActuatorConfig_ClearFaults;
	ros::ServiceServer m_serviceSetServoing;
	ros::ServiceServer m_serviceMoveToPosition;
	ros::ServiceServer m_serviceGetCommandMode;
	ros::ServiceServer m_serviceGetServoing;
	ros::ServiceServer m_serviceGetTorqueOffset;
	ros::ServiceServer m_serviceSetCoggingFeedforwardMode;
	ros::ServiceServer m_serviceGetCoggingFeedforwardMode;
};
#endif
