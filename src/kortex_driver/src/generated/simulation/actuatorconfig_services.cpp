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
 
#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"
#include "kortex_driver/generated/simulation/actuatorconfig_services.h"

ActuatorConfigSimulationServices::ActuatorConfigSimulationServices(rclcpp::Node::SharedPtr node_handle): 
	IActuatorConfigServices(node_handle)
{
	m_pub_Error = m_node_handle.advertise<kortex_driver::msg::KortexError>("kortex_error", 1000);

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("actuator_config/set_device_id", std::bind(&ActuatorConfigSimulationServices::SetDeviceID, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("actuator_config/set_api_options", std::bind(&ActuatorConfigSimulationServices::SetApiOptions, this, std::placeholders::_1, std::placeholders::_2));

	m_serviceGetAxisOffsets = m_node_handle->create_service<kortex_driver::srv::GetAxisOffsets>("actuator_config/get_axis_offsets", std::bind(&ActuatorConfigSimulationServices::GetAxisOffsets, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetAxisOffsets = m_node_handle->create_service<kortex_driver::srv::SetAxisOffsets>("actuator_config/set_axis_offsets", std::bind(&ActuatorConfigSimulationServices::SetAxisOffsets, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetTorqueOffset = m_node_handle->create_service<kortex_driver::srv::SetTorqueOffset>("actuator_config/set_torque_offset", std::bind(&ActuatorConfigSimulationServices::SetTorqueOffset, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceActuatorConfig_GetControlMode = m_node_handle->create_service<kortex_driver::srv::ActuatorConfigGetControlMode>("actuator_config/get_control_mode", std::bind(&ActuatorConfigSimulationServices::ActuatorConfig_GetControlMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetControlMode = m_node_handle->create_service<kortex_driver::srv::SetControlMode>("actuator_config/set_control_mode", std::bind(&ActuatorConfigSimulationServices::SetControlMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetActivatedControlLoop = m_node_handle->create_service<kortex_driver::srv::GetActivatedControlLoop>("actuator_config/get_activated_control_loop", std::bind(&ActuatorConfigSimulationServices::GetActivatedControlLoop, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetActivatedControlLoop = m_node_handle->create_service<kortex_driver::srv::SetActivatedControlLoop>("actuator_config/set_activated_control_loop", std::bind(&ActuatorConfigSimulationServices::SetActivatedControlLoop, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetControlLoopParameters = m_node_handle->create_service<kortex_driver::srv::GetControlLoopParameters>("actuator_config/get_control_loop_parameters", std::bind(&ActuatorConfigSimulationServices::GetControlLoopParameters, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetControlLoopParameters = m_node_handle->create_service<kortex_driver::srv::SetControlLoopParameters>("actuator_config/set_control_loop_parameters", std::bind(&ActuatorConfigSimulationServices::SetControlLoopParameters, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSelectCustomData = m_node_handle->create_service<kortex_driver::srv::SelectCustomData>("actuator_config/select_custom_data", std::bind(&ActuatorConfigSimulationServices::SelectCustomData, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetSelectedCustomData = m_node_handle->create_service<kortex_driver::srv::GetSelectedCustomData>("actuator_config/get_selected_custom_data", std::bind(&ActuatorConfigSimulationServices::GetSelectedCustomData, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetCommandMode = m_node_handle->create_service<kortex_driver::srv::SetCommandMode>("actuator_config/set_command_mode", std::bind(&ActuatorConfigSimulationServices::SetCommandMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceActuatorConfig_ClearFaults = m_node_handle->create_service<kortex_driver::srv::ActuatorConfigClearFaults>("actuator_config/clear_faults", std::bind(&ActuatorConfigSimulationServices::ActuatorConfig_ClearFaults, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetServoing = m_node_handle->create_service<kortex_driver::srv::SetServoing>("actuator_config/set_servoing", std::bind(&ActuatorConfigSimulationServices::SetServoing, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceMoveToPosition = m_node_handle->create_service<kortex_driver::srv::MoveToPosition>("actuator_config/move_to_position", std::bind(&ActuatorConfigSimulationServices::MoveToPosition, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetCommandMode = m_node_handle->create_service<kortex_driver::srv::GetCommandMode>("actuator_config/get_command_mode", std::bind(&ActuatorConfigSimulationServices::GetCommandMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetServoing = m_node_handle->create_service<kortex_driver::srv::GetServoing>("actuator_config/get_servoing", std::bind(&ActuatorConfigSimulationServices::GetServoing, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetTorqueOffset = m_node_handle->create_service<kortex_driver::srv::GetTorqueOffset>("actuator_config/get_torque_offset", std::bind(&ActuatorConfigSimulationServices::GetTorqueOffset, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetCoggingFeedforwardMode = m_node_handle->create_service<kortex_driver::srv::SetCoggingFeedforwardMode>("actuator_config/set_cogging_feedforward_mode", std::bind(&ActuatorConfigSimulationServices::SetCoggingFeedforwardMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetCoggingFeedforwardMode = m_node_handle->create_service<kortex_driver::srv::GetCoggingFeedforwardMode>("actuator_config/get_cogging_feedforward_mode", std::bind(&ActuatorConfigSimulationServices::GetCoggingFeedforwardMode, this, std::placeholders::_1, std::placeholders::_2));
}

bool ActuatorConfigSimulationServices::SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}

bool ActuatorConfigSimulationServices::SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}


bool ActuatorConfigSimulationServices::GetAxisOffsets(const std::shared_ptr<kortex_driver::srv::GetAxisOffsets::Request> req, std::shared_ptr<kortex_driver::srv::GetAxisOffsets::Response> res)
{
	
	
	if (GetAxisOffsetsHandler)
	{
		res = GetAxisOffsetsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_axis_offsets is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetAxisOffsets(const std::shared_ptr<kortex_driver::srv::SetAxisOffsets::Request> req, std::shared_ptr<kortex_driver::srv::SetAxisOffsets::Response> res)
{
	
	
	if (SetAxisOffsetsHandler)
	{
		res = SetAxisOffsetsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_axis_offsets is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetTorqueOffset(const std::shared_ptr<kortex_driver::srv::SetTorqueOffset::Request> req, std::shared_ptr<kortex_driver::srv::SetTorqueOffset::Response> res)
{
	
	
	if (SetTorqueOffsetHandler)
	{
		res = SetTorqueOffsetHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_torque_offset is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::ActuatorConfig_GetControlMode(const std::shared_ptr<kortex_driver::srv::ActuatorConfigGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::ActuatorConfigGetControlMode::Response> res)
{
	
	
	if (ActuatorConfig_GetControlModeHandler)
	{
		res = ActuatorConfig_GetControlModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_control_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetControlMode(const std::shared_ptr<kortex_driver::srv::SetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::SetControlMode::Response> res)
{
	
	
	if (SetControlModeHandler)
	{
		res = SetControlModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_control_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetActivatedControlLoop(const std::shared_ptr<kortex_driver::srv::GetActivatedControlLoop::Request> req, std::shared_ptr<kortex_driver::srv::GetActivatedControlLoop::Response> res)
{
	
	
	if (GetActivatedControlLoopHandler)
	{
		res = GetActivatedControlLoopHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_activated_control_loop is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetActivatedControlLoop(const std::shared_ptr<kortex_driver::srv::SetActivatedControlLoop::Request> req, std::shared_ptr<kortex_driver::srv::SetActivatedControlLoop::Response> res)
{
	
	
	if (SetActivatedControlLoopHandler)
	{
		res = SetActivatedControlLoopHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_activated_control_loop is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetControlLoopParameters(const std::shared_ptr<kortex_driver::srv::GetControlLoopParameters::Request> req, std::shared_ptr<kortex_driver::srv::GetControlLoopParameters::Response> res)
{
	
	
	if (GetControlLoopParametersHandler)
	{
		res = GetControlLoopParametersHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_control_loop_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetControlLoopParameters(const std::shared_ptr<kortex_driver::srv::SetControlLoopParameters::Request> req, std::shared_ptr<kortex_driver::srv::SetControlLoopParameters::Response> res)
{
	
	
	if (SetControlLoopParametersHandler)
	{
		res = SetControlLoopParametersHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_control_loop_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SelectCustomData(const std::shared_ptr<kortex_driver::srv::SelectCustomData::Request> req, std::shared_ptr<kortex_driver::srv::SelectCustomData::Response> res)
{
	
	
	if (SelectCustomDataHandler)
	{
		res = SelectCustomDataHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/select_custom_data is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetSelectedCustomData(const std::shared_ptr<kortex_driver::srv::GetSelectedCustomData::Request> req, std::shared_ptr<kortex_driver::srv::GetSelectedCustomData::Response> res)
{
	
	
	if (GetSelectedCustomDataHandler)
	{
		res = GetSelectedCustomDataHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_selected_custom_data is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetCommandMode(const std::shared_ptr<kortex_driver::srv::SetCommandMode::Request> req, std::shared_ptr<kortex_driver::srv::SetCommandMode::Response> res)
{
	
	
	if (SetCommandModeHandler)
	{
		res = SetCommandModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_command_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::ActuatorConfig_ClearFaults(const std::shared_ptr<kortex_driver::srv::ActuatorConfigClearFaults::Request> req, std::shared_ptr<kortex_driver::srv::ActuatorConfigClearFaults::Response> res)
{
	
	
	if (ActuatorConfig_ClearFaultsHandler)
	{
		res = ActuatorConfig_ClearFaultsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/clear_faults is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetServoing(const std::shared_ptr<kortex_driver::srv::SetServoing::Request> req, std::shared_ptr<kortex_driver::srv::SetServoing::Response> res)
{
	
	
	if (SetServoingHandler)
	{
		res = SetServoingHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_servoing is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::MoveToPosition(const std::shared_ptr<kortex_driver::srv::MoveToPosition::Request> req, std::shared_ptr<kortex_driver::srv::MoveToPosition::Response> res)
{
	
	
	if (MoveToPositionHandler)
	{
		res = MoveToPositionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/move_to_position is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetCommandMode(const std::shared_ptr<kortex_driver::srv::GetCommandMode::Request> req, std::shared_ptr<kortex_driver::srv::GetCommandMode::Response> res)
{
	
	
	if (GetCommandModeHandler)
	{
		res = GetCommandModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_command_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetServoing(const std::shared_ptr<kortex_driver::srv::GetServoing::Request> req, std::shared_ptr<kortex_driver::srv::GetServoing::Response> res)
{
	
	
	if (GetServoingHandler)
	{
		res = GetServoingHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_servoing is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetTorqueOffset(const std::shared_ptr<kortex_driver::srv::GetTorqueOffset::Request> req, std::shared_ptr<kortex_driver::srv::GetTorqueOffset::Response> res)
{
	
	
	if (GetTorqueOffsetHandler)
	{
		res = GetTorqueOffsetHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_torque_offset is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetCoggingFeedforwardMode(const std::shared_ptr<kortex_driver::srv::SetCoggingFeedforwardMode::Request> req, std::shared_ptr<kortex_driver::srv::SetCoggingFeedforwardMode::Response> res)
{
	
	
	if (SetCoggingFeedforwardModeHandler)
	{
		res = SetCoggingFeedforwardModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_cogging_feedforward_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetCoggingFeedforwardMode(const std::shared_ptr<kortex_driver::srv::GetCoggingFeedforwardMode::Request> req, std::shared_ptr<kortex_driver::srv::GetCoggingFeedforwardMode::Response> res)
{
	
	
	if (GetCoggingFeedforwardModeHandler)
	{
		res = GetCoggingFeedforwardModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_cogging_feedforward_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}
