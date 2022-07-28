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
#include "kortex_driver/generated/robot/actuatorconfig_services.h"

ActuatorConfigRobotServices::ActuatorConfigRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::ActuatorConfig::ActuatorConfigClient* actuatorconfig, uint32_t device_id, uint32_t timeout_ms): 
	IActuatorConfigServices(node_handle),
	m_actuatorconfig(actuatorconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_node_handle.advertise<kortex_driver::msg::KortexError>("kortex_error", 1000);

	m_serviceSetDeviceID = m_node_handle->create_service("actuator_config/set_device_id", &ActuatorConfigRobotServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle->create_service("actuator_config/set_api_options", &ActuatorConfigRobotServices::SetApiOptions, this);

	m_serviceGetAxisOffsets = m_node_handle->create_service("actuator_config/get_axis_offsets", &ActuatorConfigRobotServices::GetAxisOffsets, this);
	m_serviceSetAxisOffsets = m_node_handle->create_service("actuator_config/set_axis_offsets", &ActuatorConfigRobotServices::SetAxisOffsets, this);
	m_serviceSetTorqueOffset = m_node_handle->create_service("actuator_config/set_torque_offset", &ActuatorConfigRobotServices::SetTorqueOffset, this);
	m_serviceActuatorConfig_GetControlMode = m_node_handle->create_service("actuator_config/get_control_mode", &ActuatorConfigRobotServices::ActuatorConfig_GetControlMode, this);
	m_serviceSetControlMode = m_node_handle->create_service("actuator_config/set_control_mode", &ActuatorConfigRobotServices::SetControlMode, this);
	m_serviceGetActivatedControlLoop = m_node_handle->create_service("actuator_config/get_activated_control_loop", &ActuatorConfigRobotServices::GetActivatedControlLoop, this);
	m_serviceSetActivatedControlLoop = m_node_handle->create_service("actuator_config/set_activated_control_loop", &ActuatorConfigRobotServices::SetActivatedControlLoop, this);
	m_serviceGetControlLoopParameters = m_node_handle->create_service("actuator_config/get_control_loop_parameters", &ActuatorConfigRobotServices::GetControlLoopParameters, this);
	m_serviceSetControlLoopParameters = m_node_handle->create_service("actuator_config/set_control_loop_parameters", &ActuatorConfigRobotServices::SetControlLoopParameters, this);
	m_serviceSelectCustomData = m_node_handle->create_service("actuator_config/select_custom_data", &ActuatorConfigRobotServices::SelectCustomData, this);
	m_serviceGetSelectedCustomData = m_node_handle->create_service("actuator_config/get_selected_custom_data", &ActuatorConfigRobotServices::GetSelectedCustomData, this);
	m_serviceSetCommandMode = m_node_handle->create_service("actuator_config/set_command_mode", &ActuatorConfigRobotServices::SetCommandMode, this);
	m_serviceActuatorConfig_ClearFaults = m_node_handle->create_service("actuator_config/clear_faults", &ActuatorConfigRobotServices::ActuatorConfig_ClearFaults, this);
	m_serviceSetServoing = m_node_handle->create_service("actuator_config/set_servoing", &ActuatorConfigRobotServices::SetServoing, this);
	m_serviceMoveToPosition = m_node_handle->create_service("actuator_config/move_to_position", &ActuatorConfigRobotServices::MoveToPosition, this);
	m_serviceGetCommandMode = m_node_handle->create_service("actuator_config/get_command_mode", &ActuatorConfigRobotServices::GetCommandMode, this);
	m_serviceGetServoing = m_node_handle->create_service("actuator_config/get_servoing", &ActuatorConfigRobotServices::GetServoing, this);
	m_serviceGetTorqueOffset = m_node_handle->create_service("actuator_config/get_torque_offset", &ActuatorConfigRobotServices::GetTorqueOffset, this);
	m_serviceSetCoggingFeedforwardMode = m_node_handle->create_service("actuator_config/set_cogging_feedforward_mode", &ActuatorConfigRobotServices::SetCoggingFeedforwardMode, this);
	m_serviceGetCoggingFeedforwardMode = m_node_handle->create_service("actuator_config/get_cogging_feedforward_mode", &ActuatorConfigRobotServices::GetCoggingFeedforwardMode, this);
}

bool ActuatorConfigRobotServices::SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool ActuatorConfigRobotServices::SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool ActuatorConfigRobotServices::GetAxisOffsets(kortex_driver::srv::GetAxisOffsets::Request  &req, kortex_driver::srv::GetAxisOffsets::Response &res)
{
	
	Kinova::Api::ActuatorConfig::AxisOffsets output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetAxisOffsets(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool ActuatorConfigRobotServices::SetAxisOffsets(kortex_driver::srv::SetAxisOffsets::Request  &req, kortex_driver::srv::SetAxisOffsets::Response &res)
{
	
	Kinova::Api::ActuatorConfig::AxisPosition input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetAxisOffsets(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::SetTorqueOffset(kortex_driver::srv::SetTorqueOffset::Request  &req, kortex_driver::srv::SetTorqueOffset::Response &res)
{
	
	Kinova::Api::ActuatorConfig::TorqueOffset input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetTorqueOffset(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::ActuatorConfig_GetControlMode(kortex_driver::srv::ActuatorConfigGetControlMode::Request  &req, kortex_driver::srv::ActuatorConfigGetControlMode::Response &res)
{
	
	Kinova::Api::ActuatorConfig::ControlModeInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetControlMode(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool ActuatorConfigRobotServices::SetControlMode(kortex_driver::srv::SetControlMode::Request  &req, kortex_driver::srv::SetControlMode::Response &res)
{
	
	Kinova::Api::ActuatorConfig::ControlModeInformation input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetControlMode(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::GetActivatedControlLoop(kortex_driver::srv::GetActivatedControlLoop::Request  &req, kortex_driver::srv::GetActivatedControlLoop::Response &res)
{
	
	Kinova::Api::ActuatorConfig::ControlLoop output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetActivatedControlLoop(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool ActuatorConfigRobotServices::SetActivatedControlLoop(kortex_driver::srv::SetActivatedControlLoop::Request  &req, kortex_driver::srv::SetActivatedControlLoop::Response &res)
{
	
	Kinova::Api::ActuatorConfig::ControlLoop input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetActivatedControlLoop(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::GetControlLoopParameters(kortex_driver::srv::GetControlLoopParameters::Request  &req, kortex_driver::srv::GetControlLoopParameters::Response &res)
{
	
	Kinova::Api::ActuatorConfig::LoopSelection input;
	ToProtoData(req.input, &input);
	Kinova::Api::ActuatorConfig::ControlLoopParameters output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetControlLoopParameters(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool ActuatorConfigRobotServices::SetControlLoopParameters(kortex_driver::srv::SetControlLoopParameters::Request  &req, kortex_driver::srv::SetControlLoopParameters::Response &res)
{
	
	Kinova::Api::ActuatorConfig::ControlLoopParameters input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetControlLoopParameters(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::SelectCustomData(kortex_driver::srv::SelectCustomData::Request  &req, kortex_driver::srv::SelectCustomData::Response &res)
{
	
	Kinova::Api::ActuatorConfig::CustomDataSelection input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SelectCustomData(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::GetSelectedCustomData(kortex_driver::srv::GetSelectedCustomData::Request  &req, kortex_driver::srv::GetSelectedCustomData::Response &res)
{
	
	Kinova::Api::ActuatorConfig::CustomDataSelection output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetSelectedCustomData(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool ActuatorConfigRobotServices::SetCommandMode(kortex_driver::srv::SetCommandMode::Request  &req, kortex_driver::srv::SetCommandMode::Response &res)
{
	
	Kinova::Api::ActuatorConfig::CommandModeInformation input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetCommandMode(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::ActuatorConfig_ClearFaults(kortex_driver::srv::ActuatorConfigClearFaults::Request  &req, kortex_driver::srv::ActuatorConfigClearFaults::Response &res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_actuatorconfig->ClearFaults(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::SetServoing(kortex_driver::srv::SetServoing::Request  &req, kortex_driver::srv::SetServoing::Response &res)
{
	
	Kinova::Api::ActuatorConfig::Servoing input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetServoing(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::MoveToPosition(kortex_driver::srv::MoveToPosition::Request  &req, kortex_driver::srv::MoveToPosition::Response &res)
{
	
	Kinova::Api::ActuatorConfig::PositionCommand input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_actuatorconfig->MoveToPosition(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::GetCommandMode(kortex_driver::srv::GetCommandMode::Request  &req, kortex_driver::srv::GetCommandMode::Response &res)
{
	
	Kinova::Api::ActuatorConfig::CommandModeInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetCommandMode(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool ActuatorConfigRobotServices::GetServoing(kortex_driver::srv::GetServoing::Request  &req, kortex_driver::srv::GetServoing::Response &res)
{
	
	Kinova::Api::ActuatorConfig::Servoing output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetServoing(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool ActuatorConfigRobotServices::GetTorqueOffset(kortex_driver::srv::GetTorqueOffset::Request  &req, kortex_driver::srv::GetTorqueOffset::Response &res)
{
	
	Kinova::Api::ActuatorConfig::TorqueOffset output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetTorqueOffset(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool ActuatorConfigRobotServices::SetCoggingFeedforwardMode(kortex_driver::srv::SetCoggingFeedforwardMode::Request  &req, kortex_driver::srv::SetCoggingFeedforwardMode::Response &res)
{
	
	Kinova::Api::ActuatorConfig::CoggingFeedforwardModeInformation input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetCoggingFeedforwardMode(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::GetCoggingFeedforwardMode(kortex_driver::srv::GetCoggingFeedforwardMode::Request  &req, kortex_driver::srv::GetCoggingFeedforwardMode::Response &res)
{
	
	Kinova::Api::ActuatorConfig::CoggingFeedforwardModeInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetCoggingFeedforwardMode(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
