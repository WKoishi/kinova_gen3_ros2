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

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("actuator_config/set_device_id", std::bind(&ActuatorConfigRobotServices::SetDeviceID, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("actuator_config/set_api_options", std::bind(&ActuatorConfigRobotServices::SetApiOptions, this, std::placeholders::_1, std::placeholders::_2));

	m_serviceGetAxisOffsets = m_node_handle->create_service<kortex_driver::srv::GetAxisOffsets>("actuator_config/get_axis_offsets", std::bind(&ActuatorConfigRobotServices::GetAxisOffsets, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetAxisOffsets = m_node_handle->create_service<kortex_driver::srv::SetAxisOffsets>("actuator_config/set_axis_offsets", std::bind(&ActuatorConfigRobotServices::SetAxisOffsets, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetTorqueOffset = m_node_handle->create_service<kortex_driver::srv::SetTorqueOffset>("actuator_config/set_torque_offset", std::bind(&ActuatorConfigRobotServices::SetTorqueOffset, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceActuatorConfig_GetControlMode = m_node_handle->create_service<kortex_driver::srv::ActuatorConfigGetControlMode>("actuator_config/get_control_mode", std::bind(&ActuatorConfigRobotServices::ActuatorConfig_GetControlMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetControlMode = m_node_handle->create_service<kortex_driver::srv::SetControlMode>("actuator_config/set_control_mode", std::bind(&ActuatorConfigRobotServices::SetControlMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetActivatedControlLoop = m_node_handle->create_service<kortex_driver::srv::GetActivatedControlLoop>("actuator_config/get_activated_control_loop", std::bind(&ActuatorConfigRobotServices::GetActivatedControlLoop, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetActivatedControlLoop = m_node_handle->create_service<kortex_driver::srv::SetActivatedControlLoop>("actuator_config/set_activated_control_loop", std::bind(&ActuatorConfigRobotServices::SetActivatedControlLoop, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetControlLoopParameters = m_node_handle->create_service<kortex_driver::srv::GetControlLoopParameters>("actuator_config/get_control_loop_parameters", std::bind(&ActuatorConfigRobotServices::GetControlLoopParameters, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetControlLoopParameters = m_node_handle->create_service<kortex_driver::srv::SetControlLoopParameters>("actuator_config/set_control_loop_parameters", std::bind(&ActuatorConfigRobotServices::SetControlLoopParameters, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSelectCustomData = m_node_handle->create_service<kortex_driver::srv::SelectCustomData>("actuator_config/select_custom_data", std::bind(&ActuatorConfigRobotServices::SelectCustomData, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetSelectedCustomData = m_node_handle->create_service<kortex_driver::srv::GetSelectedCustomData>("actuator_config/get_selected_custom_data", std::bind(&ActuatorConfigRobotServices::GetSelectedCustomData, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetCommandMode = m_node_handle->create_service<kortex_driver::srv::SetCommandMode>("actuator_config/set_command_mode", std::bind(&ActuatorConfigRobotServices::SetCommandMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceActuatorConfig_ClearFaults = m_node_handle->create_service<kortex_driver::srv::ActuatorConfigClearFaults>("actuator_config/clear_faults", std::bind(&ActuatorConfigRobotServices::ActuatorConfig_ClearFaults, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetServoing = m_node_handle->create_service<kortex_driver::srv::SetServoing>("actuator_config/set_servoing", std::bind(&ActuatorConfigRobotServices::SetServoing, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceMoveToPosition = m_node_handle->create_service<kortex_driver::srv::MoveToPosition>("actuator_config/move_to_position", std::bind(&ActuatorConfigRobotServices::MoveToPosition, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetCommandMode = m_node_handle->create_service<kortex_driver::srv::GetCommandMode>("actuator_config/get_command_mode", std::bind(&ActuatorConfigRobotServices::GetCommandMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetServoing = m_node_handle->create_service<kortex_driver::srv::GetServoing>("actuator_config/get_servoing", std::bind(&ActuatorConfigRobotServices::GetServoing, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetTorqueOffset = m_node_handle->create_service<kortex_driver::srv::GetTorqueOffset>("actuator_config/get_torque_offset", std::bind(&ActuatorConfigRobotServices::GetTorqueOffset, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetCoggingFeedforwardMode = m_node_handle->create_service<kortex_driver::srv::SetCoggingFeedforwardMode>("actuator_config/set_cogging_feedforward_mode", std::bind(&ActuatorConfigRobotServices::SetCoggingFeedforwardMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetCoggingFeedforwardMode = m_node_handle->create_service<kortex_driver::srv::GetCoggingFeedforwardMode>("actuator_config/get_cogging_feedforward_mode", std::bind(&ActuatorConfigRobotServices::GetCoggingFeedforwardMode, this, std::placeholders::_1, std::placeholders::_2));
}

bool ActuatorConfigRobotServices::SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res)
{
	m_current_device_id = req->device_id;

	return true;
}

bool ActuatorConfigRobotServices::SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res)
{
	m_api_options.timeout_ms = req->input.timeout_ms;

	return true;
}


bool ActuatorConfigRobotServices::GetAxisOffsets(const std::shared_ptr<kortex_driver::srv::GetAxisOffsets::Request> req, std::shared_ptr<kortex_driver::srv::GetAxisOffsets::Response> res)
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ActuatorConfigRobotServices::SetAxisOffsets(const std::shared_ptr<kortex_driver::srv::SetAxisOffsets::Request> req, std::shared_ptr<kortex_driver::srv::SetAxisOffsets::Response> res)
{
	
	Kinova::Api::ActuatorConfig::AxisPosition input;
	ToProtoData(req->input, &input);
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::SetTorqueOffset(const std::shared_ptr<kortex_driver::srv::SetTorqueOffset::Request> req, std::shared_ptr<kortex_driver::srv::SetTorqueOffset::Response> res)
{
	
	Kinova::Api::ActuatorConfig::TorqueOffset input;
	ToProtoData(req->input, &input);
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::ActuatorConfig_GetControlMode(const std::shared_ptr<kortex_driver::srv::ActuatorConfigGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::ActuatorConfigGetControlMode::Response> res)
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ActuatorConfigRobotServices::SetControlMode(const std::shared_ptr<kortex_driver::srv::SetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::SetControlMode::Response> res)
{
	
	Kinova::Api::ActuatorConfig::ControlModeInformation input;
	ToProtoData(req->input, &input);
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::GetActivatedControlLoop(const std::shared_ptr<kortex_driver::srv::GetActivatedControlLoop::Request> req, std::shared_ptr<kortex_driver::srv::GetActivatedControlLoop::Response> res)
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ActuatorConfigRobotServices::SetActivatedControlLoop(const std::shared_ptr<kortex_driver::srv::SetActivatedControlLoop::Request> req, std::shared_ptr<kortex_driver::srv::SetActivatedControlLoop::Response> res)
{
	
	Kinova::Api::ActuatorConfig::ControlLoop input;
	ToProtoData(req->input, &input);
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::GetControlLoopParameters(const std::shared_ptr<kortex_driver::srv::GetControlLoopParameters::Request> req, std::shared_ptr<kortex_driver::srv::GetControlLoopParameters::Response> res)
{
	
	Kinova::Api::ActuatorConfig::LoopSelection input;
	ToProtoData(req->input, &input);
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ActuatorConfigRobotServices::SetControlLoopParameters(const std::shared_ptr<kortex_driver::srv::SetControlLoopParameters::Request> req, std::shared_ptr<kortex_driver::srv::SetControlLoopParameters::Response> res)
{
	
	Kinova::Api::ActuatorConfig::ControlLoopParameters input;
	ToProtoData(req->input, &input);
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::SelectCustomData(const std::shared_ptr<kortex_driver::srv::SelectCustomData::Request> req, std::shared_ptr<kortex_driver::srv::SelectCustomData::Response> res)
{
	
	Kinova::Api::ActuatorConfig::CustomDataSelection input;
	ToProtoData(req->input, &input);
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::GetSelectedCustomData(const std::shared_ptr<kortex_driver::srv::GetSelectedCustomData::Request> req, std::shared_ptr<kortex_driver::srv::GetSelectedCustomData::Response> res)
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ActuatorConfigRobotServices::SetCommandMode(const std::shared_ptr<kortex_driver::srv::SetCommandMode::Request> req, std::shared_ptr<kortex_driver::srv::SetCommandMode::Response> res)
{
	
	Kinova::Api::ActuatorConfig::CommandModeInformation input;
	ToProtoData(req->input, &input);
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::ActuatorConfig_ClearFaults(const std::shared_ptr<kortex_driver::srv::ActuatorConfigClearFaults::Request> req, std::shared_ptr<kortex_driver::srv::ActuatorConfigClearFaults::Response> res)
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::SetServoing(const std::shared_ptr<kortex_driver::srv::SetServoing::Request> req, std::shared_ptr<kortex_driver::srv::SetServoing::Response> res)
{
	
	Kinova::Api::ActuatorConfig::Servoing input;
	ToProtoData(req->input, &input);
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::MoveToPosition(const std::shared_ptr<kortex_driver::srv::MoveToPosition::Request> req, std::shared_ptr<kortex_driver::srv::MoveToPosition::Response> res)
{
	
	Kinova::Api::ActuatorConfig::PositionCommand input;
	ToProtoData(req->input, &input);
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::GetCommandMode(const std::shared_ptr<kortex_driver::srv::GetCommandMode::Request> req, std::shared_ptr<kortex_driver::srv::GetCommandMode::Response> res)
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ActuatorConfigRobotServices::GetServoing(const std::shared_ptr<kortex_driver::srv::GetServoing::Request> req, std::shared_ptr<kortex_driver::srv::GetServoing::Response> res)
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ActuatorConfigRobotServices::GetTorqueOffset(const std::shared_ptr<kortex_driver::srv::GetTorqueOffset::Request> req, std::shared_ptr<kortex_driver::srv::GetTorqueOffset::Response> res)
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ActuatorConfigRobotServices::SetCoggingFeedforwardMode(const std::shared_ptr<kortex_driver::srv::SetCoggingFeedforwardMode::Request> req, std::shared_ptr<kortex_driver::srv::SetCoggingFeedforwardMode::Response> res)
{
	
	Kinova::Api::ActuatorConfig::CoggingFeedforwardModeInformation input;
	ToProtoData(req->input, &input);
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ActuatorConfigRobotServices::GetCoggingFeedforwardMode(const std::shared_ptr<kortex_driver::srv::GetCoggingFeedforwardMode::Request> req, std::shared_ptr<kortex_driver::srv::GetCoggingFeedforwardMode::Response> res)
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
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}
