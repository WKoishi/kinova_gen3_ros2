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
#include "kortex_driver/generated/robot/interconnectconfig_services.h"

InterconnectConfigRobotServices::InterconnectConfigRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::InterconnectConfig::InterconnectConfigClient* interconnectconfig, uint32_t device_id, uint32_t timeout_ms): 
	IInterconnectConfigServices(node_handle),
	m_interconnectconfig(interconnectconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_node_handle->create_publisher<kortex_driver::msg::KortexError>("kortex_error", 1000);

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("interconnect_config/set_device_id", std::bind(&InterconnectConfigRobotServices::SetDeviceID, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("interconnect_config/set_api_options", std::bind(&InterconnectConfigRobotServices::SetApiOptions, this, std::placeholders::_1, std::placeholders::_2));

	m_serviceGetUARTConfiguration = m_node_handle->create_service<kortex_driver::srv::GetUARTConfiguration>("interconnect_config/get_u_a_r_t_configuration", std::bind(&InterconnectConfigRobotServices::GetUARTConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetUARTConfiguration = m_node_handle->create_service<kortex_driver::srv::SetUARTConfiguration>("interconnect_config/set_u_a_r_t_configuration", std::bind(&InterconnectConfigRobotServices::SetUARTConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetEthernetConfiguration = m_node_handle->create_service<kortex_driver::srv::GetEthernetConfiguration>("interconnect_config/get_ethernet_configuration", std::bind(&InterconnectConfigRobotServices::GetEthernetConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetEthernetConfiguration = m_node_handle->create_service<kortex_driver::srv::SetEthernetConfiguration>("interconnect_config/set_ethernet_configuration", std::bind(&InterconnectConfigRobotServices::SetEthernetConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetGPIOConfiguration = m_node_handle->create_service<kortex_driver::srv::GetGPIOConfiguration>("interconnect_config/get_g_p_i_o_configuration", std::bind(&InterconnectConfigRobotServices::GetGPIOConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetGPIOConfiguration = m_node_handle->create_service<kortex_driver::srv::SetGPIOConfiguration>("interconnect_config/set_g_p_i_o_configuration", std::bind(&InterconnectConfigRobotServices::SetGPIOConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetGPIOState = m_node_handle->create_service<kortex_driver::srv::GetGPIOState>("interconnect_config/get_g_p_i_o_state", std::bind(&InterconnectConfigRobotServices::GetGPIOState, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetGPIOState = m_node_handle->create_service<kortex_driver::srv::SetGPIOState>("interconnect_config/set_g_p_i_o_state", std::bind(&InterconnectConfigRobotServices::SetGPIOState, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetI2CConfiguration = m_node_handle->create_service<kortex_driver::srv::GetI2CConfiguration>("interconnect_config/get_i2_c_configuration", std::bind(&InterconnectConfigRobotServices::GetI2CConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetI2CConfiguration = m_node_handle->create_service<kortex_driver::srv::SetI2CConfiguration>("interconnect_config/set_i2_c_configuration", std::bind(&InterconnectConfigRobotServices::SetI2CConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceI2CRead = m_node_handle->create_service<kortex_driver::srv::I2CRead>("interconnect_config/i2_c_read", std::bind(&InterconnectConfigRobotServices::I2CRead, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceI2CReadRegister = m_node_handle->create_service<kortex_driver::srv::I2CReadRegister>("interconnect_config/i2_c_read_register", std::bind(&InterconnectConfigRobotServices::I2CReadRegister, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceI2CWrite = m_node_handle->create_service<kortex_driver::srv::I2CWrite>("interconnect_config/i2_c_write", std::bind(&InterconnectConfigRobotServices::I2CWrite, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceI2CWriteRegister = m_node_handle->create_service<kortex_driver::srv::I2CWriteRegister>("interconnect_config/i2_c_write_register", std::bind(&InterconnectConfigRobotServices::I2CWriteRegister, this, std::placeholders::_1, std::placeholders::_2));
}

bool InterconnectConfigRobotServices::SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res)
{
	m_current_device_id = req->device_id;

	return true;
}

bool InterconnectConfigRobotServices::SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res)
{
	m_api_options.timeout_ms = req->input.timeout_ms;

	return true;
}


bool InterconnectConfigRobotServices::GetUARTConfiguration(const std::shared_ptr<kortex_driver::srv::GetUARTConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetUARTConfiguration::Response> res)
{
	
	Kinova::Api::Common::UARTDeviceIdentification input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::UARTConfiguration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->GetUARTConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::SetUARTConfiguration(const std::shared_ptr<kortex_driver::srv::SetUARTConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetUARTConfiguration::Response> res)
{
	
	Kinova::Api::Common::UARTConfiguration input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_interconnectconfig->SetUARTConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::GetEthernetConfiguration(const std::shared_ptr<kortex_driver::srv::GetEthernetConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetEthernetConfiguration::Response> res)
{
	
	Kinova::Api::InterconnectConfig::EthernetDeviceIdentification input;
	ToProtoData(req->input, &input);
	Kinova::Api::InterconnectConfig::EthernetConfiguration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->GetEthernetConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::SetEthernetConfiguration(const std::shared_ptr<kortex_driver::srv::SetEthernetConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetEthernetConfiguration::Response> res)
{
	
	Kinova::Api::InterconnectConfig::EthernetConfiguration input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_interconnectconfig->SetEthernetConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::GetGPIOConfiguration(const std::shared_ptr<kortex_driver::srv::GetGPIOConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetGPIOConfiguration::Response> res)
{
	
	Kinova::Api::InterconnectConfig::GPIOIdentification input;
	ToProtoData(req->input, &input);
	Kinova::Api::InterconnectConfig::GPIOConfiguration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->GetGPIOConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::SetGPIOConfiguration(const std::shared_ptr<kortex_driver::srv::SetGPIOConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetGPIOConfiguration::Response> res)
{
	
	Kinova::Api::InterconnectConfig::GPIOConfiguration input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_interconnectconfig->SetGPIOConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::GetGPIOState(const std::shared_ptr<kortex_driver::srv::GetGPIOState::Request> req, std::shared_ptr<kortex_driver::srv::GetGPIOState::Response> res)
{
	
	Kinova::Api::InterconnectConfig::GPIOIdentification input;
	ToProtoData(req->input, &input);
	Kinova::Api::InterconnectConfig::GPIOState output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->GetGPIOState(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::SetGPIOState(const std::shared_ptr<kortex_driver::srv::SetGPIOState::Request> req, std::shared_ptr<kortex_driver::srv::SetGPIOState::Response> res)
{
	
	Kinova::Api::InterconnectConfig::GPIOState input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_interconnectconfig->SetGPIOState(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::GetI2CConfiguration(const std::shared_ptr<kortex_driver::srv::GetI2CConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetI2CConfiguration::Response> res)
{
	
	Kinova::Api::InterconnectConfig::I2CDeviceIdentification input;
	ToProtoData(req->input, &input);
	Kinova::Api::InterconnectConfig::I2CConfiguration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->GetI2CConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::SetI2CConfiguration(const std::shared_ptr<kortex_driver::srv::SetI2CConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetI2CConfiguration::Response> res)
{
	
	Kinova::Api::InterconnectConfig::I2CConfiguration input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_interconnectconfig->SetI2CConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::I2CRead(const std::shared_ptr<kortex_driver::srv::I2CRead::Request> req, std::shared_ptr<kortex_driver::srv::I2CRead::Response> res)
{
	
	Kinova::Api::InterconnectConfig::I2CReadParameter input;
	ToProtoData(req->input, &input);
	Kinova::Api::InterconnectConfig::I2CData output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->I2CRead(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::I2CReadRegister(const std::shared_ptr<kortex_driver::srv::I2CReadRegister::Request> req, std::shared_ptr<kortex_driver::srv::I2CReadRegister::Response> res)
{
	
	Kinova::Api::InterconnectConfig::I2CReadRegisterParameter input;
	ToProtoData(req->input, &input);
	Kinova::Api::InterconnectConfig::I2CData output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->I2CReadRegister(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::I2CWrite(const std::shared_ptr<kortex_driver::srv::I2CWrite::Request> req, std::shared_ptr<kortex_driver::srv::I2CWrite::Response> res)
{
	
	Kinova::Api::InterconnectConfig::I2CWriteParameter input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_interconnectconfig->I2CWrite(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::I2CWriteRegister(const std::shared_ptr<kortex_driver::srv::I2CWriteRegister::Request> req, std::shared_ptr<kortex_driver::srv::I2CWriteRegister::Response> res)
{
	
	Kinova::Api::InterconnectConfig::I2CWriteRegisterParameter input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_interconnectconfig->I2CWriteRegister(input, m_current_device_id, m_api_options);
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
