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
#include "kortex_driver/generated/simulation/interconnectconfig_services.h"

InterconnectConfigSimulationServices::InterconnectConfigSimulationServices(rclcpp::Node::SharedPtr node_handle): 
	IInterconnectConfigServices(node_handle)
{
	m_pub_Error = m_node_handle->create_publisher<kortex_driver::msg::KortexError>("kortex_error", 1000);

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("interconnect_config/set_device_id", std::bind(&InterconnectConfigSimulationServices::SetDeviceID, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("interconnect_config/set_api_options", std::bind(&InterconnectConfigSimulationServices::SetApiOptions, this, std::placeholders::_1, std::placeholders::_2));

	m_serviceGetUARTConfiguration = m_node_handle->create_service<kortex_driver::srv::GetUARTConfiguration>("interconnect_config/get_u_a_r_t_configuration", std::bind(&InterconnectConfigSimulationServices::GetUARTConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetUARTConfiguration = m_node_handle->create_service<kortex_driver::srv::SetUARTConfiguration>("interconnect_config/set_u_a_r_t_configuration", std::bind(&InterconnectConfigSimulationServices::SetUARTConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetEthernetConfiguration = m_node_handle->create_service<kortex_driver::srv::GetEthernetConfiguration>("interconnect_config/get_ethernet_configuration", std::bind(&InterconnectConfigSimulationServices::GetEthernetConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetEthernetConfiguration = m_node_handle->create_service<kortex_driver::srv::SetEthernetConfiguration>("interconnect_config/set_ethernet_configuration", std::bind(&InterconnectConfigSimulationServices::SetEthernetConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetGPIOConfiguration = m_node_handle->create_service<kortex_driver::srv::GetGPIOConfiguration>("interconnect_config/get_g_p_i_o_configuration", std::bind(&InterconnectConfigSimulationServices::GetGPIOConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetGPIOConfiguration = m_node_handle->create_service<kortex_driver::srv::SetGPIOConfiguration>("interconnect_config/set_g_p_i_o_configuration", std::bind(&InterconnectConfigSimulationServices::SetGPIOConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetGPIOState = m_node_handle->create_service<kortex_driver::srv::GetGPIOState>("interconnect_config/get_g_p_i_o_state", std::bind(&InterconnectConfigSimulationServices::GetGPIOState, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetGPIOState = m_node_handle->create_service<kortex_driver::srv::SetGPIOState>("interconnect_config/set_g_p_i_o_state", std::bind(&InterconnectConfigSimulationServices::SetGPIOState, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetI2CConfiguration = m_node_handle->create_service<kortex_driver::srv::GetI2CConfiguration>("interconnect_config/get_i2_c_configuration", std::bind(&InterconnectConfigSimulationServices::GetI2CConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetI2CConfiguration = m_node_handle->create_service<kortex_driver::srv::SetI2CConfiguration>("interconnect_config/set_i2_c_configuration", std::bind(&InterconnectConfigSimulationServices::SetI2CConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceI2CRead = m_node_handle->create_service<kortex_driver::srv::I2CRead>("interconnect_config/i2_c_read", std::bind(&InterconnectConfigSimulationServices::I2CRead, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceI2CReadRegister = m_node_handle->create_service<kortex_driver::srv::I2CReadRegister>("interconnect_config/i2_c_read_register", std::bind(&InterconnectConfigSimulationServices::I2CReadRegister, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceI2CWrite = m_node_handle->create_service<kortex_driver::srv::I2CWrite>("interconnect_config/i2_c_write", std::bind(&InterconnectConfigSimulationServices::I2CWrite, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceI2CWriteRegister = m_node_handle->create_service<kortex_driver::srv::I2CWriteRegister>("interconnect_config/i2_c_write_register", std::bind(&InterconnectConfigSimulationServices::I2CWriteRegister, this, std::placeholders::_1, std::placeholders::_2));
}

bool InterconnectConfigSimulationServices::SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res)
{
	RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}

bool InterconnectConfigSimulationServices::SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res)
{
	RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}


bool InterconnectConfigSimulationServices::GetUARTConfiguration(const std::shared_ptr<kortex_driver::srv::GetUARTConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetUARTConfiguration::Response> res)
{
	
	
	if (GetUARTConfigurationHandler)
	{
		res = GetUARTConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/get_u_a_r_t_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::SetUARTConfiguration(const std::shared_ptr<kortex_driver::srv::SetUARTConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetUARTConfiguration::Response> res)
{
	
	
	if (SetUARTConfigurationHandler)
	{
		res = SetUARTConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/set_u_a_r_t_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::GetEthernetConfiguration(const std::shared_ptr<kortex_driver::srv::GetEthernetConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetEthernetConfiguration::Response> res)
{
	
	
	if (GetEthernetConfigurationHandler)
	{
		res = GetEthernetConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/get_ethernet_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::SetEthernetConfiguration(const std::shared_ptr<kortex_driver::srv::SetEthernetConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetEthernetConfiguration::Response> res)
{
	
	
	if (SetEthernetConfigurationHandler)
	{
		res = SetEthernetConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/set_ethernet_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::GetGPIOConfiguration(const std::shared_ptr<kortex_driver::srv::GetGPIOConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetGPIOConfiguration::Response> res)
{
	
	
	if (GetGPIOConfigurationHandler)
	{
		res = GetGPIOConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/get_g_p_i_o_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::SetGPIOConfiguration(const std::shared_ptr<kortex_driver::srv::SetGPIOConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetGPIOConfiguration::Response> res)
{
	
	
	if (SetGPIOConfigurationHandler)
	{
		res = SetGPIOConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/set_g_p_i_o_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::GetGPIOState(const std::shared_ptr<kortex_driver::srv::GetGPIOState::Request> req, std::shared_ptr<kortex_driver::srv::GetGPIOState::Response> res)
{
	
	
	if (GetGPIOStateHandler)
	{
		res = GetGPIOStateHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/get_g_p_i_o_state is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::SetGPIOState(const std::shared_ptr<kortex_driver::srv::SetGPIOState::Request> req, std::shared_ptr<kortex_driver::srv::SetGPIOState::Response> res)
{
	
	
	if (SetGPIOStateHandler)
	{
		res = SetGPIOStateHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/set_g_p_i_o_state is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::GetI2CConfiguration(const std::shared_ptr<kortex_driver::srv::GetI2CConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetI2CConfiguration::Response> res)
{
	
	
	if (GetI2CConfigurationHandler)
	{
		res = GetI2CConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/get_i2_c_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::SetI2CConfiguration(const std::shared_ptr<kortex_driver::srv::SetI2CConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetI2CConfiguration::Response> res)
{
	
	
	if (SetI2CConfigurationHandler)
	{
		res = SetI2CConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/set_i2_c_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::I2CRead(const std::shared_ptr<kortex_driver::srv::I2CRead::Request> req, std::shared_ptr<kortex_driver::srv::I2CRead::Response> res)
{
	
	
	if (I2CReadHandler)
	{
		res = I2CReadHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/i2_c_read is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::I2CReadRegister(const std::shared_ptr<kortex_driver::srv::I2CReadRegister::Request> req, std::shared_ptr<kortex_driver::srv::I2CReadRegister::Response> res)
{
	
	
	if (I2CReadRegisterHandler)
	{
		res = I2CReadRegisterHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/i2_c_read_register is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::I2CWrite(const std::shared_ptr<kortex_driver::srv::I2CWrite::Request> req, std::shared_ptr<kortex_driver::srv::I2CWrite::Response> res)
{
	
	
	if (I2CWriteHandler)
	{
		res = I2CWriteHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/i2_c_write is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::I2CWriteRegister(const std::shared_ptr<kortex_driver::srv::I2CWriteRegister::Request> req, std::shared_ptr<kortex_driver::srv::I2CWriteRegister::Response> res)
{
	
	
	if (I2CWriteRegisterHandler)
	{
		res = I2CWriteRegisterHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for interconnect_config/i2_c_write_register is not implemented, so the service calls will return the default response.");
	}
	return true;
}
