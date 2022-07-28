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
	m_pub_Error = m_node_handle.advertise<kortex_driver::msg::KortexError>("kortex_error", 1000);

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("interconnect_config/set_device_id", &InterconnectConfigSimulationServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("interconnect_config/set_api_options", &InterconnectConfigSimulationServices::SetApiOptions, this);

	m_serviceGetUARTConfiguration = m_node_handle->create_service<kortex_driver::srv::GetUARTConfiguration>("interconnect_config/get_u_a_r_t_configuration", &InterconnectConfigSimulationServices::GetUARTConfiguration, this);
	m_serviceSetUARTConfiguration = m_node_handle->create_service<kortex_driver::srv::SetUARTConfiguration>("interconnect_config/set_u_a_r_t_configuration", &InterconnectConfigSimulationServices::SetUARTConfiguration, this);
	m_serviceGetEthernetConfiguration = m_node_handle->create_service<kortex_driver::srv::GetEthernetConfiguration>("interconnect_config/get_ethernet_configuration", &InterconnectConfigSimulationServices::GetEthernetConfiguration, this);
	m_serviceSetEthernetConfiguration = m_node_handle->create_service<kortex_driver::srv::SetEthernetConfiguration>("interconnect_config/set_ethernet_configuration", &InterconnectConfigSimulationServices::SetEthernetConfiguration, this);
	m_serviceGetGPIOConfiguration = m_node_handle->create_service<kortex_driver::srv::GetGPIOConfiguration>("interconnect_config/get_g_p_i_o_configuration", &InterconnectConfigSimulationServices::GetGPIOConfiguration, this);
	m_serviceSetGPIOConfiguration = m_node_handle->create_service<kortex_driver::srv::SetGPIOConfiguration>("interconnect_config/set_g_p_i_o_configuration", &InterconnectConfigSimulationServices::SetGPIOConfiguration, this);
	m_serviceGetGPIOState = m_node_handle->create_service<kortex_driver::srv::GetGPIOState>("interconnect_config/get_g_p_i_o_state", &InterconnectConfigSimulationServices::GetGPIOState, this);
	m_serviceSetGPIOState = m_node_handle->create_service<kortex_driver::srv::SetGPIOState>("interconnect_config/set_g_p_i_o_state", &InterconnectConfigSimulationServices::SetGPIOState, this);
	m_serviceGetI2CConfiguration = m_node_handle->create_service<kortex_driver::srv::GetI2CConfiguration>("interconnect_config/get_i2_c_configuration", &InterconnectConfigSimulationServices::GetI2CConfiguration, this);
	m_serviceSetI2CConfiguration = m_node_handle->create_service<kortex_driver::srv::SetI2CConfiguration>("interconnect_config/set_i2_c_configuration", &InterconnectConfigSimulationServices::SetI2CConfiguration, this);
	m_serviceI2CRead = m_node_handle->create_service<kortex_driver::srv::I2CRead>("interconnect_config/i2_c_read", &InterconnectConfigSimulationServices::I2CRead, this);
	m_serviceI2CReadRegister = m_node_handle->create_service<kortex_driver::srv::I2CReadRegister>("interconnect_config/i2_c_read_register", &InterconnectConfigSimulationServices::I2CReadRegister, this);
	m_serviceI2CWrite = m_node_handle->create_service<kortex_driver::srv::I2CWrite>("interconnect_config/i2_c_write", &InterconnectConfigSimulationServices::I2CWrite, this);
	m_serviceI2CWriteRegister = m_node_handle->create_service<kortex_driver::srv::I2CWriteRegister>("interconnect_config/i2_c_write_register", &InterconnectConfigSimulationServices::I2CWriteRegister, this);
}

bool InterconnectConfigSimulationServices::SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}

bool InterconnectConfigSimulationServices::SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}


bool InterconnectConfigSimulationServices::GetUARTConfiguration(kortex_driver::srv::GetUARTConfiguration::Request  &req, kortex_driver::srv::GetUARTConfiguration::Response &res)
{
	
	
	if (GetUARTConfigurationHandler)
	{
		res = GetUARTConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/get_u_a_r_t_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::SetUARTConfiguration(kortex_driver::srv::SetUARTConfiguration::Request  &req, kortex_driver::srv::SetUARTConfiguration::Response &res)
{
	
	
	if (SetUARTConfigurationHandler)
	{
		res = SetUARTConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/set_u_a_r_t_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::GetEthernetConfiguration(kortex_driver::srv::GetEthernetConfiguration::Request  &req, kortex_driver::srv::GetEthernetConfiguration::Response &res)
{
	
	
	if (GetEthernetConfigurationHandler)
	{
		res = GetEthernetConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/get_ethernet_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::SetEthernetConfiguration(kortex_driver::srv::SetEthernetConfiguration::Request  &req, kortex_driver::srv::SetEthernetConfiguration::Response &res)
{
	
	
	if (SetEthernetConfigurationHandler)
	{
		res = SetEthernetConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/set_ethernet_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::GetGPIOConfiguration(kortex_driver::srv::GetGPIOConfiguration::Request  &req, kortex_driver::srv::GetGPIOConfiguration::Response &res)
{
	
	
	if (GetGPIOConfigurationHandler)
	{
		res = GetGPIOConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/get_g_p_i_o_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::SetGPIOConfiguration(kortex_driver::srv::SetGPIOConfiguration::Request  &req, kortex_driver::srv::SetGPIOConfiguration::Response &res)
{
	
	
	if (SetGPIOConfigurationHandler)
	{
		res = SetGPIOConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/set_g_p_i_o_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::GetGPIOState(kortex_driver::srv::GetGPIOState::Request  &req, kortex_driver::srv::GetGPIOState::Response &res)
{
	
	
	if (GetGPIOStateHandler)
	{
		res = GetGPIOStateHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/get_g_p_i_o_state is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::SetGPIOState(kortex_driver::srv::SetGPIOState::Request  &req, kortex_driver::srv::SetGPIOState::Response &res)
{
	
	
	if (SetGPIOStateHandler)
	{
		res = SetGPIOStateHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/set_g_p_i_o_state is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::GetI2CConfiguration(kortex_driver::srv::GetI2CConfiguration::Request  &req, kortex_driver::srv::GetI2CConfiguration::Response &res)
{
	
	
	if (GetI2CConfigurationHandler)
	{
		res = GetI2CConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/get_i2_c_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::SetI2CConfiguration(kortex_driver::srv::SetI2CConfiguration::Request  &req, kortex_driver::srv::SetI2CConfiguration::Response &res)
{
	
	
	if (SetI2CConfigurationHandler)
	{
		res = SetI2CConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/set_i2_c_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::I2CRead(kortex_driver::srv::I2CRead::Request  &req, kortex_driver::srv::I2CRead::Response &res)
{
	
	
	if (I2CReadHandler)
	{
		res = I2CReadHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/i2_c_read is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::I2CReadRegister(kortex_driver::srv::I2CReadRegister::Request  &req, kortex_driver::srv::I2CReadRegister::Response &res)
{
	
	
	if (I2CReadRegisterHandler)
	{
		res = I2CReadRegisterHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/i2_c_read_register is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::I2CWrite(kortex_driver::srv::I2CWrite::Request  &req, kortex_driver::srv::I2CWrite::Response &res)
{
	
	
	if (I2CWriteHandler)
	{
		res = I2CWriteHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/i2_c_write is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool InterconnectConfigSimulationServices::I2CWriteRegister(kortex_driver::srv::I2CWriteRegister::Request  &req, kortex_driver::srv::I2CWriteRegister::Response &res)
{
	
	
	if (I2CWriteRegisterHandler)
	{
		res = I2CWriteRegisterHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for interconnect_config/i2_c_write_register is not implemented, so the service calls will return the default response.");
	}
	return true;
}
