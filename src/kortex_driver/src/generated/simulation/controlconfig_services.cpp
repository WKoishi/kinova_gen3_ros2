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
#include "kortex_driver/generated/simulation/controlconfig_services.h"

ControlConfigSimulationServices::ControlConfigSimulationServices(rclcpp::Node::SharedPtr node_handle): 
	IControlConfigServices(node_handle)
{
	m_pub_Error = m_node_handle.advertise<kortex_driver::msg::KortexError>("kortex_error", 1000);
	m_pub_ControlConfigurationTopic = m_node_handle.advertise<kortex_driver::msg::ControlConfigurationNotification>("control_configuration_topic", 1000);
	m_is_activated_ControlConfigurationTopic = false;
	m_pub_ControlModeTopic = m_node_handle.advertise<kortex_driver::msg::ControlConfigControlModeNotification>("control_mode_topic", 1000);
	m_is_activated_ControlModeTopic = false;

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("control_config/set_device_id", std::bind(&ControlConfigSimulationServices::SetDeviceID, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("control_config/set_api_options", std::bind(&ControlConfigSimulationServices::SetApiOptions, this, std::placeholders::_1, std::placeholders::_2));

	m_serviceSetGravityVector = m_node_handle->create_service<kortex_driver::srv::SetGravityVector>("control_config/set_gravity_vector", std::bind(&ControlConfigSimulationServices::SetGravityVector, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetGravityVector = m_node_handle->create_service<kortex_driver::srv::GetGravityVector>("control_config/get_gravity_vector", std::bind(&ControlConfigSimulationServices::GetGravityVector, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetPayloadInformation = m_node_handle->create_service<kortex_driver::srv::SetPayloadInformation>("control_config/set_payload_information", std::bind(&ControlConfigSimulationServices::SetPayloadInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetPayloadInformation = m_node_handle->create_service<kortex_driver::srv::GetPayloadInformation>("control_config/get_payload_information", std::bind(&ControlConfigSimulationServices::GetPayloadInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetToolConfiguration = m_node_handle->create_service<kortex_driver::srv::SetToolConfiguration>("control_config/set_tool_configuration", std::bind(&ControlConfigSimulationServices::SetToolConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetToolConfiguration = m_node_handle->create_service<kortex_driver::srv::GetToolConfiguration>("control_config/get_tool_configuration", std::bind(&ControlConfigSimulationServices::GetToolConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationControlConfigurationTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationControlConfigurationTopic>("control_config/activate_publishing_of_control_configuration_topic", std::bind(&ControlConfigSimulationServices::OnNotificationControlConfigurationTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceControlConfig_Unsubscribe = m_node_handle->create_service<kortex_driver::srv::ControlConfigUnsubscribe>("control_config/unsubscribe", std::bind(&ControlConfigSimulationServices::ControlConfig_Unsubscribe, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetCartesianReferenceFrame = m_node_handle->create_service<kortex_driver::srv::SetCartesianReferenceFrame>("control_config/set_cartesian_reference_frame", std::bind(&ControlConfigSimulationServices::SetCartesianReferenceFrame, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetCartesianReferenceFrame = m_node_handle->create_service<kortex_driver::srv::GetCartesianReferenceFrame>("control_config/get_cartesian_reference_frame", std::bind(&ControlConfigSimulationServices::GetCartesianReferenceFrame, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceControlConfig_GetControlMode = m_node_handle->create_service<kortex_driver::srv::ControlConfigGetControlMode>("control_config/get_control_mode", std::bind(&ControlConfigSimulationServices::ControlConfig_GetControlMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetJointSpeedSoftLimits = m_node_handle->create_service<kortex_driver::srv::SetJointSpeedSoftLimits>("control_config/set_joint_speed_soft_limits", std::bind(&ControlConfigSimulationServices::SetJointSpeedSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetTwistLinearSoftLimit = m_node_handle->create_service<kortex_driver::srv::SetTwistLinearSoftLimit>("control_config/set_twist_linear_soft_limit", std::bind(&ControlConfigSimulationServices::SetTwistLinearSoftLimit, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetTwistAngularSoftLimit = m_node_handle->create_service<kortex_driver::srv::SetTwistAngularSoftLimit>("control_config/set_twist_angular_soft_limit", std::bind(&ControlConfigSimulationServices::SetTwistAngularSoftLimit, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetJointAccelerationSoftLimits = m_node_handle->create_service<kortex_driver::srv::SetJointAccelerationSoftLimits>("control_config/set_joint_acceleration_soft_limits", std::bind(&ControlConfigSimulationServices::SetJointAccelerationSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetKinematicHardLimits = m_node_handle->create_service<kortex_driver::srv::GetKinematicHardLimits>("control_config/get_kinematic_hard_limits", std::bind(&ControlConfigSimulationServices::GetKinematicHardLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetKinematicSoftLimits = m_node_handle->create_service<kortex_driver::srv::GetKinematicSoftLimits>("control_config/get_kinematic_soft_limits", std::bind(&ControlConfigSimulationServices::GetKinematicSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllKinematicSoftLimits = m_node_handle->create_service<kortex_driver::srv::GetAllKinematicSoftLimits>("control_config/get_all_kinematic_soft_limits", std::bind(&ControlConfigSimulationServices::GetAllKinematicSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetDesiredLinearTwist = m_node_handle->create_service<kortex_driver::srv::SetDesiredLinearTwist>("control_config/set_desired_linear_twist", std::bind(&ControlConfigSimulationServices::SetDesiredLinearTwist, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetDesiredAngularTwist = m_node_handle->create_service<kortex_driver::srv::SetDesiredAngularTwist>("control_config/set_desired_angular_twist", std::bind(&ControlConfigSimulationServices::SetDesiredAngularTwist, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetDesiredJointSpeeds = m_node_handle->create_service<kortex_driver::srv::SetDesiredJointSpeeds>("control_config/set_desired_joint_speeds", std::bind(&ControlConfigSimulationServices::SetDesiredJointSpeeds, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetDesiredSpeeds = m_node_handle->create_service<kortex_driver::srv::GetDesiredSpeeds>("control_config/get_desired_speeds", std::bind(&ControlConfigSimulationServices::GetDesiredSpeeds, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetGravityVector = m_node_handle->create_service<kortex_driver::srv::ResetGravityVector>("control_config/reset_gravity_vector", std::bind(&ControlConfigSimulationServices::ResetGravityVector, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetPayloadInformation = m_node_handle->create_service<kortex_driver::srv::ResetPayloadInformation>("control_config/reset_payload_information", std::bind(&ControlConfigSimulationServices::ResetPayloadInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetToolConfiguration = m_node_handle->create_service<kortex_driver::srv::ResetToolConfiguration>("control_config/reset_tool_configuration", std::bind(&ControlConfigSimulationServices::ResetToolConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetJointSpeedSoftLimits = m_node_handle->create_service<kortex_driver::srv::ResetJointSpeedSoftLimits>("control_config/reset_joint_speed_soft_limits", std::bind(&ControlConfigSimulationServices::ResetJointSpeedSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetTwistLinearSoftLimit = m_node_handle->create_service<kortex_driver::srv::ResetTwistLinearSoftLimit>("control_config/reset_twist_linear_soft_limit", std::bind(&ControlConfigSimulationServices::ResetTwistLinearSoftLimit, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetTwistAngularSoftLimit = m_node_handle->create_service<kortex_driver::srv::ResetTwistAngularSoftLimit>("control_config/reset_twist_angular_soft_limit", std::bind(&ControlConfigSimulationServices::ResetTwistAngularSoftLimit, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetJointAccelerationSoftLimits = m_node_handle->create_service<kortex_driver::srv::ResetJointAccelerationSoftLimits>("control_config/reset_joint_acceleration_soft_limits", std::bind(&ControlConfigSimulationServices::ResetJointAccelerationSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceControlConfig_OnNotificationControlModeTopic = m_node_handle->create_service<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic>("control_config/activate_publishing_of_control_mode_topic", std::bind(&ControlConfigSimulationServices::ControlConfig_OnNotificationControlModeTopic, this, std::placeholders::_1, std::placeholders::_2));
}

bool ControlConfigSimulationServices::SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}

bool ControlConfigSimulationServices::SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}


bool ControlConfigSimulationServices::SetGravityVector(kortex_driver::srv::SetGravityVector::Request  &req, kortex_driver::srv::SetGravityVector::Response &res)
{
	
	
	if (SetGravityVectorHandler)
	{
		res = SetGravityVectorHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/set_gravity_vector is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::GetGravityVector(kortex_driver::srv::GetGravityVector::Request  &req, kortex_driver::srv::GetGravityVector::Response &res)
{
	
	
	if (GetGravityVectorHandler)
	{
		res = GetGravityVectorHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/get_gravity_vector is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::SetPayloadInformation(kortex_driver::srv::SetPayloadInformation::Request  &req, kortex_driver::srv::SetPayloadInformation::Response &res)
{
	
	
	if (SetPayloadInformationHandler)
	{
		res = SetPayloadInformationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/set_payload_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::GetPayloadInformation(kortex_driver::srv::GetPayloadInformation::Request  &req, kortex_driver::srv::GetPayloadInformation::Response &res)
{
	
	
	if (GetPayloadInformationHandler)
	{
		res = GetPayloadInformationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/get_payload_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::SetToolConfiguration(kortex_driver::srv::SetToolConfiguration::Request  &req, kortex_driver::srv::SetToolConfiguration::Response &res)
{
	
	
	if (SetToolConfigurationHandler)
	{
		res = SetToolConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/set_tool_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::GetToolConfiguration(kortex_driver::srv::GetToolConfiguration::Request  &req, kortex_driver::srv::GetToolConfiguration::Response &res)
{
	
	
	if (GetToolConfigurationHandler)
	{
		res = GetToolConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/get_tool_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::OnNotificationControlConfigurationTopic(kortex_driver::srv::OnNotificationControlConfigurationTopic::Request  &req, kortex_driver::srv::OnNotificationControlConfigurationTopic::Response &res)
{
	
	m_is_activated_ControlConfigurationTopic = true;
	
	if (OnNotificationControlConfigurationTopicHandler)
	{
		res = OnNotificationControlConfigurationTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/activate_publishing_of_control_configuration_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void ControlConfigSimulationServices::cb_ControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification notif)
{
	kortex_driver::msg::ControlConfigurationNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlConfigurationTopic->publish(ros_msg);
}

bool ControlConfigSimulationServices::ControlConfig_Unsubscribe(kortex_driver::srv::ControlConfigUnsubscribe::Request  &req, kortex_driver::srv::ControlConfigUnsubscribe::Response &res)
{
	
	
	if (ControlConfig_UnsubscribeHandler)
	{
		res = ControlConfig_UnsubscribeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/unsubscribe is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::SetCartesianReferenceFrame(kortex_driver::srv::SetCartesianReferenceFrame::Request  &req, kortex_driver::srv::SetCartesianReferenceFrame::Response &res)
{
	
	
	if (SetCartesianReferenceFrameHandler)
	{
		res = SetCartesianReferenceFrameHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/set_cartesian_reference_frame is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::GetCartesianReferenceFrame(kortex_driver::srv::GetCartesianReferenceFrame::Request  &req, kortex_driver::srv::GetCartesianReferenceFrame::Response &res)
{
	
	
	if (GetCartesianReferenceFrameHandler)
	{
		res = GetCartesianReferenceFrameHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/get_cartesian_reference_frame is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::ControlConfig_GetControlMode(kortex_driver::srv::ControlConfigGetControlMode::Request  &req, kortex_driver::srv::ControlConfigGetControlMode::Response &res)
{
	
	
	if (ControlConfig_GetControlModeHandler)
	{
		res = ControlConfig_GetControlModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/get_control_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::SetJointSpeedSoftLimits(kortex_driver::srv::SetJointSpeedSoftLimits::Request  &req, kortex_driver::srv::SetJointSpeedSoftLimits::Response &res)
{
	
	
	if (SetJointSpeedSoftLimitsHandler)
	{
		res = SetJointSpeedSoftLimitsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/set_joint_speed_soft_limits is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::SetTwistLinearSoftLimit(kortex_driver::srv::SetTwistLinearSoftLimit::Request  &req, kortex_driver::srv::SetTwistLinearSoftLimit::Response &res)
{
	
	
	if (SetTwistLinearSoftLimitHandler)
	{
		res = SetTwistLinearSoftLimitHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/set_twist_linear_soft_limit is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::SetTwistAngularSoftLimit(kortex_driver::srv::SetTwistAngularSoftLimit::Request  &req, kortex_driver::srv::SetTwistAngularSoftLimit::Response &res)
{
	
	
	if (SetTwistAngularSoftLimitHandler)
	{
		res = SetTwistAngularSoftLimitHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/set_twist_angular_soft_limit is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::SetJointAccelerationSoftLimits(kortex_driver::srv::SetJointAccelerationSoftLimits::Request  &req, kortex_driver::srv::SetJointAccelerationSoftLimits::Response &res)
{
	
	
	if (SetJointAccelerationSoftLimitsHandler)
	{
		res = SetJointAccelerationSoftLimitsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/set_joint_acceleration_soft_limits is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::GetKinematicHardLimits(kortex_driver::srv::GetKinematicHardLimits::Request  &req, kortex_driver::srv::GetKinematicHardLimits::Response &res)
{
	
	
	if (GetKinematicHardLimitsHandler)
	{
		res = GetKinematicHardLimitsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/get_kinematic_hard_limits is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::GetKinematicSoftLimits(kortex_driver::srv::GetKinematicSoftLimits::Request  &req, kortex_driver::srv::GetKinematicSoftLimits::Response &res)
{
	
	
	if (GetKinematicSoftLimitsHandler)
	{
		res = GetKinematicSoftLimitsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/get_kinematic_soft_limits is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::GetAllKinematicSoftLimits(kortex_driver::srv::GetAllKinematicSoftLimits::Request  &req, kortex_driver::srv::GetAllKinematicSoftLimits::Response &res)
{
	
	
	if (GetAllKinematicSoftLimitsHandler)
	{
		res = GetAllKinematicSoftLimitsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/get_all_kinematic_soft_limits is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::SetDesiredLinearTwist(kortex_driver::srv::SetDesiredLinearTwist::Request  &req, kortex_driver::srv::SetDesiredLinearTwist::Response &res)
{
	
	
	if (SetDesiredLinearTwistHandler)
	{
		res = SetDesiredLinearTwistHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/set_desired_linear_twist is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::SetDesiredAngularTwist(kortex_driver::srv::SetDesiredAngularTwist::Request  &req, kortex_driver::srv::SetDesiredAngularTwist::Response &res)
{
	
	
	if (SetDesiredAngularTwistHandler)
	{
		res = SetDesiredAngularTwistHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/set_desired_angular_twist is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::SetDesiredJointSpeeds(kortex_driver::srv::SetDesiredJointSpeeds::Request  &req, kortex_driver::srv::SetDesiredJointSpeeds::Response &res)
{
	
	
	if (SetDesiredJointSpeedsHandler)
	{
		res = SetDesiredJointSpeedsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/set_desired_joint_speeds is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::GetDesiredSpeeds(kortex_driver::srv::GetDesiredSpeeds::Request  &req, kortex_driver::srv::GetDesiredSpeeds::Response &res)
{
	
	
	if (GetDesiredSpeedsHandler)
	{
		res = GetDesiredSpeedsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/get_desired_speeds is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::ResetGravityVector(kortex_driver::srv::ResetGravityVector::Request  &req, kortex_driver::srv::ResetGravityVector::Response &res)
{
	
	
	if (ResetGravityVectorHandler)
	{
		res = ResetGravityVectorHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/reset_gravity_vector is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::ResetPayloadInformation(kortex_driver::srv::ResetPayloadInformation::Request  &req, kortex_driver::srv::ResetPayloadInformation::Response &res)
{
	
	
	if (ResetPayloadInformationHandler)
	{
		res = ResetPayloadInformationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/reset_payload_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::ResetToolConfiguration(kortex_driver::srv::ResetToolConfiguration::Request  &req, kortex_driver::srv::ResetToolConfiguration::Response &res)
{
	
	
	if (ResetToolConfigurationHandler)
	{
		res = ResetToolConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/reset_tool_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::ResetJointSpeedSoftLimits(kortex_driver::srv::ResetJointSpeedSoftLimits::Request  &req, kortex_driver::srv::ResetJointSpeedSoftLimits::Response &res)
{
	
	
	if (ResetJointSpeedSoftLimitsHandler)
	{
		res = ResetJointSpeedSoftLimitsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/reset_joint_speed_soft_limits is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::ResetTwistLinearSoftLimit(kortex_driver::srv::ResetTwistLinearSoftLimit::Request  &req, kortex_driver::srv::ResetTwistLinearSoftLimit::Response &res)
{
	
	
	if (ResetTwistLinearSoftLimitHandler)
	{
		res = ResetTwistLinearSoftLimitHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/reset_twist_linear_soft_limit is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::ResetTwistAngularSoftLimit(kortex_driver::srv::ResetTwistAngularSoftLimit::Request  &req, kortex_driver::srv::ResetTwistAngularSoftLimit::Response &res)
{
	
	
	if (ResetTwistAngularSoftLimitHandler)
	{
		res = ResetTwistAngularSoftLimitHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/reset_twist_angular_soft_limit is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::ResetJointAccelerationSoftLimits(kortex_driver::srv::ResetJointAccelerationSoftLimits::Request  &req, kortex_driver::srv::ResetJointAccelerationSoftLimits::Response &res)
{
	
	
	if (ResetJointAccelerationSoftLimitsHandler)
	{
		res = ResetJointAccelerationSoftLimitsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/reset_joint_acceleration_soft_limits is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ControlConfigSimulationServices::ControlConfig_OnNotificationControlModeTopic(kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Request  &req, kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Response &res)
{
	
	m_is_activated_ControlModeTopic = true;
	
	if (ControlConfig_OnNotificationControlModeTopicHandler)
	{
		res = ControlConfig_OnNotificationControlModeTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for control_config/activate_publishing_of_control_mode_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void ControlConfigSimulationServices::cb_ControlModeTopic(Kinova::Api::ControlConfig::ControlModeNotification notif)
{
	kortex_driver::msg::ControlConfigControlModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlModeTopic->publish(ros_msg);
}
