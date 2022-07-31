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
#include "kortex_driver/generated/simulation/base_services.h"

BaseSimulationServices::BaseSimulationServices(rclcpp::Node::SharedPtr node_handle): 
	IBaseServices(node_handle)
{
	m_pub_Error = m_node_handle->create_publisher<kortex_driver::msg::KortexError>("kortex_error", 1000);
	m_pub_ConfigurationChangeTopic = m_node_handle->create_publisher<kortex_driver::msg::ConfigurationChangeNotification>("configuration_change_topic", 1000);
	m_is_activated_ConfigurationChangeTopic = false;
	m_pub_MappingInfoTopic = m_node_handle->create_publisher<kortex_driver::msg::MappingInfoNotification>("mapping_info_topic", 1000);
	m_is_activated_MappingInfoTopic = false;
	m_pub_ControlModeTopic = m_node_handle->create_publisher<kortex_driver::msg::BaseControlModeNotification>("control_mode_topic", 1000);
	m_is_activated_ControlModeTopic = false;
	m_pub_OperatingModeTopic = m_node_handle->create_publisher<kortex_driver::msg::OperatingModeNotification>("operating_mode_topic", 1000);
	m_is_activated_OperatingModeTopic = false;
	m_pub_SequenceInfoTopic = m_node_handle->create_publisher<kortex_driver::msg::SequenceInfoNotification>("sequence_info_topic", 1000);
	m_is_activated_SequenceInfoTopic = false;
	m_pub_ProtectionZoneTopic = m_node_handle->create_publisher<kortex_driver::msg::ProtectionZoneNotification>("protection_zone_topic", 1000);
	m_is_activated_ProtectionZoneTopic = false;
	m_pub_UserTopic = m_node_handle->create_publisher<kortex_driver::msg::UserNotification>("user_topic", 1000);
	m_is_activated_UserTopic = false;
	m_pub_ControllerTopic = m_node_handle->create_publisher<kortex_driver::msg::ControllerNotification>("controller_topic", 1000);
	m_is_activated_ControllerTopic = false;
	m_pub_ActionTopic = m_node_handle->create_publisher<kortex_driver::msg::ActionNotification>("action_topic", 1000);
	m_is_activated_ActionTopic = false;
	m_pub_RobotEventTopic = m_node_handle->create_publisher<kortex_driver::msg::RobotEventNotification>("robot_event_topic", 1000);
	m_is_activated_RobotEventTopic = false;
	m_pub_ServoingModeTopic = m_node_handle->create_publisher<kortex_driver::msg::ServoingModeNotification>("servoing_mode_topic", 1000);
	m_is_activated_ServoingModeTopic = false;
	m_pub_FactoryTopic = m_node_handle->create_publisher<kortex_driver::msg::FactoryNotification>("factory_topic", 1000);
	m_is_activated_FactoryTopic = false;
	m_pub_NetworkTopic = m_node_handle->create_publisher<kortex_driver::msg::NetworkNotification>("network_topic", 1000);
	m_is_activated_NetworkTopic = false;
	m_pub_ArmStateTopic = m_node_handle->create_publisher<kortex_driver::msg::ArmStateNotification>("arm_state_topic", 1000);
	m_is_activated_ArmStateTopic = false;

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("base/set_device_id", std::bind(&BaseSimulationServices::SetDeviceID, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("base/set_api_options", std::bind(&BaseSimulationServices::SetApiOptions, this, std::placeholders::_1, std::placeholders::_2));

	m_serviceCreateUserProfile = m_node_handle->create_service<kortex_driver::srv::CreateUserProfile>("base/create_user_profile", std::bind(&BaseSimulationServices::CreateUserProfile, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateUserProfile = m_node_handle->create_service<kortex_driver::srv::UpdateUserProfile>("base/update_user_profile", std::bind(&BaseSimulationServices::UpdateUserProfile, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadUserProfile = m_node_handle->create_service<kortex_driver::srv::ReadUserProfile>("base/read_user_profile", std::bind(&BaseSimulationServices::ReadUserProfile, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteUserProfile = m_node_handle->create_service<kortex_driver::srv::DeleteUserProfile>("base/delete_user_profile", std::bind(&BaseSimulationServices::DeleteUserProfile, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllUserProfiles = m_node_handle->create_service<kortex_driver::srv::ReadAllUserProfiles>("base/read_all_user_profiles", std::bind(&BaseSimulationServices::ReadAllUserProfiles, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllUsers = m_node_handle->create_service<kortex_driver::srv::ReadAllUsers>("base/read_all_users", std::bind(&BaseSimulationServices::ReadAllUsers, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceChangePassword = m_node_handle->create_service<kortex_driver::srv::ChangePassword>("base/change_password", std::bind(&BaseSimulationServices::ChangePassword, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceCreateSequence = m_node_handle->create_service<kortex_driver::srv::CreateSequence>("base/create_sequence", std::bind(&BaseSimulationServices::CreateSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateSequence = m_node_handle->create_service<kortex_driver::srv::UpdateSequence>("base/update_sequence", std::bind(&BaseSimulationServices::UpdateSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadSequence = m_node_handle->create_service<kortex_driver::srv::ReadSequence>("base/read_sequence", std::bind(&BaseSimulationServices::ReadSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteSequence = m_node_handle->create_service<kortex_driver::srv::DeleteSequence>("base/delete_sequence", std::bind(&BaseSimulationServices::DeleteSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllSequences = m_node_handle->create_service<kortex_driver::srv::ReadAllSequences>("base/read_all_sequences", std::bind(&BaseSimulationServices::ReadAllSequences, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlaySequence = m_node_handle->create_service<kortex_driver::srv::PlaySequence>("base/play_sequence", std::bind(&BaseSimulationServices::PlaySequence, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayAdvancedSequence = m_node_handle->create_service<kortex_driver::srv::PlayAdvancedSequence>("base/play_advanced_sequence", std::bind(&BaseSimulationServices::PlayAdvancedSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStopSequence = m_node_handle->create_service<kortex_driver::srv::StopSequence>("base/stop_sequence", std::bind(&BaseSimulationServices::StopSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePauseSequence = m_node_handle->create_service<kortex_driver::srv::PauseSequence>("base/pause_sequence", std::bind(&BaseSimulationServices::PauseSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResumeSequence = m_node_handle->create_service<kortex_driver::srv::ResumeSequence>("base/resume_sequence", std::bind(&BaseSimulationServices::ResumeSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceCreateProtectionZone = m_node_handle->create_service<kortex_driver::srv::CreateProtectionZone>("base/create_protection_zone", std::bind(&BaseSimulationServices::CreateProtectionZone, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateProtectionZone = m_node_handle->create_service<kortex_driver::srv::UpdateProtectionZone>("base/update_protection_zone", std::bind(&BaseSimulationServices::UpdateProtectionZone, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadProtectionZone = m_node_handle->create_service<kortex_driver::srv::ReadProtectionZone>("base/read_protection_zone", std::bind(&BaseSimulationServices::ReadProtectionZone, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteProtectionZone = m_node_handle->create_service<kortex_driver::srv::DeleteProtectionZone>("base/delete_protection_zone", std::bind(&BaseSimulationServices::DeleteProtectionZone, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllProtectionZones = m_node_handle->create_service<kortex_driver::srv::ReadAllProtectionZones>("base/read_all_protection_zones", std::bind(&BaseSimulationServices::ReadAllProtectionZones, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceCreateMapping = m_node_handle->create_service<kortex_driver::srv::CreateMapping>("base/create_mapping", std::bind(&BaseSimulationServices::CreateMapping, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadMapping = m_node_handle->create_service<kortex_driver::srv::ReadMapping>("base/read_mapping", std::bind(&BaseSimulationServices::ReadMapping, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateMapping = m_node_handle->create_service<kortex_driver::srv::UpdateMapping>("base/update_mapping", std::bind(&BaseSimulationServices::UpdateMapping, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteMapping = m_node_handle->create_service<kortex_driver::srv::DeleteMapping>("base/delete_mapping", std::bind(&BaseSimulationServices::DeleteMapping, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllMappings = m_node_handle->create_service<kortex_driver::srv::ReadAllMappings>("base/read_all_mappings", std::bind(&BaseSimulationServices::ReadAllMappings, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceCreateMap = m_node_handle->create_service<kortex_driver::srv::CreateMap>("base/create_map", std::bind(&BaseSimulationServices::CreateMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadMap = m_node_handle->create_service<kortex_driver::srv::ReadMap>("base/read_map", std::bind(&BaseSimulationServices::ReadMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateMap = m_node_handle->create_service<kortex_driver::srv::UpdateMap>("base/update_map", std::bind(&BaseSimulationServices::UpdateMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteMap = m_node_handle->create_service<kortex_driver::srv::DeleteMap>("base/delete_map", std::bind(&BaseSimulationServices::DeleteMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllMaps = m_node_handle->create_service<kortex_driver::srv::ReadAllMaps>("base/read_all_maps", std::bind(&BaseSimulationServices::ReadAllMaps, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceActivateMap = m_node_handle->create_service<kortex_driver::srv::ActivateMap>("base/activate_map", std::bind(&BaseSimulationServices::ActivateMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceCreateAction = m_node_handle->create_service<kortex_driver::srv::CreateAction>("base/create_action", std::bind(&BaseSimulationServices::CreateAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAction = m_node_handle->create_service<kortex_driver::srv::ReadAction>("base/read_action", std::bind(&BaseSimulationServices::ReadAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllActions = m_node_handle->create_service<kortex_driver::srv::ReadAllActions>("base/read_all_actions", std::bind(&BaseSimulationServices::ReadAllActions, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteAction = m_node_handle->create_service<kortex_driver::srv::DeleteAction>("base/delete_action", std::bind(&BaseSimulationServices::DeleteAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateAction = m_node_handle->create_service<kortex_driver::srv::UpdateAction>("base/update_action", std::bind(&BaseSimulationServices::UpdateAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceExecuteActionFromReference = m_node_handle->create_service<kortex_driver::srv::ExecuteActionFromReference>("base/execute_action_from_reference", std::bind(&BaseSimulationServices::ExecuteActionFromReference, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceExecuteAction = m_node_handle->create_service<kortex_driver::srv::ExecuteAction>("base/execute_action", std::bind(&BaseSimulationServices::ExecuteAction, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePauseAction = m_node_handle->create_service<kortex_driver::srv::PauseAction>("base/pause_action", std::bind(&BaseSimulationServices::PauseAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStopAction = m_node_handle->create_service<kortex_driver::srv::StopAction>("base/stop_action", std::bind(&BaseSimulationServices::StopAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResumeAction = m_node_handle->create_service<kortex_driver::srv::ResumeAction>("base/resume_action", std::bind(&BaseSimulationServices::ResumeAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetIPv4Configuration = m_node_handle->create_service<kortex_driver::srv::GetIPv4Configuration>("base/get_i_pv4_configuration", std::bind(&BaseSimulationServices::GetIPv4Configuration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetIPv4Configuration = m_node_handle->create_service<kortex_driver::srv::SetIPv4Configuration>("base/set_i_pv4_configuration", std::bind(&BaseSimulationServices::SetIPv4Configuration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetCommunicationInterfaceEnable = m_node_handle->create_service<kortex_driver::srv::SetCommunicationInterfaceEnable>("base/set_communication_interface_enable", std::bind(&BaseSimulationServices::SetCommunicationInterfaceEnable, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceIsCommunicationInterfaceEnable = m_node_handle->create_service<kortex_driver::srv::IsCommunicationInterfaceEnable>("base/is_communication_interface_enable", std::bind(&BaseSimulationServices::IsCommunicationInterfaceEnable, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAvailableWifi = m_node_handle->create_service<kortex_driver::srv::GetAvailableWifi>("base/get_available_wifi", std::bind(&BaseSimulationServices::GetAvailableWifi, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetWifiInformation = m_node_handle->create_service<kortex_driver::srv::GetWifiInformation>("base/get_wifi_information", std::bind(&BaseSimulationServices::GetWifiInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceAddWifiConfiguration = m_node_handle->create_service<kortex_driver::srv::AddWifiConfiguration>("base/add_wifi_configuration", std::bind(&BaseSimulationServices::AddWifiConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteWifiConfiguration = m_node_handle->create_service<kortex_driver::srv::DeleteWifiConfiguration>("base/delete_wifi_configuration", std::bind(&BaseSimulationServices::DeleteWifiConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllConfiguredWifis = m_node_handle->create_service<kortex_driver::srv::GetAllConfiguredWifis>("base/get_all_configured_wifis", std::bind(&BaseSimulationServices::GetAllConfiguredWifis, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceConnectWifi = m_node_handle->create_service<kortex_driver::srv::ConnectWifi>("base/connect_wifi", std::bind(&BaseSimulationServices::ConnectWifi, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDisconnectWifi = m_node_handle->create_service<kortex_driver::srv::DisconnectWifi>("base/disconnect_wifi", std::bind(&BaseSimulationServices::DisconnectWifi, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetConnectedWifiInformation = m_node_handle->create_service<kortex_driver::srv::GetConnectedWifiInformation>("base/get_connected_wifi_information", std::bind(&BaseSimulationServices::GetConnectedWifiInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_Unsubscribe = m_node_handle->create_service<kortex_driver::srv::BaseUnsubscribe>("base/unsubscribe", std::bind(&BaseSimulationServices::Base_Unsubscribe, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationConfigurationChangeTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationConfigurationChangeTopic>("base/activate_publishing_of_configuration_change_topic", std::bind(&BaseSimulationServices::OnNotificationConfigurationChangeTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationMappingInfoTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationMappingInfoTopic>("base/activate_publishing_of_mapping_info_topic", std::bind(&BaseSimulationServices::OnNotificationMappingInfoTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_OnNotificationControlModeTopic = m_node_handle->create_service<kortex_driver::srv::BaseOnNotificationControlModeTopic>("base/activate_publishing_of_control_mode_topic", std::bind(&BaseSimulationServices::Base_OnNotificationControlModeTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationOperatingModeTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationOperatingModeTopic>("base/activate_publishing_of_operating_mode_topic", std::bind(&BaseSimulationServices::OnNotificationOperatingModeTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationSequenceInfoTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationSequenceInfoTopic>("base/activate_publishing_of_sequence_info_topic", std::bind(&BaseSimulationServices::OnNotificationSequenceInfoTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationProtectionZoneTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationProtectionZoneTopic>("base/activate_publishing_of_protection_zone_topic", std::bind(&BaseSimulationServices::OnNotificationProtectionZoneTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationUserTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationUserTopic>("base/activate_publishing_of_user_topic", std::bind(&BaseSimulationServices::OnNotificationUserTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationControllerTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationControllerTopic>("base/activate_publishing_of_controller_topic", std::bind(&BaseSimulationServices::OnNotificationControllerTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationActionTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationActionTopic>("base/activate_publishing_of_action_topic", std::bind(&BaseSimulationServices::OnNotificationActionTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationRobotEventTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationRobotEventTopic>("base/activate_publishing_of_robot_event_topic", std::bind(&BaseSimulationServices::OnNotificationRobotEventTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayCartesianTrajectory = m_node_handle->create_service<kortex_driver::srv::PlayCartesianTrajectory>("base/play_cartesian_trajectory", std::bind(&BaseSimulationServices::PlayCartesianTrajectory, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayCartesianTrajectoryPosition = m_node_handle->create_service<kortex_driver::srv::PlayCartesianTrajectoryPosition>("base/play_cartesian_trajectory_position", std::bind(&BaseSimulationServices::PlayCartesianTrajectoryPosition, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayCartesianTrajectoryOrientation = m_node_handle->create_service<kortex_driver::srv::PlayCartesianTrajectoryOrientation>("base/play_cartesian_trajectory_orientation", std::bind(&BaseSimulationServices::PlayCartesianTrajectoryOrientation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStop = m_node_handle->create_service<kortex_driver::srv::Stop>("base/stop", std::bind(&BaseSimulationServices::Stop, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetMeasuredCartesianPose = m_node_handle->create_service<kortex_driver::srv::GetMeasuredCartesianPose>("base/get_measured_cartesian_pose", std::bind(&BaseSimulationServices::GetMeasuredCartesianPose, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendWrenchCommand = m_node_handle->create_service<kortex_driver::srv::SendWrenchCommand>("base/send_wrench_command", std::bind(&BaseSimulationServices::SendWrenchCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendWrenchJoystickCommand = m_node_handle->create_service<kortex_driver::srv::SendWrenchJoystickCommand>("base/send_wrench_joystick_command", std::bind(&BaseSimulationServices::SendWrenchJoystickCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendTwistJoystickCommand = m_node_handle->create_service<kortex_driver::srv::SendTwistJoystickCommand>("base/send_twist_joystick_command", std::bind(&BaseSimulationServices::SendTwistJoystickCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendTwistCommand = m_node_handle->create_service<kortex_driver::srv::SendTwistCommand>("base/send_twist_command", std::bind(&BaseSimulationServices::SendTwistCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayJointTrajectory = m_node_handle->create_service<kortex_driver::srv::PlayJointTrajectory>("base/play_joint_trajectory", std::bind(&BaseSimulationServices::PlayJointTrajectory, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlaySelectedJointTrajectory = m_node_handle->create_service<kortex_driver::srv::PlaySelectedJointTrajectory>("base/play_selected_joint_trajectory", std::bind(&BaseSimulationServices::PlaySelectedJointTrajectory, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetMeasuredJointAngles = m_node_handle->create_service<kortex_driver::srv::GetMeasuredJointAngles>("base/get_measured_joint_angles", std::bind(&BaseSimulationServices::GetMeasuredJointAngles, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendJointSpeedsCommand = m_node_handle->create_service<kortex_driver::srv::SendJointSpeedsCommand>("base/send_joint_speeds_command", std::bind(&BaseSimulationServices::SendJointSpeedsCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendSelectedJointSpeedCommand = m_node_handle->create_service<kortex_driver::srv::SendSelectedJointSpeedCommand>("base/send_selected_joint_speed_command", std::bind(&BaseSimulationServices::SendSelectedJointSpeedCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendGripperCommand = m_node_handle->create_service<kortex_driver::srv::SendGripperCommand>("base/send_gripper_command", std::bind(&BaseSimulationServices::SendGripperCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetMeasuredGripperMovement = m_node_handle->create_service<kortex_driver::srv::GetMeasuredGripperMovement>("base/get_measured_gripper_movement", std::bind(&BaseSimulationServices::GetMeasuredGripperMovement, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetAdmittance = m_node_handle->create_service<kortex_driver::srv::SetAdmittance>("base/set_admittance", std::bind(&BaseSimulationServices::SetAdmittance, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetOperatingMode = m_node_handle->create_service<kortex_driver::srv::SetOperatingMode>("base/set_operating_mode", std::bind(&BaseSimulationServices::SetOperatingMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceApplyEmergencyStop = m_node_handle->create_service<kortex_driver::srv::ApplyEmergencyStop>("base/apply_emergency_stop", std::bind(&BaseSimulationServices::ApplyEmergencyStop, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_ClearFaults = m_node_handle->create_service<kortex_driver::srv::BaseClearFaults>("base/clear_faults", std::bind(&BaseSimulationServices::Base_ClearFaults, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_GetControlMode = m_node_handle->create_service<kortex_driver::srv::BaseGetControlMode>("base/get_control_mode", std::bind(&BaseSimulationServices::Base_GetControlMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetOperatingMode = m_node_handle->create_service<kortex_driver::srv::GetOperatingMode>("base/get_operating_mode", std::bind(&BaseSimulationServices::GetOperatingMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetServoingMode = m_node_handle->create_service<kortex_driver::srv::SetServoingMode>("base/set_servoing_mode", std::bind(&BaseSimulationServices::SetServoingMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetServoingMode = m_node_handle->create_service<kortex_driver::srv::GetServoingMode>("base/get_servoing_mode", std::bind(&BaseSimulationServices::GetServoingMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationServoingModeTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationServoingModeTopic>("base/activate_publishing_of_servoing_mode_topic", std::bind(&BaseSimulationServices::OnNotificationServoingModeTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceRestoreFactorySettings = m_node_handle->create_service<kortex_driver::srv::RestoreFactorySettings>("base/restore_factory_settings", std::bind(&BaseSimulationServices::RestoreFactorySettings, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationFactoryTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationFactoryTopic>("base/activate_publishing_of_factory_topic", std::bind(&BaseSimulationServices::OnNotificationFactoryTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllConnectedControllers = m_node_handle->create_service<kortex_driver::srv::GetAllConnectedControllers>("base/get_all_connected_controllers", std::bind(&BaseSimulationServices::GetAllConnectedControllers, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetControllerState = m_node_handle->create_service<kortex_driver::srv::GetControllerState>("base/get_controller_state", std::bind(&BaseSimulationServices::GetControllerState, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetActuatorCount = m_node_handle->create_service<kortex_driver::srv::GetActuatorCount>("base/get_actuator_count", std::bind(&BaseSimulationServices::GetActuatorCount, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStartWifiScan = m_node_handle->create_service<kortex_driver::srv::StartWifiScan>("base/start_wifi_scan", std::bind(&BaseSimulationServices::StartWifiScan, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetConfiguredWifi = m_node_handle->create_service<kortex_driver::srv::GetConfiguredWifi>("base/get_configured_wifi", std::bind(&BaseSimulationServices::GetConfiguredWifi, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationNetworkTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationNetworkTopic>("base/activate_publishing_of_network_topic", std::bind(&BaseSimulationServices::OnNotificationNetworkTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetArmState = m_node_handle->create_service<kortex_driver::srv::GetArmState>("base/get_arm_state", std::bind(&BaseSimulationServices::GetArmState, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationArmStateTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationArmStateTopic>("base/activate_publishing_of_arm_state_topic", std::bind(&BaseSimulationServices::OnNotificationArmStateTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetIPv4Information = m_node_handle->create_service<kortex_driver::srv::GetIPv4Information>("base/get_i_pv4_information", std::bind(&BaseSimulationServices::GetIPv4Information, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetWifiCountryCode = m_node_handle->create_service<kortex_driver::srv::SetWifiCountryCode>("base/set_wifi_country_code", std::bind(&BaseSimulationServices::SetWifiCountryCode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetWifiCountryCode = m_node_handle->create_service<kortex_driver::srv::GetWifiCountryCode>("base/get_wifi_country_code", std::bind(&BaseSimulationServices::GetWifiCountryCode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_SetCapSenseConfig = m_node_handle->create_service<kortex_driver::srv::BaseSetCapSenseConfig>("base/set_cap_sense_config", std::bind(&BaseSimulationServices::Base_SetCapSenseConfig, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_GetCapSenseConfig = m_node_handle->create_service<kortex_driver::srv::BaseGetCapSenseConfig>("base/get_cap_sense_config", std::bind(&BaseSimulationServices::Base_GetCapSenseConfig, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllJointsSpeedHardLimitation = m_node_handle->create_service<kortex_driver::srv::GetAllJointsSpeedHardLimitation>("base/get_all_joints_speed_hard_limitation", std::bind(&BaseSimulationServices::GetAllJointsSpeedHardLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllJointsTorqueHardLimitation = m_node_handle->create_service<kortex_driver::srv::GetAllJointsTorqueHardLimitation>("base/get_all_joints_torque_hard_limitation", std::bind(&BaseSimulationServices::GetAllJointsTorqueHardLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetTwistHardLimitation = m_node_handle->create_service<kortex_driver::srv::GetTwistHardLimitation>("base/get_twist_hard_limitation", std::bind(&BaseSimulationServices::GetTwistHardLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetWrenchHardLimitation = m_node_handle->create_service<kortex_driver::srv::GetWrenchHardLimitation>("base/get_wrench_hard_limitation", std::bind(&BaseSimulationServices::GetWrenchHardLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendJointSpeedsJoystickCommand = m_node_handle->create_service<kortex_driver::srv::SendJointSpeedsJoystickCommand>("base/send_joint_speeds_joystick_command", std::bind(&BaseSimulationServices::SendJointSpeedsJoystickCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendSelectedJointSpeedJoystickCommand = m_node_handle->create_service<kortex_driver::srv::SendSelectedJointSpeedJoystickCommand>("base/send_selected_joint_speed_joystick_command", std::bind(&BaseSimulationServices::SendSelectedJointSpeedJoystickCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceEnableBridge = m_node_handle->create_service<kortex_driver::srv::EnableBridge>("base/enable_bridge", std::bind(&BaseSimulationServices::EnableBridge, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDisableBridge = m_node_handle->create_service<kortex_driver::srv::DisableBridge>("base/disable_bridge", std::bind(&BaseSimulationServices::DisableBridge, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetBridgeList = m_node_handle->create_service<kortex_driver::srv::GetBridgeList>("base/get_bridge_list", std::bind(&BaseSimulationServices::GetBridgeList, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetBridgeConfig = m_node_handle->create_service<kortex_driver::srv::GetBridgeConfig>("base/get_bridge_config", std::bind(&BaseSimulationServices::GetBridgeConfig, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayPreComputedJointTrajectory = m_node_handle->create_service<kortex_driver::srv::PlayPreComputedJointTrajectory>("base/play_pre_computed_joint_trajectory", std::bind(&BaseSimulationServices::PlayPreComputedJointTrajectory, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetProductConfiguration = m_node_handle->create_service<kortex_driver::srv::GetProductConfiguration>("base/get_product_configuration", std::bind(&BaseSimulationServices::GetProductConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateEndEffectorTypeConfiguration = m_node_handle->create_service<kortex_driver::srv::UpdateEndEffectorTypeConfiguration>("base/update_end_effector_type_configuration", std::bind(&BaseSimulationServices::UpdateEndEffectorTypeConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceRestoreFactoryProductConfiguration = m_node_handle->create_service<kortex_driver::srv::RestoreFactoryProductConfiguration>("base/restore_factory_product_configuration", std::bind(&BaseSimulationServices::RestoreFactoryProductConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetTrajectoryErrorReport = m_node_handle->create_service<kortex_driver::srv::GetTrajectoryErrorReport>("base/get_trajectory_error_report", std::bind(&BaseSimulationServices::GetTrajectoryErrorReport, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllJointsSpeedSoftLimitation = m_node_handle->create_service<kortex_driver::srv::GetAllJointsSpeedSoftLimitation>("base/get_all_joints_speed_soft_limitation", std::bind(&BaseSimulationServices::GetAllJointsSpeedSoftLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllJointsTorqueSoftLimitation = m_node_handle->create_service<kortex_driver::srv::GetAllJointsTorqueSoftLimitation>("base/get_all_joints_torque_soft_limitation", std::bind(&BaseSimulationServices::GetAllJointsTorqueSoftLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetTwistSoftLimitation = m_node_handle->create_service<kortex_driver::srv::GetTwistSoftLimitation>("base/get_twist_soft_limitation", std::bind(&BaseSimulationServices::GetTwistSoftLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetWrenchSoftLimitation = m_node_handle->create_service<kortex_driver::srv::GetWrenchSoftLimitation>("base/get_wrench_soft_limitation", std::bind(&BaseSimulationServices::GetWrenchSoftLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetControllerConfigurationMode = m_node_handle->create_service<kortex_driver::srv::SetControllerConfigurationMode>("base/set_controller_configuration_mode", std::bind(&BaseSimulationServices::SetControllerConfigurationMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetControllerConfigurationMode = m_node_handle->create_service<kortex_driver::srv::GetControllerConfigurationMode>("base/get_controller_configuration_mode", std::bind(&BaseSimulationServices::GetControllerConfigurationMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStartTeaching = m_node_handle->create_service<kortex_driver::srv::StartTeaching>("base/start_teaching", std::bind(&BaseSimulationServices::StartTeaching, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStopTeaching = m_node_handle->create_service<kortex_driver::srv::StopTeaching>("base/stop_teaching", std::bind(&BaseSimulationServices::StopTeaching, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceAddSequenceTasks = m_node_handle->create_service<kortex_driver::srv::AddSequenceTasks>("base/add_sequence_tasks", std::bind(&BaseSimulationServices::AddSequenceTasks, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateSequenceTask = m_node_handle->create_service<kortex_driver::srv::UpdateSequenceTask>("base/update_sequence_task", std::bind(&BaseSimulationServices::UpdateSequenceTask, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSwapSequenceTasks = m_node_handle->create_service<kortex_driver::srv::SwapSequenceTasks>("base/swap_sequence_tasks", std::bind(&BaseSimulationServices::SwapSequenceTasks, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadSequenceTask = m_node_handle->create_service<kortex_driver::srv::ReadSequenceTask>("base/read_sequence_task", std::bind(&BaseSimulationServices::ReadSequenceTask, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllSequenceTasks = m_node_handle->create_service<kortex_driver::srv::ReadAllSequenceTasks>("base/read_all_sequence_tasks", std::bind(&BaseSimulationServices::ReadAllSequenceTasks, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteSequenceTask = m_node_handle->create_service<kortex_driver::srv::DeleteSequenceTask>("base/delete_sequence_task", std::bind(&BaseSimulationServices::DeleteSequenceTask, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteAllSequenceTasks = m_node_handle->create_service<kortex_driver::srv::DeleteAllSequenceTasks>("base/delete_all_sequence_tasks", std::bind(&BaseSimulationServices::DeleteAllSequenceTasks, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceTakeSnapshot = m_node_handle->create_service<kortex_driver::srv::TakeSnapshot>("base/take_snapshot", std::bind(&BaseSimulationServices::TakeSnapshot, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetFirmwareBundleVersions = m_node_handle->create_service<kortex_driver::srv::GetFirmwareBundleVersions>("base/get_firmware_bundle_versions", std::bind(&BaseSimulationServices::GetFirmwareBundleVersions, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceExecuteWaypointTrajectory = m_node_handle->create_service<kortex_driver::srv::ExecuteWaypointTrajectory>("base/execute_waypoint_trajectory", std::bind(&BaseSimulationServices::ExecuteWaypointTrajectory, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceMoveSequenceTask = m_node_handle->create_service<kortex_driver::srv::MoveSequenceTask>("base/move_sequence_task", std::bind(&BaseSimulationServices::MoveSequenceTask, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDuplicateMapping = m_node_handle->create_service<kortex_driver::srv::DuplicateMapping>("base/duplicate_mapping", std::bind(&BaseSimulationServices::DuplicateMapping, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDuplicateMap = m_node_handle->create_service<kortex_driver::srv::DuplicateMap>("base/duplicate_map", std::bind(&BaseSimulationServices::DuplicateMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetControllerConfiguration = m_node_handle->create_service<kortex_driver::srv::SetControllerConfiguration>("base/set_controller_configuration", std::bind(&BaseSimulationServices::SetControllerConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetControllerConfiguration = m_node_handle->create_service<kortex_driver::srv::GetControllerConfiguration>("base/get_controller_configuration", std::bind(&BaseSimulationServices::GetControllerConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllControllerConfigurations = m_node_handle->create_service<kortex_driver::srv::GetAllControllerConfigurations>("base/get_all_controller_configurations", std::bind(&BaseSimulationServices::GetAllControllerConfigurations, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceComputeForwardKinematics = m_node_handle->create_service<kortex_driver::srv::ComputeForwardKinematics>("base/compute_forward_kinematics", std::bind(&BaseSimulationServices::ComputeForwardKinematics, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceComputeInverseKinematics = m_node_handle->create_service<kortex_driver::srv::ComputeInverseKinematics>("base/compute_inverse_kinematics", std::bind(&BaseSimulationServices::ComputeInverseKinematics, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceValidateWaypointList = m_node_handle->create_service<kortex_driver::srv::ValidateWaypointList>("base/validate_waypoint_list", std::bind(&BaseSimulationServices::ValidateWaypointList, this, std::placeholders::_1, std::placeholders::_2));
}

bool BaseSimulationServices::SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res)
{
	RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}

bool BaseSimulationServices::SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res)
{
	RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}


bool BaseSimulationServices::CreateUserProfile(const std::shared_ptr<kortex_driver::srv::CreateUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::CreateUserProfile::Response> res)
{
	
	
	if (CreateUserProfileHandler)
	{
		res = CreateUserProfileHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/create_user_profile is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateUserProfile(const std::shared_ptr<kortex_driver::srv::UpdateUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::UpdateUserProfile::Response> res)
{
	
	
	if (UpdateUserProfileHandler)
	{
		res = UpdateUserProfileHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/update_user_profile is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadUserProfile(const std::shared_ptr<kortex_driver::srv::ReadUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::ReadUserProfile::Response> res)
{
	
	
	if (ReadUserProfileHandler)
	{
		res = ReadUserProfileHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_user_profile is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteUserProfile(const std::shared_ptr<kortex_driver::srv::DeleteUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::DeleteUserProfile::Response> res)
{
	
	
	if (DeleteUserProfileHandler)
	{
		res = DeleteUserProfileHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/delete_user_profile is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllUserProfiles(const std::shared_ptr<kortex_driver::srv::ReadAllUserProfiles::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllUserProfiles::Response> res)
{
	
	
	if (ReadAllUserProfilesHandler)
	{
		res = ReadAllUserProfilesHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_all_user_profiles is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllUsers(const std::shared_ptr<kortex_driver::srv::ReadAllUsers::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllUsers::Response> res)
{
	
	
	if (ReadAllUsersHandler)
	{
		res = ReadAllUsersHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_all_users is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ChangePassword(const std::shared_ptr<kortex_driver::srv::ChangePassword::Request> req, std::shared_ptr<kortex_driver::srv::ChangePassword::Response> res)
{
	
	
	if (ChangePasswordHandler)
	{
		res = ChangePasswordHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/change_password is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::CreateSequence(const std::shared_ptr<kortex_driver::srv::CreateSequence::Request> req, std::shared_ptr<kortex_driver::srv::CreateSequence::Response> res)
{
	
	
	if (CreateSequenceHandler)
	{
		res = CreateSequenceHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/create_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateSequence(const std::shared_ptr<kortex_driver::srv::UpdateSequence::Request> req, std::shared_ptr<kortex_driver::srv::UpdateSequence::Response> res)
{
	
	
	if (UpdateSequenceHandler)
	{
		res = UpdateSequenceHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/update_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadSequence(const std::shared_ptr<kortex_driver::srv::ReadSequence::Request> req, std::shared_ptr<kortex_driver::srv::ReadSequence::Response> res)
{
	
	
	if (ReadSequenceHandler)
	{
		res = ReadSequenceHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteSequence(const std::shared_ptr<kortex_driver::srv::DeleteSequence::Request> req, std::shared_ptr<kortex_driver::srv::DeleteSequence::Response> res)
{
	
	
	if (DeleteSequenceHandler)
	{
		res = DeleteSequenceHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/delete_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllSequences(const std::shared_ptr<kortex_driver::srv::ReadAllSequences::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllSequences::Response> res)
{
	
	
	if (ReadAllSequencesHandler)
	{
		res = ReadAllSequencesHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_all_sequences is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlaySequence(const std::shared_ptr<kortex_driver::srv::PlaySequence::Request> req, std::shared_ptr<kortex_driver::srv::PlaySequence::Response> res)
{
	
	
	if (PlaySequenceHandler)
	{
		res = PlaySequenceHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/play_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlayAdvancedSequence(const std::shared_ptr<kortex_driver::srv::PlayAdvancedSequence::Request> req, std::shared_ptr<kortex_driver::srv::PlayAdvancedSequence::Response> res)
{
	
	
	if (PlayAdvancedSequenceHandler)
	{
		res = PlayAdvancedSequenceHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/play_advanced_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::StopSequence(const std::shared_ptr<kortex_driver::srv::StopSequence::Request> req, std::shared_ptr<kortex_driver::srv::StopSequence::Response> res)
{
	
	
	if (StopSequenceHandler)
	{
		res = StopSequenceHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/stop_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PauseSequence(const std::shared_ptr<kortex_driver::srv::PauseSequence::Request> req, std::shared_ptr<kortex_driver::srv::PauseSequence::Response> res)
{
	
	
	if (PauseSequenceHandler)
	{
		res = PauseSequenceHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/pause_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ResumeSequence(const std::shared_ptr<kortex_driver::srv::ResumeSequence::Request> req, std::shared_ptr<kortex_driver::srv::ResumeSequence::Response> res)
{
	
	
	if (ResumeSequenceHandler)
	{
		res = ResumeSequenceHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/resume_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::CreateProtectionZone(const std::shared_ptr<kortex_driver::srv::CreateProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::CreateProtectionZone::Response> res)
{
	
	
	if (CreateProtectionZoneHandler)
	{
		res = CreateProtectionZoneHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/create_protection_zone is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateProtectionZone(const std::shared_ptr<kortex_driver::srv::UpdateProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::UpdateProtectionZone::Response> res)
{
	
	
	if (UpdateProtectionZoneHandler)
	{
		res = UpdateProtectionZoneHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/update_protection_zone is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadProtectionZone(const std::shared_ptr<kortex_driver::srv::ReadProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::ReadProtectionZone::Response> res)
{
	
	
	if (ReadProtectionZoneHandler)
	{
		res = ReadProtectionZoneHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_protection_zone is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteProtectionZone(const std::shared_ptr<kortex_driver::srv::DeleteProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::DeleteProtectionZone::Response> res)
{
	
	
	if (DeleteProtectionZoneHandler)
	{
		res = DeleteProtectionZoneHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/delete_protection_zone is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllProtectionZones(const std::shared_ptr<kortex_driver::srv::ReadAllProtectionZones::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllProtectionZones::Response> res)
{
	
	
	if (ReadAllProtectionZonesHandler)
	{
		res = ReadAllProtectionZonesHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_all_protection_zones is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::CreateMapping(const std::shared_ptr<kortex_driver::srv::CreateMapping::Request> req, std::shared_ptr<kortex_driver::srv::CreateMapping::Response> res)
{
	
	
	if (CreateMappingHandler)
	{
		res = CreateMappingHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/create_mapping is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadMapping(const std::shared_ptr<kortex_driver::srv::ReadMapping::Request> req, std::shared_ptr<kortex_driver::srv::ReadMapping::Response> res)
{
	
	
	if (ReadMappingHandler)
	{
		res = ReadMappingHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_mapping is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateMapping(const std::shared_ptr<kortex_driver::srv::UpdateMapping::Request> req, std::shared_ptr<kortex_driver::srv::UpdateMapping::Response> res)
{
	
	
	if (UpdateMappingHandler)
	{
		res = UpdateMappingHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/update_mapping is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteMapping(const std::shared_ptr<kortex_driver::srv::DeleteMapping::Request> req, std::shared_ptr<kortex_driver::srv::DeleteMapping::Response> res)
{
	
	
	if (DeleteMappingHandler)
	{
		res = DeleteMappingHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/delete_mapping is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllMappings(const std::shared_ptr<kortex_driver::srv::ReadAllMappings::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllMappings::Response> res)
{
	
	
	if (ReadAllMappingsHandler)
	{
		res = ReadAllMappingsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_all_mappings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::CreateMap(const std::shared_ptr<kortex_driver::srv::CreateMap::Request> req, std::shared_ptr<kortex_driver::srv::CreateMap::Response> res)
{
	
	
	if (CreateMapHandler)
	{
		res = CreateMapHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/create_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadMap(const std::shared_ptr<kortex_driver::srv::ReadMap::Request> req, std::shared_ptr<kortex_driver::srv::ReadMap::Response> res)
{
	
	
	if (ReadMapHandler)
	{
		res = ReadMapHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateMap(const std::shared_ptr<kortex_driver::srv::UpdateMap::Request> req, std::shared_ptr<kortex_driver::srv::UpdateMap::Response> res)
{
	
	
	if (UpdateMapHandler)
	{
		res = UpdateMapHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/update_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteMap(const std::shared_ptr<kortex_driver::srv::DeleteMap::Request> req, std::shared_ptr<kortex_driver::srv::DeleteMap::Response> res)
{
	
	
	if (DeleteMapHandler)
	{
		res = DeleteMapHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/delete_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllMaps(const std::shared_ptr<kortex_driver::srv::ReadAllMaps::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllMaps::Response> res)
{
	
	
	if (ReadAllMapsHandler)
	{
		res = ReadAllMapsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_all_maps is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ActivateMap(const std::shared_ptr<kortex_driver::srv::ActivateMap::Request> req, std::shared_ptr<kortex_driver::srv::ActivateMap::Response> res)
{
	
	
	if (ActivateMapHandler)
	{
		res = ActivateMapHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::CreateAction(const std::shared_ptr<kortex_driver::srv::CreateAction::Request> req, std::shared_ptr<kortex_driver::srv::CreateAction::Response> res)
{
	
	
	if (CreateActionHandler)
	{
		res = CreateActionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/create_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAction(const std::shared_ptr<kortex_driver::srv::ReadAction::Request> req, std::shared_ptr<kortex_driver::srv::ReadAction::Response> res)
{
	
	
	if (ReadActionHandler)
	{
		res = ReadActionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllActions(const std::shared_ptr<kortex_driver::srv::ReadAllActions::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllActions::Response> res)
{
	
	
	if (ReadAllActionsHandler)
	{
		res = ReadAllActionsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_all_actions is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteAction(const std::shared_ptr<kortex_driver::srv::DeleteAction::Request> req, std::shared_ptr<kortex_driver::srv::DeleteAction::Response> res)
{
	
	
	if (DeleteActionHandler)
	{
		res = DeleteActionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/delete_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateAction(const std::shared_ptr<kortex_driver::srv::UpdateAction::Request> req, std::shared_ptr<kortex_driver::srv::UpdateAction::Response> res)
{
	
	
	if (UpdateActionHandler)
	{
		res = UpdateActionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/update_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ExecuteActionFromReference(const std::shared_ptr<kortex_driver::srv::ExecuteActionFromReference::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteActionFromReference::Response> res)
{
	
	
	if (ExecuteActionFromReferenceHandler)
	{
		res = ExecuteActionFromReferenceHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/execute_action_from_reference is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ExecuteAction(const std::shared_ptr<kortex_driver::srv::ExecuteAction::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteAction::Response> res)
{
	
	
	if (ExecuteActionHandler)
	{
		res = ExecuteActionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/execute_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PauseAction(const std::shared_ptr<kortex_driver::srv::PauseAction::Request> req, std::shared_ptr<kortex_driver::srv::PauseAction::Response> res)
{
	
	
	if (PauseActionHandler)
	{
		res = PauseActionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/pause_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::StopAction(const std::shared_ptr<kortex_driver::srv::StopAction::Request> req, std::shared_ptr<kortex_driver::srv::StopAction::Response> res)
{
	
	
	if (StopActionHandler)
	{
		res = StopActionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/stop_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ResumeAction(const std::shared_ptr<kortex_driver::srv::ResumeAction::Request> req, std::shared_ptr<kortex_driver::srv::ResumeAction::Response> res)
{
	
	
	if (ResumeActionHandler)
	{
		res = ResumeActionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/resume_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetIPv4Configuration(const std::shared_ptr<kortex_driver::srv::GetIPv4Configuration::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Configuration::Response> res)
{
	
	
	if (GetIPv4ConfigurationHandler)
	{
		res = GetIPv4ConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_i_pv4_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetIPv4Configuration(const std::shared_ptr<kortex_driver::srv::SetIPv4Configuration::Request> req, std::shared_ptr<kortex_driver::srv::SetIPv4Configuration::Response> res)
{
	
	
	if (SetIPv4ConfigurationHandler)
	{
		res = SetIPv4ConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/set_i_pv4_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetCommunicationInterfaceEnable(const std::shared_ptr<kortex_driver::srv::SetCommunicationInterfaceEnable::Request> req, std::shared_ptr<kortex_driver::srv::SetCommunicationInterfaceEnable::Response> res)
{
	
	
	if (SetCommunicationInterfaceEnableHandler)
	{
		res = SetCommunicationInterfaceEnableHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/set_communication_interface_enable is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::IsCommunicationInterfaceEnable(const std::shared_ptr<kortex_driver::srv::IsCommunicationInterfaceEnable::Request> req, std::shared_ptr<kortex_driver::srv::IsCommunicationInterfaceEnable::Response> res)
{
	
	
	if (IsCommunicationInterfaceEnableHandler)
	{
		res = IsCommunicationInterfaceEnableHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/is_communication_interface_enable is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAvailableWifi(const std::shared_ptr<kortex_driver::srv::GetAvailableWifi::Request> req, std::shared_ptr<kortex_driver::srv::GetAvailableWifi::Response> res)
{
	
	
	if (GetAvailableWifiHandler)
	{
		res = GetAvailableWifiHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_available_wifi is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetWifiInformation(const std::shared_ptr<kortex_driver::srv::GetWifiInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetWifiInformation::Response> res)
{
	
	
	if (GetWifiInformationHandler)
	{
		res = GetWifiInformationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_wifi_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::AddWifiConfiguration(const std::shared_ptr<kortex_driver::srv::AddWifiConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::AddWifiConfiguration::Response> res)
{
	
	
	if (AddWifiConfigurationHandler)
	{
		res = AddWifiConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/add_wifi_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteWifiConfiguration(const std::shared_ptr<kortex_driver::srv::DeleteWifiConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::DeleteWifiConfiguration::Response> res)
{
	
	
	if (DeleteWifiConfigurationHandler)
	{
		res = DeleteWifiConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/delete_wifi_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllConfiguredWifis(const std::shared_ptr<kortex_driver::srv::GetAllConfiguredWifis::Request> req, std::shared_ptr<kortex_driver::srv::GetAllConfiguredWifis::Response> res)
{
	
	
	if (GetAllConfiguredWifisHandler)
	{
		res = GetAllConfiguredWifisHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_all_configured_wifis is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ConnectWifi(const std::shared_ptr<kortex_driver::srv::ConnectWifi::Request> req, std::shared_ptr<kortex_driver::srv::ConnectWifi::Response> res)
{
	
	
	if (ConnectWifiHandler)
	{
		res = ConnectWifiHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/connect_wifi is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DisconnectWifi(const std::shared_ptr<kortex_driver::srv::DisconnectWifi::Request> req, std::shared_ptr<kortex_driver::srv::DisconnectWifi::Response> res)
{
	
	
	if (DisconnectWifiHandler)
	{
		res = DisconnectWifiHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/disconnect_wifi is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetConnectedWifiInformation(const std::shared_ptr<kortex_driver::srv::GetConnectedWifiInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetConnectedWifiInformation::Response> res)
{
	
	
	if (GetConnectedWifiInformationHandler)
	{
		res = GetConnectedWifiInformationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_connected_wifi_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Base_Unsubscribe(const std::shared_ptr<kortex_driver::srv::BaseUnsubscribe::Request> req, std::shared_ptr<kortex_driver::srv::BaseUnsubscribe::Response> res)
{
	
	
	if (Base_UnsubscribeHandler)
	{
		res = Base_UnsubscribeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/unsubscribe is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::OnNotificationConfigurationChangeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationConfigurationChangeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationConfigurationChangeTopic::Response> res)
{
	
	m_is_activated_ConfigurationChangeTopic = true;
	
	if (OnNotificationConfigurationChangeTopicHandler)
	{
		res = OnNotificationConfigurationChangeTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_configuration_change_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif)
{
	kortex_driver::msg::ConfigurationChangeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ConfigurationChangeTopic->publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationMappingInfoTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationMappingInfoTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationMappingInfoTopic::Response> res)
{
	
	m_is_activated_MappingInfoTopic = true;
	
	if (OnNotificationMappingInfoTopicHandler)
	{
		res = OnNotificationMappingInfoTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_mapping_info_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif)
{
	kortex_driver::msg::MappingInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_MappingInfoTopic->publish(ros_msg);
}

bool BaseSimulationServices::Base_OnNotificationControlModeTopic(const std::shared_ptr<kortex_driver::srv::BaseOnNotificationControlModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::BaseOnNotificationControlModeTopic::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/activate_publishing_of_control_mode_topic service is now deprecated and will be removed in a future release.");
	
	m_is_activated_ControlModeTopic = true;
	
	if (Base_OnNotificationControlModeTopicHandler)
	{
		res = Base_OnNotificationControlModeTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_control_mode_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif)
{
	kortex_driver::msg::BaseControlModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlModeTopic->publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationOperatingModeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationOperatingModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationOperatingModeTopic::Response> res)
{
	
	m_is_activated_OperatingModeTopic = true;
	
	if (OnNotificationOperatingModeTopicHandler)
	{
		res = OnNotificationOperatingModeTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_operating_mode_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif)
{
	kortex_driver::msg::OperatingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_OperatingModeTopic->publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationSequenceInfoTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationSequenceInfoTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationSequenceInfoTopic::Response> res)
{
	
	m_is_activated_SequenceInfoTopic = true;
	
	if (OnNotificationSequenceInfoTopicHandler)
	{
		res = OnNotificationSequenceInfoTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_sequence_info_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif)
{
	kortex_driver::msg::SequenceInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SequenceInfoTopic->publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationProtectionZoneTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationProtectionZoneTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationProtectionZoneTopic::Response> res)
{
	
	m_is_activated_ProtectionZoneTopic = true;
	
	if (OnNotificationProtectionZoneTopicHandler)
	{
		res = OnNotificationProtectionZoneTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_protection_zone_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif)
{
	kortex_driver::msg::ProtectionZoneNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ProtectionZoneTopic->publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationUserTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationUserTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationUserTopic::Response> res)
{
	
	m_is_activated_UserTopic = true;
	
	if (OnNotificationUserTopicHandler)
	{
		res = OnNotificationUserTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_user_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_UserTopic(Kinova::Api::Base::UserNotification notif)
{
	kortex_driver::msg::UserNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_UserTopic->publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationControllerTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationControllerTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationControllerTopic::Response> res)
{
	
	m_is_activated_ControllerTopic = true;
	
	if (OnNotificationControllerTopicHandler)
	{
		res = OnNotificationControllerTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_controller_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif)
{
	kortex_driver::msg::ControllerNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControllerTopic->publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationActionTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationActionTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationActionTopic::Response> res)
{
	
	m_is_activated_ActionTopic = true;
	
	if (OnNotificationActionTopicHandler)
	{
		res = OnNotificationActionTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_action_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ActionTopic(Kinova::Api::Base::ActionNotification notif)
{
	kortex_driver::msg::ActionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ActionTopic->publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationRobotEventTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationRobotEventTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationRobotEventTopic::Response> res)
{
	
	m_is_activated_RobotEventTopic = true;
	
	if (OnNotificationRobotEventTopicHandler)
	{
		res = OnNotificationRobotEventTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_robot_event_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif)
{
	kortex_driver::msg::RobotEventNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_RobotEventTopic->publish(ros_msg);
}

bool BaseSimulationServices::PlayCartesianTrajectory(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectory::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/play_cartesian_trajectory service is now deprecated and will be removed in a future release.");
	
	
	if (PlayCartesianTrajectoryHandler)
	{
		res = PlayCartesianTrajectoryHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/play_cartesian_trajectory is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlayCartesianTrajectoryPosition(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryPosition::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryPosition::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/play_cartesian_trajectory_position service is now deprecated and will be removed in a future release.");
	
	
	if (PlayCartesianTrajectoryPositionHandler)
	{
		res = PlayCartesianTrajectoryPositionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/play_cartesian_trajectory_position is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlayCartesianTrajectoryOrientation(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryOrientation::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryOrientation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/play_cartesian_trajectory_orientation service is now deprecated and will be removed in a future release.");
	
	
	if (PlayCartesianTrajectoryOrientationHandler)
	{
		res = PlayCartesianTrajectoryOrientationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/play_cartesian_trajectory_orientation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Stop(const std::shared_ptr<kortex_driver::srv::Stop::Request> req, std::shared_ptr<kortex_driver::srv::Stop::Response> res)
{
	
	
	if (StopHandler)
	{
		res = StopHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/stop is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetMeasuredCartesianPose(const std::shared_ptr<kortex_driver::srv::GetMeasuredCartesianPose::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredCartesianPose::Response> res)
{
	
	
	if (GetMeasuredCartesianPoseHandler)
	{
		res = GetMeasuredCartesianPoseHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_measured_cartesian_pose is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendWrenchCommand(const std::shared_ptr<kortex_driver::srv::SendWrenchCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendWrenchCommand::Response> res)
{
	
	
	if (SendWrenchCommandHandler)
	{
		res = SendWrenchCommandHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/send_wrench_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendWrenchJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendWrenchJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendWrenchJoystickCommand::Response> res)
{
	
	
	if (SendWrenchJoystickCommandHandler)
	{
		res = SendWrenchJoystickCommandHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/send_wrench_joystick_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendTwistJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendTwistJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendTwistJoystickCommand::Response> res)
{
	
	
	if (SendTwistJoystickCommandHandler)
	{
		res = SendTwistJoystickCommandHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/send_twist_joystick_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendTwistCommand(const std::shared_ptr<kortex_driver::srv::SendTwistCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendTwistCommand::Response> res)
{
	
	
	if (SendTwistCommandHandler)
	{
		res = SendTwistCommandHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/send_twist_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlayJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlayJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayJointTrajectory::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/play_joint_trajectory service is now deprecated and will be removed in a future release.");
	
	
	if (PlayJointTrajectoryHandler)
	{
		res = PlayJointTrajectoryHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/play_joint_trajectory is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlaySelectedJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlaySelectedJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlaySelectedJointTrajectory::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/play_selected_joint_trajectory service is now deprecated and will be removed in a future release.");
	
	
	if (PlaySelectedJointTrajectoryHandler)
	{
		res = PlaySelectedJointTrajectoryHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/play_selected_joint_trajectory is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetMeasuredJointAngles(const std::shared_ptr<kortex_driver::srv::GetMeasuredJointAngles::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredJointAngles::Response> res)
{
	
	
	if (GetMeasuredJointAnglesHandler)
	{
		res = GetMeasuredJointAnglesHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_measured_joint_angles is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendJointSpeedsCommand(const std::shared_ptr<kortex_driver::srv::SendJointSpeedsCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendJointSpeedsCommand::Response> res)
{
	
	
	if (SendJointSpeedsCommandHandler)
	{
		res = SendJointSpeedsCommandHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/send_joint_speeds_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendSelectedJointSpeedCommand(const std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedCommand::Response> res)
{
	
	
	if (SendSelectedJointSpeedCommandHandler)
	{
		res = SendSelectedJointSpeedCommandHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/send_selected_joint_speed_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendGripperCommand(const std::shared_ptr<kortex_driver::srv::SendGripperCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendGripperCommand::Response> res)
{
	
	
	if (SendGripperCommandHandler)
	{
		res = SendGripperCommandHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/send_gripper_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetMeasuredGripperMovement(const std::shared_ptr<kortex_driver::srv::GetMeasuredGripperMovement::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredGripperMovement::Response> res)
{
	
	
	if (GetMeasuredGripperMovementHandler)
	{
		res = GetMeasuredGripperMovementHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_measured_gripper_movement is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetAdmittance(const std::shared_ptr<kortex_driver::srv::SetAdmittance::Request> req, std::shared_ptr<kortex_driver::srv::SetAdmittance::Response> res)
{
	
	
	if (SetAdmittanceHandler)
	{
		res = SetAdmittanceHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/set_admittance is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetOperatingMode(const std::shared_ptr<kortex_driver::srv::SetOperatingMode::Request> req, std::shared_ptr<kortex_driver::srv::SetOperatingMode::Response> res)
{
	
	
	if (SetOperatingModeHandler)
	{
		res = SetOperatingModeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/set_operating_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ApplyEmergencyStop(const std::shared_ptr<kortex_driver::srv::ApplyEmergencyStop::Request> req, std::shared_ptr<kortex_driver::srv::ApplyEmergencyStop::Response> res)
{
	
	
	if (ApplyEmergencyStopHandler)
	{
		res = ApplyEmergencyStopHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/apply_emergency_stop is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Base_ClearFaults(const std::shared_ptr<kortex_driver::srv::BaseClearFaults::Request> req, std::shared_ptr<kortex_driver::srv::BaseClearFaults::Response> res)
{
	
	
	if (Base_ClearFaultsHandler)
	{
		res = Base_ClearFaultsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/clear_faults is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Base_GetControlMode(const std::shared_ptr<kortex_driver::srv::BaseGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::BaseGetControlMode::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_control_mode service is now deprecated and will be removed in a future release.");
	
	
	if (Base_GetControlModeHandler)
	{
		res = Base_GetControlModeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_control_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetOperatingMode(const std::shared_ptr<kortex_driver::srv::GetOperatingMode::Request> req, std::shared_ptr<kortex_driver::srv::GetOperatingMode::Response> res)
{
	
	
	if (GetOperatingModeHandler)
	{
		res = GetOperatingModeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_operating_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetServoingMode(const std::shared_ptr<kortex_driver::srv::SetServoingMode::Request> req, std::shared_ptr<kortex_driver::srv::SetServoingMode::Response> res)
{
	
	
	if (SetServoingModeHandler)
	{
		res = SetServoingModeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/set_servoing_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetServoingMode(const std::shared_ptr<kortex_driver::srv::GetServoingMode::Request> req, std::shared_ptr<kortex_driver::srv::GetServoingMode::Response> res)
{
	
	
	if (GetServoingModeHandler)
	{
		res = GetServoingModeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_servoing_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::OnNotificationServoingModeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationServoingModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationServoingModeTopic::Response> res)
{
	
	m_is_activated_ServoingModeTopic = true;
	
	if (OnNotificationServoingModeTopicHandler)
	{
		res = OnNotificationServoingModeTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_servoing_mode_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif)
{
	kortex_driver::msg::ServoingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ServoingModeTopic->publish(ros_msg);
}

bool BaseSimulationServices::RestoreFactorySettings(const std::shared_ptr<kortex_driver::srv::RestoreFactorySettings::Request> req, std::shared_ptr<kortex_driver::srv::RestoreFactorySettings::Response> res)
{
	
	
	if (RestoreFactorySettingsHandler)
	{
		res = RestoreFactorySettingsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/restore_factory_settings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::OnNotificationFactoryTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationFactoryTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationFactoryTopic::Response> res)
{
	
	m_is_activated_FactoryTopic = true;
	
	if (OnNotificationFactoryTopicHandler)
	{
		res = OnNotificationFactoryTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_factory_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif)
{
	kortex_driver::msg::FactoryNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_FactoryTopic->publish(ros_msg);
}

bool BaseSimulationServices::GetAllConnectedControllers(const std::shared_ptr<kortex_driver::srv::GetAllConnectedControllers::Request> req, std::shared_ptr<kortex_driver::srv::GetAllConnectedControllers::Response> res)
{
	
	
	if (GetAllConnectedControllersHandler)
	{
		res = GetAllConnectedControllersHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_all_connected_controllers is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetControllerState(const std::shared_ptr<kortex_driver::srv::GetControllerState::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerState::Response> res)
{
	
	
	if (GetControllerStateHandler)
	{
		res = GetControllerStateHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_controller_state is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetActuatorCount(const std::shared_ptr<kortex_driver::srv::GetActuatorCount::Request> req, std::shared_ptr<kortex_driver::srv::GetActuatorCount::Response> res)
{
	
	
	if (GetActuatorCountHandler)
	{
		res = GetActuatorCountHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_actuator_count is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::StartWifiScan(const std::shared_ptr<kortex_driver::srv::StartWifiScan::Request> req, std::shared_ptr<kortex_driver::srv::StartWifiScan::Response> res)
{
	
	
	if (StartWifiScanHandler)
	{
		res = StartWifiScanHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/start_wifi_scan is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetConfiguredWifi(const std::shared_ptr<kortex_driver::srv::GetConfiguredWifi::Request> req, std::shared_ptr<kortex_driver::srv::GetConfiguredWifi::Response> res)
{
	
	
	if (GetConfiguredWifiHandler)
	{
		res = GetConfiguredWifiHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_configured_wifi is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::OnNotificationNetworkTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationNetworkTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationNetworkTopic::Response> res)
{
	
	m_is_activated_NetworkTopic = true;
	
	if (OnNotificationNetworkTopicHandler)
	{
		res = OnNotificationNetworkTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_network_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif)
{
	kortex_driver::msg::NetworkNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_NetworkTopic->publish(ros_msg);
}

bool BaseSimulationServices::GetArmState(const std::shared_ptr<kortex_driver::srv::GetArmState::Request> req, std::shared_ptr<kortex_driver::srv::GetArmState::Response> res)
{
	
	
	if (GetArmStateHandler)
	{
		res = GetArmStateHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_arm_state is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::OnNotificationArmStateTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationArmStateTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationArmStateTopic::Response> res)
{
	
	m_is_activated_ArmStateTopic = true;
	
	if (OnNotificationArmStateTopicHandler)
	{
		res = OnNotificationArmStateTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/activate_publishing_of_arm_state_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif)
{
	kortex_driver::msg::ArmStateNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ArmStateTopic->publish(ros_msg);
}

bool BaseSimulationServices::GetIPv4Information(const std::shared_ptr<kortex_driver::srv::GetIPv4Information::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Information::Response> res)
{
	
	
	if (GetIPv4InformationHandler)
	{
		res = GetIPv4InformationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_i_pv4_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetWifiCountryCode(const std::shared_ptr<kortex_driver::srv::SetWifiCountryCode::Request> req, std::shared_ptr<kortex_driver::srv::SetWifiCountryCode::Response> res)
{
	
	
	if (SetWifiCountryCodeHandler)
	{
		res = SetWifiCountryCodeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/set_wifi_country_code is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetWifiCountryCode(const std::shared_ptr<kortex_driver::srv::GetWifiCountryCode::Request> req, std::shared_ptr<kortex_driver::srv::GetWifiCountryCode::Response> res)
{
	
	
	if (GetWifiCountryCodeHandler)
	{
		res = GetWifiCountryCodeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_wifi_country_code is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Base_SetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::BaseSetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::BaseSetCapSenseConfig::Response> res)
{
	
	
	if (Base_SetCapSenseConfigHandler)
	{
		res = Base_SetCapSenseConfigHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/set_cap_sense_config is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Base_GetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::BaseGetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::BaseGetCapSenseConfig::Response> res)
{
	
	
	if (Base_GetCapSenseConfigHandler)
	{
		res = Base_GetCapSenseConfigHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_cap_sense_config is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllJointsSpeedHardLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedHardLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_all_joints_speed_hard_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetAllJointsSpeedHardLimitationHandler)
	{
		res = GetAllJointsSpeedHardLimitationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_all_joints_speed_hard_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllJointsTorqueHardLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueHardLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_all_joints_torque_hard_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetAllJointsTorqueHardLimitationHandler)
	{
		res = GetAllJointsTorqueHardLimitationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_all_joints_torque_hard_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetTwistHardLimitation(const std::shared_ptr<kortex_driver::srv::GetTwistHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetTwistHardLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_twist_hard_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetTwistHardLimitationHandler)
	{
		res = GetTwistHardLimitationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_twist_hard_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetWrenchHardLimitation(const std::shared_ptr<kortex_driver::srv::GetWrenchHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetWrenchHardLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_wrench_hard_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetWrenchHardLimitationHandler)
	{
		res = GetWrenchHardLimitationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_wrench_hard_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendJointSpeedsJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendJointSpeedsJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendJointSpeedsJoystickCommand::Response> res)
{
	
	
	if (SendJointSpeedsJoystickCommandHandler)
	{
		res = SendJointSpeedsJoystickCommandHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/send_joint_speeds_joystick_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendSelectedJointSpeedJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Response> res)
{
	
	
	if (SendSelectedJointSpeedJoystickCommandHandler)
	{
		res = SendSelectedJointSpeedJoystickCommandHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/send_selected_joint_speed_joystick_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::EnableBridge(const std::shared_ptr<kortex_driver::srv::EnableBridge::Request> req, std::shared_ptr<kortex_driver::srv::EnableBridge::Response> res)
{
	
	
	if (EnableBridgeHandler)
	{
		res = EnableBridgeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/enable_bridge is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DisableBridge(const std::shared_ptr<kortex_driver::srv::DisableBridge::Request> req, std::shared_ptr<kortex_driver::srv::DisableBridge::Response> res)
{
	
	
	if (DisableBridgeHandler)
	{
		res = DisableBridgeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/disable_bridge is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetBridgeList(const std::shared_ptr<kortex_driver::srv::GetBridgeList::Request> req, std::shared_ptr<kortex_driver::srv::GetBridgeList::Response> res)
{
	
	
	if (GetBridgeListHandler)
	{
		res = GetBridgeListHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_bridge_list is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetBridgeConfig(const std::shared_ptr<kortex_driver::srv::GetBridgeConfig::Request> req, std::shared_ptr<kortex_driver::srv::GetBridgeConfig::Response> res)
{
	
	
	if (GetBridgeConfigHandler)
	{
		res = GetBridgeConfigHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_bridge_config is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlayPreComputedJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlayPreComputedJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayPreComputedJointTrajectory::Response> res)
{
	
	
	if (PlayPreComputedJointTrajectoryHandler)
	{
		res = PlayPreComputedJointTrajectoryHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/play_pre_computed_joint_trajectory is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetProductConfiguration(const std::shared_ptr<kortex_driver::srv::GetProductConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetProductConfiguration::Response> res)
{
	
	
	if (GetProductConfigurationHandler)
	{
		res = GetProductConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_product_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateEndEffectorTypeConfiguration(const std::shared_ptr<kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Response> res)
{
	
	
	if (UpdateEndEffectorTypeConfigurationHandler)
	{
		res = UpdateEndEffectorTypeConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/update_end_effector_type_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::RestoreFactoryProductConfiguration(const std::shared_ptr<kortex_driver::srv::RestoreFactoryProductConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::RestoreFactoryProductConfiguration::Response> res)
{
	
	
	if (RestoreFactoryProductConfigurationHandler)
	{
		res = RestoreFactoryProductConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/restore_factory_product_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetTrajectoryErrorReport(const std::shared_ptr<kortex_driver::srv::GetTrajectoryErrorReport::Request> req, std::shared_ptr<kortex_driver::srv::GetTrajectoryErrorReport::Response> res)
{
	
	
	if (GetTrajectoryErrorReportHandler)
	{
		res = GetTrajectoryErrorReportHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_trajectory_error_report is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllJointsSpeedSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_all_joints_speed_soft_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetAllJointsSpeedSoftLimitationHandler)
	{
		res = GetAllJointsSpeedSoftLimitationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_all_joints_speed_soft_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllJointsTorqueSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_all_joints_torque_soft_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetAllJointsTorqueSoftLimitationHandler)
	{
		res = GetAllJointsTorqueSoftLimitationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_all_joints_torque_soft_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetTwistSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetTwistSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetTwistSoftLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_twist_soft_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetTwistSoftLimitationHandler)
	{
		res = GetTwistSoftLimitationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_twist_soft_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetWrenchSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetWrenchSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetWrenchSoftLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_wrench_soft_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetWrenchSoftLimitationHandler)
	{
		res = GetWrenchSoftLimitationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_wrench_soft_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetControllerConfigurationMode(const std::shared_ptr<kortex_driver::srv::SetControllerConfigurationMode::Request> req, std::shared_ptr<kortex_driver::srv::SetControllerConfigurationMode::Response> res)
{
	
	
	if (SetControllerConfigurationModeHandler)
	{
		res = SetControllerConfigurationModeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/set_controller_configuration_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetControllerConfigurationMode(const std::shared_ptr<kortex_driver::srv::GetControllerConfigurationMode::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerConfigurationMode::Response> res)
{
	
	
	if (GetControllerConfigurationModeHandler)
	{
		res = GetControllerConfigurationModeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_controller_configuration_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::StartTeaching(const std::shared_ptr<kortex_driver::srv::StartTeaching::Request> req, std::shared_ptr<kortex_driver::srv::StartTeaching::Response> res)
{
	
	
	if (StartTeachingHandler)
	{
		res = StartTeachingHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/start_teaching is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::StopTeaching(const std::shared_ptr<kortex_driver::srv::StopTeaching::Request> req, std::shared_ptr<kortex_driver::srv::StopTeaching::Response> res)
{
	
	
	if (StopTeachingHandler)
	{
		res = StopTeachingHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/stop_teaching is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::AddSequenceTasks(const std::shared_ptr<kortex_driver::srv::AddSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::AddSequenceTasks::Response> res)
{
	
	
	if (AddSequenceTasksHandler)
	{
		res = AddSequenceTasksHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/add_sequence_tasks is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateSequenceTask(const std::shared_ptr<kortex_driver::srv::UpdateSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::UpdateSequenceTask::Response> res)
{
	
	
	if (UpdateSequenceTaskHandler)
	{
		res = UpdateSequenceTaskHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/update_sequence_task is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SwapSequenceTasks(const std::shared_ptr<kortex_driver::srv::SwapSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::SwapSequenceTasks::Response> res)
{
	
	
	if (SwapSequenceTasksHandler)
	{
		res = SwapSequenceTasksHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/swap_sequence_tasks is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadSequenceTask(const std::shared_ptr<kortex_driver::srv::ReadSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::ReadSequenceTask::Response> res)
{
	
	
	if (ReadSequenceTaskHandler)
	{
		res = ReadSequenceTaskHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_sequence_task is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllSequenceTasks(const std::shared_ptr<kortex_driver::srv::ReadAllSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllSequenceTasks::Response> res)
{
	
	
	if (ReadAllSequenceTasksHandler)
	{
		res = ReadAllSequenceTasksHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/read_all_sequence_tasks is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteSequenceTask(const std::shared_ptr<kortex_driver::srv::DeleteSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::DeleteSequenceTask::Response> res)
{
	
	
	if (DeleteSequenceTaskHandler)
	{
		res = DeleteSequenceTaskHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/delete_sequence_task is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteAllSequenceTasks(const std::shared_ptr<kortex_driver::srv::DeleteAllSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::DeleteAllSequenceTasks::Response> res)
{
	
	
	if (DeleteAllSequenceTasksHandler)
	{
		res = DeleteAllSequenceTasksHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/delete_all_sequence_tasks is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::TakeSnapshot(const std::shared_ptr<kortex_driver::srv::TakeSnapshot::Request> req, std::shared_ptr<kortex_driver::srv::TakeSnapshot::Response> res)
{
	
	
	if (TakeSnapshotHandler)
	{
		res = TakeSnapshotHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/take_snapshot is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetFirmwareBundleVersions(const std::shared_ptr<kortex_driver::srv::GetFirmwareBundleVersions::Request> req, std::shared_ptr<kortex_driver::srv::GetFirmwareBundleVersions::Response> res)
{
	
	
	if (GetFirmwareBundleVersionsHandler)
	{
		res = GetFirmwareBundleVersionsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_firmware_bundle_versions is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ExecuteWaypointTrajectory(const std::shared_ptr<kortex_driver::srv::ExecuteWaypointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteWaypointTrajectory::Response> res)
{
	
	
	if (ExecuteWaypointTrajectoryHandler)
	{
		res = ExecuteWaypointTrajectoryHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/execute_waypoint_trajectory is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::MoveSequenceTask(const std::shared_ptr<kortex_driver::srv::MoveSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::MoveSequenceTask::Response> res)
{
	
	
	if (MoveSequenceTaskHandler)
	{
		res = MoveSequenceTaskHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/move_sequence_task is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DuplicateMapping(const std::shared_ptr<kortex_driver::srv::DuplicateMapping::Request> req, std::shared_ptr<kortex_driver::srv::DuplicateMapping::Response> res)
{
	
	
	if (DuplicateMappingHandler)
	{
		res = DuplicateMappingHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/duplicate_mapping is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DuplicateMap(const std::shared_ptr<kortex_driver::srv::DuplicateMap::Request> req, std::shared_ptr<kortex_driver::srv::DuplicateMap::Response> res)
{
	
	
	if (DuplicateMapHandler)
	{
		res = DuplicateMapHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/duplicate_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetControllerConfiguration(const std::shared_ptr<kortex_driver::srv::SetControllerConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetControllerConfiguration::Response> res)
{
	
	
	if (SetControllerConfigurationHandler)
	{
		res = SetControllerConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/set_controller_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetControllerConfiguration(const std::shared_ptr<kortex_driver::srv::GetControllerConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerConfiguration::Response> res)
{
	
	
	if (GetControllerConfigurationHandler)
	{
		res = GetControllerConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_controller_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllControllerConfigurations(const std::shared_ptr<kortex_driver::srv::GetAllControllerConfigurations::Request> req, std::shared_ptr<kortex_driver::srv::GetAllControllerConfigurations::Response> res)
{
	
	
	if (GetAllControllerConfigurationsHandler)
	{
		res = GetAllControllerConfigurationsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/get_all_controller_configurations is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ComputeForwardKinematics(const std::shared_ptr<kortex_driver::srv::ComputeForwardKinematics::Request> req, std::shared_ptr<kortex_driver::srv::ComputeForwardKinematics::Response> res)
{
	
	
	if (ComputeForwardKinematicsHandler)
	{
		res = ComputeForwardKinematicsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/compute_forward_kinematics is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ComputeInverseKinematics(const std::shared_ptr<kortex_driver::srv::ComputeInverseKinematics::Request> req, std::shared_ptr<kortex_driver::srv::ComputeInverseKinematics::Response> res)
{
	
	
	if (ComputeInverseKinematicsHandler)
	{
		res = ComputeInverseKinematicsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/compute_inverse_kinematics is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ValidateWaypointList(const std::shared_ptr<kortex_driver::srv::ValidateWaypointList::Request> req, std::shared_ptr<kortex_driver::srv::ValidateWaypointList::Response> res)
{
	
	
	if (ValidateWaypointListHandler)
	{
		res = ValidateWaypointListHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for base/validate_waypoint_list is not implemented, so the service calls will return the default response.");
	}
	return true;
}
