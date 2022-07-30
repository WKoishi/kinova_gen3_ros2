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
#include "kortex_driver/generated/robot/base_services.h"

BaseRobotServices::BaseRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::Base::BaseClient* base, uint32_t device_id, uint32_t timeout_ms): 
	IBaseServices(node_handle),
	m_base(base),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_node_handle.advertise<kortex_driver::msg::KortexError>("kortex_error", 1000);
	m_pub_ConfigurationChangeTopic = m_node_handle.advertise<kortex_driver::msg::ConfigurationChangeNotification>("configuration_change_topic", 1000);
	m_is_activated_ConfigurationChangeTopic = false;
	m_pub_MappingInfoTopic = m_node_handle.advertise<kortex_driver::msg::MappingInfoNotification>("mapping_info_topic", 1000);
	m_is_activated_MappingInfoTopic = false;
	m_pub_ControlModeTopic = m_node_handle.advertise<kortex_driver::msg::BaseControlModeNotification>("control_mode_topic", 1000);
	m_is_activated_ControlModeTopic = false;
	m_pub_OperatingModeTopic = m_node_handle.advertise<kortex_driver::msg::OperatingModeNotification>("operating_mode_topic", 1000);
	m_is_activated_OperatingModeTopic = false;
	m_pub_SequenceInfoTopic = m_node_handle.advertise<kortex_driver::msg::SequenceInfoNotification>("sequence_info_topic", 1000);
	m_is_activated_SequenceInfoTopic = false;
	m_pub_ProtectionZoneTopic = m_node_handle.advertise<kortex_driver::msg::ProtectionZoneNotification>("protection_zone_topic", 1000);
	m_is_activated_ProtectionZoneTopic = false;
	m_pub_UserTopic = m_node_handle.advertise<kortex_driver::msg::UserNotification>("user_topic", 1000);
	m_is_activated_UserTopic = false;
	m_pub_ControllerTopic = m_node_handle.advertise<kortex_driver::msg::ControllerNotification>("controller_topic", 1000);
	m_is_activated_ControllerTopic = false;
	m_pub_ActionTopic = m_node_handle.advertise<kortex_driver::msg::ActionNotification>("action_topic", 1000);
	m_is_activated_ActionTopic = false;
	m_pub_RobotEventTopic = m_node_handle.advertise<kortex_driver::msg::RobotEventNotification>("robot_event_topic", 1000);
	m_is_activated_RobotEventTopic = false;
	m_pub_ServoingModeTopic = m_node_handle.advertise<kortex_driver::msg::ServoingModeNotification>("servoing_mode_topic", 1000);
	m_is_activated_ServoingModeTopic = false;
	m_pub_FactoryTopic = m_node_handle.advertise<kortex_driver::msg::FactoryNotification>("factory_topic", 1000);
	m_is_activated_FactoryTopic = false;
	m_pub_NetworkTopic = m_node_handle.advertise<kortex_driver::msg::NetworkNotification>("network_topic", 1000);
	m_is_activated_NetworkTopic = false;
	m_pub_ArmStateTopic = m_node_handle.advertise<kortex_driver::msg::ArmStateNotification>("arm_state_topic", 1000);
	m_is_activated_ArmStateTopic = false;

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("base/set_device_id", std::bind(&BaseRobotServices::SetDeviceID, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("base/set_api_options", std::bind(&BaseRobotServices::SetApiOptions, this, std::placeholders::_1, std::placeholders::_2));

	m_serviceCreateUserProfile = m_node_handle->create_service<kortex_driver::srv::CreateUserProfile>("base/create_user_profile", std::bind(&BaseRobotServices::CreateUserProfile, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateUserProfile = m_node_handle->create_service<kortex_driver::srv::UpdateUserProfile>("base/update_user_profile", std::bind(&BaseRobotServices::UpdateUserProfile, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadUserProfile = m_node_handle->create_service<kortex_driver::srv::ReadUserProfile>("base/read_user_profile", std::bind(&BaseRobotServices::ReadUserProfile, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteUserProfile = m_node_handle->create_service<kortex_driver::srv::DeleteUserProfile>("base/delete_user_profile", std::bind(&BaseRobotServices::DeleteUserProfile, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllUserProfiles = m_node_handle->create_service<kortex_driver::srv::ReadAllUserProfiles>("base/read_all_user_profiles", std::bind(&BaseRobotServices::ReadAllUserProfiles, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllUsers = m_node_handle->create_service<kortex_driver::srv::ReadAllUsers>("base/read_all_users", std::bind(&BaseRobotServices::ReadAllUsers, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceChangePassword = m_node_handle->create_service<kortex_driver::srv::ChangePassword>("base/change_password", std::bind(&BaseRobotServices::ChangePassword, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceCreateSequence = m_node_handle->create_service<kortex_driver::srv::CreateSequence>("base/create_sequence", std::bind(&BaseRobotServices::CreateSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateSequence = m_node_handle->create_service<kortex_driver::srv::UpdateSequence>("base/update_sequence", std::bind(&BaseRobotServices::UpdateSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadSequence = m_node_handle->create_service<kortex_driver::srv::ReadSequence>("base/read_sequence", std::bind(&BaseRobotServices::ReadSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteSequence = m_node_handle->create_service<kortex_driver::srv::DeleteSequence>("base/delete_sequence", std::bind(&BaseRobotServices::DeleteSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllSequences = m_node_handle->create_service<kortex_driver::srv::ReadAllSequences>("base/read_all_sequences", std::bind(&BaseRobotServices::ReadAllSequences, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlaySequence = m_node_handle->create_service<kortex_driver::srv::PlaySequence>("base/play_sequence", std::bind(&BaseRobotServices::PlaySequence, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayAdvancedSequence = m_node_handle->create_service<kortex_driver::srv::PlayAdvancedSequence>("base/play_advanced_sequence", std::bind(&BaseRobotServices::PlayAdvancedSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStopSequence = m_node_handle->create_service<kortex_driver::srv::StopSequence>("base/stop_sequence", std::bind(&BaseRobotServices::StopSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePauseSequence = m_node_handle->create_service<kortex_driver::srv::PauseSequence>("base/pause_sequence", std::bind(&BaseRobotServices::PauseSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResumeSequence = m_node_handle->create_service<kortex_driver::srv::ResumeSequence>("base/resume_sequence", std::bind(&BaseRobotServices::ResumeSequence, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceCreateProtectionZone = m_node_handle->create_service<kortex_driver::srv::CreateProtectionZone>("base/create_protection_zone", std::bind(&BaseRobotServices::CreateProtectionZone, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateProtectionZone = m_node_handle->create_service<kortex_driver::srv::UpdateProtectionZone>("base/update_protection_zone", std::bind(&BaseRobotServices::UpdateProtectionZone, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadProtectionZone = m_node_handle->create_service<kortex_driver::srv::ReadProtectionZone>("base/read_protection_zone", std::bind(&BaseRobotServices::ReadProtectionZone, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteProtectionZone = m_node_handle->create_service<kortex_driver::srv::DeleteProtectionZone>("base/delete_protection_zone", std::bind(&BaseRobotServices::DeleteProtectionZone, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllProtectionZones = m_node_handle->create_service<kortex_driver::srv::ReadAllProtectionZones>("base/read_all_protection_zones", std::bind(&BaseRobotServices::ReadAllProtectionZones, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceCreateMapping = m_node_handle->create_service<kortex_driver::srv::CreateMapping>("base/create_mapping", std::bind(&BaseRobotServices::CreateMapping, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadMapping = m_node_handle->create_service<kortex_driver::srv::ReadMapping>("base/read_mapping", std::bind(&BaseRobotServices::ReadMapping, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateMapping = m_node_handle->create_service<kortex_driver::srv::UpdateMapping>("base/update_mapping", std::bind(&BaseRobotServices::UpdateMapping, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteMapping = m_node_handle->create_service<kortex_driver::srv::DeleteMapping>("base/delete_mapping", std::bind(&BaseRobotServices::DeleteMapping, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllMappings = m_node_handle->create_service<kortex_driver::srv::ReadAllMappings>("base/read_all_mappings", std::bind(&BaseRobotServices::ReadAllMappings, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceCreateMap = m_node_handle->create_service<kortex_driver::srv::CreateMap>("base/create_map", std::bind(&BaseRobotServices::CreateMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadMap = m_node_handle->create_service<kortex_driver::srv::ReadMap>("base/read_map", std::bind(&BaseRobotServices::ReadMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateMap = m_node_handle->create_service<kortex_driver::srv::UpdateMap>("base/update_map", std::bind(&BaseRobotServices::UpdateMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteMap = m_node_handle->create_service<kortex_driver::srv::DeleteMap>("base/delete_map", std::bind(&BaseRobotServices::DeleteMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllMaps = m_node_handle->create_service<kortex_driver::srv::ReadAllMaps>("base/read_all_maps", std::bind(&BaseRobotServices::ReadAllMaps, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceActivateMap = m_node_handle->create_service<kortex_driver::srv::ActivateMap>("base/activate_map", std::bind(&BaseRobotServices::ActivateMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceCreateAction = m_node_handle->create_service<kortex_driver::srv::CreateAction>("base/create_action", std::bind(&BaseRobotServices::CreateAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAction = m_node_handle->create_service<kortex_driver::srv::ReadAction>("base/read_action", std::bind(&BaseRobotServices::ReadAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllActions = m_node_handle->create_service<kortex_driver::srv::ReadAllActions>("base/read_all_actions", std::bind(&BaseRobotServices::ReadAllActions, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteAction = m_node_handle->create_service<kortex_driver::srv::DeleteAction>("base/delete_action", std::bind(&BaseRobotServices::DeleteAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateAction = m_node_handle->create_service<kortex_driver::srv::UpdateAction>("base/update_action", std::bind(&BaseRobotServices::UpdateAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceExecuteActionFromReference = m_node_handle->create_service<kortex_driver::srv::ExecuteActionFromReference>("base/execute_action_from_reference", std::bind(&BaseRobotServices::ExecuteActionFromReference, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceExecuteAction = m_node_handle->create_service<kortex_driver::srv::ExecuteAction>("base/execute_action", std::bind(&BaseRobotServices::ExecuteAction, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePauseAction = m_node_handle->create_service<kortex_driver::srv::PauseAction>("base/pause_action", std::bind(&BaseRobotServices::PauseAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStopAction = m_node_handle->create_service<kortex_driver::srv::StopAction>("base/stop_action", std::bind(&BaseRobotServices::StopAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResumeAction = m_node_handle->create_service<kortex_driver::srv::ResumeAction>("base/resume_action", std::bind(&BaseRobotServices::ResumeAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetIPv4Configuration = m_node_handle->create_service<kortex_driver::srv::GetIPv4Configuration>("base/get_i_pv4_configuration", std::bind(&BaseRobotServices::GetIPv4Configuration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetIPv4Configuration = m_node_handle->create_service<kortex_driver::srv::SetIPv4Configuration>("base/set_i_pv4_configuration", std::bind(&BaseRobotServices::SetIPv4Configuration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetCommunicationInterfaceEnable = m_node_handle->create_service<kortex_driver::srv::SetCommunicationInterfaceEnable>("base/set_communication_interface_enable", std::bind(&BaseRobotServices::SetCommunicationInterfaceEnable, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceIsCommunicationInterfaceEnable = m_node_handle->create_service<kortex_driver::srv::IsCommunicationInterfaceEnable>("base/is_communication_interface_enable", std::bind(&BaseRobotServices::IsCommunicationInterfaceEnable, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAvailableWifi = m_node_handle->create_service<kortex_driver::srv::GetAvailableWifi>("base/get_available_wifi", std::bind(&BaseRobotServices::GetAvailableWifi, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetWifiInformation = m_node_handle->create_service<kortex_driver::srv::GetWifiInformation>("base/get_wifi_information", std::bind(&BaseRobotServices::GetWifiInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceAddWifiConfiguration = m_node_handle->create_service<kortex_driver::srv::AddWifiConfiguration>("base/add_wifi_configuration", std::bind(&BaseRobotServices::AddWifiConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteWifiConfiguration = m_node_handle->create_service<kortex_driver::srv::DeleteWifiConfiguration>("base/delete_wifi_configuration", std::bind(&BaseRobotServices::DeleteWifiConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllConfiguredWifis = m_node_handle->create_service<kortex_driver::srv::GetAllConfiguredWifis>("base/get_all_configured_wifis", std::bind(&BaseRobotServices::GetAllConfiguredWifis, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceConnectWifi = m_node_handle->create_service<kortex_driver::srv::ConnectWifi>("base/connect_wifi", std::bind(&BaseRobotServices::ConnectWifi, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDisconnectWifi = m_node_handle->create_service<kortex_driver::srv::DisconnectWifi>("base/disconnect_wifi", std::bind(&BaseRobotServices::DisconnectWifi, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetConnectedWifiInformation = m_node_handle->create_service<kortex_driver::srv::GetConnectedWifiInformation>("base/get_connected_wifi_information", std::bind(&BaseRobotServices::GetConnectedWifiInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_Unsubscribe = m_node_handle->create_service<kortex_driver::srv::BaseUnsubscribe>("base/unsubscribe", std::bind(&BaseRobotServices::Base_Unsubscribe, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationConfigurationChangeTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationConfigurationChangeTopic>("base/activate_publishing_of_configuration_change_topic", std::bind(&BaseRobotServices::OnNotificationConfigurationChangeTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationMappingInfoTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationMappingInfoTopic>("base/activate_publishing_of_mapping_info_topic", std::bind(&BaseRobotServices::OnNotificationMappingInfoTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_OnNotificationControlModeTopic = m_node_handle->create_service<kortex_driver::srv::BaseOnNotificationControlModeTopic>("base/activate_publishing_of_control_mode_topic", std::bind(&BaseRobotServices::Base_OnNotificationControlModeTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationOperatingModeTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationOperatingModeTopic>("base/activate_publishing_of_operating_mode_topic", std::bind(&BaseRobotServices::OnNotificationOperatingModeTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationSequenceInfoTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationSequenceInfoTopic>("base/activate_publishing_of_sequence_info_topic", std::bind(&BaseRobotServices::OnNotificationSequenceInfoTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationProtectionZoneTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationProtectionZoneTopic>("base/activate_publishing_of_protection_zone_topic", std::bind(&BaseRobotServices::OnNotificationProtectionZoneTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationUserTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationUserTopic>("base/activate_publishing_of_user_topic", std::bind(&BaseRobotServices::OnNotificationUserTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationControllerTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationControllerTopic>("base/activate_publishing_of_controller_topic", std::bind(&BaseRobotServices::OnNotificationControllerTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationActionTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationActionTopic>("base/activate_publishing_of_action_topic", std::bind(&BaseRobotServices::OnNotificationActionTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationRobotEventTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationRobotEventTopic>("base/activate_publishing_of_robot_event_topic", std::bind(&BaseRobotServices::OnNotificationRobotEventTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayCartesianTrajectory = m_node_handle->create_service<kortex_driver::srv::PlayCartesianTrajectory>("base/play_cartesian_trajectory", std::bind(&BaseRobotServices::PlayCartesianTrajectory, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayCartesianTrajectoryPosition = m_node_handle->create_service<kortex_driver::srv::PlayCartesianTrajectoryPosition>("base/play_cartesian_trajectory_position", std::bind(&BaseRobotServices::PlayCartesianTrajectoryPosition, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayCartesianTrajectoryOrientation = m_node_handle->create_service<kortex_driver::srv::PlayCartesianTrajectoryOrientation>("base/play_cartesian_trajectory_orientation", std::bind(&BaseRobotServices::PlayCartesianTrajectoryOrientation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStop = m_node_handle->create_service<kortex_driver::srv::Stop>("base/stop", std::bind(&BaseRobotServices::Stop, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetMeasuredCartesianPose = m_node_handle->create_service<kortex_driver::srv::GetMeasuredCartesianPose>("base/get_measured_cartesian_pose", std::bind(&BaseRobotServices::GetMeasuredCartesianPose, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendWrenchCommand = m_node_handle->create_service<kortex_driver::srv::SendWrenchCommand>("base/send_wrench_command", std::bind(&BaseRobotServices::SendWrenchCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendWrenchJoystickCommand = m_node_handle->create_service<kortex_driver::srv::SendWrenchJoystickCommand>("base/send_wrench_joystick_command", std::bind(&BaseRobotServices::SendWrenchJoystickCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendTwistJoystickCommand = m_node_handle->create_service<kortex_driver::srv::SendTwistJoystickCommand>("base/send_twist_joystick_command", std::bind(&BaseRobotServices::SendTwistJoystickCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendTwistCommand = m_node_handle->create_service<kortex_driver::srv::SendTwistCommand>("base/send_twist_command", std::bind(&BaseRobotServices::SendTwistCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayJointTrajectory = m_node_handle->create_service<kortex_driver::srv::PlayJointTrajectory>("base/play_joint_trajectory", std::bind(&BaseRobotServices::PlayJointTrajectory, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlaySelectedJointTrajectory = m_node_handle->create_service<kortex_driver::srv::PlaySelectedJointTrajectory>("base/play_selected_joint_trajectory", std::bind(&BaseRobotServices::PlaySelectedJointTrajectory, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetMeasuredJointAngles = m_node_handle->create_service<kortex_driver::srv::GetMeasuredJointAngles>("base/get_measured_joint_angles", std::bind(&BaseRobotServices::GetMeasuredJointAngles, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendJointSpeedsCommand = m_node_handle->create_service<kortex_driver::srv::SendJointSpeedsCommand>("base/send_joint_speeds_command", std::bind(&BaseRobotServices::SendJointSpeedsCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendSelectedJointSpeedCommand = m_node_handle->create_service<kortex_driver::srv::SendSelectedJointSpeedCommand>("base/send_selected_joint_speed_command", std::bind(&BaseRobotServices::SendSelectedJointSpeedCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendGripperCommand = m_node_handle->create_service<kortex_driver::srv::SendGripperCommand>("base/send_gripper_command", std::bind(&BaseRobotServices::SendGripperCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetMeasuredGripperMovement = m_node_handle->create_service<kortex_driver::srv::GetMeasuredGripperMovement>("base/get_measured_gripper_movement", std::bind(&BaseRobotServices::GetMeasuredGripperMovement, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetAdmittance = m_node_handle->create_service<kortex_driver::srv::SetAdmittance>("base/set_admittance", std::bind(&BaseRobotServices::SetAdmittance, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetOperatingMode = m_node_handle->create_service<kortex_driver::srv::SetOperatingMode>("base/set_operating_mode", std::bind(&BaseRobotServices::SetOperatingMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceApplyEmergencyStop = m_node_handle->create_service<kortex_driver::srv::ApplyEmergencyStop>("base/apply_emergency_stop", std::bind(&BaseRobotServices::ApplyEmergencyStop, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_ClearFaults = m_node_handle->create_service<kortex_driver::srv::BaseClearFaults>("base/clear_faults", std::bind(&BaseRobotServices::Base_ClearFaults, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_GetControlMode = m_node_handle->create_service<kortex_driver::srv::BaseGetControlMode>("base/get_control_mode", std::bind(&BaseRobotServices::Base_GetControlMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetOperatingMode = m_node_handle->create_service<kortex_driver::srv::GetOperatingMode>("base/get_operating_mode", std::bind(&BaseRobotServices::GetOperatingMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetServoingMode = m_node_handle->create_service<kortex_driver::srv::SetServoingMode>("base/set_servoing_mode", std::bind(&BaseRobotServices::SetServoingMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetServoingMode = m_node_handle->create_service<kortex_driver::srv::GetServoingMode>("base/get_servoing_mode", std::bind(&BaseRobotServices::GetServoingMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationServoingModeTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationServoingModeTopic>("base/activate_publishing_of_servoing_mode_topic", std::bind(&BaseRobotServices::OnNotificationServoingModeTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceRestoreFactorySettings = m_node_handle->create_service<kortex_driver::srv::RestoreFactorySettings>("base/restore_factory_settings", std::bind(&BaseRobotServices::RestoreFactorySettings, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationFactoryTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationFactoryTopic>("base/activate_publishing_of_factory_topic", std::bind(&BaseRobotServices::OnNotificationFactoryTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllConnectedControllers = m_node_handle->create_service<kortex_driver::srv::GetAllConnectedControllers>("base/get_all_connected_controllers", std::bind(&BaseRobotServices::GetAllConnectedControllers, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetControllerState = m_node_handle->create_service<kortex_driver::srv::GetControllerState>("base/get_controller_state", std::bind(&BaseRobotServices::GetControllerState, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetActuatorCount = m_node_handle->create_service<kortex_driver::srv::GetActuatorCount>("base/get_actuator_count", std::bind(&BaseRobotServices::GetActuatorCount, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStartWifiScan = m_node_handle->create_service<kortex_driver::srv::StartWifiScan>("base/start_wifi_scan", std::bind(&BaseRobotServices::StartWifiScan, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetConfiguredWifi = m_node_handle->create_service<kortex_driver::srv::GetConfiguredWifi>("base/get_configured_wifi", std::bind(&BaseRobotServices::GetConfiguredWifi, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationNetworkTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationNetworkTopic>("base/activate_publishing_of_network_topic", std::bind(&BaseRobotServices::OnNotificationNetworkTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetArmState = m_node_handle->create_service<kortex_driver::srv::GetArmState>("base/get_arm_state", std::bind(&BaseRobotServices::GetArmState, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationArmStateTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationArmStateTopic>("base/activate_publishing_of_arm_state_topic", std::bind(&BaseRobotServices::OnNotificationArmStateTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetIPv4Information = m_node_handle->create_service<kortex_driver::srv::GetIPv4Information>("base/get_i_pv4_information", std::bind(&BaseRobotServices::GetIPv4Information, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetWifiCountryCode = m_node_handle->create_service<kortex_driver::srv::SetWifiCountryCode>("base/set_wifi_country_code", std::bind(&BaseRobotServices::SetWifiCountryCode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetWifiCountryCode = m_node_handle->create_service<kortex_driver::srv::GetWifiCountryCode>("base/get_wifi_country_code", std::bind(&BaseRobotServices::GetWifiCountryCode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_SetCapSenseConfig = m_node_handle->create_service<kortex_driver::srv::BaseSetCapSenseConfig>("base/set_cap_sense_config", std::bind(&BaseRobotServices::Base_SetCapSenseConfig, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceBase_GetCapSenseConfig = m_node_handle->create_service<kortex_driver::srv::BaseGetCapSenseConfig>("base/get_cap_sense_config", std::bind(&BaseRobotServices::Base_GetCapSenseConfig, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllJointsSpeedHardLimitation = m_node_handle->create_service<kortex_driver::srv::GetAllJointsSpeedHardLimitation>("base/get_all_joints_speed_hard_limitation", std::bind(&BaseRobotServices::GetAllJointsSpeedHardLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllJointsTorqueHardLimitation = m_node_handle->create_service<kortex_driver::srv::GetAllJointsTorqueHardLimitation>("base/get_all_joints_torque_hard_limitation", std::bind(&BaseRobotServices::GetAllJointsTorqueHardLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetTwistHardLimitation = m_node_handle->create_service<kortex_driver::srv::GetTwistHardLimitation>("base/get_twist_hard_limitation", std::bind(&BaseRobotServices::GetTwistHardLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetWrenchHardLimitation = m_node_handle->create_service<kortex_driver::srv::GetWrenchHardLimitation>("base/get_wrench_hard_limitation", std::bind(&BaseRobotServices::GetWrenchHardLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendJointSpeedsJoystickCommand = m_node_handle->create_service<kortex_driver::srv::SendJointSpeedsJoystickCommand>("base/send_joint_speeds_joystick_command", std::bind(&BaseRobotServices::SendJointSpeedsJoystickCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSendSelectedJointSpeedJoystickCommand = m_node_handle->create_service<kortex_driver::srv::SendSelectedJointSpeedJoystickCommand>("base/send_selected_joint_speed_joystick_command", std::bind(&BaseRobotServices::SendSelectedJointSpeedJoystickCommand, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceEnableBridge = m_node_handle->create_service<kortex_driver::srv::EnableBridge>("base/enable_bridge", std::bind(&BaseRobotServices::EnableBridge, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDisableBridge = m_node_handle->create_service<kortex_driver::srv::DisableBridge>("base/disable_bridge", std::bind(&BaseRobotServices::DisableBridge, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetBridgeList = m_node_handle->create_service<kortex_driver::srv::GetBridgeList>("base/get_bridge_list", std::bind(&BaseRobotServices::GetBridgeList, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetBridgeConfig = m_node_handle->create_service<kortex_driver::srv::GetBridgeConfig>("base/get_bridge_config", std::bind(&BaseRobotServices::GetBridgeConfig, this, std::placeholders::_1, std::placeholders::_2));
	m_servicePlayPreComputedJointTrajectory = m_node_handle->create_service<kortex_driver::srv::PlayPreComputedJointTrajectory>("base/play_pre_computed_joint_trajectory", std::bind(&BaseRobotServices::PlayPreComputedJointTrajectory, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetProductConfiguration = m_node_handle->create_service<kortex_driver::srv::GetProductConfiguration>("base/get_product_configuration", std::bind(&BaseRobotServices::GetProductConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateEndEffectorTypeConfiguration = m_node_handle->create_service<kortex_driver::srv::UpdateEndEffectorTypeConfiguration>("base/update_end_effector_type_configuration", std::bind(&BaseRobotServices::UpdateEndEffectorTypeConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceRestoreFactoryProductConfiguration = m_node_handle->create_service<kortex_driver::srv::RestoreFactoryProductConfiguration>("base/restore_factory_product_configuration", std::bind(&BaseRobotServices::RestoreFactoryProductConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetTrajectoryErrorReport = m_node_handle->create_service<kortex_driver::srv::GetTrajectoryErrorReport>("base/get_trajectory_error_report", std::bind(&BaseRobotServices::GetTrajectoryErrorReport, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllJointsSpeedSoftLimitation = m_node_handle->create_service<kortex_driver::srv::GetAllJointsSpeedSoftLimitation>("base/get_all_joints_speed_soft_limitation", std::bind(&BaseRobotServices::GetAllJointsSpeedSoftLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllJointsTorqueSoftLimitation = m_node_handle->create_service<kortex_driver::srv::GetAllJointsTorqueSoftLimitation>("base/get_all_joints_torque_soft_limitation", std::bind(&BaseRobotServices::GetAllJointsTorqueSoftLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetTwistSoftLimitation = m_node_handle->create_service<kortex_driver::srv::GetTwistSoftLimitation>("base/get_twist_soft_limitation", std::bind(&BaseRobotServices::GetTwistSoftLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetWrenchSoftLimitation = m_node_handle->create_service<kortex_driver::srv::GetWrenchSoftLimitation>("base/get_wrench_soft_limitation", std::bind(&BaseRobotServices::GetWrenchSoftLimitation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetControllerConfigurationMode = m_node_handle->create_service<kortex_driver::srv::SetControllerConfigurationMode>("base/set_controller_configuration_mode", std::bind(&BaseRobotServices::SetControllerConfigurationMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetControllerConfigurationMode = m_node_handle->create_service<kortex_driver::srv::GetControllerConfigurationMode>("base/get_controller_configuration_mode", std::bind(&BaseRobotServices::GetControllerConfigurationMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStartTeaching = m_node_handle->create_service<kortex_driver::srv::StartTeaching>("base/start_teaching", std::bind(&BaseRobotServices::StartTeaching, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStopTeaching = m_node_handle->create_service<kortex_driver::srv::StopTeaching>("base/stop_teaching", std::bind(&BaseRobotServices::StopTeaching, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceAddSequenceTasks = m_node_handle->create_service<kortex_driver::srv::AddSequenceTasks>("base/add_sequence_tasks", std::bind(&BaseRobotServices::AddSequenceTasks, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceUpdateSequenceTask = m_node_handle->create_service<kortex_driver::srv::UpdateSequenceTask>("base/update_sequence_task", std::bind(&BaseRobotServices::UpdateSequenceTask, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSwapSequenceTasks = m_node_handle->create_service<kortex_driver::srv::SwapSequenceTasks>("base/swap_sequence_tasks", std::bind(&BaseRobotServices::SwapSequenceTasks, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadSequenceTask = m_node_handle->create_service<kortex_driver::srv::ReadSequenceTask>("base/read_sequence_task", std::bind(&BaseRobotServices::ReadSequenceTask, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceReadAllSequenceTasks = m_node_handle->create_service<kortex_driver::srv::ReadAllSequenceTasks>("base/read_all_sequence_tasks", std::bind(&BaseRobotServices::ReadAllSequenceTasks, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteSequenceTask = m_node_handle->create_service<kortex_driver::srv::DeleteSequenceTask>("base/delete_sequence_task", std::bind(&BaseRobotServices::DeleteSequenceTask, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeleteAllSequenceTasks = m_node_handle->create_service<kortex_driver::srv::DeleteAllSequenceTasks>("base/delete_all_sequence_tasks", std::bind(&BaseRobotServices::DeleteAllSequenceTasks, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceTakeSnapshot = m_node_handle->create_service<kortex_driver::srv::TakeSnapshot>("base/take_snapshot", std::bind(&BaseRobotServices::TakeSnapshot, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetFirmwareBundleVersions = m_node_handle->create_service<kortex_driver::srv::GetFirmwareBundleVersions>("base/get_firmware_bundle_versions", std::bind(&BaseRobotServices::GetFirmwareBundleVersions, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceExecuteWaypointTrajectory = m_node_handle->create_service<kortex_driver::srv::ExecuteWaypointTrajectory>("base/execute_waypoint_trajectory", std::bind(&BaseRobotServices::ExecuteWaypointTrajectory, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceMoveSequenceTask = m_node_handle->create_service<kortex_driver::srv::MoveSequenceTask>("base/move_sequence_task", std::bind(&BaseRobotServices::MoveSequenceTask, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDuplicateMapping = m_node_handle->create_service<kortex_driver::srv::DuplicateMapping>("base/duplicate_mapping", std::bind(&BaseRobotServices::DuplicateMapping, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDuplicateMap = m_node_handle->create_service<kortex_driver::srv::DuplicateMap>("base/duplicate_map", std::bind(&BaseRobotServices::DuplicateMap, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetControllerConfiguration = m_node_handle->create_service<kortex_driver::srv::SetControllerConfiguration>("base/set_controller_configuration", std::bind(&BaseRobotServices::SetControllerConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetControllerConfiguration = m_node_handle->create_service<kortex_driver::srv::GetControllerConfiguration>("base/get_controller_configuration", std::bind(&BaseRobotServices::GetControllerConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllControllerConfigurations = m_node_handle->create_service<kortex_driver::srv::GetAllControllerConfigurations>("base/get_all_controller_configurations", std::bind(&BaseRobotServices::GetAllControllerConfigurations, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceComputeForwardKinematics = m_node_handle->create_service<kortex_driver::srv::ComputeForwardKinematics>("base/compute_forward_kinematics", std::bind(&BaseRobotServices::ComputeForwardKinematics, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceComputeInverseKinematics = m_node_handle->create_service<kortex_driver::srv::ComputeInverseKinematics>("base/compute_inverse_kinematics", std::bind(&BaseRobotServices::ComputeInverseKinematics, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceValidateWaypointList = m_node_handle->create_service<kortex_driver::srv::ValidateWaypointList>("base/validate_waypoint_list", std::bind(&BaseRobotServices::ValidateWaypointList, this, std::placeholders::_1, std::placeholders::_2));
}

bool BaseRobotServices::SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res)
{
	m_current_device_id = req->device_id;

	return true;
}

bool BaseRobotServices::SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res)
{
	m_api_options.timeout_ms = req->input.timeout_ms;

	return true;
}


bool BaseRobotServices::CreateUserProfile(const std::shared_ptr<kortex_driver::srv::CreateUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::CreateUserProfile::Response> res)
{
	
	Kinova::Api::Base::FullUserProfile input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::UserProfileHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->CreateUserProfile(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::UpdateUserProfile(const std::shared_ptr<kortex_driver::srv::UpdateUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::UpdateUserProfile::Response> res)
{
	
	Kinova::Api::Base::UserProfile input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->UpdateUserProfile(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadUserProfile(const std::shared_ptr<kortex_driver::srv::ReadUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::ReadUserProfile::Response> res)
{
	
	Kinova::Api::Common::UserProfileHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::UserProfile output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadUserProfile(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteUserProfile(const std::shared_ptr<kortex_driver::srv::DeleteUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::DeleteUserProfile::Response> res)
{
	
	Kinova::Api::Common::UserProfileHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->DeleteUserProfile(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAllUserProfiles(const std::shared_ptr<kortex_driver::srv::ReadAllUserProfiles::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllUserProfiles::Response> res)
{
	
	Kinova::Api::Base::UserProfileList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllUserProfiles(m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAllUsers(const std::shared_ptr<kortex_driver::srv::ReadAllUsers::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllUsers::Response> res)
{
	
	Kinova::Api::Base::UserList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllUsers(m_current_device_id, m_api_options);
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

bool BaseRobotServices::ChangePassword(const std::shared_ptr<kortex_driver::srv::ChangePassword::Request> req, std::shared_ptr<kortex_driver::srv::ChangePassword::Response> res)
{
	
	Kinova::Api::Base::PasswordChange input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->ChangePassword(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::CreateSequence(const std::shared_ptr<kortex_driver::srv::CreateSequence::Request> req, std::shared_ptr<kortex_driver::srv::CreateSequence::Response> res)
{
	
	Kinova::Api::Base::Sequence input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::SequenceHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->CreateSequence(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::UpdateSequence(const std::shared_ptr<kortex_driver::srv::UpdateSequence::Request> req, std::shared_ptr<kortex_driver::srv::UpdateSequence::Response> res)
{
	
	Kinova::Api::Base::Sequence input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->UpdateSequence(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadSequence(const std::shared_ptr<kortex_driver::srv::ReadSequence::Request> req, std::shared_ptr<kortex_driver::srv::ReadSequence::Response> res)
{
	
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::Sequence output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadSequence(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteSequence(const std::shared_ptr<kortex_driver::srv::DeleteSequence::Request> req, std::shared_ptr<kortex_driver::srv::DeleteSequence::Response> res)
{
	
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->DeleteSequence(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAllSequences(const std::shared_ptr<kortex_driver::srv::ReadAllSequences::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllSequences::Response> res)
{
	
	Kinova::Api::Base::SequenceList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllSequences(m_current_device_id, m_api_options);
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

bool BaseRobotServices::PlaySequence(const std::shared_ptr<kortex_driver::srv::PlaySequence::Request> req, std::shared_ptr<kortex_driver::srv::PlaySequence::Response> res)
{
	
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->PlaySequence(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::PlayAdvancedSequence(const std::shared_ptr<kortex_driver::srv::PlayAdvancedSequence::Request> req, std::shared_ptr<kortex_driver::srv::PlayAdvancedSequence::Response> res)
{
	
	Kinova::Api::Base::AdvancedSequenceHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->PlayAdvancedSequence(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::StopSequence(const std::shared_ptr<kortex_driver::srv::StopSequence::Request> req, std::shared_ptr<kortex_driver::srv::StopSequence::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->StopSequence(m_current_device_id, m_api_options);
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

bool BaseRobotServices::PauseSequence(const std::shared_ptr<kortex_driver::srv::PauseSequence::Request> req, std::shared_ptr<kortex_driver::srv::PauseSequence::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->PauseSequence(m_current_device_id, m_api_options);
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

bool BaseRobotServices::ResumeSequence(const std::shared_ptr<kortex_driver::srv::ResumeSequence::Request> req, std::shared_ptr<kortex_driver::srv::ResumeSequence::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->ResumeSequence(m_current_device_id, m_api_options);
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

bool BaseRobotServices::CreateProtectionZone(const std::shared_ptr<kortex_driver::srv::CreateProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::CreateProtectionZone::Response> res)
{
	
	Kinova::Api::Base::ProtectionZone input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::ProtectionZoneHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->CreateProtectionZone(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::UpdateProtectionZone(const std::shared_ptr<kortex_driver::srv::UpdateProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::UpdateProtectionZone::Response> res)
{
	
	Kinova::Api::Base::ProtectionZone input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->UpdateProtectionZone(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadProtectionZone(const std::shared_ptr<kortex_driver::srv::ReadProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::ReadProtectionZone::Response> res)
{
	
	Kinova::Api::Base::ProtectionZoneHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::ProtectionZone output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadProtectionZone(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteProtectionZone(const std::shared_ptr<kortex_driver::srv::DeleteProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::DeleteProtectionZone::Response> res)
{
	
	Kinova::Api::Base::ProtectionZoneHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->DeleteProtectionZone(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAllProtectionZones(const std::shared_ptr<kortex_driver::srv::ReadAllProtectionZones::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllProtectionZones::Response> res)
{
	
	Kinova::Api::Base::ProtectionZoneList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllProtectionZones(m_current_device_id, m_api_options);
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

bool BaseRobotServices::CreateMapping(const std::shared_ptr<kortex_driver::srv::CreateMapping::Request> req, std::shared_ptr<kortex_driver::srv::CreateMapping::Response> res)
{
	
	Kinova::Api::Base::Mapping input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::MappingHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->CreateMapping(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadMapping(const std::shared_ptr<kortex_driver::srv::ReadMapping::Request> req, std::shared_ptr<kortex_driver::srv::ReadMapping::Response> res)
{
	
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::Mapping output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadMapping(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::UpdateMapping(const std::shared_ptr<kortex_driver::srv::UpdateMapping::Request> req, std::shared_ptr<kortex_driver::srv::UpdateMapping::Response> res)
{
	
	Kinova::Api::Base::Mapping input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->UpdateMapping(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteMapping(const std::shared_ptr<kortex_driver::srv::DeleteMapping::Request> req, std::shared_ptr<kortex_driver::srv::DeleteMapping::Response> res)
{
	
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->DeleteMapping(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAllMappings(const std::shared_ptr<kortex_driver::srv::ReadAllMappings::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllMappings::Response> res)
{
	
	Kinova::Api::Base::MappingList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllMappings(m_current_device_id, m_api_options);
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

bool BaseRobotServices::CreateMap(const std::shared_ptr<kortex_driver::srv::CreateMap::Request> req, std::shared_ptr<kortex_driver::srv::CreateMap::Response> res)
{
	
	Kinova::Api::Base::Map input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::MapHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->CreateMap(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadMap(const std::shared_ptr<kortex_driver::srv::ReadMap::Request> req, std::shared_ptr<kortex_driver::srv::ReadMap::Response> res)
{
	
	Kinova::Api::Base::MapHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::Map output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadMap(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::UpdateMap(const std::shared_ptr<kortex_driver::srv::UpdateMap::Request> req, std::shared_ptr<kortex_driver::srv::UpdateMap::Response> res)
{
	
	Kinova::Api::Base::Map input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->UpdateMap(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteMap(const std::shared_ptr<kortex_driver::srv::DeleteMap::Request> req, std::shared_ptr<kortex_driver::srv::DeleteMap::Response> res)
{
	
	Kinova::Api::Base::MapHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->DeleteMap(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAllMaps(const std::shared_ptr<kortex_driver::srv::ReadAllMaps::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllMaps::Response> res)
{
	
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::MapList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllMaps(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ActivateMap(const std::shared_ptr<kortex_driver::srv::ActivateMap::Request> req, std::shared_ptr<kortex_driver::srv::ActivateMap::Response> res)
{
	
	Kinova::Api::Base::ActivateMapHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->ActivateMap(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::CreateAction(const std::shared_ptr<kortex_driver::srv::CreateAction::Request> req, std::shared_ptr<kortex_driver::srv::CreateAction::Response> res)
{
	
	Kinova::Api::Base::Action input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::ActionHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->CreateAction(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAction(const std::shared_ptr<kortex_driver::srv::ReadAction::Request> req, std::shared_ptr<kortex_driver::srv::ReadAction::Response> res)
{
	
	Kinova::Api::Base::ActionHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::Action output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadAction(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAllActions(const std::shared_ptr<kortex_driver::srv::ReadAllActions::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllActions::Response> res)
{
	
	Kinova::Api::Base::RequestedActionType input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::ActionList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllActions(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteAction(const std::shared_ptr<kortex_driver::srv::DeleteAction::Request> req, std::shared_ptr<kortex_driver::srv::DeleteAction::Response> res)
{
	
	Kinova::Api::Base::ActionHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->DeleteAction(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::UpdateAction(const std::shared_ptr<kortex_driver::srv::UpdateAction::Request> req, std::shared_ptr<kortex_driver::srv::UpdateAction::Response> res)
{
	
	Kinova::Api::Base::Action input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->UpdateAction(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ExecuteActionFromReference(const std::shared_ptr<kortex_driver::srv::ExecuteActionFromReference::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteActionFromReference::Response> res)
{
	
	Kinova::Api::Base::ActionHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->ExecuteActionFromReference(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ExecuteAction(const std::shared_ptr<kortex_driver::srv::ExecuteAction::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteAction::Response> res)
{
	
	Kinova::Api::Base::Action input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->ExecuteAction(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::PauseAction(const std::shared_ptr<kortex_driver::srv::PauseAction::Request> req, std::shared_ptr<kortex_driver::srv::PauseAction::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->PauseAction(m_current_device_id, m_api_options);
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

bool BaseRobotServices::StopAction(const std::shared_ptr<kortex_driver::srv::StopAction::Request> req, std::shared_ptr<kortex_driver::srv::StopAction::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->StopAction(m_current_device_id, m_api_options);
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

bool BaseRobotServices::ResumeAction(const std::shared_ptr<kortex_driver::srv::ResumeAction::Request> req, std::shared_ptr<kortex_driver::srv::ResumeAction::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->ResumeAction(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetIPv4Configuration(const std::shared_ptr<kortex_driver::srv::GetIPv4Configuration::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Configuration::Response> res)
{
	
	Kinova::Api::Base::NetworkHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::IPv4Configuration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetIPv4Configuration(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SetIPv4Configuration(const std::shared_ptr<kortex_driver::srv::SetIPv4Configuration::Request> req, std::shared_ptr<kortex_driver::srv::SetIPv4Configuration::Response> res)
{
	
	Kinova::Api::Base::FullIPv4Configuration input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SetIPv4Configuration(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SetCommunicationInterfaceEnable(const std::shared_ptr<kortex_driver::srv::SetCommunicationInterfaceEnable::Request> req, std::shared_ptr<kortex_driver::srv::SetCommunicationInterfaceEnable::Response> res)
{
	
	Kinova::Api::Base::CommunicationInterfaceConfiguration input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SetCommunicationInterfaceEnable(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::IsCommunicationInterfaceEnable(const std::shared_ptr<kortex_driver::srv::IsCommunicationInterfaceEnable::Request> req, std::shared_ptr<kortex_driver::srv::IsCommunicationInterfaceEnable::Response> res)
{
	
	Kinova::Api::Base::NetworkHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::CommunicationInterfaceConfiguration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->IsCommunicationInterfaceEnable(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetAvailableWifi(const std::shared_ptr<kortex_driver::srv::GetAvailableWifi::Request> req, std::shared_ptr<kortex_driver::srv::GetAvailableWifi::Response> res)
{
	
	Kinova::Api::Base::WifiInformationList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetAvailableWifi(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetWifiInformation(const std::shared_ptr<kortex_driver::srv::GetWifiInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetWifiInformation::Response> res)
{
	
	Kinova::Api::Base::Ssid input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::WifiInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetWifiInformation(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::AddWifiConfiguration(const std::shared_ptr<kortex_driver::srv::AddWifiConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::AddWifiConfiguration::Response> res)
{
	
	Kinova::Api::Base::WifiConfiguration input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->AddWifiConfiguration(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteWifiConfiguration(const std::shared_ptr<kortex_driver::srv::DeleteWifiConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::DeleteWifiConfiguration::Response> res)
{
	
	Kinova::Api::Base::Ssid input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->DeleteWifiConfiguration(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetAllConfiguredWifis(const std::shared_ptr<kortex_driver::srv::GetAllConfiguredWifis::Request> req, std::shared_ptr<kortex_driver::srv::GetAllConfiguredWifis::Response> res)
{
	
	Kinova::Api::Base::WifiConfigurationList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetAllConfiguredWifis(m_current_device_id, m_api_options);
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

bool BaseRobotServices::ConnectWifi(const std::shared_ptr<kortex_driver::srv::ConnectWifi::Request> req, std::shared_ptr<kortex_driver::srv::ConnectWifi::Response> res)
{
	
	Kinova::Api::Base::Ssid input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->ConnectWifi(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DisconnectWifi(const std::shared_ptr<kortex_driver::srv::DisconnectWifi::Request> req, std::shared_ptr<kortex_driver::srv::DisconnectWifi::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->DisconnectWifi(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetConnectedWifiInformation(const std::shared_ptr<kortex_driver::srv::GetConnectedWifiInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetConnectedWifiInformation::Response> res)
{
	
	Kinova::Api::Base::WifiInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetConnectedWifiInformation(m_current_device_id, m_api_options);
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

bool BaseRobotServices::Base_Unsubscribe(const std::shared_ptr<kortex_driver::srv::BaseUnsubscribe::Request> req, std::shared_ptr<kortex_driver::srv::BaseUnsubscribe::Response> res)
{
	
	Kinova::Api::Common::NotificationHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->Unsubscribe(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::OnNotificationConfigurationChangeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationConfigurationChangeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationConfigurationChangeTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ConfigurationChangeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ConfigurationChangeNotification) > callback = std::bind(&BaseRobotServices::cb_ConfigurationChangeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationConfigurationChangeTopic(callback, input, m_current_device_id);
		m_is_activated_ConfigurationChangeTopic = true;
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
void BaseRobotServices::cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif)
{
	kortex_driver::msg::ConfigurationChangeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ConfigurationChangeTopic->publish(ros_msg);
}

bool BaseRobotServices::OnNotificationMappingInfoTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationMappingInfoTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationMappingInfoTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_MappingInfoTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::MappingInfoNotification) > callback = std::bind(&BaseRobotServices::cb_MappingInfoTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationMappingInfoTopic(callback, input, m_current_device_id);
		m_is_activated_MappingInfoTopic = true;
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
void BaseRobotServices::cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif)
{
	kortex_driver::msg::MappingInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_MappingInfoTopic->publish(ros_msg);
}

bool BaseRobotServices::Base_OnNotificationControlModeTopic(const std::shared_ptr<kortex_driver::srv::BaseOnNotificationControlModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::BaseOnNotificationControlModeTopic::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/activate_publishing_of_control_mode_topic service is now deprecated and will be removed in a future release.");
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ControlModeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ControlModeNotification) > callback = std::bind(&BaseRobotServices::cb_ControlModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationControlModeTopic(callback, input, m_current_device_id);
		m_is_activated_ControlModeTopic = true;
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
void BaseRobotServices::cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif)
{
	kortex_driver::msg::BaseControlModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlModeTopic->publish(ros_msg);
}

bool BaseRobotServices::OnNotificationOperatingModeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationOperatingModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationOperatingModeTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_OperatingModeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::OperatingModeNotification) > callback = std::bind(&BaseRobotServices::cb_OperatingModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationOperatingModeTopic(callback, input, m_current_device_id);
		m_is_activated_OperatingModeTopic = true;
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
void BaseRobotServices::cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif)
{
	kortex_driver::msg::OperatingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_OperatingModeTopic->publish(ros_msg);
}

bool BaseRobotServices::OnNotificationSequenceInfoTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationSequenceInfoTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationSequenceInfoTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_SequenceInfoTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::SequenceInfoNotification) > callback = std::bind(&BaseRobotServices::cb_SequenceInfoTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationSequenceInfoTopic(callback, input, m_current_device_id);
		m_is_activated_SequenceInfoTopic = true;
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
void BaseRobotServices::cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif)
{
	kortex_driver::msg::SequenceInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SequenceInfoTopic->publish(ros_msg);
}

bool BaseRobotServices::OnNotificationProtectionZoneTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationProtectionZoneTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationProtectionZoneTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ProtectionZoneTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ProtectionZoneNotification) > callback = std::bind(&BaseRobotServices::cb_ProtectionZoneTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationProtectionZoneTopic(callback, input, m_current_device_id);
		m_is_activated_ProtectionZoneTopic = true;
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
void BaseRobotServices::cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif)
{
	kortex_driver::msg::ProtectionZoneNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ProtectionZoneTopic->publish(ros_msg);
}

bool BaseRobotServices::OnNotificationUserTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationUserTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationUserTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_UserTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::UserNotification) > callback = std::bind(&BaseRobotServices::cb_UserTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationUserTopic(callback, input, m_current_device_id);
		m_is_activated_UserTopic = true;
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
void BaseRobotServices::cb_UserTopic(Kinova::Api::Base::UserNotification notif)
{
	kortex_driver::msg::UserNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_UserTopic->publish(ros_msg);
}

bool BaseRobotServices::OnNotificationControllerTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationControllerTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationControllerTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ControllerTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ControllerNotification) > callback = std::bind(&BaseRobotServices::cb_ControllerTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationControllerTopic(callback, input, m_current_device_id);
		m_is_activated_ControllerTopic = true;
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
void BaseRobotServices::cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif)
{
	kortex_driver::msg::ControllerNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControllerTopic->publish(ros_msg);
}

bool BaseRobotServices::OnNotificationActionTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationActionTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationActionTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ActionTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ActionNotification) > callback = std::bind(&BaseRobotServices::cb_ActionTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationActionTopic(callback, input, m_current_device_id);
		m_is_activated_ActionTopic = true;
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
void BaseRobotServices::cb_ActionTopic(Kinova::Api::Base::ActionNotification notif)
{
	kortex_driver::msg::ActionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ActionTopic->publish(ros_msg);
}

bool BaseRobotServices::OnNotificationRobotEventTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationRobotEventTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationRobotEventTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_RobotEventTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::RobotEventNotification) > callback = std::bind(&BaseRobotServices::cb_RobotEventTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationRobotEventTopic(callback, input, m_current_device_id);
		m_is_activated_RobotEventTopic = true;
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
void BaseRobotServices::cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif)
{
	kortex_driver::msg::RobotEventNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_RobotEventTopic->publish(ros_msg);
}

bool BaseRobotServices::PlayCartesianTrajectory(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectory::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/play_cartesian_trajectory service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::ConstrainedPose input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->PlayCartesianTrajectory(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::PlayCartesianTrajectoryPosition(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryPosition::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryPosition::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/play_cartesian_trajectory_position service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::ConstrainedPosition input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->PlayCartesianTrajectoryPosition(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::PlayCartesianTrajectoryOrientation(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryOrientation::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryOrientation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/play_cartesian_trajectory_orientation service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::ConstrainedOrientation input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->PlayCartesianTrajectoryOrientation(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::Stop(const std::shared_ptr<kortex_driver::srv::Stop::Request> req, std::shared_ptr<kortex_driver::srv::Stop::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->Stop(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetMeasuredCartesianPose(const std::shared_ptr<kortex_driver::srv::GetMeasuredCartesianPose::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredCartesianPose::Response> res)
{
	
	Kinova::Api::Base::Pose output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetMeasuredCartesianPose(m_current_device_id, m_api_options);
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

bool BaseRobotServices::SendWrenchCommand(const std::shared_ptr<kortex_driver::srv::SendWrenchCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendWrenchCommand::Response> res)
{
	
	Kinova::Api::Base::WrenchCommand input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SendWrenchCommand(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SendWrenchJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendWrenchJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendWrenchJoystickCommand::Response> res)
{
	
	Kinova::Api::Base::WrenchCommand input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SendWrenchJoystickCommand(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SendTwistJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendTwistJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendTwistJoystickCommand::Response> res)
{
	
	Kinova::Api::Base::TwistCommand input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SendTwistJoystickCommand(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SendTwistCommand(const std::shared_ptr<kortex_driver::srv::SendTwistCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendTwistCommand::Response> res)
{
	
	Kinova::Api::Base::TwistCommand input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SendTwistCommand(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::PlayJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlayJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayJointTrajectory::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/play_joint_trajectory service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::ConstrainedJointAngles input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->PlayJointTrajectory(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::PlaySelectedJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlaySelectedJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlaySelectedJointTrajectory::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/play_selected_joint_trajectory service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::ConstrainedJointAngle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->PlaySelectedJointTrajectory(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetMeasuredJointAngles(const std::shared_ptr<kortex_driver::srv::GetMeasuredJointAngles::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredJointAngles::Response> res)
{
	
	Kinova::Api::Base::JointAngles output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetMeasuredJointAngles(m_current_device_id, m_api_options);
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

bool BaseRobotServices::SendJointSpeedsCommand(const std::shared_ptr<kortex_driver::srv::SendJointSpeedsCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendJointSpeedsCommand::Response> res)
{
	
	Kinova::Api::Base::JointSpeeds input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SendJointSpeedsCommand(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SendSelectedJointSpeedCommand(const std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedCommand::Response> res)
{
	
	Kinova::Api::Base::JointSpeed input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SendSelectedJointSpeedCommand(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SendGripperCommand(const std::shared_ptr<kortex_driver::srv::SendGripperCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendGripperCommand::Response> res)
{
	
	Kinova::Api::Base::GripperCommand input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SendGripperCommand(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetMeasuredGripperMovement(const std::shared_ptr<kortex_driver::srv::GetMeasuredGripperMovement::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredGripperMovement::Response> res)
{
	
	Kinova::Api::Base::GripperRequest input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::Gripper output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetMeasuredGripperMovement(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SetAdmittance(const std::shared_ptr<kortex_driver::srv::SetAdmittance::Request> req, std::shared_ptr<kortex_driver::srv::SetAdmittance::Response> res)
{
	
	Kinova::Api::Base::Admittance input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SetAdmittance(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SetOperatingMode(const std::shared_ptr<kortex_driver::srv::SetOperatingMode::Request> req, std::shared_ptr<kortex_driver::srv::SetOperatingMode::Response> res)
{
	
	Kinova::Api::Base::OperatingModeInformation input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SetOperatingMode(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ApplyEmergencyStop(const std::shared_ptr<kortex_driver::srv::ApplyEmergencyStop::Request> req, std::shared_ptr<kortex_driver::srv::ApplyEmergencyStop::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->ApplyEmergencyStop(m_current_device_id, m_api_options);
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

bool BaseRobotServices::Base_ClearFaults(const std::shared_ptr<kortex_driver::srv::BaseClearFaults::Request> req, std::shared_ptr<kortex_driver::srv::BaseClearFaults::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->ClearFaults(m_current_device_id, m_api_options);
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

bool BaseRobotServices::Base_GetControlMode(const std::shared_ptr<kortex_driver::srv::BaseGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::BaseGetControlMode::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_control_mode service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::ControlModeInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetControlMode(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetOperatingMode(const std::shared_ptr<kortex_driver::srv::GetOperatingMode::Request> req, std::shared_ptr<kortex_driver::srv::GetOperatingMode::Response> res)
{
	
	Kinova::Api::Base::OperatingModeInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetOperatingMode(m_current_device_id, m_api_options);
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

bool BaseRobotServices::SetServoingMode(const std::shared_ptr<kortex_driver::srv::SetServoingMode::Request> req, std::shared_ptr<kortex_driver::srv::SetServoingMode::Response> res)
{
	
	Kinova::Api::Base::ServoingModeInformation input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SetServoingMode(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetServoingMode(const std::shared_ptr<kortex_driver::srv::GetServoingMode::Request> req, std::shared_ptr<kortex_driver::srv::GetServoingMode::Response> res)
{
	
	Kinova::Api::Base::ServoingModeInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetServoingMode(m_current_device_id, m_api_options);
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

bool BaseRobotServices::OnNotificationServoingModeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationServoingModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationServoingModeTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ServoingModeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ServoingModeNotification) > callback = std::bind(&BaseRobotServices::cb_ServoingModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationServoingModeTopic(callback, input, m_current_device_id);
		m_is_activated_ServoingModeTopic = true;
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
void BaseRobotServices::cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif)
{
	kortex_driver::msg::ServoingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ServoingModeTopic->publish(ros_msg);
}

bool BaseRobotServices::RestoreFactorySettings(const std::shared_ptr<kortex_driver::srv::RestoreFactorySettings::Request> req, std::shared_ptr<kortex_driver::srv::RestoreFactorySettings::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->RestoreFactorySettings(m_current_device_id, m_api_options);
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

bool BaseRobotServices::OnNotificationFactoryTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationFactoryTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationFactoryTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_FactoryTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::FactoryNotification) > callback = std::bind(&BaseRobotServices::cb_FactoryTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationFactoryTopic(callback, input, m_current_device_id);
		m_is_activated_FactoryTopic = true;
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
void BaseRobotServices::cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif)
{
	kortex_driver::msg::FactoryNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_FactoryTopic->publish(ros_msg);
}

bool BaseRobotServices::GetAllConnectedControllers(const std::shared_ptr<kortex_driver::srv::GetAllConnectedControllers::Request> req, std::shared_ptr<kortex_driver::srv::GetAllConnectedControllers::Response> res)
{
	
	Kinova::Api::Base::ControllerList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetAllConnectedControllers(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetControllerState(const std::shared_ptr<kortex_driver::srv::GetControllerState::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerState::Response> res)
{
	
	Kinova::Api::Base::ControllerHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::ControllerState output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetControllerState(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetActuatorCount(const std::shared_ptr<kortex_driver::srv::GetActuatorCount::Request> req, std::shared_ptr<kortex_driver::srv::GetActuatorCount::Response> res)
{
	
	Kinova::Api::Base::ActuatorInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetActuatorCount(m_current_device_id, m_api_options);
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

bool BaseRobotServices::StartWifiScan(const std::shared_ptr<kortex_driver::srv::StartWifiScan::Request> req, std::shared_ptr<kortex_driver::srv::StartWifiScan::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->StartWifiScan(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetConfiguredWifi(const std::shared_ptr<kortex_driver::srv::GetConfiguredWifi::Request> req, std::shared_ptr<kortex_driver::srv::GetConfiguredWifi::Response> res)
{
	
	Kinova::Api::Base::Ssid input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::WifiConfiguration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetConfiguredWifi(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::OnNotificationNetworkTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationNetworkTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationNetworkTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_NetworkTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::NetworkNotification) > callback = std::bind(&BaseRobotServices::cb_NetworkTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationNetworkTopic(callback, input, m_current_device_id);
		m_is_activated_NetworkTopic = true;
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
void BaseRobotServices::cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif)
{
	kortex_driver::msg::NetworkNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_NetworkTopic->publish(ros_msg);
}

bool BaseRobotServices::GetArmState(const std::shared_ptr<kortex_driver::srv::GetArmState::Request> req, std::shared_ptr<kortex_driver::srv::GetArmState::Response> res)
{
	
	Kinova::Api::Base::ArmStateInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetArmState(m_current_device_id, m_api_options);
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

bool BaseRobotServices::OnNotificationArmStateTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationArmStateTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationArmStateTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ArmStateTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ArmStateNotification) > callback = std::bind(&BaseRobotServices::cb_ArmStateTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationArmStateTopic(callback, input, m_current_device_id);
		m_is_activated_ArmStateTopic = true;
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
void BaseRobotServices::cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif)
{
	kortex_driver::msg::ArmStateNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ArmStateTopic->publish(ros_msg);
}

bool BaseRobotServices::GetIPv4Information(const std::shared_ptr<kortex_driver::srv::GetIPv4Information::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Information::Response> res)
{
	
	Kinova::Api::Base::NetworkHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::IPv4Information output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetIPv4Information(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SetWifiCountryCode(const std::shared_ptr<kortex_driver::srv::SetWifiCountryCode::Request> req, std::shared_ptr<kortex_driver::srv::SetWifiCountryCode::Response> res)
{
	
	Kinova::Api::Common::CountryCode input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SetWifiCountryCode(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetWifiCountryCode(const std::shared_ptr<kortex_driver::srv::GetWifiCountryCode::Request> req, std::shared_ptr<kortex_driver::srv::GetWifiCountryCode::Response> res)
{
	
	Kinova::Api::Common::CountryCode output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetWifiCountryCode(m_current_device_id, m_api_options);
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

bool BaseRobotServices::Base_SetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::BaseSetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::BaseSetCapSenseConfig::Response> res)
{
	
	Kinova::Api::Base::CapSenseConfig input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SetCapSenseConfig(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::Base_GetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::BaseGetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::BaseGetCapSenseConfig::Response> res)
{
	
	Kinova::Api::Base::CapSenseConfig output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetCapSenseConfig(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetAllJointsSpeedHardLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedHardLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_all_joints_speed_hard_limitation service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::JointsLimitationsList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetAllJointsSpeedHardLimitation(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetAllJointsTorqueHardLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueHardLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_all_joints_torque_hard_limitation service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::JointsLimitationsList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetAllJointsTorqueHardLimitation(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetTwistHardLimitation(const std::shared_ptr<kortex_driver::srv::GetTwistHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetTwistHardLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_twist_hard_limitation service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::TwistLimitation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetTwistHardLimitation(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetWrenchHardLimitation(const std::shared_ptr<kortex_driver::srv::GetWrenchHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetWrenchHardLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_wrench_hard_limitation service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::WrenchLimitation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetWrenchHardLimitation(m_current_device_id, m_api_options);
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

bool BaseRobotServices::SendJointSpeedsJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendJointSpeedsJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendJointSpeedsJoystickCommand::Response> res)
{
	
	Kinova::Api::Base::JointSpeeds input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SendJointSpeedsJoystickCommand(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SendSelectedJointSpeedJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Response> res)
{
	
	Kinova::Api::Base::JointSpeed input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SendSelectedJointSpeedJoystickCommand(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::EnableBridge(const std::shared_ptr<kortex_driver::srv::EnableBridge::Request> req, std::shared_ptr<kortex_driver::srv::EnableBridge::Response> res)
{
	
	Kinova::Api::Base::BridgeConfig input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::BridgeResult output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->EnableBridge(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DisableBridge(const std::shared_ptr<kortex_driver::srv::DisableBridge::Request> req, std::shared_ptr<kortex_driver::srv::DisableBridge::Response> res)
{
	
	Kinova::Api::Base::BridgeIdentifier input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::BridgeResult output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->DisableBridge(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetBridgeList(const std::shared_ptr<kortex_driver::srv::GetBridgeList::Request> req, std::shared_ptr<kortex_driver::srv::GetBridgeList::Response> res)
{
	
	Kinova::Api::Base::BridgeList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetBridgeList(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetBridgeConfig(const std::shared_ptr<kortex_driver::srv::GetBridgeConfig::Request> req, std::shared_ptr<kortex_driver::srv::GetBridgeConfig::Response> res)
{
	
	Kinova::Api::Base::BridgeIdentifier input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::BridgeConfig output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetBridgeConfig(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::PlayPreComputedJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlayPreComputedJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayPreComputedJointTrajectory::Response> res)
{
	
	Kinova::Api::Base::PreComputedJointTrajectory input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->PlayPreComputedJointTrajectory(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetProductConfiguration(const std::shared_ptr<kortex_driver::srv::GetProductConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetProductConfiguration::Response> res)
{
	
	Kinova::Api::ProductConfiguration::CompleteProductConfiguration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetProductConfiguration(m_current_device_id, m_api_options);
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

bool BaseRobotServices::UpdateEndEffectorTypeConfiguration(const std::shared_ptr<kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Response> res)
{
	
	Kinova::Api::ProductConfiguration::ProductConfigurationEndEffectorType input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->UpdateEndEffectorTypeConfiguration(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::RestoreFactoryProductConfiguration(const std::shared_ptr<kortex_driver::srv::RestoreFactoryProductConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::RestoreFactoryProductConfiguration::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->RestoreFactoryProductConfiguration(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetTrajectoryErrorReport(const std::shared_ptr<kortex_driver::srv::GetTrajectoryErrorReport::Request> req, std::shared_ptr<kortex_driver::srv::GetTrajectoryErrorReport::Response> res)
{
	
	Kinova::Api::Base::TrajectoryErrorReport output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetTrajectoryErrorReport(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetAllJointsSpeedSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_all_joints_speed_soft_limitation service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::JointsLimitationsList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetAllJointsSpeedSoftLimitation(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetAllJointsTorqueSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_all_joints_torque_soft_limitation service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::JointsLimitationsList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetAllJointsTorqueSoftLimitation(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetTwistSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetTwistSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetTwistSoftLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_twist_soft_limitation service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::TwistLimitation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetTwistSoftLimitation(m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetWrenchSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetWrenchSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetWrenchSoftLimitation::Response> res)
{
	RCLCPP_WARN(m_node_handle->get_logger(), "The base/get_wrench_soft_limitation service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::WrenchLimitation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetWrenchSoftLimitation(m_current_device_id, m_api_options);
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

bool BaseRobotServices::SetControllerConfigurationMode(const std::shared_ptr<kortex_driver::srv::SetControllerConfigurationMode::Request> req, std::shared_ptr<kortex_driver::srv::SetControllerConfigurationMode::Response> res)
{
	
	Kinova::Api::Base::ControllerConfigurationMode input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SetControllerConfigurationMode(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetControllerConfigurationMode(const std::shared_ptr<kortex_driver::srv::GetControllerConfigurationMode::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerConfigurationMode::Response> res)
{
	
	Kinova::Api::Base::ControllerConfigurationMode output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetControllerConfigurationMode(m_current_device_id, m_api_options);
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

bool BaseRobotServices::StartTeaching(const std::shared_ptr<kortex_driver::srv::StartTeaching::Request> req, std::shared_ptr<kortex_driver::srv::StartTeaching::Response> res)
{
	
	Kinova::Api::Base::SequenceTaskHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->StartTeaching(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::StopTeaching(const std::shared_ptr<kortex_driver::srv::StopTeaching::Request> req, std::shared_ptr<kortex_driver::srv::StopTeaching::Response> res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->StopTeaching(m_current_device_id, m_api_options);
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

bool BaseRobotServices::AddSequenceTasks(const std::shared_ptr<kortex_driver::srv::AddSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::AddSequenceTasks::Response> res)
{
	
	Kinova::Api::Base::SequenceTasksConfiguration input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::SequenceTasksRange output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->AddSequenceTasks(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::UpdateSequenceTask(const std::shared_ptr<kortex_driver::srv::UpdateSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::UpdateSequenceTask::Response> res)
{
	
	Kinova::Api::Base::SequenceTaskConfiguration input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->UpdateSequenceTask(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SwapSequenceTasks(const std::shared_ptr<kortex_driver::srv::SwapSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::SwapSequenceTasks::Response> res)
{
	
	Kinova::Api::Base::SequenceTasksPair input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SwapSequenceTasks(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadSequenceTask(const std::shared_ptr<kortex_driver::srv::ReadSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::ReadSequenceTask::Response> res)
{
	
	Kinova::Api::Base::SequenceTaskHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::SequenceTask output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadSequenceTask(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAllSequenceTasks(const std::shared_ptr<kortex_driver::srv::ReadAllSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllSequenceTasks::Response> res)
{
	
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::SequenceTasks output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllSequenceTasks(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteSequenceTask(const std::shared_ptr<kortex_driver::srv::DeleteSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::DeleteSequenceTask::Response> res)
{
	
	Kinova::Api::Base::SequenceTaskHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->DeleteSequenceTask(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteAllSequenceTasks(const std::shared_ptr<kortex_driver::srv::DeleteAllSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::DeleteAllSequenceTasks::Response> res)
{
	
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->DeleteAllSequenceTasks(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::TakeSnapshot(const std::shared_ptr<kortex_driver::srv::TakeSnapshot::Request> req, std::shared_ptr<kortex_driver::srv::TakeSnapshot::Response> res)
{
	
	Kinova::Api::Base::Snapshot input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->TakeSnapshot(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetFirmwareBundleVersions(const std::shared_ptr<kortex_driver::srv::GetFirmwareBundleVersions::Request> req, std::shared_ptr<kortex_driver::srv::GetFirmwareBundleVersions::Response> res)
{
	
	Kinova::Api::Base::FirmwareBundleVersions output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetFirmwareBundleVersions(m_current_device_id, m_api_options);
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

bool BaseRobotServices::ExecuteWaypointTrajectory(const std::shared_ptr<kortex_driver::srv::ExecuteWaypointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteWaypointTrajectory::Response> res)
{
	
	Kinova::Api::Base::WaypointList input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->ExecuteWaypointTrajectory(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::MoveSequenceTask(const std::shared_ptr<kortex_driver::srv::MoveSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::MoveSequenceTask::Response> res)
{
	
	Kinova::Api::Base::SequenceTasksPair input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->MoveSequenceTask(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DuplicateMapping(const std::shared_ptr<kortex_driver::srv::DuplicateMapping::Request> req, std::shared_ptr<kortex_driver::srv::DuplicateMapping::Response> res)
{
	
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::MappingHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->DuplicateMapping(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DuplicateMap(const std::shared_ptr<kortex_driver::srv::DuplicateMap::Request> req, std::shared_ptr<kortex_driver::srv::DuplicateMap::Response> res)
{
	
	Kinova::Api::Base::MapHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::MapHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->DuplicateMap(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SetControllerConfiguration(const std::shared_ptr<kortex_driver::srv::SetControllerConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetControllerConfiguration::Response> res)
{
	
	Kinova::Api::Base::ControllerConfiguration input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_base->SetControllerConfiguration(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetControllerConfiguration(const std::shared_ptr<kortex_driver::srv::GetControllerConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerConfiguration::Response> res)
{
	
	Kinova::Api::Base::ControllerHandle input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::ControllerConfiguration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetControllerConfiguration(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetAllControllerConfigurations(const std::shared_ptr<kortex_driver::srv::GetAllControllerConfigurations::Request> req, std::shared_ptr<kortex_driver::srv::GetAllControllerConfigurations::Response> res)
{
	
	Kinova::Api::Base::ControllerConfigurationList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->GetAllControllerConfigurations(m_current_device_id, m_api_options);
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

bool BaseRobotServices::ComputeForwardKinematics(const std::shared_ptr<kortex_driver::srv::ComputeForwardKinematics::Request> req, std::shared_ptr<kortex_driver::srv::ComputeForwardKinematics::Response> res)
{
	
	Kinova::Api::Base::JointAngles input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::Pose output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ComputeForwardKinematics(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ComputeInverseKinematics(const std::shared_ptr<kortex_driver::srv::ComputeInverseKinematics::Request> req, std::shared_ptr<kortex_driver::srv::ComputeInverseKinematics::Response> res)
{
	
	Kinova::Api::Base::IKData input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::JointAngles output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ComputeInverseKinematics(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ValidateWaypointList(const std::shared_ptr<kortex_driver::srv::ValidateWaypointList::Request> req, std::shared_ptr<kortex_driver::srv::ValidateWaypointList::Response> res)
{
	
	Kinova::Api::Base::WaypointList input;
	ToProtoData(req->input, &input);
	Kinova::Api::Base::WaypointValidationReport output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_base->ValidateWaypointList(input, m_current_device_id, m_api_options);
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
