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

	m_serviceSetDeviceID = m_node_handle->create_service("base/set_device_id", &BaseRobotServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle->create_service("base/set_api_options", &BaseRobotServices::SetApiOptions, this);

	m_serviceCreateUserProfile = m_node_handle->create_service("base/create_user_profile", &BaseRobotServices::CreateUserProfile, this);
	m_serviceUpdateUserProfile = m_node_handle->create_service("base/update_user_profile", &BaseRobotServices::UpdateUserProfile, this);
	m_serviceReadUserProfile = m_node_handle->create_service("base/read_user_profile", &BaseRobotServices::ReadUserProfile, this);
	m_serviceDeleteUserProfile = m_node_handle->create_service("base/delete_user_profile", &BaseRobotServices::DeleteUserProfile, this);
	m_serviceReadAllUserProfiles = m_node_handle->create_service("base/read_all_user_profiles", &BaseRobotServices::ReadAllUserProfiles, this);
	m_serviceReadAllUsers = m_node_handle->create_service("base/read_all_users", &BaseRobotServices::ReadAllUsers, this);
	m_serviceChangePassword = m_node_handle->create_service("base/change_password", &BaseRobotServices::ChangePassword, this);
	m_serviceCreateSequence = m_node_handle->create_service("base/create_sequence", &BaseRobotServices::CreateSequence, this);
	m_serviceUpdateSequence = m_node_handle->create_service("base/update_sequence", &BaseRobotServices::UpdateSequence, this);
	m_serviceReadSequence = m_node_handle->create_service("base/read_sequence", &BaseRobotServices::ReadSequence, this);
	m_serviceDeleteSequence = m_node_handle->create_service("base/delete_sequence", &BaseRobotServices::DeleteSequence, this);
	m_serviceReadAllSequences = m_node_handle->create_service("base/read_all_sequences", &BaseRobotServices::ReadAllSequences, this);
	m_servicePlaySequence = m_node_handle->create_service("base/play_sequence", &BaseRobotServices::PlaySequence, this);
	m_servicePlayAdvancedSequence = m_node_handle->create_service("base/play_advanced_sequence", &BaseRobotServices::PlayAdvancedSequence, this);
	m_serviceStopSequence = m_node_handle->create_service("base/stop_sequence", &BaseRobotServices::StopSequence, this);
	m_servicePauseSequence = m_node_handle->create_service("base/pause_sequence", &BaseRobotServices::PauseSequence, this);
	m_serviceResumeSequence = m_node_handle->create_service("base/resume_sequence", &BaseRobotServices::ResumeSequence, this);
	m_serviceCreateProtectionZone = m_node_handle->create_service("base/create_protection_zone", &BaseRobotServices::CreateProtectionZone, this);
	m_serviceUpdateProtectionZone = m_node_handle->create_service("base/update_protection_zone", &BaseRobotServices::UpdateProtectionZone, this);
	m_serviceReadProtectionZone = m_node_handle->create_service("base/read_protection_zone", &BaseRobotServices::ReadProtectionZone, this);
	m_serviceDeleteProtectionZone = m_node_handle->create_service("base/delete_protection_zone", &BaseRobotServices::DeleteProtectionZone, this);
	m_serviceReadAllProtectionZones = m_node_handle->create_service("base/read_all_protection_zones", &BaseRobotServices::ReadAllProtectionZones, this);
	m_serviceCreateMapping = m_node_handle->create_service("base/create_mapping", &BaseRobotServices::CreateMapping, this);
	m_serviceReadMapping = m_node_handle->create_service("base/read_mapping", &BaseRobotServices::ReadMapping, this);
	m_serviceUpdateMapping = m_node_handle->create_service("base/update_mapping", &BaseRobotServices::UpdateMapping, this);
	m_serviceDeleteMapping = m_node_handle->create_service("base/delete_mapping", &BaseRobotServices::DeleteMapping, this);
	m_serviceReadAllMappings = m_node_handle->create_service("base/read_all_mappings", &BaseRobotServices::ReadAllMappings, this);
	m_serviceCreateMap = m_node_handle->create_service("base/create_map", &BaseRobotServices::CreateMap, this);
	m_serviceReadMap = m_node_handle->create_service("base/read_map", &BaseRobotServices::ReadMap, this);
	m_serviceUpdateMap = m_node_handle->create_service("base/update_map", &BaseRobotServices::UpdateMap, this);
	m_serviceDeleteMap = m_node_handle->create_service("base/delete_map", &BaseRobotServices::DeleteMap, this);
	m_serviceReadAllMaps = m_node_handle->create_service("base/read_all_maps", &BaseRobotServices::ReadAllMaps, this);
	m_serviceActivateMap = m_node_handle->create_service("base/activate_map", &BaseRobotServices::ActivateMap, this);
	m_serviceCreateAction = m_node_handle->create_service("base/create_action", &BaseRobotServices::CreateAction, this);
	m_serviceReadAction = m_node_handle->create_service("base/read_action", &BaseRobotServices::ReadAction, this);
	m_serviceReadAllActions = m_node_handle->create_service("base/read_all_actions", &BaseRobotServices::ReadAllActions, this);
	m_serviceDeleteAction = m_node_handle->create_service("base/delete_action", &BaseRobotServices::DeleteAction, this);
	m_serviceUpdateAction = m_node_handle->create_service("base/update_action", &BaseRobotServices::UpdateAction, this);
	m_serviceExecuteActionFromReference = m_node_handle->create_service("base/execute_action_from_reference", &BaseRobotServices::ExecuteActionFromReference, this);
	m_serviceExecuteAction = m_node_handle->create_service("base/execute_action", &BaseRobotServices::ExecuteAction, this);
	m_servicePauseAction = m_node_handle->create_service("base/pause_action", &BaseRobotServices::PauseAction, this);
	m_serviceStopAction = m_node_handle->create_service("base/stop_action", &BaseRobotServices::StopAction, this);
	m_serviceResumeAction = m_node_handle->create_service("base/resume_action", &BaseRobotServices::ResumeAction, this);
	m_serviceGetIPv4Configuration = m_node_handle->create_service("base/get_i_pv4_configuration", &BaseRobotServices::GetIPv4Configuration, this);
	m_serviceSetIPv4Configuration = m_node_handle->create_service("base/set_i_pv4_configuration", &BaseRobotServices::SetIPv4Configuration, this);
	m_serviceSetCommunicationInterfaceEnable = m_node_handle->create_service("base/set_communication_interface_enable", &BaseRobotServices::SetCommunicationInterfaceEnable, this);
	m_serviceIsCommunicationInterfaceEnable = m_node_handle->create_service("base/is_communication_interface_enable", &BaseRobotServices::IsCommunicationInterfaceEnable, this);
	m_serviceGetAvailableWifi = m_node_handle->create_service("base/get_available_wifi", &BaseRobotServices::GetAvailableWifi, this);
	m_serviceGetWifiInformation = m_node_handle->create_service("base/get_wifi_information", &BaseRobotServices::GetWifiInformation, this);
	m_serviceAddWifiConfiguration = m_node_handle->create_service("base/add_wifi_configuration", &BaseRobotServices::AddWifiConfiguration, this);
	m_serviceDeleteWifiConfiguration = m_node_handle->create_service("base/delete_wifi_configuration", &BaseRobotServices::DeleteWifiConfiguration, this);
	m_serviceGetAllConfiguredWifis = m_node_handle->create_service("base/get_all_configured_wifis", &BaseRobotServices::GetAllConfiguredWifis, this);
	m_serviceConnectWifi = m_node_handle->create_service("base/connect_wifi", &BaseRobotServices::ConnectWifi, this);
	m_serviceDisconnectWifi = m_node_handle->create_service("base/disconnect_wifi", &BaseRobotServices::DisconnectWifi, this);
	m_serviceGetConnectedWifiInformation = m_node_handle->create_service("base/get_connected_wifi_information", &BaseRobotServices::GetConnectedWifiInformation, this);
	m_serviceBase_Unsubscribe = m_node_handle->create_service("base/unsubscribe", &BaseRobotServices::Base_Unsubscribe, this);
	m_serviceOnNotificationConfigurationChangeTopic = m_node_handle->create_service("base/activate_publishing_of_configuration_change_topic", &BaseRobotServices::OnNotificationConfigurationChangeTopic, this);
	m_serviceOnNotificationMappingInfoTopic = m_node_handle->create_service("base/activate_publishing_of_mapping_info_topic", &BaseRobotServices::OnNotificationMappingInfoTopic, this);
	m_serviceBase_OnNotificationControlModeTopic = m_node_handle->create_service("base/activate_publishing_of_control_mode_topic", &BaseRobotServices::Base_OnNotificationControlModeTopic, this);
	m_serviceOnNotificationOperatingModeTopic = m_node_handle->create_service("base/activate_publishing_of_operating_mode_topic", &BaseRobotServices::OnNotificationOperatingModeTopic, this);
	m_serviceOnNotificationSequenceInfoTopic = m_node_handle->create_service("base/activate_publishing_of_sequence_info_topic", &BaseRobotServices::OnNotificationSequenceInfoTopic, this);
	m_serviceOnNotificationProtectionZoneTopic = m_node_handle->create_service("base/activate_publishing_of_protection_zone_topic", &BaseRobotServices::OnNotificationProtectionZoneTopic, this);
	m_serviceOnNotificationUserTopic = m_node_handle->create_service("base/activate_publishing_of_user_topic", &BaseRobotServices::OnNotificationUserTopic, this);
	m_serviceOnNotificationControllerTopic = m_node_handle->create_service("base/activate_publishing_of_controller_topic", &BaseRobotServices::OnNotificationControllerTopic, this);
	m_serviceOnNotificationActionTopic = m_node_handle->create_service("base/activate_publishing_of_action_topic", &BaseRobotServices::OnNotificationActionTopic, this);
	m_serviceOnNotificationRobotEventTopic = m_node_handle->create_service("base/activate_publishing_of_robot_event_topic", &BaseRobotServices::OnNotificationRobotEventTopic, this);
	m_servicePlayCartesianTrajectory = m_node_handle->create_service("base/play_cartesian_trajectory", &BaseRobotServices::PlayCartesianTrajectory, this);
	m_servicePlayCartesianTrajectoryPosition = m_node_handle->create_service("base/play_cartesian_trajectory_position", &BaseRobotServices::PlayCartesianTrajectoryPosition, this);
	m_servicePlayCartesianTrajectoryOrientation = m_node_handle->create_service("base/play_cartesian_trajectory_orientation", &BaseRobotServices::PlayCartesianTrajectoryOrientation, this);
	m_serviceStop = m_node_handle->create_service("base/stop", &BaseRobotServices::Stop, this);
	m_serviceGetMeasuredCartesianPose = m_node_handle->create_service("base/get_measured_cartesian_pose", &BaseRobotServices::GetMeasuredCartesianPose, this);
	m_serviceSendWrenchCommand = m_node_handle->create_service("base/send_wrench_command", &BaseRobotServices::SendWrenchCommand, this);
	m_serviceSendWrenchJoystickCommand = m_node_handle->create_service("base/send_wrench_joystick_command", &BaseRobotServices::SendWrenchJoystickCommand, this);
	m_serviceSendTwistJoystickCommand = m_node_handle->create_service("base/send_twist_joystick_command", &BaseRobotServices::SendTwistJoystickCommand, this);
	m_serviceSendTwistCommand = m_node_handle->create_service("base/send_twist_command", &BaseRobotServices::SendTwistCommand, this);
	m_servicePlayJointTrajectory = m_node_handle->create_service("base/play_joint_trajectory", &BaseRobotServices::PlayJointTrajectory, this);
	m_servicePlaySelectedJointTrajectory = m_node_handle->create_service("base/play_selected_joint_trajectory", &BaseRobotServices::PlaySelectedJointTrajectory, this);
	m_serviceGetMeasuredJointAngles = m_node_handle->create_service("base/get_measured_joint_angles", &BaseRobotServices::GetMeasuredJointAngles, this);
	m_serviceSendJointSpeedsCommand = m_node_handle->create_service("base/send_joint_speeds_command", &BaseRobotServices::SendJointSpeedsCommand, this);
	m_serviceSendSelectedJointSpeedCommand = m_node_handle->create_service("base/send_selected_joint_speed_command", &BaseRobotServices::SendSelectedJointSpeedCommand, this);
	m_serviceSendGripperCommand = m_node_handle->create_service("base/send_gripper_command", &BaseRobotServices::SendGripperCommand, this);
	m_serviceGetMeasuredGripperMovement = m_node_handle->create_service("base/get_measured_gripper_movement", &BaseRobotServices::GetMeasuredGripperMovement, this);
	m_serviceSetAdmittance = m_node_handle->create_service("base/set_admittance", &BaseRobotServices::SetAdmittance, this);
	m_serviceSetOperatingMode = m_node_handle->create_service("base/set_operating_mode", &BaseRobotServices::SetOperatingMode, this);
	m_serviceApplyEmergencyStop = m_node_handle->create_service("base/apply_emergency_stop", &BaseRobotServices::ApplyEmergencyStop, this);
	m_serviceBase_ClearFaults = m_node_handle->create_service("base/clear_faults", &BaseRobotServices::Base_ClearFaults, this);
	m_serviceBase_GetControlMode = m_node_handle->create_service("base/get_control_mode", &BaseRobotServices::Base_GetControlMode, this);
	m_serviceGetOperatingMode = m_node_handle->create_service("base/get_operating_mode", &BaseRobotServices::GetOperatingMode, this);
	m_serviceSetServoingMode = m_node_handle->create_service("base/set_servoing_mode", &BaseRobotServices::SetServoingMode, this);
	m_serviceGetServoingMode = m_node_handle->create_service("base/get_servoing_mode", &BaseRobotServices::GetServoingMode, this);
	m_serviceOnNotificationServoingModeTopic = m_node_handle->create_service("base/activate_publishing_of_servoing_mode_topic", &BaseRobotServices::OnNotificationServoingModeTopic, this);
	m_serviceRestoreFactorySettings = m_node_handle->create_service("base/restore_factory_settings", &BaseRobotServices::RestoreFactorySettings, this);
	m_serviceOnNotificationFactoryTopic = m_node_handle->create_service("base/activate_publishing_of_factory_topic", &BaseRobotServices::OnNotificationFactoryTopic, this);
	m_serviceGetAllConnectedControllers = m_node_handle->create_service("base/get_all_connected_controllers", &BaseRobotServices::GetAllConnectedControllers, this);
	m_serviceGetControllerState = m_node_handle->create_service("base/get_controller_state", &BaseRobotServices::GetControllerState, this);
	m_serviceGetActuatorCount = m_node_handle->create_service("base/get_actuator_count", &BaseRobotServices::GetActuatorCount, this);
	m_serviceStartWifiScan = m_node_handle->create_service("base/start_wifi_scan", &BaseRobotServices::StartWifiScan, this);
	m_serviceGetConfiguredWifi = m_node_handle->create_service("base/get_configured_wifi", &BaseRobotServices::GetConfiguredWifi, this);
	m_serviceOnNotificationNetworkTopic = m_node_handle->create_service("base/activate_publishing_of_network_topic", &BaseRobotServices::OnNotificationNetworkTopic, this);
	m_serviceGetArmState = m_node_handle->create_service("base/get_arm_state", &BaseRobotServices::GetArmState, this);
	m_serviceOnNotificationArmStateTopic = m_node_handle->create_service("base/activate_publishing_of_arm_state_topic", &BaseRobotServices::OnNotificationArmStateTopic, this);
	m_serviceGetIPv4Information = m_node_handle->create_service("base/get_i_pv4_information", &BaseRobotServices::GetIPv4Information, this);
	m_serviceSetWifiCountryCode = m_node_handle->create_service("base/set_wifi_country_code", &BaseRobotServices::SetWifiCountryCode, this);
	m_serviceGetWifiCountryCode = m_node_handle->create_service("base/get_wifi_country_code", &BaseRobotServices::GetWifiCountryCode, this);
	m_serviceBase_SetCapSenseConfig = m_node_handle->create_service("base/set_cap_sense_config", &BaseRobotServices::Base_SetCapSenseConfig, this);
	m_serviceBase_GetCapSenseConfig = m_node_handle->create_service("base/get_cap_sense_config", &BaseRobotServices::Base_GetCapSenseConfig, this);
	m_serviceGetAllJointsSpeedHardLimitation = m_node_handle->create_service("base/get_all_joints_speed_hard_limitation", &BaseRobotServices::GetAllJointsSpeedHardLimitation, this);
	m_serviceGetAllJointsTorqueHardLimitation = m_node_handle->create_service("base/get_all_joints_torque_hard_limitation", &BaseRobotServices::GetAllJointsTorqueHardLimitation, this);
	m_serviceGetTwistHardLimitation = m_node_handle->create_service("base/get_twist_hard_limitation", &BaseRobotServices::GetTwistHardLimitation, this);
	m_serviceGetWrenchHardLimitation = m_node_handle->create_service("base/get_wrench_hard_limitation", &BaseRobotServices::GetWrenchHardLimitation, this);
	m_serviceSendJointSpeedsJoystickCommand = m_node_handle->create_service("base/send_joint_speeds_joystick_command", &BaseRobotServices::SendJointSpeedsJoystickCommand, this);
	m_serviceSendSelectedJointSpeedJoystickCommand = m_node_handle->create_service("base/send_selected_joint_speed_joystick_command", &BaseRobotServices::SendSelectedJointSpeedJoystickCommand, this);
	m_serviceEnableBridge = m_node_handle->create_service("base/enable_bridge", &BaseRobotServices::EnableBridge, this);
	m_serviceDisableBridge = m_node_handle->create_service("base/disable_bridge", &BaseRobotServices::DisableBridge, this);
	m_serviceGetBridgeList = m_node_handle->create_service("base/get_bridge_list", &BaseRobotServices::GetBridgeList, this);
	m_serviceGetBridgeConfig = m_node_handle->create_service("base/get_bridge_config", &BaseRobotServices::GetBridgeConfig, this);
	m_servicePlayPreComputedJointTrajectory = m_node_handle->create_service("base/play_pre_computed_joint_trajectory", &BaseRobotServices::PlayPreComputedJointTrajectory, this);
	m_serviceGetProductConfiguration = m_node_handle->create_service("base/get_product_configuration", &BaseRobotServices::GetProductConfiguration, this);
	m_serviceUpdateEndEffectorTypeConfiguration = m_node_handle->create_service("base/update_end_effector_type_configuration", &BaseRobotServices::UpdateEndEffectorTypeConfiguration, this);
	m_serviceRestoreFactoryProductConfiguration = m_node_handle->create_service("base/restore_factory_product_configuration", &BaseRobotServices::RestoreFactoryProductConfiguration, this);
	m_serviceGetTrajectoryErrorReport = m_node_handle->create_service("base/get_trajectory_error_report", &BaseRobotServices::GetTrajectoryErrorReport, this);
	m_serviceGetAllJointsSpeedSoftLimitation = m_node_handle->create_service("base/get_all_joints_speed_soft_limitation", &BaseRobotServices::GetAllJointsSpeedSoftLimitation, this);
	m_serviceGetAllJointsTorqueSoftLimitation = m_node_handle->create_service("base/get_all_joints_torque_soft_limitation", &BaseRobotServices::GetAllJointsTorqueSoftLimitation, this);
	m_serviceGetTwistSoftLimitation = m_node_handle->create_service("base/get_twist_soft_limitation", &BaseRobotServices::GetTwistSoftLimitation, this);
	m_serviceGetWrenchSoftLimitation = m_node_handle->create_service("base/get_wrench_soft_limitation", &BaseRobotServices::GetWrenchSoftLimitation, this);
	m_serviceSetControllerConfigurationMode = m_node_handle->create_service("base/set_controller_configuration_mode", &BaseRobotServices::SetControllerConfigurationMode, this);
	m_serviceGetControllerConfigurationMode = m_node_handle->create_service("base/get_controller_configuration_mode", &BaseRobotServices::GetControllerConfigurationMode, this);
	m_serviceStartTeaching = m_node_handle->create_service("base/start_teaching", &BaseRobotServices::StartTeaching, this);
	m_serviceStopTeaching = m_node_handle->create_service("base/stop_teaching", &BaseRobotServices::StopTeaching, this);
	m_serviceAddSequenceTasks = m_node_handle->create_service("base/add_sequence_tasks", &BaseRobotServices::AddSequenceTasks, this);
	m_serviceUpdateSequenceTask = m_node_handle->create_service("base/update_sequence_task", &BaseRobotServices::UpdateSequenceTask, this);
	m_serviceSwapSequenceTasks = m_node_handle->create_service("base/swap_sequence_tasks", &BaseRobotServices::SwapSequenceTasks, this);
	m_serviceReadSequenceTask = m_node_handle->create_service("base/read_sequence_task", &BaseRobotServices::ReadSequenceTask, this);
	m_serviceReadAllSequenceTasks = m_node_handle->create_service("base/read_all_sequence_tasks", &BaseRobotServices::ReadAllSequenceTasks, this);
	m_serviceDeleteSequenceTask = m_node_handle->create_service("base/delete_sequence_task", &BaseRobotServices::DeleteSequenceTask, this);
	m_serviceDeleteAllSequenceTasks = m_node_handle->create_service("base/delete_all_sequence_tasks", &BaseRobotServices::DeleteAllSequenceTasks, this);
	m_serviceTakeSnapshot = m_node_handle->create_service("base/take_snapshot", &BaseRobotServices::TakeSnapshot, this);
	m_serviceGetFirmwareBundleVersions = m_node_handle->create_service("base/get_firmware_bundle_versions", &BaseRobotServices::GetFirmwareBundleVersions, this);
	m_serviceExecuteWaypointTrajectory = m_node_handle->create_service("base/execute_waypoint_trajectory", &BaseRobotServices::ExecuteWaypointTrajectory, this);
	m_serviceMoveSequenceTask = m_node_handle->create_service("base/move_sequence_task", &BaseRobotServices::MoveSequenceTask, this);
	m_serviceDuplicateMapping = m_node_handle->create_service("base/duplicate_mapping", &BaseRobotServices::DuplicateMapping, this);
	m_serviceDuplicateMap = m_node_handle->create_service("base/duplicate_map", &BaseRobotServices::DuplicateMap, this);
	m_serviceSetControllerConfiguration = m_node_handle->create_service("base/set_controller_configuration", &BaseRobotServices::SetControllerConfiguration, this);
	m_serviceGetControllerConfiguration = m_node_handle->create_service("base/get_controller_configuration", &BaseRobotServices::GetControllerConfiguration, this);
	m_serviceGetAllControllerConfigurations = m_node_handle->create_service("base/get_all_controller_configurations", &BaseRobotServices::GetAllControllerConfigurations, this);
	m_serviceComputeForwardKinematics = m_node_handle->create_service("base/compute_forward_kinematics", &BaseRobotServices::ComputeForwardKinematics, this);
	m_serviceComputeInverseKinematics = m_node_handle->create_service("base/compute_inverse_kinematics", &BaseRobotServices::ComputeInverseKinematics, this);
	m_serviceValidateWaypointList = m_node_handle->create_service("base/validate_waypoint_list", &BaseRobotServices::ValidateWaypointList, this);
}

bool BaseRobotServices::SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool BaseRobotServices::SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool BaseRobotServices::CreateUserProfile(kortex_driver::srv::CreateUserProfile::Request  &req, kortex_driver::srv::CreateUserProfile::Response &res)
{
	
	Kinova::Api::Base::FullUserProfile input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::UpdateUserProfile(kortex_driver::srv::UpdateUserProfile::Request  &req, kortex_driver::srv::UpdateUserProfile::Response &res)
{
	
	Kinova::Api::Base::UserProfile input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadUserProfile(kortex_driver::srv::ReadUserProfile::Request  &req, kortex_driver::srv::ReadUserProfile::Response &res)
{
	
	Kinova::Api::Common::UserProfileHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DeleteUserProfile(kortex_driver::srv::DeleteUserProfile::Request  &req, kortex_driver::srv::DeleteUserProfile::Response &res)
{
	
	Kinova::Api::Common::UserProfileHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadAllUserProfiles(kortex_driver::srv::ReadAllUserProfiles::Request  &req, kortex_driver::srv::ReadAllUserProfiles::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadAllUsers(kortex_driver::srv::ReadAllUsers::Request  &req, kortex_driver::srv::ReadAllUsers::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ChangePassword(kortex_driver::srv::ChangePassword::Request  &req, kortex_driver::srv::ChangePassword::Response &res)
{
	
	Kinova::Api::Base::PasswordChange input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::CreateSequence(kortex_driver::srv::CreateSequence::Request  &req, kortex_driver::srv::CreateSequence::Response &res)
{
	
	Kinova::Api::Base::Sequence input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::UpdateSequence(kortex_driver::srv::UpdateSequence::Request  &req, kortex_driver::srv::UpdateSequence::Response &res)
{
	
	Kinova::Api::Base::Sequence input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadSequence(kortex_driver::srv::ReadSequence::Request  &req, kortex_driver::srv::ReadSequence::Response &res)
{
	
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DeleteSequence(kortex_driver::srv::DeleteSequence::Request  &req, kortex_driver::srv::DeleteSequence::Response &res)
{
	
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadAllSequences(kortex_driver::srv::ReadAllSequences::Request  &req, kortex_driver::srv::ReadAllSequences::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::PlaySequence(kortex_driver::srv::PlaySequence::Request  &req, kortex_driver::srv::PlaySequence::Response &res)
{
	
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::PlayAdvancedSequence(kortex_driver::srv::PlayAdvancedSequence::Request  &req, kortex_driver::srv::PlayAdvancedSequence::Response &res)
{
	
	Kinova::Api::Base::AdvancedSequenceHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::StopSequence(kortex_driver::srv::StopSequence::Request  &req, kortex_driver::srv::StopSequence::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::PauseSequence(kortex_driver::srv::PauseSequence::Request  &req, kortex_driver::srv::PauseSequence::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ResumeSequence(kortex_driver::srv::ResumeSequence::Request  &req, kortex_driver::srv::ResumeSequence::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::CreateProtectionZone(kortex_driver::srv::CreateProtectionZone::Request  &req, kortex_driver::srv::CreateProtectionZone::Response &res)
{
	
	Kinova::Api::Base::ProtectionZone input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::UpdateProtectionZone(kortex_driver::srv::UpdateProtectionZone::Request  &req, kortex_driver::srv::UpdateProtectionZone::Response &res)
{
	
	Kinova::Api::Base::ProtectionZone input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadProtectionZone(kortex_driver::srv::ReadProtectionZone::Request  &req, kortex_driver::srv::ReadProtectionZone::Response &res)
{
	
	Kinova::Api::Base::ProtectionZoneHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DeleteProtectionZone(kortex_driver::srv::DeleteProtectionZone::Request  &req, kortex_driver::srv::DeleteProtectionZone::Response &res)
{
	
	Kinova::Api::Base::ProtectionZoneHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadAllProtectionZones(kortex_driver::srv::ReadAllProtectionZones::Request  &req, kortex_driver::srv::ReadAllProtectionZones::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::CreateMapping(kortex_driver::srv::CreateMapping::Request  &req, kortex_driver::srv::CreateMapping::Response &res)
{
	
	Kinova::Api::Base::Mapping input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadMapping(kortex_driver::srv::ReadMapping::Request  &req, kortex_driver::srv::ReadMapping::Response &res)
{
	
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::UpdateMapping(kortex_driver::srv::UpdateMapping::Request  &req, kortex_driver::srv::UpdateMapping::Response &res)
{
	
	Kinova::Api::Base::Mapping input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DeleteMapping(kortex_driver::srv::DeleteMapping::Request  &req, kortex_driver::srv::DeleteMapping::Response &res)
{
	
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadAllMappings(kortex_driver::srv::ReadAllMappings::Request  &req, kortex_driver::srv::ReadAllMappings::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::CreateMap(kortex_driver::srv::CreateMap::Request  &req, kortex_driver::srv::CreateMap::Response &res)
{
	
	Kinova::Api::Base::Map input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadMap(kortex_driver::srv::ReadMap::Request  &req, kortex_driver::srv::ReadMap::Response &res)
{
	
	Kinova::Api::Base::MapHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::UpdateMap(kortex_driver::srv::UpdateMap::Request  &req, kortex_driver::srv::UpdateMap::Response &res)
{
	
	Kinova::Api::Base::Map input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DeleteMap(kortex_driver::srv::DeleteMap::Request  &req, kortex_driver::srv::DeleteMap::Response &res)
{
	
	Kinova::Api::Base::MapHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadAllMaps(kortex_driver::srv::ReadAllMaps::Request  &req, kortex_driver::srv::ReadAllMaps::Response &res)
{
	
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ActivateMap(kortex_driver::srv::ActivateMap::Request  &req, kortex_driver::srv::ActivateMap::Response &res)
{
	
	Kinova::Api::Base::ActivateMapHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::CreateAction(kortex_driver::srv::CreateAction::Request  &req, kortex_driver::srv::CreateAction::Response &res)
{
	
	Kinova::Api::Base::Action input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadAction(kortex_driver::srv::ReadAction::Request  &req, kortex_driver::srv::ReadAction::Response &res)
{
	
	Kinova::Api::Base::ActionHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadAllActions(kortex_driver::srv::ReadAllActions::Request  &req, kortex_driver::srv::ReadAllActions::Response &res)
{
	
	Kinova::Api::Base::RequestedActionType input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DeleteAction(kortex_driver::srv::DeleteAction::Request  &req, kortex_driver::srv::DeleteAction::Response &res)
{
	
	Kinova::Api::Base::ActionHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::UpdateAction(kortex_driver::srv::UpdateAction::Request  &req, kortex_driver::srv::UpdateAction::Response &res)
{
	
	Kinova::Api::Base::Action input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ExecuteActionFromReference(kortex_driver::srv::ExecuteActionFromReference::Request  &req, kortex_driver::srv::ExecuteActionFromReference::Response &res)
{
	
	Kinova::Api::Base::ActionHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ExecuteAction(kortex_driver::srv::ExecuteAction::Request  &req, kortex_driver::srv::ExecuteAction::Response &res)
{
	
	Kinova::Api::Base::Action input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::PauseAction(kortex_driver::srv::PauseAction::Request  &req, kortex_driver::srv::PauseAction::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::StopAction(kortex_driver::srv::StopAction::Request  &req, kortex_driver::srv::StopAction::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ResumeAction(kortex_driver::srv::ResumeAction::Request  &req, kortex_driver::srv::ResumeAction::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetIPv4Configuration(kortex_driver::srv::GetIPv4Configuration::Request  &req, kortex_driver::srv::GetIPv4Configuration::Response &res)
{
	
	Kinova::Api::Base::NetworkHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SetIPv4Configuration(kortex_driver::srv::SetIPv4Configuration::Request  &req, kortex_driver::srv::SetIPv4Configuration::Response &res)
{
	
	Kinova::Api::Base::FullIPv4Configuration input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SetCommunicationInterfaceEnable(kortex_driver::srv::SetCommunicationInterfaceEnable::Request  &req, kortex_driver::srv::SetCommunicationInterfaceEnable::Response &res)
{
	
	Kinova::Api::Base::CommunicationInterfaceConfiguration input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::IsCommunicationInterfaceEnable(kortex_driver::srv::IsCommunicationInterfaceEnable::Request  &req, kortex_driver::srv::IsCommunicationInterfaceEnable::Response &res)
{
	
	Kinova::Api::Base::NetworkHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetAvailableWifi(kortex_driver::srv::GetAvailableWifi::Request  &req, kortex_driver::srv::GetAvailableWifi::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetWifiInformation(kortex_driver::srv::GetWifiInformation::Request  &req, kortex_driver::srv::GetWifiInformation::Response &res)
{
	
	Kinova::Api::Base::Ssid input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::AddWifiConfiguration(kortex_driver::srv::AddWifiConfiguration::Request  &req, kortex_driver::srv::AddWifiConfiguration::Response &res)
{
	
	Kinova::Api::Base::WifiConfiguration input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DeleteWifiConfiguration(kortex_driver::srv::DeleteWifiConfiguration::Request  &req, kortex_driver::srv::DeleteWifiConfiguration::Response &res)
{
	
	Kinova::Api::Base::Ssid input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetAllConfiguredWifis(kortex_driver::srv::GetAllConfiguredWifis::Request  &req, kortex_driver::srv::GetAllConfiguredWifis::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ConnectWifi(kortex_driver::srv::ConnectWifi::Request  &req, kortex_driver::srv::ConnectWifi::Response &res)
{
	
	Kinova::Api::Base::Ssid input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DisconnectWifi(kortex_driver::srv::DisconnectWifi::Request  &req, kortex_driver::srv::DisconnectWifi::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetConnectedWifiInformation(kortex_driver::srv::GetConnectedWifiInformation::Request  &req, kortex_driver::srv::GetConnectedWifiInformation::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::Base_Unsubscribe(kortex_driver::srv::BaseUnsubscribe::Request  &req, kortex_driver::srv::BaseUnsubscribe::Response &res)
{
	
	Kinova::Api::Common::NotificationHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::OnNotificationConfigurationChangeTopic(kortex_driver::srv::OnNotificationConfigurationChangeTopic::Request  &req, kortex_driver::srv::OnNotificationConfigurationChangeTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ConfigurationChangeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif)
{
	kortex_driver::msg::ConfigurationChangeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ConfigurationChangeTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationMappingInfoTopic(kortex_driver::srv::OnNotificationMappingInfoTopic::Request  &req, kortex_driver::srv::OnNotificationMappingInfoTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_MappingInfoTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif)
{
	kortex_driver::msg::MappingInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_MappingInfoTopic.publish(ros_msg);
}

bool BaseRobotServices::Base_OnNotificationControlModeTopic(kortex_driver::srv::BaseOnNotificationControlModeTopic::Request  &req, kortex_driver::srv::BaseOnNotificationControlModeTopic::Response &res)
{
	ROS_WARN("The base/activate_publishing_of_control_mode_topic service is now deprecated and will be removed in a future release.");
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ControlModeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif)
{
	kortex_driver::msg::BaseControlModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlModeTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationOperatingModeTopic(kortex_driver::srv::OnNotificationOperatingModeTopic::Request  &req, kortex_driver::srv::OnNotificationOperatingModeTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_OperatingModeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif)
{
	kortex_driver::msg::OperatingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_OperatingModeTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationSequenceInfoTopic(kortex_driver::srv::OnNotificationSequenceInfoTopic::Request  &req, kortex_driver::srv::OnNotificationSequenceInfoTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_SequenceInfoTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif)
{
	kortex_driver::msg::SequenceInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SequenceInfoTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationProtectionZoneTopic(kortex_driver::srv::OnNotificationProtectionZoneTopic::Request  &req, kortex_driver::srv::OnNotificationProtectionZoneTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ProtectionZoneTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif)
{
	kortex_driver::msg::ProtectionZoneNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ProtectionZoneTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationUserTopic(kortex_driver::srv::OnNotificationUserTopic::Request  &req, kortex_driver::srv::OnNotificationUserTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_UserTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_UserTopic(Kinova::Api::Base::UserNotification notif)
{
	kortex_driver::msg::UserNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_UserTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationControllerTopic(kortex_driver::srv::OnNotificationControllerTopic::Request  &req, kortex_driver::srv::OnNotificationControllerTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ControllerTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif)
{
	kortex_driver::msg::ControllerNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControllerTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationActionTopic(kortex_driver::srv::OnNotificationActionTopic::Request  &req, kortex_driver::srv::OnNotificationActionTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ActionTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_ActionTopic(Kinova::Api::Base::ActionNotification notif)
{
	kortex_driver::msg::ActionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ActionTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationRobotEventTopic(kortex_driver::srv::OnNotificationRobotEventTopic::Request  &req, kortex_driver::srv::OnNotificationRobotEventTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_RobotEventTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif)
{
	kortex_driver::msg::RobotEventNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_RobotEventTopic.publish(ros_msg);
}

bool BaseRobotServices::PlayCartesianTrajectory(kortex_driver::srv::PlayCartesianTrajectory::Request  &req, kortex_driver::srv::PlayCartesianTrajectory::Response &res)
{
	ROS_WARN("The base/play_cartesian_trajectory service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::ConstrainedPose input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::PlayCartesianTrajectoryPosition(kortex_driver::srv::PlayCartesianTrajectoryPosition::Request  &req, kortex_driver::srv::PlayCartesianTrajectoryPosition::Response &res)
{
	ROS_WARN("The base/play_cartesian_trajectory_position service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::ConstrainedPosition input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::PlayCartesianTrajectoryOrientation(kortex_driver::srv::PlayCartesianTrajectoryOrientation::Request  &req, kortex_driver::srv::PlayCartesianTrajectoryOrientation::Response &res)
{
	ROS_WARN("The base/play_cartesian_trajectory_orientation service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::ConstrainedOrientation input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::Stop(kortex_driver::srv::Stop::Request  &req, kortex_driver::srv::Stop::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetMeasuredCartesianPose(kortex_driver::srv::GetMeasuredCartesianPose::Request  &req, kortex_driver::srv::GetMeasuredCartesianPose::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SendWrenchCommand(kortex_driver::srv::SendWrenchCommand::Request  &req, kortex_driver::srv::SendWrenchCommand::Response &res)
{
	
	Kinova::Api::Base::WrenchCommand input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SendWrenchJoystickCommand(kortex_driver::srv::SendWrenchJoystickCommand::Request  &req, kortex_driver::srv::SendWrenchJoystickCommand::Response &res)
{
	
	Kinova::Api::Base::WrenchCommand input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SendTwistJoystickCommand(kortex_driver::srv::SendTwistJoystickCommand::Request  &req, kortex_driver::srv::SendTwistJoystickCommand::Response &res)
{
	
	Kinova::Api::Base::TwistCommand input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SendTwistCommand(kortex_driver::srv::SendTwistCommand::Request  &req, kortex_driver::srv::SendTwistCommand::Response &res)
{
	
	Kinova::Api::Base::TwistCommand input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::PlayJointTrajectory(kortex_driver::srv::PlayJointTrajectory::Request  &req, kortex_driver::srv::PlayJointTrajectory::Response &res)
{
	ROS_WARN("The base/play_joint_trajectory service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::ConstrainedJointAngles input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::PlaySelectedJointTrajectory(kortex_driver::srv::PlaySelectedJointTrajectory::Request  &req, kortex_driver::srv::PlaySelectedJointTrajectory::Response &res)
{
	ROS_WARN("The base/play_selected_joint_trajectory service is now deprecated and will be removed in a future release.");
	
	Kinova::Api::Base::ConstrainedJointAngle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetMeasuredJointAngles(kortex_driver::srv::GetMeasuredJointAngles::Request  &req, kortex_driver::srv::GetMeasuredJointAngles::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SendJointSpeedsCommand(kortex_driver::srv::SendJointSpeedsCommand::Request  &req, kortex_driver::srv::SendJointSpeedsCommand::Response &res)
{
	
	Kinova::Api::Base::JointSpeeds input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SendSelectedJointSpeedCommand(kortex_driver::srv::SendSelectedJointSpeedCommand::Request  &req, kortex_driver::srv::SendSelectedJointSpeedCommand::Response &res)
{
	
	Kinova::Api::Base::JointSpeed input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SendGripperCommand(kortex_driver::srv::SendGripperCommand::Request  &req, kortex_driver::srv::SendGripperCommand::Response &res)
{
	
	Kinova::Api::Base::GripperCommand input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetMeasuredGripperMovement(kortex_driver::srv::GetMeasuredGripperMovement::Request  &req, kortex_driver::srv::GetMeasuredGripperMovement::Response &res)
{
	
	Kinova::Api::Base::GripperRequest input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SetAdmittance(kortex_driver::srv::SetAdmittance::Request  &req, kortex_driver::srv::SetAdmittance::Response &res)
{
	
	Kinova::Api::Base::Admittance input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SetOperatingMode(kortex_driver::srv::SetOperatingMode::Request  &req, kortex_driver::srv::SetOperatingMode::Response &res)
{
	
	Kinova::Api::Base::OperatingModeInformation input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ApplyEmergencyStop(kortex_driver::srv::ApplyEmergencyStop::Request  &req, kortex_driver::srv::ApplyEmergencyStop::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::Base_ClearFaults(kortex_driver::srv::BaseClearFaults::Request  &req, kortex_driver::srv::BaseClearFaults::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::Base_GetControlMode(kortex_driver::srv::BaseGetControlMode::Request  &req, kortex_driver::srv::BaseGetControlMode::Response &res)
{
	ROS_WARN("The base/get_control_mode service is now deprecated and will be removed in a future release.");
	
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetOperatingMode(kortex_driver::srv::GetOperatingMode::Request  &req, kortex_driver::srv::GetOperatingMode::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SetServoingMode(kortex_driver::srv::SetServoingMode::Request  &req, kortex_driver::srv::SetServoingMode::Response &res)
{
	
	Kinova::Api::Base::ServoingModeInformation input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetServoingMode(kortex_driver::srv::GetServoingMode::Request  &req, kortex_driver::srv::GetServoingMode::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::OnNotificationServoingModeTopic(kortex_driver::srv::OnNotificationServoingModeTopic::Request  &req, kortex_driver::srv::OnNotificationServoingModeTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ServoingModeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif)
{
	kortex_driver::msg::ServoingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ServoingModeTopic.publish(ros_msg);
}

bool BaseRobotServices::RestoreFactorySettings(kortex_driver::srv::RestoreFactorySettings::Request  &req, kortex_driver::srv::RestoreFactorySettings::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::OnNotificationFactoryTopic(kortex_driver::srv::OnNotificationFactoryTopic::Request  &req, kortex_driver::srv::OnNotificationFactoryTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_FactoryTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif)
{
	kortex_driver::msg::FactoryNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_FactoryTopic.publish(ros_msg);
}

bool BaseRobotServices::GetAllConnectedControllers(kortex_driver::srv::GetAllConnectedControllers::Request  &req, kortex_driver::srv::GetAllConnectedControllers::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetControllerState(kortex_driver::srv::GetControllerState::Request  &req, kortex_driver::srv::GetControllerState::Response &res)
{
	
	Kinova::Api::Base::ControllerHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetActuatorCount(kortex_driver::srv::GetActuatorCount::Request  &req, kortex_driver::srv::GetActuatorCount::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::StartWifiScan(kortex_driver::srv::StartWifiScan::Request  &req, kortex_driver::srv::StartWifiScan::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetConfiguredWifi(kortex_driver::srv::GetConfiguredWifi::Request  &req, kortex_driver::srv::GetConfiguredWifi::Response &res)
{
	
	Kinova::Api::Base::Ssid input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::OnNotificationNetworkTopic(kortex_driver::srv::OnNotificationNetworkTopic::Request  &req, kortex_driver::srv::OnNotificationNetworkTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_NetworkTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif)
{
	kortex_driver::msg::NetworkNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_NetworkTopic.publish(ros_msg);
}

bool BaseRobotServices::GetArmState(kortex_driver::srv::GetArmState::Request  &req, kortex_driver::srv::GetArmState::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::OnNotificationArmStateTopic(kortex_driver::srv::OnNotificationArmStateTopic::Request  &req, kortex_driver::srv::OnNotificationArmStateTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ArmStateTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
void BaseRobotServices::cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif)
{
	kortex_driver::msg::ArmStateNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ArmStateTopic.publish(ros_msg);
}

bool BaseRobotServices::GetIPv4Information(kortex_driver::srv::GetIPv4Information::Request  &req, kortex_driver::srv::GetIPv4Information::Response &res)
{
	
	Kinova::Api::Base::NetworkHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SetWifiCountryCode(kortex_driver::srv::SetWifiCountryCode::Request  &req, kortex_driver::srv::SetWifiCountryCode::Response &res)
{
	
	Kinova::Api::Common::CountryCode input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetWifiCountryCode(kortex_driver::srv::GetWifiCountryCode::Request  &req, kortex_driver::srv::GetWifiCountryCode::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::Base_SetCapSenseConfig(kortex_driver::srv::BaseSetCapSenseConfig::Request  &req, kortex_driver::srv::BaseSetCapSenseConfig::Response &res)
{
	
	Kinova::Api::Base::CapSenseConfig input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::Base_GetCapSenseConfig(kortex_driver::srv::BaseGetCapSenseConfig::Request  &req, kortex_driver::srv::BaseGetCapSenseConfig::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetAllJointsSpeedHardLimitation(kortex_driver::srv::GetAllJointsSpeedHardLimitation::Request  &req, kortex_driver::srv::GetAllJointsSpeedHardLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_speed_hard_limitation service is now deprecated and will be removed in a future release.");
	
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetAllJointsTorqueHardLimitation(kortex_driver::srv::GetAllJointsTorqueHardLimitation::Request  &req, kortex_driver::srv::GetAllJointsTorqueHardLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_torque_hard_limitation service is now deprecated and will be removed in a future release.");
	
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetTwistHardLimitation(kortex_driver::srv::GetTwistHardLimitation::Request  &req, kortex_driver::srv::GetTwistHardLimitation::Response &res)
{
	ROS_WARN("The base/get_twist_hard_limitation service is now deprecated and will be removed in a future release.");
	
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetWrenchHardLimitation(kortex_driver::srv::GetWrenchHardLimitation::Request  &req, kortex_driver::srv::GetWrenchHardLimitation::Response &res)
{
	ROS_WARN("The base/get_wrench_hard_limitation service is now deprecated and will be removed in a future release.");
	
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SendJointSpeedsJoystickCommand(kortex_driver::srv::SendJointSpeedsJoystickCommand::Request  &req, kortex_driver::srv::SendJointSpeedsJoystickCommand::Response &res)
{
	
	Kinova::Api::Base::JointSpeeds input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SendSelectedJointSpeedJoystickCommand(kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Request  &req, kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Response &res)
{
	
	Kinova::Api::Base::JointSpeed input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::EnableBridge(kortex_driver::srv::EnableBridge::Request  &req, kortex_driver::srv::EnableBridge::Response &res)
{
	
	Kinova::Api::Base::BridgeConfig input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DisableBridge(kortex_driver::srv::DisableBridge::Request  &req, kortex_driver::srv::DisableBridge::Response &res)
{
	
	Kinova::Api::Base::BridgeIdentifier input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetBridgeList(kortex_driver::srv::GetBridgeList::Request  &req, kortex_driver::srv::GetBridgeList::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetBridgeConfig(kortex_driver::srv::GetBridgeConfig::Request  &req, kortex_driver::srv::GetBridgeConfig::Response &res)
{
	
	Kinova::Api::Base::BridgeIdentifier input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::PlayPreComputedJointTrajectory(kortex_driver::srv::PlayPreComputedJointTrajectory::Request  &req, kortex_driver::srv::PlayPreComputedJointTrajectory::Response &res)
{
	
	Kinova::Api::Base::PreComputedJointTrajectory input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetProductConfiguration(kortex_driver::srv::GetProductConfiguration::Request  &req, kortex_driver::srv::GetProductConfiguration::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::UpdateEndEffectorTypeConfiguration(kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Request  &req, kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Response &res)
{
	
	Kinova::Api::ProductConfiguration::ProductConfigurationEndEffectorType input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::RestoreFactoryProductConfiguration(kortex_driver::srv::RestoreFactoryProductConfiguration::Request  &req, kortex_driver::srv::RestoreFactoryProductConfiguration::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetTrajectoryErrorReport(kortex_driver::srv::GetTrajectoryErrorReport::Request  &req, kortex_driver::srv::GetTrajectoryErrorReport::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetAllJointsSpeedSoftLimitation(kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Request  &req, kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_speed_soft_limitation service is now deprecated and will be removed in a future release.");
	
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetAllJointsTorqueSoftLimitation(kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Request  &req, kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_torque_soft_limitation service is now deprecated and will be removed in a future release.");
	
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetTwistSoftLimitation(kortex_driver::srv::GetTwistSoftLimitation::Request  &req, kortex_driver::srv::GetTwistSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_twist_soft_limitation service is now deprecated and will be removed in a future release.");
	
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetWrenchSoftLimitation(kortex_driver::srv::GetWrenchSoftLimitation::Request  &req, kortex_driver::srv::GetWrenchSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_wrench_soft_limitation service is now deprecated and will be removed in a future release.");
	
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SetControllerConfigurationMode(kortex_driver::srv::SetControllerConfigurationMode::Request  &req, kortex_driver::srv::SetControllerConfigurationMode::Response &res)
{
	
	Kinova::Api::Base::ControllerConfigurationMode input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetControllerConfigurationMode(kortex_driver::srv::GetControllerConfigurationMode::Request  &req, kortex_driver::srv::GetControllerConfigurationMode::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::StartTeaching(kortex_driver::srv::StartTeaching::Request  &req, kortex_driver::srv::StartTeaching::Response &res)
{
	
	Kinova::Api::Base::SequenceTaskHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::StopTeaching(kortex_driver::srv::StopTeaching::Request  &req, kortex_driver::srv::StopTeaching::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::AddSequenceTasks(kortex_driver::srv::AddSequenceTasks::Request  &req, kortex_driver::srv::AddSequenceTasks::Response &res)
{
	
	Kinova::Api::Base::SequenceTasksConfiguration input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::UpdateSequenceTask(kortex_driver::srv::UpdateSequenceTask::Request  &req, kortex_driver::srv::UpdateSequenceTask::Response &res)
{
	
	Kinova::Api::Base::SequenceTaskConfiguration input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SwapSequenceTasks(kortex_driver::srv::SwapSequenceTasks::Request  &req, kortex_driver::srv::SwapSequenceTasks::Response &res)
{
	
	Kinova::Api::Base::SequenceTasksPair input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadSequenceTask(kortex_driver::srv::ReadSequenceTask::Request  &req, kortex_driver::srv::ReadSequenceTask::Response &res)
{
	
	Kinova::Api::Base::SequenceTaskHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ReadAllSequenceTasks(kortex_driver::srv::ReadAllSequenceTasks::Request  &req, kortex_driver::srv::ReadAllSequenceTasks::Response &res)
{
	
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DeleteSequenceTask(kortex_driver::srv::DeleteSequenceTask::Request  &req, kortex_driver::srv::DeleteSequenceTask::Response &res)
{
	
	Kinova::Api::Base::SequenceTaskHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DeleteAllSequenceTasks(kortex_driver::srv::DeleteAllSequenceTasks::Request  &req, kortex_driver::srv::DeleteAllSequenceTasks::Response &res)
{
	
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::TakeSnapshot(kortex_driver::srv::TakeSnapshot::Request  &req, kortex_driver::srv::TakeSnapshot::Response &res)
{
	
	Kinova::Api::Base::Snapshot input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetFirmwareBundleVersions(kortex_driver::srv::GetFirmwareBundleVersions::Request  &req, kortex_driver::srv::GetFirmwareBundleVersions::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ExecuteWaypointTrajectory(kortex_driver::srv::ExecuteWaypointTrajectory::Request  &req, kortex_driver::srv::ExecuteWaypointTrajectory::Response &res)
{
	
	Kinova::Api::Base::WaypointList input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::MoveSequenceTask(kortex_driver::srv::MoveSequenceTask::Request  &req, kortex_driver::srv::MoveSequenceTask::Response &res)
{
	
	Kinova::Api::Base::SequenceTasksPair input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DuplicateMapping(kortex_driver::srv::DuplicateMapping::Request  &req, kortex_driver::srv::DuplicateMapping::Response &res)
{
	
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::DuplicateMap(kortex_driver::srv::DuplicateMap::Request  &req, kortex_driver::srv::DuplicateMap::Response &res)
{
	
	Kinova::Api::Base::MapHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::SetControllerConfiguration(kortex_driver::srv::SetControllerConfiguration::Request  &req, kortex_driver::srv::SetControllerConfiguration::Response &res)
{
	
	Kinova::Api::Base::ControllerConfiguration input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetControllerConfiguration(kortex_driver::srv::GetControllerConfiguration::Request  &req, kortex_driver::srv::GetControllerConfiguration::Response &res)
{
	
	Kinova::Api::Base::ControllerHandle input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::GetAllControllerConfigurations(kortex_driver::srv::GetAllControllerConfigurations::Request  &req, kortex_driver::srv::GetAllControllerConfigurations::Response &res)
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ComputeForwardKinematics(kortex_driver::srv::ComputeForwardKinematics::Request  &req, kortex_driver::srv::ComputeForwardKinematics::Response &res)
{
	
	Kinova::Api::Base::JointAngles input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ComputeInverseKinematics(kortex_driver::srv::ComputeInverseKinematics::Request  &req, kortex_driver::srv::ComputeInverseKinematics::Response &res)
{
	
	Kinova::Api::Base::IKData input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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

bool BaseRobotServices::ValidateWaypointList(kortex_driver::srv::ValidateWaypointList::Request  &req, kortex_driver::srv::ValidateWaypointList::Response &res)
{
	
	Kinova::Api::Base::WaypointList input;
	ToProtoData(req.input, &input);
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
		m_pub_Error.publish(result_error);
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
