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
 
#ifndef _KORTEX_BASE_SERVICES_INTERFACE_H_
#define _KORTEX_BASE_SERVICES_INTERFACE_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>
#include "kortex_driver/srv/create_user_profile.hpp"
#include "kortex_driver/srv/update_user_profile.hpp"
#include "kortex_driver/srv/read_user_profile.hpp"
#include "kortex_driver/srv/delete_user_profile.hpp"
#include "kortex_driver/srv/read_all_user_profiles.hpp"
#include "kortex_driver/srv/read_all_users.hpp"
#include "kortex_driver/srv/change_password.hpp"
#include "kortex_driver/srv/create_sequence.hpp"
#include "kortex_driver/srv/update_sequence.hpp"
#include "kortex_driver/srv/read_sequence.hpp"
#include "kortex_driver/srv/delete_sequence.hpp"
#include "kortex_driver/srv/read_all_sequences.hpp"
#include "kortex_driver/srv/play_sequence.hpp"
#include "kortex_driver/srv/play_advanced_sequence.hpp"
#include "kortex_driver/srv/stop_sequence.hpp"
#include "kortex_driver/srv/pause_sequence.hpp"
#include "kortex_driver/srv/resume_sequence.hpp"
#include "kortex_driver/srv/create_protection_zone.hpp"
#include "kortex_driver/srv/update_protection_zone.hpp"
#include "kortex_driver/srv/read_protection_zone.hpp"
#include "kortex_driver/srv/delete_protection_zone.hpp"
#include "kortex_driver/srv/read_all_protection_zones.hpp"
#include "kortex_driver/srv/create_mapping.hpp"
#include "kortex_driver/srv/read_mapping.hpp"
#include "kortex_driver/srv/update_mapping.hpp"
#include "kortex_driver/srv/delete_mapping.hpp"
#include "kortex_driver/srv/read_all_mappings.hpp"
#include "kortex_driver/srv/create_map.hpp"
#include "kortex_driver/srv/read_map.hpp"
#include "kortex_driver/srv/update_map.hpp"
#include "kortex_driver/srv/delete_map.hpp"
#include "kortex_driver/srv/read_all_maps.hpp"
#include "kortex_driver/srv/activate_map.hpp"
#include "kortex_driver/srv/create_action.hpp"
#include "kortex_driver/srv/read_action.hpp"
#include "kortex_driver/srv/read_all_actions.hpp"
#include "kortex_driver/srv/delete_action.hpp"
#include "kortex_driver/srv/update_action.hpp"
#include "kortex_driver/srv/execute_action_from_reference.hpp"
#include "kortex_driver/srv/execute_action.hpp"
#include "kortex_driver/srv/pause_action.hpp"
#include "kortex_driver/srv/stop_action.hpp"
#include "kortex_driver/srv/resume_action.hpp"
#include "kortex_driver/srv/get_i_pv4_configuration.hpp"
#include "kortex_driver/srv/set_i_pv4_configuration.hpp"
#include "kortex_driver/srv/set_communication_interface_enable.hpp"
#include "kortex_driver/srv/is_communication_interface_enable.hpp"
#include "kortex_driver/srv/get_available_wifi.hpp"
#include "kortex_driver/srv/get_wifi_information.hpp"
#include "kortex_driver/srv/add_wifi_configuration.hpp"
#include "kortex_driver/srv/delete_wifi_configuration.hpp"
#include "kortex_driver/srv/get_all_configured_wifis.hpp"
#include "kortex_driver/srv/connect_wifi.hpp"
#include "kortex_driver/srv/disconnect_wifi.hpp"
#include "kortex_driver/srv/get_connected_wifi_information.hpp"
#include "kortex_driver/srv/base_unsubscribe.hpp"
#include "kortex_driver/srv/on_notification_configuration_change_topic.hpp"
#include "kortex_driver/msg/configuration_change_notification.hpp"
#include "kortex_driver/srv/on_notification_mapping_info_topic.hpp"
#include "kortex_driver/msg/mapping_info_notification.hpp"
#include "kortex_driver/srv/base_on_notification_control_mode_topic.hpp"
#include "kortex_driver/msg/base_control_mode_notification.hpp"
#include "kortex_driver/srv/on_notification_operating_mode_topic.hpp"
#include "kortex_driver/msg/operating_mode_notification.hpp"
#include "kortex_driver/srv/on_notification_sequence_info_topic.hpp"
#include "kortex_driver/msg/sequence_info_notification.hpp"
#include "kortex_driver/srv/on_notification_protection_zone_topic.hpp"
#include "kortex_driver/msg/protection_zone_notification.hpp"
#include "kortex_driver/srv/on_notification_user_topic.hpp"
#include "kortex_driver/msg/user_notification.hpp"
#include "kortex_driver/srv/on_notification_controller_topic.hpp"
#include "kortex_driver/msg/controller_notification.hpp"
#include "kortex_driver/srv/on_notification_action_topic.hpp"
#include "kortex_driver/msg/action_notification.hpp"
#include "kortex_driver/srv/on_notification_robot_event_topic.hpp"
#include "kortex_driver/msg/robot_event_notification.hpp"
#include "kortex_driver/srv/play_cartesian_trajectory.hpp"
#include "kortex_driver/srv/play_cartesian_trajectory_position.hpp"
#include "kortex_driver/srv/play_cartesian_trajectory_orientation.hpp"
#include "kortex_driver/srv/stop.hpp"
#include "kortex_driver/srv/get_measured_cartesian_pose.hpp"
#include "kortex_driver/srv/send_wrench_command.hpp"
#include "kortex_driver/srv/send_wrench_joystick_command.hpp"
#include "kortex_driver/srv/send_twist_joystick_command.hpp"
#include "kortex_driver/srv/send_twist_command.hpp"
#include "kortex_driver/srv/play_joint_trajectory.hpp"
#include "kortex_driver/srv/play_selected_joint_trajectory.hpp"
#include "kortex_driver/srv/get_measured_joint_angles.hpp"
#include "kortex_driver/srv/send_joint_speeds_command.hpp"
#include "kortex_driver/srv/send_selected_joint_speed_command.hpp"
#include "kortex_driver/srv/send_gripper_command.hpp"
#include "kortex_driver/srv/get_measured_gripper_movement.hpp"
#include "kortex_driver/srv/set_admittance.hpp"
#include "kortex_driver/srv/set_operating_mode.hpp"
#include "kortex_driver/srv/apply_emergency_stop.hpp"
#include "kortex_driver/srv/base_clear_faults.hpp"
#include "kortex_driver/srv/base_get_control_mode.hpp"
#include "kortex_driver/srv/get_operating_mode.hpp"
#include "kortex_driver/srv/set_servoing_mode.hpp"
#include "kortex_driver/srv/get_servoing_mode.hpp"
#include "kortex_driver/srv/on_notification_servoing_mode_topic.hpp"
#include "kortex_driver/msg/servoing_mode_notification.hpp"
#include "kortex_driver/srv/restore_factory_settings.hpp"
#include "kortex_driver/srv/on_notification_factory_topic.hpp"
#include "kortex_driver/msg/factory_notification.hpp"
#include "kortex_driver/srv/get_all_connected_controllers.hpp"
#include "kortex_driver/srv/get_controller_state.hpp"
#include "kortex_driver/srv/get_actuator_count.hpp"
#include "kortex_driver/srv/start_wifi_scan.hpp"
#include "kortex_driver/srv/get_configured_wifi.hpp"
#include "kortex_driver/srv/on_notification_network_topic.hpp"
#include "kortex_driver/msg/network_notification.hpp"
#include "kortex_driver/srv/get_arm_state.hpp"
#include "kortex_driver/srv/on_notification_arm_state_topic.hpp"
#include "kortex_driver/msg/arm_state_notification.hpp"
#include "kortex_driver/srv/get_i_pv4_information.hpp"
#include "kortex_driver/srv/set_wifi_country_code.hpp"
#include "kortex_driver/srv/get_wifi_country_code.hpp"
#include "kortex_driver/srv/base_set_cap_sense_config.hpp"
#include "kortex_driver/srv/base_get_cap_sense_config.hpp"
#include "kortex_driver/srv/get_all_joints_speed_hard_limitation.hpp"
#include "kortex_driver/srv/get_all_joints_torque_hard_limitation.hpp"
#include "kortex_driver/srv/get_twist_hard_limitation.hpp"
#include "kortex_driver/srv/get_wrench_hard_limitation.hpp"
#include "kortex_driver/srv/send_joint_speeds_joystick_command.hpp"
#include "kortex_driver/srv/send_selected_joint_speed_joystick_command.hpp"
#include "kortex_driver/srv/enable_bridge.hpp"
#include "kortex_driver/srv/disable_bridge.hpp"
#include "kortex_driver/srv/get_bridge_list.hpp"
#include "kortex_driver/srv/get_bridge_config.hpp"
#include "kortex_driver/srv/play_pre_computed_joint_trajectory.hpp"
#include "kortex_driver/srv/get_product_configuration.hpp"
#include "kortex_driver/srv/update_end_effector_type_configuration.hpp"
#include "kortex_driver/srv/restore_factory_product_configuration.hpp"
#include "kortex_driver/srv/get_trajectory_error_report.hpp"
#include "kortex_driver/srv/get_all_joints_speed_soft_limitation.hpp"
#include "kortex_driver/srv/get_all_joints_torque_soft_limitation.hpp"
#include "kortex_driver/srv/get_twist_soft_limitation.hpp"
#include "kortex_driver/srv/get_wrench_soft_limitation.hpp"
#include "kortex_driver/srv/set_controller_configuration_mode.hpp"
#include "kortex_driver/srv/get_controller_configuration_mode.hpp"
#include "kortex_driver/srv/start_teaching.hpp"
#include "kortex_driver/srv/stop_teaching.hpp"
#include "kortex_driver/srv/add_sequence_tasks.hpp"
#include "kortex_driver/srv/update_sequence_task.hpp"
#include "kortex_driver/srv/swap_sequence_tasks.hpp"
#include "kortex_driver/srv/read_sequence_task.hpp"
#include "kortex_driver/srv/read_all_sequence_tasks.hpp"
#include "kortex_driver/srv/delete_sequence_task.hpp"
#include "kortex_driver/srv/delete_all_sequence_tasks.hpp"
#include "kortex_driver/srv/take_snapshot.hpp"
#include "kortex_driver/srv/get_firmware_bundle_versions.hpp"
#include "kortex_driver/srv/execute_waypoint_trajectory.hpp"
#include "kortex_driver/srv/move_sequence_task.hpp"
#include "kortex_driver/srv/duplicate_mapping.hpp"
#include "kortex_driver/srv/duplicate_map.hpp"
#include "kortex_driver/srv/set_controller_configuration.hpp"
#include "kortex_driver/srv/get_controller_configuration.hpp"
#include "kortex_driver/srv/get_all_controller_configurations.hpp"
#include "kortex_driver/srv/compute_forward_kinematics.hpp"
#include "kortex_driver/srv/compute_inverse_kinematics.hpp"
#include "kortex_driver/srv/validate_waypoint_list.hpp"

#include "kortex_driver/msg/kortex_error.hpp"
#include "kortex_driver/srv/set_device_id.hpp"
#include "kortex_driver/srv/set_api_options.hpp"
#include "kortex_driver/msg/api_options.hpp"

using namespace std;

class IBaseServices
{
    public:
        IBaseServices(ros::NodeHandle& node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) = 0;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) = 0;
        virtual bool CreateUserProfile(kortex_driver::srv::CreateUserProfile::Request  &req, kortex_driver::srv::CreateUserProfile::Response &res) = 0;
        virtual bool UpdateUserProfile(kortex_driver::srv::UpdateUserProfile::Request  &req, kortex_driver::srv::UpdateUserProfile::Response &res) = 0;
        virtual bool ReadUserProfile(kortex_driver::srv::ReadUserProfile::Request  &req, kortex_driver::srv::ReadUserProfile::Response &res) = 0;
        virtual bool DeleteUserProfile(kortex_driver::srv::DeleteUserProfile::Request  &req, kortex_driver::srv::DeleteUserProfile::Response &res) = 0;
        virtual bool ReadAllUserProfiles(kortex_driver::srv::ReadAllUserProfiles::Request  &req, kortex_driver::srv::ReadAllUserProfiles::Response &res) = 0;
        virtual bool ReadAllUsers(kortex_driver::srv::ReadAllUsers::Request  &req, kortex_driver::srv::ReadAllUsers::Response &res) = 0;
        virtual bool ChangePassword(kortex_driver::srv::ChangePassword::Request  &req, kortex_driver::srv::ChangePassword::Response &res) = 0;
        virtual bool CreateSequence(kortex_driver::srv::CreateSequence::Request  &req, kortex_driver::srv::CreateSequence::Response &res) = 0;
        virtual bool UpdateSequence(kortex_driver::srv::UpdateSequence::Request  &req, kortex_driver::srv::UpdateSequence::Response &res) = 0;
        virtual bool ReadSequence(kortex_driver::srv::ReadSequence::Request  &req, kortex_driver::srv::ReadSequence::Response &res) = 0;
        virtual bool DeleteSequence(kortex_driver::srv::DeleteSequence::Request  &req, kortex_driver::srv::DeleteSequence::Response &res) = 0;
        virtual bool ReadAllSequences(kortex_driver::srv::ReadAllSequences::Request  &req, kortex_driver::srv::ReadAllSequences::Response &res) = 0;
        virtual bool PlaySequence(kortex_driver::srv::PlaySequence::Request  &req, kortex_driver::srv::PlaySequence::Response &res) = 0;
        virtual bool PlayAdvancedSequence(kortex_driver::srv::PlayAdvancedSequence::Request  &req, kortex_driver::srv::PlayAdvancedSequence::Response &res) = 0;
        virtual bool StopSequence(kortex_driver::srv::StopSequence::Request  &req, kortex_driver::srv::StopSequence::Response &res) = 0;
        virtual bool PauseSequence(kortex_driver::srv::PauseSequence::Request  &req, kortex_driver::srv::PauseSequence::Response &res) = 0;
        virtual bool ResumeSequence(kortex_driver::srv::ResumeSequence::Request  &req, kortex_driver::srv::ResumeSequence::Response &res) = 0;
        virtual bool CreateProtectionZone(kortex_driver::srv::CreateProtectionZone::Request  &req, kortex_driver::srv::CreateProtectionZone::Response &res) = 0;
        virtual bool UpdateProtectionZone(kortex_driver::srv::UpdateProtectionZone::Request  &req, kortex_driver::srv::UpdateProtectionZone::Response &res) = 0;
        virtual bool ReadProtectionZone(kortex_driver::srv::ReadProtectionZone::Request  &req, kortex_driver::srv::ReadProtectionZone::Response &res) = 0;
        virtual bool DeleteProtectionZone(kortex_driver::srv::DeleteProtectionZone::Request  &req, kortex_driver::srv::DeleteProtectionZone::Response &res) = 0;
        virtual bool ReadAllProtectionZones(kortex_driver::srv::ReadAllProtectionZones::Request  &req, kortex_driver::srv::ReadAllProtectionZones::Response &res) = 0;
        virtual bool CreateMapping(kortex_driver::srv::CreateMapping::Request  &req, kortex_driver::srv::CreateMapping::Response &res) = 0;
        virtual bool ReadMapping(kortex_driver::srv::ReadMapping::Request  &req, kortex_driver::srv::ReadMapping::Response &res) = 0;
        virtual bool UpdateMapping(kortex_driver::srv::UpdateMapping::Request  &req, kortex_driver::srv::UpdateMapping::Response &res) = 0;
        virtual bool DeleteMapping(kortex_driver::srv::DeleteMapping::Request  &req, kortex_driver::srv::DeleteMapping::Response &res) = 0;
        virtual bool ReadAllMappings(kortex_driver::srv::ReadAllMappings::Request  &req, kortex_driver::srv::ReadAllMappings::Response &res) = 0;
        virtual bool CreateMap(kortex_driver::srv::CreateMap::Request  &req, kortex_driver::srv::CreateMap::Response &res) = 0;
        virtual bool ReadMap(kortex_driver::srv::ReadMap::Request  &req, kortex_driver::srv::ReadMap::Response &res) = 0;
        virtual bool UpdateMap(kortex_driver::srv::UpdateMap::Request  &req, kortex_driver::srv::UpdateMap::Response &res) = 0;
        virtual bool DeleteMap(kortex_driver::srv::DeleteMap::Request  &req, kortex_driver::srv::DeleteMap::Response &res) = 0;
        virtual bool ReadAllMaps(kortex_driver::srv::ReadAllMaps::Request  &req, kortex_driver::srv::ReadAllMaps::Response &res) = 0;
        virtual bool ActivateMap(kortex_driver::srv::ActivateMap::Request  &req, kortex_driver::srv::ActivateMap::Response &res) = 0;
        virtual bool CreateAction(kortex_driver::srv::CreateAction::Request  &req, kortex_driver::srv::CreateAction::Response &res) = 0;
        virtual bool ReadAction(kortex_driver::srv::ReadAction::Request  &req, kortex_driver::srv::ReadAction::Response &res) = 0;
        virtual bool ReadAllActions(kortex_driver::srv::ReadAllActions::Request  &req, kortex_driver::srv::ReadAllActions::Response &res) = 0;
        virtual bool DeleteAction(kortex_driver::srv::DeleteAction::Request  &req, kortex_driver::srv::DeleteAction::Response &res) = 0;
        virtual bool UpdateAction(kortex_driver::srv::UpdateAction::Request  &req, kortex_driver::srv::UpdateAction::Response &res) = 0;
        virtual bool ExecuteActionFromReference(kortex_driver::srv::ExecuteActionFromReference::Request  &req, kortex_driver::srv::ExecuteActionFromReference::Response &res) = 0;
        virtual bool ExecuteAction(kortex_driver::srv::ExecuteAction::Request  &req, kortex_driver::srv::ExecuteAction::Response &res) = 0;
        virtual bool PauseAction(kortex_driver::srv::PauseAction::Request  &req, kortex_driver::srv::PauseAction::Response &res) = 0;
        virtual bool StopAction(kortex_driver::srv::StopAction::Request  &req, kortex_driver::srv::StopAction::Response &res) = 0;
        virtual bool ResumeAction(kortex_driver::srv::ResumeAction::Request  &req, kortex_driver::srv::ResumeAction::Response &res) = 0;
        virtual bool GetIPv4Configuration(kortex_driver::srv::GetIPv4Configuration::Request  &req, kortex_driver::srv::GetIPv4Configuration::Response &res) = 0;
        virtual bool SetIPv4Configuration(kortex_driver::srv::SetIPv4Configuration::Request  &req, kortex_driver::srv::SetIPv4Configuration::Response &res) = 0;
        virtual bool SetCommunicationInterfaceEnable(kortex_driver::srv::SetCommunicationInterfaceEnable::Request  &req, kortex_driver::srv::SetCommunicationInterfaceEnable::Response &res) = 0;
        virtual bool IsCommunicationInterfaceEnable(kortex_driver::srv::IsCommunicationInterfaceEnable::Request  &req, kortex_driver::srv::IsCommunicationInterfaceEnable::Response &res) = 0;
        virtual bool GetAvailableWifi(kortex_driver::srv::GetAvailableWifi::Request  &req, kortex_driver::srv::GetAvailableWifi::Response &res) = 0;
        virtual bool GetWifiInformation(kortex_driver::srv::GetWifiInformation::Request  &req, kortex_driver::srv::GetWifiInformation::Response &res) = 0;
        virtual bool AddWifiConfiguration(kortex_driver::srv::AddWifiConfiguration::Request  &req, kortex_driver::srv::AddWifiConfiguration::Response &res) = 0;
        virtual bool DeleteWifiConfiguration(kortex_driver::srv::DeleteWifiConfiguration::Request  &req, kortex_driver::srv::DeleteWifiConfiguration::Response &res) = 0;
        virtual bool GetAllConfiguredWifis(kortex_driver::srv::GetAllConfiguredWifis::Request  &req, kortex_driver::srv::GetAllConfiguredWifis::Response &res) = 0;
        virtual bool ConnectWifi(kortex_driver::srv::ConnectWifi::Request  &req, kortex_driver::srv::ConnectWifi::Response &res) = 0;
        virtual bool DisconnectWifi(kortex_driver::srv::DisconnectWifi::Request  &req, kortex_driver::srv::DisconnectWifi::Response &res) = 0;
        virtual bool GetConnectedWifiInformation(kortex_driver::srv::GetConnectedWifiInformation::Request  &req, kortex_driver::srv::GetConnectedWifiInformation::Response &res) = 0;
        virtual bool Base_Unsubscribe(kortex_driver::srv::BaseUnsubscribe::Request  &req, kortex_driver::srv::BaseUnsubscribe::Response &res) = 0;
        virtual bool OnNotificationConfigurationChangeTopic(kortex_driver::srv::OnNotificationConfigurationChangeTopic::Request  &req, kortex_driver::srv::OnNotificationConfigurationChangeTopic::Response &res) = 0;
        virtual void cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif) = 0;
        virtual bool OnNotificationMappingInfoTopic(kortex_driver::srv::OnNotificationMappingInfoTopic::Request  &req, kortex_driver::srv::OnNotificationMappingInfoTopic::Response &res) = 0;
        virtual void cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif) = 0;
        virtual bool Base_OnNotificationControlModeTopic(kortex_driver::srv::BaseOnNotificationControlModeTopic::Request  &req, kortex_driver::srv::BaseOnNotificationControlModeTopic::Response &res) = 0;
        virtual void cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif) = 0;
        virtual bool OnNotificationOperatingModeTopic(kortex_driver::srv::OnNotificationOperatingModeTopic::Request  &req, kortex_driver::srv::OnNotificationOperatingModeTopic::Response &res) = 0;
        virtual void cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif) = 0;
        virtual bool OnNotificationSequenceInfoTopic(kortex_driver::srv::OnNotificationSequenceInfoTopic::Request  &req, kortex_driver::srv::OnNotificationSequenceInfoTopic::Response &res) = 0;
        virtual void cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif) = 0;
        virtual bool OnNotificationProtectionZoneTopic(kortex_driver::srv::OnNotificationProtectionZoneTopic::Request  &req, kortex_driver::srv::OnNotificationProtectionZoneTopic::Response &res) = 0;
        virtual void cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif) = 0;
        virtual bool OnNotificationUserTopic(kortex_driver::srv::OnNotificationUserTopic::Request  &req, kortex_driver::srv::OnNotificationUserTopic::Response &res) = 0;
        virtual void cb_UserTopic(Kinova::Api::Base::UserNotification notif) = 0;
        virtual bool OnNotificationControllerTopic(kortex_driver::srv::OnNotificationControllerTopic::Request  &req, kortex_driver::srv::OnNotificationControllerTopic::Response &res) = 0;
        virtual void cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif) = 0;
        virtual bool OnNotificationActionTopic(kortex_driver::srv::OnNotificationActionTopic::Request  &req, kortex_driver::srv::OnNotificationActionTopic::Response &res) = 0;
        virtual void cb_ActionTopic(Kinova::Api::Base::ActionNotification notif) = 0;
        virtual bool OnNotificationRobotEventTopic(kortex_driver::srv::OnNotificationRobotEventTopic::Request  &req, kortex_driver::srv::OnNotificationRobotEventTopic::Response &res) = 0;
        virtual void cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif) = 0;
        virtual bool PlayCartesianTrajectory(kortex_driver::srv::PlayCartesianTrajectory::Request  &req, kortex_driver::srv::PlayCartesianTrajectory::Response &res) = 0;
        virtual bool PlayCartesianTrajectoryPosition(kortex_driver::srv::PlayCartesianTrajectoryPosition::Request  &req, kortex_driver::srv::PlayCartesianTrajectoryPosition::Response &res) = 0;
        virtual bool PlayCartesianTrajectoryOrientation(kortex_driver::srv::PlayCartesianTrajectoryOrientation::Request  &req, kortex_driver::srv::PlayCartesianTrajectoryOrientation::Response &res) = 0;
        virtual bool Stop(kortex_driver::srv::Stop::Request  &req, kortex_driver::srv::Stop::Response &res) = 0;
        virtual bool GetMeasuredCartesianPose(kortex_driver::srv::GetMeasuredCartesianPose::Request  &req, kortex_driver::srv::GetMeasuredCartesianPose::Response &res) = 0;
        virtual bool SendWrenchCommand(kortex_driver::srv::SendWrenchCommand::Request  &req, kortex_driver::srv::SendWrenchCommand::Response &res) = 0;
        virtual bool SendWrenchJoystickCommand(kortex_driver::srv::SendWrenchJoystickCommand::Request  &req, kortex_driver::srv::SendWrenchJoystickCommand::Response &res) = 0;
        virtual bool SendTwistJoystickCommand(kortex_driver::srv::SendTwistJoystickCommand::Request  &req, kortex_driver::srv::SendTwistJoystickCommand::Response &res) = 0;
        virtual bool SendTwistCommand(kortex_driver::srv::SendTwistCommand::Request  &req, kortex_driver::srv::SendTwistCommand::Response &res) = 0;
        virtual bool PlayJointTrajectory(kortex_driver::srv::PlayJointTrajectory::Request  &req, kortex_driver::srv::PlayJointTrajectory::Response &res) = 0;
        virtual bool PlaySelectedJointTrajectory(kortex_driver::srv::PlaySelectedJointTrajectory::Request  &req, kortex_driver::srv::PlaySelectedJointTrajectory::Response &res) = 0;
        virtual bool GetMeasuredJointAngles(kortex_driver::srv::GetMeasuredJointAngles::Request  &req, kortex_driver::srv::GetMeasuredJointAngles::Response &res) = 0;
        virtual bool SendJointSpeedsCommand(kortex_driver::srv::SendJointSpeedsCommand::Request  &req, kortex_driver::srv::SendJointSpeedsCommand::Response &res) = 0;
        virtual bool SendSelectedJointSpeedCommand(kortex_driver::srv::SendSelectedJointSpeedCommand::Request  &req, kortex_driver::srv::SendSelectedJointSpeedCommand::Response &res) = 0;
        virtual bool SendGripperCommand(kortex_driver::srv::SendGripperCommand::Request  &req, kortex_driver::srv::SendGripperCommand::Response &res) = 0;
        virtual bool GetMeasuredGripperMovement(kortex_driver::srv::GetMeasuredGripperMovement::Request  &req, kortex_driver::srv::GetMeasuredGripperMovement::Response &res) = 0;
        virtual bool SetAdmittance(kortex_driver::srv::SetAdmittance::Request  &req, kortex_driver::srv::SetAdmittance::Response &res) = 0;
        virtual bool SetOperatingMode(kortex_driver::srv::SetOperatingMode::Request  &req, kortex_driver::srv::SetOperatingMode::Response &res) = 0;
        virtual bool ApplyEmergencyStop(kortex_driver::srv::ApplyEmergencyStop::Request  &req, kortex_driver::srv::ApplyEmergencyStop::Response &res) = 0;
        virtual bool Base_ClearFaults(kortex_driver::srv::BaseClearFaults::Request  &req, kortex_driver::srv::BaseClearFaults::Response &res) = 0;
        virtual bool Base_GetControlMode(kortex_driver::srv::BaseGetControlMode::Request  &req, kortex_driver::srv::BaseGetControlMode::Response &res) = 0;
        virtual bool GetOperatingMode(kortex_driver::srv::GetOperatingMode::Request  &req, kortex_driver::srv::GetOperatingMode::Response &res) = 0;
        virtual bool SetServoingMode(kortex_driver::srv::SetServoingMode::Request  &req, kortex_driver::srv::SetServoingMode::Response &res) = 0;
        virtual bool GetServoingMode(kortex_driver::srv::GetServoingMode::Request  &req, kortex_driver::srv::GetServoingMode::Response &res) = 0;
        virtual bool OnNotificationServoingModeTopic(kortex_driver::srv::OnNotificationServoingModeTopic::Request  &req, kortex_driver::srv::OnNotificationServoingModeTopic::Response &res) = 0;
        virtual void cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif) = 0;
        virtual bool RestoreFactorySettings(kortex_driver::srv::RestoreFactorySettings::Request  &req, kortex_driver::srv::RestoreFactorySettings::Response &res) = 0;
        virtual bool OnNotificationFactoryTopic(kortex_driver::srv::OnNotificationFactoryTopic::Request  &req, kortex_driver::srv::OnNotificationFactoryTopic::Response &res) = 0;
        virtual void cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif) = 0;
        virtual bool GetAllConnectedControllers(kortex_driver::srv::GetAllConnectedControllers::Request  &req, kortex_driver::srv::GetAllConnectedControllers::Response &res) = 0;
        virtual bool GetControllerState(kortex_driver::srv::GetControllerState::Request  &req, kortex_driver::srv::GetControllerState::Response &res) = 0;
        virtual bool GetActuatorCount(kortex_driver::srv::GetActuatorCount::Request  &req, kortex_driver::srv::GetActuatorCount::Response &res) = 0;
        virtual bool StartWifiScan(kortex_driver::srv::StartWifiScan::Request  &req, kortex_driver::srv::StartWifiScan::Response &res) = 0;
        virtual bool GetConfiguredWifi(kortex_driver::srv::GetConfiguredWifi::Request  &req, kortex_driver::srv::GetConfiguredWifi::Response &res) = 0;
        virtual bool OnNotificationNetworkTopic(kortex_driver::srv::OnNotificationNetworkTopic::Request  &req, kortex_driver::srv::OnNotificationNetworkTopic::Response &res) = 0;
        virtual void cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif) = 0;
        virtual bool GetArmState(kortex_driver::srv::GetArmState::Request  &req, kortex_driver::srv::GetArmState::Response &res) = 0;
        virtual bool OnNotificationArmStateTopic(kortex_driver::srv::OnNotificationArmStateTopic::Request  &req, kortex_driver::srv::OnNotificationArmStateTopic::Response &res) = 0;
        virtual void cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif) = 0;
        virtual bool GetIPv4Information(kortex_driver::srv::GetIPv4Information::Request  &req, kortex_driver::srv::GetIPv4Information::Response &res) = 0;
        virtual bool SetWifiCountryCode(kortex_driver::srv::SetWifiCountryCode::Request  &req, kortex_driver::srv::SetWifiCountryCode::Response &res) = 0;
        virtual bool GetWifiCountryCode(kortex_driver::srv::GetWifiCountryCode::Request  &req, kortex_driver::srv::GetWifiCountryCode::Response &res) = 0;
        virtual bool Base_SetCapSenseConfig(kortex_driver::srv::BaseSetCapSenseConfig::Request  &req, kortex_driver::srv::BaseSetCapSenseConfig::Response &res) = 0;
        virtual bool Base_GetCapSenseConfig(kortex_driver::srv::BaseGetCapSenseConfig::Request  &req, kortex_driver::srv::BaseGetCapSenseConfig::Response &res) = 0;
        virtual bool GetAllJointsSpeedHardLimitation(kortex_driver::srv::GetAllJointsSpeedHardLimitation::Request  &req, kortex_driver::srv::GetAllJointsSpeedHardLimitation::Response &res) = 0;
        virtual bool GetAllJointsTorqueHardLimitation(kortex_driver::srv::GetAllJointsTorqueHardLimitation::Request  &req, kortex_driver::srv::GetAllJointsTorqueHardLimitation::Response &res) = 0;
        virtual bool GetTwistHardLimitation(kortex_driver::srv::GetTwistHardLimitation::Request  &req, kortex_driver::srv::GetTwistHardLimitation::Response &res) = 0;
        virtual bool GetWrenchHardLimitation(kortex_driver::srv::GetWrenchHardLimitation::Request  &req, kortex_driver::srv::GetWrenchHardLimitation::Response &res) = 0;
        virtual bool SendJointSpeedsJoystickCommand(kortex_driver::srv::SendJointSpeedsJoystickCommand::Request  &req, kortex_driver::srv::SendJointSpeedsJoystickCommand::Response &res) = 0;
        virtual bool SendSelectedJointSpeedJoystickCommand(kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Request  &req, kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Response &res) = 0;
        virtual bool EnableBridge(kortex_driver::srv::EnableBridge::Request  &req, kortex_driver::srv::EnableBridge::Response &res) = 0;
        virtual bool DisableBridge(kortex_driver::srv::DisableBridge::Request  &req, kortex_driver::srv::DisableBridge::Response &res) = 0;
        virtual bool GetBridgeList(kortex_driver::srv::GetBridgeList::Request  &req, kortex_driver::srv::GetBridgeList::Response &res) = 0;
        virtual bool GetBridgeConfig(kortex_driver::srv::GetBridgeConfig::Request  &req, kortex_driver::srv::GetBridgeConfig::Response &res) = 0;
        virtual bool PlayPreComputedJointTrajectory(kortex_driver::srv::PlayPreComputedJointTrajectory::Request  &req, kortex_driver::srv::PlayPreComputedJointTrajectory::Response &res) = 0;
        virtual bool GetProductConfiguration(kortex_driver::srv::GetProductConfiguration::Request  &req, kortex_driver::srv::GetProductConfiguration::Response &res) = 0;
        virtual bool UpdateEndEffectorTypeConfiguration(kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Request  &req, kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Response &res) = 0;
        virtual bool RestoreFactoryProductConfiguration(kortex_driver::srv::RestoreFactoryProductConfiguration::Request  &req, kortex_driver::srv::RestoreFactoryProductConfiguration::Response &res) = 0;
        virtual bool GetTrajectoryErrorReport(kortex_driver::srv::GetTrajectoryErrorReport::Request  &req, kortex_driver::srv::GetTrajectoryErrorReport::Response &res) = 0;
        virtual bool GetAllJointsSpeedSoftLimitation(kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Request  &req, kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Response &res) = 0;
        virtual bool GetAllJointsTorqueSoftLimitation(kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Request  &req, kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Response &res) = 0;
        virtual bool GetTwistSoftLimitation(kortex_driver::srv::GetTwistSoftLimitation::Request  &req, kortex_driver::srv::GetTwistSoftLimitation::Response &res) = 0;
        virtual bool GetWrenchSoftLimitation(kortex_driver::srv::GetWrenchSoftLimitation::Request  &req, kortex_driver::srv::GetWrenchSoftLimitation::Response &res) = 0;
        virtual bool SetControllerConfigurationMode(kortex_driver::srv::SetControllerConfigurationMode::Request  &req, kortex_driver::srv::SetControllerConfigurationMode::Response &res) = 0;
        virtual bool GetControllerConfigurationMode(kortex_driver::srv::GetControllerConfigurationMode::Request  &req, kortex_driver::srv::GetControllerConfigurationMode::Response &res) = 0;
        virtual bool StartTeaching(kortex_driver::srv::StartTeaching::Request  &req, kortex_driver::srv::StartTeaching::Response &res) = 0;
        virtual bool StopTeaching(kortex_driver::srv::StopTeaching::Request  &req, kortex_driver::srv::StopTeaching::Response &res) = 0;
        virtual bool AddSequenceTasks(kortex_driver::srv::AddSequenceTasks::Request  &req, kortex_driver::srv::AddSequenceTasks::Response &res) = 0;
        virtual bool UpdateSequenceTask(kortex_driver::srv::UpdateSequenceTask::Request  &req, kortex_driver::srv::UpdateSequenceTask::Response &res) = 0;
        virtual bool SwapSequenceTasks(kortex_driver::srv::SwapSequenceTasks::Request  &req, kortex_driver::srv::SwapSequenceTasks::Response &res) = 0;
        virtual bool ReadSequenceTask(kortex_driver::srv::ReadSequenceTask::Request  &req, kortex_driver::srv::ReadSequenceTask::Response &res) = 0;
        virtual bool ReadAllSequenceTasks(kortex_driver::srv::ReadAllSequenceTasks::Request  &req, kortex_driver::srv::ReadAllSequenceTasks::Response &res) = 0;
        virtual bool DeleteSequenceTask(kortex_driver::srv::DeleteSequenceTask::Request  &req, kortex_driver::srv::DeleteSequenceTask::Response &res) = 0;
        virtual bool DeleteAllSequenceTasks(kortex_driver::srv::DeleteAllSequenceTasks::Request  &req, kortex_driver::srv::DeleteAllSequenceTasks::Response &res) = 0;
        virtual bool TakeSnapshot(kortex_driver::srv::TakeSnapshot::Request  &req, kortex_driver::srv::TakeSnapshot::Response &res) = 0;
        virtual bool GetFirmwareBundleVersions(kortex_driver::srv::GetFirmwareBundleVersions::Request  &req, kortex_driver::srv::GetFirmwareBundleVersions::Response &res) = 0;
        virtual bool ExecuteWaypointTrajectory(kortex_driver::srv::ExecuteWaypointTrajectory::Request  &req, kortex_driver::srv::ExecuteWaypointTrajectory::Response &res) = 0;
        virtual bool MoveSequenceTask(kortex_driver::srv::MoveSequenceTask::Request  &req, kortex_driver::srv::MoveSequenceTask::Response &res) = 0;
        virtual bool DuplicateMapping(kortex_driver::srv::DuplicateMapping::Request  &req, kortex_driver::srv::DuplicateMapping::Response &res) = 0;
        virtual bool DuplicateMap(kortex_driver::srv::DuplicateMap::Request  &req, kortex_driver::srv::DuplicateMap::Response &res) = 0;
        virtual bool SetControllerConfiguration(kortex_driver::srv::SetControllerConfiguration::Request  &req, kortex_driver::srv::SetControllerConfiguration::Response &res) = 0;
        virtual bool GetControllerConfiguration(kortex_driver::srv::GetControllerConfiguration::Request  &req, kortex_driver::srv::GetControllerConfiguration::Response &res) = 0;
        virtual bool GetAllControllerConfigurations(kortex_driver::srv::GetAllControllerConfigurations::Request  &req, kortex_driver::srv::GetAllControllerConfigurations::Response &res) = 0;
        virtual bool ComputeForwardKinematics(kortex_driver::srv::ComputeForwardKinematics::Request  &req, kortex_driver::srv::ComputeForwardKinematics::Response &res) = 0;
        virtual bool ComputeInverseKinematics(kortex_driver::srv::ComputeInverseKinematics::Request  &req, kortex_driver::srv::ComputeInverseKinematics::Response &res) = 0;
        virtual bool ValidateWaypointList(kortex_driver::srv::ValidateWaypointList::Request  &req, kortex_driver::srv::ValidateWaypointList::Response &res) = 0;

protected:
        ros::NodeHandle m_node_handle;
        ros::Publisher m_pub_Error;
        ros::Publisher m_pub_ConfigurationChangeTopic;
        bool m_is_activated_ConfigurationChangeTopic;
        ros::Publisher m_pub_MappingInfoTopic;
        bool m_is_activated_MappingInfoTopic;
        ros::Publisher m_pub_ControlModeTopic;
        bool m_is_activated_ControlModeTopic;
        ros::Publisher m_pub_OperatingModeTopic;
        bool m_is_activated_OperatingModeTopic;
        ros::Publisher m_pub_SequenceInfoTopic;
        bool m_is_activated_SequenceInfoTopic;
        ros::Publisher m_pub_ProtectionZoneTopic;
        bool m_is_activated_ProtectionZoneTopic;
        ros::Publisher m_pub_UserTopic;
        bool m_is_activated_UserTopic;
        ros::Publisher m_pub_ControllerTopic;
        bool m_is_activated_ControllerTopic;
        ros::Publisher m_pub_ActionTopic;
        bool m_is_activated_ActionTopic;
        ros::Publisher m_pub_RobotEventTopic;
        bool m_is_activated_RobotEventTopic;
        ros::Publisher m_pub_ServoingModeTopic;
        bool m_is_activated_ServoingModeTopic;
        ros::Publisher m_pub_FactoryTopic;
        bool m_is_activated_FactoryTopic;
        ros::Publisher m_pub_NetworkTopic;
        bool m_is_activated_NetworkTopic;
        ros::Publisher m_pub_ArmStateTopic;
        bool m_is_activated_ArmStateTopic;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceCreateUserProfile;
	ros::ServiceServer m_serviceUpdateUserProfile;
	ros::ServiceServer m_serviceReadUserProfile;
	ros::ServiceServer m_serviceDeleteUserProfile;
	ros::ServiceServer m_serviceReadAllUserProfiles;
	ros::ServiceServer m_serviceReadAllUsers;
	ros::ServiceServer m_serviceChangePassword;
	ros::ServiceServer m_serviceCreateSequence;
	ros::ServiceServer m_serviceUpdateSequence;
	ros::ServiceServer m_serviceReadSequence;
	ros::ServiceServer m_serviceDeleteSequence;
	ros::ServiceServer m_serviceReadAllSequences;
	ros::ServiceServer m_servicePlaySequence;
	ros::ServiceServer m_servicePlayAdvancedSequence;
	ros::ServiceServer m_serviceStopSequence;
	ros::ServiceServer m_servicePauseSequence;
	ros::ServiceServer m_serviceResumeSequence;
	ros::ServiceServer m_serviceCreateProtectionZone;
	ros::ServiceServer m_serviceUpdateProtectionZone;
	ros::ServiceServer m_serviceReadProtectionZone;
	ros::ServiceServer m_serviceDeleteProtectionZone;
	ros::ServiceServer m_serviceReadAllProtectionZones;
	ros::ServiceServer m_serviceCreateMapping;
	ros::ServiceServer m_serviceReadMapping;
	ros::ServiceServer m_serviceUpdateMapping;
	ros::ServiceServer m_serviceDeleteMapping;
	ros::ServiceServer m_serviceReadAllMappings;
	ros::ServiceServer m_serviceCreateMap;
	ros::ServiceServer m_serviceReadMap;
	ros::ServiceServer m_serviceUpdateMap;
	ros::ServiceServer m_serviceDeleteMap;
	ros::ServiceServer m_serviceReadAllMaps;
	ros::ServiceServer m_serviceActivateMap;
	ros::ServiceServer m_serviceCreateAction;
	ros::ServiceServer m_serviceReadAction;
	ros::ServiceServer m_serviceReadAllActions;
	ros::ServiceServer m_serviceDeleteAction;
	ros::ServiceServer m_serviceUpdateAction;
	ros::ServiceServer m_serviceExecuteActionFromReference;
	ros::ServiceServer m_serviceExecuteAction;
	ros::ServiceServer m_servicePauseAction;
	ros::ServiceServer m_serviceStopAction;
	ros::ServiceServer m_serviceResumeAction;
	ros::ServiceServer m_serviceGetIPv4Configuration;
	ros::ServiceServer m_serviceSetIPv4Configuration;
	ros::ServiceServer m_serviceSetCommunicationInterfaceEnable;
	ros::ServiceServer m_serviceIsCommunicationInterfaceEnable;
	ros::ServiceServer m_serviceGetAvailableWifi;
	ros::ServiceServer m_serviceGetWifiInformation;
	ros::ServiceServer m_serviceAddWifiConfiguration;
	ros::ServiceServer m_serviceDeleteWifiConfiguration;
	ros::ServiceServer m_serviceGetAllConfiguredWifis;
	ros::ServiceServer m_serviceConnectWifi;
	ros::ServiceServer m_serviceDisconnectWifi;
	ros::ServiceServer m_serviceGetConnectedWifiInformation;
	ros::ServiceServer m_serviceBase_Unsubscribe;
	ros::ServiceServer m_serviceOnNotificationConfigurationChangeTopic;
	ros::ServiceServer m_serviceOnNotificationMappingInfoTopic;
	ros::ServiceServer m_serviceBase_OnNotificationControlModeTopic;
	ros::ServiceServer m_serviceOnNotificationOperatingModeTopic;
	ros::ServiceServer m_serviceOnNotificationSequenceInfoTopic;
	ros::ServiceServer m_serviceOnNotificationProtectionZoneTopic;
	ros::ServiceServer m_serviceOnNotificationUserTopic;
	ros::ServiceServer m_serviceOnNotificationControllerTopic;
	ros::ServiceServer m_serviceOnNotificationActionTopic;
	ros::ServiceServer m_serviceOnNotificationRobotEventTopic;
	ros::ServiceServer m_servicePlayCartesianTrajectory;
	ros::ServiceServer m_servicePlayCartesianTrajectoryPosition;
	ros::ServiceServer m_servicePlayCartesianTrajectoryOrientation;
	ros::ServiceServer m_serviceStop;
	ros::ServiceServer m_serviceGetMeasuredCartesianPose;
	ros::ServiceServer m_serviceSendWrenchCommand;
	ros::ServiceServer m_serviceSendWrenchJoystickCommand;
	ros::ServiceServer m_serviceSendTwistJoystickCommand;
	ros::ServiceServer m_serviceSendTwistCommand;
	ros::ServiceServer m_servicePlayJointTrajectory;
	ros::ServiceServer m_servicePlaySelectedJointTrajectory;
	ros::ServiceServer m_serviceGetMeasuredJointAngles;
	ros::ServiceServer m_serviceSendJointSpeedsCommand;
	ros::ServiceServer m_serviceSendSelectedJointSpeedCommand;
	ros::ServiceServer m_serviceSendGripperCommand;
	ros::ServiceServer m_serviceGetMeasuredGripperMovement;
	ros::ServiceServer m_serviceSetAdmittance;
	ros::ServiceServer m_serviceSetOperatingMode;
	ros::ServiceServer m_serviceApplyEmergencyStop;
	ros::ServiceServer m_serviceBase_ClearFaults;
	ros::ServiceServer m_serviceBase_GetControlMode;
	ros::ServiceServer m_serviceGetOperatingMode;
	ros::ServiceServer m_serviceSetServoingMode;
	ros::ServiceServer m_serviceGetServoingMode;
	ros::ServiceServer m_serviceOnNotificationServoingModeTopic;
	ros::ServiceServer m_serviceRestoreFactorySettings;
	ros::ServiceServer m_serviceOnNotificationFactoryTopic;
	ros::ServiceServer m_serviceGetAllConnectedControllers;
	ros::ServiceServer m_serviceGetControllerState;
	ros::ServiceServer m_serviceGetActuatorCount;
	ros::ServiceServer m_serviceStartWifiScan;
	ros::ServiceServer m_serviceGetConfiguredWifi;
	ros::ServiceServer m_serviceOnNotificationNetworkTopic;
	ros::ServiceServer m_serviceGetArmState;
	ros::ServiceServer m_serviceOnNotificationArmStateTopic;
	ros::ServiceServer m_serviceGetIPv4Information;
	ros::ServiceServer m_serviceSetWifiCountryCode;
	ros::ServiceServer m_serviceGetWifiCountryCode;
	ros::ServiceServer m_serviceBase_SetCapSenseConfig;
	ros::ServiceServer m_serviceBase_GetCapSenseConfig;
	ros::ServiceServer m_serviceGetAllJointsSpeedHardLimitation;
	ros::ServiceServer m_serviceGetAllJointsTorqueHardLimitation;
	ros::ServiceServer m_serviceGetTwistHardLimitation;
	ros::ServiceServer m_serviceGetWrenchHardLimitation;
	ros::ServiceServer m_serviceSendJointSpeedsJoystickCommand;
	ros::ServiceServer m_serviceSendSelectedJointSpeedJoystickCommand;
	ros::ServiceServer m_serviceEnableBridge;
	ros::ServiceServer m_serviceDisableBridge;
	ros::ServiceServer m_serviceGetBridgeList;
	ros::ServiceServer m_serviceGetBridgeConfig;
	ros::ServiceServer m_servicePlayPreComputedJointTrajectory;
	ros::ServiceServer m_serviceGetProductConfiguration;
	ros::ServiceServer m_serviceUpdateEndEffectorTypeConfiguration;
	ros::ServiceServer m_serviceRestoreFactoryProductConfiguration;
	ros::ServiceServer m_serviceGetTrajectoryErrorReport;
	ros::ServiceServer m_serviceGetAllJointsSpeedSoftLimitation;
	ros::ServiceServer m_serviceGetAllJointsTorqueSoftLimitation;
	ros::ServiceServer m_serviceGetTwistSoftLimitation;
	ros::ServiceServer m_serviceGetWrenchSoftLimitation;
	ros::ServiceServer m_serviceSetControllerConfigurationMode;
	ros::ServiceServer m_serviceGetControllerConfigurationMode;
	ros::ServiceServer m_serviceStartTeaching;
	ros::ServiceServer m_serviceStopTeaching;
	ros::ServiceServer m_serviceAddSequenceTasks;
	ros::ServiceServer m_serviceUpdateSequenceTask;
	ros::ServiceServer m_serviceSwapSequenceTasks;
	ros::ServiceServer m_serviceReadSequenceTask;
	ros::ServiceServer m_serviceReadAllSequenceTasks;
	ros::ServiceServer m_serviceDeleteSequenceTask;
	ros::ServiceServer m_serviceDeleteAllSequenceTasks;
	ros::ServiceServer m_serviceTakeSnapshot;
	ros::ServiceServer m_serviceGetFirmwareBundleVersions;
	ros::ServiceServer m_serviceExecuteWaypointTrajectory;
	ros::ServiceServer m_serviceMoveSequenceTask;
	ros::ServiceServer m_serviceDuplicateMapping;
	ros::ServiceServer m_serviceDuplicateMap;
	ros::ServiceServer m_serviceSetControllerConfiguration;
	ros::ServiceServer m_serviceGetControllerConfiguration;
	ros::ServiceServer m_serviceGetAllControllerConfigurations;
	ros::ServiceServer m_serviceComputeForwardKinematics;
	ros::ServiceServer m_serviceComputeInverseKinematics;
	ros::ServiceServer m_serviceValidateWaypointList;
};
#endif
