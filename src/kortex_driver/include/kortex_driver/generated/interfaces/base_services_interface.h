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

#include <memory>
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
        IBaseServices(rclcpp::Node::SharedPtr node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) = 0;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) = 0;
        virtual bool CreateUserProfile(const std::shared_ptr<kortex_driver::srv::CreateUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::CreateUserProfile::Response> res) = 0;
        virtual bool UpdateUserProfile(const std::shared_ptr<kortex_driver::srv::UpdateUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::UpdateUserProfile::Response> res) = 0;
        virtual bool ReadUserProfile(const std::shared_ptr<kortex_driver::srv::ReadUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::ReadUserProfile::Response> res) = 0;
        virtual bool DeleteUserProfile(const std::shared_ptr<kortex_driver::srv::DeleteUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::DeleteUserProfile::Response> res) = 0;
        virtual bool ReadAllUserProfiles(const std::shared_ptr<kortex_driver::srv::ReadAllUserProfiles::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllUserProfiles::Response> res) = 0;
        virtual bool ReadAllUsers(const std::shared_ptr<kortex_driver::srv::ReadAllUsers::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllUsers::Response> res) = 0;
        virtual bool ChangePassword(const std::shared_ptr<kortex_driver::srv::ChangePassword::Request> req, std::shared_ptr<kortex_driver::srv::ChangePassword::Response> res) = 0;
        virtual bool CreateSequence(const std::shared_ptr<kortex_driver::srv::CreateSequence::Request> req, std::shared_ptr<kortex_driver::srv::CreateSequence::Response> res) = 0;
        virtual bool UpdateSequence(const std::shared_ptr<kortex_driver::srv::UpdateSequence::Request> req, std::shared_ptr<kortex_driver::srv::UpdateSequence::Response> res) = 0;
        virtual bool ReadSequence(const std::shared_ptr<kortex_driver::srv::ReadSequence::Request> req, std::shared_ptr<kortex_driver::srv::ReadSequence::Response> res) = 0;
        virtual bool DeleteSequence(const std::shared_ptr<kortex_driver::srv::DeleteSequence::Request> req, std::shared_ptr<kortex_driver::srv::DeleteSequence::Response> res) = 0;
        virtual bool ReadAllSequences(const std::shared_ptr<kortex_driver::srv::ReadAllSequences::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllSequences::Response> res) = 0;
        virtual bool PlaySequence(const std::shared_ptr<kortex_driver::srv::PlaySequence::Request> req, std::shared_ptr<kortex_driver::srv::PlaySequence::Response> res) = 0;
        virtual bool PlayAdvancedSequence(const std::shared_ptr<kortex_driver::srv::PlayAdvancedSequence::Request> req, std::shared_ptr<kortex_driver::srv::PlayAdvancedSequence::Response> res) = 0;
        virtual bool StopSequence(const std::shared_ptr<kortex_driver::srv::StopSequence::Request> req, std::shared_ptr<kortex_driver::srv::StopSequence::Response> res) = 0;
        virtual bool PauseSequence(const std::shared_ptr<kortex_driver::srv::PauseSequence::Request> req, std::shared_ptr<kortex_driver::srv::PauseSequence::Response> res) = 0;
        virtual bool ResumeSequence(const std::shared_ptr<kortex_driver::srv::ResumeSequence::Request> req, std::shared_ptr<kortex_driver::srv::ResumeSequence::Response> res) = 0;
        virtual bool CreateProtectionZone(const std::shared_ptr<kortex_driver::srv::CreateProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::CreateProtectionZone::Response> res) = 0;
        virtual bool UpdateProtectionZone(const std::shared_ptr<kortex_driver::srv::UpdateProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::UpdateProtectionZone::Response> res) = 0;
        virtual bool ReadProtectionZone(const std::shared_ptr<kortex_driver::srv::ReadProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::ReadProtectionZone::Response> res) = 0;
        virtual bool DeleteProtectionZone(const std::shared_ptr<kortex_driver::srv::DeleteProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::DeleteProtectionZone::Response> res) = 0;
        virtual bool ReadAllProtectionZones(const std::shared_ptr<kortex_driver::srv::ReadAllProtectionZones::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllProtectionZones::Response> res) = 0;
        virtual bool CreateMapping(const std::shared_ptr<kortex_driver::srv::CreateMapping::Request> req, std::shared_ptr<kortex_driver::srv::CreateMapping::Response> res) = 0;
        virtual bool ReadMapping(const std::shared_ptr<kortex_driver::srv::ReadMapping::Request> req, std::shared_ptr<kortex_driver::srv::ReadMapping::Response> res) = 0;
        virtual bool UpdateMapping(const std::shared_ptr<kortex_driver::srv::UpdateMapping::Request> req, std::shared_ptr<kortex_driver::srv::UpdateMapping::Response> res) = 0;
        virtual bool DeleteMapping(const std::shared_ptr<kortex_driver::srv::DeleteMapping::Request> req, std::shared_ptr<kortex_driver::srv::DeleteMapping::Response> res) = 0;
        virtual bool ReadAllMappings(const std::shared_ptr<kortex_driver::srv::ReadAllMappings::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllMappings::Response> res) = 0;
        virtual bool CreateMap(const std::shared_ptr<kortex_driver::srv::CreateMap::Request> req, std::shared_ptr<kortex_driver::srv::CreateMap::Response> res) = 0;
        virtual bool ReadMap(const std::shared_ptr<kortex_driver::srv::ReadMap::Request> req, std::shared_ptr<kortex_driver::srv::ReadMap::Response> res) = 0;
        virtual bool UpdateMap(const std::shared_ptr<kortex_driver::srv::UpdateMap::Request> req, std::shared_ptr<kortex_driver::srv::UpdateMap::Response> res) = 0;
        virtual bool DeleteMap(const std::shared_ptr<kortex_driver::srv::DeleteMap::Request> req, std::shared_ptr<kortex_driver::srv::DeleteMap::Response> res) = 0;
        virtual bool ReadAllMaps(const std::shared_ptr<kortex_driver::srv::ReadAllMaps::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllMaps::Response> res) = 0;
        virtual bool ActivateMap(const std::shared_ptr<kortex_driver::srv::ActivateMap::Request> req, std::shared_ptr<kortex_driver::srv::ActivateMap::Response> res) = 0;
        virtual bool CreateAction(const std::shared_ptr<kortex_driver::srv::CreateAction::Request> req, std::shared_ptr<kortex_driver::srv::CreateAction::Response> res) = 0;
        virtual bool ReadAction(const std::shared_ptr<kortex_driver::srv::ReadAction::Request> req, std::shared_ptr<kortex_driver::srv::ReadAction::Response> res) = 0;
        virtual bool ReadAllActions(const std::shared_ptr<kortex_driver::srv::ReadAllActions::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllActions::Response> res) = 0;
        virtual bool DeleteAction(const std::shared_ptr<kortex_driver::srv::DeleteAction::Request> req, std::shared_ptr<kortex_driver::srv::DeleteAction::Response> res) = 0;
        virtual bool UpdateAction(const std::shared_ptr<kortex_driver::srv::UpdateAction::Request> req, std::shared_ptr<kortex_driver::srv::UpdateAction::Response> res) = 0;
        virtual bool ExecuteActionFromReference(const std::shared_ptr<kortex_driver::srv::ExecuteActionFromReference::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteActionFromReference::Response> res) = 0;
        virtual bool ExecuteAction(const std::shared_ptr<kortex_driver::srv::ExecuteAction::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteAction::Response> res) = 0;
        virtual bool PauseAction(const std::shared_ptr<kortex_driver::srv::PauseAction::Request> req, std::shared_ptr<kortex_driver::srv::PauseAction::Response> res) = 0;
        virtual bool StopAction(const std::shared_ptr<kortex_driver::srv::StopAction::Request> req, std::shared_ptr<kortex_driver::srv::StopAction::Response> res) = 0;
        virtual bool ResumeAction(const std::shared_ptr<kortex_driver::srv::ResumeAction::Request> req, std::shared_ptr<kortex_driver::srv::ResumeAction::Response> res) = 0;
        virtual bool GetIPv4Configuration(const std::shared_ptr<kortex_driver::srv::GetIPv4Configuration::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Configuration::Response> res) = 0;
        virtual bool SetIPv4Configuration(const std::shared_ptr<kortex_driver::srv::SetIPv4Configuration::Request> req, std::shared_ptr<kortex_driver::srv::SetIPv4Configuration::Response> res) = 0;
        virtual bool SetCommunicationInterfaceEnable(const std::shared_ptr<kortex_driver::srv::SetCommunicationInterfaceEnable::Request> req, std::shared_ptr<kortex_driver::srv::SetCommunicationInterfaceEnable::Response> res) = 0;
        virtual bool IsCommunicationInterfaceEnable(const std::shared_ptr<kortex_driver::srv::IsCommunicationInterfaceEnable::Request> req, std::shared_ptr<kortex_driver::srv::IsCommunicationInterfaceEnable::Response> res) = 0;
        virtual bool GetAvailableWifi(const std::shared_ptr<kortex_driver::srv::GetAvailableWifi::Request> req, std::shared_ptr<kortex_driver::srv::GetAvailableWifi::Response> res) = 0;
        virtual bool GetWifiInformation(const std::shared_ptr<kortex_driver::srv::GetWifiInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetWifiInformation::Response> res) = 0;
        virtual bool AddWifiConfiguration(const std::shared_ptr<kortex_driver::srv::AddWifiConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::AddWifiConfiguration::Response> res) = 0;
        virtual bool DeleteWifiConfiguration(const std::shared_ptr<kortex_driver::srv::DeleteWifiConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::DeleteWifiConfiguration::Response> res) = 0;
        virtual bool GetAllConfiguredWifis(const std::shared_ptr<kortex_driver::srv::GetAllConfiguredWifis::Request> req, std::shared_ptr<kortex_driver::srv::GetAllConfiguredWifis::Response> res) = 0;
        virtual bool ConnectWifi(const std::shared_ptr<kortex_driver::srv::ConnectWifi::Request> req, std::shared_ptr<kortex_driver::srv::ConnectWifi::Response> res) = 0;
        virtual bool DisconnectWifi(const std::shared_ptr<kortex_driver::srv::DisconnectWifi::Request> req, std::shared_ptr<kortex_driver::srv::DisconnectWifi::Response> res) = 0;
        virtual bool GetConnectedWifiInformation(const std::shared_ptr<kortex_driver::srv::GetConnectedWifiInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetConnectedWifiInformation::Response> res) = 0;
        virtual bool Base_Unsubscribe(const std::shared_ptr<kortex_driver::srv::BaseUnsubscribe::Request> req, std::shared_ptr<kortex_driver::srv::BaseUnsubscribe::Response> res) = 0;
        virtual bool OnNotificationConfigurationChangeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationConfigurationChangeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationConfigurationChangeTopic::Response> res) = 0;
        virtual void cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif) = 0;
        virtual bool OnNotificationMappingInfoTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationMappingInfoTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationMappingInfoTopic::Response> res) = 0;
        virtual void cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif) = 0;
        virtual bool Base_OnNotificationControlModeTopic(const std::shared_ptr<kortex_driver::srv::BaseOnNotificationControlModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::BaseOnNotificationControlModeTopic::Response> res) = 0;
        virtual void cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif) = 0;
        virtual bool OnNotificationOperatingModeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationOperatingModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationOperatingModeTopic::Response> res) = 0;
        virtual void cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif) = 0;
        virtual bool OnNotificationSequenceInfoTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationSequenceInfoTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationSequenceInfoTopic::Response> res) = 0;
        virtual void cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif) = 0;
        virtual bool OnNotificationProtectionZoneTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationProtectionZoneTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationProtectionZoneTopic::Response> res) = 0;
        virtual void cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif) = 0;
        virtual bool OnNotificationUserTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationUserTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationUserTopic::Response> res) = 0;
        virtual void cb_UserTopic(Kinova::Api::Base::UserNotification notif) = 0;
        virtual bool OnNotificationControllerTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationControllerTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationControllerTopic::Response> res) = 0;
        virtual void cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif) = 0;
        virtual bool OnNotificationActionTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationActionTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationActionTopic::Response> res) = 0;
        virtual void cb_ActionTopic(Kinova::Api::Base::ActionNotification notif) = 0;
        virtual bool OnNotificationRobotEventTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationRobotEventTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationRobotEventTopic::Response> res) = 0;
        virtual void cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif) = 0;
        virtual bool PlayCartesianTrajectory(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectory::Response> res) = 0;
        virtual bool PlayCartesianTrajectoryPosition(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryPosition::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryPosition::Response> res) = 0;
        virtual bool PlayCartesianTrajectoryOrientation(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryOrientation::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryOrientation::Response> res) = 0;
        virtual bool Stop(const std::shared_ptr<kortex_driver::srv::Stop::Request> req, std::shared_ptr<kortex_driver::srv::Stop::Response> res) = 0;
        virtual bool GetMeasuredCartesianPose(const std::shared_ptr<kortex_driver::srv::GetMeasuredCartesianPose::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredCartesianPose::Response> res) = 0;
        virtual bool SendWrenchCommand(const std::shared_ptr<kortex_driver::srv::SendWrenchCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendWrenchCommand::Response> res) = 0;
        virtual bool SendWrenchJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendWrenchJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendWrenchJoystickCommand::Response> res) = 0;
        virtual bool SendTwistJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendTwistJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendTwistJoystickCommand::Response> res) = 0;
        virtual bool SendTwistCommand(const std::shared_ptr<kortex_driver::srv::SendTwistCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendTwistCommand::Response> res) = 0;
        virtual bool PlayJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlayJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayJointTrajectory::Response> res) = 0;
        virtual bool PlaySelectedJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlaySelectedJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlaySelectedJointTrajectory::Response> res) = 0;
        virtual bool GetMeasuredJointAngles(const std::shared_ptr<kortex_driver::srv::GetMeasuredJointAngles::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredJointAngles::Response> res) = 0;
        virtual bool SendJointSpeedsCommand(const std::shared_ptr<kortex_driver::srv::SendJointSpeedsCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendJointSpeedsCommand::Response> res) = 0;
        virtual bool SendSelectedJointSpeedCommand(const std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedCommand::Response> res) = 0;
        virtual bool SendGripperCommand(const std::shared_ptr<kortex_driver::srv::SendGripperCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendGripperCommand::Response> res) = 0;
        virtual bool GetMeasuredGripperMovement(const std::shared_ptr<kortex_driver::srv::GetMeasuredGripperMovement::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredGripperMovement::Response> res) = 0;
        virtual bool SetAdmittance(const std::shared_ptr<kortex_driver::srv::SetAdmittance::Request> req, std::shared_ptr<kortex_driver::srv::SetAdmittance::Response> res) = 0;
        virtual bool SetOperatingMode(const std::shared_ptr<kortex_driver::srv::SetOperatingMode::Request> req, std::shared_ptr<kortex_driver::srv::SetOperatingMode::Response> res) = 0;
        virtual bool ApplyEmergencyStop(const std::shared_ptr<kortex_driver::srv::ApplyEmergencyStop::Request> req, std::shared_ptr<kortex_driver::srv::ApplyEmergencyStop::Response> res) = 0;
        virtual bool Base_ClearFaults(const std::shared_ptr<kortex_driver::srv::BaseClearFaults::Request> req, std::shared_ptr<kortex_driver::srv::BaseClearFaults::Response> res) = 0;
        virtual bool Base_GetControlMode(const std::shared_ptr<kortex_driver::srv::BaseGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::BaseGetControlMode::Response> res) = 0;
        virtual bool GetOperatingMode(const std::shared_ptr<kortex_driver::srv::GetOperatingMode::Request> req, std::shared_ptr<kortex_driver::srv::GetOperatingMode::Response> res) = 0;
        virtual bool SetServoingMode(const std::shared_ptr<kortex_driver::srv::SetServoingMode::Request> req, std::shared_ptr<kortex_driver::srv::SetServoingMode::Response> res) = 0;
        virtual bool GetServoingMode(const std::shared_ptr<kortex_driver::srv::GetServoingMode::Request> req, std::shared_ptr<kortex_driver::srv::GetServoingMode::Response> res) = 0;
        virtual bool OnNotificationServoingModeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationServoingModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationServoingModeTopic::Response> res) = 0;
        virtual void cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif) = 0;
        virtual bool RestoreFactorySettings(const std::shared_ptr<kortex_driver::srv::RestoreFactorySettings::Request> req, std::shared_ptr<kortex_driver::srv::RestoreFactorySettings::Response> res) = 0;
        virtual bool OnNotificationFactoryTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationFactoryTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationFactoryTopic::Response> res) = 0;
        virtual void cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif) = 0;
        virtual bool GetAllConnectedControllers(const std::shared_ptr<kortex_driver::srv::GetAllConnectedControllers::Request> req, std::shared_ptr<kortex_driver::srv::GetAllConnectedControllers::Response> res) = 0;
        virtual bool GetControllerState(const std::shared_ptr<kortex_driver::srv::GetControllerState::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerState::Response> res) = 0;
        virtual bool GetActuatorCount(const std::shared_ptr<kortex_driver::srv::GetActuatorCount::Request> req, std::shared_ptr<kortex_driver::srv::GetActuatorCount::Response> res) = 0;
        virtual bool StartWifiScan(const std::shared_ptr<kortex_driver::srv::StartWifiScan::Request> req, std::shared_ptr<kortex_driver::srv::StartWifiScan::Response> res) = 0;
        virtual bool GetConfiguredWifi(const std::shared_ptr<kortex_driver::srv::GetConfiguredWifi::Request> req, std::shared_ptr<kortex_driver::srv::GetConfiguredWifi::Response> res) = 0;
        virtual bool OnNotificationNetworkTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationNetworkTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationNetworkTopic::Response> res) = 0;
        virtual void cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif) = 0;
        virtual bool GetArmState(const std::shared_ptr<kortex_driver::srv::GetArmState::Request> req, std::shared_ptr<kortex_driver::srv::GetArmState::Response> res) = 0;
        virtual bool OnNotificationArmStateTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationArmStateTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationArmStateTopic::Response> res) = 0;
        virtual void cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif) = 0;
        virtual bool GetIPv4Information(const std::shared_ptr<kortex_driver::srv::GetIPv4Information::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Information::Response> res) = 0;
        virtual bool SetWifiCountryCode(const std::shared_ptr<kortex_driver::srv::SetWifiCountryCode::Request> req, std::shared_ptr<kortex_driver::srv::SetWifiCountryCode::Response> res) = 0;
        virtual bool GetWifiCountryCode(const std::shared_ptr<kortex_driver::srv::GetWifiCountryCode::Request> req, std::shared_ptr<kortex_driver::srv::GetWifiCountryCode::Response> res) = 0;
        virtual bool Base_SetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::BaseSetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::BaseSetCapSenseConfig::Response> res) = 0;
        virtual bool Base_GetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::BaseGetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::BaseGetCapSenseConfig::Response> res) = 0;
        virtual bool GetAllJointsSpeedHardLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedHardLimitation::Response> res) = 0;
        virtual bool GetAllJointsTorqueHardLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueHardLimitation::Response> res) = 0;
        virtual bool GetTwistHardLimitation(const std::shared_ptr<kortex_driver::srv::GetTwistHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetTwistHardLimitation::Response> res) = 0;
        virtual bool GetWrenchHardLimitation(const std::shared_ptr<kortex_driver::srv::GetWrenchHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetWrenchHardLimitation::Response> res) = 0;
        virtual bool SendJointSpeedsJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendJointSpeedsJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendJointSpeedsJoystickCommand::Response> res) = 0;
        virtual bool SendSelectedJointSpeedJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Response> res) = 0;
        virtual bool EnableBridge(const std::shared_ptr<kortex_driver::srv::EnableBridge::Request> req, std::shared_ptr<kortex_driver::srv::EnableBridge::Response> res) = 0;
        virtual bool DisableBridge(const std::shared_ptr<kortex_driver::srv::DisableBridge::Request> req, std::shared_ptr<kortex_driver::srv::DisableBridge::Response> res) = 0;
        virtual bool GetBridgeList(const std::shared_ptr<kortex_driver::srv::GetBridgeList::Request> req, std::shared_ptr<kortex_driver::srv::GetBridgeList::Response> res) = 0;
        virtual bool GetBridgeConfig(const std::shared_ptr<kortex_driver::srv::GetBridgeConfig::Request> req, std::shared_ptr<kortex_driver::srv::GetBridgeConfig::Response> res) = 0;
        virtual bool PlayPreComputedJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlayPreComputedJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayPreComputedJointTrajectory::Response> res) = 0;
        virtual bool GetProductConfiguration(const std::shared_ptr<kortex_driver::srv::GetProductConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetProductConfiguration::Response> res) = 0;
        virtual bool UpdateEndEffectorTypeConfiguration(const std::shared_ptr<kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Response> res) = 0;
        virtual bool RestoreFactoryProductConfiguration(const std::shared_ptr<kortex_driver::srv::RestoreFactoryProductConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::RestoreFactoryProductConfiguration::Response> res) = 0;
        virtual bool GetTrajectoryErrorReport(const std::shared_ptr<kortex_driver::srv::GetTrajectoryErrorReport::Request> req, std::shared_ptr<kortex_driver::srv::GetTrajectoryErrorReport::Response> res) = 0;
        virtual bool GetAllJointsSpeedSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Response> res) = 0;
        virtual bool GetAllJointsTorqueSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Response> res) = 0;
        virtual bool GetTwistSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetTwistSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetTwistSoftLimitation::Response> res) = 0;
        virtual bool GetWrenchSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetWrenchSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetWrenchSoftLimitation::Response> res) = 0;
        virtual bool SetControllerConfigurationMode(const std::shared_ptr<kortex_driver::srv::SetControllerConfigurationMode::Request> req, std::shared_ptr<kortex_driver::srv::SetControllerConfigurationMode::Response> res) = 0;
        virtual bool GetControllerConfigurationMode(const std::shared_ptr<kortex_driver::srv::GetControllerConfigurationMode::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerConfigurationMode::Response> res) = 0;
        virtual bool StartTeaching(const std::shared_ptr<kortex_driver::srv::StartTeaching::Request> req, std::shared_ptr<kortex_driver::srv::StartTeaching::Response> res) = 0;
        virtual bool StopTeaching(const std::shared_ptr<kortex_driver::srv::StopTeaching::Request> req, std::shared_ptr<kortex_driver::srv::StopTeaching::Response> res) = 0;
        virtual bool AddSequenceTasks(const std::shared_ptr<kortex_driver::srv::AddSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::AddSequenceTasks::Response> res) = 0;
        virtual bool UpdateSequenceTask(const std::shared_ptr<kortex_driver::srv::UpdateSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::UpdateSequenceTask::Response> res) = 0;
        virtual bool SwapSequenceTasks(const std::shared_ptr<kortex_driver::srv::SwapSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::SwapSequenceTasks::Response> res) = 0;
        virtual bool ReadSequenceTask(const std::shared_ptr<kortex_driver::srv::ReadSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::ReadSequenceTask::Response> res) = 0;
        virtual bool ReadAllSequenceTasks(const std::shared_ptr<kortex_driver::srv::ReadAllSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllSequenceTasks::Response> res) = 0;
        virtual bool DeleteSequenceTask(const std::shared_ptr<kortex_driver::srv::DeleteSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::DeleteSequenceTask::Response> res) = 0;
        virtual bool DeleteAllSequenceTasks(const std::shared_ptr<kortex_driver::srv::DeleteAllSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::DeleteAllSequenceTasks::Response> res) = 0;
        virtual bool TakeSnapshot(const std::shared_ptr<kortex_driver::srv::TakeSnapshot::Request> req, std::shared_ptr<kortex_driver::srv::TakeSnapshot::Response> res) = 0;
        virtual bool GetFirmwareBundleVersions(const std::shared_ptr<kortex_driver::srv::GetFirmwareBundleVersions::Request> req, std::shared_ptr<kortex_driver::srv::GetFirmwareBundleVersions::Response> res) = 0;
        virtual bool ExecuteWaypointTrajectory(const std::shared_ptr<kortex_driver::srv::ExecuteWaypointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteWaypointTrajectory::Response> res) = 0;
        virtual bool MoveSequenceTask(const std::shared_ptr<kortex_driver::srv::MoveSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::MoveSequenceTask::Response> res) = 0;
        virtual bool DuplicateMapping(const std::shared_ptr<kortex_driver::srv::DuplicateMapping::Request> req, std::shared_ptr<kortex_driver::srv::DuplicateMapping::Response> res) = 0;
        virtual bool DuplicateMap(const std::shared_ptr<kortex_driver::srv::DuplicateMap::Request> req, std::shared_ptr<kortex_driver::srv::DuplicateMap::Response> res) = 0;
        virtual bool SetControllerConfiguration(const std::shared_ptr<kortex_driver::srv::SetControllerConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetControllerConfiguration::Response> res) = 0;
        virtual bool GetControllerConfiguration(const std::shared_ptr<kortex_driver::srv::GetControllerConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerConfiguration::Response> res) = 0;
        virtual bool GetAllControllerConfigurations(const std::shared_ptr<kortex_driver::srv::GetAllControllerConfigurations::Request> req, std::shared_ptr<kortex_driver::srv::GetAllControllerConfigurations::Response> res) = 0;
        virtual bool ComputeForwardKinematics(const std::shared_ptr<kortex_driver::srv::ComputeForwardKinematics::Request> req, std::shared_ptr<kortex_driver::srv::ComputeForwardKinematics::Response> res) = 0;
        virtual bool ComputeInverseKinematics(const std::shared_ptr<kortex_driver::srv::ComputeInverseKinematics::Request> req, std::shared_ptr<kortex_driver::srv::ComputeInverseKinematics::Response> res) = 0;
        virtual bool ValidateWaypointList(const std::shared_ptr<kortex_driver::srv::ValidateWaypointList::Request> req, std::shared_ptr<kortex_driver::srv::ValidateWaypointList::Response> res) = 0;

protected:
        rclcpp::Node::SharedPtr m_node_handle;
        rclcpp::Publisher<kortex_driver::msg::KortexError>::SharedPtr m_pub_Error;
        rclcpp::Publisher<kortex_driver::msg::ConfigurationChangeNotification>::SharedPtr m_pub_ConfigurationChangeTopic;
        bool m_is_activated_ConfigurationChangeTopic;
        rclcpp::Publisher<kortex_driver::msg::MappingInfoNotification>::SharedPtr m_pub_MappingInfoTopic;
        bool m_is_activated_MappingInfoTopic;
        rclcpp::Publisher<kortex_driver::msg::BaseControlModeNotification>::SharedPtr m_pub_ControlModeTopic;
        bool m_is_activated_ControlModeTopic;
        rclcpp::Publisher<kortex_driver::msg::OperatingModeNotification>::SharedPtr m_pub_OperatingModeTopic;
        bool m_is_activated_OperatingModeTopic;
        rclcpp::Publisher<kortex_driver::msg::SequenceInfoNotification>::SharedPtr m_pub_SequenceInfoTopic;
        bool m_is_activated_SequenceInfoTopic;
        rclcpp::Publisher<kortex_driver::msg::ProtectionZoneNotification>::SharedPtr m_pub_ProtectionZoneTopic;
        bool m_is_activated_ProtectionZoneTopic;
        rclcpp::Publisher<kortex_driver::msg::UserNotification>::SharedPtr m_pub_UserTopic;
        bool m_is_activated_UserTopic;
        rclcpp::Publisher<kortex_driver::msg::ControllerNotification>::SharedPtr m_pub_ControllerTopic;
        bool m_is_activated_ControllerTopic;
        rclcpp::Publisher<kortex_driver::msg::ActionNotification>::SharedPtr m_pub_ActionTopic;
        bool m_is_activated_ActionTopic;
        rclcpp::Publisher<kortex_driver::msg::RobotEventNotification>::SharedPtr m_pub_RobotEventTopic;
        bool m_is_activated_RobotEventTopic;
        rclcpp::Publisher<kortex_driver::msg::ServoingModeNotification>::SharedPtr m_pub_ServoingModeTopic;
        bool m_is_activated_ServoingModeTopic;
        rclcpp::Publisher<kortex_driver::msg::FactoryNotification>::SharedPtr m_pub_FactoryTopic;
        bool m_is_activated_FactoryTopic;
        rclcpp::Publisher<kortex_driver::msg::NetworkNotification>::SharedPtr m_pub_NetworkTopic;
        bool m_is_activated_NetworkTopic;
        rclcpp::Publisher<kortex_driver::msg::ArmStateNotification>::SharedPtr m_pub_ArmStateTopic;
        bool m_is_activated_ArmStateTopic;

        rclcpp::Service<kortex_driver::srv::SetDeviceID>::SharedPtr m_serviceSetDeviceID;
        rclcpp::Service<kortex_driver::srv::SetApiOptions>::SharedPtr m_serviceSetApiOptions;

	rclcpp::Service<kortex_driver::srv::CreateUserProfile>::SharedPtr m_serviceCreateUserProfile;
	rclcpp::Service<kortex_driver::srv::UpdateUserProfile>::SharedPtr m_serviceUpdateUserProfile;
	rclcpp::Service<kortex_driver::srv::ReadUserProfile>::SharedPtr m_serviceReadUserProfile;
	rclcpp::Service<kortex_driver::srv::DeleteUserProfile>::SharedPtr m_serviceDeleteUserProfile;
	rclcpp::Service<kortex_driver::srv::ReadAllUserProfiles>::SharedPtr m_serviceReadAllUserProfiles;
	rclcpp::Service<kortex_driver::srv::ReadAllUsers>::SharedPtr m_serviceReadAllUsers;
	rclcpp::Service<kortex_driver::srv::ChangePassword>::SharedPtr m_serviceChangePassword;
	rclcpp::Service<kortex_driver::srv::CreateSequence>::SharedPtr m_serviceCreateSequence;
	rclcpp::Service<kortex_driver::srv::UpdateSequence>::SharedPtr m_serviceUpdateSequence;
	rclcpp::Service<kortex_driver::srv::ReadSequence>::SharedPtr m_serviceReadSequence;
	rclcpp::Service<kortex_driver::srv::DeleteSequence>::SharedPtr m_serviceDeleteSequence;
	rclcpp::Service<kortex_driver::srv::ReadAllSequences>::SharedPtr m_serviceReadAllSequences;
	rclcpp::Service<kortex_driver::srv::PlaySequence>::SharedPtr m_servicePlaySequence;
	rclcpp::Service<kortex_driver::srv::PlayAdvancedSequence>::SharedPtr m_servicePlayAdvancedSequence;
	rclcpp::Service<kortex_driver::srv::StopSequence>::SharedPtr m_serviceStopSequence;
	rclcpp::Service<kortex_driver::srv::PauseSequence>::SharedPtr m_servicePauseSequence;
	rclcpp::Service<kortex_driver::srv::ResumeSequence>::SharedPtr m_serviceResumeSequence;
	rclcpp::Service<kortex_driver::srv::CreateProtectionZone>::SharedPtr m_serviceCreateProtectionZone;
	rclcpp::Service<kortex_driver::srv::UpdateProtectionZone>::SharedPtr m_serviceUpdateProtectionZone;
	rclcpp::Service<kortex_driver::srv::ReadProtectionZone>::SharedPtr m_serviceReadProtectionZone;
	rclcpp::Service<kortex_driver::srv::DeleteProtectionZone>::SharedPtr m_serviceDeleteProtectionZone;
	rclcpp::Service<kortex_driver::srv::ReadAllProtectionZones>::SharedPtr m_serviceReadAllProtectionZones;
	rclcpp::Service<kortex_driver::srv::CreateMapping>::SharedPtr m_serviceCreateMapping;
	rclcpp::Service<kortex_driver::srv::ReadMapping>::SharedPtr m_serviceReadMapping;
	rclcpp::Service<kortex_driver::srv::UpdateMapping>::SharedPtr m_serviceUpdateMapping;
	rclcpp::Service<kortex_driver::srv::DeleteMapping>::SharedPtr m_serviceDeleteMapping;
	rclcpp::Service<kortex_driver::srv::ReadAllMappings>::SharedPtr m_serviceReadAllMappings;
	rclcpp::Service<kortex_driver::srv::CreateMap>::SharedPtr m_serviceCreateMap;
	rclcpp::Service<kortex_driver::srv::ReadMap>::SharedPtr m_serviceReadMap;
	rclcpp::Service<kortex_driver::srv::UpdateMap>::SharedPtr m_serviceUpdateMap;
	rclcpp::Service<kortex_driver::srv::DeleteMap>::SharedPtr m_serviceDeleteMap;
	rclcpp::Service<kortex_driver::srv::ReadAllMaps>::SharedPtr m_serviceReadAllMaps;
	rclcpp::Service<kortex_driver::srv::ActivateMap>::SharedPtr m_serviceActivateMap;
	rclcpp::Service<kortex_driver::srv::CreateAction>::SharedPtr m_serviceCreateAction;
	rclcpp::Service<kortex_driver::srv::ReadAction>::SharedPtr m_serviceReadAction;
	rclcpp::Service<kortex_driver::srv::ReadAllActions>::SharedPtr m_serviceReadAllActions;
	rclcpp::Service<kortex_driver::srv::DeleteAction>::SharedPtr m_serviceDeleteAction;
	rclcpp::Service<kortex_driver::srv::UpdateAction>::SharedPtr m_serviceUpdateAction;
	rclcpp::Service<kortex_driver::srv::ExecuteActionFromReference>::SharedPtr m_serviceExecuteActionFromReference;
	rclcpp::Service<kortex_driver::srv::ExecuteAction>::SharedPtr m_serviceExecuteAction;
	rclcpp::Service<kortex_driver::srv::PauseAction>::SharedPtr m_servicePauseAction;
	rclcpp::Service<kortex_driver::srv::StopAction>::SharedPtr m_serviceStopAction;
	rclcpp::Service<kortex_driver::srv::ResumeAction>::SharedPtr m_serviceResumeAction;
	rclcpp::Service<kortex_driver::srv::GetIPv4Configuration>::SharedPtr m_serviceGetIPv4Configuration;
	rclcpp::Service<kortex_driver::srv::SetIPv4Configuration>::SharedPtr m_serviceSetIPv4Configuration;
	rclcpp::Service<kortex_driver::srv::SetCommunicationInterfaceEnable>::SharedPtr m_serviceSetCommunicationInterfaceEnable;
	rclcpp::Service<kortex_driver::srv::IsCommunicationInterfaceEnable>::SharedPtr m_serviceIsCommunicationInterfaceEnable;
	rclcpp::Service<kortex_driver::srv::GetAvailableWifi>::SharedPtr m_serviceGetAvailableWifi;
	rclcpp::Service<kortex_driver::srv::GetWifiInformation>::SharedPtr m_serviceGetWifiInformation;
	rclcpp::Service<kortex_driver::srv::AddWifiConfiguration>::SharedPtr m_serviceAddWifiConfiguration;
	rclcpp::Service<kortex_driver::srv::DeleteWifiConfiguration>::SharedPtr m_serviceDeleteWifiConfiguration;
	rclcpp::Service<kortex_driver::srv::GetAllConfiguredWifis>::SharedPtr m_serviceGetAllConfiguredWifis;
	rclcpp::Service<kortex_driver::srv::ConnectWifi>::SharedPtr m_serviceConnectWifi;
	rclcpp::Service<kortex_driver::srv::DisconnectWifi>::SharedPtr m_serviceDisconnectWifi;
	rclcpp::Service<kortex_driver::srv::GetConnectedWifiInformation>::SharedPtr m_serviceGetConnectedWifiInformation;
	rclcpp::Service<kortex_driver::srv::BaseUnsubscribe>::SharedPtr m_serviceBase_Unsubscribe;
	rclcpp::Service<kortex_driver::srv::OnNotificationConfigurationChangeTopic>::SharedPtr m_serviceOnNotificationConfigurationChangeTopic;
	rclcpp::Service<kortex_driver::srv::OnNotificationMappingInfoTopic>::SharedPtr m_serviceOnNotificationMappingInfoTopic;
	rclcpp::Service<kortex_driver::srv::BaseOnNotificationControlModeTopic>::SharedPtr m_serviceBase_OnNotificationControlModeTopic;
	rclcpp::Service<kortex_driver::srv::OnNotificationOperatingModeTopic>::SharedPtr m_serviceOnNotificationOperatingModeTopic;
	rclcpp::Service<kortex_driver::srv::OnNotificationSequenceInfoTopic>::SharedPtr m_serviceOnNotificationSequenceInfoTopic;
	rclcpp::Service<kortex_driver::srv::OnNotificationProtectionZoneTopic>::SharedPtr m_serviceOnNotificationProtectionZoneTopic;
	rclcpp::Service<kortex_driver::srv::OnNotificationUserTopic>::SharedPtr m_serviceOnNotificationUserTopic;
	rclcpp::Service<kortex_driver::srv::OnNotificationControllerTopic>::SharedPtr m_serviceOnNotificationControllerTopic;
	rclcpp::Service<kortex_driver::srv::OnNotificationActionTopic>::SharedPtr m_serviceOnNotificationActionTopic;
	rclcpp::Service<kortex_driver::srv::OnNotificationRobotEventTopic>::SharedPtr m_serviceOnNotificationRobotEventTopic;
	rclcpp::Service<kortex_driver::srv::PlayCartesianTrajectory>::SharedPtr m_servicePlayCartesianTrajectory;
	rclcpp::Service<kortex_driver::srv::PlayCartesianTrajectoryPosition>::SharedPtr m_servicePlayCartesianTrajectoryPosition;
	rclcpp::Service<kortex_driver::srv::PlayCartesianTrajectoryOrientation>::SharedPtr m_servicePlayCartesianTrajectoryOrientation;
	rclcpp::Service<kortex_driver::srv::Stop>::SharedPtr m_serviceStop;
	rclcpp::Service<kortex_driver::srv::GetMeasuredCartesianPose>::SharedPtr m_serviceGetMeasuredCartesianPose;
	rclcpp::Service<kortex_driver::srv::SendWrenchCommand>::SharedPtr m_serviceSendWrenchCommand;
	rclcpp::Service<kortex_driver::srv::SendWrenchJoystickCommand>::SharedPtr m_serviceSendWrenchJoystickCommand;
	rclcpp::Service<kortex_driver::srv::SendTwistJoystickCommand>::SharedPtr m_serviceSendTwistJoystickCommand;
	rclcpp::Service<kortex_driver::srv::SendTwistCommand>::SharedPtr m_serviceSendTwistCommand;
	rclcpp::Service<kortex_driver::srv::PlayJointTrajectory>::SharedPtr m_servicePlayJointTrajectory;
	rclcpp::Service<kortex_driver::srv::PlaySelectedJointTrajectory>::SharedPtr m_servicePlaySelectedJointTrajectory;
	rclcpp::Service<kortex_driver::srv::GetMeasuredJointAngles>::SharedPtr m_serviceGetMeasuredJointAngles;
	rclcpp::Service<kortex_driver::srv::SendJointSpeedsCommand>::SharedPtr m_serviceSendJointSpeedsCommand;
	rclcpp::Service<kortex_driver::srv::SendSelectedJointSpeedCommand>::SharedPtr m_serviceSendSelectedJointSpeedCommand;
	rclcpp::Service<kortex_driver::srv::SendGripperCommand>::SharedPtr m_serviceSendGripperCommand;
	rclcpp::Service<kortex_driver::srv::GetMeasuredGripperMovement>::SharedPtr m_serviceGetMeasuredGripperMovement;
	rclcpp::Service<kortex_driver::srv::SetAdmittance>::SharedPtr m_serviceSetAdmittance;
	rclcpp::Service<kortex_driver::srv::SetOperatingMode>::SharedPtr m_serviceSetOperatingMode;
	rclcpp::Service<kortex_driver::srv::ApplyEmergencyStop>::SharedPtr m_serviceApplyEmergencyStop;
	rclcpp::Service<kortex_driver::srv::BaseClearFaults>::SharedPtr m_serviceBase_ClearFaults;
	rclcpp::Service<kortex_driver::srv::BaseGetControlMode>::SharedPtr m_serviceBase_GetControlMode;
	rclcpp::Service<kortex_driver::srv::GetOperatingMode>::SharedPtr m_serviceGetOperatingMode;
	rclcpp::Service<kortex_driver::srv::SetServoingMode>::SharedPtr m_serviceSetServoingMode;
	rclcpp::Service<kortex_driver::srv::GetServoingMode>::SharedPtr m_serviceGetServoingMode;
	rclcpp::Service<kortex_driver::srv::OnNotificationServoingModeTopic>::SharedPtr m_serviceOnNotificationServoingModeTopic;
	rclcpp::Service<kortex_driver::srv::RestoreFactorySettings>::SharedPtr m_serviceRestoreFactorySettings;
	rclcpp::Service<kortex_driver::srv::OnNotificationFactoryTopic>::SharedPtr m_serviceOnNotificationFactoryTopic;
	rclcpp::Service<kortex_driver::srv::GetAllConnectedControllers>::SharedPtr m_serviceGetAllConnectedControllers;
	rclcpp::Service<kortex_driver::srv::GetControllerState>::SharedPtr m_serviceGetControllerState;
	rclcpp::Service<kortex_driver::srv::GetActuatorCount>::SharedPtr m_serviceGetActuatorCount;
	rclcpp::Service<kortex_driver::srv::StartWifiScan>::SharedPtr m_serviceStartWifiScan;
	rclcpp::Service<kortex_driver::srv::GetConfiguredWifi>::SharedPtr m_serviceGetConfiguredWifi;
	rclcpp::Service<kortex_driver::srv::OnNotificationNetworkTopic>::SharedPtr m_serviceOnNotificationNetworkTopic;
	rclcpp::Service<kortex_driver::srv::GetArmState>::SharedPtr m_serviceGetArmState;
	rclcpp::Service<kortex_driver::srv::OnNotificationArmStateTopic>::SharedPtr m_serviceOnNotificationArmStateTopic;
	rclcpp::Service<kortex_driver::srv::GetIPv4Information>::SharedPtr m_serviceGetIPv4Information;
	rclcpp::Service<kortex_driver::srv::SetWifiCountryCode>::SharedPtr m_serviceSetWifiCountryCode;
	rclcpp::Service<kortex_driver::srv::GetWifiCountryCode>::SharedPtr m_serviceGetWifiCountryCode;
	rclcpp::Service<kortex_driver::srv::BaseSetCapSenseConfig>::SharedPtr m_serviceBase_SetCapSenseConfig;
	rclcpp::Service<kortex_driver::srv::BaseGetCapSenseConfig>::SharedPtr m_serviceBase_GetCapSenseConfig;
	rclcpp::Service<kortex_driver::srv::GetAllJointsSpeedHardLimitation>::SharedPtr m_serviceGetAllJointsSpeedHardLimitation;
	rclcpp::Service<kortex_driver::srv::GetAllJointsTorqueHardLimitation>::SharedPtr m_serviceGetAllJointsTorqueHardLimitation;
	rclcpp::Service<kortex_driver::srv::GetTwistHardLimitation>::SharedPtr m_serviceGetTwistHardLimitation;
	rclcpp::Service<kortex_driver::srv::GetWrenchHardLimitation>::SharedPtr m_serviceGetWrenchHardLimitation;
	rclcpp::Service<kortex_driver::srv::SendJointSpeedsJoystickCommand>::SharedPtr m_serviceSendJointSpeedsJoystickCommand;
	rclcpp::Service<kortex_driver::srv::SendSelectedJointSpeedJoystickCommand>::SharedPtr m_serviceSendSelectedJointSpeedJoystickCommand;
	rclcpp::Service<kortex_driver::srv::EnableBridge>::SharedPtr m_serviceEnableBridge;
	rclcpp::Service<kortex_driver::srv::DisableBridge>::SharedPtr m_serviceDisableBridge;
	rclcpp::Service<kortex_driver::srv::GetBridgeList>::SharedPtr m_serviceGetBridgeList;
	rclcpp::Service<kortex_driver::srv::GetBridgeConfig>::SharedPtr m_serviceGetBridgeConfig;
	rclcpp::Service<kortex_driver::srv::PlayPreComputedJointTrajectory>::SharedPtr m_servicePlayPreComputedJointTrajectory;
	rclcpp::Service<kortex_driver::srv::GetProductConfiguration>::SharedPtr m_serviceGetProductConfiguration;
	rclcpp::Service<kortex_driver::srv::UpdateEndEffectorTypeConfiguration>::SharedPtr m_serviceUpdateEndEffectorTypeConfiguration;
	rclcpp::Service<kortex_driver::srv::RestoreFactoryProductConfiguration>::SharedPtr m_serviceRestoreFactoryProductConfiguration;
	rclcpp::Service<kortex_driver::srv::GetTrajectoryErrorReport>::SharedPtr m_serviceGetTrajectoryErrorReport;
	rclcpp::Service<kortex_driver::srv::GetAllJointsSpeedSoftLimitation>::SharedPtr m_serviceGetAllJointsSpeedSoftLimitation;
	rclcpp::Service<kortex_driver::srv::GetAllJointsTorqueSoftLimitation>::SharedPtr m_serviceGetAllJointsTorqueSoftLimitation;
	rclcpp::Service<kortex_driver::srv::GetTwistSoftLimitation>::SharedPtr m_serviceGetTwistSoftLimitation;
	rclcpp::Service<kortex_driver::srv::GetWrenchSoftLimitation>::SharedPtr m_serviceGetWrenchSoftLimitation;
	rclcpp::Service<kortex_driver::srv::SetControllerConfigurationMode>::SharedPtr m_serviceSetControllerConfigurationMode;
	rclcpp::Service<kortex_driver::srv::GetControllerConfigurationMode>::SharedPtr m_serviceGetControllerConfigurationMode;
	rclcpp::Service<kortex_driver::srv::StartTeaching>::SharedPtr m_serviceStartTeaching;
	rclcpp::Service<kortex_driver::srv::StopTeaching>::SharedPtr m_serviceStopTeaching;
	rclcpp::Service<kortex_driver::srv::AddSequenceTasks>::SharedPtr m_serviceAddSequenceTasks;
	rclcpp::Service<kortex_driver::srv::UpdateSequenceTask>::SharedPtr m_serviceUpdateSequenceTask;
	rclcpp::Service<kortex_driver::srv::SwapSequenceTasks>::SharedPtr m_serviceSwapSequenceTasks;
	rclcpp::Service<kortex_driver::srv::ReadSequenceTask>::SharedPtr m_serviceReadSequenceTask;
	rclcpp::Service<kortex_driver::srv::ReadAllSequenceTasks>::SharedPtr m_serviceReadAllSequenceTasks;
	rclcpp::Service<kortex_driver::srv::DeleteSequenceTask>::SharedPtr m_serviceDeleteSequenceTask;
	rclcpp::Service<kortex_driver::srv::DeleteAllSequenceTasks>::SharedPtr m_serviceDeleteAllSequenceTasks;
	rclcpp::Service<kortex_driver::srv::TakeSnapshot>::SharedPtr m_serviceTakeSnapshot;
	rclcpp::Service<kortex_driver::srv::GetFirmwareBundleVersions>::SharedPtr m_serviceGetFirmwareBundleVersions;
	rclcpp::Service<kortex_driver::srv::ExecuteWaypointTrajectory>::SharedPtr m_serviceExecuteWaypointTrajectory;
	rclcpp::Service<kortex_driver::srv::MoveSequenceTask>::SharedPtr m_serviceMoveSequenceTask;
	rclcpp::Service<kortex_driver::srv::DuplicateMapping>::SharedPtr m_serviceDuplicateMapping;
	rclcpp::Service<kortex_driver::srv::DuplicateMap>::SharedPtr m_serviceDuplicateMap;
	rclcpp::Service<kortex_driver::srv::SetControllerConfiguration>::SharedPtr m_serviceSetControllerConfiguration;
	rclcpp::Service<kortex_driver::srv::GetControllerConfiguration>::SharedPtr m_serviceGetControllerConfiguration;
	rclcpp::Service<kortex_driver::srv::GetAllControllerConfigurations>::SharedPtr m_serviceGetAllControllerConfigurations;
	rclcpp::Service<kortex_driver::srv::ComputeForwardKinematics>::SharedPtr m_serviceComputeForwardKinematics;
	rclcpp::Service<kortex_driver::srv::ComputeInverseKinematics>::SharedPtr m_serviceComputeInverseKinematics;
	rclcpp::Service<kortex_driver::srv::ValidateWaypointList>::SharedPtr m_serviceValidateWaypointList;
};
#endif
