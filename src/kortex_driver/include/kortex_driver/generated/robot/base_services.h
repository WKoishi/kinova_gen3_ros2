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
 
#ifndef _KORTEX_BASE_ROBOT_SERVICES_H_
#define _KORTEX_BASE_ROBOT_SERVICES_H_

#include "kortex_driver/generated/interfaces/base_services_interface.h"

#include <Base.pb.h>
#include <BaseClientRpc.h>

using namespace std;

class BaseRobotServices : public IBaseServices
{
    public:
        BaseRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::Base::BaseClient* base, uint32_t device_id, uint32_t timeout_ms);

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) override;
        virtual bool CreateUserProfile(kortex_driver::srv::CreateUserProfile::Request  &req, kortex_driver::srv::CreateUserProfile::Response &res) override;
        virtual bool UpdateUserProfile(kortex_driver::srv::UpdateUserProfile::Request  &req, kortex_driver::srv::UpdateUserProfile::Response &res) override;
        virtual bool ReadUserProfile(kortex_driver::srv::ReadUserProfile::Request  &req, kortex_driver::srv::ReadUserProfile::Response &res) override;
        virtual bool DeleteUserProfile(kortex_driver::srv::DeleteUserProfile::Request  &req, kortex_driver::srv::DeleteUserProfile::Response &res) override;
        virtual bool ReadAllUserProfiles(kortex_driver::srv::ReadAllUserProfiles::Request  &req, kortex_driver::srv::ReadAllUserProfiles::Response &res) override;
        virtual bool ReadAllUsers(kortex_driver::srv::ReadAllUsers::Request  &req, kortex_driver::srv::ReadAllUsers::Response &res) override;
        virtual bool ChangePassword(kortex_driver::srv::ChangePassword::Request  &req, kortex_driver::srv::ChangePassword::Response &res) override;
        virtual bool CreateSequence(kortex_driver::srv::CreateSequence::Request  &req, kortex_driver::srv::CreateSequence::Response &res) override;
        virtual bool UpdateSequence(kortex_driver::srv::UpdateSequence::Request  &req, kortex_driver::srv::UpdateSequence::Response &res) override;
        virtual bool ReadSequence(kortex_driver::srv::ReadSequence::Request  &req, kortex_driver::srv::ReadSequence::Response &res) override;
        virtual bool DeleteSequence(kortex_driver::srv::DeleteSequence::Request  &req, kortex_driver::srv::DeleteSequence::Response &res) override;
        virtual bool ReadAllSequences(kortex_driver::srv::ReadAllSequences::Request  &req, kortex_driver::srv::ReadAllSequences::Response &res) override;
        virtual bool PlaySequence(kortex_driver::srv::PlaySequence::Request  &req, kortex_driver::srv::PlaySequence::Response &res) override;
        virtual bool PlayAdvancedSequence(kortex_driver::srv::PlayAdvancedSequence::Request  &req, kortex_driver::srv::PlayAdvancedSequence::Response &res) override;
        virtual bool StopSequence(kortex_driver::srv::StopSequence::Request  &req, kortex_driver::srv::StopSequence::Response &res) override;
        virtual bool PauseSequence(kortex_driver::srv::PauseSequence::Request  &req, kortex_driver::srv::PauseSequence::Response &res) override;
        virtual bool ResumeSequence(kortex_driver::srv::ResumeSequence::Request  &req, kortex_driver::srv::ResumeSequence::Response &res) override;
        virtual bool CreateProtectionZone(kortex_driver::srv::CreateProtectionZone::Request  &req, kortex_driver::srv::CreateProtectionZone::Response &res) override;
        virtual bool UpdateProtectionZone(kortex_driver::srv::UpdateProtectionZone::Request  &req, kortex_driver::srv::UpdateProtectionZone::Response &res) override;
        virtual bool ReadProtectionZone(kortex_driver::srv::ReadProtectionZone::Request  &req, kortex_driver::srv::ReadProtectionZone::Response &res) override;
        virtual bool DeleteProtectionZone(kortex_driver::srv::DeleteProtectionZone::Request  &req, kortex_driver::srv::DeleteProtectionZone::Response &res) override;
        virtual bool ReadAllProtectionZones(kortex_driver::srv::ReadAllProtectionZones::Request  &req, kortex_driver::srv::ReadAllProtectionZones::Response &res) override;
        virtual bool CreateMapping(kortex_driver::srv::CreateMapping::Request  &req, kortex_driver::srv::CreateMapping::Response &res) override;
        virtual bool ReadMapping(kortex_driver::srv::ReadMapping::Request  &req, kortex_driver::srv::ReadMapping::Response &res) override;
        virtual bool UpdateMapping(kortex_driver::srv::UpdateMapping::Request  &req, kortex_driver::srv::UpdateMapping::Response &res) override;
        virtual bool DeleteMapping(kortex_driver::srv::DeleteMapping::Request  &req, kortex_driver::srv::DeleteMapping::Response &res) override;
        virtual bool ReadAllMappings(kortex_driver::srv::ReadAllMappings::Request  &req, kortex_driver::srv::ReadAllMappings::Response &res) override;
        virtual bool CreateMap(kortex_driver::srv::CreateMap::Request  &req, kortex_driver::srv::CreateMap::Response &res) override;
        virtual bool ReadMap(kortex_driver::srv::ReadMap::Request  &req, kortex_driver::srv::ReadMap::Response &res) override;
        virtual bool UpdateMap(kortex_driver::srv::UpdateMap::Request  &req, kortex_driver::srv::UpdateMap::Response &res) override;
        virtual bool DeleteMap(kortex_driver::srv::DeleteMap::Request  &req, kortex_driver::srv::DeleteMap::Response &res) override;
        virtual bool ReadAllMaps(kortex_driver::srv::ReadAllMaps::Request  &req, kortex_driver::srv::ReadAllMaps::Response &res) override;
        virtual bool ActivateMap(kortex_driver::srv::ActivateMap::Request  &req, kortex_driver::srv::ActivateMap::Response &res) override;
        virtual bool CreateAction(kortex_driver::srv::CreateAction::Request  &req, kortex_driver::srv::CreateAction::Response &res) override;
        virtual bool ReadAction(kortex_driver::srv::ReadAction::Request  &req, kortex_driver::srv::ReadAction::Response &res) override;
        virtual bool ReadAllActions(kortex_driver::srv::ReadAllActions::Request  &req, kortex_driver::srv::ReadAllActions::Response &res) override;
        virtual bool DeleteAction(kortex_driver::srv::DeleteAction::Request  &req, kortex_driver::srv::DeleteAction::Response &res) override;
        virtual bool UpdateAction(kortex_driver::srv::UpdateAction::Request  &req, kortex_driver::srv::UpdateAction::Response &res) override;
        virtual bool ExecuteActionFromReference(kortex_driver::srv::ExecuteActionFromReference::Request  &req, kortex_driver::srv::ExecuteActionFromReference::Response &res) override;
        virtual bool ExecuteAction(kortex_driver::srv::ExecuteAction::Request  &req, kortex_driver::srv::ExecuteAction::Response &res) override;
        virtual bool PauseAction(kortex_driver::srv::PauseAction::Request  &req, kortex_driver::srv::PauseAction::Response &res) override;
        virtual bool StopAction(kortex_driver::srv::StopAction::Request  &req, kortex_driver::srv::StopAction::Response &res) override;
        virtual bool ResumeAction(kortex_driver::srv::ResumeAction::Request  &req, kortex_driver::srv::ResumeAction::Response &res) override;
        virtual bool GetIPv4Configuration(kortex_driver::srv::GetIPv4Configuration::Request  &req, kortex_driver::srv::GetIPv4Configuration::Response &res) override;
        virtual bool SetIPv4Configuration(kortex_driver::srv::SetIPv4Configuration::Request  &req, kortex_driver::srv::SetIPv4Configuration::Response &res) override;
        virtual bool SetCommunicationInterfaceEnable(kortex_driver::srv::SetCommunicationInterfaceEnable::Request  &req, kortex_driver::srv::SetCommunicationInterfaceEnable::Response &res) override;
        virtual bool IsCommunicationInterfaceEnable(kortex_driver::srv::IsCommunicationInterfaceEnable::Request  &req, kortex_driver::srv::IsCommunicationInterfaceEnable::Response &res) override;
        virtual bool GetAvailableWifi(kortex_driver::srv::GetAvailableWifi::Request  &req, kortex_driver::srv::GetAvailableWifi::Response &res) override;
        virtual bool GetWifiInformation(kortex_driver::srv::GetWifiInformation::Request  &req, kortex_driver::srv::GetWifiInformation::Response &res) override;
        virtual bool AddWifiConfiguration(kortex_driver::srv::AddWifiConfiguration::Request  &req, kortex_driver::srv::AddWifiConfiguration::Response &res) override;
        virtual bool DeleteWifiConfiguration(kortex_driver::srv::DeleteWifiConfiguration::Request  &req, kortex_driver::srv::DeleteWifiConfiguration::Response &res) override;
        virtual bool GetAllConfiguredWifis(kortex_driver::srv::GetAllConfiguredWifis::Request  &req, kortex_driver::srv::GetAllConfiguredWifis::Response &res) override;
        virtual bool ConnectWifi(kortex_driver::srv::ConnectWifi::Request  &req, kortex_driver::srv::ConnectWifi::Response &res) override;
        virtual bool DisconnectWifi(kortex_driver::srv::DisconnectWifi::Request  &req, kortex_driver::srv::DisconnectWifi::Response &res) override;
        virtual bool GetConnectedWifiInformation(kortex_driver::srv::GetConnectedWifiInformation::Request  &req, kortex_driver::srv::GetConnectedWifiInformation::Response &res) override;
        virtual bool Base_Unsubscribe(kortex_driver::srv::BaseUnsubscribe::Request  &req, kortex_driver::srv::BaseUnsubscribe::Response &res) override;
        virtual bool OnNotificationConfigurationChangeTopic(kortex_driver::srv::OnNotificationConfigurationChangeTopic::Request  &req, kortex_driver::srv::OnNotificationConfigurationChangeTopic::Response &res) override;
        virtual void cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif) override;
        virtual bool OnNotificationMappingInfoTopic(kortex_driver::srv::OnNotificationMappingInfoTopic::Request  &req, kortex_driver::srv::OnNotificationMappingInfoTopic::Response &res) override;
        virtual void cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif) override;
        virtual bool Base_OnNotificationControlModeTopic(kortex_driver::srv::BaseOnNotificationControlModeTopic::Request  &req, kortex_driver::srv::BaseOnNotificationControlModeTopic::Response &res) override;
        virtual void cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif) override;
        virtual bool OnNotificationOperatingModeTopic(kortex_driver::srv::OnNotificationOperatingModeTopic::Request  &req, kortex_driver::srv::OnNotificationOperatingModeTopic::Response &res) override;
        virtual void cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif) override;
        virtual bool OnNotificationSequenceInfoTopic(kortex_driver::srv::OnNotificationSequenceInfoTopic::Request  &req, kortex_driver::srv::OnNotificationSequenceInfoTopic::Response &res) override;
        virtual void cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif) override;
        virtual bool OnNotificationProtectionZoneTopic(kortex_driver::srv::OnNotificationProtectionZoneTopic::Request  &req, kortex_driver::srv::OnNotificationProtectionZoneTopic::Response &res) override;
        virtual void cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif) override;
        virtual bool OnNotificationUserTopic(kortex_driver::srv::OnNotificationUserTopic::Request  &req, kortex_driver::srv::OnNotificationUserTopic::Response &res) override;
        virtual void cb_UserTopic(Kinova::Api::Base::UserNotification notif) override;
        virtual bool OnNotificationControllerTopic(kortex_driver::srv::OnNotificationControllerTopic::Request  &req, kortex_driver::srv::OnNotificationControllerTopic::Response &res) override;
        virtual void cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif) override;
        virtual bool OnNotificationActionTopic(kortex_driver::srv::OnNotificationActionTopic::Request  &req, kortex_driver::srv::OnNotificationActionTopic::Response &res) override;
        virtual void cb_ActionTopic(Kinova::Api::Base::ActionNotification notif) override;
        virtual bool OnNotificationRobotEventTopic(kortex_driver::srv::OnNotificationRobotEventTopic::Request  &req, kortex_driver::srv::OnNotificationRobotEventTopic::Response &res) override;
        virtual void cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif) override;
        virtual bool PlayCartesianTrajectory(kortex_driver::srv::PlayCartesianTrajectory::Request  &req, kortex_driver::srv::PlayCartesianTrajectory::Response &res) override;
        virtual bool PlayCartesianTrajectoryPosition(kortex_driver::srv::PlayCartesianTrajectoryPosition::Request  &req, kortex_driver::srv::PlayCartesianTrajectoryPosition::Response &res) override;
        virtual bool PlayCartesianTrajectoryOrientation(kortex_driver::srv::PlayCartesianTrajectoryOrientation::Request  &req, kortex_driver::srv::PlayCartesianTrajectoryOrientation::Response &res) override;
        virtual bool Stop(kortex_driver::srv::Stop::Request  &req, kortex_driver::srv::Stop::Response &res) override;
        virtual bool GetMeasuredCartesianPose(kortex_driver::srv::GetMeasuredCartesianPose::Request  &req, kortex_driver::srv::GetMeasuredCartesianPose::Response &res) override;
        virtual bool SendWrenchCommand(kortex_driver::srv::SendWrenchCommand::Request  &req, kortex_driver::srv::SendWrenchCommand::Response &res) override;
        virtual bool SendWrenchJoystickCommand(kortex_driver::srv::SendWrenchJoystickCommand::Request  &req, kortex_driver::srv::SendWrenchJoystickCommand::Response &res) override;
        virtual bool SendTwistJoystickCommand(kortex_driver::srv::SendTwistJoystickCommand::Request  &req, kortex_driver::srv::SendTwistJoystickCommand::Response &res) override;
        virtual bool SendTwistCommand(kortex_driver::srv::SendTwistCommand::Request  &req, kortex_driver::srv::SendTwistCommand::Response &res) override;
        virtual bool PlayJointTrajectory(kortex_driver::srv::PlayJointTrajectory::Request  &req, kortex_driver::srv::PlayJointTrajectory::Response &res) override;
        virtual bool PlaySelectedJointTrajectory(kortex_driver::srv::PlaySelectedJointTrajectory::Request  &req, kortex_driver::srv::PlaySelectedJointTrajectory::Response &res) override;
        virtual bool GetMeasuredJointAngles(kortex_driver::srv::GetMeasuredJointAngles::Request  &req, kortex_driver::srv::GetMeasuredJointAngles::Response &res) override;
        virtual bool SendJointSpeedsCommand(kortex_driver::srv::SendJointSpeedsCommand::Request  &req, kortex_driver::srv::SendJointSpeedsCommand::Response &res) override;
        virtual bool SendSelectedJointSpeedCommand(kortex_driver::srv::SendSelectedJointSpeedCommand::Request  &req, kortex_driver::srv::SendSelectedJointSpeedCommand::Response &res) override;
        virtual bool SendGripperCommand(kortex_driver::srv::SendGripperCommand::Request  &req, kortex_driver::srv::SendGripperCommand::Response &res) override;
        virtual bool GetMeasuredGripperMovement(kortex_driver::srv::GetMeasuredGripperMovement::Request  &req, kortex_driver::srv::GetMeasuredGripperMovement::Response &res) override;
        virtual bool SetAdmittance(kortex_driver::srv::SetAdmittance::Request  &req, kortex_driver::srv::SetAdmittance::Response &res) override;
        virtual bool SetOperatingMode(kortex_driver::srv::SetOperatingMode::Request  &req, kortex_driver::srv::SetOperatingMode::Response &res) override;
        virtual bool ApplyEmergencyStop(kortex_driver::srv::ApplyEmergencyStop::Request  &req, kortex_driver::srv::ApplyEmergencyStop::Response &res) override;
        virtual bool Base_ClearFaults(kortex_driver::srv::BaseClearFaults::Request  &req, kortex_driver::srv::BaseClearFaults::Response &res) override;
        virtual bool Base_GetControlMode(kortex_driver::srv::BaseGetControlMode::Request  &req, kortex_driver::srv::BaseGetControlMode::Response &res) override;
        virtual bool GetOperatingMode(kortex_driver::srv::GetOperatingMode::Request  &req, kortex_driver::srv::GetOperatingMode::Response &res) override;
        virtual bool SetServoingMode(kortex_driver::srv::SetServoingMode::Request  &req, kortex_driver::srv::SetServoingMode::Response &res) override;
        virtual bool GetServoingMode(kortex_driver::srv::GetServoingMode::Request  &req, kortex_driver::srv::GetServoingMode::Response &res) override;
        virtual bool OnNotificationServoingModeTopic(kortex_driver::srv::OnNotificationServoingModeTopic::Request  &req, kortex_driver::srv::OnNotificationServoingModeTopic::Response &res) override;
        virtual void cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif) override;
        virtual bool RestoreFactorySettings(kortex_driver::srv::RestoreFactorySettings::Request  &req, kortex_driver::srv::RestoreFactorySettings::Response &res) override;
        virtual bool OnNotificationFactoryTopic(kortex_driver::srv::OnNotificationFactoryTopic::Request  &req, kortex_driver::srv::OnNotificationFactoryTopic::Response &res) override;
        virtual void cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif) override;
        virtual bool GetAllConnectedControllers(kortex_driver::srv::GetAllConnectedControllers::Request  &req, kortex_driver::srv::GetAllConnectedControllers::Response &res) override;
        virtual bool GetControllerState(kortex_driver::srv::GetControllerState::Request  &req, kortex_driver::srv::GetControllerState::Response &res) override;
        virtual bool GetActuatorCount(kortex_driver::srv::GetActuatorCount::Request  &req, kortex_driver::srv::GetActuatorCount::Response &res) override;
        virtual bool StartWifiScan(kortex_driver::srv::StartWifiScan::Request  &req, kortex_driver::srv::StartWifiScan::Response &res) override;
        virtual bool GetConfiguredWifi(kortex_driver::srv::GetConfiguredWifi::Request  &req, kortex_driver::srv::GetConfiguredWifi::Response &res) override;
        virtual bool OnNotificationNetworkTopic(kortex_driver::srv::OnNotificationNetworkTopic::Request  &req, kortex_driver::srv::OnNotificationNetworkTopic::Response &res) override;
        virtual void cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif) override;
        virtual bool GetArmState(kortex_driver::srv::GetArmState::Request  &req, kortex_driver::srv::GetArmState::Response &res) override;
        virtual bool OnNotificationArmStateTopic(kortex_driver::srv::OnNotificationArmStateTopic::Request  &req, kortex_driver::srv::OnNotificationArmStateTopic::Response &res) override;
        virtual void cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif) override;
        virtual bool GetIPv4Information(kortex_driver::srv::GetIPv4Information::Request  &req, kortex_driver::srv::GetIPv4Information::Response &res) override;
        virtual bool SetWifiCountryCode(kortex_driver::srv::SetWifiCountryCode::Request  &req, kortex_driver::srv::SetWifiCountryCode::Response &res) override;
        virtual bool GetWifiCountryCode(kortex_driver::srv::GetWifiCountryCode::Request  &req, kortex_driver::srv::GetWifiCountryCode::Response &res) override;
        virtual bool Base_SetCapSenseConfig(kortex_driver::srv::BaseSetCapSenseConfig::Request  &req, kortex_driver::srv::BaseSetCapSenseConfig::Response &res) override;
        virtual bool Base_GetCapSenseConfig(kortex_driver::srv::BaseGetCapSenseConfig::Request  &req, kortex_driver::srv::BaseGetCapSenseConfig::Response &res) override;
        virtual bool GetAllJointsSpeedHardLimitation(kortex_driver::srv::GetAllJointsSpeedHardLimitation::Request  &req, kortex_driver::srv::GetAllJointsSpeedHardLimitation::Response &res) override;
        virtual bool GetAllJointsTorqueHardLimitation(kortex_driver::srv::GetAllJointsTorqueHardLimitation::Request  &req, kortex_driver::srv::GetAllJointsTorqueHardLimitation::Response &res) override;
        virtual bool GetTwistHardLimitation(kortex_driver::srv::GetTwistHardLimitation::Request  &req, kortex_driver::srv::GetTwistHardLimitation::Response &res) override;
        virtual bool GetWrenchHardLimitation(kortex_driver::srv::GetWrenchHardLimitation::Request  &req, kortex_driver::srv::GetWrenchHardLimitation::Response &res) override;
        virtual bool SendJointSpeedsJoystickCommand(kortex_driver::srv::SendJointSpeedsJoystickCommand::Request  &req, kortex_driver::srv::SendJointSpeedsJoystickCommand::Response &res) override;
        virtual bool SendSelectedJointSpeedJoystickCommand(kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Request  &req, kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Response &res) override;
        virtual bool EnableBridge(kortex_driver::srv::EnableBridge::Request  &req, kortex_driver::srv::EnableBridge::Response &res) override;
        virtual bool DisableBridge(kortex_driver::srv::DisableBridge::Request  &req, kortex_driver::srv::DisableBridge::Response &res) override;
        virtual bool GetBridgeList(kortex_driver::srv::GetBridgeList::Request  &req, kortex_driver::srv::GetBridgeList::Response &res) override;
        virtual bool GetBridgeConfig(kortex_driver::srv::GetBridgeConfig::Request  &req, kortex_driver::srv::GetBridgeConfig::Response &res) override;
        virtual bool PlayPreComputedJointTrajectory(kortex_driver::srv::PlayPreComputedJointTrajectory::Request  &req, kortex_driver::srv::PlayPreComputedJointTrajectory::Response &res) override;
        virtual bool GetProductConfiguration(kortex_driver::srv::GetProductConfiguration::Request  &req, kortex_driver::srv::GetProductConfiguration::Response &res) override;
        virtual bool UpdateEndEffectorTypeConfiguration(kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Request  &req, kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Response &res) override;
        virtual bool RestoreFactoryProductConfiguration(kortex_driver::srv::RestoreFactoryProductConfiguration::Request  &req, kortex_driver::srv::RestoreFactoryProductConfiguration::Response &res) override;
        virtual bool GetTrajectoryErrorReport(kortex_driver::srv::GetTrajectoryErrorReport::Request  &req, kortex_driver::srv::GetTrajectoryErrorReport::Response &res) override;
        virtual bool GetAllJointsSpeedSoftLimitation(kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Request  &req, kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Response &res) override;
        virtual bool GetAllJointsTorqueSoftLimitation(kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Request  &req, kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Response &res) override;
        virtual bool GetTwistSoftLimitation(kortex_driver::srv::GetTwistSoftLimitation::Request  &req, kortex_driver::srv::GetTwistSoftLimitation::Response &res) override;
        virtual bool GetWrenchSoftLimitation(kortex_driver::srv::GetWrenchSoftLimitation::Request  &req, kortex_driver::srv::GetWrenchSoftLimitation::Response &res) override;
        virtual bool SetControllerConfigurationMode(kortex_driver::srv::SetControllerConfigurationMode::Request  &req, kortex_driver::srv::SetControllerConfigurationMode::Response &res) override;
        virtual bool GetControllerConfigurationMode(kortex_driver::srv::GetControllerConfigurationMode::Request  &req, kortex_driver::srv::GetControllerConfigurationMode::Response &res) override;
        virtual bool StartTeaching(kortex_driver::srv::StartTeaching::Request  &req, kortex_driver::srv::StartTeaching::Response &res) override;
        virtual bool StopTeaching(kortex_driver::srv::StopTeaching::Request  &req, kortex_driver::srv::StopTeaching::Response &res) override;
        virtual bool AddSequenceTasks(kortex_driver::srv::AddSequenceTasks::Request  &req, kortex_driver::srv::AddSequenceTasks::Response &res) override;
        virtual bool UpdateSequenceTask(kortex_driver::srv::UpdateSequenceTask::Request  &req, kortex_driver::srv::UpdateSequenceTask::Response &res) override;
        virtual bool SwapSequenceTasks(kortex_driver::srv::SwapSequenceTasks::Request  &req, kortex_driver::srv::SwapSequenceTasks::Response &res) override;
        virtual bool ReadSequenceTask(kortex_driver::srv::ReadSequenceTask::Request  &req, kortex_driver::srv::ReadSequenceTask::Response &res) override;
        virtual bool ReadAllSequenceTasks(kortex_driver::srv::ReadAllSequenceTasks::Request  &req, kortex_driver::srv::ReadAllSequenceTasks::Response &res) override;
        virtual bool DeleteSequenceTask(kortex_driver::srv::DeleteSequenceTask::Request  &req, kortex_driver::srv::DeleteSequenceTask::Response &res) override;
        virtual bool DeleteAllSequenceTasks(kortex_driver::srv::DeleteAllSequenceTasks::Request  &req, kortex_driver::srv::DeleteAllSequenceTasks::Response &res) override;
        virtual bool TakeSnapshot(kortex_driver::srv::TakeSnapshot::Request  &req, kortex_driver::srv::TakeSnapshot::Response &res) override;
        virtual bool GetFirmwareBundleVersions(kortex_driver::srv::GetFirmwareBundleVersions::Request  &req, kortex_driver::srv::GetFirmwareBundleVersions::Response &res) override;
        virtual bool ExecuteWaypointTrajectory(kortex_driver::srv::ExecuteWaypointTrajectory::Request  &req, kortex_driver::srv::ExecuteWaypointTrajectory::Response &res) override;
        virtual bool MoveSequenceTask(kortex_driver::srv::MoveSequenceTask::Request  &req, kortex_driver::srv::MoveSequenceTask::Response &res) override;
        virtual bool DuplicateMapping(kortex_driver::srv::DuplicateMapping::Request  &req, kortex_driver::srv::DuplicateMapping::Response &res) override;
        virtual bool DuplicateMap(kortex_driver::srv::DuplicateMap::Request  &req, kortex_driver::srv::DuplicateMap::Response &res) override;
        virtual bool SetControllerConfiguration(kortex_driver::srv::SetControllerConfiguration::Request  &req, kortex_driver::srv::SetControllerConfiguration::Response &res) override;
        virtual bool GetControllerConfiguration(kortex_driver::srv::GetControllerConfiguration::Request  &req, kortex_driver::srv::GetControllerConfiguration::Response &res) override;
        virtual bool GetAllControllerConfigurations(kortex_driver::srv::GetAllControllerConfigurations::Request  &req, kortex_driver::srv::GetAllControllerConfigurations::Response &res) override;
        virtual bool ComputeForwardKinematics(kortex_driver::srv::ComputeForwardKinematics::Request  &req, kortex_driver::srv::ComputeForwardKinematics::Response &res) override;
        virtual bool ComputeInverseKinematics(kortex_driver::srv::ComputeInverseKinematics::Request  &req, kortex_driver::srv::ComputeInverseKinematics::Response &res) override;
        virtual bool ValidateWaypointList(kortex_driver::srv::ValidateWaypointList::Request  &req, kortex_driver::srv::ValidateWaypointList::Response &res) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::Base::BaseClient* m_base;
};
#endif
