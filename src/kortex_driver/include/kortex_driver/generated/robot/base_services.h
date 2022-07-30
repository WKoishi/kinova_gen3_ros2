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

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) override;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) override;
        virtual bool CreateUserProfile(const std::shared_ptr<kortex_driver::srv::CreateUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::CreateUserProfile::Response> res) override;
        virtual bool UpdateUserProfile(const std::shared_ptr<kortex_driver::srv::UpdateUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::UpdateUserProfile::Response> res) override;
        virtual bool ReadUserProfile(const std::shared_ptr<kortex_driver::srv::ReadUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::ReadUserProfile::Response> res) override;
        virtual bool DeleteUserProfile(const std::shared_ptr<kortex_driver::srv::DeleteUserProfile::Request> req, std::shared_ptr<kortex_driver::srv::DeleteUserProfile::Response> res) override;
        virtual bool ReadAllUserProfiles(const std::shared_ptr<kortex_driver::srv::ReadAllUserProfiles::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllUserProfiles::Response> res) override;
        virtual bool ReadAllUsers(const std::shared_ptr<kortex_driver::srv::ReadAllUsers::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllUsers::Response> res) override;
        virtual bool ChangePassword(const std::shared_ptr<kortex_driver::srv::ChangePassword::Request> req, std::shared_ptr<kortex_driver::srv::ChangePassword::Response> res) override;
        virtual bool CreateSequence(const std::shared_ptr<kortex_driver::srv::CreateSequence::Request> req, std::shared_ptr<kortex_driver::srv::CreateSequence::Response> res) override;
        virtual bool UpdateSequence(const std::shared_ptr<kortex_driver::srv::UpdateSequence::Request> req, std::shared_ptr<kortex_driver::srv::UpdateSequence::Response> res) override;
        virtual bool ReadSequence(const std::shared_ptr<kortex_driver::srv::ReadSequence::Request> req, std::shared_ptr<kortex_driver::srv::ReadSequence::Response> res) override;
        virtual bool DeleteSequence(const std::shared_ptr<kortex_driver::srv::DeleteSequence::Request> req, std::shared_ptr<kortex_driver::srv::DeleteSequence::Response> res) override;
        virtual bool ReadAllSequences(const std::shared_ptr<kortex_driver::srv::ReadAllSequences::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllSequences::Response> res) override;
        virtual bool PlaySequence(const std::shared_ptr<kortex_driver::srv::PlaySequence::Request> req, std::shared_ptr<kortex_driver::srv::PlaySequence::Response> res) override;
        virtual bool PlayAdvancedSequence(const std::shared_ptr<kortex_driver::srv::PlayAdvancedSequence::Request> req, std::shared_ptr<kortex_driver::srv::PlayAdvancedSequence::Response> res) override;
        virtual bool StopSequence(const std::shared_ptr<kortex_driver::srv::StopSequence::Request> req, std::shared_ptr<kortex_driver::srv::StopSequence::Response> res) override;
        virtual bool PauseSequence(const std::shared_ptr<kortex_driver::srv::PauseSequence::Request> req, std::shared_ptr<kortex_driver::srv::PauseSequence::Response> res) override;
        virtual bool ResumeSequence(const std::shared_ptr<kortex_driver::srv::ResumeSequence::Request> req, std::shared_ptr<kortex_driver::srv::ResumeSequence::Response> res) override;
        virtual bool CreateProtectionZone(const std::shared_ptr<kortex_driver::srv::CreateProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::CreateProtectionZone::Response> res) override;
        virtual bool UpdateProtectionZone(const std::shared_ptr<kortex_driver::srv::UpdateProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::UpdateProtectionZone::Response> res) override;
        virtual bool ReadProtectionZone(const std::shared_ptr<kortex_driver::srv::ReadProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::ReadProtectionZone::Response> res) override;
        virtual bool DeleteProtectionZone(const std::shared_ptr<kortex_driver::srv::DeleteProtectionZone::Request> req, std::shared_ptr<kortex_driver::srv::DeleteProtectionZone::Response> res) override;
        virtual bool ReadAllProtectionZones(const std::shared_ptr<kortex_driver::srv::ReadAllProtectionZones::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllProtectionZones::Response> res) override;
        virtual bool CreateMapping(const std::shared_ptr<kortex_driver::srv::CreateMapping::Request> req, std::shared_ptr<kortex_driver::srv::CreateMapping::Response> res) override;
        virtual bool ReadMapping(const std::shared_ptr<kortex_driver::srv::ReadMapping::Request> req, std::shared_ptr<kortex_driver::srv::ReadMapping::Response> res) override;
        virtual bool UpdateMapping(const std::shared_ptr<kortex_driver::srv::UpdateMapping::Request> req, std::shared_ptr<kortex_driver::srv::UpdateMapping::Response> res) override;
        virtual bool DeleteMapping(const std::shared_ptr<kortex_driver::srv::DeleteMapping::Request> req, std::shared_ptr<kortex_driver::srv::DeleteMapping::Response> res) override;
        virtual bool ReadAllMappings(const std::shared_ptr<kortex_driver::srv::ReadAllMappings::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllMappings::Response> res) override;
        virtual bool CreateMap(const std::shared_ptr<kortex_driver::srv::CreateMap::Request> req, std::shared_ptr<kortex_driver::srv::CreateMap::Response> res) override;
        virtual bool ReadMap(const std::shared_ptr<kortex_driver::srv::ReadMap::Request> req, std::shared_ptr<kortex_driver::srv::ReadMap::Response> res) override;
        virtual bool UpdateMap(const std::shared_ptr<kortex_driver::srv::UpdateMap::Request> req, std::shared_ptr<kortex_driver::srv::UpdateMap::Response> res) override;
        virtual bool DeleteMap(const std::shared_ptr<kortex_driver::srv::DeleteMap::Request> req, std::shared_ptr<kortex_driver::srv::DeleteMap::Response> res) override;
        virtual bool ReadAllMaps(const std::shared_ptr<kortex_driver::srv::ReadAllMaps::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllMaps::Response> res) override;
        virtual bool ActivateMap(const std::shared_ptr<kortex_driver::srv::ActivateMap::Request> req, std::shared_ptr<kortex_driver::srv::ActivateMap::Response> res) override;
        virtual bool CreateAction(const std::shared_ptr<kortex_driver::srv::CreateAction::Request> req, std::shared_ptr<kortex_driver::srv::CreateAction::Response> res) override;
        virtual bool ReadAction(const std::shared_ptr<kortex_driver::srv::ReadAction::Request> req, std::shared_ptr<kortex_driver::srv::ReadAction::Response> res) override;
        virtual bool ReadAllActions(const std::shared_ptr<kortex_driver::srv::ReadAllActions::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllActions::Response> res) override;
        virtual bool DeleteAction(const std::shared_ptr<kortex_driver::srv::DeleteAction::Request> req, std::shared_ptr<kortex_driver::srv::DeleteAction::Response> res) override;
        virtual bool UpdateAction(const std::shared_ptr<kortex_driver::srv::UpdateAction::Request> req, std::shared_ptr<kortex_driver::srv::UpdateAction::Response> res) override;
        virtual bool ExecuteActionFromReference(const std::shared_ptr<kortex_driver::srv::ExecuteActionFromReference::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteActionFromReference::Response> res) override;
        virtual bool ExecuteAction(const std::shared_ptr<kortex_driver::srv::ExecuteAction::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteAction::Response> res) override;
        virtual bool PauseAction(const std::shared_ptr<kortex_driver::srv::PauseAction::Request> req, std::shared_ptr<kortex_driver::srv::PauseAction::Response> res) override;
        virtual bool StopAction(const std::shared_ptr<kortex_driver::srv::StopAction::Request> req, std::shared_ptr<kortex_driver::srv::StopAction::Response> res) override;
        virtual bool ResumeAction(const std::shared_ptr<kortex_driver::srv::ResumeAction::Request> req, std::shared_ptr<kortex_driver::srv::ResumeAction::Response> res) override;
        virtual bool GetIPv4Configuration(const std::shared_ptr<kortex_driver::srv::GetIPv4Configuration::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Configuration::Response> res) override;
        virtual bool SetIPv4Configuration(const std::shared_ptr<kortex_driver::srv::SetIPv4Configuration::Request> req, std::shared_ptr<kortex_driver::srv::SetIPv4Configuration::Response> res) override;
        virtual bool SetCommunicationInterfaceEnable(const std::shared_ptr<kortex_driver::srv::SetCommunicationInterfaceEnable::Request> req, std::shared_ptr<kortex_driver::srv::SetCommunicationInterfaceEnable::Response> res) override;
        virtual bool IsCommunicationInterfaceEnable(const std::shared_ptr<kortex_driver::srv::IsCommunicationInterfaceEnable::Request> req, std::shared_ptr<kortex_driver::srv::IsCommunicationInterfaceEnable::Response> res) override;
        virtual bool GetAvailableWifi(const std::shared_ptr<kortex_driver::srv::GetAvailableWifi::Request> req, std::shared_ptr<kortex_driver::srv::GetAvailableWifi::Response> res) override;
        virtual bool GetWifiInformation(const std::shared_ptr<kortex_driver::srv::GetWifiInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetWifiInformation::Response> res) override;
        virtual bool AddWifiConfiguration(const std::shared_ptr<kortex_driver::srv::AddWifiConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::AddWifiConfiguration::Response> res) override;
        virtual bool DeleteWifiConfiguration(const std::shared_ptr<kortex_driver::srv::DeleteWifiConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::DeleteWifiConfiguration::Response> res) override;
        virtual bool GetAllConfiguredWifis(const std::shared_ptr<kortex_driver::srv::GetAllConfiguredWifis::Request> req, std::shared_ptr<kortex_driver::srv::GetAllConfiguredWifis::Response> res) override;
        virtual bool ConnectWifi(const std::shared_ptr<kortex_driver::srv::ConnectWifi::Request> req, std::shared_ptr<kortex_driver::srv::ConnectWifi::Response> res) override;
        virtual bool DisconnectWifi(const std::shared_ptr<kortex_driver::srv::DisconnectWifi::Request> req, std::shared_ptr<kortex_driver::srv::DisconnectWifi::Response> res) override;
        virtual bool GetConnectedWifiInformation(const std::shared_ptr<kortex_driver::srv::GetConnectedWifiInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetConnectedWifiInformation::Response> res) override;
        virtual bool Base_Unsubscribe(const std::shared_ptr<kortex_driver::srv::BaseUnsubscribe::Request> req, std::shared_ptr<kortex_driver::srv::BaseUnsubscribe::Response> res) override;
        virtual bool OnNotificationConfigurationChangeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationConfigurationChangeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationConfigurationChangeTopic::Response> res) override;
        virtual void cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif) override;
        virtual bool OnNotificationMappingInfoTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationMappingInfoTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationMappingInfoTopic::Response> res) override;
        virtual void cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif) override;
        virtual bool Base_OnNotificationControlModeTopic(const std::shared_ptr<kortex_driver::srv::BaseOnNotificationControlModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::BaseOnNotificationControlModeTopic::Response> res) override;
        virtual void cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif) override;
        virtual bool OnNotificationOperatingModeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationOperatingModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationOperatingModeTopic::Response> res) override;
        virtual void cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif) override;
        virtual bool OnNotificationSequenceInfoTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationSequenceInfoTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationSequenceInfoTopic::Response> res) override;
        virtual void cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif) override;
        virtual bool OnNotificationProtectionZoneTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationProtectionZoneTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationProtectionZoneTopic::Response> res) override;
        virtual void cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif) override;
        virtual bool OnNotificationUserTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationUserTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationUserTopic::Response> res) override;
        virtual void cb_UserTopic(Kinova::Api::Base::UserNotification notif) override;
        virtual bool OnNotificationControllerTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationControllerTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationControllerTopic::Response> res) override;
        virtual void cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif) override;
        virtual bool OnNotificationActionTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationActionTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationActionTopic::Response> res) override;
        virtual void cb_ActionTopic(Kinova::Api::Base::ActionNotification notif) override;
        virtual bool OnNotificationRobotEventTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationRobotEventTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationRobotEventTopic::Response> res) override;
        virtual void cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif) override;
        virtual bool PlayCartesianTrajectory(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectory::Response> res) override;
        virtual bool PlayCartesianTrajectoryPosition(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryPosition::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryPosition::Response> res) override;
        virtual bool PlayCartesianTrajectoryOrientation(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryOrientation::Request> req, std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectoryOrientation::Response> res) override;
        virtual bool Stop(const std::shared_ptr<kortex_driver::srv::Stop::Request> req, std::shared_ptr<kortex_driver::srv::Stop::Response> res) override;
        virtual bool GetMeasuredCartesianPose(const std::shared_ptr<kortex_driver::srv::GetMeasuredCartesianPose::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredCartesianPose::Response> res) override;
        virtual bool SendWrenchCommand(const std::shared_ptr<kortex_driver::srv::SendWrenchCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendWrenchCommand::Response> res) override;
        virtual bool SendWrenchJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendWrenchJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendWrenchJoystickCommand::Response> res) override;
        virtual bool SendTwistJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendTwistJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendTwistJoystickCommand::Response> res) override;
        virtual bool SendTwistCommand(const std::shared_ptr<kortex_driver::srv::SendTwistCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendTwistCommand::Response> res) override;
        virtual bool PlayJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlayJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayJointTrajectory::Response> res) override;
        virtual bool PlaySelectedJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlaySelectedJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlaySelectedJointTrajectory::Response> res) override;
        virtual bool GetMeasuredJointAngles(const std::shared_ptr<kortex_driver::srv::GetMeasuredJointAngles::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredJointAngles::Response> res) override;
        virtual bool SendJointSpeedsCommand(const std::shared_ptr<kortex_driver::srv::SendJointSpeedsCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendJointSpeedsCommand::Response> res) override;
        virtual bool SendSelectedJointSpeedCommand(const std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedCommand::Response> res) override;
        virtual bool SendGripperCommand(const std::shared_ptr<kortex_driver::srv::SendGripperCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendGripperCommand::Response> res) override;
        virtual bool GetMeasuredGripperMovement(const std::shared_ptr<kortex_driver::srv::GetMeasuredGripperMovement::Request> req, std::shared_ptr<kortex_driver::srv::GetMeasuredGripperMovement::Response> res) override;
        virtual bool SetAdmittance(const std::shared_ptr<kortex_driver::srv::SetAdmittance::Request> req, std::shared_ptr<kortex_driver::srv::SetAdmittance::Response> res) override;
        virtual bool SetOperatingMode(const std::shared_ptr<kortex_driver::srv::SetOperatingMode::Request> req, std::shared_ptr<kortex_driver::srv::SetOperatingMode::Response> res) override;
        virtual bool ApplyEmergencyStop(const std::shared_ptr<kortex_driver::srv::ApplyEmergencyStop::Request> req, std::shared_ptr<kortex_driver::srv::ApplyEmergencyStop::Response> res) override;
        virtual bool Base_ClearFaults(const std::shared_ptr<kortex_driver::srv::BaseClearFaults::Request> req, std::shared_ptr<kortex_driver::srv::BaseClearFaults::Response> res) override;
        virtual bool Base_GetControlMode(const std::shared_ptr<kortex_driver::srv::BaseGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::BaseGetControlMode::Response> res) override;
        virtual bool GetOperatingMode(const std::shared_ptr<kortex_driver::srv::GetOperatingMode::Request> req, std::shared_ptr<kortex_driver::srv::GetOperatingMode::Response> res) override;
        virtual bool SetServoingMode(const std::shared_ptr<kortex_driver::srv::SetServoingMode::Request> req, std::shared_ptr<kortex_driver::srv::SetServoingMode::Response> res) override;
        virtual bool GetServoingMode(const std::shared_ptr<kortex_driver::srv::GetServoingMode::Request> req, std::shared_ptr<kortex_driver::srv::GetServoingMode::Response> res) override;
        virtual bool OnNotificationServoingModeTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationServoingModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationServoingModeTopic::Response> res) override;
        virtual void cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif) override;
        virtual bool RestoreFactorySettings(const std::shared_ptr<kortex_driver::srv::RestoreFactorySettings::Request> req, std::shared_ptr<kortex_driver::srv::RestoreFactorySettings::Response> res) override;
        virtual bool OnNotificationFactoryTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationFactoryTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationFactoryTopic::Response> res) override;
        virtual void cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif) override;
        virtual bool GetAllConnectedControllers(const std::shared_ptr<kortex_driver::srv::GetAllConnectedControllers::Request> req, std::shared_ptr<kortex_driver::srv::GetAllConnectedControllers::Response> res) override;
        virtual bool GetControllerState(const std::shared_ptr<kortex_driver::srv::GetControllerState::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerState::Response> res) override;
        virtual bool GetActuatorCount(const std::shared_ptr<kortex_driver::srv::GetActuatorCount::Request> req, std::shared_ptr<kortex_driver::srv::GetActuatorCount::Response> res) override;
        virtual bool StartWifiScan(const std::shared_ptr<kortex_driver::srv::StartWifiScan::Request> req, std::shared_ptr<kortex_driver::srv::StartWifiScan::Response> res) override;
        virtual bool GetConfiguredWifi(const std::shared_ptr<kortex_driver::srv::GetConfiguredWifi::Request> req, std::shared_ptr<kortex_driver::srv::GetConfiguredWifi::Response> res) override;
        virtual bool OnNotificationNetworkTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationNetworkTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationNetworkTopic::Response> res) override;
        virtual void cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif) override;
        virtual bool GetArmState(const std::shared_ptr<kortex_driver::srv::GetArmState::Request> req, std::shared_ptr<kortex_driver::srv::GetArmState::Response> res) override;
        virtual bool OnNotificationArmStateTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationArmStateTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationArmStateTopic::Response> res) override;
        virtual void cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif) override;
        virtual bool GetIPv4Information(const std::shared_ptr<kortex_driver::srv::GetIPv4Information::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Information::Response> res) override;
        virtual bool SetWifiCountryCode(const std::shared_ptr<kortex_driver::srv::SetWifiCountryCode::Request> req, std::shared_ptr<kortex_driver::srv::SetWifiCountryCode::Response> res) override;
        virtual bool GetWifiCountryCode(const std::shared_ptr<kortex_driver::srv::GetWifiCountryCode::Request> req, std::shared_ptr<kortex_driver::srv::GetWifiCountryCode::Response> res) override;
        virtual bool Base_SetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::BaseSetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::BaseSetCapSenseConfig::Response> res) override;
        virtual bool Base_GetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::BaseGetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::BaseGetCapSenseConfig::Response> res) override;
        virtual bool GetAllJointsSpeedHardLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedHardLimitation::Response> res) override;
        virtual bool GetAllJointsTorqueHardLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueHardLimitation::Response> res) override;
        virtual bool GetTwistHardLimitation(const std::shared_ptr<kortex_driver::srv::GetTwistHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetTwistHardLimitation::Response> res) override;
        virtual bool GetWrenchHardLimitation(const std::shared_ptr<kortex_driver::srv::GetWrenchHardLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetWrenchHardLimitation::Response> res) override;
        virtual bool SendJointSpeedsJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendJointSpeedsJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendJointSpeedsJoystickCommand::Response> res) override;
        virtual bool SendSelectedJointSpeedJoystickCommand(const std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Request> req, std::shared_ptr<kortex_driver::srv::SendSelectedJointSpeedJoystickCommand::Response> res) override;
        virtual bool EnableBridge(const std::shared_ptr<kortex_driver::srv::EnableBridge::Request> req, std::shared_ptr<kortex_driver::srv::EnableBridge::Response> res) override;
        virtual bool DisableBridge(const std::shared_ptr<kortex_driver::srv::DisableBridge::Request> req, std::shared_ptr<kortex_driver::srv::DisableBridge::Response> res) override;
        virtual bool GetBridgeList(const std::shared_ptr<kortex_driver::srv::GetBridgeList::Request> req, std::shared_ptr<kortex_driver::srv::GetBridgeList::Response> res) override;
        virtual bool GetBridgeConfig(const std::shared_ptr<kortex_driver::srv::GetBridgeConfig::Request> req, std::shared_ptr<kortex_driver::srv::GetBridgeConfig::Response> res) override;
        virtual bool PlayPreComputedJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlayPreComputedJointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::PlayPreComputedJointTrajectory::Response> res) override;
        virtual bool GetProductConfiguration(const std::shared_ptr<kortex_driver::srv::GetProductConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetProductConfiguration::Response> res) override;
        virtual bool UpdateEndEffectorTypeConfiguration(const std::shared_ptr<kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::UpdateEndEffectorTypeConfiguration::Response> res) override;
        virtual bool RestoreFactoryProductConfiguration(const std::shared_ptr<kortex_driver::srv::RestoreFactoryProductConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::RestoreFactoryProductConfiguration::Response> res) override;
        virtual bool GetTrajectoryErrorReport(const std::shared_ptr<kortex_driver::srv::GetTrajectoryErrorReport::Request> req, std::shared_ptr<kortex_driver::srv::GetTrajectoryErrorReport::Response> res) override;
        virtual bool GetAllJointsSpeedSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsSpeedSoftLimitation::Response> res) override;
        virtual bool GetAllJointsTorqueSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllJointsTorqueSoftLimitation::Response> res) override;
        virtual bool GetTwistSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetTwistSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetTwistSoftLimitation::Response> res) override;
        virtual bool GetWrenchSoftLimitation(const std::shared_ptr<kortex_driver::srv::GetWrenchSoftLimitation::Request> req, std::shared_ptr<kortex_driver::srv::GetWrenchSoftLimitation::Response> res) override;
        virtual bool SetControllerConfigurationMode(const std::shared_ptr<kortex_driver::srv::SetControllerConfigurationMode::Request> req, std::shared_ptr<kortex_driver::srv::SetControllerConfigurationMode::Response> res) override;
        virtual bool GetControllerConfigurationMode(const std::shared_ptr<kortex_driver::srv::GetControllerConfigurationMode::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerConfigurationMode::Response> res) override;
        virtual bool StartTeaching(const std::shared_ptr<kortex_driver::srv::StartTeaching::Request> req, std::shared_ptr<kortex_driver::srv::StartTeaching::Response> res) override;
        virtual bool StopTeaching(const std::shared_ptr<kortex_driver::srv::StopTeaching::Request> req, std::shared_ptr<kortex_driver::srv::StopTeaching::Response> res) override;
        virtual bool AddSequenceTasks(const std::shared_ptr<kortex_driver::srv::AddSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::AddSequenceTasks::Response> res) override;
        virtual bool UpdateSequenceTask(const std::shared_ptr<kortex_driver::srv::UpdateSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::UpdateSequenceTask::Response> res) override;
        virtual bool SwapSequenceTasks(const std::shared_ptr<kortex_driver::srv::SwapSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::SwapSequenceTasks::Response> res) override;
        virtual bool ReadSequenceTask(const std::shared_ptr<kortex_driver::srv::ReadSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::ReadSequenceTask::Response> res) override;
        virtual bool ReadAllSequenceTasks(const std::shared_ptr<kortex_driver::srv::ReadAllSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::ReadAllSequenceTasks::Response> res) override;
        virtual bool DeleteSequenceTask(const std::shared_ptr<kortex_driver::srv::DeleteSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::DeleteSequenceTask::Response> res) override;
        virtual bool DeleteAllSequenceTasks(const std::shared_ptr<kortex_driver::srv::DeleteAllSequenceTasks::Request> req, std::shared_ptr<kortex_driver::srv::DeleteAllSequenceTasks::Response> res) override;
        virtual bool TakeSnapshot(const std::shared_ptr<kortex_driver::srv::TakeSnapshot::Request> req, std::shared_ptr<kortex_driver::srv::TakeSnapshot::Response> res) override;
        virtual bool GetFirmwareBundleVersions(const std::shared_ptr<kortex_driver::srv::GetFirmwareBundleVersions::Request> req, std::shared_ptr<kortex_driver::srv::GetFirmwareBundleVersions::Response> res) override;
        virtual bool ExecuteWaypointTrajectory(const std::shared_ptr<kortex_driver::srv::ExecuteWaypointTrajectory::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteWaypointTrajectory::Response> res) override;
        virtual bool MoveSequenceTask(const std::shared_ptr<kortex_driver::srv::MoveSequenceTask::Request> req, std::shared_ptr<kortex_driver::srv::MoveSequenceTask::Response> res) override;
        virtual bool DuplicateMapping(const std::shared_ptr<kortex_driver::srv::DuplicateMapping::Request> req, std::shared_ptr<kortex_driver::srv::DuplicateMapping::Response> res) override;
        virtual bool DuplicateMap(const std::shared_ptr<kortex_driver::srv::DuplicateMap::Request> req, std::shared_ptr<kortex_driver::srv::DuplicateMap::Response> res) override;
        virtual bool SetControllerConfiguration(const std::shared_ptr<kortex_driver::srv::SetControllerConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetControllerConfiguration::Response> res) override;
        virtual bool GetControllerConfiguration(const std::shared_ptr<kortex_driver::srv::GetControllerConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetControllerConfiguration::Response> res) override;
        virtual bool GetAllControllerConfigurations(const std::shared_ptr<kortex_driver::srv::GetAllControllerConfigurations::Request> req, std::shared_ptr<kortex_driver::srv::GetAllControllerConfigurations::Response> res) override;
        virtual bool ComputeForwardKinematics(const std::shared_ptr<kortex_driver::srv::ComputeForwardKinematics::Request> req, std::shared_ptr<kortex_driver::srv::ComputeForwardKinematics::Response> res) override;
        virtual bool ComputeInverseKinematics(const std::shared_ptr<kortex_driver::srv::ComputeInverseKinematics::Request> req, std::shared_ptr<kortex_driver::srv::ComputeInverseKinematics::Response> res) override;
        virtual bool ValidateWaypointList(const std::shared_ptr<kortex_driver::srv::ValidateWaypointList::Request> req, std::shared_ptr<kortex_driver::srv::ValidateWaypointList::Response> res) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::Base::BaseClient* m_base;
};
#endif
