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
 
#ifndef _KORTEX_CONTROLCONFIG_SIMULATION_SERVICES_H_
#define _KORTEX_CONTROLCONFIG_SIMULATION_SERVICES_H_

#include "kortex_driver/generated/interfaces/controlconfig_services_interface.h"

using namespace std;

class ControlConfigSimulationServices : public IControlConfigServices
{
    public:
        ControlConfigSimulationServices(rclcpp::Node::SharedPtr node_handle);

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) override;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetGravityVector::Response>(const std::shared_ptr<kortex_driver::srv::SetGravityVector::Request>)> SetGravityVectorHandler = nullptr;
        virtual bool SetGravityVector(const std::shared_ptr<kortex_driver::srv::SetGravityVector::Request> req, std::shared_ptr<kortex_driver::srv::SetGravityVector::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetGravityVector::Response>(const std::shared_ptr<kortex_driver::srv::GetGravityVector::Request>)> GetGravityVectorHandler = nullptr;
        virtual bool GetGravityVector(const std::shared_ptr<kortex_driver::srv::GetGravityVector::Request> req, std::shared_ptr<kortex_driver::srv::GetGravityVector::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetPayloadInformation::Response>(const std::shared_ptr<kortex_driver::srv::SetPayloadInformation::Request>)> SetPayloadInformationHandler = nullptr;
        virtual bool SetPayloadInformation(const std::shared_ptr<kortex_driver::srv::SetPayloadInformation::Request> req, std::shared_ptr<kortex_driver::srv::SetPayloadInformation::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetPayloadInformation::Response>(const std::shared_ptr<kortex_driver::srv::GetPayloadInformation::Request>)> GetPayloadInformationHandler = nullptr;
        virtual bool GetPayloadInformation(const std::shared_ptr<kortex_driver::srv::GetPayloadInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetPayloadInformation::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetToolConfiguration::Response>(const std::shared_ptr<kortex_driver::srv::SetToolConfiguration::Request>)> SetToolConfigurationHandler = nullptr;
        virtual bool SetToolConfiguration(const std::shared_ptr<kortex_driver::srv::SetToolConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetToolConfiguration::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetToolConfiguration::Response>(const std::shared_ptr<kortex_driver::srv::GetToolConfiguration::Request>)> GetToolConfigurationHandler = nullptr;
        virtual bool GetToolConfiguration(const std::shared_ptr<kortex_driver::srv::GetToolConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetToolConfiguration::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::OnNotificationControlConfigurationTopic::Response>(const std::shared_ptr<kortex_driver::srv::OnNotificationControlConfigurationTopic::Request>)> OnNotificationControlConfigurationTopicHandler = nullptr;
        virtual bool OnNotificationControlConfigurationTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationControlConfigurationTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationControlConfigurationTopic::Response> res) override;
        virtual void cb_ControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification notif) override;
        std::function<std::shared_ptr<kortex_driver::srv::ControlConfigUnsubscribe::Response>(const std::shared_ptr<kortex_driver::srv::ControlConfigUnsubscribe::Request>)> ControlConfig_UnsubscribeHandler = nullptr;
        virtual bool ControlConfig_Unsubscribe(const std::shared_ptr<kortex_driver::srv::ControlConfigUnsubscribe::Request> req, std::shared_ptr<kortex_driver::srv::ControlConfigUnsubscribe::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetCartesianReferenceFrame::Response>(const std::shared_ptr<kortex_driver::srv::SetCartesianReferenceFrame::Request>)> SetCartesianReferenceFrameHandler = nullptr;
        virtual bool SetCartesianReferenceFrame(const std::shared_ptr<kortex_driver::srv::SetCartesianReferenceFrame::Request> req, std::shared_ptr<kortex_driver::srv::SetCartesianReferenceFrame::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetCartesianReferenceFrame::Response>(const std::shared_ptr<kortex_driver::srv::GetCartesianReferenceFrame::Request>)> GetCartesianReferenceFrameHandler = nullptr;
        virtual bool GetCartesianReferenceFrame(const std::shared_ptr<kortex_driver::srv::GetCartesianReferenceFrame::Request> req, std::shared_ptr<kortex_driver::srv::GetCartesianReferenceFrame::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ControlConfigGetControlMode::Response>(const std::shared_ptr<kortex_driver::srv::ControlConfigGetControlMode::Request>)> ControlConfig_GetControlModeHandler = nullptr;
        virtual bool ControlConfig_GetControlMode(const std::shared_ptr<kortex_driver::srv::ControlConfigGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::ControlConfigGetControlMode::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetJointSpeedSoftLimits::Response>(const std::shared_ptr<kortex_driver::srv::SetJointSpeedSoftLimits::Request>)> SetJointSpeedSoftLimitsHandler = nullptr;
        virtual bool SetJointSpeedSoftLimits(const std::shared_ptr<kortex_driver::srv::SetJointSpeedSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::SetJointSpeedSoftLimits::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetTwistLinearSoftLimit::Response>(const std::shared_ptr<kortex_driver::srv::SetTwistLinearSoftLimit::Request>)> SetTwistLinearSoftLimitHandler = nullptr;
        virtual bool SetTwistLinearSoftLimit(const std::shared_ptr<kortex_driver::srv::SetTwistLinearSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::SetTwistLinearSoftLimit::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetTwistAngularSoftLimit::Response>(const std::shared_ptr<kortex_driver::srv::SetTwistAngularSoftLimit::Request>)> SetTwistAngularSoftLimitHandler = nullptr;
        virtual bool SetTwistAngularSoftLimit(const std::shared_ptr<kortex_driver::srv::SetTwistAngularSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::SetTwistAngularSoftLimit::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetJointAccelerationSoftLimits::Response>(const std::shared_ptr<kortex_driver::srv::SetJointAccelerationSoftLimits::Request>)> SetJointAccelerationSoftLimitsHandler = nullptr;
        virtual bool SetJointAccelerationSoftLimits(const std::shared_ptr<kortex_driver::srv::SetJointAccelerationSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::SetJointAccelerationSoftLimits::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetKinematicHardLimits::Response>(const std::shared_ptr<kortex_driver::srv::GetKinematicHardLimits::Request>)> GetKinematicHardLimitsHandler = nullptr;
        virtual bool GetKinematicHardLimits(const std::shared_ptr<kortex_driver::srv::GetKinematicHardLimits::Request> req, std::shared_ptr<kortex_driver::srv::GetKinematicHardLimits::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetKinematicSoftLimits::Response>(const std::shared_ptr<kortex_driver::srv::GetKinematicSoftLimits::Request>)> GetKinematicSoftLimitsHandler = nullptr;
        virtual bool GetKinematicSoftLimits(const std::shared_ptr<kortex_driver::srv::GetKinematicSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::GetKinematicSoftLimits::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetAllKinematicSoftLimits::Response>(const std::shared_ptr<kortex_driver::srv::GetAllKinematicSoftLimits::Request>)> GetAllKinematicSoftLimitsHandler = nullptr;
        virtual bool GetAllKinematicSoftLimits(const std::shared_ptr<kortex_driver::srv::GetAllKinematicSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::GetAllKinematicSoftLimits::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetDesiredLinearTwist::Response>(const std::shared_ptr<kortex_driver::srv::SetDesiredLinearTwist::Request>)> SetDesiredLinearTwistHandler = nullptr;
        virtual bool SetDesiredLinearTwist(const std::shared_ptr<kortex_driver::srv::SetDesiredLinearTwist::Request> req, std::shared_ptr<kortex_driver::srv::SetDesiredLinearTwist::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetDesiredAngularTwist::Response>(const std::shared_ptr<kortex_driver::srv::SetDesiredAngularTwist::Request>)> SetDesiredAngularTwistHandler = nullptr;
        virtual bool SetDesiredAngularTwist(const std::shared_ptr<kortex_driver::srv::SetDesiredAngularTwist::Request> req, std::shared_ptr<kortex_driver::srv::SetDesiredAngularTwist::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetDesiredJointSpeeds::Response>(const std::shared_ptr<kortex_driver::srv::SetDesiredJointSpeeds::Request>)> SetDesiredJointSpeedsHandler = nullptr;
        virtual bool SetDesiredJointSpeeds(const std::shared_ptr<kortex_driver::srv::SetDesiredJointSpeeds::Request> req, std::shared_ptr<kortex_driver::srv::SetDesiredJointSpeeds::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetDesiredSpeeds::Response>(const std::shared_ptr<kortex_driver::srv::GetDesiredSpeeds::Request>)> GetDesiredSpeedsHandler = nullptr;
        virtual bool GetDesiredSpeeds(const std::shared_ptr<kortex_driver::srv::GetDesiredSpeeds::Request> req, std::shared_ptr<kortex_driver::srv::GetDesiredSpeeds::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ResetGravityVector::Response>(const std::shared_ptr<kortex_driver::srv::ResetGravityVector::Request>)> ResetGravityVectorHandler = nullptr;
        virtual bool ResetGravityVector(const std::shared_ptr<kortex_driver::srv::ResetGravityVector::Request> req, std::shared_ptr<kortex_driver::srv::ResetGravityVector::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ResetPayloadInformation::Response>(const std::shared_ptr<kortex_driver::srv::ResetPayloadInformation::Request>)> ResetPayloadInformationHandler = nullptr;
        virtual bool ResetPayloadInformation(const std::shared_ptr<kortex_driver::srv::ResetPayloadInformation::Request> req, std::shared_ptr<kortex_driver::srv::ResetPayloadInformation::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ResetToolConfiguration::Response>(const std::shared_ptr<kortex_driver::srv::ResetToolConfiguration::Request>)> ResetToolConfigurationHandler = nullptr;
        virtual bool ResetToolConfiguration(const std::shared_ptr<kortex_driver::srv::ResetToolConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::ResetToolConfiguration::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ResetJointSpeedSoftLimits::Response>(const std::shared_ptr<kortex_driver::srv::ResetJointSpeedSoftLimits::Request>)> ResetJointSpeedSoftLimitsHandler = nullptr;
        virtual bool ResetJointSpeedSoftLimits(const std::shared_ptr<kortex_driver::srv::ResetJointSpeedSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::ResetJointSpeedSoftLimits::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ResetTwistLinearSoftLimit::Response>(const std::shared_ptr<kortex_driver::srv::ResetTwistLinearSoftLimit::Request>)> ResetTwistLinearSoftLimitHandler = nullptr;
        virtual bool ResetTwistLinearSoftLimit(const std::shared_ptr<kortex_driver::srv::ResetTwistLinearSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::ResetTwistLinearSoftLimit::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ResetTwistAngularSoftLimit::Response>(const std::shared_ptr<kortex_driver::srv::ResetTwistAngularSoftLimit::Request>)> ResetTwistAngularSoftLimitHandler = nullptr;
        virtual bool ResetTwistAngularSoftLimit(const std::shared_ptr<kortex_driver::srv::ResetTwistAngularSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::ResetTwistAngularSoftLimit::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ResetJointAccelerationSoftLimits::Response>(const std::shared_ptr<kortex_driver::srv::ResetJointAccelerationSoftLimits::Request>)> ResetJointAccelerationSoftLimitsHandler = nullptr;
        virtual bool ResetJointAccelerationSoftLimits(const std::shared_ptr<kortex_driver::srv::ResetJointAccelerationSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::ResetJointAccelerationSoftLimits::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Response>(const std::shared_ptr<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Request>)> ControlConfig_OnNotificationControlModeTopicHandler = nullptr;
        virtual bool ControlConfig_OnNotificationControlModeTopic(const std::shared_ptr<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Response> res) override;
        virtual void cb_ControlModeTopic(Kinova::Api::ControlConfig::ControlModeNotification notif) override;

};
#endif
