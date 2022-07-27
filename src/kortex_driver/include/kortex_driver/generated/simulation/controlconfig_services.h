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

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) override;
        std::function<kortex_driver::srv::SetGravityVector::Response(const kortex_driver::srv::SetGravityVector::Request&)> SetGravityVectorHandler = nullptr;
        virtual bool SetGravityVector(kortex_driver::srv::SetGravityVector::Request  &req, kortex_driver::srv::SetGravityVector::Response &res) override;
        std::function<kortex_driver::srv::GetGravityVector::Response(const kortex_driver::srv::GetGravityVector::Request&)> GetGravityVectorHandler = nullptr;
        virtual bool GetGravityVector(kortex_driver::srv::GetGravityVector::Request  &req, kortex_driver::srv::GetGravityVector::Response &res) override;
        std::function<kortex_driver::srv::SetPayloadInformation::Response(const kortex_driver::srv::SetPayloadInformation::Request&)> SetPayloadInformationHandler = nullptr;
        virtual bool SetPayloadInformation(kortex_driver::srv::SetPayloadInformation::Request  &req, kortex_driver::srv::SetPayloadInformation::Response &res) override;
        std::function<kortex_driver::srv::GetPayloadInformation::Response(const kortex_driver::srv::GetPayloadInformation::Request&)> GetPayloadInformationHandler = nullptr;
        virtual bool GetPayloadInformation(kortex_driver::srv::GetPayloadInformation::Request  &req, kortex_driver::srv::GetPayloadInformation::Response &res) override;
        std::function<kortex_driver::srv::SetToolConfiguration::Response(const kortex_driver::srv::SetToolConfiguration::Request&)> SetToolConfigurationHandler = nullptr;
        virtual bool SetToolConfiguration(kortex_driver::srv::SetToolConfiguration::Request  &req, kortex_driver::srv::SetToolConfiguration::Response &res) override;
        std::function<kortex_driver::srv::GetToolConfiguration::Response(const kortex_driver::srv::GetToolConfiguration::Request&)> GetToolConfigurationHandler = nullptr;
        virtual bool GetToolConfiguration(kortex_driver::srv::GetToolConfiguration::Request  &req, kortex_driver::srv::GetToolConfiguration::Response &res) override;
        std::function<kortex_driver::srv::OnNotificationControlConfigurationTopic::Response(const kortex_driver::srv::OnNotificationControlConfigurationTopic::Request&)> OnNotificationControlConfigurationTopicHandler = nullptr;
        virtual bool OnNotificationControlConfigurationTopic(kortex_driver::srv::OnNotificationControlConfigurationTopic::Request  &req, kortex_driver::srv::OnNotificationControlConfigurationTopic::Response &res) override;
        virtual void cb_ControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification notif) override;
        std::function<kortex_driver::srv::ControlConfigUnsubscribe::Response(const kortex_driver::srv::ControlConfigUnsubscribe::Request&)> ControlConfig_UnsubscribeHandler = nullptr;
        virtual bool ControlConfig_Unsubscribe(kortex_driver::srv::ControlConfigUnsubscribe::Request  &req, kortex_driver::srv::ControlConfigUnsubscribe::Response &res) override;
        std::function<kortex_driver::srv::SetCartesianReferenceFrame::Response(const kortex_driver::srv::SetCartesianReferenceFrame::Request&)> SetCartesianReferenceFrameHandler = nullptr;
        virtual bool SetCartesianReferenceFrame(kortex_driver::srv::SetCartesianReferenceFrame::Request  &req, kortex_driver::srv::SetCartesianReferenceFrame::Response &res) override;
        std::function<kortex_driver::srv::GetCartesianReferenceFrame::Response(const kortex_driver::srv::GetCartesianReferenceFrame::Request&)> GetCartesianReferenceFrameHandler = nullptr;
        virtual bool GetCartesianReferenceFrame(kortex_driver::srv::GetCartesianReferenceFrame::Request  &req, kortex_driver::srv::GetCartesianReferenceFrame::Response &res) override;
        std::function<kortex_driver::srv::ControlConfigGetControlMode::Response(const kortex_driver::srv::ControlConfigGetControlMode::Request&)> ControlConfig_GetControlModeHandler = nullptr;
        virtual bool ControlConfig_GetControlMode(kortex_driver::srv::ControlConfigGetControlMode::Request  &req, kortex_driver::srv::ControlConfigGetControlMode::Response &res) override;
        std::function<kortex_driver::srv::SetJointSpeedSoftLimits::Response(const kortex_driver::srv::SetJointSpeedSoftLimits::Request&)> SetJointSpeedSoftLimitsHandler = nullptr;
        virtual bool SetJointSpeedSoftLimits(kortex_driver::srv::SetJointSpeedSoftLimits::Request  &req, kortex_driver::srv::SetJointSpeedSoftLimits::Response &res) override;
        std::function<kortex_driver::srv::SetTwistLinearSoftLimit::Response(const kortex_driver::srv::SetTwistLinearSoftLimit::Request&)> SetTwistLinearSoftLimitHandler = nullptr;
        virtual bool SetTwistLinearSoftLimit(kortex_driver::srv::SetTwistLinearSoftLimit::Request  &req, kortex_driver::srv::SetTwistLinearSoftLimit::Response &res) override;
        std::function<kortex_driver::srv::SetTwistAngularSoftLimit::Response(const kortex_driver::srv::SetTwistAngularSoftLimit::Request&)> SetTwistAngularSoftLimitHandler = nullptr;
        virtual bool SetTwistAngularSoftLimit(kortex_driver::srv::SetTwistAngularSoftLimit::Request  &req, kortex_driver::srv::SetTwistAngularSoftLimit::Response &res) override;
        std::function<kortex_driver::srv::SetJointAccelerationSoftLimits::Response(const kortex_driver::srv::SetJointAccelerationSoftLimits::Request&)> SetJointAccelerationSoftLimitsHandler = nullptr;
        virtual bool SetJointAccelerationSoftLimits(kortex_driver::srv::SetJointAccelerationSoftLimits::Request  &req, kortex_driver::srv::SetJointAccelerationSoftLimits::Response &res) override;
        std::function<kortex_driver::srv::GetKinematicHardLimits::Response(const kortex_driver::srv::GetKinematicHardLimits::Request&)> GetKinematicHardLimitsHandler = nullptr;
        virtual bool GetKinematicHardLimits(kortex_driver::srv::GetKinematicHardLimits::Request  &req, kortex_driver::srv::GetKinematicHardLimits::Response &res) override;
        std::function<kortex_driver::srv::GetKinematicSoftLimits::Response(const kortex_driver::srv::GetKinematicSoftLimits::Request&)> GetKinematicSoftLimitsHandler = nullptr;
        virtual bool GetKinematicSoftLimits(kortex_driver::srv::GetKinematicSoftLimits::Request  &req, kortex_driver::srv::GetKinematicSoftLimits::Response &res) override;
        std::function<kortex_driver::srv::GetAllKinematicSoftLimits::Response(const kortex_driver::srv::GetAllKinematicSoftLimits::Request&)> GetAllKinematicSoftLimitsHandler = nullptr;
        virtual bool GetAllKinematicSoftLimits(kortex_driver::srv::GetAllKinematicSoftLimits::Request  &req, kortex_driver::srv::GetAllKinematicSoftLimits::Response &res) override;
        std::function<kortex_driver::srv::SetDesiredLinearTwist::Response(const kortex_driver::srv::SetDesiredLinearTwist::Request&)> SetDesiredLinearTwistHandler = nullptr;
        virtual bool SetDesiredLinearTwist(kortex_driver::srv::SetDesiredLinearTwist::Request  &req, kortex_driver::srv::SetDesiredLinearTwist::Response &res) override;
        std::function<kortex_driver::srv::SetDesiredAngularTwist::Response(const kortex_driver::srv::SetDesiredAngularTwist::Request&)> SetDesiredAngularTwistHandler = nullptr;
        virtual bool SetDesiredAngularTwist(kortex_driver::srv::SetDesiredAngularTwist::Request  &req, kortex_driver::srv::SetDesiredAngularTwist::Response &res) override;
        std::function<kortex_driver::srv::SetDesiredJointSpeeds::Response(const kortex_driver::srv::SetDesiredJointSpeeds::Request&)> SetDesiredJointSpeedsHandler = nullptr;
        virtual bool SetDesiredJointSpeeds(kortex_driver::srv::SetDesiredJointSpeeds::Request  &req, kortex_driver::srv::SetDesiredJointSpeeds::Response &res) override;
        std::function<kortex_driver::srv::GetDesiredSpeeds::Response(const kortex_driver::srv::GetDesiredSpeeds::Request&)> GetDesiredSpeedsHandler = nullptr;
        virtual bool GetDesiredSpeeds(kortex_driver::srv::GetDesiredSpeeds::Request  &req, kortex_driver::srv::GetDesiredSpeeds::Response &res) override;
        std::function<kortex_driver::srv::ResetGravityVector::Response(const kortex_driver::srv::ResetGravityVector::Request&)> ResetGravityVectorHandler = nullptr;
        virtual bool ResetGravityVector(kortex_driver::srv::ResetGravityVector::Request  &req, kortex_driver::srv::ResetGravityVector::Response &res) override;
        std::function<kortex_driver::srv::ResetPayloadInformation::Response(const kortex_driver::srv::ResetPayloadInformation::Request&)> ResetPayloadInformationHandler = nullptr;
        virtual bool ResetPayloadInformation(kortex_driver::srv::ResetPayloadInformation::Request  &req, kortex_driver::srv::ResetPayloadInformation::Response &res) override;
        std::function<kortex_driver::srv::ResetToolConfiguration::Response(const kortex_driver::srv::ResetToolConfiguration::Request&)> ResetToolConfigurationHandler = nullptr;
        virtual bool ResetToolConfiguration(kortex_driver::srv::ResetToolConfiguration::Request  &req, kortex_driver::srv::ResetToolConfiguration::Response &res) override;
        std::function<kortex_driver::srv::ResetJointSpeedSoftLimits::Response(const kortex_driver::srv::ResetJointSpeedSoftLimits::Request&)> ResetJointSpeedSoftLimitsHandler = nullptr;
        virtual bool ResetJointSpeedSoftLimits(kortex_driver::srv::ResetJointSpeedSoftLimits::Request  &req, kortex_driver::srv::ResetJointSpeedSoftLimits::Response &res) override;
        std::function<kortex_driver::srv::ResetTwistLinearSoftLimit::Response(const kortex_driver::srv::ResetTwistLinearSoftLimit::Request&)> ResetTwistLinearSoftLimitHandler = nullptr;
        virtual bool ResetTwistLinearSoftLimit(kortex_driver::srv::ResetTwistLinearSoftLimit::Request  &req, kortex_driver::srv::ResetTwistLinearSoftLimit::Response &res) override;
        std::function<kortex_driver::srv::ResetTwistAngularSoftLimit::Response(const kortex_driver::srv::ResetTwistAngularSoftLimit::Request&)> ResetTwistAngularSoftLimitHandler = nullptr;
        virtual bool ResetTwistAngularSoftLimit(kortex_driver::srv::ResetTwistAngularSoftLimit::Request  &req, kortex_driver::srv::ResetTwistAngularSoftLimit::Response &res) override;
        std::function<kortex_driver::srv::ResetJointAccelerationSoftLimits::Response(const kortex_driver::srv::ResetJointAccelerationSoftLimits::Request&)> ResetJointAccelerationSoftLimitsHandler = nullptr;
        virtual bool ResetJointAccelerationSoftLimits(kortex_driver::srv::ResetJointAccelerationSoftLimits::Request  &req, kortex_driver::srv::ResetJointAccelerationSoftLimits::Response &res) override;
        std::function<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Response(const kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Request&)> ControlConfig_OnNotificationControlModeTopicHandler = nullptr;
        virtual bool ControlConfig_OnNotificationControlModeTopic(kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Request  &req, kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Response &res) override;
        virtual void cb_ControlModeTopic(Kinova::Api::ControlConfig::ControlModeNotification notif) override;

};
#endif
