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
 
#ifndef _KORTEX_CONTROLCONFIG_ROBOT_SERVICES_H_
#define _KORTEX_CONTROLCONFIG_ROBOT_SERVICES_H_

#include "kortex_driver/generated/interfaces/controlconfig_services_interface.h"

#include <ControlConfig.pb.h>
#include <ControlConfigClientRpc.h>

using namespace std;

class ControlConfigRobotServices : public IControlConfigServices
{
    public:
        ControlConfigRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::ControlConfig::ControlConfigClient* controlconfig, uint32_t device_id, uint32_t timeout_ms);

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) override;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) override;
        virtual bool SetGravityVector(const std::shared_ptr<kortex_driver::srv::SetGravityVector::Request> req, std::shared_ptr<kortex_driver::srv::SetGravityVector::Response> res) override;
        virtual bool GetGravityVector(const std::shared_ptr<kortex_driver::srv::GetGravityVector::Request> req, std::shared_ptr<kortex_driver::srv::GetGravityVector::Response> res) override;
        virtual bool SetPayloadInformation(const std::shared_ptr<kortex_driver::srv::SetPayloadInformation::Request> req, std::shared_ptr<kortex_driver::srv::SetPayloadInformation::Response> res) override;
        virtual bool GetPayloadInformation(const std::shared_ptr<kortex_driver::srv::GetPayloadInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetPayloadInformation::Response> res) override;
        virtual bool SetToolConfiguration(const std::shared_ptr<kortex_driver::srv::SetToolConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetToolConfiguration::Response> res) override;
        virtual bool GetToolConfiguration(const std::shared_ptr<kortex_driver::srv::GetToolConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetToolConfiguration::Response> res) override;
        virtual bool OnNotificationControlConfigurationTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationControlConfigurationTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationControlConfigurationTopic::Response> res) override;
        virtual void cb_ControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification notif) override;
        virtual bool ControlConfig_Unsubscribe(const std::shared_ptr<kortex_driver::srv::ControlConfigUnsubscribe::Request> req, std::shared_ptr<kortex_driver::srv::ControlConfigUnsubscribe::Response> res) override;
        virtual bool SetCartesianReferenceFrame(const std::shared_ptr<kortex_driver::srv::SetCartesianReferenceFrame::Request> req, std::shared_ptr<kortex_driver::srv::SetCartesianReferenceFrame::Response> res) override;
        virtual bool GetCartesianReferenceFrame(const std::shared_ptr<kortex_driver::srv::GetCartesianReferenceFrame::Request> req, std::shared_ptr<kortex_driver::srv::GetCartesianReferenceFrame::Response> res) override;
        virtual bool ControlConfig_GetControlMode(const std::shared_ptr<kortex_driver::srv::ControlConfigGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::ControlConfigGetControlMode::Response> res) override;
        virtual bool SetJointSpeedSoftLimits(const std::shared_ptr<kortex_driver::srv::SetJointSpeedSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::SetJointSpeedSoftLimits::Response> res) override;
        virtual bool SetTwistLinearSoftLimit(const std::shared_ptr<kortex_driver::srv::SetTwistLinearSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::SetTwistLinearSoftLimit::Response> res) override;
        virtual bool SetTwistAngularSoftLimit(const std::shared_ptr<kortex_driver::srv::SetTwistAngularSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::SetTwistAngularSoftLimit::Response> res) override;
        virtual bool SetJointAccelerationSoftLimits(const std::shared_ptr<kortex_driver::srv::SetJointAccelerationSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::SetJointAccelerationSoftLimits::Response> res) override;
        virtual bool GetKinematicHardLimits(const std::shared_ptr<kortex_driver::srv::GetKinematicHardLimits::Request> req, std::shared_ptr<kortex_driver::srv::GetKinematicHardLimits::Response> res) override;
        virtual bool GetKinematicSoftLimits(const std::shared_ptr<kortex_driver::srv::GetKinematicSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::GetKinematicSoftLimits::Response> res) override;
        virtual bool GetAllKinematicSoftLimits(const std::shared_ptr<kortex_driver::srv::GetAllKinematicSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::GetAllKinematicSoftLimits::Response> res) override;
        virtual bool SetDesiredLinearTwist(const std::shared_ptr<kortex_driver::srv::SetDesiredLinearTwist::Request> req, std::shared_ptr<kortex_driver::srv::SetDesiredLinearTwist::Response> res) override;
        virtual bool SetDesiredAngularTwist(const std::shared_ptr<kortex_driver::srv::SetDesiredAngularTwist::Request> req, std::shared_ptr<kortex_driver::srv::SetDesiredAngularTwist::Response> res) override;
        virtual bool SetDesiredJointSpeeds(const std::shared_ptr<kortex_driver::srv::SetDesiredJointSpeeds::Request> req, std::shared_ptr<kortex_driver::srv::SetDesiredJointSpeeds::Response> res) override;
        virtual bool GetDesiredSpeeds(const std::shared_ptr<kortex_driver::srv::GetDesiredSpeeds::Request> req, std::shared_ptr<kortex_driver::srv::GetDesiredSpeeds::Response> res) override;
        virtual bool ResetGravityVector(const std::shared_ptr<kortex_driver::srv::ResetGravityVector::Request> req, std::shared_ptr<kortex_driver::srv::ResetGravityVector::Response> res) override;
        virtual bool ResetPayloadInformation(const std::shared_ptr<kortex_driver::srv::ResetPayloadInformation::Request> req, std::shared_ptr<kortex_driver::srv::ResetPayloadInformation::Response> res) override;
        virtual bool ResetToolConfiguration(const std::shared_ptr<kortex_driver::srv::ResetToolConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::ResetToolConfiguration::Response> res) override;
        virtual bool ResetJointSpeedSoftLimits(const std::shared_ptr<kortex_driver::srv::ResetJointSpeedSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::ResetJointSpeedSoftLimits::Response> res) override;
        virtual bool ResetTwistLinearSoftLimit(const std::shared_ptr<kortex_driver::srv::ResetTwistLinearSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::ResetTwistLinearSoftLimit::Response> res) override;
        virtual bool ResetTwistAngularSoftLimit(const std::shared_ptr<kortex_driver::srv::ResetTwistAngularSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::ResetTwistAngularSoftLimit::Response> res) override;
        virtual bool ResetJointAccelerationSoftLimits(const std::shared_ptr<kortex_driver::srv::ResetJointAccelerationSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::ResetJointAccelerationSoftLimits::Response> res) override;
        virtual bool ControlConfig_OnNotificationControlModeTopic(const std::shared_ptr<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Response> res) override;
        virtual void cb_ControlModeTopic(Kinova::Api::ControlConfig::ControlModeNotification notif) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::ControlConfig::ControlConfigClient* m_controlconfig;
};
#endif
