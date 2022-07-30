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
 
#ifndef _KORTEX_VISIONCONFIG_SIMULATION_SERVICES_H_
#define _KORTEX_VISIONCONFIG_SIMULATION_SERVICES_H_

#include "kortex_driver/generated/interfaces/visionconfig_services_interface.h"

using namespace std;

class VisionConfigSimulationServices : public IVisionConfigServices
{
    public:
        VisionConfigSimulationServices(rclcpp::Node::SharedPtr node_handle);

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) override;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) override;
        std::function<kortex_driver::srv::SetSensorSettings::Response(const kortex_driver::srv::SetSensorSettings::Request&)> SetSensorSettingsHandler = nullptr;
        virtual bool SetSensorSettings(const std::shared_ptr<kortex_driver::srv::SetSensorSettings::Request> req, std::shared_ptr<kortex_driver::srv::SetSensorSettings::Response> res) override;
        std::function<kortex_driver::srv::GetSensorSettings::Response(const kortex_driver::srv::GetSensorSettings::Request&)> GetSensorSettingsHandler = nullptr;
        virtual bool GetSensorSettings(const std::shared_ptr<kortex_driver::srv::GetSensorSettings::Request> req, std::shared_ptr<kortex_driver::srv::GetSensorSettings::Response> res) override;
        std::function<kortex_driver::srv::GetOptionValue::Response(const kortex_driver::srv::GetOptionValue::Request&)> GetOptionValueHandler = nullptr;
        virtual bool GetOptionValue(const std::shared_ptr<kortex_driver::srv::GetOptionValue::Request> req, std::shared_ptr<kortex_driver::srv::GetOptionValue::Response> res) override;
        std::function<kortex_driver::srv::SetOptionValue::Response(const kortex_driver::srv::SetOptionValue::Request&)> SetOptionValueHandler = nullptr;
        virtual bool SetOptionValue(const std::shared_ptr<kortex_driver::srv::SetOptionValue::Request> req, std::shared_ptr<kortex_driver::srv::SetOptionValue::Response> res) override;
        std::function<kortex_driver::srv::GetOptionInformation::Response(const kortex_driver::srv::GetOptionInformation::Request&)> GetOptionInformationHandler = nullptr;
        virtual bool GetOptionInformation(const std::shared_ptr<kortex_driver::srv::GetOptionInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetOptionInformation::Response> res) override;
        std::function<kortex_driver::srv::OnNotificationVisionTopic::Response(const kortex_driver::srv::OnNotificationVisionTopic::Request&)> OnNotificationVisionTopicHandler = nullptr;
        virtual bool OnNotificationVisionTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationVisionTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationVisionTopic::Response> res) override;
        virtual void cb_VisionTopic(Kinova::Api::VisionConfig::VisionNotification notif) override;
        std::function<kortex_driver::srv::DoSensorFocusAction::Response(const kortex_driver::srv::DoSensorFocusAction::Request&)> DoSensorFocusActionHandler = nullptr;
        virtual bool DoSensorFocusAction(const std::shared_ptr<kortex_driver::srv::DoSensorFocusAction::Request> req, std::shared_ptr<kortex_driver::srv::DoSensorFocusAction::Response> res) override;
        std::function<kortex_driver::srv::GetIntrinsicParameters::Response(const kortex_driver::srv::GetIntrinsicParameters::Request&)> GetIntrinsicParametersHandler = nullptr;
        virtual bool GetIntrinsicParameters(const std::shared_ptr<kortex_driver::srv::GetIntrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::GetIntrinsicParameters::Response> res) override;
        std::function<kortex_driver::srv::GetIntrinsicParametersProfile::Response(const kortex_driver::srv::GetIntrinsicParametersProfile::Request&)> GetIntrinsicParametersProfileHandler = nullptr;
        virtual bool GetIntrinsicParametersProfile(const std::shared_ptr<kortex_driver::srv::GetIntrinsicParametersProfile::Request> req, std::shared_ptr<kortex_driver::srv::GetIntrinsicParametersProfile::Response> res) override;
        std::function<kortex_driver::srv::SetIntrinsicParameters::Response(const kortex_driver::srv::SetIntrinsicParameters::Request&)> SetIntrinsicParametersHandler = nullptr;
        virtual bool SetIntrinsicParameters(const std::shared_ptr<kortex_driver::srv::SetIntrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::SetIntrinsicParameters::Response> res) override;
        std::function<kortex_driver::srv::GetExtrinsicParameters::Response(const kortex_driver::srv::GetExtrinsicParameters::Request&)> GetExtrinsicParametersHandler = nullptr;
        virtual bool GetExtrinsicParameters(const std::shared_ptr<kortex_driver::srv::GetExtrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::GetExtrinsicParameters::Response> res) override;
        std::function<kortex_driver::srv::SetExtrinsicParameters::Response(const kortex_driver::srv::SetExtrinsicParameters::Request&)> SetExtrinsicParametersHandler = nullptr;
        virtual bool SetExtrinsicParameters(const std::shared_ptr<kortex_driver::srv::SetExtrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::SetExtrinsicParameters::Response> res) override;

};
#endif
