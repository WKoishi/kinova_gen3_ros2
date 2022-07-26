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
 
#ifndef _KORTEX_VISIONCONFIG_ROBOT_SERVICES_H_
#define _KORTEX_VISIONCONFIG_ROBOT_SERVICES_H_

#include "kortex_driver/generated/interfaces/visionconfig_services_interface.h"

#include <VisionConfig.pb.h>
#include <VisionConfigClientRpc.h>

using namespace std;

class VisionConfigRobotServices : public IVisionConfigServices
{
    public:
        VisionConfigRobotServices(ros::NodeHandle& node_handle, Kinova::Api::VisionConfig::VisionConfigClient* visionconfig, uint32_t device_id, uint32_t timeout_ms);

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) override;
        virtual bool SetSensorSettings(kortex_driver::srv::SetSensorSettings::Request  &req, kortex_driver::srv::SetSensorSettings::Response &res) override;
        virtual bool GetSensorSettings(kortex_driver::srv::GetSensorSettings::Request  &req, kortex_driver::srv::GetSensorSettings::Response &res) override;
        virtual bool GetOptionValue(kortex_driver::srv::GetOptionValue::Request  &req, kortex_driver::srv::GetOptionValue::Response &res) override;
        virtual bool SetOptionValue(kortex_driver::srv::SetOptionValue::Request  &req, kortex_driver::srv::SetOptionValue::Response &res) override;
        virtual bool GetOptionInformation(kortex_driver::srv::GetOptionInformation::Request  &req, kortex_driver::srv::GetOptionInformation::Response &res) override;
        virtual bool OnNotificationVisionTopic(kortex_driver::srv::OnNotificationVisionTopic::Request  &req, kortex_driver::srv::OnNotificationVisionTopic::Response &res) override;
        virtual void cb_VisionTopic(Kinova::Api::VisionConfig::VisionNotification notif) override;
        virtual bool DoSensorFocusAction(kortex_driver::srv::DoSensorFocusAction::Request  &req, kortex_driver::srv::DoSensorFocusAction::Response &res) override;
        virtual bool GetIntrinsicParameters(kortex_driver::srv::GetIntrinsicParameters::Request  &req, kortex_driver::srv::GetIntrinsicParameters::Response &res) override;
        virtual bool GetIntrinsicParametersProfile(kortex_driver::srv::GetIntrinsicParametersProfile::Request  &req, kortex_driver::srv::GetIntrinsicParametersProfile::Response &res) override;
        virtual bool SetIntrinsicParameters(kortex_driver::srv::SetIntrinsicParameters::Request  &req, kortex_driver::srv::SetIntrinsicParameters::Response &res) override;
        virtual bool GetExtrinsicParameters(kortex_driver::srv::GetExtrinsicParameters::Request  &req, kortex_driver::srv::GetExtrinsicParameters::Response &res) override;
        virtual bool SetExtrinsicParameters(kortex_driver::srv::SetExtrinsicParameters::Request  &req, kortex_driver::srv::SetExtrinsicParameters::Response &res) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::VisionConfig::VisionConfigClient* m_visionconfig;
};
#endif
