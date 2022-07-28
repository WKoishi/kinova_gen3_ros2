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
 
#ifndef _KORTEX_VISIONCONFIG_SERVICES_INTERFACE_H_
#define _KORTEX_VISIONCONFIG_SERVICES_INTERFACE_H_

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>
#include "kortex_driver/srv/set_sensor_settings.hpp"
#include "kortex_driver/srv/get_sensor_settings.hpp"
#include "kortex_driver/srv/get_option_value.hpp"
#include "kortex_driver/srv/set_option_value.hpp"
#include "kortex_driver/srv/get_option_information.hpp"
#include "kortex_driver/srv/on_notification_vision_topic.hpp"
#include "kortex_driver/msg/vision_notification.hpp"
#include "kortex_driver/srv/do_sensor_focus_action.hpp"
#include "kortex_driver/srv/get_intrinsic_parameters.hpp"
#include "kortex_driver/srv/get_intrinsic_parameters_profile.hpp"
#include "kortex_driver/srv/set_intrinsic_parameters.hpp"
#include "kortex_driver/srv/get_extrinsic_parameters.hpp"
#include "kortex_driver/srv/set_extrinsic_parameters.hpp"

#include "kortex_driver/msg/kortex_error.hpp"
#include "kortex_driver/srv/set_device_id.hpp"
#include "kortex_driver/srv/set_api_options.hpp"
#include "kortex_driver/msg/api_options.hpp"

using namespace std;

class IVisionConfigServices
{
    public:
        IVisionConfigServices(rclcpp::Node::SharedPtr node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) = 0;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) = 0;
        virtual bool SetSensorSettings(kortex_driver::srv::SetSensorSettings::Request  &req, kortex_driver::srv::SetSensorSettings::Response &res) = 0;
        virtual bool GetSensorSettings(kortex_driver::srv::GetSensorSettings::Request  &req, kortex_driver::srv::GetSensorSettings::Response &res) = 0;
        virtual bool GetOptionValue(kortex_driver::srv::GetOptionValue::Request  &req, kortex_driver::srv::GetOptionValue::Response &res) = 0;
        virtual bool SetOptionValue(kortex_driver::srv::SetOptionValue::Request  &req, kortex_driver::srv::SetOptionValue::Response &res) = 0;
        virtual bool GetOptionInformation(kortex_driver::srv::GetOptionInformation::Request  &req, kortex_driver::srv::GetOptionInformation::Response &res) = 0;
        virtual bool OnNotificationVisionTopic(kortex_driver::srv::OnNotificationVisionTopic::Request  &req, kortex_driver::srv::OnNotificationVisionTopic::Response &res) = 0;
        virtual void cb_VisionTopic(Kinova::Api::VisionConfig::VisionNotification notif) = 0;
        virtual bool DoSensorFocusAction(kortex_driver::srv::DoSensorFocusAction::Request  &req, kortex_driver::srv::DoSensorFocusAction::Response &res) = 0;
        virtual bool GetIntrinsicParameters(kortex_driver::srv::GetIntrinsicParameters::Request  &req, kortex_driver::srv::GetIntrinsicParameters::Response &res) = 0;
        virtual bool GetIntrinsicParametersProfile(kortex_driver::srv::GetIntrinsicParametersProfile::Request  &req, kortex_driver::srv::GetIntrinsicParametersProfile::Response &res) = 0;
        virtual bool SetIntrinsicParameters(kortex_driver::srv::SetIntrinsicParameters::Request  &req, kortex_driver::srv::SetIntrinsicParameters::Response &res) = 0;
        virtual bool GetExtrinsicParameters(kortex_driver::srv::GetExtrinsicParameters::Request  &req, kortex_driver::srv::GetExtrinsicParameters::Response &res) = 0;
        virtual bool SetExtrinsicParameters(kortex_driver::srv::SetExtrinsicParameters::Request  &req, kortex_driver::srv::SetExtrinsicParameters::Response &res) = 0;

protected:
        rclcpp::Node::SharedPtr m_node_handle;
        rclcpp::Publisher<kortex_driver::msg::KortexError>::SharedPtr m_pub_Error;
        rclcpp::Publisher<kortex_driver::msg::VisionNotification>::SharedPtr m_pub_VisionTopic;
        bool m_is_activated_VisionTopic;

        rclcpp::Service<kortex_driver::srv::SetDeviceID>::SharedPtr m_serviceSetDeviceID;
        rclcpp::Service<kortex_driver::srv::SetApiOptions>::SharedPtr m_serviceSetApiOptions;

	rclcpp::Service<kortex_driver::srv::SetSensorSettings>::SharedPtr m_serviceSetSensorSettings;
	rclcpp::Service<kortex_driver::srv::GetSensorSettings>::SharedPtr m_serviceGetSensorSettings;
	rclcpp::Service<kortex_driver::srv::GetOptionValue>::SharedPtr m_serviceGetOptionValue;
	rclcpp::Service<kortex_driver::srv::SetOptionValue>::SharedPtr m_serviceSetOptionValue;
	rclcpp::Service<kortex_driver::srv::GetOptionInformation>::SharedPtr m_serviceGetOptionInformation;
	rclcpp::Service<kortex_driver::srv::OnNotificationVisionTopic>::SharedPtr m_serviceOnNotificationVisionTopic;
	rclcpp::Service<kortex_driver::srv::DoSensorFocusAction>::SharedPtr m_serviceDoSensorFocusAction;
	rclcpp::Service<kortex_driver::srv::GetIntrinsicParameters>::SharedPtr m_serviceGetIntrinsicParameters;
	rclcpp::Service<kortex_driver::srv::GetIntrinsicParametersProfile>::SharedPtr m_serviceGetIntrinsicParametersProfile;
	rclcpp::Service<kortex_driver::srv::SetIntrinsicParameters>::SharedPtr m_serviceSetIntrinsicParameters;
	rclcpp::Service<kortex_driver::srv::GetExtrinsicParameters>::SharedPtr m_serviceGetExtrinsicParameters;
	rclcpp::Service<kortex_driver::srv::SetExtrinsicParameters>::SharedPtr m_serviceSetExtrinsicParameters;
};
#endif
