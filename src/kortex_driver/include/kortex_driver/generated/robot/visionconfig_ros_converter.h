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
 
#ifndef _KORTEX_VISIONCONFIG_ROS_CONVERTER_H_
#define _KORTEX_VISIONCONFIG_ROS_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <VisionConfig.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"


#include "kortex_driver/msg/sensor_settings.hpp"
#include "kortex_driver/msg/sensor_identifier.hpp"
#include "kortex_driver/msg/intrinsic_profile_identifier.hpp"
#include "kortex_driver/msg/option_identifier.hpp"
#include "kortex_driver/msg/option_value.hpp"
#include "kortex_driver/msg/option_information.hpp"
#include "kortex_driver/msg/sensor_focus_action.hpp"
#include "kortex_driver/msg/focus_point.hpp"
#include "kortex_driver/msg/manual_focus.hpp"
#include "kortex_driver/msg/vision_notification.hpp"
#include "kortex_driver/msg/intrinsic_parameters.hpp"
#include "kortex_driver/msg/distortion_coefficients.hpp"
#include "kortex_driver/msg/extrinsic_parameters.hpp"
#include "kortex_driver/msg/vision_config_rotation_matrix.hpp"
#include "kortex_driver/msg/vision_config_rotation_matrix_row.hpp"
#include "kortex_driver/msg/translation_vector.hpp"


int ToRosData(Kinova::Api::VisionConfig::SensorSettings input, kortex_driver::msg::SensorSettings &output);
int ToRosData(Kinova::Api::VisionConfig::SensorIdentifier input, kortex_driver::msg::SensorIdentifier &output);
int ToRosData(Kinova::Api::VisionConfig::IntrinsicProfileIdentifier input, kortex_driver::msg::IntrinsicProfileIdentifier &output);
int ToRosData(Kinova::Api::VisionConfig::OptionIdentifier input, kortex_driver::msg::OptionIdentifier &output);
int ToRosData(Kinova::Api::VisionConfig::OptionValue input, kortex_driver::msg::OptionValue &output);
int ToRosData(Kinova::Api::VisionConfig::OptionInformation input, kortex_driver::msg::OptionInformation &output);
int ToRosData(Kinova::Api::VisionConfig::SensorFocusAction input, kortex_driver::msg::SensorFocusAction &output);
int ToRosData(Kinova::Api::VisionConfig::FocusPoint input, kortex_driver::msg::FocusPoint &output);
int ToRosData(Kinova::Api::VisionConfig::ManualFocus input, kortex_driver::msg::ManualFocus &output);
int ToRosData(Kinova::Api::VisionConfig::VisionNotification input, kortex_driver::msg::VisionNotification &output);
int ToRosData(Kinova::Api::VisionConfig::IntrinsicParameters input, kortex_driver::msg::IntrinsicParameters &output);
int ToRosData(Kinova::Api::VisionConfig::DistortionCoefficients input, kortex_driver::msg::DistortionCoefficients &output);
int ToRosData(Kinova::Api::VisionConfig::ExtrinsicParameters input, kortex_driver::msg::ExtrinsicParameters &output);
int ToRosData(Kinova::Api::VisionConfig::RotationMatrix input, kortex_driver::msg::VisionConfigRotationMatrix &output);
int ToRosData(Kinova::Api::VisionConfig::RotationMatrixRow input, kortex_driver::msg::VisionConfigRotationMatrixRow &output);
int ToRosData(Kinova::Api::VisionConfig::TranslationVector input, kortex_driver::msg::TranslationVector &output);

#endif