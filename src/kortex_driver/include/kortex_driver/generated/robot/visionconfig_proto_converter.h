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
 
#ifndef _KORTEX_VISIONCONFIG_PROTO_CONVERTER_H_
#define _KORTEX_VISIONCONFIG_PROTO_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <VisionConfig.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"


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


int ToProtoData(kortex_driver::msg::SensorSettings input, Kinova::Api::VisionConfig::SensorSettings *output);
int ToProtoData(kortex_driver::msg::SensorIdentifier input, Kinova::Api::VisionConfig::SensorIdentifier *output);
int ToProtoData(kortex_driver::msg::IntrinsicProfileIdentifier input, Kinova::Api::VisionConfig::IntrinsicProfileIdentifier *output);
int ToProtoData(kortex_driver::msg::OptionIdentifier input, Kinova::Api::VisionConfig::OptionIdentifier *output);
int ToProtoData(kortex_driver::msg::OptionValue input, Kinova::Api::VisionConfig::OptionValue *output);
int ToProtoData(kortex_driver::msg::OptionInformation input, Kinova::Api::VisionConfig::OptionInformation *output);
int ToProtoData(kortex_driver::msg::SensorFocusAction input, Kinova::Api::VisionConfig::SensorFocusAction *output);
int ToProtoData(kortex_driver::msg::FocusPoint input, Kinova::Api::VisionConfig::FocusPoint *output);
int ToProtoData(kortex_driver::msg::ManualFocus input, Kinova::Api::VisionConfig::ManualFocus *output);
int ToProtoData(kortex_driver::msg::VisionNotification input, Kinova::Api::VisionConfig::VisionNotification *output);
int ToProtoData(kortex_driver::msg::IntrinsicParameters input, Kinova::Api::VisionConfig::IntrinsicParameters *output);
int ToProtoData(kortex_driver::msg::DistortionCoefficients input, Kinova::Api::VisionConfig::DistortionCoefficients *output);
int ToProtoData(kortex_driver::msg::ExtrinsicParameters input, Kinova::Api::VisionConfig::ExtrinsicParameters *output);
int ToProtoData(kortex_driver::msg::VisionConfigRotationMatrix input, Kinova::Api::VisionConfig::RotationMatrix *output);
int ToProtoData(kortex_driver::msg::VisionConfigRotationMatrixRow input, Kinova::Api::VisionConfig::RotationMatrixRow *output);
int ToProtoData(kortex_driver::msg::TranslationVector input, Kinova::Api::VisionConfig::TranslationVector *output);

#endif