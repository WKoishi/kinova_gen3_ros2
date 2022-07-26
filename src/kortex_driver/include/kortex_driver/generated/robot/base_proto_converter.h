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
 
#ifndef _KORTEX_BASE_PROTO_CONVERTER_H_
#define _KORTEX_BASE_PROTO_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Base.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


#include "kortex_driver/msg/gpio_configuration_list.hpp"
#include "kortex_driver/msg/base_gpio_configuration.hpp"
#include "kortex_driver/msg/gpio_pin_configuration.hpp"
#include "kortex_driver/msg/full_user_profile.hpp"
#include "kortex_driver/msg/user_profile.hpp"
#include "kortex_driver/msg/user_profile_list.hpp"
#include "kortex_driver/msg/user_list.hpp"
#include "kortex_driver/msg/password_change.hpp"
#include "kortex_driver/msg/sequence_handle.hpp"
#include "kortex_driver/msg/advanced_sequence_handle.hpp"
#include "kortex_driver/msg/sequence_task_handle.hpp"
#include "kortex_driver/msg/sequence_task.hpp"
#include "kortex_driver/msg/sequence_tasks.hpp"
#include "kortex_driver/msg/sequence_tasks_configuration.hpp"
#include "kortex_driver/msg/sequence_task_configuration.hpp"
#include "kortex_driver/msg/sequence_tasks_range.hpp"
#include "kortex_driver/msg/sequence_tasks_pair.hpp"
#include "kortex_driver/msg/sequence.hpp"
#include "kortex_driver/msg/sequence_list.hpp"
#include "kortex_driver/msg/append_action_information.hpp"
#include "kortex_driver/msg/action_handle.hpp"
#include "kortex_driver/msg/requested_action_type.hpp"
#include "kortex_driver/msg/action.hpp"
#include "kortex_driver/msg/snapshot.hpp"
#include "kortex_driver/msg/switch_control_mapping.hpp"
#include "kortex_driver/msg/change_twist.hpp"
#include "kortex_driver/msg/change_joint_speeds.hpp"
#include "kortex_driver/msg/change_wrench.hpp"
#include "kortex_driver/msg/emergency_stop.hpp"
#include "kortex_driver/msg/faults.hpp"
#include "kortex_driver/msg/delay.hpp"
#include "kortex_driver/msg/base_stop.hpp"
#include "kortex_driver/msg/action_list.hpp"
#include "kortex_driver/msg/timeout.hpp"
#include "kortex_driver/msg/ssid.hpp"
#include "kortex_driver/msg/communication_interface_configuration.hpp"
#include "kortex_driver/msg/network_handle.hpp"
#include "kortex_driver/msg/i_pv4_configuration.hpp"
#include "kortex_driver/msg/i_pv4_information.hpp"
#include "kortex_driver/msg/full_i_pv4_configuration.hpp"
#include "kortex_driver/msg/wifi_information.hpp"
#include "kortex_driver/msg/wifi_information_list.hpp"
#include "kortex_driver/msg/wifi_configuration.hpp"
#include "kortex_driver/msg/wifi_configuration_list.hpp"
#include "kortex_driver/msg/protection_zone_handle.hpp"
#include "kortex_driver/msg/base_rotation_matrix_row.hpp"
#include "kortex_driver/msg/base_rotation_matrix.hpp"
#include "kortex_driver/msg/point.hpp"
#include "kortex_driver/msg/zone_shape.hpp"
#include "kortex_driver/msg/protection_zone.hpp"
#include "kortex_driver/msg/protection_zone_list.hpp"
#include "kortex_driver/msg/cartesian_limitation.hpp"
#include "kortex_driver/msg/twist_limitation.hpp"
#include "kortex_driver/msg/wrench_limitation.hpp"
#include "kortex_driver/msg/cartesian_limitation_list.hpp"
#include "kortex_driver/msg/joint_limitation.hpp"
#include "kortex_driver/msg/joints_limitations_list.hpp"
#include "kortex_driver/msg/query.hpp"
#include "kortex_driver/msg/configuration_change_notification.hpp"
#include "kortex_driver/msg/mapping_info_notification.hpp"
#include "kortex_driver/msg/base_control_mode_information.hpp"
#include "kortex_driver/msg/base_control_mode_notification.hpp"
#include "kortex_driver/msg/servoing_mode_information.hpp"
#include "kortex_driver/msg/operating_mode_information.hpp"
#include "kortex_driver/msg/operating_mode_notification.hpp"
#include "kortex_driver/msg/servoing_mode_notification.hpp"
#include "kortex_driver/msg/sequence_info_notification.hpp"
#include "kortex_driver/msg/sequence_information.hpp"
#include "kortex_driver/msg/protection_zone_notification.hpp"
#include "kortex_driver/msg/protection_zone_information.hpp"
#include "kortex_driver/msg/user_notification.hpp"
#include "kortex_driver/msg/controller_handle.hpp"
#include "kortex_driver/msg/controller_element_handle.hpp"
#include "kortex_driver/msg/controller_notification.hpp"
#include "kortex_driver/msg/controller_list.hpp"
#include "kortex_driver/msg/controller_state.hpp"
#include "kortex_driver/msg/controller_element_state.hpp"
#include "kortex_driver/msg/action_notification.hpp"
#include "kortex_driver/msg/trajectory_info.hpp"
#include "kortex_driver/msg/action_execution_state.hpp"
#include "kortex_driver/msg/robot_event_notification.hpp"
#include "kortex_driver/msg/factory_notification.hpp"
#include "kortex_driver/msg/network_notification.hpp"
#include "kortex_driver/msg/configuration_change_notification_list.hpp"
#include "kortex_driver/msg/mapping_info_notification_list.hpp"
#include "kortex_driver/msg/control_mode_notification_list.hpp"
#include "kortex_driver/msg/operating_mode_notification_list.hpp"
#include "kortex_driver/msg/servoing_mode_notification_list.hpp"
#include "kortex_driver/msg/sequence_info_notification_list.hpp"
#include "kortex_driver/msg/protection_zone_notification_list.hpp"
#include "kortex_driver/msg/user_notification_list.hpp"
#include "kortex_driver/msg/safety_notification_list.hpp"
#include "kortex_driver/msg/controller_notification_list.hpp"
#include "kortex_driver/msg/action_notification_list.hpp"
#include "kortex_driver/msg/robot_event_notification_list.hpp"
#include "kortex_driver/msg/network_notification_list.hpp"
#include "kortex_driver/msg/mapping_handle.hpp"
#include "kortex_driver/msg/safety_event.hpp"
#include "kortex_driver/msg/controller_event.hpp"
#include "kortex_driver/msg/gpio_event.hpp"
#include "kortex_driver/msg/map_event.hpp"
#include "kortex_driver/msg/map_element.hpp"
#include "kortex_driver/msg/activate_map_handle.hpp"
#include "kortex_driver/msg/map.hpp"
#include "kortex_driver/msg/map_handle.hpp"
#include "kortex_driver/msg/map_list.hpp"
#include "kortex_driver/msg/map_group_handle.hpp"
#include "kortex_driver/msg/map_group.hpp"
#include "kortex_driver/msg/map_group_list.hpp"
#include "kortex_driver/msg/mapping.hpp"
#include "kortex_driver/msg/mapping_list.hpp"
#include "kortex_driver/msg/transformation_matrix.hpp"
#include "kortex_driver/msg/transformation_row.hpp"
#include "kortex_driver/msg/pose.hpp"
#include "kortex_driver/msg/base_position.hpp"
#include "kortex_driver/msg/orientation.hpp"
#include "kortex_driver/msg/cartesian_speed.hpp"
#include "kortex_driver/msg/cartesian_trajectory_constraint.hpp"
#include "kortex_driver/msg/joint_trajectory_constraint.hpp"
#include "kortex_driver/msg/wrench.hpp"
#include "kortex_driver/msg/twist.hpp"
#include "kortex_driver/msg/admittance.hpp"
#include "kortex_driver/msg/constrained_pose.hpp"
#include "kortex_driver/msg/constrained_position.hpp"
#include "kortex_driver/msg/constrained_orientation.hpp"
#include "kortex_driver/msg/wrench_command.hpp"
#include "kortex_driver/msg/twist_command.hpp"
#include "kortex_driver/msg/constrained_joint_angles.hpp"
#include "kortex_driver/msg/constrained_joint_angle.hpp"
#include "kortex_driver/msg/joint_angles.hpp"
#include "kortex_driver/msg/joint_angle.hpp"
#include "kortex_driver/msg/base_joint_speeds.hpp"
#include "kortex_driver/msg/joint_speed.hpp"
#include "kortex_driver/msg/joint_torques.hpp"
#include "kortex_driver/msg/joint_torque.hpp"
#include "kortex_driver/msg/gripper_command.hpp"
#include "kortex_driver/msg/gripper_request.hpp"
#include "kortex_driver/msg/gripper.hpp"
#include "kortex_driver/msg/finger.hpp"
#include "kortex_driver/msg/gpio_command.hpp"
#include "kortex_driver/msg/system_time.hpp"
#include "kortex_driver/msg/controller_configuration_mode.hpp"
#include "kortex_driver/msg/controller_configuration.hpp"
#include "kortex_driver/msg/controller_configuration_list.hpp"
#include "kortex_driver/msg/actuator_information.hpp"
#include "kortex_driver/msg/arm_state_information.hpp"
#include "kortex_driver/msg/arm_state_notification.hpp"
#include "kortex_driver/msg/base_cap_sense_config.hpp"
#include "kortex_driver/msg/bridge_list.hpp"
#include "kortex_driver/msg/bridge_result.hpp"
#include "kortex_driver/msg/bridge_identifier.hpp"
#include "kortex_driver/msg/bridge_config.hpp"
#include "kortex_driver/msg/bridge_port_config.hpp"
#include "kortex_driver/msg/pre_computed_joint_trajectory.hpp"
#include "kortex_driver/msg/pre_computed_joint_trajectory_element.hpp"
#include "kortex_driver/msg/trajectory_error_element.hpp"
#include "kortex_driver/msg/trajectory_error_report.hpp"
#include "kortex_driver/msg/waypoint_validation_report.hpp"
#include "kortex_driver/msg/waypoint.hpp"
#include "kortex_driver/msg/angular_waypoint.hpp"
#include "kortex_driver/msg/cartesian_waypoint.hpp"
#include "kortex_driver/msg/waypoint_list.hpp"
#include "kortex_driver/msg/kinematic_trajectory_constraints.hpp"
#include "kortex_driver/msg/firmware_bundle_versions.hpp"
#include "kortex_driver/msg/firmware_component_version.hpp"
#include "kortex_driver/msg/ik_data.hpp"


int ToProtoData(kortex_driver::msg::GpioConfigurationList input, Kinova::Api::Base::GpioConfigurationList *output);
int ToProtoData(kortex_driver::msg::BaseGpioConfiguration input, Kinova::Api::Base::GpioConfiguration *output);
int ToProtoData(kortex_driver::msg::GpioPinConfiguration input, Kinova::Api::Base::GpioPinConfiguration *output);
int ToProtoData(kortex_driver::msg::FullUserProfile input, Kinova::Api::Base::FullUserProfile *output);
int ToProtoData(kortex_driver::msg::UserProfile input, Kinova::Api::Base::UserProfile *output);
int ToProtoData(kortex_driver::msg::UserProfileList input, Kinova::Api::Base::UserProfileList *output);
int ToProtoData(kortex_driver::msg::UserList input, Kinova::Api::Base::UserList *output);
int ToProtoData(kortex_driver::msg::PasswordChange input, Kinova::Api::Base::PasswordChange *output);
int ToProtoData(kortex_driver::msg::SequenceHandle input, Kinova::Api::Base::SequenceHandle *output);
int ToProtoData(kortex_driver::msg::AdvancedSequenceHandle input, Kinova::Api::Base::AdvancedSequenceHandle *output);
int ToProtoData(kortex_driver::msg::SequenceTaskHandle input, Kinova::Api::Base::SequenceTaskHandle *output);
int ToProtoData(kortex_driver::msg::SequenceTask input, Kinova::Api::Base::SequenceTask *output);
int ToProtoData(kortex_driver::msg::SequenceTasks input, Kinova::Api::Base::SequenceTasks *output);
int ToProtoData(kortex_driver::msg::SequenceTasksConfiguration input, Kinova::Api::Base::SequenceTasksConfiguration *output);
int ToProtoData(kortex_driver::msg::SequenceTaskConfiguration input, Kinova::Api::Base::SequenceTaskConfiguration *output);
int ToProtoData(kortex_driver::msg::SequenceTasksRange input, Kinova::Api::Base::SequenceTasksRange *output);
int ToProtoData(kortex_driver::msg::SequenceTasksPair input, Kinova::Api::Base::SequenceTasksPair *output);
int ToProtoData(kortex_driver::msg::Sequence input, Kinova::Api::Base::Sequence *output);
int ToProtoData(kortex_driver::msg::SequenceList input, Kinova::Api::Base::SequenceList *output);
int ToProtoData(kortex_driver::msg::AppendActionInformation input, Kinova::Api::Base::AppendActionInformation *output);
int ToProtoData(kortex_driver::msg::ActionHandle input, Kinova::Api::Base::ActionHandle *output);
int ToProtoData(kortex_driver::msg::RequestedActionType input, Kinova::Api::Base::RequestedActionType *output);
int ToProtoData(kortex_driver::msg::Action input, Kinova::Api::Base::Action *output);
int ToProtoData(kortex_driver::msg::Snapshot input, Kinova::Api::Base::Snapshot *output);
int ToProtoData(kortex_driver::msg::SwitchControlMapping input, Kinova::Api::Base::SwitchControlMapping *output);
int ToProtoData(kortex_driver::msg::ChangeTwist input, Kinova::Api::Base::ChangeTwist *output);
int ToProtoData(kortex_driver::msg::ChangeJointSpeeds input, Kinova::Api::Base::ChangeJointSpeeds *output);
int ToProtoData(kortex_driver::msg::ChangeWrench input, Kinova::Api::Base::ChangeWrench *output);
int ToProtoData(kortex_driver::msg::EmergencyStop input, Kinova::Api::Base::EmergencyStop *output);
int ToProtoData(kortex_driver::msg::Faults input, Kinova::Api::Base::Faults *output);
int ToProtoData(kortex_driver::msg::Delay input, Kinova::Api::Base::Delay *output);
int ToProtoData(kortex_driver::msg::BaseStop input, Kinova::Api::Base::Stop *output);
int ToProtoData(kortex_driver::msg::ActionList input, Kinova::Api::Base::ActionList *output);
int ToProtoData(kortex_driver::msg::Timeout input, Kinova::Api::Base::Timeout *output);
int ToProtoData(kortex_driver::msg::Ssid input, Kinova::Api::Base::Ssid *output);
int ToProtoData(kortex_driver::msg::CommunicationInterfaceConfiguration input, Kinova::Api::Base::CommunicationInterfaceConfiguration *output);
int ToProtoData(kortex_driver::msg::NetworkHandle input, Kinova::Api::Base::NetworkHandle *output);
int ToProtoData(kortex_driver::msg::IPv4Configuration input, Kinova::Api::Base::IPv4Configuration *output);
int ToProtoData(kortex_driver::msg::IPv4Information input, Kinova::Api::Base::IPv4Information *output);
int ToProtoData(kortex_driver::msg::FullIPv4Configuration input, Kinova::Api::Base::FullIPv4Configuration *output);
int ToProtoData(kortex_driver::msg::WifiInformation input, Kinova::Api::Base::WifiInformation *output);
int ToProtoData(kortex_driver::msg::WifiInformationList input, Kinova::Api::Base::WifiInformationList *output);
int ToProtoData(kortex_driver::msg::WifiConfiguration input, Kinova::Api::Base::WifiConfiguration *output);
int ToProtoData(kortex_driver::msg::WifiConfigurationList input, Kinova::Api::Base::WifiConfigurationList *output);
int ToProtoData(kortex_driver::msg::ProtectionZoneHandle input, Kinova::Api::Base::ProtectionZoneHandle *output);
int ToProtoData(kortex_driver::msg::BaseRotationMatrixRow input, Kinova::Api::Base::RotationMatrixRow *output);
int ToProtoData(kortex_driver::msg::BaseRotationMatrix input, Kinova::Api::Base::RotationMatrix *output);
int ToProtoData(kortex_driver::msg::Point input, Kinova::Api::Base::Point *output);
int ToProtoData(kortex_driver::msg::ZoneShape input, Kinova::Api::Base::ZoneShape *output);
int ToProtoData(kortex_driver::msg::ProtectionZone input, Kinova::Api::Base::ProtectionZone *output);
int ToProtoData(kortex_driver::msg::ProtectionZoneList input, Kinova::Api::Base::ProtectionZoneList *output);
int ToProtoData(kortex_driver::msg::CartesianLimitation input, Kinova::Api::Base::CartesianLimitation *output);
int ToProtoData(kortex_driver::msg::TwistLimitation input, Kinova::Api::Base::TwistLimitation *output);
int ToProtoData(kortex_driver::msg::WrenchLimitation input, Kinova::Api::Base::WrenchLimitation *output);
int ToProtoData(kortex_driver::msg::CartesianLimitationList input, Kinova::Api::Base::CartesianLimitationList *output);
int ToProtoData(kortex_driver::msg::JointLimitation input, Kinova::Api::Base::JointLimitation *output);
int ToProtoData(kortex_driver::msg::JointsLimitationsList input, Kinova::Api::Base::JointsLimitationsList *output);
int ToProtoData(kortex_driver::msg::Query input, Kinova::Api::Base::Query *output);
int ToProtoData(kortex_driver::msg::ConfigurationChangeNotification input, Kinova::Api::Base::ConfigurationChangeNotification *output);
int ToProtoData(kortex_driver::msg::MappingInfoNotification input, Kinova::Api::Base::MappingInfoNotification *output);
int ToProtoData(kortex_driver::msg::BaseControlModeInformation input, Kinova::Api::Base::ControlModeInformation *output);
int ToProtoData(kortex_driver::msg::BaseControlModeNotification input, Kinova::Api::Base::ControlModeNotification *output);
int ToProtoData(kortex_driver::msg::ServoingModeInformation input, Kinova::Api::Base::ServoingModeInformation *output);
int ToProtoData(kortex_driver::msg::OperatingModeInformation input, Kinova::Api::Base::OperatingModeInformation *output);
int ToProtoData(kortex_driver::msg::OperatingModeNotification input, Kinova::Api::Base::OperatingModeNotification *output);
int ToProtoData(kortex_driver::msg::ServoingModeNotification input, Kinova::Api::Base::ServoingModeNotification *output);
int ToProtoData(kortex_driver::msg::SequenceInfoNotification input, Kinova::Api::Base::SequenceInfoNotification *output);
int ToProtoData(kortex_driver::msg::SequenceInformation input, Kinova::Api::Base::SequenceInformation *output);
int ToProtoData(kortex_driver::msg::ProtectionZoneNotification input, Kinova::Api::Base::ProtectionZoneNotification *output);
int ToProtoData(kortex_driver::msg::ProtectionZoneInformation input, Kinova::Api::Base::ProtectionZoneInformation *output);
int ToProtoData(kortex_driver::msg::UserNotification input, Kinova::Api::Base::UserNotification *output);
int ToProtoData(kortex_driver::msg::ControllerHandle input, Kinova::Api::Base::ControllerHandle *output);
int ToProtoData(kortex_driver::msg::ControllerElementHandle input, Kinova::Api::Base::ControllerElementHandle *output);
int ToProtoData(kortex_driver::msg::ControllerNotification input, Kinova::Api::Base::ControllerNotification *output);
int ToProtoData(kortex_driver::msg::ControllerList input, Kinova::Api::Base::ControllerList *output);
int ToProtoData(kortex_driver::msg::ControllerState input, Kinova::Api::Base::ControllerState *output);
int ToProtoData(kortex_driver::msg::ControllerElementState input, Kinova::Api::Base::ControllerElementState *output);
int ToProtoData(kortex_driver::msg::ActionNotification input, Kinova::Api::Base::ActionNotification *output);
int ToProtoData(kortex_driver::msg::TrajectoryInfo input, Kinova::Api::Base::TrajectoryInfo *output);
int ToProtoData(kortex_driver::msg::ActionExecutionState input, Kinova::Api::Base::ActionExecutionState *output);
int ToProtoData(kortex_driver::msg::RobotEventNotification input, Kinova::Api::Base::RobotEventNotification *output);
int ToProtoData(kortex_driver::msg::FactoryNotification input, Kinova::Api::Base::FactoryNotification *output);
int ToProtoData(kortex_driver::msg::NetworkNotification input, Kinova::Api::Base::NetworkNotification *output);
int ToProtoData(kortex_driver::msg::ConfigurationChangeNotificationList input, Kinova::Api::Base::ConfigurationChangeNotificationList *output);
int ToProtoData(kortex_driver::msg::MappingInfoNotificationList input, Kinova::Api::Base::MappingInfoNotificationList *output);
int ToProtoData(kortex_driver::msg::ControlModeNotificationList input, Kinova::Api::Base::ControlModeNotificationList *output);
int ToProtoData(kortex_driver::msg::OperatingModeNotificationList input, Kinova::Api::Base::OperatingModeNotificationList *output);
int ToProtoData(kortex_driver::msg::ServoingModeNotificationList input, Kinova::Api::Base::ServoingModeNotificationList *output);
int ToProtoData(kortex_driver::msg::SequenceInfoNotificationList input, Kinova::Api::Base::SequenceInfoNotificationList *output);
int ToProtoData(kortex_driver::msg::ProtectionZoneNotificationList input, Kinova::Api::Base::ProtectionZoneNotificationList *output);
int ToProtoData(kortex_driver::msg::UserNotificationList input, Kinova::Api::Base::UserNotificationList *output);
int ToProtoData(kortex_driver::msg::SafetyNotificationList input, Kinova::Api::Base::SafetyNotificationList *output);
int ToProtoData(kortex_driver::msg::ControllerNotificationList input, Kinova::Api::Base::ControllerNotificationList *output);
int ToProtoData(kortex_driver::msg::ActionNotificationList input, Kinova::Api::Base::ActionNotificationList *output);
int ToProtoData(kortex_driver::msg::RobotEventNotificationList input, Kinova::Api::Base::RobotEventNotificationList *output);
int ToProtoData(kortex_driver::msg::NetworkNotificationList input, Kinova::Api::Base::NetworkNotificationList *output);
int ToProtoData(kortex_driver::msg::MappingHandle input, Kinova::Api::Base::MappingHandle *output);
int ToProtoData(kortex_driver::msg::SafetyEvent input, Kinova::Api::Base::SafetyEvent *output);
int ToProtoData(kortex_driver::msg::ControllerEvent input, Kinova::Api::Base::ControllerEvent *output);
int ToProtoData(kortex_driver::msg::GpioEvent input, Kinova::Api::Base::GpioEvent *output);
int ToProtoData(kortex_driver::msg::MapEvent input, Kinova::Api::Base::MapEvent *output);
int ToProtoData(kortex_driver::msg::MapElement input, Kinova::Api::Base::MapElement *output);
int ToProtoData(kortex_driver::msg::ActivateMapHandle input, Kinova::Api::Base::ActivateMapHandle *output);
int ToProtoData(kortex_driver::msg::Map input, Kinova::Api::Base::Map *output);
int ToProtoData(kortex_driver::msg::MapHandle input, Kinova::Api::Base::MapHandle *output);
int ToProtoData(kortex_driver::msg::MapList input, Kinova::Api::Base::MapList *output);
int ToProtoData(kortex_driver::msg::MapGroupHandle input, Kinova::Api::Base::MapGroupHandle *output);
int ToProtoData(kortex_driver::msg::MapGroup input, Kinova::Api::Base::MapGroup *output);
int ToProtoData(kortex_driver::msg::MapGroupList input, Kinova::Api::Base::MapGroupList *output);
int ToProtoData(kortex_driver::msg::Mapping input, Kinova::Api::Base::Mapping *output);
int ToProtoData(kortex_driver::msg::MappingList input, Kinova::Api::Base::MappingList *output);
int ToProtoData(kortex_driver::msg::TransformationMatrix input, Kinova::Api::Base::TransformationMatrix *output);
int ToProtoData(kortex_driver::msg::TransformationRow input, Kinova::Api::Base::TransformationRow *output);
int ToProtoData(kortex_driver::msg::Pose input, Kinova::Api::Base::Pose *output);
int ToProtoData(kortex_driver::msg::BasePosition input, Kinova::Api::Base::Position *output);
int ToProtoData(kortex_driver::msg::Orientation input, Kinova::Api::Base::Orientation *output);
int ToProtoData(kortex_driver::msg::CartesianSpeed input, Kinova::Api::Base::CartesianSpeed *output);
int ToProtoData(kortex_driver::msg::CartesianTrajectoryConstraint input, Kinova::Api::Base::CartesianTrajectoryConstraint *output);
int ToProtoData(kortex_driver::msg::JointTrajectoryConstraint input, Kinova::Api::Base::JointTrajectoryConstraint *output);
int ToProtoData(kortex_driver::msg::Wrench input, Kinova::Api::Base::Wrench *output);
int ToProtoData(kortex_driver::msg::Twist input, Kinova::Api::Base::Twist *output);
int ToProtoData(kortex_driver::msg::Admittance input, Kinova::Api::Base::Admittance *output);
int ToProtoData(kortex_driver::msg::ConstrainedPose input, Kinova::Api::Base::ConstrainedPose *output);
int ToProtoData(kortex_driver::msg::ConstrainedPosition input, Kinova::Api::Base::ConstrainedPosition *output);
int ToProtoData(kortex_driver::msg::ConstrainedOrientation input, Kinova::Api::Base::ConstrainedOrientation *output);
int ToProtoData(kortex_driver::msg::WrenchCommand input, Kinova::Api::Base::WrenchCommand *output);
int ToProtoData(kortex_driver::msg::TwistCommand input, Kinova::Api::Base::TwistCommand *output);
int ToProtoData(kortex_driver::msg::ConstrainedJointAngles input, Kinova::Api::Base::ConstrainedJointAngles *output);
int ToProtoData(kortex_driver::msg::ConstrainedJointAngle input, Kinova::Api::Base::ConstrainedJointAngle *output);
int ToProtoData(kortex_driver::msg::JointAngles input, Kinova::Api::Base::JointAngles *output);
int ToProtoData(kortex_driver::msg::JointAngle input, Kinova::Api::Base::JointAngle *output);
int ToProtoData(kortex_driver::msg::BaseJointSpeeds input, Kinova::Api::Base::JointSpeeds *output);
int ToProtoData(kortex_driver::msg::JointSpeed input, Kinova::Api::Base::JointSpeed *output);
int ToProtoData(kortex_driver::msg::JointTorques input, Kinova::Api::Base::JointTorques *output);
int ToProtoData(kortex_driver::msg::JointTorque input, Kinova::Api::Base::JointTorque *output);
int ToProtoData(kortex_driver::msg::GripperCommand input, Kinova::Api::Base::GripperCommand *output);
int ToProtoData(kortex_driver::msg::GripperRequest input, Kinova::Api::Base::GripperRequest *output);
int ToProtoData(kortex_driver::msg::Gripper input, Kinova::Api::Base::Gripper *output);
int ToProtoData(kortex_driver::msg::Finger input, Kinova::Api::Base::Finger *output);
int ToProtoData(kortex_driver::msg::GpioCommand input, Kinova::Api::Base::GpioCommand *output);
int ToProtoData(kortex_driver::msg::SystemTime input, Kinova::Api::Base::SystemTime *output);
int ToProtoData(kortex_driver::msg::ControllerConfigurationMode input, Kinova::Api::Base::ControllerConfigurationMode *output);
int ToProtoData(kortex_driver::msg::ControllerConfiguration input, Kinova::Api::Base::ControllerConfiguration *output);
int ToProtoData(kortex_driver::msg::ControllerConfigurationList input, Kinova::Api::Base::ControllerConfigurationList *output);
int ToProtoData(kortex_driver::msg::ActuatorInformation input, Kinova::Api::Base::ActuatorInformation *output);
int ToProtoData(kortex_driver::msg::ArmStateInformation input, Kinova::Api::Base::ArmStateInformation *output);
int ToProtoData(kortex_driver::msg::ArmStateNotification input, Kinova::Api::Base::ArmStateNotification *output);
int ToProtoData(kortex_driver::msg::BaseCapSenseConfig input, Kinova::Api::Base::CapSenseConfig *output);
int ToProtoData(kortex_driver::msg::BridgeList input, Kinova::Api::Base::BridgeList *output);
int ToProtoData(kortex_driver::msg::BridgeResult input, Kinova::Api::Base::BridgeResult *output);
int ToProtoData(kortex_driver::msg::BridgeIdentifier input, Kinova::Api::Base::BridgeIdentifier *output);
int ToProtoData(kortex_driver::msg::BridgeConfig input, Kinova::Api::Base::BridgeConfig *output);
int ToProtoData(kortex_driver::msg::BridgePortConfig input, Kinova::Api::Base::BridgePortConfig *output);
int ToProtoData(kortex_driver::msg::PreComputedJointTrajectory input, Kinova::Api::Base::PreComputedJointTrajectory *output);
int ToProtoData(kortex_driver::msg::PreComputedJointTrajectoryElement input, Kinova::Api::Base::PreComputedJointTrajectoryElement *output);
int ToProtoData(kortex_driver::msg::TrajectoryErrorElement input, Kinova::Api::Base::TrajectoryErrorElement *output);
int ToProtoData(kortex_driver::msg::TrajectoryErrorReport input, Kinova::Api::Base::TrajectoryErrorReport *output);
int ToProtoData(kortex_driver::msg::WaypointValidationReport input, Kinova::Api::Base::WaypointValidationReport *output);
int ToProtoData(kortex_driver::msg::Waypoint input, Kinova::Api::Base::Waypoint *output);
int ToProtoData(kortex_driver::msg::AngularWaypoint input, Kinova::Api::Base::AngularWaypoint *output);
int ToProtoData(kortex_driver::msg::CartesianWaypoint input, Kinova::Api::Base::CartesianWaypoint *output);
int ToProtoData(kortex_driver::msg::WaypointList input, Kinova::Api::Base::WaypointList *output);
int ToProtoData(kortex_driver::msg::KinematicTrajectoryConstraints input, Kinova::Api::Base::KinematicTrajectoryConstraints *output);
int ToProtoData(kortex_driver::msg::FirmwareBundleVersions input, Kinova::Api::Base::FirmwareBundleVersions *output);
int ToProtoData(kortex_driver::msg::FirmwareComponentVersion input, Kinova::Api::Base::FirmwareComponentVersion *output);
int ToProtoData(kortex_driver::msg::IKData input, Kinova::Api::Base::IKData *output);

#endif