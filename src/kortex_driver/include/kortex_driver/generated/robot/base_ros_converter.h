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
 
#ifndef _KORTEX_BASE_ROS_CONVERTER_H_
#define _KORTEX_BASE_ROS_CONVERTER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Base.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


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


int ToRosData(Kinova::Api::Base::GpioConfigurationList input, kortex_driver::msg::GpioConfigurationList &output);
int ToRosData(Kinova::Api::Base::GpioConfiguration input, kortex_driver::msg::BaseGpioConfiguration &output);
int ToRosData(Kinova::Api::Base::GpioPinConfiguration input, kortex_driver::msg::GpioPinConfiguration &output);
int ToRosData(Kinova::Api::Base::FullUserProfile input, kortex_driver::msg::FullUserProfile &output);
int ToRosData(Kinova::Api::Base::UserProfile input, kortex_driver::msg::UserProfile &output);
int ToRosData(Kinova::Api::Base::UserProfileList input, kortex_driver::msg::UserProfileList &output);
int ToRosData(Kinova::Api::Base::UserList input, kortex_driver::msg::UserList &output);
int ToRosData(Kinova::Api::Base::PasswordChange input, kortex_driver::msg::PasswordChange &output);
int ToRosData(Kinova::Api::Base::SequenceHandle input, kortex_driver::msg::SequenceHandle &output);
int ToRosData(Kinova::Api::Base::AdvancedSequenceHandle input, kortex_driver::msg::AdvancedSequenceHandle &output);
int ToRosData(Kinova::Api::Base::SequenceTaskHandle input, kortex_driver::msg::SequenceTaskHandle &output);
int ToRosData(Kinova::Api::Base::SequenceTask input, kortex_driver::msg::SequenceTask &output);
int ToRosData(Kinova::Api::Base::SequenceTasks input, kortex_driver::msg::SequenceTasks &output);
int ToRosData(Kinova::Api::Base::SequenceTasksConfiguration input, kortex_driver::msg::SequenceTasksConfiguration &output);
int ToRosData(Kinova::Api::Base::SequenceTaskConfiguration input, kortex_driver::msg::SequenceTaskConfiguration &output);
int ToRosData(Kinova::Api::Base::SequenceTasksRange input, kortex_driver::msg::SequenceTasksRange &output);
int ToRosData(Kinova::Api::Base::SequenceTasksPair input, kortex_driver::msg::SequenceTasksPair &output);
int ToRosData(Kinova::Api::Base::Sequence input, kortex_driver::msg::Sequence &output);
int ToRosData(Kinova::Api::Base::SequenceList input, kortex_driver::msg::SequenceList &output);
int ToRosData(Kinova::Api::Base::AppendActionInformation input, kortex_driver::msg::AppendActionInformation &output);
int ToRosData(Kinova::Api::Base::ActionHandle input, kortex_driver::msg::ActionHandle &output);
int ToRosData(Kinova::Api::Base::RequestedActionType input, kortex_driver::msg::RequestedActionType &output);
int ToRosData(Kinova::Api::Base::Action input, kortex_driver::msg::Action &output);
int ToRosData(Kinova::Api::Base::Snapshot input, kortex_driver::msg::Snapshot &output);
int ToRosData(Kinova::Api::Base::SwitchControlMapping input, kortex_driver::msg::SwitchControlMapping &output);
int ToRosData(Kinova::Api::Base::ChangeTwist input, kortex_driver::msg::ChangeTwist &output);
int ToRosData(Kinova::Api::Base::ChangeJointSpeeds input, kortex_driver::msg::ChangeJointSpeeds &output);
int ToRosData(Kinova::Api::Base::ChangeWrench input, kortex_driver::msg::ChangeWrench &output);
int ToRosData(Kinova::Api::Base::EmergencyStop input, kortex_driver::msg::EmergencyStop &output);
int ToRosData(Kinova::Api::Base::Faults input, kortex_driver::msg::Faults &output);
int ToRosData(Kinova::Api::Base::Delay input, kortex_driver::msg::Delay &output);
int ToRosData(Kinova::Api::Base::Stop input, kortex_driver::msg::BaseStop &output);
int ToRosData(Kinova::Api::Base::ActionList input, kortex_driver::msg::ActionList &output);
int ToRosData(Kinova::Api::Base::Timeout input, kortex_driver::msg::Timeout &output);
int ToRosData(Kinova::Api::Base::Ssid input, kortex_driver::msg::Ssid &output);
int ToRosData(Kinova::Api::Base::CommunicationInterfaceConfiguration input, kortex_driver::msg::CommunicationInterfaceConfiguration &output);
int ToRosData(Kinova::Api::Base::NetworkHandle input, kortex_driver::msg::NetworkHandle &output);
int ToRosData(Kinova::Api::Base::IPv4Configuration input, kortex_driver::msg::IPv4Configuration &output);
int ToRosData(Kinova::Api::Base::IPv4Information input, kortex_driver::msg::IPv4Information &output);
int ToRosData(Kinova::Api::Base::FullIPv4Configuration input, kortex_driver::msg::FullIPv4Configuration &output);
int ToRosData(Kinova::Api::Base::WifiInformation input, kortex_driver::msg::WifiInformation &output);
int ToRosData(Kinova::Api::Base::WifiInformationList input, kortex_driver::msg::WifiInformationList &output);
int ToRosData(Kinova::Api::Base::WifiConfiguration input, kortex_driver::msg::WifiConfiguration &output);
int ToRosData(Kinova::Api::Base::WifiConfigurationList input, kortex_driver::msg::WifiConfigurationList &output);
int ToRosData(Kinova::Api::Base::ProtectionZoneHandle input, kortex_driver::msg::ProtectionZoneHandle &output);
int ToRosData(Kinova::Api::Base::RotationMatrixRow input, kortex_driver::msg::BaseRotationMatrixRow &output);
int ToRosData(Kinova::Api::Base::RotationMatrix input, kortex_driver::msg::BaseRotationMatrix &output);
int ToRosData(Kinova::Api::Base::Point input, kortex_driver::msg::Point &output);
int ToRosData(Kinova::Api::Base::ZoneShape input, kortex_driver::msg::ZoneShape &output);
int ToRosData(Kinova::Api::Base::ProtectionZone input, kortex_driver::msg::ProtectionZone &output);
int ToRosData(Kinova::Api::Base::ProtectionZoneList input, kortex_driver::msg::ProtectionZoneList &output);
int ToRosData(Kinova::Api::Base::CartesianLimitation input, kortex_driver::msg::CartesianLimitation &output);
int ToRosData(Kinova::Api::Base::TwistLimitation input, kortex_driver::msg::TwistLimitation &output);
int ToRosData(Kinova::Api::Base::WrenchLimitation input, kortex_driver::msg::WrenchLimitation &output);
int ToRosData(Kinova::Api::Base::CartesianLimitationList input, kortex_driver::msg::CartesianLimitationList &output);
int ToRosData(Kinova::Api::Base::JointLimitation input, kortex_driver::msg::JointLimitation &output);
int ToRosData(Kinova::Api::Base::JointsLimitationsList input, kortex_driver::msg::JointsLimitationsList &output);
int ToRosData(Kinova::Api::Base::Query input, kortex_driver::msg::Query &output);
int ToRosData(Kinova::Api::Base::ConfigurationChangeNotification input, kortex_driver::msg::ConfigurationChangeNotification &output);
int ToRosData(Kinova::Api::Base::MappingInfoNotification input, kortex_driver::msg::MappingInfoNotification &output);
int ToRosData(Kinova::Api::Base::ControlModeInformation input, kortex_driver::msg::BaseControlModeInformation &output);
int ToRosData(Kinova::Api::Base::ControlModeNotification input, kortex_driver::msg::BaseControlModeNotification &output);
int ToRosData(Kinova::Api::Base::ServoingModeInformation input, kortex_driver::msg::ServoingModeInformation &output);
int ToRosData(Kinova::Api::Base::OperatingModeInformation input, kortex_driver::msg::OperatingModeInformation &output);
int ToRosData(Kinova::Api::Base::OperatingModeNotification input, kortex_driver::msg::OperatingModeNotification &output);
int ToRosData(Kinova::Api::Base::ServoingModeNotification input, kortex_driver::msg::ServoingModeNotification &output);
int ToRosData(Kinova::Api::Base::SequenceInfoNotification input, kortex_driver::msg::SequenceInfoNotification &output);
int ToRosData(Kinova::Api::Base::SequenceInformation input, kortex_driver::msg::SequenceInformation &output);
int ToRosData(Kinova::Api::Base::ProtectionZoneNotification input, kortex_driver::msg::ProtectionZoneNotification &output);
int ToRosData(Kinova::Api::Base::ProtectionZoneInformation input, kortex_driver::msg::ProtectionZoneInformation &output);
int ToRosData(Kinova::Api::Base::UserNotification input, kortex_driver::msg::UserNotification &output);
int ToRosData(Kinova::Api::Base::ControllerHandle input, kortex_driver::msg::ControllerHandle &output);
int ToRosData(Kinova::Api::Base::ControllerElementHandle input, kortex_driver::msg::ControllerElementHandle &output);
int ToRosData(Kinova::Api::Base::ControllerNotification input, kortex_driver::msg::ControllerNotification &output);
int ToRosData(Kinova::Api::Base::ControllerList input, kortex_driver::msg::ControllerList &output);
int ToRosData(Kinova::Api::Base::ControllerState input, kortex_driver::msg::ControllerState &output);
int ToRosData(Kinova::Api::Base::ControllerElementState input, kortex_driver::msg::ControllerElementState &output);
int ToRosData(Kinova::Api::Base::ActionNotification input, kortex_driver::msg::ActionNotification &output);
int ToRosData(Kinova::Api::Base::TrajectoryInfo input, kortex_driver::msg::TrajectoryInfo &output);
int ToRosData(Kinova::Api::Base::ActionExecutionState input, kortex_driver::msg::ActionExecutionState &output);
int ToRosData(Kinova::Api::Base::RobotEventNotification input, kortex_driver::msg::RobotEventNotification &output);
int ToRosData(Kinova::Api::Base::FactoryNotification input, kortex_driver::msg::FactoryNotification &output);
int ToRosData(Kinova::Api::Base::NetworkNotification input, kortex_driver::msg::NetworkNotification &output);
int ToRosData(Kinova::Api::Base::ConfigurationChangeNotificationList input, kortex_driver::msg::ConfigurationChangeNotificationList &output);
int ToRosData(Kinova::Api::Base::MappingInfoNotificationList input, kortex_driver::msg::MappingInfoNotificationList &output);
int ToRosData(Kinova::Api::Base::ControlModeNotificationList input, kortex_driver::msg::ControlModeNotificationList &output);
int ToRosData(Kinova::Api::Base::OperatingModeNotificationList input, kortex_driver::msg::OperatingModeNotificationList &output);
int ToRosData(Kinova::Api::Base::ServoingModeNotificationList input, kortex_driver::msg::ServoingModeNotificationList &output);
int ToRosData(Kinova::Api::Base::SequenceInfoNotificationList input, kortex_driver::msg::SequenceInfoNotificationList &output);
int ToRosData(Kinova::Api::Base::ProtectionZoneNotificationList input, kortex_driver::msg::ProtectionZoneNotificationList &output);
int ToRosData(Kinova::Api::Base::UserNotificationList input, kortex_driver::msg::UserNotificationList &output);
int ToRosData(Kinova::Api::Base::SafetyNotificationList input, kortex_driver::msg::SafetyNotificationList &output);
int ToRosData(Kinova::Api::Base::ControllerNotificationList input, kortex_driver::msg::ControllerNotificationList &output);
int ToRosData(Kinova::Api::Base::ActionNotificationList input, kortex_driver::msg::ActionNotificationList &output);
int ToRosData(Kinova::Api::Base::RobotEventNotificationList input, kortex_driver::msg::RobotEventNotificationList &output);
int ToRosData(Kinova::Api::Base::NetworkNotificationList input, kortex_driver::msg::NetworkNotificationList &output);
int ToRosData(Kinova::Api::Base::MappingHandle input, kortex_driver::msg::MappingHandle &output);
int ToRosData(Kinova::Api::Base::SafetyEvent input, kortex_driver::msg::SafetyEvent &output);
int ToRosData(Kinova::Api::Base::ControllerEvent input, kortex_driver::msg::ControllerEvent &output);
int ToRosData(Kinova::Api::Base::GpioEvent input, kortex_driver::msg::GpioEvent &output);
int ToRosData(Kinova::Api::Base::MapEvent input, kortex_driver::msg::MapEvent &output);
int ToRosData(Kinova::Api::Base::MapElement input, kortex_driver::msg::MapElement &output);
int ToRosData(Kinova::Api::Base::ActivateMapHandle input, kortex_driver::msg::ActivateMapHandle &output);
int ToRosData(Kinova::Api::Base::Map input, kortex_driver::msg::Map &output);
int ToRosData(Kinova::Api::Base::MapHandle input, kortex_driver::msg::MapHandle &output);
int ToRosData(Kinova::Api::Base::MapList input, kortex_driver::msg::MapList &output);
int ToRosData(Kinova::Api::Base::MapGroupHandle input, kortex_driver::msg::MapGroupHandle &output);
int ToRosData(Kinova::Api::Base::MapGroup input, kortex_driver::msg::MapGroup &output);
int ToRosData(Kinova::Api::Base::MapGroupList input, kortex_driver::msg::MapGroupList &output);
int ToRosData(Kinova::Api::Base::Mapping input, kortex_driver::msg::Mapping &output);
int ToRosData(Kinova::Api::Base::MappingList input, kortex_driver::msg::MappingList &output);
int ToRosData(Kinova::Api::Base::TransformationMatrix input, kortex_driver::msg::TransformationMatrix &output);
int ToRosData(Kinova::Api::Base::TransformationRow input, kortex_driver::msg::TransformationRow &output);
int ToRosData(Kinova::Api::Base::Pose input, kortex_driver::msg::Pose &output);
int ToRosData(Kinova::Api::Base::Position input, kortex_driver::msg::BasePosition &output);
int ToRosData(Kinova::Api::Base::Orientation input, kortex_driver::msg::Orientation &output);
int ToRosData(Kinova::Api::Base::CartesianSpeed input, kortex_driver::msg::CartesianSpeed &output);
int ToRosData(Kinova::Api::Base::CartesianTrajectoryConstraint input, kortex_driver::msg::CartesianTrajectoryConstraint &output);
int ToRosData(Kinova::Api::Base::JointTrajectoryConstraint input, kortex_driver::msg::JointTrajectoryConstraint &output);
int ToRosData(Kinova::Api::Base::Wrench input, kortex_driver::msg::Wrench &output);
int ToRosData(Kinova::Api::Base::Twist input, kortex_driver::msg::Twist &output);
int ToRosData(Kinova::Api::Base::Admittance input, kortex_driver::msg::Admittance &output);
int ToRosData(Kinova::Api::Base::ConstrainedPose input, kortex_driver::msg::ConstrainedPose &output);
int ToRosData(Kinova::Api::Base::ConstrainedPosition input, kortex_driver::msg::ConstrainedPosition &output);
int ToRosData(Kinova::Api::Base::ConstrainedOrientation input, kortex_driver::msg::ConstrainedOrientation &output);
int ToRosData(Kinova::Api::Base::WrenchCommand input, kortex_driver::msg::WrenchCommand &output);
int ToRosData(Kinova::Api::Base::TwistCommand input, kortex_driver::msg::TwistCommand &output);
int ToRosData(Kinova::Api::Base::ConstrainedJointAngles input, kortex_driver::msg::ConstrainedJointAngles &output);
int ToRosData(Kinova::Api::Base::ConstrainedJointAngle input, kortex_driver::msg::ConstrainedJointAngle &output);
int ToRosData(Kinova::Api::Base::JointAngles input, kortex_driver::msg::JointAngles &output);
int ToRosData(Kinova::Api::Base::JointAngle input, kortex_driver::msg::JointAngle &output);
int ToRosData(Kinova::Api::Base::JointSpeeds input, kortex_driver::msg::BaseJointSpeeds &output);
int ToRosData(Kinova::Api::Base::JointSpeed input, kortex_driver::msg::JointSpeed &output);
int ToRosData(Kinova::Api::Base::JointTorques input, kortex_driver::msg::JointTorques &output);
int ToRosData(Kinova::Api::Base::JointTorque input, kortex_driver::msg::JointTorque &output);
int ToRosData(Kinova::Api::Base::GripperCommand input, kortex_driver::msg::GripperCommand &output);
int ToRosData(Kinova::Api::Base::GripperRequest input, kortex_driver::msg::GripperRequest &output);
int ToRosData(Kinova::Api::Base::Gripper input, kortex_driver::msg::Gripper &output);
int ToRosData(Kinova::Api::Base::Finger input, kortex_driver::msg::Finger &output);
int ToRosData(Kinova::Api::Base::GpioCommand input, kortex_driver::msg::GpioCommand &output);
int ToRosData(Kinova::Api::Base::SystemTime input, kortex_driver::msg::SystemTime &output);
int ToRosData(Kinova::Api::Base::ControllerConfigurationMode input, kortex_driver::msg::ControllerConfigurationMode &output);
int ToRosData(Kinova::Api::Base::ControllerConfiguration input, kortex_driver::msg::ControllerConfiguration &output);
int ToRosData(Kinova::Api::Base::ControllerConfigurationList input, kortex_driver::msg::ControllerConfigurationList &output);
int ToRosData(Kinova::Api::Base::ActuatorInformation input, kortex_driver::msg::ActuatorInformation &output);
int ToRosData(Kinova::Api::Base::ArmStateInformation input, kortex_driver::msg::ArmStateInformation &output);
int ToRosData(Kinova::Api::Base::ArmStateNotification input, kortex_driver::msg::ArmStateNotification &output);
int ToRosData(Kinova::Api::Base::CapSenseConfig input, kortex_driver::msg::BaseCapSenseConfig &output);
int ToRosData(Kinova::Api::Base::BridgeList input, kortex_driver::msg::BridgeList &output);
int ToRosData(Kinova::Api::Base::BridgeResult input, kortex_driver::msg::BridgeResult &output);
int ToRosData(Kinova::Api::Base::BridgeIdentifier input, kortex_driver::msg::BridgeIdentifier &output);
int ToRosData(Kinova::Api::Base::BridgeConfig input, kortex_driver::msg::BridgeConfig &output);
int ToRosData(Kinova::Api::Base::BridgePortConfig input, kortex_driver::msg::BridgePortConfig &output);
int ToRosData(Kinova::Api::Base::PreComputedJointTrajectory input, kortex_driver::msg::PreComputedJointTrajectory &output);
int ToRosData(Kinova::Api::Base::PreComputedJointTrajectoryElement input, kortex_driver::msg::PreComputedJointTrajectoryElement &output);
int ToRosData(Kinova::Api::Base::TrajectoryErrorElement input, kortex_driver::msg::TrajectoryErrorElement &output);
int ToRosData(Kinova::Api::Base::TrajectoryErrorReport input, kortex_driver::msg::TrajectoryErrorReport &output);
int ToRosData(Kinova::Api::Base::WaypointValidationReport input, kortex_driver::msg::WaypointValidationReport &output);
int ToRosData(Kinova::Api::Base::Waypoint input, kortex_driver::msg::Waypoint &output);
int ToRosData(Kinova::Api::Base::AngularWaypoint input, kortex_driver::msg::AngularWaypoint &output);
int ToRosData(Kinova::Api::Base::CartesianWaypoint input, kortex_driver::msg::CartesianWaypoint &output);
int ToRosData(Kinova::Api::Base::WaypointList input, kortex_driver::msg::WaypointList &output);
int ToRosData(Kinova::Api::Base::KinematicTrajectoryConstraints input, kortex_driver::msg::KinematicTrajectoryConstraints &output);
int ToRosData(Kinova::Api::Base::FirmwareBundleVersions input, kortex_driver::msg::FirmwareBundleVersions &output);
int ToRosData(Kinova::Api::Base::FirmwareComponentVersion input, kortex_driver::msg::FirmwareComponentVersion &output);
int ToRosData(Kinova::Api::Base::IKData input, kortex_driver::msg::IKData &output);

#endif