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
 
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"

int ToRosData(Kinova::Api::ControlConfig::GravityVector input, kortex_driver::msg::GravityVector &output)
{
	
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::Position input, kortex_driver::msg::ControlConfigPosition &output)
{
	
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::PayloadInformation input, kortex_driver::msg::PayloadInformation &output)
{
	
	output.payload_mass = input.payload_mass();
	ToRosData(input.payload_mass_center(), output.payload_mass_center);

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::CartesianTransform input, kortex_driver::msg::CartesianTransform &output)
{
	
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();
	output.theta_x = input.theta_x();
	output.theta_y = input.theta_y();
	output.theta_z = input.theta_z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::ToolConfiguration input, kortex_driver::msg::ToolConfiguration &output)
{
	
	ToRosData(input.tool_transform(), output.tool_transform);
	output.tool_mass = input.tool_mass();
	ToRosData(input.tool_mass_center(), output.tool_mass_center);

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::ControlConfigurationNotification input, kortex_driver::msg::ControlConfigurationNotification &output)
{
	
	output.event = input.event();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::CartesianReferenceFrameInfo input, kortex_driver::msg::CartesianReferenceFrameInfo &output)
{
	
	output.reference_frame = input.reference_frame();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::TwistLinearSoftLimit input, kortex_driver::msg::TwistLinearSoftLimit &output)
{
	
	output.control_mode = input.control_mode();
	output.twist_linear_soft_limit = input.twist_linear_soft_limit();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::TwistAngularSoftLimit input, kortex_driver::msg::TwistAngularSoftLimit &output)
{
	
	output.control_mode = input.control_mode();
	output.twist_angular_soft_limit = input.twist_angular_soft_limit();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::JointSpeedSoftLimits input, kortex_driver::msg::JointSpeedSoftLimits &output)
{
	
	output.control_mode = input.control_mode();
	output.joint_speed_soft_limits.clear();
	for(int i = 0; i < input.joint_speed_soft_limits_size(); i++)
	{
		output.joint_speed_soft_limits.push_back(input.joint_speed_soft_limits(i));
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::JointAccelerationSoftLimits input, kortex_driver::msg::JointAccelerationSoftLimits &output)
{
	
	output.control_mode = input.control_mode();
	output.joint_acceleration_soft_limits.clear();
	for(int i = 0; i < input.joint_acceleration_soft_limits_size(); i++)
	{
		output.joint_acceleration_soft_limits.push_back(input.joint_acceleration_soft_limits(i));
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::KinematicLimits input, kortex_driver::msg::KinematicLimits &output)
{
	
	output.control_mode = input.control_mode();
	output.twist_linear = input.twist_linear();
	output.twist_angular = input.twist_angular();
	output.joint_speed_limits.clear();
	for(int i = 0; i < input.joint_speed_limits_size(); i++)
	{
		output.joint_speed_limits.push_back(input.joint_speed_limits(i));
	}
	output.joint_acceleration_limits.clear();
	for(int i = 0; i < input.joint_acceleration_limits_size(); i++)
	{
		output.joint_acceleration_limits.push_back(input.joint_acceleration_limits(i));
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::KinematicLimitsList input, kortex_driver::msg::KinematicLimitsList &output)
{
	
	output.kinematic_limits_list.clear();
	for(int i = 0; i < input.kinematic_limits_list_size(); i++)
	{
		decltype(output.kinematic_limits_list)::value_type temp;
		ToRosData(input.kinematic_limits_list(i), temp);
		output.kinematic_limits_list.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::DesiredSpeeds input, kortex_driver::msg::DesiredSpeeds &output)
{
	
	output.linear = input.linear();
	output.angular = input.angular();
	output.joint_speed.clear();
	for(int i = 0; i < input.joint_speed_size(); i++)
	{
		output.joint_speed.push_back(input.joint_speed(i));
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::LinearTwist input, kortex_driver::msg::LinearTwist &output)
{
	
	output.linear = input.linear();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::AngularTwist input, kortex_driver::msg::AngularTwist &output)
{
	
	output.angular = input.angular();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::JointSpeeds input, kortex_driver::msg::ControlConfigJointSpeeds &output)
{
	
	output.joint_speed.clear();
	for(int i = 0; i < input.joint_speed_size(); i++)
	{
		output.joint_speed.push_back(input.joint_speed(i));
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::ControlModeInformation input, kortex_driver::msg::ControlConfigControlModeInformation &output)
{
	
	output.control_mode = input.control_mode();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::ControlModeNotification input, kortex_driver::msg::ControlConfigControlModeNotification &output)
{
	
	output.control_mode = input.control_mode();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
