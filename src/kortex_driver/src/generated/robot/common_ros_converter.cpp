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
 
#include "kortex_driver/generated/robot/common_ros_converter.h"

int ToRosData(Kinova::Api::Common::DeviceHandle input, kortex_driver::msg::DeviceHandle &output)
{
	
	output.device_type = input.device_type();
	output.device_identifier = input.device_identifier();
	output.order = input.order();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Common::Empty input, kortex_driver::msg::Empty &output)
{
	

	
	
	return 0;
}
int ToRosData(Kinova::Api::Common::NotificationOptions input, kortex_driver::msg::NotificationOptions &output)
{
	
	output.type = input.type();
	output.rate_m_sec = input.rate_m_sec();
	output.threshold_value = input.threshold_value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Common::SafetyHandle input, kortex_driver::msg::SafetyHandle &output)
{
	
	output.identifier = input.identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Common::NotificationHandle input, kortex_driver::msg::NotificationHandle &output)
{
	
	output.identifier = input.identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Common::SafetyNotification input, kortex_driver::msg::SafetyNotification &output)
{
	
	ToRosData(input.safety_handle(), output.safety_handle);
	output.value = input.value();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Common::Timestamp input, kortex_driver::msg::Timestamp &output)
{
	
	output.sec = input.sec();
	output.usec = input.usec();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Common::UserProfileHandle input, kortex_driver::msg::UserProfileHandle &output)
{
	
	output.identifier = input.identifier();
	output.permission = input.permission();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Common::Connection input, kortex_driver::msg::Connection &output)
{
	
	ToRosData(input.user_handle(), output.user_handle);
	output.connection_information = input.connection_information();
	output.connection_identifier = input.connection_identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Common::UARTConfiguration input, kortex_driver::msg::UARTConfiguration &output)
{
	
	output.port_id = input.port_id();
	output.enabled = input.enabled();
	output.speed = input.speed();
	output.word_length = input.word_length();
	output.stop_bits = input.stop_bits();
	output.parity = input.parity();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Common::UARTDeviceIdentification input, kortex_driver::msg::UARTDeviceIdentification &output)
{
	
	output.port_id = input.port_id();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Common::CountryCode input, kortex_driver::msg::CountryCode &output)
{
	
	output.identifier = input.identifier();

	
	
	return 0;
}
