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

#include "kortex_driver/non-generated/kortex_arm_driver.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //     ros::console::notifyLoggerLevelsChanged();
    // }
    
    std::shared_ptr<rclcpp::Node> n = rclcpp::Node::make_shared("kortex_arm_driver");

    KortexArmDriver kortex_arm_driver(n);

    rclcpp::spin(n);
    rclcpp::shutdown();

    return 1;
}