#ifndef _KORTEX_SUBSCRIBERS_H_
#define _KORTEX_SUBSCRIBERS_H_

/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2020 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/Empty.h"
#include <thread>
#include "BaseClientRpc.h"

#include "kortex_driver/msg/twist_command.hpp"
#include "kortex_driver/msg/base_joint_speeds.hpp"

#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/non-generated/kortex_math_util.h"

class KortexSubscribers
{

public:
    
    KortexSubscribers(rclcpp::Node::SharedPtr node_handle, Kinova::Api::Base::BaseClient* base);
    ~KortexSubscribers();

private:

    rclcpp::Node::SharedPtr m_node_handle;
    Kinova::Api::Base::BaseClient* m_base;

    // Subscribers
    ros::Subscriber m_joint_speeds_sub;
    ros::Subscriber m_twist_sub;
    ros::Subscriber m_clear_faults_sub;
    ros::Subscriber m_stop_sub;
    ros::Subscriber m_emergency_stop_sub;

    // Callbacks
    void new_joint_speeds_cb(const kortex_driver::Base_JointSpeeds& joint_speeds);
    void new_twist_cb(const kortex_driver::TwistCommand& twist);
    void clear_faults_cb(const std_msgs::Empty& empty);
    void stop_cb(const std_msgs::Empty& empty);
    void emergency_stop_cb(const std_msgs::Empty& empty);
};

#endif //_KORTEX_SUBSCRIBERS_H_
