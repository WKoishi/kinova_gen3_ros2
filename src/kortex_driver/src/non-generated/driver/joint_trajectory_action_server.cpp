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

#include "kortex_driver/non-generated/joint_trajectory_action_server.h"
#include <sstream>
#include <fstream>

namespace {
    constexpr float STARTING_POINT_ARBITRARY_DURATION = 0.5f;
}

JointTrajectoryActionServer::JointTrajectoryActionServer(const std::string& server_name, rclcpp::Node::SharedPtr nh, Kinova::Api::Base::BaseClient* base, Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic, Kinova::Api::ControlConfig::ControlConfigClient* control_config, bool use_hard_limits):
    m_server_name(server_name),
    m_node_handle(nh),
    m_base(base),
    m_base_cyclic(base_cyclic),
    m_control_config(control_config),
    m_server_state(ActionServerState::INITIALIZING),
    m_use_hard_limits(use_hard_limits)
{
    // Get the ROS params
    if (!m_node_handle->get_parameter("~default_goal_time_tolerance", m_default_goal_time_tolerance))
    {
        RCLCPP_WARN(m_node_handle->get_logger(), "Parameter default_goal_time_tolerance was not specified; assuming 0.5 as default value.");
        m_default_goal_time_tolerance = 0.5;
    }
    if (!m_node_handle->get_parameter("~default_goal_tolerance", m_default_goal_tolerance))
    {
        RCLCPP_WARN(m_node_handle->get_logger(), "Parameter default_goal_tolerance was not specified; assuming 0.005 as default value.");
        m_default_goal_tolerance = 0.005;
    }
    if (!m_node_handle->get_parameter("~prefix", m_prefix))
    {
        std::string error_string = "Prefix name was not specified in the launch file, shutting down the node...";
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }
    if (!m_node_handle->get_parameter("~joint_names", m_joint_names))
    {
        std::string error_string = "Parameter joint_names was not specified";
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }

    // Get the hard limits
    m_hard_limits = m_control_config->GetKinematicHardLimits();
    
    // Subscribe to the arm's Action Notifications
    m_sub_action_notif_handle = m_base->OnNotificationActionTopic(std::bind(&JointTrajectoryActionServer::action_notif_callback, this, std::placeholders::_1), Kinova::Api::Common::NotificationOptions());

    // Ready to receive goal
    m_server = rclcpp_action::create_server<FollowJointTrajectoryAction>(
        m_node_handle, m_server_name,
        std::bind(&JointTrajectoryActionServer::ros_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&JointTrajectoryActionServer::ros_cancel_callback, this, std::placeholders::_1),
        std::bind(&JointTrajectoryActionServer::ros_accepted_callback, this, std::placeholders::_1));

    set_server_state(ActionServerState::IDLE);
}

JointTrajectoryActionServer::~JointTrajectoryActionServer()
{
    m_base->Unsubscribe(m_sub_action_notif_handle);
}

rclcpp_action::GoalResponse JointTrajectoryActionServer::ros_goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal)
{
    (void)uuid;

    RCLCPP_INFO(m_node_handle->get_logger(), "New goal received.");
    if (!is_goal_acceptable(goal))
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Joint Trajectory Goal is rejected.");
        // new_goal_handle.setRejected();
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(m_node_handle->get_logger(), "Joint Trajectory Goal is accepted.");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void JointTrajectoryActionServer::ros_accepted_callback(const std::shared_ptr<GoalHandle_FollowJointTrajectoryAction> new_goal_handle)
{
    const auto result_msg = std::make_shared<FollowJointTrajectoryAction::Result>();

    // Accept the goal
    kortex_callback.goal_handle = new_goal_handle;

    if (m_server_state != ActionServerState::IDLE)
    {
        RCLCPP_WARN(m_node_handle->get_logger(), "There is already an active goal. It is being cancelled.");
        // We have to call Stop after having received the ACTION_START notification from the arm
        stop_all_movement();
    }

    // // Accept the goal
    // RCLCPP_INFO(m_node_handle->get_logger(), "Joint Trajectory Goal is accepted.");
    // m_goal = new_goal_handle;
    // m_goal.setAccepted();

    // Construct the Protobuf object trajectory
    Kinova::Api::Base::WaypointList proto_trajectory;

    // Uncomment for logging purposes
    // std::ofstream myfile;
    // myfile.open ("/tmp/moveit_trajectory.csv");
    // for (unsigned int i = 0; i < new_goal_handle.getGoal()->trajectory.points.size(); i++)
    // {
    //     myfile << new_goal_handle.getGoal()->trajectory.points[i].time_from_start.toSec() << ",";
    //     for (auto position : new_goal_handle.getGoal()->trajectory.points[i].positions)
    //     {
    //         myfile << position << ",";
    //     }
    //     myfile << "\n";
    // }
    // myfile.close();

    // Copy the trajectory points from the ROS structure to the Protobuf structure
    for (unsigned int i = 0; i < new_goal_handle->get_goal()->trajectory.points.size(); i++)
    {
        // Create the waypoint
        Kinova::Api::Base::Waypoint* proto_waypoint = proto_trajectory.add_waypoints();
        proto_waypoint->set_name("waypoint_" + std::to_string(i));
        auto angular_waypoint = proto_waypoint->mutable_angular_waypoint();
        
        // Calculate the duration of this waypoint
        const auto traj_point = new_goal_handle->get_goal()->trajectory.points.at(i);
        float waypoint_duration;
        // FIXME We have to include the starting point for the trajectory as the first waypoint AND since optimal durations are not
        //       supported yet, we also have to give it a non-zero duration. Putting something too small gives errors so I've put 
        //       an arbitrary value of 500ms here for now.
        if (i == 0)
        {
            waypoint_duration = STARTING_POINT_ARBITRARY_DURATION;
        }
        else
        {
            auto previous_wp_duration = KortexMathUtil::duration_toSec(new_goal_handle->get_goal()->trajectory.points.at(i-1).time_from_start);
            waypoint_duration = KortexMathUtil::duration_toSec(traj_point.time_from_start) - previous_wp_duration;
            // Very small durations are rejected so we cap them
            if (waypoint_duration < 0.01)
            {
                waypoint_duration = 0.01f;
            }
        }
        
        // Fill the waypoint
        for (auto position : traj_point.positions)
        {
            angular_waypoint->add_angles(KortexMathUtil::toDeg(position));
        }
        angular_waypoint->set_duration(waypoint_duration);
    }

    // If the hard limits are used for the trajectory
    if (m_use_hard_limits)
    {
        // Get the soft limits
        m_soft_limits = getAngularTrajectorySoftLimits();

        // Set the soft limits to hard limits
        setAngularTrajectorySoftLimitsToMax();
    }

    try
    {
        // Validate the waypoints and reject the goal if they fail validation
        auto report = m_base->ValidateWaypointList(proto_trajectory);

        if (report.trajectory_error_report().trajectory_error_elements_size() > 0)
        {
            RCLCPP_ERROR(m_node_handle->get_logger(), "Joint Trajectory failed validation in the arm.");
            // Go through report and print errors
            for (unsigned int i = 0; i < report.trajectory_error_report().trajectory_error_elements_size(); i++)
            {
                RCLCPP_ERROR(m_node_handle->get_logger(), "Error %i : %s", i+1, report.trajectory_error_report().trajectory_error_elements(i).message().c_str());
            }
            setAngularTrajectorySoftLimits(m_soft_limits);
            new_goal_handle->abort(result_msg);
            return;
        }
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Kortex exception while validating the trajectory");
        RCLCPP_ERROR(m_node_handle->get_logger(), "Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        RCLCPP_ERROR(m_node_handle->get_logger(), "Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        RCLCPP_ERROR(m_node_handle->get_logger(), "Error description: %s\n", ex.what());
        new_goal_handle->abort(result_msg);
    }
    catch (std::runtime_error& ex_runtime)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Runtime exception detected while sending the trajectory");
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", ex_runtime.what());
        new_goal_handle->abort(result_msg);
    }
    catch (std::future_error& ex_future)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Future exception detected while getting feedback");
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", ex_future.what());
        new_goal_handle->abort(result_msg);
    }
    
    try
    {
        // Make sure to clear the faults before moving the robot
        m_base->ClearFaults();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Send the trajectory to the robot
        m_base->ExecuteWaypointTrajectory(proto_trajectory);
        set_server_state(ActionServerState::PRE_PROCESSING_PENDING);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Kortex exception while sending the trajectory");
        RCLCPP_ERROR(m_node_handle->get_logger(), "Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        RCLCPP_ERROR(m_node_handle->get_logger(), "Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        RCLCPP_ERROR(m_node_handle->get_logger(), "Error description: %s\n", ex.what());
        setAngularTrajectorySoftLimits(m_soft_limits);
        new_goal_handle->abort(result_msg);
    }
    catch (std::runtime_error& ex_runtime)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Runtime exception detected while sending the trajectory");
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", ex_runtime.what());
        setAngularTrajectorySoftLimits(m_soft_limits);
        new_goal_handle->abort(result_msg);
    }
    catch (std::future_error& ex_future)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Future exception detected while getting feedback");
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", ex_future.what());
        setAngularTrajectorySoftLimits(m_soft_limits);
        new_goal_handle->abort(result_msg);
    }
}

// Called in a separate thread when a preempt request comes in from the Action Client
rclcpp_action::CancelResponse JointTrajectoryActionServer::ros_cancel_callback(const std::shared_ptr<GoalHandle_FollowJointTrajectoryAction> goal_handle)
{
    (void)goal_handle;
    if (m_server_state == ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS)
    {
        stop_all_movement();
    }

    return rclcpp_action::CancelResponse::ACCEPT;
}

// Called in a separate thread when a notification comes in
void JointTrajectoryActionServer::action_notif_callback(Kinova::Api::Base::ActionNotification notif)
{
    Kinova::Api::Base::ActionEvent event = notif.action_event();
    Kinova::Api::Base::ActionHandle handle = notif.handle();
    Kinova::Api::Base::ActionType type = handle.action_type();
    RCLCPP_DEBUG(m_node_handle->get_logger(), "Action notification received of type %s", Kinova::Api::Base::ActionEvent_Name(event).c_str());
    std::lock_guard<std::mutex> guard(m_action_notification_thread_lock);

    auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
    std::ostringstream oss;

    if (type == Kinova::Api::Base::ActionType::EXECUTE_WAYPOINT_LIST)
    {
        switch (event)
        {
        // The pre-processing is starting in the arm
        case Kinova::Api::Base::ActionEvent::ACTION_PREPROCESS_START:
            // It should be starting
            if (m_server_state == ActionServerState::PRE_PROCESSING_PENDING)
            {
                RCLCPP_INFO(m_node_handle->get_logger(), "Preprocessing has started in the arm.");
                set_server_state(ActionServerState::PRE_PROCESSING_IN_PROGRESS);
            }
            // We should not have received that
            else
            {
                RCLCPP_DEBUG(m_node_handle->get_logger(), "Notification mismatch : received ACTION_PREPROCESS_START but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;

        // The pre-processing has ended successfully in the arm
        case Kinova::Api::Base::ActionEvent::ACTION_PREPROCESS_END:
            // It was ongoing and now it ended
            if (m_server_state == ActionServerState::PRE_PROCESSING_PENDING ||
                m_server_state == ActionServerState::PRE_PROCESSING_IN_PROGRESS)
            {
                RCLCPP_INFO(m_node_handle->get_logger(), "Preprocessing has finished in the arm and goal has been accepted.");
                set_server_state(ActionServerState::TRAJECTORY_EXECUTION_PENDING);
            }
            // FIXME KOR-3563 Sometimes the notifications arrive in the wrong order so it is possible to receive
            // a ACTION_PREPROCESS_END notification after the ACTION_START
            // When this bug will be fixed this else if can be removed
            else if (m_server_state == ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS)
            {
                RCLCPP_DEBUG(m_node_handle->get_logger(), "Notification order mismatch : We received the ACTION_PREPROCESS_END after the ACTION_START");
                break;
            }
            // We should not have received that
            else
            {
                RCLCPP_DEBUG(m_node_handle->get_logger(), "Notification mismatch : received ACTION_PREPROCESS_END but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;

        // The pre-processing has failed in the arm
        case Kinova::Api::Base::ActionEvent::ACTION_PREPROCESS_ABORT:
            // It was ongoing and now it ended (and failed)
            if ((m_server_state == ActionServerState::PRE_PROCESSING_IN_PROGRESS))
            {
                RCLCPP_ERROR(m_node_handle->get_logger(), "Preprocessing has finished in the arm and goal has been rejected. Fetching the error report from the arm...");

                result->error_code = result->INVALID_GOAL;

                // Get the error report and show errors here
                Kinova::Api::Base::TrajectoryErrorReport report = m_base->GetTrajectoryErrorReport();
                oss << "Error report has been fetched and error elements are listed below : " << std::endl;
                int i = 1;
                for (auto error_element : report.trajectory_error_elements())
                {
                    oss << "-----------------------------" << std::endl;
                    oss << "Error #" << i << std::endl;
                    oss << "Type : " << Kinova::Api::Base::TrajectoryErrorType_Name(error_element.error_type()) << std::endl;
                    oss << "Actuator : " << error_element.index()+1 << std::endl;
                    oss << "Erroneous value is " << error_element.error_value() << " but minimum permitted is " << error_element.min_value() << " and maximum permitted is " << error_element.max_value() << std::endl;
                    if (error_element.message() != "")
                    {
                        oss << "Additional message is : " << error_element.message() << std::endl;
                    }
                    oss << "-----------------------------" << std::endl;

                    i++;
                }

                RCLCPP_ERROR(m_node_handle->get_logger(), "%s", oss.str().c_str());

                result->error_string = oss.str();
                kortex_callback.goal_handle->abort(result);

                set_server_state(ActionServerState::IDLE);
            }
            // We should not have received that
            else
            {
                RCLCPP_DEBUG(m_node_handle->get_logger(), "Notification mismatch : received ACTION_PREPROCESS_ABORT but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;

        // The arm is starting to move
        case Kinova::Api::Base::ActionEvent::ACTION_START:
            // The preprocessing was done and the goal is still active (not preempted)
            if ((m_server_state == ActionServerState::TRAJECTORY_EXECUTION_PENDING ||
                 m_server_state == ActionServerState::PRE_PROCESSING_IN_PROGRESS) && // FIXME KOR-3563 this happens if we received a ACTION_START before a ACTION_PREPROCESS_END
                 kortex_callback.goal_handle->is_active())
            {
                RCLCPP_INFO(m_node_handle->get_logger(), "Trajectory has started.");
                set_server_state(ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS);
                // Remember when the trajectory started
                m_trajectory_start_time = std::chrono::system_clock::now();
            }
            // The preprocessing was done but the goal put to "PREEMPTING" by the client while preprocessing
            // The stop_all_movement() call will trigger a ACTION_ABORT notification
            else if ((m_server_state == ActionServerState::TRAJECTORY_EXECUTION_PENDING) &&
                      kortex_callback.goal_handle->is_canceling())
            {
                RCLCPP_INFO(m_node_handle->get_logger(), "Trajectory has started but goal was cancelled : stopping all movement.");
                stop_all_movement();
            }
            // We should not have received that
            else
            {
                RCLCPP_DEBUG(m_node_handle->get_logger(), "Notification mismatch : received ACTION_START but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;

        case Kinova::Api::Base::ActionEvent::ACTION_FEEDBACK:
        {
            // debug trace to indicate we've reached waypoints
            for (unsigned int i = 0; i < notif.trajectory_info_size(); i++)
            {
                auto info = notif.trajectory_info(i);
                if (info.trajectory_info_type() == Kinova::Api::Base::TrajectoryInfoType::WAYPOINT_REACHED)
                {
                    RCLCPP_DEBUG(m_node_handle->get_logger(), "Waypoint %d reached", info.waypoint_index());
                }
            }
            break;
        }   

        // The action was started in the arm, but it aborted
        case Kinova::Api::Base::ActionEvent::ACTION_ABORT:
            // The goal is still active, but we received a ABORT before starting, or during execution
            if (kortex_callback.goal_handle->is_active() &&
                (m_server_state == ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS ||
                 m_server_state == ActionServerState::TRAJECTORY_EXECUTION_PENDING))
            {
                RCLCPP_ERROR(m_node_handle->get_logger(), "Trajectory has been aborted.");

                result->error_code = result->PATH_TOLERANCE_VIOLATED;
                oss << "Trajectory execution failed in the arm with sub error code " << notif.abort_details() << std::endl;
                if (notif.abort_details() == Kinova::Api::SubErrorCodes::CONTROL_WRONG_STARTING_POINT)
                {
                    oss << "The starting point for the trajectory did not match the actual commanded joint angles." << std::endl;
                }
                else if (notif.abort_details() == Kinova::Api::SubErrorCodes::CONTROL_MANUAL_STOP)
                {
                    oss << "The speed while executing the trajectory was too damn high and caused the robot to stop." << std::endl;
                }
                result->error_string = oss.str();
                kortex_callback.goal_handle->abort(result);

                RCLCPP_ERROR(m_node_handle->get_logger(), "%s", oss.str().c_str());
                set_server_state(ActionServerState::IDLE);
            }
            // The goal was cancelled and we received a ACTION_ABORT : this means the trajectory was cancelled successfully in the arm
            else if  (kortex_callback.goal_handle->is_canceling() &&
                     (m_server_state == ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS ||
                      m_server_state == ActionServerState::TRAJECTORY_EXECUTION_PENDING))
            {
                RCLCPP_INFO(m_node_handle->get_logger(), "Trajectory has been cancelled successfully in the arm.");
                kortex_callback.goal_handle->canceled(result);
                set_server_state(ActionServerState::IDLE);
            }
            // We should not have received that
            else
            {
                RCLCPP_DEBUG(m_node_handle->get_logger(), "Notification mismatch : received ACTION_ABORT but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;

        // The trajectory just ended
        case Kinova::Api::Base::ActionEvent::ACTION_END:
        {
            // The trajectory was ongoing
            if ((m_server_state == ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS))
            {
                RCLCPP_INFO(m_node_handle->get_logger(), "Trajectory has finished in the arm.");
                m_trajectory_end_time = std::chrono::system_clock::now();
                // When going at full speed we have to stabilize a bit before checking the final position
                if (m_use_hard_limits)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                bool is_tolerance_respected = is_goal_tolerance_respected(kortex_callback.goal_handle->get_goal(), true, true);
                if (is_tolerance_respected)
                {
                    result->error_code = result->SUCCESSFUL;
                    RCLCPP_INFO(m_node_handle->get_logger(), "Trajectory execution succeeded.");
                    kortex_callback.goal_handle->succeed(result);
                }
                else
                {
                    result->error_code = result->PATH_TOLERANCE_VIOLATED;
                    oss << "After validation, trajectory execution failed in the arm with sub error code " << Kinova::Api::SubErrorCodes_Name(notif.abort_details());
                    result->error_string = oss.str();

                    RCLCPP_ERROR(m_node_handle->get_logger(), "%s", oss.str().c_str());
                    kortex_callback.goal_handle->abort(result);
                }
                set_server_state(ActionServerState::IDLE);
            }
            // We should not have received that
            else
            {
                RCLCPP_DEBUG(m_node_handle->get_logger(), "Notification mismatch : received ACTION_END but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;
        }

        case Kinova::Api::Base::ActionEvent::ACTION_PAUSE:
            RCLCPP_DEBUG(m_node_handle->get_logger(), "Action pause event was just received and this should never happen.");
            break;

        default:
            RCLCPP_DEBUG(m_node_handle->get_logger(), "Unknown action event was just received and this should never happen.");
            break;
        }
    }
    // Wrong action type. Rejecting the notification. Action server state unchanged.
    else
    {
        return;
    }

    oss.flush();
}

bool JointTrajectoryActionServer::is_goal_acceptable(const std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal)
{
    // // First check if goal is valid
    // if (!goal_handle.isValid())
    // {
    //     RCLCPP_ERROR(m_node_handle->get_logger(), "Invalid goal.");
    //     return false;
    // }

    // // Retrieve the goal
    // control_msgs::FollowJointTrajectoryGoalConstPtr goal = goal_handle.getGoal();

    // Goal does not command the right number of actuators
    if (goal->trajectory.joint_names.size() != m_joint_names.size())
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Goal commands %lu actuators, but arm has %lu.", goal->trajectory.joint_names.size(), m_joint_names.size());
        return false;
    }

    // Goal does not command the right actuators
    if (m_joint_names != goal->trajectory.joint_names)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "There is a mismatch between the goal's joint names and the action server's joint names.");
        RCLCPP_INFO(m_node_handle->get_logger(), "Action server joint names are :");
        for (auto j : m_joint_names)
        {
            std::cout << j << ", ";
        }
        std::cout << std::endl;
        RCLCPP_INFO(m_node_handle->get_logger(), "Goal joint names are :");
        for (auto j : goal->trajectory.joint_names)
        {
            std::cout << j << ", ";
        }
        std::cout << std::endl;
        return false;
    }
    
    return true;
}

bool JointTrajectoryActionServer::is_goal_tolerance_respected(const std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal, bool enable_prints, bool check_time_tolerance)
{
    // Get feedback from arm (retry if timeouts occur)
    bool is_goal_respected = true;
    Kinova::Api::BaseCyclic::Feedback feedback;
    bool got_feedback = true;
    while (!got_feedback)
    {
        try
        {
            feedback = m_base_cyclic->RefreshFeedback();
            got_feedback = true;
        }
        catch(const std::exception& e)
        {
        }
    }
    // auto goal = m_goal.getGoal();

    // Check the goal_time_tolerance for trajectory execution
    if (check_time_tolerance)
    {
        double actual_trajectory_duration = std::chrono::duration<double>(m_trajectory_end_time - m_trajectory_start_time).count();
        double desired_trajectory_duration = KortexMathUtil::duration_toSec(goal->trajectory.points.at(goal->trajectory.points.size()-1).time_from_start) + STARTING_POINT_ARBITRARY_DURATION;
        double time_tolerance = KortexMathUtil::duration_toSec(goal->goal_time_tolerance) == 0.0 ? m_default_goal_time_tolerance : KortexMathUtil::duration_toSec(goal->goal_time_tolerance);
        if (actual_trajectory_duration > desired_trajectory_duration + time_tolerance )
        {
            if (enable_prints)
                RCLCPP_ERROR(m_node_handle->get_logger(), "Goal duration tolerance was exceeded. Maximum desired duration was %f seconds and actual duration was %f", desired_trajectory_duration + time_tolerance, actual_trajectory_duration);
            return false;
        }
        else if (actual_trajectory_duration < desired_trajectory_duration - time_tolerance)
        {
            if (enable_prints)
                RCLCPP_ERROR(m_node_handle->get_logger(), "Goal duration threshold was not reached. Minimum desired duration was %f seconds and actual duration was %f", desired_trajectory_duration - time_tolerance, actual_trajectory_duration);
            return false;
        }
    }

    // If position tolerances were specified, use them.
    // If the goal->goal_tolerance vector is empty, fill it with default values
    std::vector<double> goal_tolerances;
    if (goal->goal_tolerance.empty())
    {
        RCLCPP_DEBUG(m_node_handle->get_logger(), "Goal did not specify tolerances, using default tolerance of %f radians for every joint.", m_default_goal_tolerance);
        for (int i = 0; i < m_joint_names.size(); i++)
        {
            goal_tolerances.push_back(m_default_goal_tolerance);
        }
    }
    else
    {
        for (auto tol : goal->goal_tolerance)
        {
            goal_tolerances.push_back(tol.position);
        }
    }

    // Check the joint tolerances on the goal's end position
    int current_index = 0;
    for (auto act: feedback.actuators())
    {
        double actual_position = act.position(); // in degrees
        double desired_position = KortexMathUtil::wrapDegreesFromZeroTo360(KortexMathUtil::toDeg(goal->trajectory.points.at(goal->trajectory.points.size()-1).positions[current_index]));
        double tolerance = 0.0;

        if (goal_tolerances[current_index] == -1.0)
        {
            current_index++;
            continue; // no tolerance set for this joint
        }
        else
        {
            tolerance = KortexMathUtil::toDeg(goal_tolerances[current_index]);
        }

        double error = KortexMathUtil::wrapDegreesFromZeroTo360(std::min(fabs(actual_position - desired_position), fabs(fabs(actual_position - desired_position) - 360.0)));
        if (error > tolerance)
        {
            is_goal_respected = false;
            if (enable_prints)
                RCLCPP_ERROR(m_node_handle->get_logger(), "The tolerance for joint %u was not met. Desired position is %f and actual position is %f", current_index + 1, desired_position, actual_position);
        }
        current_index++;
    }

    return is_goal_respected;
}

void JointTrajectoryActionServer::stop_all_movement()
{
    RCLCPP_INFO(m_node_handle->get_logger(), "Calling Stop on the robot.");
    try
    {
        m_base->Stop();
    }
    catch(const Kinova::Api::KBasicException& e)
    {
        RCLCPP_WARN(m_node_handle->get_logger(), "Stop failed : %s", e.what());
    }
}

void JointTrajectoryActionServer::set_server_state(ActionServerState s)
{
    std::lock_guard<std::mutex> guard(m_server_state_lock);
    ActionServerState old_state = m_server_state;
    m_server_state = s;
    // If we're going to IDLE, set back the soft limits
    if (s == ActionServerState::IDLE)
    {
        setAngularTrajectorySoftLimits(m_soft_limits);
    }
    RCLCPP_INFO(m_node_handle->get_logger(), "State changed from %s to %s\n", actionServerStateNames[int(old_state)], actionServerStateNames[int(s)]);
}

AngularTrajectorySoftLimits JointTrajectoryActionServer::getAngularTrajectorySoftLimits()
{
    const Kinova::Api::ControlConfig::ControlMode target_control_mode = Kinova::Api::ControlConfig::ANGULAR_TRAJECTORY;

    Kinova::Api::ControlConfig::ControlModeInformation info;
    info.set_control_mode(target_control_mode);
    auto soft_limits = m_control_config->GetKinematicSoftLimits(info);

    // Set Joint Speed limits to max
    Kinova::Api::ControlConfig::JointSpeedSoftLimits vel;
    vel.set_control_mode(target_control_mode);
    for (auto j : soft_limits.joint_speed_limits())
    {
        vel.add_joint_speed_soft_limits(j);
    }

    // Set Joint Acceleration limits to max
    Kinova::Api::ControlConfig::JointAccelerationSoftLimits acc;
    acc.set_control_mode(target_control_mode);
    for (auto j : soft_limits.joint_acceleration_limits())
    {
        acc.add_joint_acceleration_soft_limits(j);
    }

    return AngularTrajectorySoftLimits(vel, acc);
}

void JointTrajectoryActionServer::setAngularTrajectorySoftLimitsToMax()
{
    RCLCPP_DEBUG(m_node_handle->get_logger(), "Setting soft limits to hard");
    try
    {
        const Kinova::Api::ControlConfig::ControlMode target_control_mode = Kinova::Api::ControlConfig::ANGULAR_TRAJECTORY;
        
        // Set Joint Speed limits to max
        Kinova::Api::ControlConfig::JointSpeedSoftLimits jpsl;
        jpsl.set_control_mode(target_control_mode);
        for (auto j : m_hard_limits.joint_speed_limits())
        {
            jpsl.add_joint_speed_soft_limits(j);
        }
        m_control_config->SetJointSpeedSoftLimits(jpsl);

        // Set Joint Acceleration limits to max
        Kinova::Api::ControlConfig::JointAccelerationSoftLimits jasl;
        jasl.set_control_mode(target_control_mode);
        for (auto j : m_hard_limits.joint_acceleration_limits())
        {
            jasl.add_joint_acceleration_soft_limits(j);
        }
        m_control_config->SetJointAccelerationSoftLimits(jasl);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        RCLCPP_WARN(m_node_handle->get_logger(), "Kortex exception while setting the angular soft limits");
        RCLCPP_WARN(m_node_handle->get_logger(), "Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error description: %s\n", ex.what());
    }
}

void JointTrajectoryActionServer::setAngularTrajectorySoftLimits(const AngularTrajectorySoftLimits& limits)
{
    RCLCPP_DEBUG(m_node_handle->get_logger(), "Setting back soft limits");
    if (!limits.empty())
    {
        try
        {
            // Set Joint Speed limits
            m_control_config->SetJointSpeedSoftLimits(limits.m_vel);

            // Set Joint Acceleration limits to max
            m_control_config->SetJointAccelerationSoftLimits(limits.m_acc);
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            RCLCPP_WARN(m_node_handle->get_logger(), "Kortex exception while setting the angular soft limits");
            RCLCPP_WARN(m_node_handle->get_logger(), "Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
            RCLCPP_WARN(m_node_handle->get_logger(), "Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
            RCLCPP_WARN(m_node_handle->get_logger(), "Error description: %s\n", ex.what());
        }
    }
}
