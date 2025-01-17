/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2021 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */

#include "kortex_driver/non-generated/cartesian_trajectory_action_server.h"
#include <sstream>
#include <fstream>

CartesianTrajectoryActionServer::CartesianTrajectoryActionServer(const std::string& server_name, rclcpp::Node::SharedPtr nh, Kinova::Api::Base::BaseClient* base, Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic):
    m_server_name(server_name),
    m_node_handle(nh),
    m_base(base),
    m_server_state(ActionServerState::INITIALIZING),
    m_currentActionID(0)
{
    // Get the ROS params
    if (!m_node_handle->get_parameter("~prefix", m_prefix))
    {
        std::string error_string = "Prefix name was not specified in the launch file, shutting down the node...";
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }
    
    // Subscribe to the arm's Action Notifications
    m_sub_action_notif_handle = m_base->OnNotificationActionTopic(std::bind(&CartesianTrajectoryActionServer::action_notif_callback, this, std::placeholders::_1), Kinova::Api::Common::NotificationOptions());

    // Ready to receive goal
    m_server = rclcpp_action::create_server<FollowCartesianTrajectoryAction>(
        m_node_handle, m_server_name,
        std::bind(&CartesianTrajectoryActionServer::ros_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CartesianTrajectoryActionServer::ros_cancel_callback, this, std::placeholders::_1),
        std::bind(&CartesianTrajectoryActionServer::ros_accepted_callback, this, std::placeholders::_1));
    
    set_server_state(ActionServerState::IDLE);
}

CartesianTrajectoryActionServer::~CartesianTrajectoryActionServer()
{
    const auto result_msg = std::make_shared<FollowCartesianTrajectoryAction::Result>();

    try
    {
        m_base->Unsubscribe(m_sub_action_notif_handle);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Kortex exception while unsubscribing to action notification.");
        RCLCPP_ERROR(m_node_handle->get_logger(), "Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        RCLCPP_ERROR(m_node_handle->get_logger(), "Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        RCLCPP_ERROR(m_node_handle->get_logger(), "Error description: %s\n", ex.what());
        if (kortex_callback.goal_handle)
            kortex_callback.goal_handle->abort(result_msg);
    }
    catch (std::runtime_error& ex_runtime)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Runtime exception detected while unsubscribing to action notification.");
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", ex_runtime.what());
        if (kortex_callback.goal_handle)
            kortex_callback.goal_handle->abort(result_msg);
    }
    catch (std::future_error& ex_future)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Future exception detected while unsubscribing to action notification.");
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", ex_future.what());
        if (kortex_callback.goal_handle)
            kortex_callback.goal_handle->abort(result_msg);
    }
}

rclcpp_action::GoalResponse CartesianTrajectoryActionServer::ros_goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowCartesianTrajectoryAction::Goal> goal)
{
    (void)uuid;

    RCLCPP_INFO(m_node_handle->get_logger(), "New Cartesian goal received.");
    if (!is_goal_acceptable(goal))
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Cartesian Trajectory Goal is rejected.");
        // new_goal_handle->canceled(result_msg);
        return rclcpp_action::GoalResponse::REJECT;
    }
    // Accept the goal
    RCLCPP_INFO(m_node_handle->get_logger(), "Cartesian Trajectory Goal is accepted.");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void CartesianTrajectoryActionServer::ros_accepted_callback(const std::shared_ptr<GoalHandle_FollowCartesianTrajectoryAction> new_goal_handle)
{
    const auto result_msg = std::make_shared<FollowCartesianTrajectoryAction::Result>();

    // Accept the goal
    kortex_callback.goal_handle = new_goal_handle;

    if (m_server_state != ActionServerState::IDLE)
    {
        RCLCPP_WARN(m_node_handle->get_logger(), "There is already an active cartesian goal. It is being cancelled.");
        // We have to call Stop after having received the ACTION_START notification from the arm
        stop_all_movement();
    }

    // // Accept the goal
    // RCLCPP_INFO(m_node_handle->get_logger(), "Cartesian Trajectory Goal is accepted.");
    // m_goal = new_goal_handle;
    // // m_goal->setAccepted();

    auto action = Kinova::Api::Base::Action();
    action.set_name("Cartesian waypoint");
    action.set_application_data("");

    auto proto_trajectory = action.mutable_execute_waypoint_list();

    proto_trajectory->set_duration(KortexMathUtil::duration_toSec(new_goal_handle->get_goal()->goal_time_tolerance));
    proto_trajectory->set_use_optimal_blending(new_goal_handle->get_goal()->use_optimal_blending);

    for (unsigned int i = 0; i < new_goal_handle->get_goal()->trajectory.size(); i++)
    {
        const auto traj_point = new_goal_handle->get_goal()->trajectory.at(i);
        
        Kinova::Api::Base::Waypoint* proto_waypoint = proto_trajectory->add_waypoints();
        proto_waypoint->set_name("waypoint_" + std::to_string(i));
        
        auto cartesian_waypoint = proto_waypoint->mutable_cartesian_waypoint();
        
        auto tempPose = cartesian_waypoint->mutable_pose();
        tempPose->set_x(traj_point.pose.x);
        tempPose->set_y(traj_point.pose.y);
        tempPose->set_z(traj_point.pose.z);

        tempPose->set_theta_x(KortexMathUtil::toDeg(traj_point.pose.theta_x));
        tempPose->set_theta_y(KortexMathUtil::toDeg(traj_point.pose.theta_y));
        tempPose->set_theta_z(KortexMathUtil::toDeg(traj_point.pose.theta_z));
        
        cartesian_waypoint->set_reference_frame((Kinova::Api::Common::CartesianReferenceFrame)traj_point.reference_frame);
        cartesian_waypoint->set_maximum_angular_velocity(traj_point.maximum_angular_velocity);
        cartesian_waypoint->set_maximum_linear_velocity(traj_point.maximum_linear_velocity);
        cartesian_waypoint->set_blending_radius(traj_point.blending_radius);
    }

    try
    {
        // Validate the waypoints and reject the goal if they fail validation
        auto report = m_base->ValidateWaypointList(*proto_trajectory);
        if (report.trajectory_error_report().trajectory_error_elements_size() > 0)
        {
            RCLCPP_ERROR(m_node_handle->get_logger(), "Cartesian Trajectory failed validation in the arm.");
            
            // Go through report and print errors
            for (unsigned int i = 0; i < report.trajectory_error_report().trajectory_error_elements_size(); i++)
            {
                RCLCPP_ERROR(m_node_handle->get_logger(), "Error %i : %s", i+1, report.trajectory_error_report().trajectory_error_elements(i).message().c_str());
            }
            // new_goal_handle->canceled(result_msg);
            new_goal_handle->abort(result_msg);
            return;
        }

        // Make sure to clear the faults before moving the robot
        m_base->ClearFaults();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // Send the trajectory to the robot
        m_base->ExecuteAction(action);
        set_server_state(ActionServerState::PRE_PROCESSING_PENDING);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Kortex exception while sending the trajectory");
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
        RCLCPP_ERROR(m_node_handle->get_logger(), "Future exception detected while sending the trajectory");
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", ex_future.what());
        new_goal_handle->abort(result_msg);
    }
}

// Called in a separate thread when a preempt request comes in from the Action Client
rclcpp_action::CancelResponse CartesianTrajectoryActionServer::ros_cancel_callback(const std::shared_ptr<GoalHandle_FollowCartesianTrajectoryAction> goal_handle)
{
    (void)goal_handle;
    if (m_server_state == ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS)
    {
        stop_all_movement();
    }

    return rclcpp_action::CancelResponse::ACCEPT;
}

// Called in a separate thread when a notification comes in
void CartesianTrajectoryActionServer::action_notif_callback(Kinova::Api::Base::ActionNotification notif)
{
    if (kortex_callback.goal_handle == nullptr)
    {
        stop_all_movement();
        RCLCPP_ERROR(m_node_handle->get_logger(), "Use of uninitialized pointer \"goal_handle\"");
        return;
    }

    if(m_server_state == ActionServerState::IDLE)
    {
        return;
    }
    Kinova::Api::Base::ActionEvent event = notif.action_event();
    Kinova::Api::Base::ActionHandle handle = notif.handle();
    Kinova::Api::Base::ActionType type = handle.action_type();
    RCLCPP_DEBUG(m_node_handle->get_logger(), "Action notification received of type %s", Kinova::Api::Base::ActionEvent_Name(event).c_str());
    
    std::lock_guard<std::mutex> guard(m_action_notification_thread_lock);

    auto result = std::make_shared<FollowCartesianTrajectoryAction::Result>();
    std::ostringstream oss;
    
    if (type == Kinova::Api::Base::ActionType::EXECUTE_WAYPOINT_LIST)
    {
        if(event == Kinova::Api::Base::ActionEvent::ACTION_PREPROCESS_START)
        {
            m_currentActionID = notif.handle().identifier();

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
        }
        else if(m_currentActionID == notif.handle().identifier())
        {
            switch (event)
            {
            
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
            // TODO This could be much lighter since ValidateWaypoints already does that
            case Kinova::Api::Base::ActionEvent::ACTION_PREPROCESS_ABORT:
                // It was ongoing and now it ended (and failed)
                if ((m_server_state == ActionServerState::PRE_PROCESSING_IN_PROGRESS))
                {
                    RCLCPP_DEBUG(m_node_handle->get_logger(), "Preprocessing has finished in the arm and goal has been rejected. Fetching the error report from the arm...");

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
                        RCLCPP_DEBUG(m_node_handle->get_logger(), "Cartesian waypoint %d reached", info.waypoint_index());
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
                        oss << "The starting point for the trajectory did not match the actual commanded cartesian pose." << std::endl;
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
                    result->error_code = result->SUCCESSFUL;
                    RCLCPP_INFO(m_node_handle->get_logger(), "Trajectory execution succeeded.");
                    kortex_callback.goal_handle->succeed(result);
                    
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
                RCLCPP_WARN(m_node_handle->get_logger(), "Action pause event was just received and this should never happen.");
                break;

            default:
                RCLCPP_WARN(m_node_handle->get_logger(), "Unknown action event was just received and this should never happen.");
                break;
            }
        }
        
        
    }
    // Wrong action type. Rejecting the notification. Action server state unchanged.
    else
    {
        return;
    }

    oss.flush();
}

bool CartesianTrajectoryActionServer::is_goal_acceptable(const std::shared_ptr<const FollowCartesianTrajectoryAction::Goal> goal)
{
    // // First check if goal is valid
    // if (!goal_handle->is_active())
    // {
    //     RCLCPP_ERROR(m_node_handle->get_logger(), "Invalid Cartesian goal.");
    //     return false;
    // }

    // Check if the trajectory contains at least 1 waypoint.
    if (goal->trajectory.size() == 0)
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Empty Cartesian trajectory list.");
        return false;
    }

    return true;
}

void CartesianTrajectoryActionServer::stop_all_movement()
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

void CartesianTrajectoryActionServer::set_server_state(ActionServerState s)
{
    std::lock_guard<std::mutex> guard(m_server_state_lock);
    ActionServerState old_state = m_server_state;
    m_server_state = s;
    RCLCPP_INFO(m_node_handle->get_logger(), "State changed from %s to %s\n", actionServerStateNames[int(old_state)], actionServerStateNames[int(s)]);
}