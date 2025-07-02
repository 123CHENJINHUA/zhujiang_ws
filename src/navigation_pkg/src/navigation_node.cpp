#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "robot_msgs/deliveryAction.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <map>

class DeliveryActionServer
{
public:
    DeliveryActionServer(ros::NodeHandle& nh)
        : as_(nh, "delivery_action", boost::bind(&DeliveryActionServer::executeCB, this, _1), false),
          move_base_client_("move_base", true) // Initialize move_base action client
    {
        // Start the delivery action server
        as_.start();
        ROS_INFO("Delivery Action Server started.");

        // Initialize room-to-position mapping
        room_positions_ = {
            {1, {5.0, -1.9}},
            {2, {11.7, -1.9}}, 
            {3, {15.0, -1.9}}, 
            // Add more room mappings as needed
        };
    }

    void executeCB(const robot_msgs::deliveryGoalConstPtr& goal)
    {
        ROS_INFO("Received delivery goal: %dB%dU%dF%dR", goal->building, goal->unit, goal->floor, goal->room);

        // Find the target position for the room
        auto it = room_positions_.find(goal->room);
        if (it == room_positions_.end())
        {
            ROS_ERROR("Room %d not found in mapping!", goal->room);
            robot_msgs::deliveryResult result;
            result.success = false;
            result.info = "Room not found";
            as_.setAborted(result);
            return;
        }

        const auto& target_position = it->second;

        // Wait for move_base action server to be available
        if (!move_base_client_.waitForServer(ros::Duration(5.0)))
        {
            ROS_ERROR("move_base action server not available!");
            robot_msgs::deliveryResult result;
            result.success = false;
            result.info = "Navigation server unavailable";
            as_.setAborted(result);
            return;
        }

        // Create and send navigation goal
        move_base_msgs::MoveBaseGoal nav_goal;
        nav_goal.target_pose.header.frame_id = "map_2d";
        nav_goal.target_pose.header.stamp = ros::Time::now();
        nav_goal.target_pose.pose.position.x = target_position.first;
        nav_goal.target_pose.pose.position.y = target_position.second;
        nav_goal.target_pose.pose.position.z = 0.99; // Default z position
        nav_goal.target_pose.pose.orientation.w = 0.04; // Default orientation

        ROS_INFO("Sending navigation goal to room %d at position (%.2f, %.2f)", goal->room, target_position.first, target_position.second);
        move_base_client_.sendGoal(nav_goal);

        // Publish constant feedback
        robot_msgs::deliveryFeedback feedback;
        feedback.status = 100; // Constant feedback value
        as_.publishFeedback(feedback);

        // Monitor navigation progress with infinite waiting
        while (true)
        {
            actionlib::SimpleClientGoalState state = move_base_client_.getState();

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Navigation to room %d succeeded.", goal->room);

                // Set result
                robot_msgs::deliveryResult result;
                result.success = true;
                result.info = "Finish";
                as_.setSucceeded(result);
                ROS_INFO("Delivery succeeded.");
                return;
            }
            else if (state == actionlib::SimpleClientGoalState::ABORTED || state == actionlib::SimpleClientGoalState::PREEMPTED)
            {
                ROS_ERROR("Navigation to room %d failed. State: %s", goal->room, state.toString().c_str());

                robot_msgs::deliveryResult result;
                result.success = false;
                result.info = "Navigation failed";
                as_.setAborted(result);
                return;
            }
            else if (state == actionlib::SimpleClientGoalState::ACTIVE)
            {
                ROS_INFO("Navigation to room %d is still active. Waiting for completion...", goal->room);
            }

            // Sleep for a short duration to avoid busy-waiting
            ros::Duration(1.0).sleep();
        }
    }

private:
    actionlib::SimpleActionServer<robot_msgs::deliveryAction> as_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    std::map<int, std::pair<double, double>> room_positions_; // Room-to-position mapping
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delivery_as_node");
    ros::NodeHandle nh;
    DeliveryActionServer server(nh);
    ros::spin();
    return 0;
}