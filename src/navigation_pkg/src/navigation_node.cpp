#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "robot_msgs/deliveryAction.h"
#include <thread>
#include <atomic>

class DeliveryActionServer
{
public:
    DeliveryActionServer(ros::NodeHandle& nh)
        : as_(nh, "delivery_action", boost::bind(&DeliveryActionServer::executeCB, this, _1), false)
    {
        as_.start();
        ROS_INFO("Delivery Action Server started.");
    }

    void executeCB(const robot_msgs::deliveryGoalConstPtr& goal)
    {
        ROS_INFO("Received delivery goal: %dB%dU%dF%dR", goal->building, goal->unit, goal->floor, goal->room);

        // 启动独立线程发送feedback
        std::atomic<bool> done(false);
        std::thread feedback_thread([this, &done]() {
            for (int i = 0; i <= 100 && !done; ++i) {
                robot_msgs::deliveryFeedback feedback;
                feedback.status = i;
                as_.publishFeedback(feedback);
                ros::Duration(0.05).sleep(); // 5s/100 = 0.05s
            }
        });

        ros::Duration(5.0).sleep(); // 模拟配送耗时
        done = true;
        if (feedback_thread.joinable()) feedback_thread.join();

        // 结果
        robot_msgs::deliveryResult result;
        result.success = true;
        result.info = "配送完成";
        as_.setSucceeded(result);
        ROS_INFO("Delivery succeeded.");
    }

private:
    actionlib::SimpleActionServer<robot_msgs::deliveryAction> as_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delivery_as_node");
    ros::NodeHandle nh;
    DeliveryActionServer server(nh);
    ros::spin();
    return 0;
}