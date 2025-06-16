#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "robot_msgs/delivery.h"
#include "robot_msgs/ui_show.h"
#include "robot_msgs/ui_get.h"

#include <vector>
#include <string>
#include <sstream>


// 根据实际消息类型和服务类型包含头文件

class TaskManagerNode {

private:

    // 用于存储待配送目标的列表
    std::vector<std::vector<int>> task_list_; 

    // 发布者
    ros::Publisher tracer_light_pub_;
    ros::Publisher speed_pub_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher ui_show_pub_;
    ros::Publisher speach_client_;
    ros::Publisher calling_client_;

    // 订阅者
    ros::Subscriber tracer_status_sub_;
    ros::Subscriber navigation_status_sub_;

    // 客户端
    ros::ServiceClient delivery_cmd_client_;
    ros::ServiceClient delivery_door_open_client_;

    robot_msgs::delivery delivery_req;

    bool push_out(int num);
    bool door_open(int num);

    // 服务端
    ros::ServiceServer ui_get_server_;

    // // 动作客户端
    // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> navigation_to_pose_ac_;

    // 客户端调用函数
    bool deliveryCmd(robot_msgs::delivery& req);
    bool deliveryDoorOpen(robot_msgs::delivery& req);

    // 回调函数声明
    void tracerStatusCallback(const std_msgs::String::ConstPtr& msg);
    void navigationStatusCallback(const std_msgs::String::ConstPtr& msg);

    bool uiGetCallback(robot_msgs::ui_get::Request& req, robot_msgs::ui_get::Response& res);


public:

TaskManagerNode(ros::NodeHandle& nh);
void workflow();
    
};