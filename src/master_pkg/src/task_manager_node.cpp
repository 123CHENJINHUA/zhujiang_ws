#include "master_pkg/task_manager_node.h"

TaskManagerNode::TaskManagerNode(ros::NodeHandle& nh)
    // : navigation_to_pose_ac_("/navigation_to_pose", true) // 动作客户端初始化
{
    // 发布者
    tracer_light_pub_ = nh.advertise<std_msgs::String>("/tracer_light_control", 10);
    speed_pub_ = nh.advertise<geometry_msgs::Twist>("/speed", 10);
    emergency_stop_pub_ = nh.advertise<std_msgs::Bool>("/emergency_stop", 10);
    ui_show_pub_ = nh.advertise<std_msgs::String>("/UI_show", 10);
    speach_client_ = nh.advertise<std_msgs::String>("/speach", 10);
    calling_client_ = nh.advertise<std_msgs::String>("/calling", 10);

    // 订阅者
    tracer_status_sub_ = nh.subscribe("/tracer_status", 10, &TaskManagerNode::tracerStatusCallback, this);
    navigation_status_sub_ = nh.subscribe("/navigation_status", 10, &TaskManagerNode::navigationStatusCallback, this);

    // 服务客户端
    delivery_cmd_client_ = nh.serviceClient<robot_msgs::delivery>("/delivery_cmd");
    delivery_door_open_client_ = nh.serviceClient<robot_msgs::delivery>("/delivery_door_open");


    // // 服务端
    // ui_get_server_ = nh.advertiseService("/UI_get", &TaskManagerNode::uiGetCallback, this);

    ROS_INFO("TaskManagerNode initialized.");
}

// 话题订阅实现
void TaskManagerNode::tracerStatusCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received tracer status: %s", msg->data.c_str());
}

void TaskManagerNode::navigationStatusCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received navigation status: %s", msg->data.c_str());
}

// 客户端调用实现
bool TaskManagerNode::deliveryCmd(robot_msgs::delivery& req) {
    if (delivery_cmd_client_.call(req)) {
        if (req.response.status_msgs == true) {
            ROS_INFO("Delivery cmd: %s : success", req.request.delivery_msgs.c_str());
            return true;
        }
        ROS_ERROR("Delivery cmd: %s : failed", req.request.delivery_msgs.c_str());
        return false;
    } else {
        ROS_ERROR("Failed to execute delivery command: %s", req.request.delivery_msgs.c_str());
        return false;
    }
}

bool TaskManagerNode::deliveryDoorOpen(robot_msgs::delivery& req) {
    if (delivery_door_open_client_.call(req)) {
        if (req.response.status_msgs == true) {
            ROS_INFO("Delivery door: %s : success", req.request.delivery_msgs.c_str());
            return true;
        }
        ROS_ERROR("Delivery door: %s : failed", req.request.delivery_msgs.c_str());
        return false;
    } else {
        ROS_ERROR("Failed to execute delivery door open command: %s", req.request.delivery_msgs.c_str());
        return false;
    }
}

bool TaskManagerNode::push_out(int num) {
    // 发布推送消息
    delivery_req.request.delivery_msgs = "push" + std::to_string(num);
    return deliveryCmd(delivery_req);;
}

bool TaskManagerNode::door_open(int num) {
    // 发布开门消息
    delivery_req.request.delivery_msgs = "motor " + std::to_string(num);
    return deliveryDoorOpen(delivery_req);
}

// bool TaskManagerNode::uiGetCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
//     res.success = true;
//     res.message = "UI_get service called";
//     return true;
// }

void TaskManagerNode::workflow() {
    // 这里可以实现工作流逻辑
    // 例如，调用服务、发布消息等
    ROS_INFO("Workflow started.");
    // door_open(0);

    // 等待订阅者
    while (ui_show_pub_.getNumSubscribers() == 0 ||
           speach_client_.getNumSubscribers() == 0 ||
           calling_client_.getNumSubscribers() == 0) {
        ROS_INFO("Waiting for UI_show, speach, and calling subscribers...");
        ros::Duration(0.1).sleep();
    }

    std_msgs::String ui_msg;
    ui_msg.data = "ui_start ------------------";
    ui_show_pub_.publish(ui_msg);
    std_msgs::String speach_msg;
    speach_msg.data = "2";
    speach_client_.publish(speach_msg);
    std_msgs::String calling_msg;
    calling_msg.data = "1楼 1单元 1号";
    calling_client_.publish(calling_msg);




}

// main函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "task_manager_node");
    ros::NodeHandle nh;
    TaskManagerNode node(nh);
    // 等待所有服务准备就绪
    // ros::service::waitForService("/delivery_cmd");
    // ros::service::waitForService("/delivery_door_open");
    // ros::service::waitForService("/UI_get");
    // 等待动作客户端连接
    // node.navigation_to_pose_ac_.waitForServer();
    ROS_INFO("All services are ready and action client is connected.");
    // 启动工作流
    node.workflow();
    // 启动ROS事件循环
    ros::spin();
    return 0;
}