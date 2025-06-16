#include "master_pkg/task_manager_node.h"

TaskManagerNode::TaskManagerNode(ros::NodeHandle& nh)
    // : navigation_to_pose_ac_("/navigation_to_pose", true) // 动作客户端初始化
{
    // 发布者
    tracer_light_pub_ = nh.advertise<std_msgs::String>("/tracer_light_control", 10);
    speed_pub_ = nh.advertise<geometry_msgs::Twist>("/speed", 10);
    emergency_stop_pub_ = nh.advertise<std_msgs::Bool>("/emergency_stop", 10);
    ui_show_pub_ = nh.advertise<robot_msgs::ui_show>("/UI_show", 10);
    speach_client_ = nh.advertise<std_msgs::String>("/speach", 10);
    calling_client_ = nh.advertise<std_msgs::String>("/calling", 10);

    // 订阅者
    tracer_status_sub_ = nh.subscribe("/tracer_status", 10, &TaskManagerNode::tracerStatusCallback, this);
    navigation_status_sub_ = nh.subscribe("/navigation_status", 10, &TaskManagerNode::navigationStatusCallback, this);

    // 服务客户端
    delivery_cmd_client_ = nh.serviceClient<robot_msgs::delivery>("/delivery_cmd");
    delivery_door_open_client_ = nh.serviceClient<robot_msgs::delivery>("/delivery_door_open");


    // 服务端
    ui_get_server_ = nh.advertiseService("/UI_get", &TaskManagerNode::uiGetCallback, this);

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

// 服务端回调实现
bool TaskManagerNode::uiGetCallback(robot_msgs::ui_get::Request& req, robot_msgs::ui_get::Response& res) {
    // 清空原有任务列表
    task_list_.clear();

    // 解析请求字符串
    std::stringstream ss_groups(req.delivery_list.c_str());
    std::string group;
    while (std::getline(ss_groups, group, ';')) {
        std::vector<int> task;
        std::stringstream ss_values(group);
        std::string value;
        while (std::getline(ss_values, value, ',')) {
            try {
                task.push_back(std::stoi(value));
            } catch (...) {
                // 解析失败，返回错误
                res.received = false;
                return true;
            }
        }
        if (task.size() == 4) {
            task_list_.push_back(task);
        } else {
            res.received = false;
            return true;
        }
    }

    res.received = true;
    ROS_INFO("Task list updated, size: %lu", task_list_.size());
    return true;
}

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

    robot_msgs::ui_show ui_msg;
    ui_msg.battery = 100; // 假设电池电量为100%
    ui_msg.odometry = 0.0; // 假设里程计为0
    ui_msg.speed = 0.0; // 假设速度为0
    ui_msg.working_time = 0.0; // 假设工作时间为0
    ui_msg.network = "Good"; // 假设网络状态为 Good
    ui_msg.task_status = 0; // 假设任务状态为0
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

    // 等待动作客户端连接
    // node.navigation_to_pose_ac_.waitForServer();
    ROS_INFO("All services are ready and action client is connected.");
    // 启动工作流
    node.workflow();
    // 启动ROS事件循环
    ros::spin();
    return 0;
}