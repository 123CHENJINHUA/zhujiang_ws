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


    // 服务端
    ui_get_server_ = nh.advertiseService("/UI_get", &TaskManagerNode::uiGetCallback, this);

    ROS_INFO("TaskManagerNode initialized.");
}



// 机器人状态更新函数
void TaskManagerNode::robot_status_update() {

    battery_ = 80; // 假设电池电量为80%
    speed_ = 5; // 假设速度为5 m/s
    odometry_ = 100.0; // 假设里程计为100.0 m
    working_time_ = 3600.0; // 假设工作时间为3600秒
    network_ = "Good"; // 假设网络状态良好
    task_status_ = 1; // 假设任务状态为1（进行中）

    // 用 current_task_ 和 rest_task_ 更新显示内容
    if (!current_task_.empty()) {
        current_task_show_ = std::to_string(current_task_[0]) + "栋" +
                             std::to_string(current_task_[1]) + "单元" +
                             std::to_string(current_task_[2]) + "层" +
                             std::to_string(current_task_[3]) + "号";
    } else {
        current_task_show_.clear();
    }

    rest_task_show_.clear();
    for (const auto& t : rest_task_) {
        std::string task_str = std::to_string(t[0]) + "栋" +
                               std::to_string(t[1]) + "单元" +
                               std::to_string(t[2]) + "层" +
                               std::to_string(t[3]) + "号";
        rest_task_show_.push_back(task_str);
    }
}

// 话题订阅实现
void TaskManagerNode::tracerStatusCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received tracer status: %s", msg->data.c_str());
}

void TaskManagerNode::navigationStatusCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received navigation status: %s", msg->data.c_str());
}

// 发布者设置
void TaskManagerNode::pub_setup() {
    // 等待订阅者
    ROS_INFO("Waiting for UI_show, speach, and calling subscribers...");
    while (ui_show_pub_.getNumSubscribers() == 0 ||
           speach_client_.getNumSubscribers() == 0 ||
           calling_client_.getNumSubscribers() == 0) {
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("UI_show, speach, and calling subscribers connected!");

    // 启动独立线程循环发布机器人状态
    std::thread pub_thread(&TaskManagerNode::publishUiShowLoop, this);
    pub_thread.detach();
}

//机器人状态发布实现
void TaskManagerNode::publishUiShowLoop() {
    ros::Rate rate(1); // 1Hz
    while (ros::ok()) {

        // 更新机器人状态
        robot_status_update();
        // 发布机器人状态到UI
        robot_msgs::ui_show ui_msg;
        ui_msg.battery =  battery_;
        ui_msg.odometry = odometry_;
        ui_msg.speed = speed_;
        ui_msg.working_time = working_time_;
        ui_msg.network = network_;
        ui_msg.task_status = task_status_;
        ui_msg.current_task = current_task_show_;
        ui_msg.rest_task = rest_task_show_;
        ui_show_pub_.publish(ui_msg);
        rate.sleep();
    }
}

// 客户端设置
void TaskManagerNode::client_setup() {
    // 等待所有服务准备就绪
    ros::service::waitForService("/delivery_cmd");
    ROS_INFO("All services are ready and action client is connected.");
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

bool TaskManagerNode::push_out(int num) {
    delivery_req.request.delivery_msgs = "push " + std::to_string(num);
    return deliveryCmd(delivery_req);
}

bool TaskManagerNode::door_open(int num) {
    delivery_req.request.delivery_msgs = "door " + std::to_string(num);
    return deliveryCmd(delivery_req); 
}

bool TaskManagerNode::door_ir_control(const std::string& status) {
    delivery_req.request.delivery_msgs = "ir " + status;
    return deliveryCmd(delivery_req);
}

// ui指令服务端回调实现
bool TaskManagerNode::uiGetCallback(robot_msgs::ui_get::Request& req, robot_msgs::ui_get::Response& res) {
    // 清空原有任务列表
    std::lock_guard<std::mutex> lock(task_list_mutex_); // 加锁
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

    task_cv_.notify_one(); // 唤醒分配线程
    res.received = true;
    ROS_INFO("Task list updated, size: %lu", task_list_.size());
    return true;
}

// 任务分配设置
void TaskManagerNode::taskAssign_setup() {
    // 启动任务分配线程
    std::thread task_thread(&TaskManagerNode::taskAssignLoop, this);
    task_thread.detach();
}

// 任务分配函数
void TaskManagerNode::assignTask(const std::vector<std::vector<int>>& tasks) {
    if (!tasks.empty()) {
        current_task_ = tasks.front();
        rest_task_.clear();
        for (size_t i = 1; i < tasks.size(); ++i) {
            rest_task_.push_back(tasks[i]);
        }
    } else {
        current_task_.clear();
        rest_task_.clear();
    }
}

void TaskManagerNode::taskAssignLoop() {
    std::unique_lock<std::mutex> lock(task_list_mutex_);
    ROS_INFO("start delivery---.");
    while (ros::ok()) { 
        // 等待有任务或被唤醒
        task_cv_.wait(lock, [this]{ return !task_list_.empty() || !ros::ok(); });

        while (!task_list_.empty()) {

            ros::Duration(1.0).sleep(); // 处理间隔
            door_ir_control("on");

            //语音提示
            std_msgs::String speach_msg;
            speach_msg.data = "2";
            speach_client_.publish(speach_msg);
            //打电话
            std_msgs::String calling_msg;
            calling_msg.data = "1楼 1单元 1号";
            calling_client_.publish(calling_msg);

            door_open(4);

            // 发布任务分配信息
            assignTask(task_list_);
            task_list_.erase(task_list_.begin());
            lock.unlock();
            ros::Duration(5.0).sleep(); // 处理间隔
            lock.lock();

            door_ir_control("off");
        }
    }
}


// main函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "task_manager_node");
    ros::NodeHandle nh;
    TaskManagerNode node(nh);

    // 发布者设置
    node.pub_setup();
    // 客户端设置
    node.client_setup();

    // 等待动作客户端连接
    // node.navigation_to_pose_ac_.waitForServer();
    // 启动工作流
    node.taskAssign_setup();
    // 启动ROS事件循环
    ros::spin();
    return 0;
}