#include <ros/ros.h>
#include "robot_msgs/delivery.h"
#include "master_pkg/tcp_server.h"

TcpServer* g_tcp_server = nullptr;

bool deliveryCmdCallback(robot_msgs::delivery::Request& req, robot_msgs::delivery::Response& res) {
    if (g_tcp_server) {
        g_tcp_server->sendMessage(req.delivery_msgs);
        std::string reply = g_tcp_server->receiveMessage(g_tcp_server->timeout_delivery_cmd);
        if (reply.empty()) {
            ROS_ERROR("Timeout or no response for: %s", req.delivery_msgs.c_str());
            res.status_msgs = false;
            return true;
        }
        ROS_INFO("MCU reply: %s", reply.c_str());
        res.status_msgs = true;
        return true;
    }
    ROS_ERROR("tcp server not available");
    return false;
}

bool deliveryDoorOpenCallback(robot_msgs::delivery::Request& req, robot_msgs::delivery::Response& res) {
    if (g_tcp_server) {
        g_tcp_server->sendMessage(req.delivery_msgs);
        std::string reply = g_tcp_server->receiveMessage(g_tcp_server->timeout_door_open);
        if (reply.empty()) {
            ROS_ERROR("Timeout or no response for: %s", req.delivery_msgs.c_str());
            res.status_msgs = false;
            return true;
        }
        ROS_INFO("MCU reply: %s", reply.c_str());
        res.status_msgs = true;
        return true;
    }
    ROS_ERROR("tcp server not available");
    return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "delivery_node");
    ros::NodeHandle nh;

    int server_port = 5001;
    std::string allowed_ip = "192.168.0.111";

    TcpServer tcp_server(server_port,allowed_ip);
    if (!tcp_server.start()) {
        ROS_ERROR("Failed to start TCP server.");
        return -1;
    }
    ROS_INFO("Waiting for MCU(client) to connect...");
    if (!tcp_server.acceptClient()) {
        ROS_ERROR("Failed to accept client.");
        return -1;
    }
    ROS_INFO("MCU connected.");
    g_tcp_server = &tcp_server;

    ros::ServiceServer delivery_cmd_server = nh.advertiseService("/delivery_cmd", deliveryCmdCallback);
    ros::ServiceServer delivery_door_open_server = nh.advertiseService("/delivery_door_open", deliveryDoorOpenCallback);
    ROS_INFO("Service /delivery_cmd and /delivery_door_open are ready.");
    ros::spin();
    tcp_server.closeSocket();
    return 0;
}