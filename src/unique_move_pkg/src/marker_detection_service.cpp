#include <ros/ros.h>
#include <robot_msgs/MarkerArray.h>
#include <robot_msgs/CheckMarkerId.h>
#include <set>
#include <mutex>

class MarkerDetectionService {
private:
    ros::NodeHandle nh_;
    ros::Subscriber marker_sub_;
    ros::ServiceServer check_marker_service_;
    
    std::set<int32_t> detected_markers_;
    std::mutex markers_mutex_;
    
public:
    MarkerDetectionService() {
        // 订阅marker话题
        marker_sub_ = nh_.subscribe<robot_msgs::MarkerArray>(
            "/aruco_marker_publisher/markers", 10, 
            &MarkerDetectionService::markerCallback, this);
        
        // 创建服务
        check_marker_service_ = nh_.advertiseService(
            "check_marker_id", &MarkerDetectionService::checkMarkerCallback, this);
        ROS_INFO("Marker detection service node started");
    }
    
    void markerCallback(const robot_msgs::MarkerArray::ConstPtr& data) {
        std::lock_guard<std::mutex> lock(markers_mutex_);
        
        // 清空之前的检测结果
        detected_markers_.clear();
        
        // 记录当前检测到的所有marker id
        for (const auto& marker : data->markers) {
            detected_markers_.insert(marker.id);
            ROS_DEBUG("Detected marker ID: %d, Confidence: %.2f", 
                     marker.id, marker.confidence);
        }
        
        if (!data->markers.empty()) {
            ROS_INFO("Currently detected %lu markers", data->markers.size());
            
            // 打印所有检测到的marker id
            std::string marker_ids = "Detected marker IDs: ";
            for (const auto& id : detected_markers_) {
                marker_ids += std::to_string(id) + " ";
            }
            ROS_INFO("%s", marker_ids.c_str());
        }
    }
    
    bool checkMarkerCallback(robot_msgs::CheckMarkerId::Request& req,
                           robot_msgs::CheckMarkerId::Response& res) {
        std::lock_guard<std::mutex> lock(markers_mutex_);
        
        // 检查请求的marker id是否在检测到的列表中
        bool found = detected_markers_.find(req.marker_id) != detected_markers_.end();
        res.found = found;
        
        if (found) {
            ROS_INFO("Marker ID %d is currently detected - returning true", req.marker_id);
        } else {
            ROS_INFO("Marker ID %d is NOT detected - returning false", req.marker_id);
        }
        
        return true;
    }
    
    void run() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_detection_service");
    
    MarkerDetectionService service;
    service.run();
    
    return 0;
}
