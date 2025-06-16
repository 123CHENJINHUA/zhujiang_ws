#!/home/cjh/miniconda3/envs/zhujiang/bin/python

import sys
sys.path.insert(0, "/home/cjh/miniconda3/envs/zhujiang/lib/python3.10/site-packages")

import rospy
from std_msgs.msg import String

from playsound import playsound
from robot_msgs.msg import ui_show
from robot_msgs.srv import ui_get, ui_getRequest, ui_getResponse
class UiNode:
    def __init__(self):
        # 订阅 /UI_show
        self.ui_show_sub = rospy.Subscriber("/UI_show", ui_show, self.ui_show_callback)
        # 订阅 /speach
        self.speach_sub = rospy.Subscriber("/speach", String, self.speach_callback)
        # 订阅 /calling
        self.calling_sub = rospy.Subscriber("/calling", String, self.calling_callback)
        # ui客户端
        self.ui_get_client = rospy.ServiceProxy('/UI_get', ui_get)

        self.voice_msgs_path = "/home/cjh/zhujiang_ws/src/ui_pkg/voice_msgs"

    # 订阅
    def ui_show_callback(self, msg):
        rospy.loginfo("network: %s, odometry: %s, speed: %s, working_time: %s, battery: %s, task_status: %s", msg.network, msg.odometry, msg.speed, msg.working_time, msg.battery, msg.task_status)

    def speach_callback(self, msg):
        rospy.loginfo("Speach: %s", msg.data)
        wav_path = f'{self.voice_msgs_path}/{msg.data}.wav'
        playsound(wav_path)

    def calling_callback(self, msg):
        rospy.loginfo("data: %s", msg.data)

    # 客户端
    def call_ui_get(self, delivery_list):
        try:
            req = ui_getRequest()
            req.delivery_list = delivery_list
            resp = self.ui_get_client.call(req)
            if resp.received:
                rospy.loginfo("UI_get call success, task list received.")
            else:
                rospy.logwarn("UI_get call failed, task list not accepted.")
            return resp
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None



if __name__ == "__main__":
    rospy.init_node("ui_node")
    node = UiNode()
    rospy.loginfo("Waiting for UI_get service...")
    node.ui_get_client.wait_for_service()
    rospy.loginfo("UI_get service is available.")
    node.call_ui_get("1,1,1,1;2,2,2,2;3,3,3,3")
    rospy.spin()