#!/home/cjh/miniconda3/envs/zhujiang/bin/python

import sys
sys.path.insert(0, "/home/cjh/miniconda3/envs/zhujiang/lib/python3.10/site-packages")

import rospy
from std_msgs.msg import String
from robot_msgs.srv import ui, uiRequest, uiResponse

from playsound import playsound

class UiNode:
    def __init__(self):
        # 订阅 /UI_show
        self.ui_show_sub = rospy.Subscriber("/UI_show", String, self.ui_show_callback)
        # 订阅 /speach
        self.speach_sub = rospy.Subscriber("/speach", String, self.speach_callback)
        # 订阅 /calling
        self.calling_sub = rospy.Subscriber("/calling", String, self.calling_callback)
        # 如果需要客户端，可以这样写
        # self.ui_get_client = rospy.ServiceProxy('/UI_get', robot_msgs.srv.ui)

        self.voice_msgs_path = "/home/cjh/zhujiang_ws/src/ui_pkg/voice_msgs"

    def ui_show_callback(self, msg):
        rospy.loginfo("Python UI Show: %s", msg.data)

    def speach_callback(self, msg):
        rospy.loginfo("Speach: %s", msg.data)
        wav_path = f'{self.voice_msgs_path}/{msg.data}.wav'
        playsound(wav_path)

    def calling_callback(self, msg):
        rospy.loginfo("Calling: %s", msg.data)

if __name__ == "__main__":
    rospy.init_node("ui_node")
    node = UiNode()
    rospy.spin()