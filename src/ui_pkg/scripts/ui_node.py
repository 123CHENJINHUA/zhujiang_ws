#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import threading
import queue
import time

import sys
# sys.path.insert(0, "/home/cjh/miniconda3/envs/zhujiang/lib/python3.10/site-packages")
sys.path.insert(0, "/home/cjh/zhujiang_ws/src")


from PyQt5.QtWidgets import QApplication, QMainWindow, QDialog, QMessageBox
from PyQt5.QtCore import QObject, pyqtSignal
from UI.zhujiang_ui import *
from UI.tanchuang import *
from UI.my_virtual_keyboard import VirtualKeyboard


from playsound3 import playsound
from robot_msgs.msg import ui_show
from robot_msgs.srv import ui_get, ui_getRequest, ui_getResponse
from robot_msgs.srv import pick, pickResponse

class UiNode(QObject):
    signal_recv_msg = pyqtSignal(str)  # Qt信号
    signal_show_tanchuang = pyqtSignal(str)  # 用于显示弹窗的信号

    def __init__(self):
        super(UiNode, self).__init__()
        # 订阅 /UI_show
        self.ui_show_sub = rospy.Subscriber("/UI_show", ui_show, self.ui_show_callback)
        # 订阅 /speach
        self.speach_sub = rospy.Subscriber("/speach", String, self.speach_callback)
        # 订阅 /calling
        self.calling_sub = rospy.Subscriber("/calling", String, self.calling_callback)
        # ui客户端
        self.ui_get_client = rospy.ServiceProxy('/UI_get', ui_get)

        # pickup 服务端
        self.pickup_service = rospy.Service('/pickup', pick, self.handle_pickup)

        self.pickup_result = None  # 用于存储取件码结果

        # 音频播放相关
        self.voice_msgs_path = "/home/cjh/zhujiang_ws/src/ui_pkg/voice_msgs"

        self.sound = playsound(f'{self.voice_msgs_path}/{99}.wav', block=False)

        self.is_music = False  # 用于判断是否正在播放音乐

        # 创建音频播放线程
        self.audio_thread = threading.Thread(target=self._audio_worker, daemon=True)
        self.audio_thread.start()

    def _audio_worker(self):
        while not rospy.is_shutdown():
            try:
                if self.is_music and not self.sound.is_alive():  # 检查音频路径是否有效且当前没有音频在播放
                    self.sound = playsound(f'{self.voice_msgs_path}/{100}.wav', block=False)  # 播放音频
            except Exception as e:
                rospy.logerr(f"Audio playback error: {e}")


    # 订阅
    def ui_show_callback(self, msg):
        # rospy.loginfo("network: %s, odometry: %s, speed: %s, working_time: %s, battery: %s, task_status: %s", msg.network, msg.odometry, msg.speed, msg.working_time, msg.battery, msg.task_status)
        self.signal_recv_msg.emit(f"network: {msg.network}\nodometry: {msg.odometry}\nspeed: {msg.speed}\nworking_time: {msg.working_time}\nbattery: {msg.battery}\ntask_process: {msg.task_process}\ntask_status: {msg.task_status}\ncurrent_task: {msg.current_task}\nrest_task: {msg.rest_task} ")  # 发射任务状态信号

    def speach_callback(self, msg):
        wav_path = f'{self.voice_msgs_path}/{msg.data}.wav'
        rospy.loginfo("Received speech message: %s", msg.data)
        if msg.data == '100':
            self.is_music = True  # 设置正在播放音乐状态
        else:
            if self.is_music:
                self.sound.stop()  # 停止当前音乐
                self.is_music = False  # 重置音乐状态
            playsound(wav_path, block=True)  # 播放语音消息
            


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
        
    def handle_pickup(self, req):
        rospy.loginfo("Received pickup request: %d", req.pickup_code)
        self.pickup_result = None  # 每次请求前重置
        self.signal_show_tanchuang.emit(str(req.pickup_code))  # 发射信号显示弹窗
        resp = pickResponse()

        # 等待用户输入，直到 self.pickup_result 被设置
        while self.pickup_result is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        resp.success = self.pickup_result if self.pickup_result is not None else False
        return resp


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, comm_node):
        super(MainWindow, self).__init__()
        self.setupUi(self)
        self.comm_node = comm_node
        self.comm_node.signal_recv_msg.connect(self.set_recv_msgs)
        self.comm_node.signal_show_tanchuang.connect(self.show_tanchuang_dialog)

        self.delivery_list = []
        self.delivery_list_show = []
        self.manual_editing = False  #手动编辑锁

        self.label_7.setStyleSheet("font-size: 24px;")
        self.pushButton.clicked.connect(self.on_delivery_button_click)
        self.pushButton_2.clicked.connect(self.on_settings_button_click)

        # 只在点击文本框时弹出虚拟键盘
        self.textEdit_1.mousePressEvent = self.make_virtual_keyboard_handler(self.textEdit_1)
        self.textEdit_2.mousePressEvent = self.make_virtual_keyboard_handler(self.textEdit_2)
        self.textEdit_3.mousePressEvent = self.make_virtual_keyboard_handler(self.textEdit_3)
        self.textEdit_4.mousePressEvent = self.make_virtual_keyboard_handler(self.textEdit_4)

        # 初始化时不聚焦任何文本框，聚焦到主窗口
        self.setFocus()

        # 设置窗口为全屏显示
        self.showFullScreen()

    def make_virtual_keyboard_handler(self, edit_widget):
        def handler(event):
            vk = VirtualKeyboard(self)
            vk.resize(800, 400)  # Set the virtual keyboard size (width x height)
            screen_geometry = QApplication.desktop().screenGeometry()
            vk.move(screen_geometry.width() - vk.width() - 50, (screen_geometry.height() - vk.height()) // 2)  # Position at center-right
            vk.show()
            # 兼容 QLineEdit 和 QTextEdit
            if hasattr(edit_widget, "toPlainText"):
                old_text = edit_widget.toPlainText()
                text = vk.get_input(old_text)
                if text is not None:
                    edit_widget.setPlainText(text)
            else:
                old_text = edit_widget.text()
                text = vk.get_input(old_text)
                if text is not None:
                    edit_widget.setText(text)
        return handler


    def set_recv_msgs(self, msg):
        # 将多行字符串按行分割
        lines = msg.strip().split('\n')
        msg_dict = {}
        for line in lines:
            if ':' in line:
                key, value = line.split(':', 1)
                msg_dict[key.strip()] = value.strip()
        # 根据key设置不同label
        if 'current_task' in msg_dict and not self.manual_editing:
            self.label_6.setText(msg_dict['current_task'])
        if 'rest_task' in msg_dict and not self.manual_editing:
            # 处理rest_task为数组字符串的情况
            rest = msg_dict['rest_task']
            # 去除首尾的中括号和空格，然后按逗号分割
            rest = rest.strip()
            if rest.startswith('[') and rest.endswith(']'):
                rest = rest[1:-1]
            rest_list = [item.strip().strip("'").strip('"') for item in rest.split(',') if item.strip()]
            # 存放到 delivery_list_show
            self.delivery_list_show = rest_list if rest_list and rest_list != [''] else []
            self.update_delivery_list_label()
            self.delivery_list_show.clear()  # 清空显示列表
        if 'network' in msg_dict:
            self.label_11.setText("网络状态：" + msg_dict['network'] + "\n电量：" + msg_dict['battery'] + f"\n速度：{float(msg_dict['speed']):.2f} km/h" + "\n总行驶距离：" + msg_dict['odometry'] + "\n工作时间：" + msg_dict['working_time'])
        if 'task_process' in msg_dict:
            self.label_12.setText("进度：\n" + msg_dict['task_process'] + "%")
        if 'task_status' in msg_dict:
            self.label_13.setText("任务状态：\n" + msg_dict['task_status'])

    def show_tanchuang_dialog(self, pickup_code_str):
        dialog = Tanchuang(self, pickup_code_str,self.comm_node)
        dialog.exec_()

    def on_settings_button_click(self):
        self.manual_editing = True  # 开始手动编辑
        building = self.textEdit_1.toPlainText().strip()
        unit = self.textEdit_2.toPlainText().strip()
        floor = self.textEdit_3.toPlainText().strip()
        room = self.textEdit_4.toPlainText().strip()
        if building and floor and unit:
            delivery_info = f"{building},{unit},{floor},{room}"
            delivery_info_show = f"{building}栋{unit}单元{floor}层{room}室"
            self.delivery_list_show.append(delivery_info_show)
            self.delivery_list.append(delivery_info)
            self.update_delivery_list_label()
            print("设置配送信息:", delivery_info)
        else:
            print("请填写完整的配送信息！")

    def on_delivery_button_click(self):
        if self.delivery_list:
            # 拼接成服务需要的字符串格式
            delivery_str = ";".join(self.delivery_list)
            resp = self.comm_node.call_ui_get(delivery_str)
            if resp and resp.received:
                print("配送信息已发布:", delivery_str)
                self.delivery_list.clear()
                self.delivery_list_show.clear()
                self.manual_editing = False  # 恢复自动刷新
            else:
                print("配送服务调用失败！")
        else:
            print("配送列表为空，请先设置配送信息！")

    def update_delivery_list_label(self):
        if self.delivery_list_show:
            self.label_8.setText("\n".join(self.delivery_list_show))
        else:
            self.label_8.setText("配送列表为空")

class Tanchuang(QDialog, Ui_Dialog):
    def __init__(self, parent=None, pickup_code_str=None, comm_node=None):
        super(Tanchuang, self).__init__(parent)
        self.setupUi(self)
        self.pickup_code_str = pickup_code_str
        self.comm_node = comm_node
        self.label.setText("请输入取件码")
        self.attempts = 0  # 输入次数
        self.max_attempts = 5
        self.pushButton.clicked.connect(self.check_code)
        self.pushButton_2.clicked.connect(self.reject)  # 取消按钮

        # 只在点击textEdit时弹出虚拟键盘
        self.textEdit.mousePressEvent = self.show_virtual_keyboard

    def show_virtual_keyboard(self, event):
        vk = VirtualKeyboard(self)
        vk.resize(800, 400)  # Set the virtual keyboard size (width x height)
        screen_geometry = QApplication.desktop().screenGeometry()
        vk.move(screen_geometry.width() - vk.width() - 50, (screen_geometry.height() - vk.height()) // 2)  # Position at center-right
        vk.show()
        text = vk.get_input(self.textEdit.toPlainText())
        if text is not None:
            self.textEdit.setText(text)

    def get_input_code(self):
        num = self.textEdit.toPlainText().strip()
        return num

    def check_code(self):
        input_code = self.get_input_code()
        if input_code == self.pickup_code_str:
            self.comm_node.pickup_result = True
            self.accept()  # 关闭窗口
        else:
            self.attempts += 1
            if self.attempts >= self.max_attempts:
                self.label.setText("输入错误次数过多，已取消")
                self.comm_node.pickup_result = False
                self.accept()  # 关闭窗口
            else:
                self.label.setText(f"输入错误，请重试（剩余{self.max_attempts - self.attempts}次）")
    def reject(self):
        self.comm_node.pickup_result = False
        super(Tanchuang, self).reject()  # 关闭窗口


# 在单独线程中运行rospy.spin
def ros_spin():
    rospy.spin()

def main(args=None):
    app = QApplication(sys.argv)
    rospy.init_node("ui_node")
    node = UiNode()
    rospy.loginfo("Waiting for UI_get service...")
    node.ui_get_client.wait_for_service()
    rospy.loginfo("UI_get service is available.")
    w = MainWindow(node)
    w.show()
    # 启动ROS spin线程
    thread_spin = threading.Thread(target=ros_spin)
    thread_spin.daemon = True
    thread_spin.start()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()