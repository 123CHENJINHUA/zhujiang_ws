#!/home/cjh/miniconda3/envs/zhujiang/bin/python

import sys
# sys.path.insert(0, "/home/cjh/miniconda3/envs/zhujiang/lib/python3.10/site-packages")
sys.path.insert(0, "/home/cjh/zhujiang_ws/src")

import rospy
from std_msgs.msg import String
import threading
import queue
import time
from playsound3 import playsound
from robot_msgs.msg import ui_show
from robot_msgs.msg import Door_open
from robot_msgs.srv import ui_get, ui_getRequest, ui_getResponse
from robot_msgs.srv import pick, pickResponse

from PyQt5.QtCore import QObject, pyqtSignal

import ui_pkg.scripts.One_cabin.resources_rc
import sys
from PyQt5 import QtGui
from PyQt5 import QtWidgets, QtCore
from ui_pkg.scripts.One_cabin.homepage_ui import Ui_MainWindow
from ui_pkg.scripts.One_cabin.send_ui import Ui_Form as Ui_SendWindow
from ui_pkg.scripts.One_cabin.status_bar_controller import StatusBarController
from ui_pkg.scripts.One_cabin.Face_ui import Ui_MainWindow as Ui_FaceMainWindow
from ui_pkg.scripts.One_cabin.arrive import ArriveDialog  
from ui_pkg.scripts.One_cabin.closethedoor_ui import ClosethedoorDialog  
from ui_pkg.scripts.One_cabin.pickup_ui import Ui_Form
from ui_pkg.scripts.One_cabin.TaskSuccessDialog import Ui_Dialog

class UiNode(QObject):
    signal_recv_msg = pyqtSignal(int)  # Qt信号
    signal_set_pickup_code = pyqtSignal(str)  # 用于显示弹窗的信号

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

        # 添加Door_open话题发布者
        self.door_open_pub = rospy.Publisher('/ui_door_open', Door_open, queue_size=10)

        self.pickup_result = None  # 用于存储取件码结果
        self.Is_need_pickup_code = False  # 用于标记是否需要取件
        self.arrived = False  # 用于标记是否到达目的地

        # 音频播放相关
        self.voice_msgs_path = "/home/cjh/zhujiang_ws/src/ui_pkg/voice_msgs"

        self.sound = playsound(f'{self.voice_msgs_path}/{99}.wav', block=False)

        self.is_music = False  # 用于判断是否正在播放音乐

        # 创建音频播放线程
        self.audio_thread = threading.Thread(target=self._audio_worker, daemon=True)
        self.audio_thread.start()

    def publish_door_open(self, door_number):
        """发布开门消息"""
        try:
            msg = Door_open()
            msg.door_num = door_number
            self.door_open_pub.publish(msg)
            rospy.loginfo(f"Published door open message: door_num={door_number}")
        except Exception as e:
            rospy.logerr(f"Failed to publish door open message: {e}")

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
        self.signal_recv_msg.emit(msg.battery)  # 发射任务状态信号(battery)

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
        self.arrived = True
        if req.pickup_code > 0:
            self.Is_need_pickup_code = True
            self.signal_set_pickup_code.emit(str(req.pickup_code))
        else:
            self.Is_need_pickup_code = False
          # 发射信号显示弹窗
        resp = pickResponse()

        # 等待用户输入，直到 self.pickup_result 被设置
        while self.pickup_result is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        resp.success = self.pickup_result if self.pickup_result is not None else False
        return resp

# 自适应缩放混入类
class AdaptiveMixin:
    def init_adaptive(self, base_width=1208, base_height=800):
        """初始化自适应功能 - 简化版本，只调整窗口大小"""
        self.base_width = base_width
        self.base_height = base_height
        
        # 获取屏幕尺寸
        screen = QtWidgets.QApplication.primaryScreen()
        screen_geometry = screen.availableGeometry()
        
        # 计算适合1280x800屏幕的缩放比例
        target_width = min(1280, screen_geometry.width())
        target_height = min(800, screen_geometry.height())
        
        # 保持宽高比的缩放
        scale_x = target_width / base_width
        scale_y = target_height / base_height
        self.scale_factor = min(scale_x, scale_y)
        
        # 计算新的窗口大小
        new_width = int(base_width * self.scale_factor)
        new_height = int(base_height * self.scale_factor)
        
        # 移除大小限制
        self.setMaximumSize(16777215, 16777215)
        self.setMinimumSize(0, 0)
        
        # 调整窗口大小
        self.resize(new_width, new_height)
        
        # 居中显示
        self.center_window()
        
        # 设置键盘快捷键
        self.setup_shortcuts()
    
    def center_window(self):
        """窗口居中显示"""
        screen = QtWidgets.QApplication.primaryScreen()
        screen_geometry = screen.availableGeometry()
        
        x = (screen_geometry.width() - self.width()) // 2
        y = (screen_geometry.height() - self.height()) // 2
        
        self.move(x, y)
    
    def setup_shortcuts(self):
        """设置缩放快捷键"""
        # Ctrl + + 放大
        zoom_in = QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl++"), self)
        zoom_in.activated.connect(lambda: self.zoom(1.1))
        
        # Ctrl + - 缩小
        zoom_out = QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+-"), self)
        zoom_out.activated.connect(lambda: self.zoom(0.9))
        
        # Ctrl + 0 重置
        zoom_reset = QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+0"), self)
        zoom_reset.activated.connect(self.reset_zoom)
    
    def zoom(self, factor):
        """手动缩放"""
        self.scale_factor *= factor
        self.scale_factor = max(0.5, min(self.scale_factor, 2.0))
        
        # 重新计算窗口大小
        new_width = int(self.base_width * self.scale_factor)
        new_height = int(self.base_height * self.scale_factor)
        
        self.resize(new_width, new_height)
        self.center_window()
    
    def reset_zoom(self):
        """重置到适合1280x800的大小"""
        # 重新计算适合屏幕的缩放比例
        screen = QtWidgets.QApplication.primaryScreen()
        screen_geometry = screen.availableGeometry()
        
        target_width = min(1280, screen_geometry.width())
        target_height = min(800, screen_geometry.height())
        
        scale_x = target_width / self.base_width
        scale_y = target_height / self.base_height
        self.scale_factor = min(scale_x, scale_y)
        
        new_width = int(self.base_width * self.scale_factor)
        new_height = int(self.base_height * self.scale_factor)
        
        self.resize(new_width, new_height)
        self.center_window()

class PickupWindow(QtWidgets.QWidget, Ui_Form, AdaptiveMixin):
    def __init__(self, main_window=None,comm_node=None, status_bar_ctrl=None):
        super().__init__()
        self.setupUi(self)
        self.main_window = main_window
        self.comm_node = comm_node 

        # 初始化自适应功能
        self.init_adaptive(1280, 800)

        # 注册状态栏控件到全局控制器
        status_bar_ctrl.register(self.label, None, self.signalLabel_2, self.wifiLabel_2, self.label_5)

        # 验证码系统设置
        self.valid_codes =  None
        self.attempt_count = 0  # 尝试次数计数器
        self.max_attempts = 3   # 最大尝试次数
        
        # 4位输入框
        self.edit_list = [self.lineEdit, self.lineEdit_2, self.lineEdit_3, self.lineEdit_4]
        self.code = ""  # 当前输入的取件码

        # 绑定数字键
        self.pushButton_2.clicked.connect(lambda: self.input_digit("1"))
        self.pushButton_3.clicked.connect(lambda: self.input_digit("2"))
        self.pushButton_4.clicked.connect(lambda: self.input_digit("3"))
        self.pushButton_5.clicked.connect(lambda: self.input_digit("5"))
        self.pushButton_6.clicked.connect(lambda: self.input_digit("4"))
        self.pushButton_7.clicked.connect(lambda: self.input_digit("6"))
        self.pushButton_8.clicked.connect(lambda: self.input_digit("8"))
        self.pushButton_9.clicked.connect(lambda: self.input_digit("7"))
        self.pushButton_10.clicked.connect(lambda: self.input_digit("9"))
        self.pushButton_11.clicked.connect(lambda: self.input_digit("0"))

        # "确认"按钮
        self.pushButton_13.clicked.connect(self.confirm_code)
        # "清除"按钮
        self.pushButton_12.clicked.connect(self.clear_code)
        
        # 创建一个新的返回按钮，替代原来的pushButton_17，使用与send界面一致的样式
        self.back_button = QtWidgets.QPushButton("<<<返回", self)
        self.back_button.setGeometry(QtCore.QRect(1100, 700, 111, 40))
        self.back_button.setStyleSheet("""
            QPushButton {
                background: transparent;    /* 没有背景 */
                border: none;               /* 没有边框 */
                color: rgba(37, 37, 37, 80);  /* 浅灰色，和界面一致 */
                font-size: 20px;            /* 字体大小 */
                font-family: 'Microsoft YaHei';
                font-weight: normal;        /* 不加粗 */
                padding: 0 6px;
            }
            QPushButton:pressed {
                color: #888e99;             /* 按下时颜色略深 */
            }
        """)
        self.back_button.show()
        self.back_button.raise_()
        
        # 连接新按钮到show_main_window方法
        self.back_button.clicked.connect(self.show_main_window)
        # print(f"新返回按钮: {self.back_button}")
        # print(f"新返回按钮是否可见: {self.back_button.isVisible()}")
        # print(f"新返回按钮文本: {self.back_button.text()}")
        # print(f"新返回按钮大小: {self.back_button.size()}")
        # print(f"新返回按钮位置: {self.back_button.pos()}")
        
        # 不再使用原始的pushButton_17，因为它在UI文件中没有正确创建
        # 只使用新创建的back_button
        # print(f"使用新的返回按钮替代原始的pushButton_17")
        
        self.update_fields()  # 初始化

    def set_valid_codes(self,valid_codes):
        """
        设置有效的取件码列表。
        :param valid_codes: 有效取件码列表
        """
        self.valid_codes = valid_codes

    def input_digit(self, d):
        if len(self.code) < 4:
            self.code += d
            self.update_fields()

    def update_fields(self):
        # 更新所有输入框的内容
        for i, edit in enumerate(self.edit_list):
            if i < len(self.code):
                edit.setText(self.code[i])
            else:
                edit.setText("")

    def clear_code(self):
        self.code = ""
        self.update_fields()

    def confirm_code(self):
        if len(self.code) == 4:
            # 验证码验证
            if self.code == self.valid_codes:
                # 验证码正确
                
                # 可选：使用后移除此验证码（一次性验证码）
                # self.valid_codes.remove(self.code)

                self.comm_node.publish_door_open(door_number=1)  # 发布开门消息，假设门编号为1
                
                # 重置尝试次数
                self.attempt_count = 0
                
                # 创建半透明黑色背景遮罩
                self.overlay = QtWidgets.QWidget(self)
                self.overlay.setGeometry(self.rect())
                self.overlay.setStyleSheet("background-color: rgba(0, 0, 0, 150);")
                self.overlay.show()
                
                # 创建并显示开门对话框
                self.door_dialog = ClosethedoorDialog(self,self.comm_node)
                
                # 设置对话框样式
                self.door_dialog.setStyleSheet("""
                    QDialog {
                        background-color: white;
                        border-radius: 16px;
                    }
                """)
                
                # 居中显示对话框
                self.door_dialog.move(
                    self.rect().center().x() - self.door_dialog.width() // 2,
                    self.rect().center().y() - self.door_dialog.height() // 2
                )
                
                # 连接对话框的完成信号
                self.door_dialog.finished.connect(self.close_overlay)
                self.door_dialog.finished.connect(self.show_main_window)  # 或者使用 go_back
                
                # 显示对话框
                self.door_dialog.show()
            else:
                # 验证码错误，增加尝试次数
                self.attempt_count += 1
                
                if self.attempt_count >= self.max_attempts:
                    # 超过最大尝试次数
                    QtWidgets.QMessageBox.critical(
                        self, 
                        "验证失败", 
                        "您已超过最大尝试次数，请稍后再试！"
                    )
                    self.comm_node.pickup_result = False  # 标记取件码不正确
                    self.go_back()  # 返回主界面
                else:
                    # 显示错误信息和剩余尝试次数
                    QtWidgets.QMessageBox.warning(
                        self, 
                        "验证码错误", 
                        f"您输入的取件码不正确，还有{self.max_attempts - self.attempt_count}次尝试机会。"
                    )
                    self.clear_code()  # 清空输入框
        else:
            QtWidgets.QMessageBox.warning(self, "输入不完整", "请输入4位取件码！")

    def close_overlay(self):
        if hasattr(self, 'overlay'):
            self.overlay.hide()
            self.overlay.deleteLater()

    def show_main_window(self):
        # print("show_main_window方法被调用")
        try:
            # print("隐藏当前窗口")
            self.hide()
            if self.main_window:
                self.main_window.showFullScreen()
                # print("主窗口显示成功")
            self.clear_code()
            # 重置尝试次数
            self.attempt_count = 0
            # print("返回到主窗口成功")
        except Exception as e:
            # print(f"显示主窗口时出错: {e}")
            # 如果出错，尝试使用go_back方法
            self.go_back()

    def go_back(self):
        # print("\n\n返回按钮被点击 - go_back方法被调用 - " + self.__class__.__name__ + "\n\n")
        # print(f"返回按钮对象: {self.pushButton_17}")
        # print(f"返回按钮连接的槽函数: {self.pushButton_17.receivers(self.pushButton_17.clicked)}个")
        
        # 直接调用show_main_window方法，该方法已经包含了显示主窗口和隐藏当前窗口的逻辑
        try:
            # print("调用show_main_window方法")
            self.show_main_window()
            # print("show_main_window方法调用成功")
            return  # 成功调用后直接返回
        except Exception as e:
            print(f"调用show_main_window方法出错: {e}")
            import traceback
            traceback.print_exc()
            # 如果show_main_window方法调用失败，继续执行下面的备用逻辑
        
        # 备用逻辑：手动处理返回操作
        # 先隐藏当前窗口，再显示主窗口，避免界面闪烁
        try:
            # print("备用逻辑：隐藏当前窗口")
            self.hide()
            # print("当前窗口隐藏成功")
        except Exception as e:
            print(f"隐藏当前窗口时出错: {e}")
            import traceback
            traceback.print_exc()
            
        # 显示主窗口
        if hasattr(self, 'main_window') and self.main_window:
            # print(f"主窗口存在，显示主窗口: {self.main_window}")
            try:
                self.main_window.showFullScreen()
                # print("主窗口显示成功")
            except Exception as e:
                print(f"显示主窗口时出错: {e}")
                import traceback
                traceback.print_exc()
        else:
            print("主窗口不存在或为None")
            
        # 清除验证码
        try:
            # print("清除验证码")
            self.code = ""
            self.update_fields()
            # print("验证码清除成功")
        except Exception as e:
            print(f"清除验证码时出错: {e}")
            import traceback
            traceback.print_exc()
            
        # 重置尝试次数
        self.attempt_count = 0
        # print("尝试次数已重置")
        # print("\n\ngo_back方法执行完毕\n\n")
        
    # 可选：验证码管理方法
    def add_valid_code(self, code):
        """添加一个新的有效验证码"""
        if code not in self.valid_codes and len(code) == 4:
            self.valid_codes.append(code)
            return True
        return False

    def remove_valid_code(self, code):
        """移除一个有效验证码"""
        if code in self.valid_codes:
            self.valid_codes.remove(code)
            return True
        return False

    def load_codes_from_file(self, filename):
        """从文件加载验证码"""
        try:
            with open(filename, 'r') as file:
                codes = [line.strip() for line in file if len(line.strip()) == 4]
                self.valid_codes = codes
            return True
        except Exception as e:
            print(f"加载验证码文件失败: {e}")
            return False

    def save_codes_to_file(self, filename):
        """保存验证码到文件"""
        try:
            with open(filename, 'w') as file:
                for code in self.valid_codes:
                    file.write(code + '\n')
            return True
        except Exception as e:
            print(f"保存验证码文件失败: {e}")
            return False

# 创建一个可双击的标签类
class ClickableLabel(QtWidgets.QLabel):
    # 自定义信号
    clicked = QtCore.pyqtSignal()
    doubleClicked = QtCore.pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(event)
        
    def mouseDoubleClickEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.doubleClicked.emit()
        super().mouseDoubleClickEvent(event)
        
class FaceWindow(QtWidgets.QMainWindow, Ui_FaceMainWindow, AdaptiveMixin):
    def __init__(self, main_window=None, comm_node=None):
        super().__init__()
        self.setupUi(self)
        self.main_window = main_window  # 保存主页实例
        self.comm_node = comm_node

        # 初始化自适应功能
        self.init_adaptive(1280, 800)

        # --- 替换label为可双击的 ClickableLabel ---
        self.label.deleteLater()
        self.label = ClickableLabel(self.centralwidget)
        self.label.setGeometry(0, 0, 1280, 800)
        self.label.setScaledContents(True)
        self.label.setObjectName("label")
        # 放到中央
        self.label.lower()
        

        # 绑定GIF动画
        self.movie = QtGui.QMovie(":/icons/icons/Robot-Face.gif")
        self.label.setMovie(self.movie)
        self.movie.start()


        # 绑定双击事件
        self.label.doubleClicked.connect(self.go_home)

        # 创建定时器来持续检查到达条件
        self.check_arrival_timer = QtCore.QTimer(self)
        self.check_arrival_timer.timeout.connect(self.check_arrival_condition)
        self.check_arrival_timer.start(500)  # 每500毫秒检查一次
        
        # 添加标志防止重复弹窗
        self.dialog_shown = False
        
    def check_arrival_condition(self):
        """持续检查到达条件，满足条件时触发弹窗"""
        if (not self.comm_node.Is_need_pickup_code and 
            self.comm_node.arrived and 
            not self.dialog_shown):
            # 如果不需要取件码且已经到达目的地，且还未显示对话框
            self.dialog_shown = True  # 设置标志防止重复弹窗
            self.show_arrive_dialog()
        
    def show_arrive_dialog(self):
        # 半透明黑色遮罩
        self.overlay = QtWidgets.QWidget(self)
        self.overlay.setGeometry(self.rect())
        self.overlay.setStyleSheet("background-color: rgba(0, 0, 0, 120);")
        self.overlay.show()

        # 白底圆角弹窗
        self.arrive_dialog = ArriveDialog(self,comm_node=self.comm_node)

        self.arrive_dialog.setStyleSheet("""
            QDialog {
                background-color: white;
                border-radius: 18px;
            }
        """)
        # 居中弹窗
        parent_center = self.rect().center()
        dialog_rect = self.arrive_dialog.rect()
        self.arrive_dialog.move(
            parent_center.x() - dialog_rect.width() // 2,
            parent_center.y() - dialog_rect.height() // 2
        )
        self.arrive_dialog.show()
        self.arrive_dialog.finished.connect(self.close_overlay)

    def close_overlay(self):
        if hasattr(self, 'overlay'):
            self.comm_node.arrived = False  # 重置到达状态
            self.overlay.hide()
            self.overlay.deleteLater()
            # 重置对话框显示标志，允许下次条件满足时再次弹窗
            self.dialog_shown = False

    def go_home(self):
        if self.main_window:
            self.main_window.showFullScreen()
            # self.main_window.showFullScreen()
        self.hide()
        
    def closeEvent(self, event):
        """窗口关闭时停止定时器"""
        if hasattr(self, 'check_arrival_timer'):
            self.check_arrival_timer.stop()
        super().closeEvent(event)

# Send页面窗口
class SendWindow(QtWidgets.QWidget, Ui_SendWindow, AdaptiveMixin):
    def __init__(self, main_window=None,comm_node=None, status_bar_ctrl=None):
        super().__init__()
        self.setupUi(self)
        self.setWindowOpacity(0.0)
        self.main_window = main_window
        self.comm_node = comm_node

        self.delivery_list = []

        # 初始化自适应功能
        self.init_adaptive(1208, 800)
        # 注册状态栏控件（没有日期控件就传None）
        status_bar_ctrl.register(self.lblTime, None, self.lblSignal, self.lblwifi, self.lblBattery)

        # 在 MainWindow 中
        self.init_adaptive(1280, 800)  # 改为目标屏幕尺寸
        
        # 在 SendWindow 中  
        self.init_adaptive(1280, 800)  # 改为目标屏幕尺寸
        
        # 在 FaceWindow 中
        self.init_adaptive(1280, 800)  # 改为目标屏幕尺寸
        
        # 在 PickupWindow 中
        self.init_adaptive(1280, 800)  # 改为目标屏幕尺寸
        
        # 连接数字键盘按钮
        self.setup_numeric_keypad()
        
        # 连接返回按钮
        self.btnBack.clicked.connect(self.go_back)
        
        # 连接确认创建任务按钮
        self.btnCreateTaskConfirm.clicked.connect(self.on_create_task_confirm)
        
        # 初始化下拉框为"--"
        self.init_dropdowns()
        
        # 初始化常用地址
        self.setup_frequent_addresses()

        self.showFullScreen()
        
        # 连接房间号输入框点击事件
        self.editRoom.mousePressEvent = self.on_room_input_clicked
        
        # 存储键盘按钮的原始样式
        self.original_keypad_styles = {}
        for i in range(10):
            button = getattr(self, f"btnNum{i}")
            self.original_keypad_styles[button] = button.styleSheet()
        self.original_keypad_styles[self.btnNumClear] = self.btnNumClear.styleSheet()
        self.original_keypad_styles[self.btnNumConfirm] = self.btnNumConfirm.styleSheet()

        self.face_window = FaceWindow(main_window, comm_node=self.comm_node)
        

    def init_dropdowns(self):
        # 清空下拉框
        self.cmbBuilding.clear()
        self.cmbUnit.clear()
        
        # 添加"--"选项
        self.cmbBuilding.addItem("--")
        self.cmbUnit.addItem("--")
        
        # 添加其他选项 - 确保格式与查找时一致
        for i in range(1, 6):  # 假设有1-5栋
            self.cmbBuilding.addItem(f"{i}栋")
        
        for i in range(1, 4):  # 假设每栋有1-3单元
            self.cmbUnit.addItem(f"{i}单元")
            
        # 移除下拉框变化信号，地址更新只在确认按钮点击时进行

    def setup_frequent_addresses(self):
        # 这里可以从数据库或配置文件中读取常用地址
        # 暂时使用模拟数据
        frequent_addresses = [
            {"building": "1", "unit": "2", "room": "301"},
            {"building": "2", "unit": "3", "room": "502"},
            {"building": "3", "unit": "1", "room": "101"}
        ]
        
        # 设置常用地址按钮 - 避免闭包问题
        if len(frequent_addresses) > 0:
            addr = frequent_addresses[0]
            self.btnQuickAddr1.setText(f"{addr['building']}栋{addr['unit']}单元{addr['room']}室")
            # 使用深拷贝避免闭包问题
            addr_copy = addr.copy()
            self.btnQuickAddr1.clicked.connect(lambda checked=False, a=addr_copy: self.use_quick_address(a))
        
        if len(frequent_addresses) > 1:
            addr = frequent_addresses[1]
            self.btnQuickAddr2.setText(f"{addr['building']}栋{addr['unit']}单元{addr['room']}室")
            addr_copy = addr.copy()
            self.btnQuickAddr2.clicked.connect(lambda checked=False, a=addr_copy: self.use_quick_address(a))
            
        if len(frequent_addresses) > 2:
            addr = frequent_addresses[2]
            self.btnQuickAddr3.setText(f"{addr['building']}栋{addr['unit']}单元{addr['room']}室")
            addr_copy = addr.copy()
            self.btnQuickAddr3.clicked.connect(lambda checked=False, a=addr_copy: self.use_quick_address(a))

    def use_quick_address(self, address):
        # 设置下拉框选项 - 需要转换成带单位的格式
        building_text = f"{address['building']}栋"
        unit_text = f"{address['unit']}单元"
        
        # 打印调试信息
        # print(f"使用常用地址: 楼栋={building_text}, 单元={unit_text}, 房间={address['room']}")
        
        # 设置下拉框值
        building_index = self.cmbBuilding.findText(building_text)
        if building_index >= 0:
            # print(f"找到楼栋索引: {building_index}")
            self.cmbBuilding.setCurrentIndex(building_index)
        else:
            print(f"未找到楼栋: {building_text}")
                
        unit_index = self.cmbUnit.findText(unit_text)
        if unit_index >= 0:
            # print(f"找到单元索引: {unit_index}")
            self.cmbUnit.setCurrentIndex(unit_index)
        else:
            print(f"未找到单元: {unit_text}")
                
        # 设置房间号
        self.room_number = address["room"]
        self.editRoom.setText(self.room_number)
        
        # 更新地址显示
        self.update_current_address()

    def setup_numeric_keypad(self):
        # 连接数字按钮
        self.btnNum0.clicked.connect(lambda: self.on_numeric_key_pressed('0'))
        self.btnNum1.clicked.connect(lambda: self.on_numeric_key_pressed('1'))
        self.btnNum2.clicked.connect(lambda: self.on_numeric_key_pressed('2'))
        self.btnNum3.clicked.connect(lambda: self.on_numeric_key_pressed('3'))
        self.btnNum4.clicked.connect(lambda: self.on_numeric_key_pressed('4'))
        self.btnNum5.clicked.connect(lambda: self.on_numeric_key_pressed('5'))
        self.btnNum6.clicked.connect(lambda: self.on_numeric_key_pressed('6'))
        self.btnNum7.clicked.connect(lambda: self.on_numeric_key_pressed('7'))
        self.btnNum8.clicked.connect(lambda: self.on_numeric_key_pressed('8'))
        self.btnNum9.clicked.connect(lambda: self.on_numeric_key_pressed('9'))
    
        # 连接清除按钮
        self.btnNumClear.clicked.connect(self.on_clear_pressed)

        # 连接确认按钮
        self.btnNumConfirm.clicked.connect(self.on_confirm_pressed)

    def on_room_input_clicked(self, event):
        # 当用户点击房间号输入框时，振动键盘
        self.vibrate_keypad()
        # 调用原始的鼠标点击事件处理
        QtWidgets.QLineEdit.mousePressEvent(self.editRoom, event)

    def vibrate_keypad(self):
        # 为所有键盘按钮添加红色边框
        for i in range(10):
            button = getattr(self, f"btnNum{i}")
            button.setStyleSheet(self.original_keypad_styles[button] + "border: 2px solid red;")
        
        self.btnNumClear.setStyleSheet(self.original_keypad_styles[self.btnNumClear] + "border: 2px solid red;")
        self.btnNumConfirm.setStyleSheet(self.original_keypad_styles[self.btnNumConfirm] + "border: 2px solid red;")
        
        # 使用定时器在短暂延迟后恢复原始样式
        QtCore.QTimer.singleShot(300, self.restore_keypad_style)

    def restore_keypad_style(self):
        # 恢复所有键盘按钮的原始样式
        for button, style in self.original_keypad_styles.items():
            button.setStyleSheet(style)

    def vibrate_field(self, widget):
        # 保存原始样式
        original_style = widget.styleSheet()
        
        # 添加红色边框
        widget.setStyleSheet(original_style + "border: 2px solid red;")
        
        # 使用定时器在短暂延迟后恢复原始样式
        QtCore.QTimer.singleShot(300, lambda: widget.setStyleSheet(original_style))

    def on_numeric_key_pressed(self, digit):
        # 添加数字到房间号
        self.room_number += digit
        # 更新房间号输入框
        self.editRoom.setText(self.room_number)
        # 不再在此处更新地址显示，等待确认按钮点击
        # self.update_current_address()

    def on_clear_pressed(self):
        # 清空房间号
        self.room_number = ""
        self.editRoom.setText("")
        
        # 重置下拉框为初始状态
        self.cmbBuilding.setCurrentIndex(0)  # 设置为"--"
        self.cmbUnit.setCurrentIndex(0)      # 设置为"--"
        
        # 清空地址显示
        self.frmCurrentAddress.setText("")

    def on_confirm_pressed(self):
        # 如果验证通过，则更新地址显示
        self.update_current_address()
        
    def on_create_task_confirm(self):
        # 检查是否所有字段都已填写
        if self.cmbBuilding.currentText() == "--":
            self.vibrate_field(self.cmbBuilding)
            return
            
        if self.cmbUnit.currentText() == "--":
            self.vibrate_field(self.cmbUnit)
            return
            
        if not self.room_number:
            self.vibrate_field(self.editRoom)
            return
        
        # 更新地址显示
        self.update_current_address()
        
        # 获取当前地址
        current_address = self.frmCurrentAddress.text()
        
        # 如果地址为空，提示用户
        if not current_address:
            QtWidgets.QMessageBox.warning(self, "提示", "请先选择配送地址")
            return
            
        # 显示任务成功对话框
        self.show_task_success_dialog(current_address)

    def show_task_success_dialog(self, address):
        # 创建半透明黑色背景
        self.overlay = QtWidgets.QWidget(self)
        self.overlay.setGeometry(self.rect())
        self.overlay.setStyleSheet("background-color: rgba(0, 0, 0, 150);")
        self.overlay.show()
        
        # 创建任务成功对话框
        
        
        # 创建对话框
        self.task_dialog = QtWidgets.QDialog(self)
        self.task_dialog_ui = Ui_Dialog()
        self.task_dialog_ui.setupUi(self.task_dialog)
        
        # 设置窗口无边框
        self.task_dialog.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        
        # 设置对话框背景为白色
        self.task_dialog.setStyleSheet("""
            QDialog {
                background-color: white;
                border-radius: 16px;
            }
        """)
        
        # 直接使用蓝色框框中的地址
        self.task_dialog_ui.label_5.setText(address)
        
        # 连接按钮信号
        self.task_dialog_ui.pushButton.clicked.connect(self.on_task_dialog_cancel)
        self.task_dialog_ui.pushButton_2.clicked.connect(self.on_task_dialog_confirm)
        
        # 居中显示对话框
        self.task_dialog.move(
            self.rect().center().x() - self.task_dialog.width() // 2,
            self.rect().center().y() - self.task_dialog.height() // 2
        )
        
        # 显示对话框
        self.task_dialog.show()

    def on_task_dialog_cancel(self):
        # 关闭对话框
        self.delivery_list = []  # 清空配送列表
        self.task_dialog.close()
        # 移除背景遮罩
        if hasattr(self, 'overlay'):
            self.overlay.hide()
            self.overlay.deleteLater()

    def on_task_dialog_confirm(self):

        if self.delivery_list:
            # 拼接成服务需要的字符串格式
            delivery_str = ";".join(self.delivery_list)
            resp = self.comm_node.call_ui_get(delivery_str)
            if resp and resp.received:
                # print("配送信息已发布:", delivery_str)
                self.delivery_list.clear()
            else:
                print("配送服务调用失败！")
        else:
            print("配送列表为空，请先设置配送信息！")

        # 关闭对话框
        self.task_dialog.close()
        # 移除背景遮罩
        if hasattr(self, 'overlay'):
            self.overlay.hide()
            self.overlay.deleteLater()
        
        # 跳转到Face动画页面
        self.hide()              # 可选：隐藏当前窗口
        # 重新创建FaceWindow实例，确保定时器重新启动
        # self.face_window = FaceWindow(self.main_window)
        # self.face_window.show()  # 显示face窗口
        self.face_window.showFullScreen()


    def update_current_address(self):

        self.delivery_list = []  # 清空配送列表
        # 获取当前选择的楼栋和单元
        building = self.cmbBuilding.currentText()
        unit = self.cmbUnit.currentText()
        
        # 打印调试信息
        # print(f"更新地址: 楼栋={building}, 单元={unit}, 房间={self.room_number}")
        
        # 检查是否选择了有效的楼栋和单元
        if building == "--" or unit == "--" or not self.room_number:
            # 如果信息不完整，不显示地址
            self.frmCurrentAddress.setText("")
            return
        
        # 解析房间号，提取层和室   
        room_str = str(self.room_number)
        if len(room_str) == 4:
            # 4位数：前2位是层，后2位是室
            floor = room_str[:2]
            room = room_str[2:]
        elif len(room_str) == 3:
            # 3位数：第1位是层，后2位是室
            floor = room_str[:1]
            room = room_str[1:]
        else:
            # 其他情况，使用原房间号作为室号，层为空
            floor = ""
            room = room_str
                
        # 组合成配送信息格式
        delivery_info = f"{building},{unit},{floor},{room}"
        self.delivery_list.append(delivery_info)
            
        # 更新当前地址显示（保持原有显示格式）
        display_address = f"{building} {unit} {self.room_number}室"
        self.frmCurrentAddress.setText(display_address)

    def fade_in(self, duration=100):
        self.anim = QtCore.QPropertyAnimation(self, b"windowOpacity")
        self.anim.setDuration(duration)
        self.anim.setStartValue(0.0)
        self.anim.setEndValue(1.0)
        self.anim.start()

    def reset_ui(self):
        # 重置房间号输入
        self.room_number = ""
        self.editRoom.setText("")
        # 重置下拉框
        self.init_dropdowns()
        # 清空地址显示
        self.frmCurrentAddress.setText("")

    def go_back(self):
        # print("\n\nSendWindow - 返回按钮被点击 - go_back方法被调用\n\n")
        # print(f"返回按钮对象: {self.btnBack}")
        # print(f"返回按钮连接的槽函数: {self.btnBack.receivers(self.btnBack.clicked)}个")
        
        try:
            # print("隐藏当前窗口")
            self.hide()
            # print("当前窗口隐藏成功")
        except Exception as e:
            print(f"隐藏当前窗口时出错: {e}")
            import traceback
            traceback.print_exc()
            
        if hasattr(self, 'main_window') and self.main_window:
            # print(f"主窗口存在，显示主窗口: {self.main_window}")
            try:
                self.main_window.showFullScreen()
                # print("主窗口显示成功")
            except Exception as e:
                print(f"显示主窗口时出错: {e}")
                import traceback
                traceback.print_exc()
        else:
            print("主窗口不存在或为None")

# 首页窗口
# 在MainWindow类的__init__方法中添加
class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow, AdaptiveMixin):
    def __init__(self, comm_node,status_bar_ctrl):
        super().__init__()
        self.setupUi(self)

        self.comm_node = comm_node
        self.comm_node.signal_set_pickup_code.connect(self.on_set_pickup_code)

        # 初始化自适应功能（可能需要调整或注释掉）
        self.init_adaptive(1208, 800)
        
        
        # 注册状态栏控件
        status_bar_ctrl.register(self.lblTime, self.lblDate, self.lblSignal, self.lblwifi, self.lblBattery)
        # print("\n\n初始化MainWindow...\n\n")
        self.send_window = SendWindow(self, comm_node=self.comm_node,status_bar_ctrl=status_bar_ctrl)
        # print(f"创建SendWindow实例: {self.send_window}")
        self.frmCreateTask.mousePressEvent = self.open_send_window
        # print("连接寄件窗口点击事件")
        
        self.pickup_window = PickupWindow(self, comm_node=self.comm_node,status_bar_ctrl=status_bar_ctrl)
        # print(f"创建PickupWindow实例: {self.pickup_window}")
        # print(f"PickupWindow的main_window属性: {self.pickup_window.main_window}")
        self.frmPickupParcel.mousePressEvent = self.open_pickup_window
        # print("连接取件窗口点击事件")

        # 添加全屏显示
        self.showFullScreen()

    def on_set_pickup_code(self, code):
        self.pickup_window.set_valid_codes(code)

    def open_send_window(self, event):
        self.send_window.reset_ui() # 在显示SendWindow之前重置UI
        self.comm_node.publish_door_open(door_number=1) #打开仓门
        self.send_window.showFullScreen()
        self.send_window.fade_in()
        self.hide()

    def open_pickup_window(self, event):
        # print("\n\n打开取件窗口\n\n")
        try:
            # 确保pickup_window已正确初始化，并且main_window参数为self
            if not hasattr(self, 'pickup_window') or self.pickup_window is None:
                self.pickup_window = PickupWindow(self)
                print("重新创建PickupWindow实例")
            
            # 确保返回按钮连接到show_main_window方法
            try:
                self.pickup_window.back_button.clicked.disconnect()
            except Exception:
                pass
            self.pickup_window.back_button.clicked.connect(self.pickup_window.show_main_window)
            # print("返回按钮已连接到show_main_window方法")
            # print(f"返回按钮: {self.pickup_window.back_button}")
            # print(f"show_main_window方法: {self.pickup_window.show_main_window}")
            
            # 检查main_window属性
            # print(f"PickupWindow的main_window属性: {self.pickup_window.main_window}")
            
            # 确保返回按钮可见
            self.pickup_window.back_button.setVisible(True)
            self.pickup_window.back_button.raise_()
            
            # 测试返回按钮是否可点击
            # print(f"返回按钮是否可见: {self.pickup_window.back_button.isVisible()}")
            # print(f"返回按钮是否启用: {self.pickup_window.back_button.isEnabled()}")
            # print(f"返回按钮文本: {self.pickup_window.back_button.text()}")
            # print(f"返回按钮大小: {self.pickup_window.back_button.size()}")
            # print(f"返回按钮位置: {self.pickup_window.back_button.pos()}")
            
            self.pickup_window.showFullScreen()  # 显示取件窗口
            
            # 在窗口显示后再次确保返回按钮可见
            QtCore.QTimer.singleShot(100, lambda: self.ensure_button_visible())
            
            # print("取件窗口已显示")
            self.hide()                # 隐藏主页窗口
            # print("主页窗口已隐藏")
        except Exception as e:
            print(f"打开取件窗口时出错: {e}")
            import traceback
            traceback.print_exc()
            
    def ensure_button_visible(self):
        if hasattr(self, 'pickup_window') and self.pickup_window:
            # 确保返回按钮可见
            self.pickup_window.back_button.setVisible(True)
            # print(f"延迟设置返回按钮可见性: {self.pickup_window.back_button.isVisible()}")
            self.pickup_window.back_button.raise_()
            # print(f"延迟提升返回按钮到顶层")
            # print(f"返回按钮位置: {self.pickup_window.back_button.pos()}")
            # print(f"返回按钮大小: {self.pickup_window.back_button.size()}")

# 在单独线程中运行rospy.spin
def ros_spin():
    rospy.spin()

def main(args=None):
    app = QtWidgets.QApplication(sys.argv)
    
    # 添加这个函数来移除所有窗口的边框
    def remove_title_bar(cls):
        old_init = cls.__init__
        def new_init(self, *args, **kwargs):
            old_init(self, *args, **kwargs)
            self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        cls.__init__ = new_init
    
    # 应用到所有窗口类型
    remove_title_bar(QtWidgets.QMainWindow)
    remove_title_bar(QtWidgets.QDialog)
    remove_title_bar(QtWidgets.QWidget)

    rospy.init_node("ui_node")
    node = UiNode()
    rospy.loginfo("Waiting for UI_get service...")
    node.ui_get_client.wait_for_service()
    rospy.loginfo("UI_get service is available.")

    status_bar_ctrl = StatusBarController(comm_node=node)

    win = MainWindow(comm_node=node,status_bar_ctrl=status_bar_ctrl)
    win.show()
     # 启动ROS spin线程
    thread_spin = threading.Thread(target=ros_spin)
    thread_spin.daemon = True
    thread_spin.start()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()