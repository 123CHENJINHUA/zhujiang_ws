from PyQt5 import QtCore, QtGui

class StatusBarController(QtCore.QObject):
    def __init__(self, comm_node, parent=None):
        super().__init__(parent)

        self.comm_node = comm_node
        self.battery_status = 0  # 用于接收电池状态
        self.comm_node.signal_recv_msg.connect(self.set_recv_msgs)

        self.status_bars = []
        self.charging = False  # 全局充电状态
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_all)
        self.timer.start(1000)  # 1秒刷新一次

    def set_recv_msgs(self, battery_status):
        """
        接收来自comm_node的消息，更新电池状态。
        :param battery_status: 电池状态值
        """
        self.battery_status = battery_status

    def register(self, lblTime, lblDate, lblSignal, lblwifi, lblBattery):
        # 允许lblDate为None（如果没有日期标签）
        self.status_bars.append((lblTime, lblDate, lblSignal, lblwifi, lblBattery))

    def set_charging(self, is_charging):
        self.charging = is_charging
        self.update_all()

    def update_all(self):
        import datetime, random
        now = datetime.datetime.now()
        week_map = ['一', '二', '三', '四', '五', '六', '日']
        date_str = f'{now.year}年{now.month}月{now.day}日 星期{week_map[now.weekday()]}'
        signal_icons = [
            ":/icons/icons/Signal_no.png",
            ":/icons/icons/Signal_1.png",
            ":/icons/icons/Signal_2.png",
            ":/icons/icons/Signal_3.png",
            ":/icons/icons/Signal_Full.png"
        ]
        wifi_icons = [
            ":/icons/icons/wifi_no.png",
            ":/icons/icons/wifi_1.png",
            ":/icons/icons/wifi_2.png",
            ":/icons/icons/wifi_3.png",
            ":/icons/icons/wifi_full.png"
        ]
        battery_icons = [
            ":/icons/icons/battery_25.png",
            ":/icons/icons/battery_50.png",
            ":/icons/icons/battery_full.png",
            ":/icons/icons/battery_charging_full.png"
        ]
        # 充电状态处理
        if self.charging:
            bat = battery_icons[-1]  # 显示充电图标
        else:
            if self.battery_status <= 25:
                bat = battery_icons[0]  # 低电量
            elif self.battery_status <= 50:
                bat = battery_icons[1]  # 中等电量
            elif self.battery_status > 50:  # 高电量
                bat = battery_icons[2]  # 高电量
            else:
                bat = battery_icons[3]  # 默认充电图标

        sig = random.choice(signal_icons)
        wifi = random.choice(wifi_icons)

        for lblTime, lblDate, lblSignal, lblwifi, lblBattery in self.status_bars:
            lblTime.setText(now.strftime("%H:%M"))
            if lblDate is not None:
                lblDate.setText(f'<span style="font-size:10pt;">{date_str}</span>')
            lblSignal.setPixmap(QtGui.QPixmap(sig))
            lblwifi.setPixmap(QtGui.QPixmap(wifi))
            lblBattery.setPixmap(QtGui.QPixmap(bat))
