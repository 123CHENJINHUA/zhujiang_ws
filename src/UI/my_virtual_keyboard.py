from PyQt5.QtWidgets import QDialog, QGridLayout, QPushButton, QLineEdit, QDesktopWidget
from PyQt5.QtCore import Qt

class VirtualKeyboard(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("虚拟键盘")
        self.setFixedSize(240, 180)  # 更小的窗口
        self.input_line = QLineEdit(self)
        self.input_line.setFixedHeight(28)
        self.layout = QGridLayout(self)
        self.layout.setSpacing(4)  # 按钮间距更小
        self.layout.setContentsMargins(6, 6, 6, 6)  # 边距更小
        self.layout.addWidget(self.input_line, 0, 0, 1, 3)
        keys = [
            ['1','2','3'],
            ['4','5','6'],
            ['7','8','9'],
            ['0','清空','确定']
        ]
        for row, key_row in enumerate(keys, 1):
            for col, key in enumerate(key_row):
                btn = QPushButton(key)
                btn.setFixedSize(56, 32)  # 更小的按钮
                btn.clicked.connect(self.handle_button)
                self.layout.addWidget(btn, row, col)
        self.result = None

        # 设置窗口无标题栏且置顶
        self.setWindowFlags(Qt.Tool | Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)

        # 移动到屏幕底部中央
        screen = QDesktopWidget().availableGeometry()
        x = (screen.width() - self.width()) // 2
        y = screen.height() - self.height() - 10  # 距离底部10像素
        self.move(x, y)

    def handle_button(self):
        sender = self.sender()
        text = sender.text()
        if text == '清空':
            self.input_line.clear()
        elif text == '确定':
            self.result = self.input_line.text()
            self.accept()
        else:
            self.input_line.insert(text)

    def get_input(self, init_text=""):
        self.input_line.setText(init_text)
        if self.exec_() == QDialog.Accepted:
            return self.result
        return None