import sys
import serial
import serial.tools.list_ports
import struct
import threading
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QMessageBox
from collections import deque
import time

import os, PyQt5

dirname = os.path.dirname(PyQt5.__file__)
qt_dir = os.path.join(dirname, 'Qt5', 'plugins', 'platforms')
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = qt_dir #
# ================= 配置区域 =================
# 串口配置 (请修改为你的实际端口)
SERIAL_PORT = 'COM14'  # Windows 示例: COM3, COM5; Mac/Linux: /dev/ttyUSB0
BAUD_RATE = 921600  # 与 STM32 保持一致 (虚拟串口下该值其实无所谓，但建议写高)

# 数据协议配置
FRAME_HEADER = b'\xAA\xBB'
PACKET_LEN = 14  # AA BB + X(4) + Y(4) + Z(4) = 14 Bytes

# 绘图配置
MAX_POINTS = 2000  # 屏幕上保留多少个历史点
REFRESH_RATE = 30  # 刷新率 (ms)，30ms ≈ 33fps

# 传感器参数 (根据 ADS1256 参数计算)
# Vref=2.5V, PGA=64, MaxCode = 2^23 - 1
# 1 Code = 2 * 2.5 / 64 / 8388607 ≈ 9.313e-9 V
# 这里默认转换为 uV (微伏)
SCALE_FACTOR = (2 * 2.5 / 64 / 8388607) * 1e6


# ===========================================

class SerialReader:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.running = False
        self.ser = None

        # 使用 deque (双端队列) 存储绘图数据，自动挤出旧数据
        self.data_x = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)
        self.data_y = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)
        self.data_z = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)

        self.buffer = bytearray()  # 接收缓冲区

    def start(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.running = True
            # 开启后台线程读取串口
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print(f"串口 {self.port} 已连接，开始接收数据...")
            return True
        except Exception as e:
            print(f"串口打开失败: {e}")
            return False

    def stop(self):
        self.running = False
        # 2. 等待线程安全退出 (避免强制杀线程导致串口没释放)
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=1.0)  # 最多等1秒

        # 3. 最后物理关闭串口
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print("串口已安全关闭")
            except Exception as e:
                print(f"关闭串口时出错: {e}")

    def _read_loop(self):
        while self.running:
            try:
                # 1. 尽可能多地读取数据 (避免频繁调用 I/O)
                if self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting)
                    self.buffer.extend(chunk)

                # 2. 解析数据包 (处理粘包、断包)
                while len(self.buffer) >= PACKET_LEN:
                    # 查找帧头
                    header_idx = self.buffer.find(FRAME_HEADER)

                    if header_idx == -1:
                        # 没找到头，保留最后一点数据以防头被截断，其余丢弃
                        self.buffer = self.buffer[-1:]
                        break

                    if header_idx + PACKET_LEN > len(self.buffer):
                        # 找到了头，但数据不够长，等待下次读取
                        break

                    # 提取完整一帧
                    packet = self.buffer[header_idx: header_idx + PACKET_LEN]

                    # 从缓冲区移除已处理的数据 (包括前面的垃圾数据)
                    del self.buffer[:header_idx + PACKET_LEN]

                    # 3. 解析二进制数据
                    # < = 小端模式, i = int32 (4字节), x = pad byte (跳过)
                    # packet[2:] 跳过 AA BB
                    try:
                        # 解析 3个 int32 (X, Y, Z)
                        raw_x, raw_y, raw_z = struct.unpack('<iii', packet[2:])

                        # 存入队列 (转换为物理量)
                        self.data_x.append(raw_x * SCALE_FACTOR)
                        self.data_y.append(raw_y * SCALE_FACTOR)
                        self.data_z.append(raw_z * SCALE_FACTOR)

                    except struct.error:
                        print("数据解析错误")

            except Exception as e:
                print(f"读取线程错误: {e}")
                time.sleep(0.1)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"STM32 ADS1256 三维力实时监控 - {SERIAL_PORT}")
        self.resize(1000, 800)

        # 设置主界面布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # 初始化 PyQtGraph
        pg.setConfigOption('background', 'w')  # 白色背景
        pg.setConfigOption('foreground', 'k')  # 黑色坐标轴

        self.graph_layout = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graph_layout)

        # 创建三个图表 (垂直排列)
        self.p1 = self.graph_layout.addPlot(title="X 轴受力 (Fx)")
        self.p1.setLabel('left', '电压 (uV)')
        self.p1.showGrid(x=True, y=True)
        self.curve_x = self.p1.plot(pen=pg.mkPen(color='#D95319', width=2))

        self.graph_layout.nextRow()  # 换行

        self.p2 = self.graph_layout.addPlot(title="Y 轴受力 (Fy)")
        self.p2.setLabel('left', '电压 (uV)')
        self.p2.showGrid(x=True, y=True)
        self.curve_y = self.p2.plot(pen=pg.mkPen(color='#EDB120', width=2))

        self.graph_layout.nextRow()  # 换行

        self.p3 = self.graph_layout.addPlot(title="Z 轴受力 (Fz)")
        self.p3.setLabel('left', '电压 (uV)')
        self.p3.showGrid(x=True, y=True)
        self.curve_z = self.p3.plot(pen=pg.mkPen(color='#0072BD', width=2))

        # 启动串口逻辑
        self.reader = SerialReader(SERIAL_PORT, BAUD_RATE)
        if not self.reader.start():
            QMessageBox.critical(self, "错误", f"无法打开串口 {SERIAL_PORT}，请检查连接！")

        # 设置定时器刷新界面 (UI线程)
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(REFRESH_RATE)

    def update_plot(self):
        # 将 deque 转为 numpy 数组进行绘图
        # 这里不需要锁，因为 deque 的操作是原子性的，且我们只读
        self.curve_x.setData(np.array(self.reader.data_x))
        self.curve_y.setData(np.array(self.reader.data_y))
        self.curve_z.setData(np.array(self.reader.data_z))

    def closeEvent(self, event):
        """
        窗口关闭事件回调
        """
        # 弹出确认框 (可选，防止误触)
        reply = QMessageBox.question(self, '退出',
                                     "确定要断开连接并退出吗?",
                                     QMessageBox.Yes | QMessageBox.No,
                                     QMessageBox.No)

        if reply == QMessageBox.Yes:
            # 1. 停止串口线程 & 关闭端口
            self.reader.stop()

            # 2. 停止绘图定时器 (防止界面销毁后还在刷图报错)
            if self.timer.isActive():
                self.timer.stop()

            # 3. 接受关闭事件，销毁窗口
            event.accept()
        else:
            # 取消关闭
            event.ignore()


if __name__ == '__main__':
    # 获取可用串口列表并打印 (方便调试)
    ports = list(serial.tools.list_ports.comports())
    print("当前可用串口:")
    for p in ports:
        print(f"- {p.device}: {p.description}")

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())