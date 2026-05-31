"""
行为风险等级映射 + 自定义弹窗 (BehaviorDialog)
语音引擎 (SpeechEngine) — espeak-ng 子进程 + 内置冷却
"""

import sys
import time
import threading
import subprocess
from collections import deque
from enum import IntEnum

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QDialog, QLabel, QVBoxLayout


# ============================================================
# 风险等级枚举
# ============================================================
class RiskLevel(IntEnum):
    Normal = 0
    Low = 1
    Medium = 2
    High = 3


# ============================================================
# behavior → RiskLevel 映射表（22 类识别码）
# ============================================================
BEHAVIOR_RISK_MAP = {
    "c1": RiskLevel.Normal, "drive_safe": RiskLevel.Normal, "safe_driving": RiskLevel.Normal,
    "c8": RiskLevel.Medium, "make_up": RiskLevel.Medium,
    "c9": RiskLevel.Low, "look_left": RiskLevel.Low,
    "c10": RiskLevel.Low, "look_right": RiskLevel.Low,
    "c11": RiskLevel.Low, "look_up": RiskLevel.Low,
    "c12": RiskLevel.Low, "look_down": RiskLevel.Low,
    "c13": RiskLevel.Medium, "smoke_left": RiskLevel.Medium,
    "c14": RiskLevel.Medium, "smoke_right": RiskLevel.Medium,
    "c15": RiskLevel.Low, "smoke_mouth": RiskLevel.Low,
    "c22": RiskLevel.Low, "talk_to_passenger": RiskLevel.Low,
    "distracted": RiskLevel.Low, "looking_away": RiskLevel.Low,
    "c4": RiskLevel.Medium, "talk_left": RiskLevel.Medium,
    "c5": RiskLevel.Medium, "talk_right": RiskLevel.Medium,
    "c6": RiskLevel.Medium, "text_left": RiskLevel.Medium,
    "c7": RiskLevel.Medium, "text_right": RiskLevel.Medium,
    "c16": RiskLevel.Medium, "eat_left": RiskLevel.Medium,
    "c17": RiskLevel.Medium, "eat_right": RiskLevel.Medium,
    "c18": RiskLevel.Medium, "operate_radio": RiskLevel.Medium,
    "c19": RiskLevel.Medium, "operate_gps": RiskLevel.Medium,
    "c20": RiskLevel.Medium, "reach_behind": RiskLevel.Medium,
    "phone_using": RiskLevel.Medium, "phone_call": RiskLevel.Medium,
    "drinking": RiskLevel.Medium, "eating": RiskLevel.Medium,
    "c2": RiskLevel.High, "sleep": RiskLevel.High,
    "c3": RiskLevel.High, "yawning": RiskLevel.High,
    "c21": RiskLevel.High, "leave_steering_wheel": RiskLevel.High,
    "eyes_closed": RiskLevel.High, "no_driver": RiskLevel.High,
    "no_face": RiskLevel.High, "tired": RiskLevel.High,
}

BEHAVIOR_TEXT_MAP = {
    "c1": "驾驶状态：正常", "drive_safe": "驾驶状态：正常", "safe_driving": "驾驶状态：正常",
    "c2": "危险：检测到疲劳驾驶", "sleep": "危险：检测到疲劳驾驶",
    "c3": "危险：检测到打哈欠", "yawning": "危险：检测到打哈欠",
    "c4": "警告：左侧打电话", "talk_left": "警告：左侧打电话",
    "c5": "警告：右侧打电话", "talk_right": "警告：右侧打电话",
    "c6": "警告：左侧看手机", "text_left": "警告：左侧看手机",
    "c7": "警告：右侧看手机", "text_right": "警告：右侧看手机",
    "c8": "警告：驾驶中请勿化妆", "make_up": "警告：驾驶中请勿化妆",
    "c9": "注意：视线向左偏移", "look_left": "注意：视线向左偏移",
    "c10": "注意：视线向右偏移", "look_right": "注意：视线向右偏移",
    "c11": "注意：视线向上偏移", "look_up": "注意：视线向上偏移",
    "c12": "注意：视线向下偏移", "look_down": "注意：视线向下偏移",
    "c13": "警告：驾驶中请勿吸烟", "smoke_left": "警告：驾驶中请勿吸烟",
    "c14": "警告：驾驶中请勿吸烟", "smoke_right": "警告：驾驶中请勿吸烟",
    "c15": "提示：驾驶中请勿吸烟", "smoke_mouth": "提示：驾驶中请勿吸烟",
    "c16": "警告：驾驶中请勿饮食", "eat_left": "警告：驾驶中请勿饮食",
    "c17": "警告：驾驶中请勿饮食", "eat_right": "警告：驾驶中请勿饮食",
    "c18": "警告：操作中控设备", "operate_radio": "警告：操作中控设备",
    "c19": "警告：操作导航设备", "operate_gps": "警告：操作导航设备",
    "c20": "警告：向后伸手取物", "reach_behind": "警告：向后伸手取物",
    "c21": "危险：双手离开方向盘", "leave_steering_wheel": "危险：双手离开方向盘",
    "c22": "提示：请勿与乘客交谈", "talk_to_passenger": "提示：请勿与乘客交谈",
}


# ============================================================
# SpeechEngine — 语音引擎（espeak-ng 子进程 + 内置冷却）
# ============================================================
class SpeechEngine:
    """单例语音引擎。espeak-ng 子进程，后台队列播报，内置全局+行为冷却"""

    _instance = None
    _lock = threading.Lock()

    GLOBAL_COOLDOWN = 3.0      # 任意两次播报最少间隔 3 秒
    BEHAVIOR_COOLDOWN = 5.0    # 同行为不重复播报 5 秒

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True

        self._queue = deque()
        self._is_speaking = False
        self._last_speech_time = 0.0
        self._behavior_times = {}
        self._stop = threading.Event()

        self._thread = threading.Thread(target=self._process_queue, daemon=True)
        self._thread.start()

        print("[语音] espeak-ng 就绪")
        # 预检查 espeak-ng 是否可用
        try:
            subprocess.run(["espeak-ng", "--version"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=2)
        except Exception:
            print("[语音] ⚠️ espeak-ng 未安装，语音播报不可用")

    # ----------------------------------------------------------
    # 公开接口
    # ----------------------------------------------------------
    def speak(self, text: str, behavior_key: str = "") -> bool:
        """
        请求播报。返回 True=已入队，False=被冷却拒绝。
        behavior_key 传 "c2"/"c3"/等可实现同行为冷却。
        """
        now = time.time()

        if now - self._last_speech_time < self.GLOBAL_COOLDOWN:
            return False

        if behavior_key:
            last = self._behavior_times.get(behavior_key, 0.0)
            if now - last < self.BEHAVIOR_COOLDOWN:
                return False
            self._behavior_times[behavior_key] = now

        self._queue.append(text)
        return True

    def clear(self):
        self._queue.clear()

    def busy(self) -> bool:
        return self._is_speaking or len(self._queue) > 0

    def shutdown(self):
        self._stop.set()
        self.clear()

    # ----------------------------------------------------------
    # 内部
    # ----------------------------------------------------------
    def _process_queue(self):
        while not self._stop.is_set():
            if self._queue:
                text = self._queue.popleft()
                self._is_speaking = True
                self._do_speak(text)
                self._last_speech_time = time.time()
                self._is_speaking = False
            else:
                time.sleep(0.1)

    @staticmethod
    def _do_speak(text: str):
        """调用 espeak-ng 播报（同步阻塞，在后台线程执行）"""
        try:
            subprocess.run(
                ["espeak-ng", "-v", "zh", "-s", "150", text],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=30,
            )
        except Exception:
            pass


# ============================================================
# BehaviorDialog — 自定义非模态弹窗
# ============================================================
class BehaviorDialog(QDialog):
    """深色主题 + 彩色边框非模态弹窗，4 秒自动关闭"""

    def __init__(self, risk_level: RiskLevel, message: str, parent=None):
        super().__init__(parent)
        self.setWindowTitle("行为警报")
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.setAttribute(Qt.WA_ShowWithoutActivating)
        self.setModal(False)
        self.setFixedSize(380, 160)

        if risk_level == RiskLevel.High:
            bg_color, border_color, text_color = "#1A0A0A", "#f44336", "#f44336"
        else:
            bg_color, border_color, text_color = "#1A1A0A", "#ffc107", "#ffc107"

        self.setStyleSheet(
            f"BehaviorDialog {{background-color: {bg_color}; border: 3px solid {border_color}; border-radius: 12px;}}"
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        self._label = QLabel(message, self)
        self._label.setStyleSheet(f"color: {text_color}; font-size: 20px; font-weight: bold; padding: 10px;")
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setWordWrap(True)
        layout.addWidget(self._label)

        QTimer.singleShot(4000, self.close)

    # ----------------------------------------------------------
    # 静态方法
    # ----------------------------------------------------------
    _cooldown_map = {}
    _speech_engine = None

    @classmethod
    def speech_engine(cls) -> SpeechEngine:
        if cls._speech_engine is None:
            cls._speech_engine = SpeechEngine()
        return cls._speech_engine

    @classmethod
    def show_alert(cls, risk_level: RiskLevel, message: str,
                   behavior_key: str = "") -> bool:
        """统一入口：Normal/Low 不弹窗；Medium/High 弹窗 + 语音 + 蜂鸣"""
        if risk_level <= RiskLevel.Low:
            return False

        if behavior_key and not cls._check_cooldown(behavior_key):
            return False

        dlg = cls(risk_level, message)
        dlg.show()

        screen = QApplication.primaryScreen()
        if screen:
            center = screen.geometry().center()
            dlg.move(center.x() - dlg.width() // 2, center.y() - dlg.height() // 2)

        if risk_level >= RiskLevel.Medium:
            cls.speech_engine().speak(message, behavior_key)
        if risk_level == RiskLevel.High:
            cls._beep()

        return True

    @classmethod
    def behavior_to_risk(cls, behavior: str) -> RiskLevel:
        return BEHAVIOR_RISK_MAP.get(behavior.lower(), RiskLevel.Normal)

    @classmethod
    def behavior_to_text(cls, behavior: str) -> str:
        return BEHAVIOR_TEXT_MAP.get(behavior.lower(), "未知行为")

    @classmethod
    def _check_cooldown(cls, behavior_key: str) -> bool:
        now = time.time() * 1000
        last = cls._cooldown_map.get(behavior_key, 0)
        if now - last < 5000:
            return False
        cls._cooldown_map[behavior_key] = now
        return True

    @staticmethod
    def _beep():
        sys.stderr.write("\a")
        sys.stderr.flush()
