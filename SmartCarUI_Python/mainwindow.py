"""
SmartCarUI 主窗口 — PyQt5 版
NPU 模型推理 + OpenCV 摄像头 + 行为警报弹窗

对应 C++ 版 SmartCarUI/mainwindow.h/.cpp
"""

import os
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from PyQt5.QtCore import (
    QDateTime, QDir, QTimer,
    Qt,
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QMessageBox,
)
from PyQt5.uic import loadUi

from behaviordialog import BehaviorDialog, RiskLevel, SpeechEngine
from model_runner import ModelRunner


# ============================================================
# 常量
# ============================================================
CAMERA_DEVICE_MAIN = "/dev/video10"
CAMERA_DEVICE_BACKUP = "/dev/video11"


# 模型推理间隔（每 N 帧推理一次）
MODEL_INFERENCE_INTERVAL = 3
# 抓拍冷却（ms）
CAPTURE_COOLDOWN_MS = 10000

# 驾驶员语音冷却（ms）
DRIVER_ALERT_COOLDOWN_MS = 15000


# (SensorData / RoadResult 已移除 — MPU6050 禁用)


# ============================================================
# 主窗口
# ============================================================
class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        # ---- 加载 UI ----
        ui_path = Path(__file__).parent / "mainwindow.ui"
        loadUi(str(ui_path), self)

        self._app_start_ms = QDateTime.currentMSecsSinceEpoch()

        # ---- 硬件状态 ----
        self._camera_ok = False
        # self._sensor_ok = False  # MPU6050 已禁用
        self._speaker_ok = False
        self._voice_busy = False
        self._speech_playing = False
        self._face_cascade_loaded = False

        # ---- 计时起点 ----
        self._driver_start_ms = 0
        self._last_driver_seen_ms = 0
        self._no_driver_start_ms = 0
        self._last_capture_ms = 0
        self._last_driver_alert_ms = 0
        self._last_road_alert_ms = 0

        # ---- 风险状态 ----
        self._current_driver_risk = RiskLevel.Normal
        self._current_road_risk = RiskLevel.Normal
        self._last_driver_alert_risk = RiskLevel.Normal
        self._voice_sequence_id = 0

        # ---- OpenCV ----
        self._cap = cv2.VideoCapture()
        self._face_cascade = cv2.CascadeClassifier()
        self._camera_frame_counter = 0
        self._camera_error_counter = 0
        self._cached_face_detected = False
        self._cached_faces = []

        # ---- 加速度 ----
        self._last_acc_magnitude = -1.0

        # ---- 模型推理 ----
        self._model = ModelRunner()
        self._last_model_behavior = ""
        self._last_model_result = None  # (ccode, class_name, confidence)

        # ----------------------------------------------------------
        # 初始化
        # ----------------------------------------------------------
        self._init_speaker()
        self._speaker_ok = True

        self._speech_engine = SpeechEngine()

        self._init_ui_state()
        self._load_face_cascade()
        self._init_camera()

        # MPU6050 六轴传感器已禁用

        self._update_hardware_status_bar("系统启动完成")

        # ---- 定时器 ----
        self._cam_timer = QTimer(self)
        self._cam_timer.timeout.connect(self._update_camera_frame)
        self._cam_timer.start(40)  # ~25fps

        # MPU6050 定时器已禁用

        self._stat_timer = QTimer(self)
        self._stat_timer.timeout.connect(self._update_status)
        self._stat_timer.start(2000)

    # ============================================================
    # 析构
    # ============================================================
    def closeEvent(self, event):
        if self._cap and self._cap.isOpened():
            self._cap.release()
        self._speech_engine.shutdown()
        super().closeEvent(event)

    # ============================================================
    # UI 初始化
    # ============================================================
    def _init_ui_state(self):
        self.lblCameraView.setText("正在打开摄像头 /dev/video10 ...")
        self.lblCameraView.setAlignment(Qt.AlignCenter)
        self.lblCameraView.setScaledContents(True)

        self.lblCaptureImg.setText("暂无风险抓拍")
        self.lblCaptureImg.setAlignment(Qt.AlignCenter)
        self.lblCaptureImg.setScaledContents(True)

        self.lblDriverStatus.setText("驾驶员状态：等待摄像头画面")
        self.lblDriverStatus.setWordWrap(True)
        self.lblDriverStatus.setGeometry(580, 175, 281, 95)

        self.lblAttitude_3.setWordWrap(True)
        self.lblStatusBar.setWordWrap(True)
        self.lblDriveTime.setText("连续驾驶时长：0小时0分钟0秒")

        self.lblCameraView.lower()
        self.lblCaptureImg.raise_()
        self.lblDriverStatus.raise_()

        self._apply_driver_risk_style(RiskLevel.Normal)
        self._apply_road_risk_style(RiskLevel.Normal)

    # ============================================================
    # 扬声器
    # ============================================================
    def _init_speaker(self):
        """开机调大系统音量（Linux）"""
        if sys.platform == "linux":
            try:
                import subprocess
                subprocess.run(
                    ["amixer", "set", "Master", "95%", "unmute"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=2,
                )
            except Exception:
                pass

    # ============================================================
    # 摄像头
    # ============================================================
    def _init_camera(self):
        if self._cap and self._cap.isOpened():
            self._cap.release()

        self._camera_error_counter = 0
        self._camera_frame_counter = 0

        # 尝试打开主设备
        self._camera_ok = self._cap.open(CAMERA_DEVICE_MAIN, cv2.CAP_V4L2)

        if not self._camera_ok:
            self._camera_ok = self._cap.open(CAMERA_DEVICE_BACKUP, cv2.CAP_V4L2)

        if self._camera_ok:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            self._cap.set(cv2.CAP_PROP_FPS, 30)
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y', 'U', 'Y', 'V'))
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

            self._camera_ok = self._cap.isOpened()
            self.lblCameraView.setText("摄像头已打开，正在加载画面...")
        else:
            self._camera_ok = False
            self.lblCameraView.setText("摄像头打开失败\n请确认 /dev/video10 未被占用")

    def _reopen_camera(self):
        if self._cap and self._cap.isOpened():
            self._cap.release()

        self._camera_ok = False
        self.lblCameraView.setText("摄像头画面异常，正在重新打开...")
        QTimer.singleShot(500, self._init_camera)

    def _load_face_cascade(self):
        cascade_paths = [
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/local/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        ]
        for path in cascade_paths:
            if os.path.exists(path):
                self._face_cascade = cv2.CascadeClassifier()
                if self._face_cascade.load(path):
                    self._face_cascade_loaded = True
                    print(f"Face cascade loaded: {path}")
                    return

        self._face_cascade_loaded = False
        print("Face cascade not found. Using demo fallback.")

    # ============================================================
    # 摄像头帧更新
    # ============================================================
    def _update_camera_frame(self):
        if not self._cap or not self._cap.isOpened():
            self._camera_ok = False
            return

        ret, frame = self._cap.read()
        if not ret or frame is None or frame.size == 0:
            self._camera_error_counter += 1
            self._camera_ok = False

            if self._camera_error_counter >= 10:
                self._reopen_camera()
            else:
                self.lblCameraView.setText("摄像头暂时无画面，正在等待下一帧...")
            return

        self._camera_ok = True
        self._camera_error_counter = 0
        self._camera_frame_counter += 1

        # 每 MODEL_INFERENCE_INTERVAL 帧运行模型推理
        if (self._model.is_ready
                and self._camera_frame_counter % MODEL_INFERENCE_INTERVAL == 0):
            self._run_model_inference(frame)

        # 人脸检测（每 5 帧一次）
        faces = []
        face_detected = False

        if self._face_cascade_loaded:
            if self._camera_frame_counter % 5 == 1:
                try:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    scale = 0.5
                    small_gray = cv2.resize(gray, None, fx=scale, fy=scale,
                                            interpolation=cv2.INTER_LINEAR)
                    small_gray = cv2.equalizeHist(small_gray)

                    small_faces = self._face_cascade.detectMultiScale(
                        small_gray, 1.1, 3, 0, (25, 25)
                    )

                    self._cached_faces = []
                    for (x, y, w, h) in small_faces:
                        self._cached_faces.append((
                            int(x / scale),
                            int(y / scale),
                            int(w / scale),
                            int(h / scale),
                        ))
                    self._cached_face_detected = len(self._cached_faces) > 0
                except cv2.error:
                    self._cached_faces = []
                    self._cached_face_detected = False

            faces = self._cached_faces
            face_detected = self._cached_face_detected
        else:
            # 无人脸模型时的演示兜底
            face_detected = True

        self._update_driver_presence(face_detected)
        self._update_driver_analysis(face_detected, frame, faces)

        # 显示画面（画人脸框）
        try:
            show_frame = frame.copy()
            for (x, y, w, h) in faces:
                cv2.rectangle(show_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            rgb = cv2.cvtColor(show_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            img = QImage(rgb.data, w, h, w * ch, QImage.Format_RGB888)
            pix = QPixmap.fromImage(img).scaled(
                self.lblCameraView.size(),
                Qt.KeepAspectRatio,
                Qt.FastTransformation,
            )
            self.lblCameraView.setPixmap(pix)
        except cv2.error:
            pass

    # [MPU6050 全部代码已移除]
            self._last_road_alert_ms = now
            warning = ("检测到高风险路况，请立即减速慢行"
                       if road.risk_level == RiskLevel.High
                       else "检测到路况异常，请注意驾驶安全")
            self._speech_engine.speak(warning, "road_alert")
            self._set_status_text(f"路况提醒：{warning}")

    # ============================================================
    # 驾驶员状态
    # ============================================================
    def _update_driver_presence(self, face_detected: bool):
        now = QDateTime.currentMSecsSinceEpoch()

        if face_detected:
            self._last_driver_seen_ms = now
            self._no_driver_start_ms = 0
            if self._driver_start_ms == 0:
                self._driver_start_ms = now
        else:
            if self._no_driver_start_ms == 0:
                self._no_driver_start_ms = now

            if now - self._no_driver_start_ms >= 10000:
                self._driver_start_ms = 0
                self.lblDriveTime.setText("连续驾驶时长：0小时0分钟0秒")
                return

        if self._driver_start_ms > 0:
            elapsed_sec = (now - self._driver_start_ms) // 1000
            self.lblDriveTime.setText(
                f"连续驾驶时长：{self._format_duration(elapsed_sec)}")

    def _update_driver_analysis(self, face_detected: bool, frame, faces):
        now = QDateTime.currentMSecsSinceEpoch()
        risk = RiskLevel.Normal
        status_text = ""
        alert_text = ""
        capture_reason = ""
        need_capture = False

        if not face_detected:
            lost_sec = ((now - self._no_driver_start_ms) // 1000
                        if self._no_driver_start_ms > 0 else 0)

            if lost_sec >= 10:
                risk = RiskLevel.High
                status_text = ("驾驶员状态：未检测到驾驶员\n"
                               "疲劳程度：未知\n"
                               "分心情况：驾驶员离开超过10秒\n"
                               "风险等级：高风险")
                alert_text = "未检测到驾驶员，请确认驾驶安全"
                capture_reason = "驾驶员离开"
                need_capture = True
            elif lost_sec >= 3:
                risk = RiskLevel.Medium
                status_text = (f"驾驶员状态：分心疑似\n"
                               f"疲劳程度：轻微\n"
                               f"分心情况：视线离开 {lost_sec} 秒\n"
                               f"风险等级：中风险")
                alert_text = "请保持注意力，注意前方道路"
                capture_reason = "分心疑似"
                need_capture = True
            else:
                risk = RiskLevel.Low
                status_text = ("驾驶员状态：短暂视线偏移\n"
                               "疲劳程度：正常\n"
                               "分心情况：轻微注意力偏移\n"
                               "风险等级：低风险")
        else:
            risk = RiskLevel.Normal
            status_text = "驾驶员状态：正常"

        self._current_driver_risk = risk
        self.lblDriverStatus.setText(status_text)
        self._apply_driver_risk_style(risk)

        if need_capture and now - self._last_capture_ms > CAPTURE_COOLDOWN_MS:
            self._capture_and_show(frame, capture_reason)
            self._last_capture_ms = now

        if risk >= RiskLevel.Medium and alert_text:
            self._maybe_voice_alert(risk, alert_text)

    def _capture_and_show(self, frame, reason: str):
        if frame is None or frame.size == 0:
            return

        try:
            captures_dir = QDir(QDir.currentPath() + "/captures")
            if not captures_dir.exists():
                captures_dir.mkpath(".")

            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            file_path = captures_dir.filePath(f"capture_{ts}.jpg")

            cv2.imwrite(file_path, frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 90])

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            img = QImage(rgb.data, w, h, w * ch, QImage.Format_RGB888)
            pix = QPixmap.fromImage(img).scaled(
                self.lblCaptureImg.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
            self.lblCaptureImg.setPixmap(pix)
            self.lblCaptureImg.setToolTip(f"{reason}\n{file_path}")

            if not self._voice_busy:
                self._set_status_text(
                    f"已抓拍风险画面：{reason}，图片已保存到 captures 目录")
        except cv2.error:
            if not self._voice_busy:
                self._set_status_text("抓拍失败：摄像头帧异常，已自动跳过")

    def _maybe_voice_alert(self, risk_level: RiskLevel, message: str):
        now = QDateTime.currentMSecsSinceEpoch()

        if self._voice_busy:
            return

        if (now - self._last_driver_alert_ms < DRIVER_ALERT_COOLDOWN_MS
                and risk_level <= self._last_driver_alert_risk):
            return

        self._last_driver_alert_ms = now
        self._last_driver_alert_risk = risk_level

        if risk_level == RiskLevel.High:
            QApplication.beep()

        self._speech_engine.speak(message, "driver_alert")
        self._set_status_text(f"驾驶员风险提醒：{message}")

    # ============================================================
    # UI 按钮
    # ============================================================
    def on_btnVoice_clicked(self):
        """语音交互按钮"""
        if self._voice_busy:
            self._voice_busy = False
            self._voice_sequence_id += 1
            self._clear_speech_queue()
            self._speak_and_status("语音识别已取消", "语音识别已取消")
            self.btnVoice.setEnabled(True)
            self.btnVoice.setText("语音交互")
            return

        self._voice_busy = True
        self._voice_sequence_id += 1
        seq = self._voice_sequence_id

        self._clear_speech_queue()

        self.btnVoice.setEnabled(True)
        self.btnVoice.setText("取消语音")

        self._speak_and_status("语音开始识别，请说出您的指令",
                               "语音正在识别中...")

        QTimer.singleShot(10000, self, lambda: self._voice_seq_step(
            seq, "识别完成，正在为您执行指令", "识别完成，正在为您执行指令..."))
        QTimer.singleShot(17000, self, lambda: self._voice_seq_step(
            seq, "已为您打开流行音乐", "已为您打开：流行音乐"))
        QTimer.singleShot(24000, self, lambda: self._voice_seq_step(
            seq, "空调已调节至25摄氏度", "空调已调节至25摄氏度"))
        QTimer.singleShot(31000, self, lambda: self._voice_seq_step(
            seq, "座椅已调整至中档位置", "座椅已调整至中档位置"))
        QTimer.singleShot(38000, self, lambda: self._voice_seq_step(
            seq, "车内氛围灯已自动开启", "语音指令执行完成：车内氛围灯已自动开启",
            finalize=True))

    def _voice_seq_step(self, seq: int, speech: str, status: str,
                        finalize: bool = False):
        if not self._voice_busy or seq != self._voice_sequence_id:
            return
        self._speak_and_status(speech, status)
        if finalize:
            self._voice_busy = False
            self.btnVoice.setEnabled(True)
            self.btnVoice.setText("语音交互")

    def on_btnAiSet_clicked(self):
        """AI 设置按钮"""
        text = ("AI 已学习驾驶员习惯，当前为固定模拟结果：\n\n"
                "1. 音乐偏好：上车后优先播放流行音乐。\n"
                "2. 空调习惯：车内温度自动调节至 25 摄氏度。\n"
                "3. 座椅习惯：座椅调整至中档位置。\n"
                "4. 驾驶习惯：转弯前提前降低车速，偏好平稳驾驶。\n"
                "5. 安全策略：检测到分心、疲劳或高风险路况时自动语音提醒。")
        QMessageBox.information(self, "AI 智能设置", text)
        self._set_status_text("AI智能设置：已加载驾驶员习惯模拟模型")

    # ============================================================
    # 状态更新
    # ============================================================
    def _update_status(self):
        if not self._voice_busy and not self._speech_engine.busy():
            self._update_hardware_status_bar()

    def _update_hardware_status_bar(self, prefix: str = ""):
        self._speaker_ok = True  # SpeechEngine handles TTS

        camera_text = ("摄像头已打开正常工作" if self._camera_ok
                       else "摄像头未正常打开")
        speaker_text = ("语音模块已就绪" if self._speaker_ok
                        else "语音模块未就绪")

        bar = f"系统状态 | {camera_text} | {speaker_text}"
        if prefix:
            bar = f"{prefix} | {bar}"

        self.lblStatusBar.setText(bar)

        if not self._camera_ok or not self._speaker_ok:
            self.lblStatusBar.setStyleSheet(
                "color: #ffc107; font-size: 12px; padding: 4px;")
        else:
            self.lblStatusBar.setStyleSheet(
                "color: #22c55e; font-size: 12px; padding: 4px;")

    def _set_status_text(self, text: str):
        self.lblStatusBar.setText(text)
        self.lblStatusBar.setStyleSheet(
            "color: #22c55e; font-size: 12px; padding: 4px;")

    # ============================================================
    # 语音
    # ============================================================
    def _speak_and_status(self, speech_text: str, status_text: str,
                          force: bool = False):
        self._speech_engine.speak(speech_text)
        self._set_status_text(status_text)

    def _clear_speech_queue(self, stop_current: bool = False):
        self._speech_engine.clear()

    # ============================================================
    # 样式
    # ============================================================
    def _apply_driver_risk_style(self, risk_level: RiskLevel):
        if risk_level == RiskLevel.High:
            color = "#f44336"
        elif risk_level == RiskLevel.Medium:
            color = "#ffc107"
        elif risk_level == RiskLevel.Low:
            color = "#ffc107"
        else:
            color = "#22c55e"

        self.lblDriverStatus.setStyleSheet(
            f"color: {color}; font-size: 10px; background-color: "
            f"rgba(0,0,0,100); padding: 4px; border-radius: 4px;"
        )

    def _apply_road_risk_style(self, risk_level: RiskLevel):
        if risk_level == RiskLevel.High:
            color = "#f44336"
        elif risk_level >= RiskLevel.Medium:
            color = "#ffc107"
        elif risk_level == RiskLevel.Low:
            color = "#ffc107"
        else:
            color = "#22c55e"

        for name in ("lblCarStatus", "lblRoadMain", "lblShakeLevel",
                     "lblSlopeStatus", "lblRoadRisk"):
            widget = getattr(self, name, None)
            if widget:
                widget.setStyleSheet(f"color: {color}; font-size: 12px;")

    @staticmethod
    def _format_duration(seconds: int) -> str:
        h = seconds // 3600
        m = (seconds % 3600) // 60
        s = seconds % 60
        return f"{h}小时{m}分钟{s}秒"

    @staticmethod
    def _risk_name(risk_level: RiskLevel) -> str:
        return ["正常", "低风险", "中风险", "高风险"][risk_level]

    # ============================================================
    # 模型推理
    # ============================================================
    def _run_model_inference(self, frame):
        """运行模型推理 + 弹窗 + 状态更新 + 抓拍"""
        if frame is None or frame.size == 0:
            return

        try:
            ccode, full_name, confidence = self._model.infer(frame)
        except Exception as e:
            print(f"[模型推理] 错误: {e}")
            return

        self._last_model_result = (ccode, full_name, confidence)

        # 非 C1_Drive_Safe 行为才弹窗
        if ccode == "C1":
            return

        risk_level = BehaviorDialog.behavior_to_risk(ccode)
        message = BehaviorDialog.behavior_to_text(ccode)

        # 更新驱动状态 UI
        self._current_driver_risk = risk_level
        self.lblDriverStatus.setText(
            f"驾驶员状态：{message}\n"
            f"风险等级：{self._risk_name(risk_level)}"
        )
        self._apply_driver_risk_style(risk_level)

        # 弹窗（语音走 SpeechEngine 内置冷却）
        BehaviorDialog.show_alert(risk_level, message, ccode)

        # 风险行为 → 抓拍
        if self._model.is_risk_behavior(ccode):
            now = QDateTime.currentMSecsSinceEpoch()
            if now - self._last_capture_ms > CAPTURE_COOLDOWN_MS:
                self._capture_and_show(frame, message)
                self._last_capture_ms = now
