#!/usr/bin/env python3
"""
integrate.py — SmartCarUI 三通道集成脚本

三个 TCP Socket 通道：
  ① 接收帧  (Qt→Python:9996)  — Qt发来的JPEG帧，做推理
  ② 发结果  (Python→Qt:9998)  — 返回JSON识别结果
  ③ 发抓拍  (Python→Qt:9997)  — 非正常行为时推送抓拍图

用法：
  # CPU推理 (ONNX)
  python integrate.py --backend onnx

  # NPU推理 (RK3566)
  python integrate.py --backend rknn

依赖：
  pip install numpy opencv-python onnxruntime
"""

import argparse
import json
import os
import socket
import struct
import threading
import time
from collections import deque

import cv2
import numpy as np

# ============================================================
# 22 类行为定义
# ============================================================
CLASSES = [
    'C1_Drive_Safe', 'C2_Sleep', 'C3_Yawning', 'C4_Talk_Left',
    'C5_Talk_Right', 'C6_Text_Left', 'C7_Text_Right', 'C8_Make_Up',
    'C9_Look_Left', 'C10_Look_Right', 'C11_Look_Up', 'C12_Look_Down',
    'C13_Smoke_Left', 'C14_Smoke_Right', 'C15_Smoke_Mouth', 'C16_Eat_Left',
    'C17_Eat_Right', 'C18_Operate_Radio', 'C19_Operate_GPS', 'C20_Reach_Behind',
    'C21_Leave_Steering_Wheel', 'C22_Talk_to_Passenger'
]

# C编码 → 显示文字 (与 Qt 端 BEHAVIOR_TEXT_MAP 保持一致)
CCODE_TEXT = {
    "C1":  "驾驶状态：正常",
    "C2":  "危险：检测到疲劳驾驶",
    "C3":  "危险：检测到打哈欠",
    "C4":  "警告：左侧打电话",
    "C5":  "警告：右侧打电话",
    "C6":  "警告：左侧看手机",
    "C7":  "警告：右侧看手机",
    "C8":  "提示：驾驶中请勿化妆",
    "C9":  "注意：视线向左偏移",
    "C10": "注意：视线向右偏移",
    "C11": "注意：视线向上偏移",
    "C12": "注意：视线向下偏移",
    "C13": "提示：驾驶中请勿吸烟",
    "C14": "提示：驾驶中请勿吸烟",
    "C15": "提示：驾驶中请勿吸烟",
    "C16": "警告：驾驶中请勿饮食",
    "C17": "警告：驾驶中请勿饮食",
    "C18": "警告：操作中控设备",
    "C19": "警告：操作导航设备",
    "C20": "警告：向后伸手取物",
    "C21": "危险：双手离开方向盘",
    "C22": "提示：请勿与乘客交谈",
}

# 非 C1 的都是非正常行为（需要抓拍）
RISK_CODES = {f"C{i}": True for i in range(2, 23)}

# ============================================================
# 模型推理 (支持 ONNX / RKNN)
# ============================================================
class ModelRunner:
    """封装模型推理，支持 ONNX(CPU) 和 RKNN(NPU) 两种后端"""

    def __init__(self, backend: str):
        self.backend = backend
        self.session = None
        self.input_name = None

        if backend == "onnx":
            self._init_onnx()
        elif backend == "rknn":
            self._init_rknn()
        else:
            raise ValueError(f"Unknown backend: {backend}")

    def _init_onnx(self):
        import onnxruntime as ort
        model_path = os.path.join(os.path.dirname(__file__), "mobilevit_driver.onnx")
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"ONNX 模型不存在: {model_path}")
        self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        # 从模型输入形状自动获取尺寸: [1,3,H,W]
        input_shape = self.session.get_inputs()[0].shape
        self.input_size = input_shape[2]  # H, 如 224 或 256
        print(f"[模型] ONNX 加载成功: {model_path}  输入: {input_shape[2]}x{input_shape[3]}")

    def _init_rknn(self):
        from rknnlite.api import RKNNLite
        model_path = os.path.join(os.path.dirname(__file__), "mobilevit_driver_RK3566_256x256.rknn")
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"RKNN 模型不存在: {model_path}")
        self.rknn = RKNNLite()
        self.rknn.load_rknn(model_path)
        self.rknn.init_runtime()
        print(f"[模型] RKNN 加载成功: {model_path}")

    def preprocess(self, frame: np.ndarray) -> np.ndarray:
        """预处理：BGR→RGB→Resize 模型输入尺寸→Normalize→NCHW"""
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        size = getattr(self, 'input_size', 256)
        img = cv2.resize(img, (size, size))
        img = img.astype(np.float32) / 255.0
        img = (img - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
        img = np.transpose(img, (2, 0, 1))   # HWC→CHW
        img = np.expand_dims(img, axis=0)     # →NCHW
        return img.astype(np.float32)

    def infer(self, frame: np.ndarray) -> tuple:
        """
        推理一帧，返回 (ccode, class_name, confidence)
        ccode:   "C1" ~ "C22"
        class_name: 完整类名如 "C8_Make_Up"
        confidence:  置信度 0~1
        """
        input_tensor = self.preprocess(frame)

        if self.backend == "onnx":
            outputs = self.session.run(None, {self.input_name: input_tensor})
        else:  # rknn
            outputs = self.rknn.inference(inputs=[input_tensor])

        logits = outputs[0][0]
        # Softmax 把 logits 转成 0~1 概率
        exp_logits = np.exp(logits - np.max(logits))  # 减最大值防溢出
        probs = exp_logits / np.sum(exp_logits)
        pred_idx = int(np.argmax(probs))
        confidence = float(probs[pred_idx])
        full_name = CLASSES[pred_idx]
        ccode = full_name.split("_")[0]  # "C8_Make_Up" → "C8"

        return ccode, full_name, confidence


# ============================================================
# Socket 通信
# ============================================================
class FrameReceiver:
    """通道①: 接收 Qt 发来的 JPEG 帧"""

    def __init__(self, host: str, port: int, model: ModelRunner,
                 json_host: str, json_port: int,
                 capture_host: str, capture_port: int):
        self.host = host
        self.port = port
        self.model = model
        self.json_host = json_host
        self.json_port = json_port
        self.capture_host = capture_host
        self.capture_port = capture_port

        self._stop = threading.Event()
        self._json_sock = None
        self._capture_sock = None
        self._lock = threading.Lock()

        # 最近行为（用于冷却）
        self._last_behavior = ""
        self._last_behavior_time = 0.0

    def start(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)
        print(f"[通道①] 帧接收服务端启动: {self.host}:{self.port}")

        # 连接 Qt 的 JSON 和抓拍端口
        self._connect_json()
        self._connect_capture()

        server.settimeout(1.0)
        while not self._stop.is_set():
            try:
                conn, addr = server.accept()
                print(f"[通道①] Qt 已连接: {addr}")
                self._handle_client(conn)
                conn.close()
                print(f"[通道①] Qt 已断开: {addr}")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[通道①] 错误: {e}")

        server.close()

    def stop(self):
        self._stop.set()

    def _connect_json(self):
        """连接 Qt:9998 (发JSON结果)"""
        for attempt in range(10):
            try:
                self._json_sock = socket.create_connection(
                    (self.json_host, self.json_port), timeout=3)
                print(f"[通道②] JSON 已连接 Qt:{self.json_port}")
                return
            except Exception as e:
                print(f"[通道②] 连接失败(重试{attempt+1}): {e}")
                time.sleep(2)
        print("[通道②] 警告: 无法连接 Qt JSON 端口，请确认 SmartCarUI 已启动")

    def _connect_capture(self):
        """连接 Qt:9997 (发抓拍图)"""
        for attempt in range(10):
            try:
                self._capture_sock = socket.create_connection(
                    (self.capture_host, self.capture_port), timeout=3)
                print(f"[通道③] 抓拍已连接 Qt:{self.capture_port}")
                return
            except Exception as e:
                print(f"[通道③] 连接失败(重试{attempt+1}): {e}")
                time.sleep(2)
        print("[通道③] 警告: 无法连接 Qt 抓拍端口")

    def _send_json(self, ccode: str, message: str, confidence: float = 0.0):
        """发送 JSON 结果到 Qt:9998"""
        if not self._json_sock:
            return
        data = {
            "time": time.strftime("%H:%M:%S"),
            "behavior": ccode,
            "message": message,
            "confidence": round(confidence, 4),
        }
        try:
            with self._lock:
                self._json_sock.sendall(
                    (json.dumps(data, ensure_ascii=False) + "\n").encode("utf-8"))
        except Exception as e:
            print(f"[通道②] 发送失败: {e}")
            self._json_sock = None

    def _send_capture(self, frame: np.ndarray):
        """发送抓拍 JPEG 到 Qt:9997"""
        if not self._capture_sock:
            return
        try:
            ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ret:
                return
            payload = jpeg.tobytes()
            header = struct.pack("!I", len(payload))
            with self._lock:
                self._capture_sock.sendall(header + payload)
        except Exception as e:
            print(f"[通道③] 发送抓拍失败: {e}")

    def _handle_client(self, conn: socket.socket):
        """处理来自 Qt 的帧流"""
        buf = bytearray()
        reading_header = True
        expected_payload = 0

        while not self._stop.is_set():
            try:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                buf.extend(chunk)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[接收] 读取错误: {e}")
                break

            # 拆帧循环
            while True:
                if reading_header:
                    if len(buf) < 4:
                        break
                    expected_payload = struct.unpack("!I", bytes(buf[:4]))[0]
                    buf = buf[4:]
                    reading_header = False
                    if expected_payload > 5 * 1024 * 1024:
                        buf.clear()
                        reading_header = True
                        break

                if len(buf) < expected_payload:
                    break

                # 完整一帧 → 推理
                jpeg_data = bytes(buf[:expected_payload])
                buf = buf[expected_payload:]
                reading_header = True

                frame = cv2.imdecode(
                    np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    self._process_frame(frame)

    def _process_frame(self, frame: np.ndarray):
        """推理 + 发结果 + 风险时发抓拍"""
        now = time.time()

        # 推理
        try:
            ccode, full_name, confidence = self.model.infer(frame)
        except Exception as e:
            print(f"[推理] 错误: {e}")
            return

        # 置信度过低 → 认为无人或场景模糊
        if confidence < getattr(self, '_conf_threshold', 0.35):
            ccode = "no_driver"
            message = "未检测到驾驶员，请确认驾驶安全"
            print(f"  → 置信度过低({confidence:.3f})，视为无人")
        elif ccode == "C21":
            # 暂时屏蔽 C21_Leave_Steering_Wheel（双手离开方向盘）
            ccode = "C1"
            full_name = "C1_Drive_Safe"
            message = CCODE_TEXT.get("C1", "驾驶状态：正常")
            print(f"  → 已屏蔽 C21，转为 C1_Drive_Safe")
        else:
            message = CCODE_TEXT.get(ccode, full_name)
        print(f"  → {full_name} ({confidence:.3f})")

        # 发送 JSON 结果
        self._send_json(ccode, message, confidence)

        # 非正常行为 + 超过 3 秒冷却 → 发抓拍
        if ccode in RISK_CODES:
            if (ccode != self._last_behavior or
                    now - self._last_behavior_time > 3.0):
                self._send_capture(frame)
                self._last_behavior = ccode
                self._last_behavior_time = now


# ============================================================
# 主程序
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="SmartCarUI 三通道集成脚本")
    parser.add_argument("--backend", choices=["onnx", "rknn"], default="onnx",
                        help="推理后端: onnx(CPU) / rknn(NPU)")
    parser.add_argument("--host", default="0.0.0.0",
                        help="帧接收服务端地址 (Qt连接过来)")
    parser.add_argument("--qt-host", default="127.0.0.1",
                        help="Qt 所在地址 (发结果+抓拍过去)")
    parser.add_argument("--frame-port", type=int, default=9996,
                        help="帧接收端口 (Qt发帧过来)")
    parser.add_argument("--json-port", type=int, default=9998,
                        help="Qt JSON 端口")
    parser.add_argument("--capture-port", type=int, default=9997,
                        help="Qt 抓拍端口")
    parser.add_argument("--confidence", type=float, default=0.35,
                        help="置信度阈值(低于此值视为无人，默认0.35)")
    args = parser.parse_args()

    print("=" * 50)
    print("SmartCarUI 三通道集成脚本")
    print("=" * 50)
    print(f"  推理后端:    {args.backend}")
    print(f"  ① 收帧:      {args.host}:{args.frame_port}")
    print(f"  ② 发JSON:    → {args.qt_host}:{args.json_port}")
    print(f"  ③ 发抓拍:    → {args.qt_host}:{args.capture_port}")
    print("=" * 50)

    # 加载模型
    print("\n加载模型中...")
    model = ModelRunner(args.backend)
    print("模型加载完成!\n")

    # 启动帧接收
    receiver = FrameReceiver(
        host=args.host, port=args.frame_port,
        model=model,
        json_host=args.qt_host, json_port=args.json_port,
        capture_host=args.qt_host, capture_port=args.capture_port,
    )
    receiver._conf_threshold = args.confidence

    try:
        receiver.start()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        receiver.stop()
        print("程序退出")


if __name__ == "__main__":
    main()
