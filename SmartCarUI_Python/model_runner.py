"""
模型推理封装 — 支持 ONNX (CPU) 和 RKNN (NPU) 两种后端
直接集成进 SmartCarUI，替代独立的 integrate.py + Socket 通信
"""

import os
import time

import cv2
import numpy as np


# ============================================================
# 22 类行为定义（与 C++ 端 BEHAVIOR_RISK_MAP 一致）
# ============================================================
CLASSES = [
    "C1_Drive_Safe", "C2_Sleep", "C3_Yawning", "C4_Talk_Left",
    "C5_Talk_Right", "C6_Text_Left", "C7_Text_Right", "C8_Make_Up",
    "C9_Look_Left", "C10_Look_Right", "C11_Look_Up", "C12_Look_Down",
    "C13_Smoke_Left", "C14_Smoke_Right", "C15_Smoke_Mouth", "C16_Eat_Left",
    "C17_Eat_Right", "C18_Operate_Radio", "C19_Operate_GPS", "C20_Reach_Behind",
    "C21_Leave_Steering_Wheel", "C22_Talk_to_Passenger",
]

# C1 正常，C2~C22 为风险行为
RISK_CODES = {f"C{i}" for i in range(2, 23)}


class ModelRunner:
    """模型推理封装，自动选择后端"""

    def __init__(self, model_dir: str = None):
        self.session = None
        self.rknn = None
        self.input_name = None
        self.backend = None

        if model_dir is None:
            model_dir = os.path.dirname(os.path.abspath(__file__))

        # 优先 RKNN (NPU)，其次 ONNX (CPU)
        rknn_path = os.path.join(model_dir, "mobilevit_driver_RK3566_256x256.rknn")
        onnx_path = os.path.join(model_dir, "mobilevit_driver.onnx")

        if os.path.exists(rknn_path):
            self._init_rknn(rknn_path)
        elif os.path.exists(onnx_path):
            self._init_onnx(onnx_path)
        else:
            print("[模型] 未找到模型文件，推理功能不可用")
            print(f"       尝试路径:\n        {rknn_path}\n        {onnx_path}")

    def _init_rknn(self, model_path: str):
        """初始化 RKNN (NPU)"""
        try:
            from rknnlite.api import RKNNLite
            self.rknn = RKNNLite()
            ret = self.rknn.load_rknn(model_path)
            if ret != 0:
                raise RuntimeError(f"加载 RKNN 失败: {ret}")
            ret = self.rknn.init_runtime()
            if ret != 0:
                raise RuntimeError(f"初始化 NPU 失败: {ret}")
            self.backend = "rknn"
            print(f"[模型] ✅ NPU 推理已就绪: {os.path.basename(model_path)}")
        except ImportError:
            print("[模型] rknnlite 未安装，无法使用 NPU")

    def _init_onnx(self, model_path: str):
        """初始化 ONNX (CPU)"""
        try:
            import onnxruntime as ort
            self.session = ort.InferenceSession(
                model_path, providers=["CPUExecutionProvider"])
            self.input_name = self.session.get_inputs()[0].name
            self.backend = "onnx"
            print(f"[模型] ✅ CPU 推理已就绪: {os.path.basename(model_path)}")
        except ImportError:
            print("[模型] onnxruntime 未安装，无法使用 CPU 推理")

    @property
    def is_ready(self) -> bool:
        return self.backend is not None

    # ============================================================
    # 预处理
    # ============================================================
    @staticmethod
    def preprocess(frame: np.ndarray) -> np.ndarray:
        """BGR 帧 → 模型输入张量 (1,3,256,256) float32"""
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (256, 256))
        img = img.astype(np.float32) / 255.0
        img = (img - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
        img = np.transpose(img, (2, 0, 1))  # HWC → CHW
        img = np.expand_dims(img, axis=0)   # → NCHW
        return img.astype(np.float32)

    # ============================================================
    # 推理
    # ============================================================
    def infer(self, frame: np.ndarray) -> tuple:
        """
        推理一帧，返回 (ccode, class_name, confidence)
        ccode:      "C1" ~ "C22"
        class_name: "C8_Make_Up" 等完整类名
        confidence: 0~1
        """
        if not self.is_ready:
            return "C1", "C1_Drive_Safe", 0.0

        input_tensor = self.preprocess(frame)

        if self.backend == "onnx":
            outputs = self.session.run(None, {self.input_name: input_tensor})
        else:  # rknn
            outputs = self.rknn.inference(inputs=[input_tensor])

        probs = outputs[0][0]
        pred_idx = int(np.argmax(probs))
        confidence = float(probs[pred_idx])
        full_name = CLASSES[pred_idx]
        ccode = full_name.split("_")[0]  # "C8_Make_Up" → "C8"

        return ccode, full_name, confidence

    def is_risk_behavior(self, ccode: str) -> bool:
        """C2~C22 为风险行为"""
        return ccode in RISK_CODES
