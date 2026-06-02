#!/usr/bin/env python3
"""
test_accuracy.py — 测试 MobileViT 驾驶行为模型准确度

准备测试数据：
  把图片按类别放在子目录里，例如：
  test_data/
    C1_Drive_Safe/
      img001.jpg
      img002.jpg
    C2_Sleep/
      img001.jpg
    C3_Yawning/
      ...
    C22_Talk_to_Passenger/
      ...

用法：
  D:\Environment\miniconda3\envs\Reasonix_env\python.exe test_accuracy.py ^
      --model mobilevit_driver.onnx ^
      --data test_data

依赖：
  D:\Environment\miniconda3\envs\Reasonix_env\python.exe -m pip install numpy opencv-python onnxruntime
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

# ============================================================
# 22 类 (顺序必须与训练时一致)
# ============================================================
CLASSES = [
    'C1_Drive_Safe', 'C2_Sleep', 'C3_Yawning', 'C4_Talk_Left',
    'C5_Talk_Right', 'C6_Text_Left', 'C7_Text_Right', 'C8_Make_Up',
    'C9_Look_Left', 'C10_Look_Right', 'C11_Look_Up', 'C12_Look_Down',
    'C13_Smoke_Left', 'C14_Smoke_Right', 'C15_Smoke_Mouth', 'C16_Eat_Left',
    'C17_Eat_Right', 'C18_Operate_Radio', 'C19_Operate_GPS', 'C20_Reach_Behind',
    'C21_Leave_Steering_Wheel', 'C22_Talk_to_Passenger'
]

# 类别名 → 索引
CLASS_TO_IDX = {name: i for i, name in enumerate(CLASSES)}

# 短名 → 全名（支持 C1 ~ C22 缩写法）
CCODE_TO_CLASS = {}
for name in CLASSES:
    ccode = name.split("_")[0]   # "C1_Drive_Safe" → "C1"
    CCODE_TO_CLASS[ccode] = name


def load_model(onnx_path: str):
    """加载 ONNX 模型，返回 session + input_name"""
    import onnxruntime as ort
    if not os.path.exists(onnx_path):
        print(f"❌ 模型文件不存在: {onnx_path}")
        sys.exit(1)

    print(f"📦 加载模型: {onnx_path}")
    session = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    input_size = session.get_inputs()[0].shape[2]  # H, 如 224 或 256
    print(f"   输入: {input_name}, 形状: {session.get_inputs()[0].shape}")
    print(f"   输出: {session.get_outputs()[0].name}, 形状: {session.get_outputs()[0].shape}")
    return session, input_name, input_size


def preprocess(img: np.ndarray, input_size: int = 256) -> np.ndarray:
    """预处理单帧：BGR→RGB→Resize→归一化→NCHW"""
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (input_size, input_size))
    img = img.astype(np.float32) / 255.0
    img = (img - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
    img = np.transpose(img, (2, 0, 1))     # HWC → CHW
    img = np.expand_dims(img, axis=0)      # → NCHW
    return img.astype(np.float32)


def infer(session, input_name: str, img: np.ndarray, input_size: int = 256) -> tuple:
    """推理一帧，返回 (pred_idx, pred_class, confidence, all_probs)"""
    tensor = preprocess(img, input_size)
    outputs = session.run(None, {input_name: tensor})
    logits = outputs[0][0]
    # Softmax 转成 0~1 概率
    exp_logits = np.exp(logits - np.max(logits))
    probs = exp_logits / np.sum(exp_logits)
    pred_idx = int(np.argmax(probs))
    return pred_idx, CLASSES[pred_idx], float(probs[pred_idx]), probs


def discover_images(data_root: str):
    """
    扫描数据目录，返回 [(image_path, expected_label_index), ...]
    支持的目录结构：
      test_data/C1_Drive_Safe/xxx.jpg    → 目录名 = 类名
      test_data/C1/xxx.jpg               → C1 自动展开
      test_data/C1_Drive_Safe_001.jpg    → 文件名前缀匹配
    """
    samples = []
    data_root = os.path.abspath(data_root)

    if not os.path.isdir(data_root):
        print(f"❌ 数据目录不存在: {data_root}")
        sys.exit(1)

    # 情况1: 目录下有子目录（按类分）
    for entry in os.listdir(data_root):
        subpath = os.path.join(data_root, entry)
        if not os.path.isdir(subpath):
            continue

        # 尝试匹配目录名
        label = None
        if entry in CLASS_TO_IDX:
            label = CLASS_TO_IDX[entry]
        elif entry in CCODE_TO_CLASS:
            label = CLASS_TO_IDX[CCODE_TO_CLASS[entry]]
        else:
            # C1_Drive_Safe 这种格式
            ccode = entry.split("_")[0] if "_" in entry else entry
            if ccode in CCODE_TO_CLASS:
                label = CLASS_TO_IDX[CCODE_TO_CLASS[ccode]]

        if label is None:
            print(f"  ⚠ 跳过无法识别类名的目录: {entry}")
            continue

        for fname in os.listdir(subpath):
            if fname.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                samples.append((os.path.join(subpath, fname), label))

    # 情况2: 根目录下的图片文件（文件名前缀识别）
    if not samples:
        for fname in sorted(os.listdir(data_root)):
            if not fname.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                continue
            fpath = os.path.join(data_root, fname)
            # 尝试从文件名提取类名：C1_xxx.jpg 或 C1_Drive_Safe_xxx.jpg
            label = None
            parts = fname.replace('-', '_').split('_')
            if parts and parts[0] in CCODE_TO_CLASS:
                label = CLASS_TO_IDX[CCODE_TO_CLASS[parts[0]]]
            if label is not None:
                samples.append((fpath, label))

    return samples


def main():
    parser = argparse.ArgumentParser(description="MobileViT 驾驶行为模型准确度测试")
    parser.add_argument("--model", default="mobilevit_driver.onnx",
                        help="ONNX 模型路径")
    parser.add_argument("--data", default="test_data",
                        help="测试数据目录")
    parser.add_argument("--show-wrong", action="store_true",
                        help="列出所有预测错误的图片")
    args = parser.parse_args()

    print("=" * 60)
    print("  MobileViT 驾驶行为模型 — 准确度测试")
    print("=" * 60)

    # 1. 加载模型
    session, input_name, input_size = load_model(args.model)

    # 2. 扫描测试图片
    print(f"\n📂 扫描测试数据: {args.data}")
    samples = discover_images(args.data)
    if not samples:
        print("❌ 没有找到测试图片！请把图片按类别放在子目录中。")
        print("   例如: test_data/C1_Drive_Safe/xxx.jpg")
        sys.exit(1)

    print(f"   找到 {len(samples)} 张测试图片\n")

    # 3. 逐张推理
    total = len(samples)
    correct = 0
    wrong_list = []
    class_correct = {i: 0 for i in range(len(CLASSES))}
    class_total = {i: 0 for i in range(len(CLASSES))}
    total_time = 0.0
    confidences = []

    print("开始推理...")
    for i, (img_path, expected_idx) in enumerate(samples):
        img = cv2.imread(img_path)
        if img is None:
            print(f"  [{i+1}/{total}] ⚠ 无法读取: {img_path}")
            continue

        start = time.perf_counter()
        pred_idx, pred_class, conf, probs = infer(session, input_name, img, input_size)
        elapsed = (time.perf_counter() - start) * 1000  # ms

        total_time += elapsed
        confidences.append(conf)

        expected_class = CLASSES[expected_idx]
        is_correct = (pred_idx == expected_idx)
        if is_correct:
            correct += 1
        else:
            wrong_list.append((img_path, expected_class, pred_class, conf, probs))

        class_total[expected_idx] += 1
        if is_correct:
            class_correct[expected_idx] += 1

        # 进度
        if (i + 1) % 50 == 0 or i == total - 1:
            print(f"  [{i+1}/{total}] 当前准确率: {correct/(i+1)*100:.1f}%")

    # 4. 输出报告
    print("\n" + "=" * 60)
    print("  📊 测试结果")
    print("=" * 60)
    accuracy = correct / total * 100 if total > 0 else 0
    avg_conf = np.mean(confidences) * 100 if confidences else 0
    avg_time = total_time / total if total > 0 else 0

    print(f"\n  总图片:      {total}")
    print(f"  正确:        {correct}")
    print(f"  错误:        {total - correct}")
    print(f"  准确率:      {accuracy:.2f}%")
    print(f"  平均置信度:  {avg_conf:.2f}%")
    print(f"  平均推理耗时: {avg_time:.1f} ms/帧")
    print()

    # 逐类准确率
    print("  ─── 逐类准确率 ───")
    print(f"  {'类别':<28} {'数量':>5} {'正确':>5} {'准确率':>8}")
    print(f"  {'-'*28} {'-'*5} {'-'*5} {'-'*8}")
    for idx in range(len(CLASSES)):
        n = class_total[idx]
        c = class_correct[idx]
        rate = c / n * 100 if n > 0 else 0
        bar = "█" * int(rate / 5) + "░" * (20 - int(rate / 5))
        print(f"  {CLASSES[idx]:<28} {n:>5} {c:>5} {rate:>6.1f}% {bar}")
    print()

    # 错误详情
    if args.show_wrong and wrong_list:
        print(f"  ─── 预测错误 ({len(wrong_list)} 张) ───")
        for img_path, exp, pred, conf, probs in wrong_list[:30]:
            top5 = sorted(
                [(CLASSES[i], probs[i]) for i in range(len(CLASSES))],
                key=lambda x: x[1], reverse=True
            )[:3]
            top5_str = ", ".join(f"{c}({p:.2f})" for c, p in top5)
            print(f"  ❌ {os.path.basename(img_path)}")
            print(f"     期望: {exp}  →  预测: {pred} ({conf:.2f})")
            print(f"     Top-3: {top5_str}")

        if len(wrong_list) > 30:
            print(f"  ... 还有 {len(wrong_list)-30} 张错误未显示")

    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
