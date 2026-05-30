import cv2
import numpy as np
import onnxruntime as ort
import time

# 1. 定义你的 22 个动作类别
CLASSES = [
    'C1_Drive_Safe', 'C2_Sleep', 'C3_Yawning', 'C4_Talk_Left',
    'C5_Talk_Right', 'C6_Text_Left', 'C7_Text_Right', 'C8_Make_Up',
    'C9_Look_Left', 'C10_Look_Right', 'C11_Look_Up', 'C12_Look_Down',
    'C13_Smoke_Left', 'C14_Smoke_Right', 'C15_Smoke_Mouth', 'C16_Eat_Left',
    'C17_Eat_Right', 'C18_Operate_Radio', 'C19_Operate_GPS', 'C20_Reach_Behind',
    'C21_Leave_Steering_Wheel', 'C22_Talk_to_Passenger'
]

def main():
    onnx_model_path = 'mobilevit_driver.onnx' # 你的 ONNX 模型文件名
    image_path = 'test.jpg'                  # 你的测试图片名

    # ==========================================
    # 初始化并加载 ONNX 模型 (使用 CPU 引擎)
    # ==========================================
    print("--> 正在加载 ONNX 模型到 CPU...")
    session = ort.InferenceSession(onnx_model_path, providers=['CPUExecutionProvider'])

    # 获取模型的输入节点名称
    input_name = session.get_inputs()[0].name
    print(f"✅ ONNX 模型加载成功！输入节点名称: {input_name}")

    # ==========================================
    # 读取并预处理图片 (严格保持 float32)
    # ==========================================
    print(f"--> 正在处理图片: {image_path}")
    img = cv2.imread(image_path)
    if img is None:
        print("找不到测试图片！请检查路径。")
        return

    # BGR 转 RGB，并拉伸到 256x256
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (256, 256))

    # 标准化并锁死 float32 精度
    img = img.astype(np.float32) / 255.0
    img = (img - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]

    # 维度转换: HWC -> CHW -> NCHW (1, 3, 256, 256)
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    img = img.astype(np.float32) # 再次确保是 float32

    # ==========================================
    # 执行 CPU 推理并测速
    # ==========================================
    print("--> 正在执行 CPU 推理...")
    start_time = time.time()

    # ONNX Runtime 的输入接收一个字典
    outputs = session.run(None, {input_name: img})

    end_time = time.time()
    print(f"🐌 CPU 推理耗时: {(end_time - start_time) * 1000:.2f} ms")

    # ==========================================
    # 解析结果
    # ==========================================
    pred_probs = outputs[0][0]
    pred_index = np.argmax(pred_probs)
    pred_class = CLASSES[pred_index]
    confidence = pred_probs[pred_index]

    print("\n" + "="*40)
    print(f"🎯 ONNX 识别结果: 【 {pred_class} 】")
    print(f"📊 置信度: {confidence:.4f}")
    print("="*40 + "\n")

if __name__ == '__main__':
    main()
