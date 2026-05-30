import cv2
import numpy as np
from rknnlite.api import RKNNLite
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
    rknn_model_path = 'mobilevit_driver_RK3566_256x256.rknn' # 替换为你的文件名
    image_path = 'test.jpg' # 你的测试图片名

    # ==========================================
    # 初始化并加载 RKNN 模型到 NPU
    # ==========================================
    print("--> 正在初始化 NPU...")
    rknn = RKNNLite()

    ret = rknn.load_rknn(rknn_model_path)
    if ret != 0:
        print("加载 RKNN 模型失败！")
        return

    print("--> 正在唤醒 NPU 核心...")
    ret = rknn.init_runtime()
    if ret != 0:
        print("初始化 NPU 运行环境失败！")
        return
    print("✅ NPU 启动成功！")

    # ==========================================
    # 读取并预处理图片 (必须与训练时完全一致)
    # ==========================================
    print(f"--> 正在处理图片: {image_path}")
    img = cv2.imread(image_path)
    if img is None:
        print("找不到测试图片！请检查路径。")
        return

    # BGR 转 RGB，并拉伸到 256x256 (之前纯净版验证集就是这么做的)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (256, 256))

    # 标准化 Normalize (由于 GitHub Actions 没有注入均值方差，我们需要手动做)
    img = img.astype(np.float32) / 255.0
    img = (img - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]

    # 维度转换: HWC (256,256,3) -> CHW (3,256,256) -> NCHW (1,3,256,256)
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)

    img = img.astype(np.float32)
    # ==========================================
    # 执行 NPU 推理 (顺便测个速)
    # ==========================================
    print("--> 正在执行推理...")
    start_time = time.time()

    # inputs 接收一个列表，里面是你的 numpy 数据
    outputs = rknn.inference(inputs=[img])

    end_time = time.time()
    print(f"⚡ 推理耗时: {(end_time - start_time) * 1000:.2f} ms")

    # ==========================================
    # 解析结果
    # ==========================================
    # outputs[0] 的形状是 (1, 22)，表示 22 个类别的概率
    pred_probs = outputs[0][0]
    pred_index = np.argmax(pred_probs)
    pred_class = CLASSES[pred_index]
    confidence = pred_probs[pred_index]

    print("\n" + "="*40)
    print(f"🎯 最终识别结果: 【 {pred_class} 】")
    print("="*40 + "\n")

    # 释放资源
    rknn.release()

if __name__ == '__main__':
    main()
