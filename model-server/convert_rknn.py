from rknn.api import RKNN

def main():
    # 1. 创建 RKNN 对象
    rknn = RKNN(verbose=True)

    # 2. 配置模型输入参数 (将 PyTorch 的预处理直接烧录进 NPU，加速推理)
    # PyTorch 的 Normalize 是均值 [0.485, 0.456, 0.406], 方差 [0.229, 0.224, 0.225]
    # 换算成 RKNN 接受的 0-255 范围数值：
    rknn.config(
        mean_values=[[123.675, 116.28, 103.53]],
        std_values=[[58.395, 57.12, 57.375]],
        target_platform='rk3568'
    )

    # 3. 加载刚才生成的 ONNX 模型
    print('--> Loading ONNX model')
    ret = rknn.load_onnx(model='./mobilevit_driver.onnx')
    if ret != 0:
        print('Load ONNX failed!')
        return

    # 4. 构建模型 (这里先不开启 INT8 量化，使用默认的 FP16，保证准确率不掉)
    print('--> Building model')
    ret = rknn.build(do_quantization=False)
    if ret != 0:
        print('Build RKNN failed!')
        return

    # 5. 导出 RKNN 模型
    rknn_path = './mobilevit_driver.rknn'
    print('--> Exporting RKNN model')
    ret = rknn.export_rknn(rknn_path)
    if ret != 0:
        print('Export RKNN failed!')
        return

    print(f'✅ 成功导出 RKNN 模型: {rknn_path}')
    rknn.release()

if __name__ == '__main__':
    main()
