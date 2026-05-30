# 这段代码是在 RK3568 板子上跑的！
from rknnlite.api import RKNNLite
import cv2

# 初始化
rknn = RKNNLite()
rknn.load_rknn('./mobilevit_driver.rknn')
rknn.init_runtime()

# 读取一张测试图 (注意要 Resize 到 256x256)
img = cv2.imread('test.jpg')
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img = cv2.resize(img, (256, 256))

# NPU 推理 (由于在第二步配置了均值方差，这里直接喂原图就行)
outputs = rknn.inference(inputs=[img])

print("预测结果:", outputs)
rknn.release()
