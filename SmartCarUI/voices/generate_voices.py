"""
生成驾驶行为语音提示音频文件（WAV 格式）
使用 edge-tts（微软晓晓，中文女声），比 espeak 自然得多。

用法：
    pip install edge-tts
    python generate_voices.py

输出：当前目录下所有 .wav 文件，共 20 个。
"""

import asyncio
import os
import sys

try:
    import edge_tts
except ImportError:
    print("请先安装 edge-tts：pip install edge-tts")
    sys.exit(1)

# 语音文本列表：(文件名, 要说的文字)
# 文件名用英文短标识，方便代码里引用
VOICES = [
    # === 语音交互按钮 ===
    ("voice_start.wav",        "语音开始识别，请说出您的指令"),
    ("voice_complete.wav",     "车内氛围灯已自动开启"),
    ("voice_cancel.wav",       "语音识别已取消"),

    # === 本地驾驶员分析 ===
    ("alert_keep_attention.wav", "请保持注意力，注意前方道路"),
    ("alert_no_driver.wav",      "未检测到驾驶员，请确认驾驶安全"),

    # === 路况预警 ===
    ("road_medium.wav",   "检测到路况异常，请注意驾驶安全"),
    ("road_high.wav",     "检测到高风险路况，请立即减速慢行"),

    # === 行为识别 - 中风险 ===
    ("behavior_talk_left.wav",   "警告：左侧打电话"),
    ("behavior_talk_right.wav",  "警告：右侧打电话"),
    ("behavior_text_left.wav",   "警告：左侧看手机"),
    ("behavior_text_right.wav",  "警告：右侧看手机"),
    ("behavior_makeup.wav",      "警告：驾驶中请勿化妆"),
    ("behavior_smoking.wav",     "警告：驾驶中请勿吸烟"),
    ("behavior_eating.wav",      "警告：驾驶中请勿饮食"),
    ("behavior_radio.wav",       "警告：操作中控设备"),
    ("behavior_gps.wav",         "警告：操作导航设备"),
    ("behavior_reach_behind.wav","警告：向后伸手取物"),

    # === 行为识别 - 高风险 ===
    ("behavior_fatigue.wav",  "危险：检测到疲劳驾驶"),
    ("behavior_yawning.wav",  "危险：检测到打哈欠"),
    ("behavior_hands_off.wav","危险：双手离开方向盘"),
]

# edge-tts 语音配置
# zh-CN-XiaoxiaoNeural = 微软晓晓, 女声, 标准中文
VOICE = "zh-CN-XiaoxiaoNeural"
RATE = "+0%"      # 语速, 默认0%
VOLUME = "+50%"   # 音量


async def gen_one(filename: str, text: str):
    """生成单个语音文件"""
    out_path = os.path.join(os.path.dirname(__file__), filename)
    print(f"生成: {filename}  <- {text}")
    communicate = edge_tts.Communicate(text, VOICE, rate=RATE, volume=VOLUME)
    await communicate.save(out_path)
    print(f"  -> 完成: {out_path}")


async def main():
    tasks = [gen_one(name, text) for name, text in VOICES]
    await asyncio.gather(*tasks)
    print(f"\n全部生成完毕！共 {len(VOICES)} 个文件")


if __name__ == "__main__":
    asyncio.run(main())
