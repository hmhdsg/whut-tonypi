# utils_tts.py
# 同济子豪兄 2024-6-12
# 使用 edge-tts 合成中文语音 + 播放 mp3

print('导入语音合成模块')

import os
import asyncio
import edge_tts

tts_mp3_path = "/home/pi/TonyPi/OpenVINO/temp/tts.mp3"

async def tts(TEXT="我是TonyPi"):
    communicate = edge_tts.Communicate(text=TEXT, voice="zh-CN-XiaoxiaoNeural")
    await communicate.save(tts_mp3_path)

def play_mp3():
    os.system(f"mpg123 -q {tts_mp3_path}")

# 主流程
print('TTS Start')
with open('/home/pi/TonyPi/OpenVINO/temp/ai_response.txt', 'r', encoding='utf-8') as f:
    ai_response = f.read().strip()

print(f"[DEBUG] ai_response = \"{ai_response}\"")

# 运行 TTS 合成并播放
asyncio.run(tts(ai_response))
play_mp3()
