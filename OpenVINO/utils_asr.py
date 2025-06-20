# -*- coding: utf-8 -*-
# 录音+语音识别
# 同济子豪兄 2024-9-7
import os
import wave
import sys
from vosk import Model, KaldiRecognizer
import json

# 查看麦克风索引号：使用 arecord -l
# arecord -l

# 录音函数
def record(MIC_INDEX=2, DURATION=8):
    '''
    调用麦克风录音，需用 arecord -l 命令获取麦克风ID
    DURATION：录音时长（秒）
    '''
    print('开始 {} 秒录音'.format(DURATION))
    os.system('sudo arecord -D "plughw:{}" -f dat -c 1 -r 16000 -d {} /home/pi/TonyPi/OpenVINO/temp/speech_record.wav'.format(MIC_INDEX, DURATION))
    print('录音结束')

# 替代 appbuilder 的语音识别（使用 Vosk 离线识别）
class ASRWrapper:
    def __init__(self, model_path="/home/pi/TonyPi/vosk-model-small-cn"):
        print("加载Vosk语音识别模型...")
        self.model = Model(model_path)
    
    def run(self, audio_path):
        print("开始语音识别")
        wf = wave.open(audio_path, "rb")
        if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getframerate() != 16000:
            raise ValueError("音频文件必须是 16kHz 单声道 16位格式")
        
        rec = KaldiRecognizer(self.model, wf.getframerate())
        rec.SetWords(True)
        results = []
        while True:
            data = wf.readframes(4000)
            if len(data) == 0:
                break
            if rec.AcceptWaveform(data):
                res = json.loads(rec.Result())
                results.append(res.get("text", ""))
        # 获取最终结果
        final_res = json.loads(rec.FinalResult())
        results.append(final_res.get("text", ""))
        text = " ".join(results).strip()
        return {"content": {"text": text}}

asr_ab = ASRWrapper()

def speech_recognition(audio_path='/home/pi/TonyPi/OpenVINO/temp/speech_record.wav'):
    '''
    使用 Vosk 离线语音识别
    '''
    out = asr_ab.run(audio_path)
    speech_result = out["content"]["text"]

    speech_recognition_txt = '/home/pi/TonyPi/OpenVINO/temp/speech_recognition.txt'
    with open(speech_recognition_txt, 'w', encoding='utf-8') as f:
        f.write(speech_result)
        # print('语音识别结果已写入txt文件')

    return speech_result

# 主流程调用
record()
speech_recognition()
# print('完成录音+语音识别')
