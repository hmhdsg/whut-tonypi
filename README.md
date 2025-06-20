# whut-tonypi
武汉理工大学人工智能实训机器人项目代码
使用方法：
将openvino整体文件夹上传至树莓派中，同时在本机放置剩余代码，将机器人ip填入api中，接着打开终端，输入
python
from agent_go import *

agent_play()

1
在树莓派上还需安装必要环境
pip install edge-tts
pip install pyaudio
pip install vosk
