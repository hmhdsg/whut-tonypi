#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/TonyPi/')
import cv2
import time
import math
import threading
import numpy as np
import json
import queue
import os
import signal
import re
import subprocess
import grp
import pwd

# 检查串口权限
def check_serial_permissions():
    """检查串口权限和用户组"""
    try:
        current_user = pwd.getpwuid(os.getuid()).pw_name
        user_groups = [grp.getgrgid(g).gr_name for g in os.getgroups()]
        
        print(f"📋 当前用户: {current_user}")
        print(f"📋 用户组: {', '.join(user_groups)}")
        
        if 'dialout' in user_groups:
            print("✅ 用户在dialout组")
        else:
            print("❌ 用户不在dialout组")
            print("🔧 请执行: sudo usermod -a -G dialout pi")
            
        if os.path.exists('/dev/ttyAMA0'):
            stat_info = os.stat('/dev/ttyAMA0')
            print(f"📋 /dev/ttyAMA0 权限: {oct(stat_info.st_mode)[-3:]}")
        else:
            print("❌ /dev/ttyAMA0 不存在")
            
    except Exception as e:
        print(f"❌ 权限检查失败: {e}")

# 系统诊断
print("🔍 === TonyPi完整功能系统诊断 ===")
check_serial_permissions()
print("========================\n")

# 导入Vosk相关模块
try:
    import vosk
    import pyaudio
    VOSK_AVAILABLE = True
    print("✅ Vosk语音识别模块导入成功")
except Exception as e:
    print(f"❌ Vosk模块导入失败: {e}")
    VOSK_AVAILABLE = False

# 导入TonyPi模块
try:
    import hiwonder.TTS as TTS
    TTS_AVAILABLE = True
    print("✅ TTS语音合成模块导入成功")
except Exception as e:
    print(f"❌ TTS模块导入失败: {e}")
    TTS_AVAILABLE = False

try:
    import hiwonder.ASR as ASR
    ASR_AVAILABLE = True
    print("✅ ASR语音识别模块导入成功")
except Exception as e:
    print(f"❌ ASR模块导入失败: {e}")
    ASR_AVAILABLE = False

try:
    import hiwonder.Camera as Camera
    import hiwonder.Misc as Misc
    from hiwonder import yaml_handle
    CAMERA_AVAILABLE = True
    print("✅ 摄像头模块导入成功")
except Exception as e:
    print(f"❌ 摄像头模块导入失败: {e}")
    CAMERA_AVAILABLE = False

# 动作控制模块导入
ACTION_CONTROL_AVAILABLE = False
AGC_AVAILABLE = False
BOARD_AVAILABLE = False

try:
    print("🔄 尝试导入动作控制模块...")
    
    if os.path.exists('/dev/ttyAMA0'):
        import hiwonder.ActionGroupControl as AGC
        AGC_AVAILABLE = True
        print("✅ ActionGroupControl模块导入成功")
        
        import hiwonder.Board as Board
        BOARD_AVAILABLE = True
        print("✅ Board模块导入成功")
        
        try:
            Board.setBuzzer(0)
            print("✅ 硬件通信测试成功")
            ACTION_CONTROL_AVAILABLE = True
        except Exception as e:
            print(f"⚠️  硬件通信测试失败: {e}")
    else:
        print("❌ 串口设备不存在")
        
except Exception as e:
    print(f"❌ 动作控制模块导入失败: {e}")

# 模拟动作控制类
class MockActionControl:
    @staticmethod
    def runActionGroup(action_name, times=1, lock_servos='', with_stand=False):
        print(f"🎭 [模拟] 执行动作: {action_name}, 次数: {times}")
        time.sleep(0.2)
        return True

class MockBoard:
    @staticmethod
    def setPWMServoPulse(servo_id, pulse, time_ms):
        print(f"🎭 [模拟] 设置舵机 {servo_id}: {pulse}, 时间: {time_ms}ms")
        return True
    
    @staticmethod
    def setBuzzer(state):
        print(f"🎭 [模拟] 蜂鸣器: {'开' if state else '关'}")
        return True

# 智能模式选择
if not ACTION_CONTROL_AVAILABLE:
    print("🎭 启用模拟动作控制模式")
    if not AGC_AVAILABLE:
        AGC = MockActionControl()
    if not BOARD_AVAILABLE:
        Board = MockBoard()
    ACTION_CONTROL_AVAILABLE = True

# 导入AprilTag检测
try:
    import hiwonder.apriltag as apriltag
    APRILTAG_AVAILABLE = True
    print("✅ AprilTag模块导入成功")
except Exception as e:
    print(f"❌ AprilTag模块导入失败: {e}")
    APRILTAG_AVAILABLE = False

# 摄像头标定相关
try:
    if __name__ == '__main__':
        from CameraCalibration.CalibrationConfig import *
    else:
        from Functions.CameraCalibration.CalibrationConfig import *
    
    # 加载标定参数
    param_data = np.load(calibration_param_path + '.npz')
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    CALIBRATION_AVAILABLE = True
    print("✅ 摄像头标定参数加载成功")
except Exception as e:
    print(f"❌ 摄像头标定参数加载失败: {e}")
    CALIBRATION_AVAILABLE = False

class TimeoutHandler:
    """超时处理器"""
    
    def __init__(self, timeout_seconds):
        self.timeout_seconds = timeout_seconds
        self.timed_out = False
    
    def timeout_handler(self, signum, frame):
        self.timed_out = True
        raise TimeoutError(f"操作超时 ({self.timeout_seconds}秒)")
    
    def __enter__(self):
        self.timed_out = False
        signal.signal(signal.SIGALRM, self.timeout_handler)
        signal.alarm(self.timeout_seconds)
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        signal.alarm(0)
        return False

class SimpleColorDetector:
    """简单的颜色检测器（增强播报功能）"""
    
    def __init__(self):
        self.color_ranges = {
            'red': [
                [(0, 50, 50), (10, 255, 255)],
                [(170, 50, 50), (180, 255, 255)]
            ],
            'green': [[(40, 50, 50), (80, 255, 255)]],
            'blue': [[(100, 50, 50), (130, 255, 255)]],
            'yellow': [[(20, 50, 50), (40, 255, 255)]],
        }
        
        self.color_names_chinese = {
            'red': '红色',
            'green': '绿色', 
            'blue': '蓝色',
            'yellow': '黄色'
        }
        
        self.current_target_color = 'red'
        self.detection_count = 0
        self.total_frames = 0
        self.last_announce_time = 0
        self.announce_interval = 3.0
        self.voice_callback = None
        
    def set_voice_callback(self, callback):
        """设置语音播报回调函数"""
        self.voice_callback = callback
        
    def set_target_color(self, color):
        if color.lower() in self.color_ranges:
            self.current_target_color = color.lower()
            self.detection_count = 0
            self.last_announce_time = 0
            print(f"🎯 设置目标颜色: {color}")
            return True
        return False
    
    def detect_color(self, frame):
        if frame is None:
            return None
            
        self.total_frames += 1
        
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            masks = []
            for color_range in self.color_ranges[self.current_target_color]:
                lower, upper = color_range
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                masks.append(mask)
            
            final_mask = masks[0]
            for mask in masks[1:]:
                final_mask = cv2.bitwise_or(final_mask, mask)
            
            kernel = np.ones((5,5), np.uint8)
            final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)
            final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            result_frame = frame.copy()
            detected = False
            largest_area = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:
                    cv2.drawContours(result_frame, [contour], -1, (0, 255, 0), 2)
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(result_frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    cv2.putText(result_frame, f'{self.current_target_color.upper()}', 
                               (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    detected = True
                    if area > largest_area:
                        largest_area = area
            
            cv2.putText(result_frame, f'Target: {self.current_target_color.upper()}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(result_frame, f'Frame: {self.total_frames}', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            if detected:
                self.detection_count += 1
                cv2.putText(result_frame, f'DETECTED! Area: {int(largest_area)}', 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                current_time = time.time()
                if current_time - self.last_announce_time >= self.announce_interval:
                    color_chinese = self.color_names_chinese.get(self.current_target_color, self.current_target_color)
                    announce_text = f"识别到{color_chinese}"
                    print(f"🔊 颜色检测语音播报: {announce_text}")
                    if self.voice_callback:
                        try:
                            self.voice_callback(announce_text)
                        except Exception as e:
                            print(f"❌ 颜色检测语音播报失败: {e}")
                    self.last_announce_time = current_time
                    
                if self.detection_count % 10 == 1:
                    print(f"🎯 检测到{self.current_target_color}颜色! 区域: {largest_area}")
            else:
                cv2.putText(result_frame, 'NOT DETECTED', 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            return result_frame
            
        except Exception as e:
            print(f"❌ 颜色检测错误: {e}")
            return frame

class OriginalFaceDetector:
    """完全按照原始代码逻辑的人脸检测器"""
    
    def __init__(self):
        # === 完全复制原始代码的变量 ===
        self.__isRunning = False              # 主要运行状态
        self.start_greet = False              # 触发动作的信号
        self.action_finish = True             # 动作完成标志
        self.last_action_time = 0             # 上次动作时间
        self.action_interval = 3.0            # 动作间隔
        
        # 检测器设置
        self.conf_threshold = 0.6
        self.detection_count = 0
        self.total_frames = 0
        self.voice_callback = None
        
        # 舵机配置
        self.servo_data = None
        self.servo2_pulse = 1500
        self.d_pulse = 10
        self.load_servo_config()
        
        # 初始化DNN检测器
        self.dnn_available = False
        self.opencv_available = False
        self.init_detectors()
        
        # 启动动作控制线程（完全按照原始代码）
        self.control_thread = threading.Thread(target=self.move_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        print("✅ 人脸检测器动作控制线程已启动")
        
    def load_servo_config(self):
        """加载舵机配置"""
        try:
            if yaml_handle:
                self.servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)
                self.servo2_pulse = self.servo_data.get('servo2', 1500)
                print(f"✅ 舵机配置加载成功: servo2={self.servo2_pulse}")
        except Exception as e:
            print(f"❌ 舵机配置加载失败: {e}")
            self.servo2_pulse = 1500
    
    def init_detectors(self):
        """初始化检测器"""
        # OpenCV人脸检测器
        try:
            self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            self.opencv_available = True
            print("✅ OpenCV人脸检测器加载成功")
        except Exception as e:
            print(f"❌ OpenCV人脸检测器加载失败: {e}")
        
        # DNN人脸检测器
        try:
            modelFile = "/home/pi/TonyPi/models/res10_300x300_ssd_iter_140000_fp16.caffemodel"
            configFile = "/home/pi/TonyPi/models/deploy.prototxt"
            if os.path.exists(modelFile) and os.path.exists(configFile):
                self.net = cv2.dnn.readNetFromCaffe(configFile, modelFile)
                self.dnn_available = True
                print("✅ DNN人脸检测器加载成功")
            else:
                print("❌ DNN模型文件不存在")
        except Exception as e:
            print(f"❌ DNN人脸检测器加载失败: {e}")
    
    def set_voice_callback(self, callback):
        """设置语音播报回调函数"""
        self.voice_callback = callback
    
    def start_detection(self):
        """启动检测（原始代码风格）"""
        print("👤 === 启动人脸检测 ===")
        self.__isRunning = True
        self.detection_count = 0
        self.total_frames = 0
        self.action_finish = True
        self.start_greet = False
        
        # 初始化舵机位置
        if ACTION_CONTROL_AVAILABLE:
            try:
                print("🔄 初始化舵机位置...")
                Board.setPWMServoPulse(1, 1800, 500)
                Board.setPWMServoPulse(2, self.servo2_pulse, 500)
                print("✅ 舵机初始化完成")
            except Exception as e:
                print(f"❌ 舵机初始化失败: {e}")
        
    def stop_detection(self):
        """停止检测（原始代码风格）"""
        print("👤 === 停止人脸检测 ===")
        self.__isRunning = False
        self.start_greet = False
    
    def move_control_loop(self):
        """动作控制线程（完全按照原始代码）"""
        while True:
            try:
                # 只在运行状态下检查信号
                if self.__isRunning:
                    if self.start_greet:
                        print("🤖 === 检测到start_greet信号，开始执行动作 ===")
                        
                        # 重置信号
                        self.start_greet = False
                        
                        # 设置动作状态为执行中
                        self.action_finish = False
                        
                        try:
                            # 语音播报"识别到人脸"
                            if self.voice_callback:
                                print("🔊 开始语音播报...")
                                self.voice_callback("识别到人脸")
                                print("🔊 ✅ 语音播报完成")
                            
                            # 执行挥手动作
                            if ACTION_CONTROL_AVAILABLE:
                                print("🤖 执行挥手动作...")
                                AGC.runActionGroup('wave')
                                print("🤖 ✅ 挥手动作执行完成")
                                
                            # 记录时间
                            self.last_action_time = time.time()
                            
                        except Exception as e:
                            print(f"❌ 动作执行失败: {e}")
                        finally:
                            # 设置动作完成
                            self.action_finish = True
                            print("✅ 动作流程完成")
                
                time.sleep(0.01)  # 避免CPU占用过高
                
            except Exception as e:
                print(f"❌ 动作控制线程错误: {e}")
                time.sleep(0.1)
    
    def detect_faces_dnn(self, frame):
        """使用DNN检测人脸（原始代码逻辑）"""
        if not self.dnn_available:
            return frame, 0
            
        try:
            img_h, img_w = frame.shape[:2]
            blob = cv2.dnn.blobFromImage(frame, 1, (150, 150), [104, 117, 123], False, False)
            self.net.setInput(blob)
            detections = self.net.forward()
            
            face_count = 0
            result_frame = frame.copy()
            
            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                if confidence > self.conf_threshold:
                    x1 = int(detections[0, 0, i, 3] * img_w)
                    y1 = int(detections[0, 0, i, 4] * img_h)
                    x2 = int(detections[0, 0, i, 5] * img_w)
                    y2 = int(detections[0, 0, i, 6] * img_h)
                    
                    face_count += 1
                    
                    # 绘制人脸框
                    cv2.rectangle(result_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(result_frame, f'FACE {confidence:.2f}', (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # 计算人脸中心
                    face_center_x = (x1 + x2) / 2
                    face_center_y = (y1 + y2) / 2
                    img_center_x = img_w / 2
                    img_center_y = img_h / 2
                    
                    # 绘制中心点
                    cv2.circle(result_frame, (int(face_center_x), int(face_center_y)), 5, (255, 0, 0), -1)
                    cv2.circle(result_frame, (int(img_center_x), int(img_center_y)), 5, (0, 0, 255), -1)
                    
                    # 计算距离
                    center_distance = abs(face_center_x - img_center_x)
                    center_threshold = img_w / 4
                    
                    # === 原始代码的核心检测逻辑 ===
                    if center_distance < center_threshold:
                        cv2.putText(result_frame, 'CENTER!', (x1, y2+25), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        
                        # 检查动作条件（原始逻辑）
                        current_time = time.time()
                        time_diff = current_time - self.last_action_time
                        
                        if self.action_finish and time_diff >= self.action_interval:
                            self.start_greet = True  # === 关键信号 ===
                    else:
                        cv2.putText(result_frame, 'OFF CENTER', (x1, y2+25), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    
                    # 显示距离信息
                    cv2.putText(result_frame, f'Dist: {center_distance:.0f}', (x1, y2+50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            return result_frame, face_count
            
        except Exception as e:
            print(f"❌ DNN人脸检测错误: {e}")
            return frame, 0
    
    def detect_faces(self, frame):
        """主检测函数"""
        if frame is None or not self.__isRunning:
            return frame
            
        self.total_frames += 1
        
        # 优先使用DNN检测器
        if self.dnn_available:
            result_frame, face_count = self.detect_faces_dnn(frame)
        else:
            return frame
        
        # 添加状态信息
        cv2.putText(result_frame, f'Faces: {face_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(result_frame, f'Frame: {self.total_frames}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(result_frame, f'Running: {self.__isRunning}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(result_frame, f'Action: {"Ready" if self.action_finish else "Working"}', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        if face_count > 0:
            self.detection_count += 1
        
        return result_frame
    
    def test_action(self):
        """测试动作（直接触发）"""
        print("🧪 === 测试人脸动作 ===")
        if not self.__isRunning:
            self.__isRunning = True
        
        self.start_greet = True
        time.sleep(0.5)

# AprilTag检测器类
class AprilTagDetector:
    """AprilTag检测器"""
    
    def __init__(self):
        self.detector = None
        self.available = False
        self.detection_count = 0
        self.total_frames = 0
        self.voice_callback = None
        self.current_tag_id = None
        
        if APRILTAG_AVAILABLE:
            try:
                self.detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
                self.available = True
                print("✅ AprilTag检测器初始化成功")
            except Exception as e:
                print(f"❌ AprilTag检测器初始化失败: {e}")
            
        # 动作映射
        self.action_map = {
            1: ('wave', '挥手'),
            2: ('stepping', '原地踏步'),
            3: ('twist', '扭腰')
        }
        
    def set_voice_callback(self, callback):
        """设置语音播报回调函数"""
        self.voice_callback = callback
    
    def detect_apriltag(self, img):
        """检测AprilTag"""
        if not self.available or img is None:
            return None, None
            
        try:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray, return_image=False)
            
            if len(detections) != 0:
                for detection in detections:
                    corners = np.rint(detection.corners)
                    cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)
                    tag_family = str(detection.tag_family, encoding='utf-8')
                    tag_id = int(detection.tag_id)
                    
                    return tag_family, tag_id
                    
            return None, None
            
        except Exception as e:
            print(f"❌ AprilTag检测错误: {e}")
            return None, None
    
    def detect_tags(self, frame):
        """主标签检测函数"""
        if frame is None:
            return frame
            
        self.total_frames += 1
        result_frame = frame.copy()
        
        tag_family, tag_id = self.detect_apriltag(result_frame)
        
        if tag_id is not None:
            cv2.putText(result_frame, f"tag_id: {tag_id}", (10, result_frame.shape[0] - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 0, 255], 2)
            cv2.putText(result_frame, f"tag_family: {tag_family}", (10, result_frame.shape[0] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 0, 255], 2)
            
            # 设置当前标签ID
            if self.current_tag_id != tag_id:
                self.current_tag_id = tag_id
                
                # 执行对应动作
                if tag_id in self.action_map:
                    action_name, action_chinese = self.action_map[tag_id]
                    
                    if ACTION_CONTROL_AVAILABLE:
                        try:
                            print(f"🤖 执行动作: {action_chinese} (ID: {tag_id})")
                            AGC.runActionGroup(action_name)
                            
                            # 语音播报
                            if self.voice_callback:
                                announce_text = f"识别到标签{tag_id}，执行{action_chinese}"
                                try:
                                    self.voice_callback(announce_text)
                                except Exception as e:
                                    print(f"❌ 标签检测语音播报失败: {e}")
                            
                        except Exception as e:
                            print(f"❌ 动作执行失败: {e}")
                    
                self.detection_count += 1
                
        else:
            cv2.putText(result_frame, "tag_id: None", (10, result_frame.shape[0] - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 0, 255], 2)
            self.current_tag_id = None
        
        # 添加状态信息
        cv2.putText(result_frame, f'Tags: {1 if tag_id else 0}', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(result_frame, f'Frame: {self.total_frames}', 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return result_frame

class TransportSystem:
    """搬运系统"""
    
    def __init__(self, voice_callback=None):
        print("🚛 初始化搬运系统...")
        self.voice_callback = voice_callback
        
        # 基础配置
        self.lab_data = None
        self.servo_data = None
        self.load_config()
        
        # 搬运动作组名称
        self.go_forward = 'go_forward'
        self.back = 'back_fast'
        self.turn_left = 'turn_left_small_step'
        self.turn_right = 'turn_right_small_step'
        self.left_move = 'left_move'
        self.right_move = 'right_move'
        self.left_move_large = 'left_move_30'
        self.right_move_large = 'right_move_30'
        
        # 颜色对应的tag编号
        self.color_tag = {'red': 1, 'green': 2, 'blue': 3}
        self.color_names_chinese = {'red': '红色', 'green': '绿色', 'blue': '蓝色'}
        
        # 舵机锁定配置
        self.LOCK_SERVOS = {'6': 650, '7': 850, '8': 0, '14': 350, '15': 150, '16': 1000}
        
        # 系统状态
        self.is_running = False
        self.current_object_color = None
        
        # 搬运状态变量
        self.reset_transport_vars()
        
        # 初始化AprilTag检测器
        if APRILTAG_AVAILABLE:
            try:
                self.detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
                print("✅ 搬运系统AprilTag检测器初始化成功")
            except Exception as e:
                print(f"❌ 搬运系统AprilTag检测器初始化失败: {e}")
                self.detector = None
        else:
            self.detector = None
        
        # 启动动作控制线程
        self.action_thread = threading.Thread(target=self.action_control_loop)
        self.action_thread.daemon = True
        self.action_thread.start()
        
        print("✅ 搬运系统初始化完成")
    
    def load_config(self):
        """加载配置文件"""
        try:
            if yaml_handle:
                self.lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
                self.servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)
        except Exception as e:
            print(f"❌ 搬运系统配置文件加载失败: {e}")
            # 默认配置
            self.servo_data = {'servo1': 1500, 'servo2': 1500}
    
    def reset_transport_vars(self):
        """重置搬运相关变量"""
        self.d_x = 15
        self.d_y = 15
        self.step = 1
        self.time_start = 0
        self.x_dis = self.servo_data['servo2'] if self.servo_data else 1500
        self.y_dis = self.servo_data['servo1'] if self.servo_data else 1500
        self.start_count = True
        self.object_center_x, self.object_center_y, self.object_angle = -2, -2, 0
        self.turn = 'None'
        self.CENTER_X = 350
        self.find_box = True
        self.go_step = 3
        self.lock_servos = ''
        self.stop_detect = False
        self.haved_find_tag = False
        self.head_turn = 'left_right'
        self.color_center_x, self.color_center_y = -1, -1
    
    def get_area_max_contour(self, contours):
        """找出面积最大的轮廓"""
        contour_area_max = 0
        area_max_contour = None

        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp >= 300:
                    area_max_contour = c

        return area_max_contour, contour_area_max
    
    def color_detect(self, img, target_color):
        """颜色检测"""
        if not self.lab_data or target_color not in self.lab_data:
            return 'None', -1, -1, 0
        
        img_h, img_w = img.shape[:2]
        size = (320, 240)
        
        frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        
        # 颜色范围检测
        frame_mask = cv2.inRange(frame_lab,
                                 (self.lab_data[target_color]['min'][0],
                                  self.lab_data[target_color]['min'][1],
                                  self.lab_data[target_color]['min'][2]),
                                 (self.lab_data[target_color]['max'][0],
                                  self.lab_data[target_color]['max'][1],
                                  self.lab_data[target_color]['max'][2]))
        
        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        
        area_max_contour, area_max = self.get_area_max_contour(contours)
        
        if area_max > 500:
            rect = cv2.minAreaRect(area_max_contour)
            angle = rect[2]
            
            box = np.int0(cv2.boxPoints(rect))
            for j in range(4):
                box[j, 0] = int(Misc.map(box[j, 0], 0, size[0], 0, img_w))
                box[j, 1] = int(Misc.map(box[j, 1], 0, size[1], 0, img_h))

            cv2.drawContours(img, [box], -1, (0, 255, 255), 2)
            
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]            
            center_x, center_y = int((pt1_x + pt3_x) / 2), int((pt1_y + pt3_y) / 2)
            cv2.circle(img, (center_x, center_y), 5, (0, 255, 255), -1)
            
            return target_color, center_x, center_y, angle
                    
        return 'None', -1, -1, 0
    
    def apriltag_detect(self, img):
        """AprilTag检测"""
        if not self.detector:
            return [-1, -1, 0], [-1, -1, 0], [-1, -1, 0]
        
        try:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray, return_image=False)
            
            tag_1 = [-1, -1, 0]
            tag_2 = [-1, -1, 0] 
            tag_3 = [-1, -1, 0]

            if len(detections) != 0:
                for detection in detections:                       
                    corners = np.rint(detection.corners)
                    cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)

                    tag_family = str(detection.tag_family, encoding='utf-8')
                    tag_id = str(detection.tag_id)

                    object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])
                    cv2.circle(img, (object_center_x, object_center_y), 5, (0, 255, 255), -1)
                    
                    object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))
                    
                    if tag_family == 'tag36h11':
                        if tag_id == '1':
                            tag_1 = [object_center_x, object_center_y, object_angle]
                        elif tag_id == '2':
                            tag_2 = [object_center_x, object_center_y, object_angle]
                        elif tag_id == '3':
                            tag_3 = [object_center_x, object_center_y, object_angle]
            
            return tag_1, tag_2, tag_3
        except Exception as e:
            print(f"❌ 搬运系统AprilTag检测错误: {e}")
            return [-1, -1, 0], [-1, -1, 0], [-1, -1, 0]
    
    def get_turn_direction(self, tag_id, tag_data):
        """通过其他AprilTag判断目标Tag位置"""
        tag_1, tag_2, tag_3 = tag_data
        
        if tag_id == 1:
            if tag_2[0] == -1:
                if tag_3[0] != -1:
                    return 'left'
            else:
                return 'left'
        elif tag_id == 2:
            if tag_1[0] == -1:
                if tag_3[0] != -1:
                    return 'left'
            else:
                return 'right'
        elif tag_id == 3:
            if tag_1[0] == -1:
                if tag_2[0] != -1:
                    return 'right'
            else:
                return 'right'
        
        return 'None'
    
    def action_control_loop(self):
        """动作控制线程"""
        while True:
            try:
                if self.is_running and self.current_object_color:
                    self.execute_transport_actions()
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"❌ 搬运动作控制线程错误: {e}")
                time.sleep(0.1)
    
    def execute_transport_actions(self):
        """执行搬运动作逻辑（完整保留原始逻辑）"""
        if self.object_center_x == -3:  # 寻找标签但找到其他标签
            if self.turn == 'left':
                AGC.runActionGroup(self.turn_left, lock_servos=self.lock_servos)
            elif self.turn == 'right':
                AGC.runActionGroup(self.turn_right, lock_servos=self.lock_servos)
                
        elif self.haved_find_tag and self.object_center_x == -1:
            if self.x_dis > self.servo_data['servo2']:                    
                AGC.runActionGroup(self.turn_left, lock_servos=self.lock_servos)
            elif self.x_dis < self.servo_data['servo2']:
                AGC.runActionGroup(self.turn_right, lock_servos=self.lock_servos)
            else:
                self.haved_find_tag = False
                
        elif self.object_center_x >= 0:
            self.handle_object_found()
            
        elif self.object_center_x == -1:
            self.search_for_object()
        
        time.sleep(0.01)
    
    def handle_object_found(self):
        """处理找到目标物体的情况（增强头部跟随功能）"""
        if not self.find_box:  # 放置阶段避障
            if self.color_center_y > 350:
                if (self.color_center_x - self.CENTER_X) > 80:
                    AGC.runActionGroup(self.go_forward, lock_servos=self.lock_servos)
                elif (self.color_center_x > self.CENTER_X and self.object_center_x >= self.CENTER_X) or \
                     (self.color_center_x <= self.CENTER_X and self.object_center_x >= self.CENTER_X):
                    AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
                elif (self.color_center_x > self.CENTER_X and self.object_center_x < self.CENTER_X) or \
                     (self.color_center_x <= self.CENTER_X and self.object_center_x < self.CENTER_X):
                    AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos)

        # === 新增：实时头部跟随逻辑 ===
        self.update_head_tracking()

        # 转头找到物体时头回中
        if self.x_dis != self.servo_data['servo2'] and not self.haved_find_tag:
            self.head_turn = 'left_right'
            self.start_count = True
            self.d_x, self.d_y = 15, 15
            self.haved_find_tag = True
            
            Board.setPWMServoPulse(1, self.servo_data['servo1'], 500)
            Board.setPWMServoPulse(2, self.servo_data['servo2'], 500)
            time.sleep(0.6)
            
        elif self.step == 1:  # 左右调整对中
            self.x_dis = self.servo_data['servo2']
            self.y_dis = self.servo_data['servo1']                   
            self.turn = ''
            self.haved_find_tag = False
            
            if (self.object_center_x - self.CENTER_X) > 170 and self.object_center_y > 330:
                AGC.runActionGroup(self.back, lock_servos=self.lock_servos)   
            elif self.object_center_x - self.CENTER_X > 80:
                AGC.runActionGroup(self.turn_right, lock_servos=self.lock_servos)
            elif self.object_center_x - self.CENTER_X < -80:
                AGC.runActionGroup(self.turn_left, lock_servos=self.lock_servos)                        
            elif 0 < self.object_center_y <= 250:
                AGC.runActionGroup(self.go_forward, lock_servos=self.lock_servos)
            else:
                self.step = 2
                
        elif self.step == 2:  # 接近物体
            if 330 < self.object_center_y:
                AGC.runActionGroup(self.back, lock_servos=self.lock_servos)
            elif self.find_box:  # 搬运阶段
                self.handle_pickup_approach()
            else:  # 放置阶段
                self.handle_place_approach()
                
        elif self.step == 3:  # 精细调整
            self.handle_fine_adjustment()
            
        elif self.step == 4:  # 最终接近
            self.handle_final_approach()
            
        elif self.step == 5:  # 执行拿起或放下
            self.handle_pickup_or_place()
    
    def handle_pickup_approach(self):
        """处理拾取阶段的接近"""
        if self.object_center_x - self.CENTER_X > 150:
            AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
        elif self.object_center_x - self.CENTER_X < -150:
            AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos)                        
        elif -10 > self.object_angle > -45:
            AGC.runActionGroup(self.turn_left, lock_servos=self.lock_servos)
        elif -80 < self.object_angle <= -45:
            AGC.runActionGroup(self.turn_right, lock_servos=self.lock_servos)
        elif self.object_center_x - self.CENTER_X > 40:
            AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
        elif self.object_center_x - self.CENTER_X < -40:
            AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos)
        else:
            self.step = 3
    
    def handle_place_approach(self):
        """处理放置阶段的接近"""
        if self.object_center_x - self.CENTER_X > 150:
            AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
        elif self.object_center_x - self.CENTER_X < -150:
            AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos)                        
        elif self.object_angle < -5:
            AGC.runActionGroup(self.turn_left, lock_servos=self.lock_servos)
        elif 5 < self.object_angle:
            AGC.runActionGroup(self.turn_right, lock_servos=self.lock_servos)
        elif self.object_center_x - self.CENTER_X > 40:
            AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
        elif self.object_center_x - self.CENTER_X < -40:
            AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos)
        else:
            self.step = 3
    
    def handle_fine_adjustment(self):
        """处理精细调整"""
        if 340 < self.object_center_y:
            AGC.runActionGroup(self.back, lock_servos=self.lock_servos)
        elif 0 < self.object_center_y <= 250:
            AGC.runActionGroup(self.go_forward, lock_servos=self.lock_servos)
        elif self.object_center_x - self.CENTER_X >= 40:
            AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
        elif self.object_center_x - self.CENTER_X <= -40:
            AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos) 
        elif 20 <= self.object_center_x - self.CENTER_X < 40:
            AGC.runActionGroup(self.right_move, lock_servos=self.lock_servos)
        elif -40 < self.object_center_x - self.CENTER_X < -20:                      
            AGC.runActionGroup(self.left_move, lock_servos=self.lock_servos)
        else:
            self.step = 4
    
    def handle_final_approach(self):
        """处理最终接近"""
        if 300 < self.object_center_y <= 340:
            AGC.runActionGroup('go_forward_one_step', lock_servos=self.lock_servos)
            time.sleep(0.2)
        elif 0 <= self.object_center_y <= 300:
            AGC.runActionGroup(self.go_forward, lock_servos=self.lock_servos)
        else:
            if self.object_center_y >= 370:
                self.go_step = 2
            else:
                self.go_step = 3
            if abs(self.object_center_x - self.CENTER_X) <= 20:
                self.stop_detect = True
                self.step = 5
            else:
                self.step = 3
    
    def handle_pickup_or_place(self):
        """处理拾取或放置动作"""
        if self.find_box:  # 拾取
            AGC.runActionGroup('go_forward_one_step', times=3)
            AGC.runActionGroup('stand', lock_servos=self.lock_servos)
            AGC.runActionGroup('move_up')
            self.lock_servos = self.LOCK_SERVOS
            self.step = 6
            color_chinese = self.color_names_chinese.get(self.current_object_color, self.current_object_color)
            if self.voice_callback:
                self.voice_callback(f'已拾取{color_chinese}方块')
        else:  # 放置
            AGC.runActionGroup('go_forward_one_step', times=self.go_step, lock_servos=self.lock_servos)
            AGC.runActionGroup('stand', lock_servos=self.lock_servos)
            AGC.runActionGroup('put_down')
            AGC.runActionGroup(self.back, times=5, with_stand=True)
            color_chinese = self.color_names_chinese.get(self.current_object_color, self.current_object_color)
            if self.voice_callback:
                self.voice_callback(f'{color_chinese}方块搬运完成')
            self.lock_servos = ''
            self.step = 6
            self.current_object_color = None  # 完成任务
    
    def update_head_tracking(self):
        """实时头部跟随目标物体"""
        if self.object_center_x == -1 or self.object_center_x == -2:
            return  # 没有检测到目标，不调整头部
        
        # 获取图像中心和目标位置
        img_center_x = 320  # 假设图像宽度640，中心为320
        target_x = self.object_center_x
        target_y = self.object_center_y
        
        # 计算偏差
        error_x = target_x - img_center_x
        error_y = target_y - 240  # 假设图像高度480，中心为240
        
        # 头部跟随阈值
        follow_threshold_x = 50  # 水平跟随阈值
        follow_threshold_y = 40  # 垂直跟随阈值
        
        # 舵机调整步长
        servo_step_x = 3  # 水平调整步长
        servo_step_y = 3  # 垂直调整步长
        
        # 舵机限制范围
        servo2_min = self.servo_data['servo2'] - 400  # 水平舵机最小值
        servo2_max = self.servo_data['servo2'] + 400  # 水平舵机最大值
        servo1_min = self.servo_data['servo1'] - 200  # 垂直舵机最小值
        servo1_max = self.servo_data['servo1'] + 300  # 垂直舵机最大值
        
        # 水平跟随（舵机2）
        if abs(error_x) > follow_threshold_x:
            if error_x > 0:  # 目标在右边，头向右转
                new_x_dis = self.x_dis + servo_step_x
            else:  # 目标在左边，头向左转
                new_x_dis = self.x_dis - servo_step_x
            
            # 限制舵机范围
            new_x_dis = max(servo2_min, min(servo2_max, new_x_dis))
            
            if new_x_dis != self.x_dis:
                self.x_dis = new_x_dis
                Board.setPWMServoPulse(2, self.x_dis, 20)
                print(f"🔄 头部水平跟随: {self.x_dis}, 目标X: {target_x}, 误差: {error_x}")
        
        # 垂直跟随（舵机1）
        if abs(error_y) > follow_threshold_y:
            if error_y > 0:  # 目标在下方，头向下
                new_y_dis = self.y_dis + servo_step_y
            else:  # 目标在上方，头向上
                new_y_dis = self.y_dis - servo_step_y
            
            # 限制舵机范围
            new_y_dis = max(servo1_min, min(servo1_max, new_y_dis))
            
            if new_y_dis != self.y_dis:
                self.y_dis = new_y_dis
                Board.setPWMServoPulse(1, self.y_dis, 20)
                print(f"🔄 头部垂直跟随: {self.y_dis}, 目标Y: {target_y}, 误差: {error_y}")
    
    def search_for_object(self):
        """搜索目标物体（改进搜索逻辑）"""
        if self.start_count:
            self.start_count = False
            self.time_start = time.time()
            print("🔍 开始搜索目标物体...")
        else:
            if time.time() - self.time_start > 0.5:
                if 0 < self.servo_data['servo2'] - self.x_dis <= abs(self.d_x) and self.d_y > 0:
                    print("🔄 搜索完成，机器人转体寻找目标")
                    self.x_dis = self.servo_data['servo2']
                    self.y_dis = self.servo_data['servo1']
                    Board.setPWMServoPulse(1, self.y_dis, 20)
                    Board.setPWMServoPulse(2, self.x_dis, 20)
                    AGC.runActionGroup(self.turn_right, times=5, lock_servos=self.lock_servos)
                elif self.head_turn == 'left_right':
                    # 水平搜索
                    self.x_dis += self.d_x
                    print(f"🔍 水平搜索: {self.x_dis}")
                    if self.x_dis > self.servo_data['servo2'] + 400 or self.x_dis < self.servo_data['servo2'] - 400:
                        self.head_turn = 'up_down'
                        self.d_x = -self.d_x
                        print("🔄 切换到垂直搜索")
                elif self.head_turn == 'up_down':
                    # 垂直搜索
                    self.y_dis += self.d_y
                    print(f"🔍 垂直搜索: {self.y_dis}")
                    if self.y_dis > self.servo_data['servo1'] + 300 or self.y_dis < self.servo_data['servo1']:
                        self.head_turn = 'left_right'
                        self.d_y = -self.d_y
                        print("🔄 切换到水平搜索")
                
                Board.setPWMServoPulse(1, self.y_dis, 20)
                Board.setPWMServoPulse(2, self.x_dis, 20)
                time.sleep(0.02)
    
    def process_frame(self, img):
        """处理视频帧"""
        if not self.is_running or self.stop_detect:
            if self.step == 5:
                self.object_center_x = 0
            elif self.step == 6:
                self.find_box = not self.find_box
                self.object_center_x = -2
                self.step = 1
                self.stop_detect = False
            return img
        
        if not self.current_object_color:
            self.object_center_x = -4
            return img
        
        # 颜色检测
        color, self.color_center_x, self.color_center_y, color_angle = self.color_detect(img, self.current_object_color)
        
        # 根据阶段确定目标
        if self.find_box:  # 搬运阶段 - 找方块
            self.object_center_x, self.object_center_y, self.object_angle = self.color_center_x, self.color_center_y, color_angle
        else:  # 放置阶段 - 找标签
            tag_data = self.apriltag_detect(img)
            
            target_tag_id = self.color_tag[self.current_object_color]
            if tag_data[target_tag_id - 1][0] != -1:  # 找到目标标签
                self.object_center_x, self.object_center_y, self.object_angle = tag_data[target_tag_id - 1]
            else:  # 通过其他标签判断位置
                self.turn = self.get_turn_direction(target_tag_id, tag_data)
                if self.turn == 'None':
                    self.object_center_x, self.object_center_y, self.object_angle = -1, -1, 0
                else:
                    self.object_center_x, self.object_center_y, self.object_angle = -3, -1, 0
        
        return img
    
    def start_transport(self, color):
        """开始搬运指定颜色的方块（增强头部初始化）"""
        if color not in self.color_tag:
            return False
        
        self.current_object_color = color
        self.is_running = True
        self.reset_transport_vars()
        
        # 初始化头部位置
        self.init_head_position()
        
        print(f"🚛 开始搬运任务: {self.color_names_chinese[color]}")
        return True
    
    def init_head_position(self):
        """初始化头部位置"""
        try:
            print("🔄 初始化机器人头部位置...")
            # 设置头部到中心位置
            Board.setPWMServoPulse(1, self.servo_data['servo1'], 500)  # 垂直舵机
            Board.setPWMServoPulse(2, self.servo_data['servo2'], 500)  # 水平舵机
            time.sleep(1)  # 等待舵机到位
            
            # 更新当前位置记录
            self.x_dis = self.servo_data['servo2']
            self.y_dis = self.servo_data['servo1']
            
            print(f"✅ 头部初始化完成: 水平={self.x_dis}, 垂直={self.y_dis}")
        except Exception as e:
            print(f"❌ 头部初始化失败: {e}")
    
    def process_frame(self, img):
        """处理视频帧（增强显示信息）"""
        if not self.is_running or self.stop_detect:
            if self.step == 5:
                self.object_center_x = 0
            elif self.step == 6:
                self.find_box = not self.find_box
                self.object_center_x = -2
                self.step = 1
                self.stop_detect = False
            return img
        
        if not self.current_object_color:
            self.object_center_x = -4
            return img
        
        # 颜色检测
        color, self.color_center_x, self.color_center_y, color_angle = self.color_detect(img, self.current_object_color)
        
        # 根据阶段确定目标
        if self.find_box:  # 搬运阶段 - 找方块
            self.object_center_x, self.object_center_y, self.object_angle = self.color_center_x, self.color_center_y, color_angle
        else:  # 放置阶段 - 找标签
            tag_data = self.apriltag_detect(img)
            
            target_tag_id = self.color_tag[self.current_object_color]
            if tag_data[target_tag_id - 1][0] != -1:  # 找到目标标签
                self.object_center_x, self.object_center_y, self.object_angle = tag_data[target_tag_id - 1]
            else:  # 通过其他标签判断位置
                self.turn = self.get_turn_direction(target_tag_id, tag_data)
                if self.turn == 'None':
                    self.object_center_x, self.object_center_y, self.object_angle = -1, -1, 0
                else:
                    self.object_center_x, self.object_center_y, self.object_angle = -3, -1, 0
        
        # 在图像上绘制跟踪信息
        self.draw_tracking_info(img)
        
        return img
    
    def draw_tracking_info(self, img):
        """在图像上绘制跟踪信息"""
        # 绘制图像中心点
        img_center_x, img_center_y = 320, 240
        cv2.circle(img, (img_center_x, img_center_y), 10, (0, 0, 255), 2)
        cv2.putText(img, 'CENTER', (img_center_x-30, img_center_y-15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # 绘制目标位置
        if self.object_center_x > 0 and self.object_center_y > 0:
            cv2.circle(img, (self.object_center_x, self.object_center_y), 8, (0, 255, 0), -1)
            cv2.putText(img, 'TARGET', (self.object_center_x-30, self.object_center_y-15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # 绘制偏差线
            cv2.line(img, (img_center_x, img_center_y), 
                    (self.object_center_x, self.object_center_y), (255, 255, 0), 2)
            
            # 显示偏差值
            error_x = self.object_center_x - img_center_x
            error_y = self.object_center_y - img_center_y
            cv2.putText(img, f'Error: ({error_x:.0f}, {error_y:.0f})', 
                       (10, img.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # 显示头部位置信息
        cv2.putText(img, f'Head: H={self.x_dis}, V={self.y_dis}', 
                   (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    def stop_transport(self):
        """停止搬运"""
        self.is_running = False
        self.current_object_color = None
        print("⏹️ 搬运任务已停止")

class EnhancedCameraManager:
    """增强的摄像头管理器"""
    
    def __init__(self):
        self.camera = None
        self.camera_type = None
        self.frame_count = 0
        self.last_frame_time = time.time()
        self.fps = 0
        
    def initialize_camera(self):
        print("🔄 初始化摄像头...")
        
        # 尝试HiWonder摄像头
        if CAMERA_AVAILABLE:
            try:
                self.camera = Camera.Camera()
                self.camera.camera_open()
                time.sleep(2)
                frame = self.camera.frame
                if frame is not None and frame.size > 0:
                    self.camera_type = 'HiWonder'
                    print("✅ HiWonder摄像头初始化成功")
                    return True
            except Exception as e:
                print(f"❌ HiWonder摄像头初始化失败: {e}")
        
        # 尝试USB摄像头
        for i in range(3):
            try:
                camera = cv2.VideoCapture(i)
                if camera.isOpened():
                    ret, frame = camera.read()
                    if ret and frame is not None:
                        self.camera = camera
                        self.camera_type = f'USB_{i}'
                        print(f"✅ USB摄像头{i}初始化成功")
                        return True
                camera.release()
            except Exception as e:
                print(f"❌ USB摄像头{i}失败: {e}")
        
        print("❌ 所有摄像头初始化失败")
        return False
    
    def read_frame(self):
        if self.camera is None:
            return False, None
        
        try:
            if self.camera_type == 'HiWonder':
                frame = self.camera.frame
                ret = frame is not None and frame.size > 0
            else:
                ret, frame = self.camera.read()
            
            if ret and frame is not None:
                self.frame_count += 1
                current_time = time.time()
                if current_time - self.last_frame_time >= 1.0:
                    self.fps = self.frame_count / (current_time - self.last_frame_time)
                    self.frame_count = 0
                    self.last_frame_time = current_time
                
                return True, frame
            else:
                return False, None
                
        except Exception as e:
            print(f"❌ 摄像头读取错误: {e}")
            return False, None
    
    def close(self):
        if self.camera:
            try:
                if self.camera_type == 'HiWonder' and hasattr(self.camera, 'camera_close'):
                    self.camera.camera_close()
                else:
                    self.camera.release()
                print(f"✅ 摄像头已关闭")
            except Exception as e:
                print(f"❌ 摄像头关闭失败: {e}")

class VoiceControlSystem:
    """完整的语音控制系统（分步搬运模式）"""
    
    def __init__(self):
        print("🚀 初始化TonyPi完整语音控制系统...")
        
        # 初始化摄像头管理器
        self.camera_manager = EnhancedCameraManager()
        self.camera_available = self.camera_manager.initialize_camera()
        
        # 初始化检测器
        self.color_detector = SimpleColorDetector()
        self.face_detector = OriginalFaceDetector()
        self.tag_detector = AprilTagDetector()
        
        # 初始化搬运系统
        self.transport_system = TransportSystem(voice_callback=self.speak)
        
        # 初始化TTS（分离错误处理）
        self.tts_available = False
        if TTS_AVAILABLE:
            try:
                self.tts = TTS.TTS()
                self.tts.TTSModuleSpeak('[h0][v10][m3]', 'TTS测试')
                self.tts_available = True
                print("✅ TTS初始化和测试成功")
            except Exception as e:
                print(f"❌ TTS初始化失败: {e}")
        
        # 初始化ASR（优先）- 修改语音命令配置
        self.asr_available = False
        if ASR_AVAILABLE:
            try:
                self.asr = ASR.ASR()
                # 配置语音命令 - 修改为分步操作
                self.asr.eraseWords()
                self.asr.setMode(2)
                self.asr.addWords(1, 'kai shi')              # 开始/激活
                self.asr.addWords(2, 'ban yun')              # 搬运（进入搬运模式）
                self.asr.addWords(3, 'hong se')              # 红色
                self.asr.addWords(4, 'lv se')                # 绿色
                self.asr.addWords(5, 'lan se')               # 蓝色
                self.asr.addWords(6, 'ting zhi')             # 停止
                self.asr.addWords(7, 'tui chu')              # 退出
                self.asr.addWords(8, 'ren lian shi bie')     # 人脸识别
                self.asr.addWords(9, 'yan se shi bie')       # 颜色识别
                self.asr.addWords(10, 'biao qian shi bie')   # 标签识别
                self.asr_available = True
                print("✅ ASR系统初始化成功（分步搬运模式）")
            except Exception as e:
                print(f"❌ ASR初始化失败: {e}")
        
        # 设置检测器的语音回调
        self.color_detector.set_voice_callback(self.speak)
        self.face_detector.set_voice_callback(self.speak)
        self.tag_detector.set_voice_callback(self.speak)
        
        # 初始化Vosk（备用语音识别）- 修改命令映射
        self.vosk_available = False
        if VOSK_AVAILABLE:
            try:
                self.vosk_recognizer = VoskVoiceRecognizer()
                if self.vosk_recognizer.start_listening():
                    self.vosk_available = True
                    print("✅ Vosk语音识别初始化成功")
            except Exception as e:
                print(f"❌ Vosk初始化失败: {e}")
        
        # 初始化机器人
        if ACTION_CONTROL_AVAILABLE:
            try:
                print("🤖 初始化机器人动作...")
                AGC.runActionGroup('stand')
                
                # 初始化机器人头部位置
                print("🔄 初始化机器人头部位置...")
                try:
                    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path) if yaml_handle else {'servo1': 1500, 'servo2': 1500}
                    Board.setPWMServoPulse(1, servo_data['servo1'], 500)  # 垂直舵机
                    Board.setPWMServoPulse(2, servo_data['servo2'], 500)  # 水平舵机
                    time.sleep(1)
                    print(f"✅ 头部初始化完成: 水平={servo_data['servo2']}, 垂直={servo_data['servo1']}")
                except Exception as e:
                    print(f"❌ 头部初始化失败: {e}")
                
                print("✅ 机器人初始化完成")
            except Exception as e:
                print(f"❌ 机器人初始化失败: {e}")
        
        # 系统状态管理
        self.current_mode = "待机"
        self.transport_ready = False  # 搬运准备状态
        self.is_running = True
        self.vision_thread = None
        self.headless_mode = os.environ.get('DISPLAY') is None
        
        # 显示功能可用性
        print(f"\n📋 功能模块状态:")
        print(f"   颜色识别: ✅ 备用检测器")
        print(f"   人脸识别: ✅ 完整逻辑检测器")
        print(f"   标签识别: {'✅ AprilTag' if APRILTAG_AVAILABLE else '❌'}")
        print(f"   搬运功能: ✅ 分步搬运系统")
        print(f"   语音识别: {'✅ ASR' if self.asr_available else ''} {'✅ Vosk' if self.vosk_available else ''}")
        print(f"   语音合成: {'✅' if self.tts_available else '❌'}")
        print(f"   动作控制: {'✅' if ACTION_CONTROL_AVAILABLE else '❌'}")
        print(f"   摄像头: {'✅' if self.camera_available else '❌'}")
        
        print("✅ 完整语音控制系统初始化完成（分步搬运模式）")
    
    def speak(self, text):
        """语音播报函数"""
        print(f"🔊 语音播报: {text}")
        
        if self.tts_available:
            try:
                self.tts.TTSModuleSpeak('[h0][v10][m3]', text)
                return True
            except Exception as e:
                print(f"❌ TTS播报失败: {e}")
        
        print(f"💬 [模拟语音] {text}")
        return False
    
    def process_voice_commands(self):
        """处理语音命令"""
        # 处理ASR命令（优先）
        if self.asr_available:
            try:
                command = self.asr.getResult()
                if command:
                    self.handle_asr_command(command)
            except Exception as e:
                print(f"❌ ASR命令处理错误: {e}")
        
        # 处理Vosk命令（备用）
        if self.vosk_available:
            try:
                command = self.vosk_recognizer.get_result()
                if command:
                    self.handle_vosk_command(command)
            except Exception as e:
                print(f"❌ Vosk命令处理错误: {e}")
    
    def handle_asr_command(self, command):
        """处理ASR语音命令（分步搬运逻辑）"""
        print(f"🎯 ASR命令: {command}")
        
        if command == 1:  # 开始/激活
            self.speak('系统激活')
            
        elif command == 2:  # 搬运（进入搬运模式）
            self.enter_transport_mode()
            
        elif command == 3:  # 红色
            if self.transport_ready:
                self.start_transport_with_color('red')
            elif self.current_mode == "颜色识别":
                self.color_detector.set_target_color('red')
                self.speak("切换到红色检测")
            else:
                self.speak("请先说搬运")
                
        elif command == 4:  # 绿色
            if self.transport_ready:
                self.start_transport_with_color('green')
            elif self.current_mode == "颜色识别":
                self.color_detector.set_target_color('green')
                self.speak("切换到绿色检测")
            else:
                self.speak("请先说搬运")
                
        elif command == 5:  # 蓝色
            if self.transport_ready:
                self.start_transport_with_color('blue')
            elif self.current_mode == "颜色识别":
                self.color_detector.set_target_color('blue')
                self.speak("切换到蓝色检测")
            else:
                self.speak("请先说搬运")
                
        elif command == 6:  # 停止
            self.stop_current_mode()
            
        elif command == 7:  # 退出
            self.shutdown_system()
            
        elif command == 8:  # 人脸识别
            self.start_face_detection()
            
        elif command == 9:  # 颜色识别
            self.start_color_recognition()
            
        elif command == 10:  # 标签识别
            self.start_tag_detection()
    
    def handle_vosk_command(self, command):
        """处理Vosk语音命令（分步搬运逻辑）"""
        print(f"🎯 Vosk命令: {command}")
        
        if command == 1:  # 颜色识别
            self.start_color_recognition()
        elif command == 2:  # 人脸识别
            self.start_face_detection()
        elif command == 3:  # 标签识别
            self.start_tag_detection()
        elif command == 20:  # 搬运模式
            self.enter_transport_mode()
        elif 11 <= command <= 14:  # 颜色指令
            color_map = {11: 'red', 12: 'green', 13: 'blue', 14: 'yellow'}
            color_chinese_map = {11: '红色', 12: '绿色', 13: '蓝色', 14: '黄色'}
            
            if command in color_map:
                color = color_map[command]
                color_chinese = color_chinese_map[command]
                
                if self.transport_ready:
                    # 在搬运模式下，执行搬运
                    if color in ['red', 'green', 'blue']:  # 只支持RGB搬运
                        self.start_transport_with_color(color)
                    else:
                        self.speak("搬运模式不支持黄色")
                elif self.current_mode == "颜色识别":
                    # 在颜色识别模式下，切换检测颜色
                    if self.color_detector.set_target_color(color):
                        self.speak(f"切换到{color_chinese}检测")
                else:
                    # 其他情况提示先进入对应模式
                    self.speak("请先进入搬运模式或颜色识别模式")
                    
        elif command == 4:  # 停止
            self.stop_current_mode()
        elif command == 5:  # 退出
            self.shutdown_system()
        elif command == 9:  # 挥手测试
            self.test_face_action()
    
    def enter_transport_mode(self):
        """进入搬运准备模式"""
        if not self.camera_available:
            self.speak("摄像头不可用")
            return
            
        self.stop_current_mode()
        self.transport_ready = True
        self.current_mode = "搬运准备"
        
        print("🚛 进入搬运准备模式")
        self.speak("进入搬运模式，请说要搬运的颜色：红色、绿色或蓝色")
        
        # 启动搬运准备界面
        self.vision_thread = threading.Thread(target=self.transport_ready_loop)
        self.vision_thread.daemon = True
        self.vision_thread.start()
    
    def start_transport_with_color(self, color):
        """使用指定颜色开始搬运"""
        if not self.transport_ready:
            self.speak("请先说搬运进入搬运模式")
            return
            
        self.transport_ready = False
        self.current_mode = "搬运执行"
        
        color_chinese = self.transport_system.color_names_chinese[color]
        print(f"🚛 开始执行搬运: {color_chinese}")
        self.speak(f"开始搬运{color_chinese}方块")
        
        if self.transport_system.start_transport(color):
            # 重新启动线程进入执行模式
            if self.vision_thread and self.vision_thread.is_alive():
                pass  # 线程会自动切换到执行模式
        else:
            self.speak("搬运启动失败")
            self.transport_ready = True
            self.current_mode = "搬运准备"
    
    def transport_ready_loop(self):
        """搬运准备模式循环"""
        print("🚛 搬运准备模式运行中...")
        
        frame_count = 0
        last_status_time = time.time()
        
        while self.current_mode in ["搬运准备", "搬运执行"] and self.is_running:
            try:
                ret, img = self.camera_manager.read_frame()
                if ret and img is not None:
                    frame_count += 1
                    
                    # 应用畸变矫正（如果可用）
                    if CALIBRATION_AVAILABLE:
                        img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
                    
                    if self.current_mode == "搬运准备":
                        # 搬运准备状态 - 显示等待界面
                        cv2.putText(img, 'Transport Mode Ready', (10, 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.putText(img, 'Say color: Red, Green, Blue', (10, 70), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.putText(img, 'Frame: {}'.format(frame_count), (10, 100), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        
                        if self.headless_mode:
                            current_time = time.time()
                            if current_time - last_status_time >= 5.0:
                                print("🚛 搬运准备状态: 等待颜色指令")
                                print("   可说：红色、绿色、蓝色")
                                last_status_time = current_time
                        else:
                            cv2.imshow('Transport System', img)
                            key = cv2.waitKey(1) & 0xFF
                            if key == 27:  # ESC
                                break
                            elif key == ord('1'):  # 红色
                                self.start_transport_with_color('red')
                            elif key == ord('2'):  # 绿色
                                self.start_transport_with_color('green')
                            elif key == ord('3'):  # 蓝色
                                self.start_transport_with_color('blue')
                            elif key == ord('s'):  # 停止
                                self.stop_current_mode()
                                
                    elif self.current_mode == "搬运执行":
                        # 搬运执行状态 - 处理搬运逻辑
                        result_frame = self.transport_system.process_frame(img)
                        
                        # 添加搬运状态显示
                        stage = "搬运阶段" if self.transport_system.find_box else "放置阶段"
                        if self.transport_system.current_object_color:
                            color_chinese = self.transport_system.color_names_chinese[self.transport_system.current_object_color]
                            cv2.putText(result_frame, f'{stage}: {color_chinese}', (10, 30), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(result_frame, f'Step: {self.transport_system.step}', (10, 60), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        # 检查搬运是否完成
                        if not self.transport_system.current_object_color and not self.transport_system.is_running:
                            print("🚛 搬运任务完成，返回准备模式")
                            self.transport_ready = True
                            self.current_mode = "搬运准备"
                            self.speak("搬运完成，可继续说颜色进行下一次搬运")
                        
                        if self.headless_mode:
                            current_time = time.time()
                            if current_time - last_status_time >= 3.0:
                                print(f"🚛 搬运状态: {stage}")
                                if self.transport_system.current_object_color:
                                    print(f"   目标颜色: {color_chinese}")
                                    print(f"   步骤: {self.transport_system.step}")
                                last_status_time = current_time
                        else:
                            cv2.imshow('Transport System', result_frame)
                            key = cv2.waitKey(1) & 0xFF
                            if key == 27:  # ESC
                                break
                            elif key == ord('s'):  # 停止
                                self.stop_current_mode()
                else:
                    print("❌ 无法读取摄像头图像")
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"❌ 搬运循环错误: {e}")
                time.sleep(0.1)
        
        if not self.headless_mode:
            cv2.destroyWindow('Transport System')
        print("🚛 搬运模式结束")
    
    def test_face_action(self):
        """测试人脸动作"""
        print("🧪 === 直接测试人脸动作 ===")
        self.speak("开始测试人脸动作")
        self.face_detector.test_action()
    
    def start_color_recognition(self):
        """启动颜色识别"""
        if not self.camera_available:
            self.speak("摄像头不可用")
            return
            
        if self.current_mode == "颜色识别":
            self.speak("已在颜色识别模式")
            return
            
        self.stop_current_mode()
        self.current_mode = "颜色识别"
        print("🎨 启动颜色识别")
        self.speak("开始颜色识别，可说红色、绿色、蓝色、黄色切换检测目标")
        
        self.vision_thread = threading.Thread(target=self.color_recognition_loop)
        self.vision_thread.daemon = True
        self.vision_thread.start()
    
    def start_face_detection(self):
        """启动人脸识别"""
        if not self.camera_available:
            self.speak("摄像头不可用")
            return
            
        if self.current_mode == "人脸识别":
            self.speak("已在人脸识别模式")
            return
            
        self.stop_current_mode()
        self.current_mode = "人脸识别"
        print("👤 启动人脸识别")
        self.speak("开始人脸识别")
        
        self.face_detector.start_detection()
        
        self.vision_thread = threading.Thread(target=self.face_detection_loop)
        self.vision_thread.daemon = True
        self.vision_thread.start()
    
    def start_tag_detection(self):
        """启动标签识别"""
        if not self.camera_available:
            self.speak("摄像头不可用")
            return
            
        if self.current_mode == "标签识别":
            self.speak("已在标签识别模式")
            return
            
        self.stop_current_mode()
        self.current_mode = "标签识别"
        print("🏷️  启动标签识别")
        self.speak("开始标签识别")
        
        self.vision_thread = threading.Thread(target=self.tag_detection_loop)
        self.vision_thread.daemon = True
        self.vision_thread.start()
    
    def color_recognition_loop(self):
        """颜色识别循环"""
        print("🎨 颜色识别运行中...")
        
        frame_count = 0
        last_status_time = time.time()
        
        while self.current_mode == "颜色识别" and self.is_running:
            try:
                ret, img = self.camera_manager.read_frame()
                if ret and img is not None:
                    frame_count += 1
                    
                    result_frame = self.color_detector.detect_color(img)
                    
                    if result_frame is not None:
                        if self.headless_mode:
                            current_time = time.time()
                            if current_time - last_status_time >= 5.0:
                                color_chinese = self.color_detector.color_names_chinese.get(
                                    self.color_detector.current_target_color, 
                                    self.color_detector.current_target_color)
                                print(f"🎨 颜色识别状态: 检测{color_chinese}")
                                last_status_time = current_time
                        else:
                            cv2.imshow('Color Recognition', result_frame)
                            key = cv2.waitKey(1) & 0xFF
                            if key == 27:  # ESC
                                break
                            elif key == ord('1'):  # 红色
                                self.color_detector.set_target_color('red')
                                self.speak("切换到红色检测")
                            elif key == ord('2'):  # 绿色
                                self.color_detector.set_target_color('green')
                                self.speak("切换到绿色检测")
                            elif key == ord('3'):  # 蓝色
                                self.color_detector.set_target_color('blue')
                                self.speak("切换到蓝色检测")
                            elif key == ord('4'):  # 黄色
                                self.color_detector.set_target_color('yellow')
                                self.speak("切换到黄色检测")
                else:
                    print("❌ 无法读取摄像头图像")
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"❌ 颜色识别错误: {e}")
                time.sleep(0.1)
        
        if not self.headless_mode:
            cv2.destroyWindow('Color Recognition')
        print("🎨 颜色识别结束")
    
    def face_detection_loop(self):
        """人脸识别循环"""
        print("👤 人脸识别运行中...")
        
        frame_count = 0
        last_status_time = time.time()
        
        while self.current_mode == "人脸识别" and self.is_running:
            try:
                ret, img = self.camera_manager.read_frame()
                if ret and img is not None:
                    frame_count += 1
                    
                    result_frame = self.face_detector.detect_faces(img)
                    
                    if result_frame is not None:
                        if self.headless_mode:
                            current_time = time.time()
                            if current_time - last_status_time >= 3.0:
                                print(f"👤 人脸识别状态: 运行中")
                                last_status_time = current_time
                        else:
                            cv2.imshow('Face Detection', result_frame)
                            key = cv2.waitKey(1) & 0xFF
                            if key == 27:  # ESC
                                break
                            elif key == ord('t'):  # 按T测试动作
                                self.face_detector.test_action()
                else:
                    print("❌ 无法读取摄像头图像")
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"❌ 人脸识别错误: {e}")
                time.sleep(0.1)
        
        if not self.headless_mode:
            cv2.destroyWindow('Face Detection')
        
        self.face_detector.stop_detection()
        print("👤 人脸识别结束")
    
    def tag_detection_loop(self):
        """标签识别循环"""
        print("🏷️  标签识别运行中...")
        
        frame_count = 0
        last_status_time = time.time()
        
        while self.current_mode == "标签识别" and self.is_running:
            try:
                ret, img = self.camera_manager.read_frame()
                if ret and img is not None:
                    frame_count += 1
                    
                    result_frame = self.tag_detector.detect_tags(img)
                        
                    if result_frame is not None:
                        if self.headless_mode:
                            current_time = time.time()
                            if current_time - last_status_time >= 3.0:
                                print(f"🏷️  标签识别状态: 运行中")
                                last_status_time = current_time
                        else:
                            cv2.imshow('Tag Detection', result_frame)
                            key = cv2.waitKey(1) & 0xFF
                            if key == 27:  # ESC
                                break
                else:
                    print("❌ 无法读取摄像头图像")
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"❌ 标签识别错误: {e}")
                time.sleep(0.1)
        
        if not self.headless_mode:
            cv2.destroyWindow('Tag Detection')
        print("🏷️  标签识别结束")
    
    def stop_current_mode(self):
        """停止当前模式"""
        if self.current_mode != "待机":
            print(f"⏹️  停止{self.current_mode}")
            self.speak(f"停止{self.current_mode}")
            
            if self.current_mode == "人脸识别":
                self.face_detector.stop_detection()
            elif self.current_mode in ["搬运准备", "搬运执行"]:
                self.transport_system.stop_transport()
                self.transport_ready = False
            
            self.current_mode = "待机"
    
    def shutdown_system(self):
        """关闭系统"""
        print("🔄 关闭系统...")
        self.is_running = False
        self.transport_ready = False
        
        self.stop_current_mode()
        
        if self.camera_manager:
            self.camera_manager.close()
        
        if self.vosk_available:
            try:
                self.vosk_recognizer.stop_listening()
                print("✅ Vosk语音识别已停止")
            except Exception as e:
                print(f"❌ Vosk停止失败: {e}")
        
        if ACTION_CONTROL_AVAILABLE:
            try:
                AGC.runActionGroup('stand_slow')
                print("✅ 机器人复位完成")
            except Exception as e:
                print(f"❌ 机器人复位失败: {e}")
        
        cv2.destroyAllWindows()
        self.speak('系统已关闭')
        print("✅ 系统已关闭")
    
    def run(self):
        """运行主程序"""
        print("="*70)
        print("🚀 TonyPi完整语音控制系统启动（分步搬运模式）")
        print("="*70)
        
        print("\n🎤 支持的语音命令:")
        print("   📢 'kai shi' - 激活系统")
        print("   🚛 'ban yun' - 进入搬运模式")
        print("   🔴 'hong se' - 红色（搬运模式下执行搬运红色）")
        print("   🟢 'lv se' - 绿色（搬运模式下执行搬运绿色）")
        print("   🔵 'lan se' - 蓝色（搬运模式下执行搬运蓝色）")
        print("   👤 'ren lian shi bie' - 人脸识别模式")
        print("   🎨 'yan se shi bie' - 颜色识别模式")
        print("   🏷️  'biao qian shi bie' - 标签识别模式")
        print("   ⏹️  'ting zhi' - 停止当前功能")
        print("   🚪 'tui chu' - 退出系统")
        
        print("\n🎯 使用流程:")
        print("   1️⃣  说'搬运' → 进入搬运准备模式")
        print("   2️⃣  说'红色/绿色/蓝色' → 开始搬运对应颜色")
        print("   3️⃣  搬运完成后自动回到准备模式，可继续搬运")
        
        # 系统就绪提示
        self.speak('语音控制系统就绪')
        
        try:
            while self.is_running:
                # 处理语音命令
                self.process_voice_commands()
                
                # 非视觉模式下的基本等待
                if self.current_mode == "待机":
                    time.sleep(0.1)
                else:
                    time.sleep(0.01)
                    
        except KeyboardInterrupt:
            print("\n👋 用户中断")
        except Exception as e:
            print(f"❌ 系统运行错误: {e}")
        finally:
            self.shutdown_system()

# 完整的Vosk语音识别器类
class VoskVoiceRecognizer:
    def __init__(self, model_path="/home/pi/TonyPi/vosk-model-small-cn"):
        self.model_path = model_path
        self.is_listening = False
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.audio_stream = None
        self.recognizer = None
        self.model = None
        
        self.CHUNK = 4096
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        
        # 更新命令映射 - 分步搬运
        self.command_map = {
            "颜色识别": 1, "色彩识别": 1, "颜色检测": 1, "识别颜色": 1,
            "颜色": 1, "色彩": 1, "检测颜色": 1,
            
            "人脸识别": 2, "面部识别": 2, "脸部检测": 2, "人脸检测": 2,
            "人脸": 2, "面部": 2, "脸部": 2, "检测人脸": 2,
            
            "标签识别": 3, "tag识别": 3, "二维码": 3, "条码识别": 3,
            "标签": 3, "tag": 3,
            
            "搬运": 20, "搬运模式": 20, "开始搬运": 20, "进入搬运": 20,
            
            "红色": 11, "绿色": 12, "蓝色": 13, "黄色": 14,
            "红": 11, "绿": 12, "蓝": 13, "黄": 14,
            
            "停止": 4, "结束": 4, "暂停": 4, "停": 4,
            "退出": 5, "关闭": 5, "结束程序": 5, "退": 5, "关": 5,
            
            "状态": 6, "帮助": 7, "测试": 8, "挥手": 9
        }
        
    def load_model(self):
        try:
            if not os.path.exists(self.model_path):
                print(f"❌ Vosk模型路径不存在: {self.model_path}")
                return False
                
            print("🔄 加载Vosk模型...")
            self.model = vosk.Model(self.model_path)
            self.recognizer = vosk.KaldiRecognizer(self.model, self.RATE)
            print("✅ Vosk模型加载成功")
            return True
            
        except Exception as e:
            print(f"❌ 加载Vosk模型失败: {e}")
            return False
    
    def init_audio(self):
        try:
            print("🎤 初始化音频设备...")
            self.p = pyaudio.PyAudio()
            self.audio_stream = self.p.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK,
                stream_callback=self._audio_callback
            )
            print("✅ 音频设备初始化成功")
            return True
        except Exception as e:
            print(f"❌ 音频设备初始化失败: {e}")
            return False
    
    def _audio_callback(self, in_data, frame_count, time_info, status):
        if self.is_listening:
            self.audio_queue.put(in_data)
        return (None, pyaudio.paContinue)
    
    def _recognition_thread(self):
        while self.is_listening:
            try:
                if not self.audio_queue.empty():
                    audio_data = self.audio_queue.get()
                    if self.recognizer.AcceptWaveform(audio_data):
                        result = json.loads(self.recognizer.Result())
                        text = result.get('text', '').strip()
                        if text:
                            print(f"🎯 识别到语音: '{text}'")
                            command_id = self._match_command(text)
                            if command_id:
                                print(f"✅ 匹配到命令ID: {command_id}")
                                self.result_queue.put(command_id)
                            else:
                                print(f"❓ 未匹配到命令")
                time.sleep(0.01)
            except Exception as e:
                print(f"❌ 语音识别线程错误: {e}")
                time.sleep(0.1)
    
    def _match_command(self, text):
        # 原始文本匹配
        for command, cmd_id in self.command_map.items():
            if command in text:
                return cmd_id
        
        # 去除空格后匹配
        text_no_space = re.sub(r'\s+', '', text)
        for command, cmd_id in self.command_map.items():
            command_no_space = re.sub(r'\s+', '', command)
            if command_no_space in text_no_space:
                return cmd_id
        
        # 部分关键词匹配
        if "颜色" in text or "色彩" in text:
            return 1
        elif "人脸" in text or "脸" in text or "面部" in text:
            return 2
        elif "标签" in text or "tag" in text.lower():
            return 3
        elif "搬运" in text:
            return 20
        elif "红" in text:
            return 11
        elif "绿" in text:
            return 12
        elif "蓝" in text:
            return 13
        elif "黄" in text:
            return 14
        elif "停" in text or "结束" in text:
            return 4
        elif "退" in text or "关" in text:
            return 5
        elif "挥手" in text or "挥" in text:
            return 9
        
        return None
    
    def start_listening(self):
        if not VOSK_AVAILABLE:
            return False
        if not self.load_model():
            return False
        if not self.init_audio():
            return False
        
        self.is_listening = True
        self.audio_stream.start_stream()
        
        self.recognition_thread = threading.Thread(target=self._recognition_thread)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()
        
        print("🎤 Vosk语音识别开始监听...")
        return True
    
    def stop_listening(self):
        self.is_listening = False
        if self.audio_stream:
            self.audio_stream.stop_stream()
            self.audio_stream.close()
        if hasattr(self, 'p'):
            self.p.terminate()
    
    def get_result(self):
        if not self.result_queue.empty():
            return self.result_queue.get()
        return None

if __name__ == '__main__':
    # 创建并运行完整语音控制系统（分步搬运模式）
    system = VoiceControlSystem()
    system.run()