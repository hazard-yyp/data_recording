import rospy
import torch
import torch.nn as nn
from torchvision import transforms
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import os
import subprocess
import numpy as np
from PIL import Image
import argparse
import tkinter as tk
from PIL import Image, ImageTk

# 定义CNN模型（与训练时的模型保持一致）
class SimpleCNN(nn.Module):
    def __init__(self):
        super(SimpleCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        self.fc1 = nn.Linear(64 * 8 * 8, 128)
        self.fc2 = nn.Linear(128, 2)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = self.relu(self.conv1(x))
        x = self.pool(x)
        x = self.relu(self.conv2(x))
        x = self.pool(x)
        x = x.view(-1, 64 * 8 * 8)
        x = self.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class ImageClassifierNode:
    def __init__(self, model_path, topic, rosbag_path, rosbag_topics):
        self.bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # 加载模型
        self.model = SimpleCNN().to(self.device)
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()
        
        self.transform = transforms.Compose([
            transforms.Resize((32, 32)),
            transforms.ToTensor(),
        ])
        
        self.recording = False
        self.process = None
        self.rosbag_path = rosbag_path
        self.rosbag_topics = rosbag_topics
        
        # 初始化Tkinter窗口
        self.root = tk.Tk()
        self.root.title("Image Classifier Node")
        
        self.label = tk.Label(self.root, text="Waiting for image...", font=("Helvetica", 16))
        self.label.pack(pady=20)
        
        self.canvas = tk.Canvas(self.root, width=1536, height=1024)
        self.canvas.pack()
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # 初始化ROS节点
        rospy.init_node('image_classifier_node', anonymous=True)
        rospy.Subscriber(topic, CompressedImage, self.callback)
        
        # 启动Tkinter主循环
        self.root.mainloop()

    def callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # 更新Tkinter GUI
            img_pil = Image.fromarray(image_rgb)
            img_tk = ImageTk.PhotoImage(img_pil)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=img_tk)
            self.canvas.image = img_tk
            
            # 图像预处理
            image_tensor = self.transform(img_pil)
            image_tensor = image_tensor.unsqueeze(0).to(self.device)
            
            # 推理
            with torch.no_grad():
                outputs = self.model(image_tensor)
                _, predicted = torch.max(outputs.data, 1)
                label = 'land_side' if predicted.item() == 0 else 'sea_side'
                rospy.loginfo(f'Predicted: {label}')
                self.label.config(text=f'Predicted: {label}')
                
                if label == 'sea_side' and not self.recording:
                    self.start_recording()
                elif label == 'land_side' and self.recording:
                    self.stop_recording()
                    
        except Exception as e:
            rospy.logerr(f'Error in callback: {str(e)}')

    def start_recording(self):
        rospy.loginfo("Starting rosbag recording...")
        if not os.path.exists(self.rosbag_path):
            os.makedirs(self.rosbag_path)
        self.process = subprocess.Popen(['rosbag', 'record', '-o', f'{self.rosbag_path}/recording'] + self.rosbag_topics + ['--lz4'])
        self.recording = True

    def stop_recording(self):
        if self.process:
            rospy.loginfo("Stopping rosbag recording...")
            self.process.terminate()
            self.process.wait()
            self.recording = False

    def on_closing(self):
        rospy.signal_shutdown("GUI closed")
        self.root.destroy()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ROS Image Classifier Node')
    parser.add_argument('--model_path', type=str, required=True, help='Path to the model file')
    parser.add_argument('--topic', type=str, required=True, help='Topic to subscribe to')
    parser.add_argument('--rosbag_path', type=str, required=True, help='Path to save rosbag files')
    parser.add_argument('--rosbag_topics', nargs='+', required=True, help='Topics to record in rosbag')

    args = parser.parse_args()
    
    try:
        ImageClassifierNode(args.model_path, args.topic, args.rosbag_path, args.rosbag_topics)
    except rospy.ROSInterruptException:
        pass
