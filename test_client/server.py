import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import paho.mqtt.client as mqtt
import json
import base64
import io
import os
import ssl
import queue
import threading
from datetime import datetime
from PIL import Image, ImageTk

# ==========================================
# MQTT 配置参数与存储路径
# ==========================================

#这边使用的是我的EMQX服务器，大家可以替换成自己的服务器地址和认证信息
MQTT_HOST = "z9c1fa31.ala.cn-hangzhou.emqxsl.cn"
MQTT_PORT = 8883
MQTT_SUB_TOPIC = "rk3566/ai_events"
MQTT_PUB_TOPIC = "rk3566/cmd"
MQTT_USER = "windows11"
MQTT_PWD = "windows11"
CA_CERT_PATH = "emqxsl-ca.crt"

# 本地保存图片的目录
SAVE_DIR = "saved_images"

# ==========================================
# 支持的命令字典：{命令名: 是否需要参数(val)}
# ==========================================
COMMANDS = {
    "set_timeout": True,      # 设置 AI 唤醒时长 (秒)
    "set_threshold": True,    # 设置置信度阈值 (0.0~1.0)
    "snapshot": False,        # 强制抓拍
    "clear_storage": False    # 清理 RK3566 的本地 SD 卡
}

class SmartTerminalApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RK3566 AI 智能网关 - 远程控制台")
        self.root.geometry("1000x700")
        
        # 确保本地图片保存目录存在
        if not os.path.exists(SAVE_DIR):
            os.makedirs(SAVE_DIR)
        
        # 线程安全的消息队列，用于接收 MQTT 传来的 JSON
        self.msg_queue = queue.Queue()
        
        # 构建 UI
        self.setup_ui()
        
        # 初始化 MQTT
        self.mqtt_client = mqtt.Client(client_id="win11_gui_client")
        self.setup_mqtt()
        
        # 开启 UI 定时器，轮询消息队列
        self.root.after(100, self.process_queue)
        
        # 处理关闭窗口事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_ui(self):
        # 左右分栏
        left_frame = tk.Frame(self.root, width=640, padx=10, pady=10)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        right_frame = tk.Frame(self.root, width=340, padx=10, pady=10)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y)

        # ====== 左侧：画面与数据 ======
        # 数据显示栏
        self.data_label = tk.Label(left_frame, text="等待接收数据...", font=("Microsoft YaHei", 12, "bold"), fg="blue")
        self.data_label.pack(anchor=tk.W, pady=(0, 10))

        # 图像显示区 (预设一张黑底)
        self.img_label = tk.Label(left_frame, bg="black", width=80, height=25)
        self.img_label.pack(fill=tk.BOTH, expand=True)
        
        # ====== 右侧：控制与日志 ======
        tk.Label(right_frame, text="[ 云端反向控制 ]", font=("Microsoft YaHei", 12, "bold")).pack(pady=(0, 15))
        
        # 命令选择下拉框
        tk.Label(right_frame, text="选择指令:").pack(anchor=tk.W)
        self.cmd_var = tk.StringVar()
        self.cmd_combo = ttk.Combobox(right_frame, textvariable=self.cmd_var, state="readonly")
        self.cmd_combo['values'] = list(COMMANDS.keys())
        self.cmd_combo.current(0)
        self.cmd_combo.pack(fill=tk.X, pady=(0, 10))
        self.cmd_combo.bind("<<ComboboxSelected>>", self.on_cmd_change)

        # 参数输入框
        self.val_label = tk.Label(right_frame, text="参数值 (val):")
        self.val_label.pack(anchor=tk.W)
        self.val_var = tk.StringVar()
        self.val_entry = ttk.Entry(right_frame, textvariable=self.val_var)
        self.val_entry.pack(fill=tk.X, pady=(0, 15))

        # 发送按钮
        self.send_btn = tk.Button(right_frame, text="🚀 发送指令", bg="green", fg="white", font=("Microsoft YaHei", 11), command=self.send_command)
        self.send_btn.pack(fill=tk.X, pady=(0, 20))

        # 日志区 (带清空按钮的头部框)
        log_title_frame = tk.Frame(right_frame)
        log_title_frame.pack(fill=tk.X, pady=(10, 5))
        tk.Label(log_title_frame, text="运行日志:").pack(side=tk.LEFT)
        # 【新增】：清空日志按钮
        tk.Button(log_title_frame, text="🗑️ 清空", font=("Microsoft YaHei", 8), command=self.clear_log).pack(side=tk.RIGHT)

        self.log_area = scrolledtext.ScrolledText(right_frame, width=40, height=20, state='disabled', font=("Consolas", 9))
        self.log_area.pack(fill=tk.BOTH, expand=True)
        
        self.log(f"程序启动，正在初始化界面... (图片将保存在 {SAVE_DIR}/ 目录)")
        self.on_cmd_change(None) 

    def on_cmd_change(self, event):
        """当下拉框改变时，决定是否禁用 val 输入框"""
        cmd = self.cmd_var.get()
        needs_val = COMMANDS.get(cmd, False)
        if needs_val:
            self.val_entry.config(state="normal")
            self.val_label.config(fg="black")
        else:
            self.val_var.set("")
            self.val_entry.config(state="disabled")
            self.val_label.config(fg="gray")

    def send_command(self):
        """发送 MQTT 控制指令"""
        cmd = self.cmd_var.get()
        needs_val = COMMANDS.get(cmd, False)
        
        payload = {"cmd": cmd}
        if needs_val:
            val_str = self.val_var.get()
            try:
                payload["val"] = float(val_str)
            except ValueError:
                messagebox.showerror("参数错误", "该指令需要输入一个数字类型的参数！")
                return

        payload_json = json.dumps(payload)
        
        # 发布消息
        result = self.mqtt_client.publish(MQTT_PUB_TOPIC, payload_json, qos=1)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            self.log(f"📤 发送指令 -> {payload_json}")
        else:
            self.log(f"❌ 发送失败，错误码: {result.rc}")

    def clear_log(self):
        """【新增】：清空运行日志内容"""
        self.log_area.config(state='normal')
        self.log_area.delete('1.0', tk.END)
        self.log_area.config(state='disabled')

    def log(self, msg):
        """向日志框追加信息"""
        self.log_area.config(state='normal')
        time_str = datetime.now().strftime("%H:%M:%S")
        self.log_area.insert(tk.END, f"[{time_str}] {msg}\n")
        self.log_area.see(tk.END)
        self.log_area.config(state='disabled')

    # ====== MQTT 与 异步数据处理 ======
    def setup_mqtt(self):
        self.mqtt_client.username_pw_set(MQTT_USER, MQTT_PWD)
        try:
            self.mqtt_client.tls_set(ca_certs=CA_CERT_PATH, tls_version=ssl.PROTOCOL_TLS)
        except Exception as e:
            self.log(f"❌ TLS 证书错误: {e}")
            messagebox.showerror("证书缺失", f"找不到证书文件 {CA_CERT_PATH}\n请确保它与此脚本在同一目录！")
            return

        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        self.log(f"正在连接 EMQX 服务器 {MQTT_HOST}...")
        threading.Thread(target=self.mqtt_connect_thread, daemon=True).start()

    def mqtt_connect_thread(self):
        try:
            self.mqtt_client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
            self.mqtt_client.loop_start() 
        except Exception as e:
            self.root.after(0, self.log, f"❌ 网络连接失败: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.root.after(0, self.log, "✅ 成功连接到 MQTT Broker！")
            client.subscribe(MQTT_SUB_TOPIC, qos=1)
            self.root.after(0, self.log, f"🎧 已订阅主题: {MQTT_SUB_TOPIC}")
        else:
            self.root.after(0, self.log, f"❌ 连接失败，返回码: {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload_str = msg.payload.decode('utf-8')
            data = json.loads(payload_str)
            self.msg_queue.put(data)
        except Exception as e:
            print(f"解析消息失败: {e}")

    def save_image_async(self, img_bytes, timestamp, target_count):
        """【新增】：异步保存图片到本地硬盘，绝不阻塞 UI 主线程"""
        def save_task():
            try:
                # 生成带时间戳的文件名
                if timestamp > 0:
                    dt_str = datetime.fromtimestamp(timestamp).strftime("%Y%m%d_%H%M%S")
                else:
                    dt_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                    
                filename = f"cap_{dt_str}_tgt_{target_count}.jpg"
                filepath = os.path.join(SAVE_DIR, filename)
                
                # 写入硬盘
                with open(filepath, "wb") as f:
                    f.write(img_bytes)
                    
                print(f"[后台保存] 图片已落盘: {filepath}")
            except Exception as e:
                print(f"[后台保存出错] {e}")

        # 启动一个守护线程去写硬盘，写完它会自动销毁
        threading.Thread(target=save_task, daemon=True).start()

    def process_queue(self):
        """UI 线程定时从队列取数据并刷新界面"""
        try:
            while not self.msg_queue.empty():
                data = self.msg_queue.get_nowait()
                
                timestamp = data.get("timestamp", 0)
                tc = data.get("target_count", 0)
                temp = data.get("temp", 0.0)
                humi = data.get("humi", 0.0)
                co2 = data.get("co2", 0)
                tvoc = data.get("tvoc", 0)
                b64_img = data.get("image", "")

                # 1. 更新顶部文本数据
                dt_str = datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S") if timestamp > 0 else "N/A"
                info_text = (f"时间: {dt_str} | 人数: {tc} | "
                             f"🌡️ {temp:.1f}℃ | 💧 {humi:.1f}% | "
                             f"CO2: {co2}ppm | TVOC: {tvoc}ppb")
                self.data_label.config(text=info_text)
                
                # 记录日志
                self.log(f"📥 收到上报事件，画面人数: {tc}")

                # 2. 解码、显示并异步保存图像
                if b64_img:
                    if b64_img.startswith("data:image/jpeg;base64,"):
                        b64_img = b64_img.split(",", 1)[1]
                    
                    # Base64 解码非常快，放在 UI 线程没问题
                    img_bytes = base64.b64decode(b64_img)
                    
                    # ---- 【新增】甩给后台线程去保存到硬盘 ----
                    self.save_image_async(img_bytes, timestamp, tc)
                    
                    # ---- 前台继续渲染内存中的画面 ----
                    img = Image.open(io.BytesIO(img_bytes))
                    img.thumbnail((720, 480))
                    photo = ImageTk.PhotoImage(img)
                    
                    self.img_label.config(image=photo)
                    self.img_label.image = photo 

        except Exception as e:
            self.log(f"❌ 刷新界面时发生错误: {e}")
            
        # 安排下一次轮询
        self.root.after(100, self.process_queue)

    def on_closing(self):
        """窗口关闭时的清理工作"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SmartTerminalApp(root)
    root.mainloop()