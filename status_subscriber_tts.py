import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class StatusSubscriberTTS(Node):
    def __init__(self):
        super().__init__('status_subscriber_tts')
        # 创建订阅者，监听 'patient_status' 话题
        self.subscription = self.create_subscription(
            String,
            'patient_status',
            self.listener_callback,
            10
        )
        self.get_logger().info('TTS 订阅节点已启动，等待接收状态消息...')

    def listener_callback(self, msg):
        text_content = msg.data
        self.get_logger().info(f'接收到文字内容: "{text_content}"，正在转换为语音...')
        
        try:
            audio_filename = "patient_status.mp3"
            
            # 1. 调用 edge-tts 生成音频文件
            # 注意：这里的 /usr/local/bin/edge-tts 请确保是你之前通过 which edge-tts 查到的绝对路径
            # 如果你的路径不同（例如在 /root/.local/bin/edge-tts），请自行替换。
            tts_command = f'/usr/local/bin/edge-tts --text "{text_content}" --voice zh-CN-XiaoxiaoNeural --write-media {audio_filename}'
            os.system(tts_command)
            
            # 2. 使用 mplayer 播放音频
            # -nolirc: 禁用红外遥控（防 socket 报错）
            # -vo null: 禁用视频输出（防图形界面报错）
            # -ao alsa:device=hw=1.0: 【核心配置】强制将声音输出到 2号声卡(duplex-audio)的模拟耳机孔
            # -really-quiet: 屏蔽播放器多余日志
            play_command = f"/usr/bin/mplayer -nolirc -vo null -ao alsa:device=hw=1.0 -really-quiet {audio_filename}"
            os.system(play_command)
            
            # 3. 播放完成后清理临时音频文件
            if os.path.exists(audio_filename):
                os.remove(audio_filename)
                
            self.get_logger().info('语音播报完成。')
            
        except Exception as e:
            self.get_logger().error(f'语音转换或播放过程中发生错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = StatusSubscriberTTS()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()