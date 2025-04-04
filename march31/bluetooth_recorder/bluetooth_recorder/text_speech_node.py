import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import tempfile
from playsound import playsound

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String,
            '/text_speech',
            self.listener_callback,
            10
        )
        self.get_logger().info("gTTS-based TTS node started.")

    def listener_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Received message: {text}")
        # Generate speech using gTTS
        tts = gTTS(text=text, lang='en')
        # Save to a temporary file
        with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as fp:
            temp_filename = fp.name
            tts.save(temp_filename)
        # Play the audio file
        playsound(temp_filename)
        # Clean up the temporary file
        os.remove(temp_filename)

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TTS node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
