#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        # Publisher for recognized text on "speech_text" topic
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        # Subscriber to "audio_convert" topic, which triggers audio processing when a "1" is received
        self.subscription = self.create_subscription(
            String,
            'audio_convert',
            self.audio_convert_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info("SpeechToTextNode started. Waiting for audio conversion signal...")

    def audio_convert_callback(self, msg: String):
        if msg.data.strip() == "1":
            self.get_logger().info("Received audio conversion signal. Processing recorded audio...")
            recognized_text = self.speech_to_text_from_file("/home/medially/Medially_ws/audio/recorded_audio.wav")
            if recognized_text:
                publish_msg = String()
                publish_msg.data = recognized_text
                self.publisher_.publish(publish_msg)
                self.get_logger().info(f"Published recognized text to /speech_text: {recognized_text}")
            else:
                self.get_logger().warn("No text was recognized or an error occurred during processing.")

    def speech_to_text_from_file(self, audio_file_path: str):
        """Converts speech from an audio file to text using Google Speech Recognition."""
        recognizer = sr.Recognizer()
        try:
            with sr.AudioFile(audio_file_path) as source:
                audio_data = recognizer.record(source)
            # Use Google Speech Recognition to transcribe the audio
            text = recognizer.recognize_google(audio_data)
            self.get_logger().info(f"Recognized text: {text}")
            return text

        except sr.UnknownValueError:
            self.get_logger().error("Google Speech Recognition could not understand audio.")
        except sr.RequestError as e:
            self.get_logger().error(f"Could not request results from Google Speech Recognition service; {e}")
        return None

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
