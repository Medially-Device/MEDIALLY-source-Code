#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        # Create a publisher that publishes String messages on "speech_text"
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)

        self.get_logger().info("SpeechToTextNode started. Converting audio to text...")

        # Example usage: Convert "recorded_audio.wav" to text once on startup
        recognized_text = self.speech_to_text_from_file("/home/medially/Medially_ws/audio/recorded_audio.wav")
        if recognized_text:
            # Publish the recognized text to the "speech_text" topic
            msg = String()
            msg.data = recognized_text
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published recognized text to /speech_text: {recognized_text}")
        else:
            self.get_logger().warn("No text was recognized or an error occurred.")

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
    """Main entry point for the ROS 2 node."""
    rclpy.init(args=args)
    node = SpeechToTextNode()
    # Spin the node so it stays alive (e.g., for future expansions or repeated tasks).
    rclpy.spin(node)
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
