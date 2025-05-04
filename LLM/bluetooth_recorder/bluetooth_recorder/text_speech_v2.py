#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import tempfile
import subprocess
import time
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

        # Replace with your Bluetooth speaker's MAC address (for example: "06:C0:F2:50:0C:80")
        self.speaker_mac = "06:C0:F2:50:0C:80"

        # Initialize the Bluetooth speaker once at startup
        self.initialize_speaker()

    def initialize_speaker(self):
        self.get_logger().info(f"Initializing Bluetooth speaker {self.speaker_mac}...")
        # Force the connection and trust the device
        self.force_connect(self.speaker_mac)
        time.sleep(2)  # Allow time for connection stabilization

        # Optionally, switch the profile to A2DP if necessary.
        # In your case, your speaker is already using an A2DP profile (a2dp-sink-sbc_xq),
        # but we attempt to switch it anyway.
        if not self.switch_profile_to_a2dp(self.speaker_mac):
            self.get_logger().warn("Speaker profile not switched; proceeding with current configuration.")
        else:
            self.get_logger().info("Speaker is set to A2DP profile.")

        # Retrieve the PulseAudio sink name and set it as default so that TTS audio is routed correctly.
        speaker_sink = self.get_pulseaudio_sink(self.speaker_mac)
        if speaker_sink:
            self.get_logger().info(f"Setting default audio sink to {speaker_sink}...")
            subprocess.run(["pactl", "set-default-sink", speaker_sink], check=True)
        else:
            self.get_logger().warn("Could not determine the PulseAudio sink for the speaker.")
        self.get_logger().info("Bluetooth speaker initialization complete.")

    def listener_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Received message: {text}")
        self.text_to_speech(text)

    def force_connect(self, mac_address):
        """
        Powers on Bluetooth, connects, and trusts the specified device.
        """
        subprocess.run("bluetoothctl power on", shell=True, check=True)
        subprocess.run(f"bluetoothctl connect {mac_address}", shell=True)
        subprocess.run(f"bluetoothctl trust {mac_address}", shell=True)

    def switch_profile_to_a2dp(self, mac_address):
        """
        Switches the connected Bluetooth speaker to an A2DP profile for high-quality audio.
        """
        card_list = subprocess.run(
            ["pactl", "list", "cards", "short"],
            capture_output=True,
            text=True,
            check=True
        )
        card_name = None
        mac_search = mac_address.replace(":", "_")
        for line in card_list.stdout.splitlines():
            if mac_search in line:
                parts = line.split()
                if len(parts) > 1:
                    card_name = parts[1]
                    break
        if not card_name:
            self.get_logger().error("ERROR: Could not find a matching bluez_card for this MAC address.")
            return False

        # Use the profile name as seen in 'pactl list cards'
        # For your speaker, you can use "a2dp-sink-sbc_xq"
        a2dp_cmd = ["pactl", "set-card-profile", card_name, "a2dp-sink-sbc_xq"]
        result = subprocess.run(a2dp_cmd, capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info(f"Profile set to A2DP Sink for card {card_name}.")
            return True
        else:
            self.get_logger().error("ERROR: Failed to switch to A2DP Sink profile.")
            self.get_logger().error(f"Output: {result.stderr}")
            return False

    def get_pulseaudio_sink(self, mac_address):
        """
        Retrieves the PulseAudio sink name corresponding to the Bluetooth speaker.
        """
        sink_list = subprocess.run(
            ["pactl", "list", "sinks", "short"],
            capture_output=True,
            text=True,
            check=True
        )
        mac_search = mac_address.replace(":", "_")
        for line in sink_list.stdout.splitlines():
            if mac_search in line:
                parts = line.split()
                if parts:
                    return parts[1]
        return None

    def text_to_speech(self, text):
        """
        Converts text to speech using gTTS and plays the resulting audio.
        """
        tts = gTTS(text=text, lang='en')
        with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as fp:
            temp_filename = fp.name
            tts.save(temp_filename)
        time.sleep(1)
        playsound(temp_filename)
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
