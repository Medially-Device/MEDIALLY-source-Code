#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time
import lgpio
import sounddevice as sd
import wave
import numpy as np

BUTTON_PIN = 27
CHIP = 4  # Typically 0 or 4 on Raspberry Pi; adjust if needed

class BluetoothRecorderButtonNode(Node):
    def __init__(self):
        super().__init__('bluetooth_recorder_button_node')
        # Bluetooth and audio parameters
        self.headset_mac = "00:A4:1C:3C:6B:30"  # Replace with your headset's MAC address
        self.output_file = "/home/medially/Medially_ws/audio/recorded_audio.wav"
        self.sample_rate = 16000

        # Button state and recording flag
        self.recording = False
        self.button_last_state = 0

        # Publisher for signaling that a new audio file is available (publishing a proper String message)
        self.publisher_ = self.create_publisher(String, 'audio_convert', 10)

        # Setup GPIO for button using lgpio
        self.gpio_handle = lgpio.gpiochip_open(CHIP)
        lgpio.gpio_claim_input(self.gpio_handle, BUTTON_PIN)

        # Timer for polling the button (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.check_button)

        # Placeholders for audio stream and buffer
        self.audio_frames = []
        self.stream = None

        self.get_logger().info("BluetoothRecorderButtonNode started. Waiting for button press...")

    def check_button(self):
        """Poll the button and toggle recording state on state changes.
           With a latch-type button, we use a rising edge (0 -> 1) to start recording,
           and a falling edge (1 -> 0) to stop recording and trigger analysis.
        """
        current_state = lgpio.gpio_read(self.gpio_handle, BUTTON_PIN)

        # Start recording on rising edge (transition from 0 to 1) if not already recording
        if not self.recording and current_state == 1 and self.button_last_state == 0:
            self.get_logger().info("Button pressed (rising edge detected): starting recording...")
            self.start_recording()

        # Stop recording on falling edge (transition from 1 to 0) if recording
        elif self.recording and current_state == 0 and self.button_last_state == 1:
            self.get_logger().info("Button pressed (falling edge detected): stopping recording and starting analysis...")
            self.stop_recording_and_analyze()

        self.button_last_state = current_state
        # Small debounce delay
        time.sleep(0.3)

    def start_recording(self):
        """Connect to Bluetooth headset, switch profile, and start non-blocking audio recording."""
        # Connect to the headset and set the correct profile
        self.force_connect(self.headset_mac)
        time.sleep(2)  # Allow connection to stabilize
        if not self.switch_profile_to_hsp_hfp(self.headset_mac):
            self.get_logger().error("Failed to set Bluetooth profile for recording.")
            return

        # Initialize an empty list to store audio chunks
        self.audio_frames = []

        # Define a callback to capture audio data
        def audio_callback(indata, frames, time_info, status):
            if status:
                self.get_logger().warn(str(status))
            # Append a copy of the audio data so that it is preserved after the callback
            self.audio_frames.append(indata.copy())

        # Start a non-blocking audio stream using sounddevice
        self.stream = sd.InputStream(samplerate=self.sample_rate,
                                     channels=1,
                                     dtype='int16',
                                     callback=audio_callback)
        self.stream.start()
        self.recording = True

    def stop_recording_and_analyze(self):
        """Stop audio recording, write the data to a file, and perform analysis."""
        if self.recording:
            # Stop the audio stream and close it properly
            self.stream.stop()
            self.stream.close()

            # Concatenate all the recorded frames
            audio_data = np.concatenate(self.audio_frames, axis=0)

            # Write the recorded audio to a WAV file
            with wave.open(self.output_file, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)  # 2 bytes for 16-bit audio
                wf.setframerate(self.sample_rate)
                wf.writeframes(audio_data.tobytes())

            self.get_logger().info(f"Audio saved to {self.output_file}")
            self.recording = False

            # Publish a proper String message indicating that a new audio file is available
            msg = String()
            msg.data = "1"
            self.publisher_.publish(msg)

            # Begin analysis (placeholder method)
            self.analyze_audio()

    def analyze_audio(self):
        """Placeholder method for audio analysis."""
        self.get_logger().info("Analyzing audio...")
        # Insert your actual analysis code here. For now, just simulate a delay.
        time.sleep(2)
        self.get_logger().info("Audio analysis completed.")

    def force_connect(self, mac_address):
        """
        Force connection to a previously paired Bluetooth device.
        """
        subprocess.run("bluetoothctl power on", shell=True, check=True)
        subprocess.run(f"bluetoothctl connect {mac_address}", shell=True)
        subprocess.run(f"bluetoothctl trust {mac_address}", shell=True)

    def switch_profile_to_hsp_hfp(self, mac_address):
        """
        Switch the connected Bluetooth headset to HSP/HFP profile (to enable the microphone).
        """
        card_list = subprocess.run(
            ["pactl", "list", "cards", "short"],
            capture_output=True,
            text=True,
            check=True
        )
        card_name = None
        for line in card_list.stdout.splitlines():
            if mac_address.replace(":", "_") in line:
                parts = line.split()
                if len(parts) > 1:
                    card_name = parts[1]
                    break

        if not card_name:
            self.get_logger().error("ERROR: Could not find a matching bluez_card for this MAC address.")
            return False

        # Try setting the mSBC profile first
        msbc_cmd = ["pactl", "set-card-profile", card_name, "headset-head-unit-msbc"]
        result = subprocess.run(msbc_cmd, capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info(f"Profile set to HSP/HFP (mSBC) for card {card_name}.")
            return True
        else:
            self.get_logger().warn("mSBC profile failed, trying basic headset-head-unit...")
            fallback_cmd = ["pactl", "set-card-profile", card_name, "headset-head-unit"]
            fallback_result = subprocess.run(fallback_cmd, capture_output=True, text=True)
            if fallback_result.returncode == 0:
                self.get_logger().info(f"Profile set to basic HSP/HFP for card {card_name}.")
                return True
            else:
                self.get_logger().error("ERROR: Failed to switch to any HSP/HFP profile.")
                self.get_logger().error(f"Output: {fallback_result.stderr}")
                return False

    def destroy_node(self):
        """Clean up GPIO resources on node shutdown."""
        lgpio.gpiochip_close(self.gpio_handle)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothRecorderButtonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
