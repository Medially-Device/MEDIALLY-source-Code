#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import subprocess
import time
import sounddevice as sd
import wave

class BluetoothRecorderNode(Node):
    def __init__(self):
        super().__init__('bluetooth_recorder_node')

        # Replace with your actual headset's MAC address
        self.headset_mac = "00:A4:1C:3C:6B:30"
        self.output_file = "/home/medially/Medially_ws/audio/recorded_audio.wav"
        self.record_seconds = 10
        self.sample_rate = 16000

        self.get_logger().info("BluetoothRecorderNode started.")

        # Use a one-shot timer so the node has time to spin up
        self.timer_ = self.create_timer(1.0, self.run_bluetooth_recording)

    def run_bluetooth_recording(self):
        # Destroy the timer so it doesn't repeat
        self.destroy_timer(self.timer_)

        self.get_logger().info(f"Attempting to connect to {self.headset_mac}...")
        self.force_connect(self.headset_mac)

        # Allow a moment for the connection to stabilize
        time.sleep(2)

        self.get_logger().info("Switching profile to HSP/HFP...")
        success = self.switch_profile_to_hsp_hfp(self.headset_mac)
        if not success:
            self.get_logger().error("Unable to switch to HSP/HFP. Mic may not work under A2DP.")
            return

        self.get_logger().info("Recording audio...")
        self.record_audio_sounddevice(
            duration=self.record_seconds,
            filename=self.output_file,
            samplerate=self.sample_rate
        )
        self.get_logger().info(f"Audio saved to {self.output_file}")

    def force_connect(self, mac_address):
        """
        Forces connection to a previously paired Bluetooth device.
        The headset does NOT need to be in pairing mode if already known/trusted.
        """
        subprocess.run("bluetoothctl power on", shell=True, check=True)
        subprocess.run(f"bluetoothctl connect {mac_address}", shell=True)
        subprocess.run(f"bluetoothctl trust {mac_address}", shell=True)

    def switch_profile_to_hsp_hfp(self, mac_address):
        """
        Switches the connected Bluetooth headset to HSP/HFP profile (enables microphone input).
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
            self.get_logger().error(
                "ERROR: Could not find a matching bluez_card for this MAC address."
            )
            return False

        # Attempt mSBC
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

    def record_audio_sounddevice(self, duration, filename, samplerate):
        """
        Record audio from the current default audio input (should be your Bluetooth mic in HSP/HFP).
        """
        recording = sd.rec(int(duration * samplerate), samplerate=samplerate,
                           channels=1, dtype='int16')
        sd.wait()

        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 2 bytes = 16 bits
            wf.setframerate(samplerate)
            wf.writeframes(recording.tobytes())


def main(args=None):
    rclpy.init(args=args)
    node = BluetoothRecorderNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
