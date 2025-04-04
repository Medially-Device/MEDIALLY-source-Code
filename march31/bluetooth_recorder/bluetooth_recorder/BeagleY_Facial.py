#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from medially_interfaces.srv import FacialRecognition
from std_msgs.msg import String
import socket
import paramiko


class RecognitionService(Node):
    def __init__(self):
        super().__init__('recognition_service')

        self.HOST = '192.168.10.1'
        self.PORT = 12345
        self.conn = None

        # SSH config
        self.beaglebone_ip = "192.168.10.2"
        self.username = "debian"
        self.password = "MEDIALLY"

        self.text_speech_pub = self.create_publisher(String, 'text_speech', 10)

        self.get_logger().info("Connecting to BeagleY-AI via SSH...")
        if not self.execute_ssh_commands():
            self.get_logger().error("Failed to start remote BeagleY-AI script. Node will shut down.")
            rclpy.shutdown()
            return

        self.start_socket_server()

        # ROS2 Services
        self.facial_srv = self.create_service(
            FacialRecognition,
            'facial_recognition_service',
            self.handle_facial_recognition
        )



        self.get_logger().info("Facial and Pill Recognition Services ready.")

    def execute_ssh_commands(self):
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(self.beaglebone_ip, username=self.username, password=self.password)

            startup_command = "source CompVision/bin/activate && python3 SocketServer-Client_tf_lite.py &"
            stdin, stdout, stderr = ssh.exec_command(startup_command)
            exit_status = stdout.channel.recv_exit_status()

            if exit_status == 0:
                self.get_logger().info("BeagleY-AI script started via SSH successfully.")
                return True
            else:
                error_message = stderr.read().decode('utf-8')
                self.get_logger().error(f"SSH command failed. Error: {error_message}")
                return False
        except Exception as e:
            self.get_logger().error(f"SSH Error: {e}")
            return False

    def start_socket_server(self):
        self.get_logger().info(f"Starting socket server on {self.HOST}:{self.PORT}")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen(1)

        self.get_logger().info("Waiting for BeagleY-AI client connection...")
        self.conn, addr = self.server_socket.accept()
        self.get_logger().info(f"Connected by {addr}")

    def handle_facial_recognition(self, request, response):
        if not self.conn:
            self.get_logger().error("No active connection to client.")
            response.success = False
            return response

        role = request.role.lower().strip()

        try:
            if role == "facial recognition":
                self.get_logger().info("Sending START_FACEID to BeagleY-AI...")
                question = f"Look at Camera for 20 seconds for Facial Recognition"
                question_msg = String()
                question_msg.data = question
                self.text_speech_pub.publish(question_msg)
                time.sleep(5)
                self.conn.sendall(b"START_FACEID")
            elif role == "pill recognition":
                self.get_logger().info("Sending START_MAIN to BeagleY-AI for pill classification...")
                self.conn.sendall(b"START_MAIN")
            else:
                self.get_logger().warn(f"Unknown role received: {request.role}")
                response.success = False
                return response

            result = self.conn.recv(1024).decode().strip()
            self.get_logger().info(f"Received from BeagleY-AI: {result}")
            response.success = result == "True"

        except Exception as e:
            self.get_logger().error(f"Socket communication error: {e}")
            response.success = False

        return response

    def destroy_node(self):
        if self.conn:
            self.conn.close()
        self.server_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecognitionService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Recognition Service...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
