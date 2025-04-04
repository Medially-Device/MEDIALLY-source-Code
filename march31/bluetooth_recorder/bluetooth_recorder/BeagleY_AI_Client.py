#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from medially_interfaces.srv import FacialRecognition  # Replace with your actual package name

import socket


class FacialRecognitionService(Node):
    def __init__(self):
        super().__init__('facial_recognition_service')
        self.srv = self.create_service(
            FacialRecognition,
            'facial_recognition_service',
            self.handle_facial_recognition
        )

        # Set up socket
        self.HOST = '192.168.10.1'  # IP of the host computer (assumed static)
        self.PORT = 12345
        self.conn = None

        self.start_socket_server()

    def start_socket_server(self):
        self.get_logger().info(f"Starting socket server on {self.HOST}:{self.PORT}")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen(1)

        self.get_logger().info("Waiting for BeagleBone client connection...")
        self.conn, addr = self.server_socket.accept()
        self.get_logger().info(f"Connected by {addr}")

    def handle_facial_recognition(self, request, response):
        if not self.conn:
            self.get_logger().error("No active connection to client.")
            response.success = False
            return response

        if request.role.lower() == "facial recognition":
            try:
                signal = b"START_FACEID"
                self.get_logger().info("Sending START_FACEID to BeagleBone...")
                self.conn.sendall(signal)
                question = f"Look at Camera for 20 seconds for Facial Recognition"
                question_msg = String()
                question_msg.data = question
                self.medication_question_pub.publish(question_msg)
                # Receive result
                result = self.conn.recv(1024).decode().strip().lower()
                self.get_logger().info(f"Received from BeagleBone: {result}")

                response.success = result == "true"
            except Exception as e:
                self.get_logger().error(f"Socket communication error: {e}")
                response.success = False
        else:
            self.get_logger().warn(f"Unknown role received: {request.role}")
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = FacialRecognitionService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down facial recognition service...")
    finally:
        if node.conn:
            node.conn.close()
        node.server_socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
