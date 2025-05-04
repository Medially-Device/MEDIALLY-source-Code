#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import firebase_admin
from firebase_admin import credentials, firestore
from datetime import datetime


class FirebaseSubscriber(Node):
    def __init__(self):
        super().__init__('firebase_subscriber')

        # Initialize Firebase if not already initialized
        if not firebase_admin._apps:
            cred = credentials.Certificate(
                "/home/medially/Downloads/medication-database.json"
            )
            firebase_admin.initialize_app(cred)

        self.db = firestore.client()

        # Subscribe to the topic "Firebase_upload"
        self.subscription = self.create_subscription(
            String,
            "Firebase_upload",
            self.listener_callback,
            10
        )
        """
        self.subscription = self.create_subscription(
            String,
            "Firebase_medication",
            self.medication_callback,
            10
        )
        """
    def listener_callback(self, msg):
        """
        Expects messages in the format:
        "question: {self.pending_question}, Answer: {answer_msg}"
        """
        text = msg.data
        self.get_logger().info(f"Received message: {text}")

        try:
            prefix = "question: "
            answer_prefix = ", Answer: "
            if text.startswith(prefix) and answer_prefix in text:
                # Extract question and answer
                parts = text[len(prefix):].split(answer_prefix)
                if len(parts) == 2:
                    question = parts[0].strip()
                    answer = parts[1].strip()
                    self.get_logger().info(f"Parsed Q: {question}, A: {answer}")
                    self.log_qna(question, answer)
                else:
                    self.get_logger().error("Message format error: could not split question and answer properly.")
            else:
                self.get_logger().error("Message format invalid.")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def log_qna(self, question, answer):
        """
        Stores every incoming Q&A input in Firestore with a timestamp.
        """
        # Log the local date and time when the message was received
        now = datetime.now().isoformat()
        # Build the document: includes both local timestamp and server timestamp
        doc = {
            "question": question,
            "answer": answer,
            "received_at": now,  # Local date and time
            "timestamp": firestore.SERVER_TIMESTAMP,  # Firestore server timestamp
            "name": "Jane Doe"
        }
        try:
            self.db.collection("QA").add(doc)
            self.get_logger().info("Stored QA input in Firebase with timestamp.")
        except Exception as e:
            self.get_logger().error(f"Error storing QA input in Firebase: {e}")

    def medication_callback(self, msg):

        text = msg.data
        self.get_logger().info(f"Received message: {text}")

        try:
            prefix = "question: "
            answer_prefix = ", Answer: "
            if text.startswith(prefix) and answer_prefix in text:
                # Extract question and answer
                parts = text[len(prefix):].split(answer_prefix)
                if len(parts) == 2:
                    question = parts[0].strip()
                    answer = parts[1].strip()
                    self.get_logger().info(f"Parsed Q: {question}, A: {answer}")
                    self.log_qna(question, answer)
                else:
                    self.get_logger().error("Message format error: could not split question and answer properly.")
            else:
                self.get_logger().error("Message format invalid.")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
