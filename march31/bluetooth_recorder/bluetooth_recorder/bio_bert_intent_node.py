#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import torch
from transformers import AutoTokenizer, AutoModelForSequenceClassification

class BioBERTIntentNode(Node):
    def __init__(self):
        super().__init__('bio_bert_intent_node')

        # 1) Declare parameters or hard-code them
        self.declare_parameter('model_directory', '/home/medially/Downloads/fine_tuned_biobert/fine_tuned_biobert')
        self.model_directory = self.get_parameter('model_directory').get_parameter_value().string_value

        # 2) Create subscriber to "intent_question"
        self.subscription = self.create_subscription(
            String,
            'intent_question',
            self.intent_question_callback,
            10
        )

        # 3) Define possible labels and map them to ROS topics
        #    Using underscores instead of spaces is recommended in ROS.
        self.intent_mapping = {"Emergency Questions":2, "Irrelevant Questions": 3, "Medication Questions": 1, "Patient History": 0}
        self.id_to_label = {v: k for k, v in self.intent_mapping.items()}

        # Convert label to ROS topic name
        self.label_to_topic = {
            'Emergency Questions': 'Emergency_Questions',
            'Irrelevant Questions': 'Irrelevant_Questions',
            'Medication Questions': 'Medication_Questions',
            'Patient History': 'Patient_History'
        }

        # 4) Create a publisher for each label-based topic
        #    We'll publish the original text message to exactly one topic
        #    depending on the predicted label.
        self.label_publishers = {}
        for label_name, topic_name in self.label_to_topic.items():
            pub = self.create_publisher(String, topic_name, 10)
            self.label_publishers[label_name] = pub

        # 5) Load model and tokenizer
        self.get_logger().info("Loading fine-tuned BioBERT model from: " + self.model_directory)
        self.tokenizer = AutoTokenizer.from_pretrained(self.model_directory)
        self.model = AutoModelForSequenceClassification.from_pretrained(self.model_directory)

        # 6) Move model to GPU if available, else CPU
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)
        self.model.eval()
        self.get_logger().info("Model loaded successfully. Node ready to classify.")

    def intent_question_callback(self, msg: String):
        """
        Callback to handle new text data from 'intent_question' topic.
        Predict one of 4 labels, then publish the original text to
        the corresponding label-specific topic.
        """
        user_input = msg.data
        self.get_logger().info(f"Received message: {user_input}")

        # Tokenize input as a batch of size 1
        tokens = self.tokenizer(
            [user_input],
            padding=True,
            truncation=True,
            max_length=128,
            return_tensors="pt"
        )

        input_ids = tokens["input_ids"].to(self.device)
        attention_mask = tokens["attention_mask"].to(self.device)

        # Forward pass & prediction
        with torch.no_grad():
            outputs = self.model(input_ids, attention_mask=attention_mask)
            logits = outputs.logits
            predicted_label_id = torch.argmax(logits, dim=1).cpu().numpy()[0]

        # Convert label ID -> label text
        predicted_label = self.id_to_label[predicted_label_id]
        self.get_logger().info(f"Predicted intent: {predicted_label}")

        # Publish the original text to the predicted label's topic
        pub_topic = self.label_publishers[predicted_label]
        out_msg = String()
        out_msg.data = user_input
        pub_topic.publish(out_msg)
        self.get_logger().info(f"Published to topic: {self.label_to_topic[predicted_label]}")

def main(args=None):
    rclpy.init(args=args)
    node = BioBERTIntentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
