#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from transformers import pipeline
from std_msgs.msg import String
from langchain_community.llms import HuggingFacePipeline

from collections import deque
from langchain_core.outputs import LLMResult


class ChatSummarizationNode(Node):
    def __init__(self):
        super().__init__('chat_summarization_node')

        # Initialize Hugging Face Summarization Pipeline.
        # Make sure the provided paths point to the correct model/tokenizer directory.
        summarizer_pipeline = pipeline(
            "summarization",
            model="sshleifer/distilbart-cnn-12-6",
            tokenizer="sshleifer/distilbart-cnn-12-6"
        )

        # Wrap the pipeline in LangChain's HuggingFacePipeline and assign to an instance variable.
        self.summarizer = HuggingFacePipeline(pipeline=summarizer_pipeline)

        # Subscribe to chat history messages.
        self.subscription = self.create_subscription(String, 'chat_history', self.summarize_chat, 10)
        self.summary_memory = deque(maxlen=2)
        # Publisher for summarized chat history.
        self.publisher = self.create_publisher(String, 'chat_summary', 10)

        self.get_logger().info("Chat Summarization Node Ready.")

    def summarize_chat(self, msg: String):
        """
        Summarizes the chat history and publishes it back to the LLM node.
        """
        self.get_logger().info("Received chat history for summarization.")
        chat_text = msg.data  # Extract chat history

        previous_summaries = "\n".join(self.summary_memory)
        full_context = f"{previous_summaries}\n{chat_text}" if previous_summaries else chat_text


        try:
            # Generate summary from the chat history
            summary = self.summarizer.generate([full_context], max_length=80, min_length=50, do_sample=False)
            if isinstance(summary, LLMResult) and summary.generations:
                summary_text = summary.generations[0][0].text
        except Exception as e:
            self.get_logger().error(f"Summarization failed: {e}")
            summary_text = "Summary generation failed."

        # Publish the summarized chat history
        summary_msg = String()
        summary_msg.data = summary_text
        self.publisher.publish(summary_msg)
        self.get_logger().info("Published summarized chat history.")

def main(args=None):
    rclpy.init(args=args)
    node = ChatSummarizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
