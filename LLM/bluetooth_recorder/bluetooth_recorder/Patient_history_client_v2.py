#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from medially_interfaces.srv import LLMRequest
from langchain.memory import ConversationBufferWindowMemory
class PatientHistoryClientNode(Node):
    def __init__(self):
        super().__init__('patient_history_client')

        # ✅ Create a service client to the centralized LLM service.
        self.client = self.create_client(LLMRequest, 'process_llm_request')
        while not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for LLM Service...")

        # ✅ Subscribe to the 'Patient_History' topic.
        self.subscription = self.create_subscription(
            String,
            'Patient_History',
            self.patient_history_callback,
            10
        )
        self.memory = ConversationBufferWindowMemory(k=3, memory_key="chat_history", return_messages=True)
        # ✅ Publisher to send the question to the Vectorizer Node
        self.vectorizer_publisher = self.create_publisher(String, 'vectorizer_input_Patient', 10)
        self.firebase_publisher = self.create_publisher(String, 'Firebase_upload', 10)

        # ✅ Subscribe to Vectorizer output (retrieved patient data)
        self.vectorizer_subscription = self.create_subscription(
            String,
            'vectorizer_output_Patient',
            self.handle_vectorizer_response,
            10
        )

        # ✅ Publisher for the 'answer_comparison' topic (final answer output)
        self.answer_publisher = self.create_publisher(String, 'text_speech', 10)

        self.pending_question = None  # Stores the question while waiting for the vectorized response
        self.store_question = None

        self.get_logger().info("Patient History Client Node Ready.")

    def patient_history_callback(self, msg: String):
        """Callback to process the question from the Patient History topic."""
        user_question = msg.data
        self.get_logger().info(f"Received question: {user_question}")

        # ✅ Store the question while waiting for the vectorized response
        self.pending_question = user_question

        # ✅ Send the question to the Vectorizer Node for retrieving relevant patient data
        vectorizer_msg = String()
        vectorizer_msg.data = user_question
        self.vectorizer_publisher.publish(vectorizer_msg)
        self.get_logger().info(f"Sent question to Vectorizer Node: {user_question}")

    def handle_vectorizer_response(self, msg: String):
        """Handles the response from the Vectorizer Node containing relevant patient data."""
        if not self.pending_question:
            self.get_logger().error("Received vectorizer response but no pending question.")
            return

        retrieved_context = msg.data
        chat_history = self.memory.load_memory_variables({})["chat_history"]
        self.get_logger().info(f"Received retrieved context: {retrieved_context}")

        # ✅ Generate a prompt using the retrieved patient data
        prompt = f"""
        Chat History (Last 3 Turns):
        {chat_history} 
Use the patient data below to answer the question.
If unclear, respond with "NEED MORE INFO".
If the answer isn't available, respond with "ANSWER NOT IN CONTEXT".

Patient Data:
{retrieved_context}

Question: {self.pending_question}
ONLY ANSWER THE QUESTION USING THE DATA provided.  

Answer:
""".strip()

        # ✅ Send the prompt to the centralized LLM
        self.memory.save_context({"input": self.pending_question}, {"output": retrieved_context})
        request = LLMRequest.Request()

        request.role = "contextual_question"
        request.input_text = prompt

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_service_response)
        self.store_question = self.pending_question
        # ✅ Clear pending question since it's now being processed
        self.pending_question = None

    def handle_service_response(self, future):
        """Handles the LLM service response and publishes the final answer."""
        try:
            result = future.result()
            if result is not None:
                answer_msg = String()
                answer_msg.data = result.output_text
                self.answer_publisher.publish(answer_msg)
                firebase_msg = String()
                firebase_msg.data = f"question: {self.store_question}, Answer: {answer_msg.data}"
                self.firebase_publisher.publish(firebase_msg)

                self.get_logger().info(f"Published final answer: {answer_msg.data}")
            else:
                self.get_logger().error("Service call returned 'None'.")
        except Exception as e:
            self.get_logger().error(f"Service call raised an exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PatientHistoryClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
