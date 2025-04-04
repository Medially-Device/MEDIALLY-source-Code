#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import uuid
from medially_interfaces.srv import LLMRequest
from langchain.memory import ConversationBufferWindowMemory
class PatientHistoryClientNode(Node):
    def __init__(self):
        super().__init__('medication_client')

        # ✅ Create a service client to the centralized LLM service.
        self.client = self.create_client(LLMRequest, 'process_llm_request')
        while not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for LLM Service...")

        # ✅ Subscribe to the 'Patient_History' topic.
        self.subscription = self.create_subscription(
            String,
            'Medication_Questions',
            self.patient_history_callback,
            10
        )
        self.memory = ConversationBufferWindowMemory(k=3, memory_key="chat_history", return_messages=True)
        # ✅ Publisher to send the question to the Vectorizer Node
        self.vectorizer_publisher = self.create_publisher(String, 'vectorizer_input_medication', 10)

        # ✅ Subscribe to Vectorizer output (retrieved patient data)
        self.vectorizer_subscription = self.create_subscription(
            String,
            'vectorizer_output_medication',
            self.handle_vectorizer_response,
            10
        )

        # ✅ Publisher for the 'answer_comparison' topic (final answer output)
        self.answer_publisher = self.create_publisher(String, 'text_speech', 10)
        self.firebase_publisher = self.create_publisher(String, 'Firebase_upload', 10)
        # Dictionary to store pending questions with unique IDs
        self.pending_questions = {}
        # To store the question for final output
        self.store_question = None
        self.get_logger().info("Medication Client Node Ready.")

    def patient_history_callback(self, msg: String):
        """Callback to process the question from the Medication_Questions topic."""
        user_question = msg.data
        self.get_logger().info(f"Received question: {user_question}")

        # Generate a unique identifier for this question
        question_id = str(uuid.uuid4())
        # Store the question with its unique identifier
        self.pending_questions[question_id] = user_question

        # Send the question to the Vectorizer Node, prepending the unique identifier
        vectorizer_msg = String()
        vectorizer_msg.data = f"{question_id}:{user_question}"
        self.vectorizer_publisher.publish(vectorizer_msg)
        self.get_logger().info(f"Sent question to Vectorizer Node: {user_question} with ID {question_id}")

    def handle_vectorizer_response(self, msg: String):
        """Handles the response from the Vectorizer Node containing relevant patient data."""
        try:
            # Expect the response in the format "question_id:retrieved_context"
            question_id, retrieved_context = msg.data.split(":", 1)
        except ValueError:
            self.get_logger().error("Invalid vectorizer response format. Expected 'question_id:retrieved_context'.")
            return

        # Check if the received question_id exists in pending_questions
        if question_id not in self.pending_questions:
            self.get_logger().error("Received vectorizer response but no pending question for the given ID.")
            return

        # Retrieve and remove the matching question from pending_questions
        user_question = self.pending_questions.pop(question_id)
        chat_history = self.memory.load_memory_variables({})["chat_history"]
        self.get_logger().info(f"Received retrieved context: {retrieved_context}")

        # Generate a prompt using the retrieved patient data and chat history
        prompt = f"""
        Chat History (Last 3 Turns):
        {chat_history} 

Patient Data:
{retrieved_context}

Question: {user_question}
If the answer isn't available, respond with "ANSWER NOT IN CONTEXT". 
ONLY ANSWER THE QUESTION USING THE DATA provided.  

Answer:
""".strip()

        # Save the context into memory and prepare the LLM request
        self.memory.save_context({"input": user_question}, {"output": retrieved_context})
        request = LLMRequest.Request()
        request.role = "medication_question"
        request.input_text = prompt

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_service_response)
        self.store_question = user_question  # Save question for final output

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
