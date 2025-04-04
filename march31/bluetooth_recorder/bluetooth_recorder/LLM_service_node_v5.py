#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from llama_cpp import Llama
from medially_interfaces.srv import LLMRequest
from std_msgs.msg import String
# Import LangChain memory classes and HuggingFacePipeline for local summarization
from langchain.memory import ConversationSummaryBufferMemory
from transformers import pipeline
from langchain.llms import HuggingFacePipeline


class LLMServiceNode(Node):
    def __init__(self):
        super().__init__('llm_service_node')

        # Load the Llama model once
        model_path = "/home/medially/Medially_ws/install/bluetooth_recorder/lib/python3.12/site-packages/bluetooth_recorder/q4_0-orca-mini-3b.gguf"
        self.get_logger().info(f"Loading Llama model from {model_path}...")
        self.llm = Llama(model_path=model_path, n_ctx=2048, n_threads=4, verbose=False)
        self.get_logger().info("Llama model loaded successfully!")

        # ✅ ROS2 Publishers & Subscribers
        self.history_publisher = self.create_publisher(String, 'chat_history', 10)
        self.summary_subscription = self.create_subscription(String, 'chat_summary', self.update_chat_summary, 10)

        # Initialize summarizing memory: it will summarize conversation history
        # to keep the prompt concise.
        # Store summarized chat history (instead of using a memory buffer)
        self.chat_history_summary = "No prior conversation."

        # Create the ROS2 service for processing LLM requests.
        self.srv = self.create_service(LLMRequest, 'process_llm_request', self.handle_request)

    def handle_request(self, request, response):
        """
        Handles incoming LLM requests with multi-turn conversation support using summarizing memory.
        """
        role = request.role.lower()
        input_text = request.input_text
        self.get_logger().info(f"Received LLM request: Role={role}, Input={input_text}")

        ## Use the most recent chat history summary received from Chat Summarization Node
        chat_history_str = self.chat_history_summary

        # Construct a prompt that includes the summarized conversation history.
        prompt = f"""
Recent Chat History (Last 3 Turns):
{chat_history_str}

User Input: {input_text}
""".strip()

        if role == "sentence_fixer":
            prompt = self.build_strict_medical_prompt_sentence_fixer(input_text)
            llm_output = self.llm(prompt, temperature=0.1, top_p=0.9, max_tokens=256)["choices"][0]["text"].strip()
            parsed_fixed_sentence, parsed_question = self.parse_model_response(llm_output)
            response.output_text = parsed_question

        elif role == "contextual_question":
            prompt = f"""
            Recent Chat History (Last 3 Turns):
            {chat_history_str}
            You are a context-aware medical assistant. Answer based only on the given data in less than 2 sentences. Less the Better. concisely!
            Be decisive do not use may or sometimes, be direct. The context provided is correct and verified by the doctor.
            Phrase the Answer as if you were talking to someone.

            User Input: {input_text}
            """.strip()

            llm_output = self.llm(prompt, temperature=0.1, top_p=0.7, max_tokens=256)
            response.output_text = llm_output["choices"][0][
                "text"].strip() if "choices" in llm_output else "No response generated"
            history_msg = String()
            history_msg.data = f"User: {input_text}\nAssistant: {response.output_text}"  # Convert to string before sending
            self.history_publisher.publish(history_msg)
            self.get_logger().info("Sent chat history for summarization.")
            self.get_logger().info(f"Generated LLM Response: {response.output_text}")

        elif role == "medication_question":
            prompt = f"""
                        Recent Chat History (Last 3 Turns):
                        {chat_history_str}
                        You are a context-aware medical assistant. Answer based only on the given data in less than 2 sentences. Less the Better. concisely!
                        Be decisive do not use may or sometimes, be direct. The context provided is correct and verified by the doctor.
                        Phrase the Answer as if you were talking to someone. Use the chat summary to influence the next response. 
                        ie. if the previous chat is about a drug and the next question doesnt specify, assume the previous drug.

                        User Input: {input_text}
                        """.strip()

            llm_output = self.llm(prompt, temperature=0.1, top_p=0.7, max_tokens=256)
            response.output_text = llm_output["choices"][0][
                "text"].strip() if "choices" in llm_output else "No response generated"
            medication_msg = String()
            medication_msg.data = f"User: {input_text}\nAssistant: {response.output_text}"  # Convert to string before sending
            self.history_publisher.publish(medication_msg)
            self.get_logger().info("Sent chat history for summarization.")
            self.get_logger().info(f"Generated LLM Response: {response.output_text}")

        else:
            prompt += "\nUser query response:"
            llm_output = self.llm(prompt, temperature=0.05, top_p=0.7, max_tokens=256)["choices"][0]["text"].strip()
            response.output_text = llm_output
            history_msg = String()
            history_msg.data = f"User: {input_text}\nAssistant: {response.output_text}"  # Convert to string before sending
            self.history_publisher.publish(history_msg)
            self.get_logger().info("Sent chat history for summarization.")
            self.get_logger().info(f"Generated LLM Response: {response.output_text}")

        # Save the new exchange in memory (this will update the conversation summary)

        # ✅ Publish chat history for summarization

        return response

    def update_chat_summary(self, msg: String):
        """
        Updates conversation memory with summarized chat history.
        """
        self.chat_history_summary = msg.data  # Update stored chat history
        self.get_logger().info("Updated chat history with summary.")

    def build_strict_medical_prompt_sentence_fixer(self, user_input: str) -> str:
        """
        Constructs the multi-step medical prompt:
          1) Fix grammar/spelling.
          2) Rephrase as a question.
          3) Exactly two lines: Fixed Sentence / Rephrased Question.
          4) No extra text or explanations.
          5) Includes a simple example.
        """
        return f"""
You are a medical assistant. You must:

1. Correct any spelling or grammatical errors in the input.
2. Rephrase the corrected sentence into a clear question.
3. Your response MUST be in EXACTLY TWO lines:
   **Fixed Sentence:** <Corrected Sentence>
   **Rephrased Question:** <Rewritten Question>
4. DO NOT add explanations, comments, code, markdown, or extra text.
5. if the input is already in a form of a question, DO NOT CHANGE IT!!

------------------------------------------------------------
Example 1:
User input: "I can i take, the pill i guess at night or i forget. tired"
**Fixed Sentence:** "Can I take the pill at night if I often forget, and I'm tired?"
**Rephrased Question:** "Should I take my medication at night to avoid forgetting it?"
------------------------------------------------------------

Now process this:
Input: "{user_input}"
**Fixed Sentence:** 
""".strip()

    def parse_model_response(self, raw_text: str):
        """
        Extracts '**Fixed Sentence:**' and '**Rephrased Question:**' from model output.
        If either is missing or too short, returns an error message in its place.
        """
        fixed_sentence = None
        rephrased_question = None

        for line in raw_text.split("\n"):
            line = line.strip()
            if line.startswith("**Fixed Sentence:**"):
                fixed_sentence = line.split("**Fixed Sentence:**")[-1].strip()
            elif line.startswith("**Rephrased Question:**"):
                rephrased_question = line.split("**Rephrased Question:**")[-1].strip()

        if not fixed_sentence or len(fixed_sentence) < 3:
            fixed_sentence = "ERROR: Missing or incorrect Fixed Sentence."
        if not rephrased_question or len(rephrased_question) < 3:
            rephrased_question = "ERROR: Missing or incorrect Rephrased Question."

        return fixed_sentence, rephrased_question


def main(args=None):
    rclpy.init(args=args)
    node = LLMServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
