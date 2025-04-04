#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from medially_interfaces.srv import LLMRequest
from std_msgs.msg import String

# ✅ Groq-compatible OpenAI client
import openai
openai.api_key = ""  # Replace with your real key
openai.api_base = "https://api.groq.com/openai/v1"

class LLMServiceNode(Node):
    def __init__(self):
        super().__init__('llm_service_node')
        self.get_logger().info("Groq-based LLM Service Node started!")

        # ✅ ROS2 Publishers & Subscribers
        self.history_publisher = self.create_publisher(String, 'chat_history', 10)
        self.summary_subscription = self.create_subscription(String, 'chat_summary', self.update_chat_summary, 10)

        # Initialize chat summary (from external summarizer node)
        self.chat_history_summary = "No prior conversation."

        # ✅ Create ROS2 service
        self.srv = self.create_service(LLMRequest, 'process_llm_request', self.handle_request)

    def call_groq_model(self, prompt: str, model="llama3-8b-8192"):
        try:
            response = openai.ChatCompletion.create(
                model=model,
                messages=[
                    {"role": "system", "content": "You are a helpful medical assistant."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=256
            )
            return response['choices'][0]['message']['content'].strip()
        except Exception as e:
            self.get_logger().error(f"Groq API error: {e}")
            return "Error: Unable to generate response."

    def handle_request(self, request, response):
        role = request.role.lower()
        input_text = request.input_text
        self.get_logger().info(f"Received LLM request: Role={role}, Input={input_text}")

        chat_history_str = self.chat_history_summary

        if role == "sentence_fixer":
            prompt = self.build_strict_medical_prompt_sentence_fixer(input_text)
            llm_output = self.call_groq_model(prompt)
            parsed_fixed_sentence, parsed_question = self.parse_model_response(llm_output)
            response.output_text = parsed_question

        elif role == "contextual_question":
            prompt = f"""
Recent Chat History (Last 3 Turns):
{chat_history_str}

You are a context-aware medical assistant. Answer based only on the given data. Answer the full question. Concisely!
Be decisive. Do not use "may" or "sometimes". The context is verified by a doctor.
Phrase the answer as if you're speaking to someone.

User Input: {input_text}
""".strip()
            llm_output = self.call_groq_model(prompt)
            response.output_text = llm_output
            self.publish_history(input_text, llm_output)

        elif role == "medication_question":
            prompt = f"""
Recent Chat History (Last 3 Turns):
{chat_history_str}

You are a context-aware medical assistant. Answer based only on the given data in less than 2 sentences. Less the Better. Concisely!
Be decisive. Do not use "may" or "sometimes". The context is verified by a doctor.
Use the chat summary to influence the next response. 
e.g., if the previous chat is about a drug and the next question doesn't specify, assume the previous drug.

User Input: {input_text}
""".strip()
            llm_output = self.call_groq_model(prompt)
            response.output_text = llm_output
            self.publish_history(input_text, llm_output)

        else:
            prompt = f"""
Recent Chat History (Last 3 Turns):
{chat_history_str}

User Input: {input_text}
User query response:""".strip()
            llm_output = self.call_groq_model(prompt)
            response.output_text = llm_output
            self.publish_history(input_text, llm_output)

        return response

    def publish_history(self, user_input: str, assistant_response: str):
        history_msg = String()
        history_msg.data = f"User: {user_input}\nAssistant: {assistant_response}"
        self.history_publisher.publish(history_msg)
        self.get_logger().info("Sent chat history for summarization.")
        self.get_logger().info(f"Generated LLM Response: {assistant_response}")

    def update_chat_summary(self, msg: String):
        self.chat_history_summary = msg.data
        self.get_logger().info("Updated chat history with summary.")

    def build_strict_medical_prompt_sentence_fixer(self, user_input: str) -> str:
        return f"""
You are a medical assistant. You must:

1. Correct any spelling or grammatical errors in the input.
2. Rephrase the corrected sentence into a clear medical question.
3. Your response MUST be in EXACTLY TWO lines:
   **Fixed Sentence:** <Corrected Sentence>
   **Rephrased Question:** <Rewritten Question>
4. DO NOT add explanations, comments, code, markdown, or extra text.

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
