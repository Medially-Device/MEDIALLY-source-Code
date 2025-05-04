import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from medially_interfaces.srv import LLMRequest

class SentenceFixerNode(Node):
    def __init__(self):
        super().__init__('sentence_fixer_node')

        # Subscribe to speech input
        self.subscription = self.create_subscription(
            String,
            'speech_text',
            self.listener_callback,
            10
        )

        # Publisher for corrected output
        self.publisher = self.create_publisher(
            String,
            'intent_question',
            10
        )

        # LLM service client
        self.client = self.create_client(LLMRequest, 'process_llm_request')

        while not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for LLM Service...")
        self.get_logger().info("LLM Service is now available!")

    def listener_callback(self, msg: String):
        user_input = msg.data
        self.get_logger().info(f"Received input: {user_input}")

        # Create the service request
        request = LLMRequest.Request()
        request.role = "sentence_fixer"
        request.input_text = user_input

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            result = future.result()
            if result is not None:
                out_msg = String()
                out_msg.data = result.output_text
                self.publisher.publish(out_msg)
                self.get_logger().info(f"Published: {out_msg.data}")
            else:
                self.get_logger().error("Service call returned 'None'.")
        except Exception as e:
            self.get_logger().error(f"Service call raised an exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SentenceFixerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
