import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pytz
import smtplib
from email.message import EmailMessage
import openai
class IrrelevantQuestionNode(Node):
    def __init__(self):
        super().__init__('Irrelevant_question_node')
        # Subscribe to the "emergency_questions" topic
        self.subscription = self.create_subscription(
            String,
            'Irrelevant_Questions',
            self.question_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.firebase_publisher = self.create_publisher(String, 'Firebase_upload', 10)
        # Publisher for the "speech_text" topic
        self.publisher_ = self.create_publisher(String, 'text_speech', 10)
        self.get_logger().info('Irrelevant QuestionNode has been started.')
        self.store_question = None

        self.nurse_email = 'mediallynurse@gmail.com'
        self.sender_email = "mediallynurse@gmail.com"  # Your email
        self.sender_password = "password"  # Your app password
        self.patient_name = "Jane Doe"

        openai.api_key = ""

    def question_callback(self, msg: String):

        self.get_logger().info(f'Received question: "{msg.data}"')
        try:
            # Directly call the ChatCompletion API            completion = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a helpful assistant."},
                    {"role": "user", "content": msg.data}
                ]
            )
            fixed_phrase = completion.choices[0].message.content
        except Exception as e:
            fixed_phrase = f"Error generating response: {e}"
            self.get_logger().error(fixed_phrase)
        # Define the fixed phrase to be published

        self.store_question = msg.data

        # Create a new String message and publish it to "speech_text"
        speech_msg = String()
        speech_msg.data = fixed_phrase
        self.publisher_.publish(speech_msg)



        # Create a proper String message for firebase_publisher
        firebase_text = f"question: {self.store_question}, Answer: {speech_msg.data}"
        firebase_msg = String()
        firebase_msg.data = firebase_text
        self.firebase_publisher.publish(firebase_msg)

        self.get_logger().info(f'Published to text_speech: "{fixed_phrase}"')



    def send_email_alert(self, nurse_email, patient_name, msg):
        msg = EmailMessage()
        msg.set_content(
            f"ALERT: {patient_name} has asked the following question: {msg}. Please check on them.")
        msg["Subject"] = f"Emergency Alert for {patient_name}"
        msg["From"] = self.sender_email
        msg["To"] = nurse_email

        try:
            with smtplib.SMTP_SSL("smtp.gmail.com", 465) as server:
                server.login(self.sender_email, self.sender_password)
                server.send_message(msg)
                self.get_logger().info(f"Email sent to {nurse_email}")
        except Exception as e:
            self.get_logger().error(f"Failed to send email: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IrrelevantQuestionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
