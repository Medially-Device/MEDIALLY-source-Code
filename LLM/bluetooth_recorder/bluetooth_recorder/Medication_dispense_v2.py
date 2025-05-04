#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
# Import the custom service
from medially_interfaces.srv import FacialRecognition  # Replace with your actual package name

import time
import board
import busio
from adafruit_pca9685 import PCA9685
import firebase_admin
from firebase_admin import credentials, firestore
import datetime
import pytz


class DispenseNode(Node):
    def __init__(self):
        super().__init__('dispense_node')
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c, address=0x40)
        self.pca.frequency = 50
        self.medication_question_pub = self.create_publisher(String, 'Medication_Questions', 10)
        self.medication_firebase = self.create_publisher(String, 'Medication_Firebase', 10)
        self.dispense_timer = self.create_timer(60.0, self.check_medication_dispense)
        self.text_speech_pub = self.create_publisher(String, 'text_speech', 10)

        # Service client for facial recognition
        self.facial_recognition_client = self.create_client(FacialRecognition, 'facial_recognition_service')
        while not self.facial_recognition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for facial recognition service...')

        if firebase_admin._apps:
            firebase_admin.delete_app(firebase_admin.get_app())
        cred = credentials.Certificate("/home/medially/Downloads/medication-database.json")
        firebase_admin.initialize_app(cred)
        self.db = firestore.client()

        self.timezone = pytz.timezone("America/New_York")
        self.dispensed_records = {}  # {record_key: datetime_object}
        self.cleanup_timer = self.create_timer(3600.0, self.cleanup_dispensed_records)  # every hour
        self.patient_name = 'Jane Doe'
        self.servo_min = 150  # Minimum pulse length (approximately 1 ms).
        self.servo_max = 600  # Maximum pulse length (approximately 2 ms).
    def check_medication_dispense(self):
        now = datetime.datetime.now(self.timezone)
        patients_ref = self.db.collection("Patient Information").where('name', '==', self.patient_name)
        patients = patients_ref.stream()

        for patient in patients:
            patient_data = patient.to_dict()
            patient_name = patient_data.get("name", "Unknown")
            medication_schedule = patient_data.get("medication_schedule", [])


            for med in medication_schedule:
                med_time_str = med.get("time", "").replace(" EST", "").strip()
                med_bool = med.get("taken")
                if not med_time_str:
                    continue

                try:
                    med_time = datetime.datetime.strptime(med_time_str, "%H:%M")
                    med_time = self.timezone.localize(
                        med_time.replace(year=now.year, month=now.month, day=now.day)
                    )
                except ValueError:
                    self.get_logger().error(f"Invalid time format for {patient_name}: {med_time_str}")
                    continue

                time_diff = ((now - med_time).total_seconds())
                if time_diff >= -60 and med_bool==False and time_diff < 0:
                    record_key = f"{patient_name}_{med.get('medication', 'medication')}_{med_time_str}"
                    if record_key in self.dispensed_records:
                        continue
                    self.dispensed_records[record_key] = now
                    question = f"What are the common side effects for {med.get('medication', 'the medication')} and how many pills should i take with for or with water?"
                    question_msg = String()
                    question_msg.data = question
                    self.medication_question_pub.publish(question_msg)
                    time.sleep(1)
                    #question = f"What are activity, dose, food, and water for {med.get('medication', 'the medication')}?"
                    #question_msg = String()
                    #question_msg.data = question
                    #self.medication_question_pub.publish(question_msg)
                    #self.get_logger().info(f"Published query for {med.get('medication', 'medication')} at {med_time_str}")

                    # Call facial recognition service before dispensing
                    self.call_facial_recognition_service(record_key, med)



    def set_servo_angle(self, channel, angle):
        """
        Set the servo at the specified channel to the given angle (0 to 180).
        This function converts the angle to a pulse value between servo_min and servo_max,
        then scales that value to the 16-bit duty_cycle expected by the library.
        """
        # Compute pulse length in 12-bit resolution.
        pulse = int(self.servo_min + (angle / 180.0) * (self.servo_max - self.servo_min))
        # Convert 12-bit pulse (0-4095) to 16-bit duty cycle (0-65535).
        duty_cycle = int(pulse * (65535 / 4096))
        self.pca.channels[channel].duty_cycle = duty_cycle


    def call_facial_recognition_service(self, record_key, med):
        request = FacialRecognition.Request()
        request.role = "facial recognition"

        future = self.facial_recognition_client.call_async(request)
        future.add_done_callback(lambda f: self.handle_facial_recognition_response(f, record_key, med))

    def handle_facial_recognition_response(self, future, record_key, med):
        try:
            response = future.result()
            if response.success:


                # 🗣️ Send speech command for dispensing
                medication = med.get('medication', 'medication')
                compartment = med.get('compartment_id', 'unknown')
                msg = String()
                msg.data = (
                    f"The {medication} has been dispensed in compartment {compartment}. "
                    f"Please take the required medication amount."
                )
                self.text_speech_pub.publish(msg)

                # Then call pill recognition
                request = FacialRecognition.Request()
                request.role = "pill recognition"
                future = self.facial_recognition_client.call_async(request)
                self.dispense_medication(med)
                future.add_done_callback(lambda f: self.handle_pill_recognition_response(f, med, self.patient_name))

                self.dispensed_records[record_key] = datetime.datetime.now(self.timezone)
            else:
                self.get_logger().warn("Facial recognition failed. Medication will not be dispensed.")
                msg = String()
                msg.data = (
                    f"Facial recognition failed. For demonstration the pill box will open!"
                )
                self.text_speech_pub.publish(msg)
                # 🗣️ Send speech command for failed face recognition
                msg = String()
                msg.data = "Face authentication failed. You are not authorized to take this medication."
                self.text_speech_pub.publish(msg)

                request = FacialRecognition.Request()
                request.role = "pill recognition"
                future = self.facial_recognition_client.call_async(request)
                self.dispense_medication(med)
                future.add_done_callback(lambda f: self.handle_pill_recognition_response(f, med, self.patient_name))

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def handle_pill_recognition_response(self, future, med, patient_name):
        try:
            response = future.result()
            pill_success = response.success
            medication_name = med.get('medication', 'medication')
            now = datetime.datetime.now(self.timezone)
            timestamp = now.strftime("%Y-%m-%d %H:%M:%S")

            # 🗣️ Add speech if pill recognition failed
            if not pill_success:
                msg = String()
                msg.data = "Please take your medication."
                self.text_speech_pub.publish(msg)

            pill_msg = String()
            pill_msg.data = (
                f"Patient: {patient_name}, "
                f"Dispensed: {medication_name}, "
                f"Pill Recognition: {pill_success}, "
                f"Timestamp: {timestamp}"
            )
            self.medication_firebase.publish(pill_msg)
            self.get_logger().info(f"Published to Firebase: {pill_msg.data}")

        except Exception as e:
            self.get_logger().error(f"Pill recognition service call failed: {e}")

    def cleanup_dispensed_records(self):
        now = datetime.datetime.now(self.timezone)
        expired_keys = []

        for key, timestamp in self.dispensed_records.items():
            if (now - timestamp).total_seconds() > 12 * 3600:  # 12 hours
                expired_keys.append(key)

        for key in expired_keys:
            del self.dispensed_records[key]
            self.get_logger().info(f"Cleaned up old dispense record: {key}")

    def dispense_medication(self, med):
        medication = med.get('medication', 'medication')
        compartment = med.get('compartment_id', 'unknown')
        self.get_logger().info(f"Dispensing {medication} in compartment {compartment}")

        # Simulated servo actions
        if compartment == 1:
            self.set_servo_angle(0, -20)
            print("Activating servo for compartment 1")
            time.sleep(15)
            self.set_servo_angle(0, 75)
        elif compartment == 2:
            self.set_servo_angle(3, -20)
            print("Activating servo for compartment 2")

            time.sleep(15)
            self.set_servo_angle(3, 75)
        elif compartment == 3:
            self.set_servo_angle(12, -20)
            print("Activating servo for compartment 3")
            time.sleep(15)
            self.set_servo_angle(12, 75)
        elif compartment == 4:
            self.set_servo_angle(15, -20)
            print("Activating servo for compartment 4")
            time.sleep(5)
            self.set_servo_angle(15, 75)
        else:
            print("Invalid compartment ID")


def main(args=None):
    rclpy.init(args=args)
    node = DispenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Dispense node stopped cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
