#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import board
import busio
from adafruit_pca9685 import PCA9685
import firebase_admin
from firebase_admin import credentials, firestore
import datetime
import pytz


class DispenseTestNode(Node):
    def __init__(self):
        super().__init__('dispense_test_node')
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # Initialize the PCA9685 at I2C address 0x40.
        self.pca = PCA9685(self.i2c, address=0x40)
        self.pca.frequency = 50  # Set frequency to 50 Hz (typical for servos).

        # Define pulse length limits for the servo (values are out of 4096).
        servo_min = 150  # Minimum pulse length (approximately 1 ms).
        servo_max = 600  # Maximum pulse length (approximately 2 ms).
        self.medication_question_pub = self.create_publisher(String, 'Medication_Questions', 10)
        self.text_speech_pub = self.create_publisher(String, 'text_speech', 10)
        self.dispense_timer = self.create_timer(60.0, self.check_medication_dispense)
        self.servo_min = 150  # Minimum pulse length (approximately 1 ms).
        self.servo_max = 600  # Maximum pulse length (approximately 2 ms).
        if firebase_admin._apps:
            firebase_admin.delete_app(firebase_admin.get_app())
        cred = credentials.Certificate("/home/medially/Downloads/medication-database.json")
        firebase_admin.initialize_app(cred)
        self.db = firestore.client()

        self.timezone = pytz.timezone("America/New_York")
        self.dispensed_records = {}  # {record_key: datetime_object}
        self.cleanup_timer = self.create_timer(3600.0, self.cleanup_dispensed_records)
        self.patient_name = 'Jane Doe'

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


    def check_medication_dispense(self):
        now = datetime.datetime.now(self.timezone)
        patients_ref = self.db.collection("Patient Information").where('name', '==', self.patient_name)
        patients = patients_ref.stream()

        for patient in patients:
            patient_data = patient.to_dict()
            medication_schedule = patient_data.get("medication_schedule", [])

            for med in medication_schedule:
                med_time_str = med.get("time", "").replace(" EST", "").strip()
                med_bool = med.get("taken")
                if not med_time_str:
                    continue

                try:
                    med_time = datetime.datetime.strptime(med_time_str, "%H:%M")
                    med_time = self.timezone.localize(med_time.replace(year=now.year, month=now.month, day=now.day))
                except ValueError:
                    self.get_logger().error(f"Invalid time format: {med_time_str}")
                    continue

                time_diff = abs((now - med_time).total_seconds())
                if time_diff <= 60 and not med_bool:
                    record_key = f"{self.patient_name}_{med.get('medication', 'medication')}_{med_time_str}"
                    if self.dispensed_records.get(record_key):
                        continue

                    # 1. Publish question to Medication_Questions
                    question = f"What are the usage requirements and side effects for {med.get('medication', 'the medication')}?"
                    question_msg = String()
                    question_msg.data = question
                    self.medication_question_pub.publish(question_msg)

                    # 2. Simulate dispensing
                    self.dispense_medication(med)

                    # 3. Publish to text_speech
                    medication = med.get('medication', 'medication')
                    compartment = med.get('compartment_id', 'unknown')
                    speech_msg = String()
                    speech_msg.data = (
                        f"The {medication} has been dispensed in compartment {compartment}. "
                        f"Please take the required medication amount."
                    )
                    self.text_speech_pub.publish(speech_msg)

                    self.dispensed_records[record_key] = datetime.datetime.now(self.timezone)

    def dispense_medication(self, med):
        medication = med.get('medication', 'medication')
        compartment = med.get('compartment_id', 'unknown')
        self.get_logger().info(f"Dispensing {medication} in compartment {compartment}")

        # Simulated servo actions
        if compartment == 1:
            self.set_servo_angle(0, -20)
            print("Activating servo for compartment 1")
            time.sleep(5)
            self.set_servo_angle(0, 75)
        elif compartment == 2:
            self.set_servo_angle(0, -20)
            print("Activating servo for compartment 2")

            time.sleep(5)
            self.set_servo_angle(0, 75)
        elif compartment == 3:
            self.set_servo_angle(0, -20)
            print("Activating servo for compartment 3")
            time.sleep(5)
            self.set_servo_angle(0, 75)
        elif compartment == 4:
            self.set_servo_angle(0, -20)
            print("Activating servo for compartment 4")
            time.sleep(5)
            self.set_servo_angle(0, 75)
        else:
            print("Invalid compartment ID")

    def cleanup_dispensed_records(self):
        now = datetime.datetime.now(self.timezone)
        expired_keys = [key for key, timestamp in self.dispensed_records.items()
                        if (now - timestamp).total_seconds() > 12 * 3600]

        for key in expired_keys:
            del self.dispensed_records[key]
            self.get_logger().info(f"Cleaned up old dispense record: {key}")


def main(args=None):
    rclpy.init(args=args)
    node = DispenseTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Dispense test node stopped cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
