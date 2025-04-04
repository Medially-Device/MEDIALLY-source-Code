#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import firebase_admin
from firebase_admin import credentials, firestore
import datetime
import pytz


class DispenseNode(Node):
    def __init__(self):
        super().__init__('dispense_node')
        # Publisher to the "Medication_Questions" topic for retrieving usage instructions and side effects
        self.medication_question_pub = self.create_publisher(String, 'Medication_Questions', 10)
        # Timer to check for dispensing medications every 60 seconds
        self.dispense_timer = self.create_timer(60.0, self.check_medication_dispense)

        # Initialize Firebase (reinitialize if necessary)
        if firebase_admin._apps:
            firebase_admin.delete_app(firebase_admin.get_app())
        cred = credentials.Certificate("/home/medially/Downloads/medication-database.json")
        firebase_admin.initialize_app(cred)
        self.db = firestore.client()

        # Timezone settings (adjust if needed)
        self.timezone = pytz.timezone("America/New_York")

        # Dictionary to track medications already dispensed for a given schedule to avoid duplicates


    def check_medication_dispense(self):
        now = datetime.datetime.now(self.timezone)
        # Query for a specific patient (adjust query as needed)
        patients_ref = self.db.collection("Patient Information").where('name', '==', 'Jane Doe')
        patients = patients_ref.stream()

        for patient in patients:
            patient_data = patient.to_dict()
            patient_name = patient_data.get("name", "Unknown")
            medication_schedule = patient_data.get("medication_schedule", [])

            for med in medication_schedule:
                med_time_str = med.get("time", "").replace(" EST", "").strip()
                if not med_time_str:
                    continue

                try:
                    med_time = datetime.datetime.strptime(med_time_str, "%H:%M")
                    med_time = self.timezone.localize(
                        med_time.replace(year=now.year, month=now.month, day=now.day)
                    )
                except ValueError:
                    self.get_logger().error(f"Invalid time format for dispensing {patient_name}: {med_time_str}")
                    continue

                # Check if it's time to dispense (within a 30-second tolerance)
                time_diff = abs((now - med_time).total_seconds())
                if time_diff <= 60:
                    # Prevent re-dispensing for the same medication schedule
                    record_key = f"{patient_name}_{med.get('medication', 'medication')}_{med_time_str}"
                    if self.dispensed_records.get(record_key):
                        continue  # Skip if already dispensed

                    # Publish a query to request usage instructions and side effects before dispensing
                    question = f"What are the usage requirements and side effects for {med.get('medication', 'the medication')}?"
                    question_msg = String()
                    question_msg.data = question
                    self.medication_question_pub.publish(question_msg)

                    self.get_logger().info(f"Published query for {med.get('medication', 'medication')} scheduled at {med_time_str}")

                    # Simulate dispensing the medication
                    self.dispense_medication(med)

                    # Mark as dispensed to avoid duplicate processing


    def dispense_medication(self, med):
        medication = med.get('medication', 'medication')
        self.get_logger().info(f"Dispensing {medication}")
        # TODO: Insert hardware control code here to actually dispense the medication.


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
