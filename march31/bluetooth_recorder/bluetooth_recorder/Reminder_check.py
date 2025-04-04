#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import firebase_admin
from firebase_admin import credentials, firestore
import datetime
import pytz
import smtplib
from email.message import EmailMessage


class ReminderNode(Node):
    def __init__(self):
        super().__init__('reminder_node')
        # Publisher to the "Text_speech" topic
        self.publisher_ = self.create_publisher(String, 'text_speech', 10)
        # Timer to check medication schedules every 60 seconds
        self.timer_reminder = self.create_timer(60.0, self.check_medication_schedules)


        # Initialize Firebase (reinitialize if necessary)
        if firebase_admin._apps:
            firebase_admin.delete_app(firebase_admin.get_app())
        cred = credentials.Certificate(
            "/home/medially/Downloads/medication-database.json")
        firebase_admin.initialize_app(cred)
        self.db = firestore.client()

        # Timezone and email settings
        self.timezone = pytz.timezone("America/New_York")  # Adjust if needed
        self.nurse_email = 'mediallynurse@gmail.com'
        self.sender_email = "mediallynurse@gmail.com"  # Your email
        self.sender_password = "password"  # Your app password
        self.now = datetime.datetime.now(self.timezone)
        self.prev_now = None
        self.i = 0
    def check_medication_schedules(self):

        if self.i == 30:
            self.prev_now = self.now
            self.i = 0
        else:
            self.i = self.i+1
        self.now = datetime.datetime.now(self.timezone)

        # Query for a specific patient (adjust query as needed)
        patients_ref = self.db.collection("Patient Information").where('name', '==', 'Jane Doe')
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
                    med_time = self.timezone.localize(med_time.replace(year=self.now.year, month=self.now.month, day=self.now.day))
                except ValueError:
                    self.get_logger().error(f"Invalid time format for {patient_name}: {med_time_str}")
                    continue
                #self.get_logger(f"Time: {med_time - datetime.timedelta(minutes=30)}")
                # Check if the medication is coming up
                if (med_time - datetime.timedelta(minutes=30) <= self.now < med_time - datetime.timedelta(minutes=29)) and med_bool == False:
                    reminder_message = f"ALERT: This is a reminder that {patient_name} must take {med.get('medication', 'medication')} scheduled for {med_time_str}"
                    self.get_logger().info(reminder_message)

                    # Publish the reminder to the ROS2 topic
                    msg = String()
                    msg.data = reminder_message
                    self.publisher_.publish(msg)

                    # Optionally, send an email alert
                    self.send_email_alert_reminder(self.nurse_email, patient_name, med.get('medication', 'medication'),
                                          med_time_str)

                elif (self.now > med_time + datetime.timedelta(minutes=30) and med_bool is False and self.prev_now is not None and (self.now - self.prev_now) > datetime.timedelta(minutes=30)):

                    reminder_message = f"ALERT: {patient_name} missed their {med.get('medication', 'medication')} scheduled for {med_time_str}"
                    self.get_logger().info(reminder_message)

                    # Publish the reminder to the ROS2 topic
                    msg = String()
                    msg.data = reminder_message
                    self.publisher_.publish(msg)

                    # Optionally, send an email alert
                    self.send_email_alert(self.nurse_email, patient_name, med.get('medication', 'medication'), med_time_str)




    def send_email_alert(self, nurse_email, patient_name, medication, time_missed):
        msg = EmailMessage()
        msg.set_content(
            f"ALERT: {patient_name} missed their medication ({medication}) scheduled for {time_missed}. Please check on them.")
        msg["Subject"] = f"Medication Alert for {patient_name}"
        msg["From"] = self.sender_email
        msg["To"] = nurse_email

        try:
            with smtplib.SMTP_SSL("smtp.gmail.com", 465) as server:
                server.login(self.sender_email, self.sender_password)
                server.send_message(msg)
                self.get_logger().info(f"Email sent to {nurse_email}")
        except Exception as e:
            self.get_logger().error(f"Failed to send email: {e}")

    def send_email_alert_reminder(self, nurse_email, patient_name, medication, time_schedule):
        msg = EmailMessage()
        msg.set_content(
            f"ALERT: {patient_name} must take ({medication}) scheduled for {time_schedule}. Please check on them.")
        msg["Subject"] = f"Medication Alert for {patient_name}"
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
    node = ReminderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Reminder node stopped cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
