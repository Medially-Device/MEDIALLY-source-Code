#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sentence_transformers import SentenceTransformer
import hnswlib
import numpy as np
import firebase_admin
from firebase_admin import credentials, firestore
#This works with memory
class VectorizerNode(Node):
    def __init__(self):
        super().__init__('vectorizer_node')

        # ROS2 Subscriptions & Publishers
        self.subscription = self.create_subscription(String, 'vectorizer_input_Patient', self.handle_question, 10)
        self.qa_publisher = self.create_publisher(String, 'vectorizer_output_Patient', 10)
        self.storage_subscription = self.create_subscription(String, 'vectorizer_store_Patient', self.store_question_answer, 10)

        # Load Embedding Model
        self.embedding_model = SentenceTransformer("all-MiniLM-L6-v2")

        # Initialize HNSWlib Index
        self.dim = 384  # Embedding dimension (for all-MiniLM-L6-v2)
        self.index = hnswlib.Index(space='l2', dim=self.dim)
        # Initialize with a generous maximum number of elements; adjust as needed.
        self.index.init_index(max_elements=10000, ef_construction=200, M=16)
        # Set the ef parameter for query time performance (tradeoff between speed and accuracy)
        self.index.set_ef(50)
        self.stored_texts = []  # List to store the original text corresponding to each vector

        # Initialize Firebase
        if not firebase_admin._apps:
            cred = credentials.Certificate("/home/medially/Downloads/medication-database.json")
            firebase_admin.initialize_app(cred)
        self.db = firestore.client()

        # Vectorize all Firebase Data on Startup
        self.vectorize_firebase_data()
        self.get_logger().info("Vectorizer Node Ready using HNSWlib.")

    def vectorize_firebase_data(self):
        """Retrieves patient data from Firebase and stores it as vector embeddings."""
        patient_name = "Jane Doe"
        patient_dob = "02/05/1961"
        collection_ref = self.db.collection("Patient Information") \
            .where("name", "==", patient_name) \
            .where("dob", "==", patient_dob)
        patient_docs = list(collection_ref.stream())
        if not patient_docs:
            self.get_logger().error("No patient data found!")
            return

        for doc in patient_docs:
            data = doc.to_dict()

            # General Patient Information
            general_info = f"Patient: {data['name']}, DOB: {data['dob']}, Gender: {data['gender']}, Weight: {data['weight_kgs']} kg, Blood Type: {data['blood_type']}."
            self.store_text(general_info)

            # Medical Conditions
            for condition, details in data.get("medical_conditions", {}).items():
                condition_text = f"Condition: {condition}, Medications: {', '.join(details['medications'])}."
                self.store_text(condition_text)

            # Medication Schedule
            for med_entry in data.get("medication_schedule", []):
                med_text = f"{med_entry['medication']} {med_entry['dosage']} for {med_entry['condition']} at {med_entry['time']}."
                self.store_text(med_text)

            # Advance Directives
            if "advance_directives" in data:
                directives = data["advance_directives"]
                directives_text = (
                    f"Advance Directives: DNR: {directives.get('DNR', 'Unknown')}, "
                    f"Living Will: {directives.get('living_will_on_file', 'Unknown')}, "
                    f"Organ Donor: {directives.get('organ_donor', 'Unknown')}."
                )
                self.store_text(directives_text)

            # Allergies
            if "allergies" in data:
                allergies_text = f"Allergies: {', '.join(data['allergies'])}."
                self.store_text(allergies_text)

            # Dietary Restrictions
            if "dietary_restrictions" in data:
                diet_text = f"Dietary Restrictions: {', '.join(data['dietary_restrictions'])}."
                self.store_text(diet_text)

            # Family Medical History
            for parent, conditions in data.get("family_medical_history", {}).items():
                history_text = f"Family Medical History ({parent}): {', '.join(conditions)}."
                self.store_text(history_text)

            # Emergency Contact
            emergency_text = f"Emergency Contact: {data['emergency_contact']['name']}, Phone: {data['emergency_contact']['phone']}, Email: {data['emergency_contact']['email']}."
            self.store_text(emergency_text)

            # Primary Care Physician
            physician_text = f"Primary Care Physician: {data['primary_care_physician']['name']}, Phone: {data['primary_care_physician']['phone']}, Email: {data['primary_care_physician']['email']}."
            self.store_text(physician_text)

            # Vaccination History
            if "vaccination_history" in data:
                vaccines = [f"{vaccine}: {date}" for vaccine, date in data["vaccination_history"].items()]
                vaccine_text = f"Vaccination History: {', '.join(vaccines)}."
                self.store_text(vaccine_text)

            # Recent Appointment Notes
            for appointment in data.get("appointment_notes", []):
                appointment_text = f"Appointment ({appointment['date']}): {appointment['notes']}."
                self.store_text(appointment_text)

            insurance = data.get("insurance", {})
            if insurance:
                insurance_info = (
                    f"Insurance Provider: {insurance.get('provider', 'N/A')} "
                    f"Group Number: {insurance.get('group_number', 'N/A')}. "
                    f"Policy Number: {insurance.get('policy_number', 'N/A')}."
                )
                self.store_text(insurance_info)


    def handle_question(self, msg: String):
        """Retrieves relevant patient information for a given question."""
        question = msg.data
        self.get_logger().info(f"Received question: {question}")

        # Convert question to embedding vector
        query_vector = np.array([self.embedding_model.encode(question)])

        # Use HNSWlib to perform k-NN query
        labels, distances = self.index.knn_query(query_vector, k=6)

        # Retrieve matching stored text using the returned labels
        relevant_info = [self.stored_texts[label] for label in labels[0] if label < len(self.stored_texts)]
        context_str = "\n".join(relevant_info)

        # Publish retrieved data to the QA Client
        output_msg = String()
        output_msg.data = context_str
        self.qa_publisher.publish(output_msg)
        self.get_logger().info(f"Sent retrieved context to QA Client: {output_msg.data}")

    def store_text(self, text):
        """Stores new patient-related data into HNSWlib for retrieval."""
        vector = self.embedding_model.encode(text)
        # Use the next available integer as the label
        new_label = len(self.stored_texts)
        self.index.add_items(np.array([vector]), np.array([new_label]))
        self.stored_texts.append(text)

    def store_question_answer(self, msg: String):
        """Stores newly generated Q&A pairs for future retrieval."""
        # You can use the same store_text method to add new Q&A pairs.
        text = msg.data
        self.store_text(text)
        self.get_logger().info(f"Stored new Q&A pair in vector database: {text}")

def main(args=None):
    rclpy.init(args=args)
    node = VectorizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
