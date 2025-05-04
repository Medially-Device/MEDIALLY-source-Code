#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sentence_transformers import SentenceTransformer
import hnswlib
import numpy as np
import firebase_admin
from firebase_admin import credentials, firestore
#This works without memory
class VectorizerNode(Node):
    def __init__(self):
        super().__init__('vectorizer_node')

        # ROS2 Subscriptions & Publishers
        self.subscription = self.create_subscription(String, 'vectorizer_input_medication', self.handle_question, 10)
        self.qa_publisher = self.create_publisher(String, 'vectorizer_output_medication', 10)
        self.storage_subscription = self.create_subscription(String, 'vectorizer_store_medication', self.store_question_answer, 10)

        # Load Embedding Model
        self.embedding_model = SentenceTransformer("all-MiniLM-L6-v2")

        # Initialize HNSWlib Index
        self.dim = 384  # Embedding dimension (for all-MiniLM-L6-v2)
        self.index = hnswlib.Index(space='l2', dim=self.dim)
        # Initialize with a generous maximum number of elements; adjust as needed.
        self.index.init_index(max_elements=10000, ef_construction=200, M=16)
        # Set the ef parameter for query time performance (tradeoff between speed and accuracy)
        self.index.set_ef(200)
        self.stored_texts = []  # List to store the original text corresponding to each vector

        # Initialize Firebase
        if not firebase_admin._apps:
            cred = credentials.Certificate("/home/medially/Downloads/medication-database.json")
            firebase_admin.initialize_app(cred)
        self.db = firestore.client()
        self.patient_name = "Jane Doe"
        # Vectorize all Firebase Data on Startup
        patient_info = self.get_patient_info(patient_name=self.patient_name)
        self.get_medications_for_patient(patient_info)
        self.get_logger().info("Vectorizer Node Ready using HNSWlib.")



    def get_patient_info(self, patient_name):
        """Retrieve and store patient information in a variable."""
        # Retrieve the patient's information by name
        patient_docs = self.db.collection("Patient Information").where("name", "==", patient_name).stream()

        patient_data = {}
        for doc in patient_docs:
            patient_data = doc.to_dict()

        # Store patient data in a variable
        return patient_data

    def get_medications_for_patient(self, patient_data):
        """Retrieve the medications for the given patient."""
        medications = patient_data.get("medical_conditions", {})
        for condition, details in medications.items():
            for medication in details.get("medications", []):
                self.vectorize_firebase_data(medication)

    def vectorize_firebase_data(self, medication):
        """Retrieves patient data from Firebase and stores it as vector embeddings."""
        medication_docs = self.db.collection("Medication Information").where("medication", "==", medication).stream()
        if not medication_docs:
            self.get_logger().error("No patient data found!")
            return

        for doc in medication_docs:
            data = doc.to_dict()

            # General Medication Information


            # Medical Dosage
            for condition, dosage in data.get("dosage", {}).items():
                general_info = f"Medication: {data.get('medication', 'N/A')}, " \
                               f"{data.get('medication', 'N/A')}: Brand Names: {data.get('brand_name', 'N/A')}, " \
                               f"{data.get('medication', 'N/A')}: Active Ingredient: {data.get('active_ingredient', 'N/A')}."
                self.store_text(general_info)

                # Dosage Instructions
                for condition, dosage in data.get("dosage", {}).items():
                    self.store_text(f"{data.get('medication', 'N/A')}: Dosage for {condition}: {dosage}.")

                # Side Effects
                self.store_text(
                    f"{data.get('medication', 'N/A')}: Common Side Effects: {data.get('common_side_effects', 'N/A')}.")
                self.store_text(
                    f"{data.get('medication', 'N/A')}: Severe Side Effects: {data.get('severe_side_effects', 'N/A')}.")

                # Drug Interactions
                self.store_text(
                    f"{data.get('medication', 'N/A')}: Drug Interactions: {data.get('drug_interactions', 'N/A')}.")

                # Other Interactions
                if "other_interactions" in data:
                    self.store_text(
                        f"{data.get('medication', 'N/A')}: Other Interactions: {data['other_interactions']}.")

                # Medical Conditions Treated
                self.store_text(f"{data.get('medication', 'N/A')}: Used for: {data.get('medical_conditions', 'N/A')}.")

                # Storage Information
                self.store_text(f"{data.get('medication', 'N/A')}: Storage Instructions: {data.get('storage', 'N/A')}.")

                # Manufacturer Information
                manufacturer_info = data.get("manufacturer_information", {})
                contact_info = manufacturer_info.get("contact", {})
                manufacturer_text = (
                    f"{data.get('medication', 'N/A')}: Manufacturer: {manufacturer_info.get('name', 'N/A')}, "
                    f"{data.get('medication', 'N/A')}: Contact: {contact_info.get('phone', 'N/A')} "
                    f"{data.get('medication', 'N/A')}: (International: {contact_info.get('international_phone', 'N/A')})."
                )
                self.store_text(manufacturer_text)

                # Expiration and Lot Number
                self.store_text(
                    f"{data.get('medication', 'N/A')}: Expiration Date: {data.get('expiration_date', 'N/A')}.")
                self.store_text(f"{data.get('medication', 'N/A')}: Lot Number: {data.get('lot_number', 'N/A')}.")

                # Usage Instructions
                usage_instructions = data.get("usage_instructions", {})
                usage_text = (
                    f"{data.get('medication', 'N/A')}: Usage Instructions: Take {usage_instructions.get('time', 'N/A')}, {usage_instructions.get('dose', 'N/A')}, {usage_instructions.get('water', 'N/A')} "
                    f"{data.get('medication', 'N/A')}: {usage_instructions.get('food', 'N/A')}. {usage_instructions.get('activity', 'N/A')}."
                )
                self.store_text(usage_text)

                # Warnings
                warnings = data.get("warnings", [])
                self.store_text(
                    f"{data.get('medication', 'N/A')}: Warnings: {', '.join(warnings) if warnings else 'N/A'}.")

                # Inactive Ingredients
                inactive_ingredients = data.get("inactive_ingredients", [])
                self.store_text(
                    f"{data.get('medication', 'N/A')}: Inactive Ingredients: {', '.join(inactive_ingredients) if inactive_ingredients else 'N/A'}.")

                # Inventory & Compartment Info
                self.store_text(
                    f"{data.get('medication', 'N/A')}: Inventory: {data.get('inventory', 'N/A')} units available.")
                self.store_text(
                    f"{data.get('medication', 'N/A')}: Stored in Compartment ID: {data.get('compartment_id', 'N/A')}.")

                # Water Intake Recommendation
                if "water" in data:
                    self.store_text(f"{data.get('medication', 'N/A')}: Water Recommendation: {data['water']}.")

    def handle_question(self, msg: String):
        """Retrieves relevant patient information for a given question."""
        full_msg = msg.data

        # Extract question ID and question text
        try:
            question_id, question = full_msg.split(":", 1)
        except ValueError:
            self.get_logger().error("Invalid input format. Expected 'question_id:question_text'.")
            return

        self.get_logger().info(f"Received question (ID {question_id}): {question}")

        # Convert question to embedding vector
        query_vector = np.array([self.embedding_model.encode(question)])

        # Check the current number of stored vectors
        current_count = self.index.get_current_count()
        self.get_logger().info(f"Current index count: {current_count}")
        if current_count < 3:
            self.get_logger().error("Not enough vectors stored in the index to perform a k-NN query.")
            return

        try:
            labels, distances = self.index.knn_query(query_vector, k=6)
        except RuntimeError as e:
            self.get_logger().error(f"Error during knn_query: {e}")
            return

        relevant_info = [self.stored_texts[label] for label in labels[0] if label < len(self.stored_texts)]
        context_str = "\n".join(relevant_info)

        # Prepend the question ID to the response so the client can match it
        output_msg = String()
        output_msg.data = f"{question_id}:{context_str}"
        self.qa_publisher.publish(output_msg)

        self.get_logger().info(f"Sent retrieved context to QA Client: {output_msg.data}")

    def store_text(self, text):
        vector = self.embedding_model.encode(text)
        new_label = len(self.stored_texts)
        self.get_logger().info(f"Storing vector with label {new_label} and text: {text[:30]}...")
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
