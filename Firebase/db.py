
# The following imports are required to grant access to the Firebase Database
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
from datetime import datetime
import pytz 
# The following import is used for indicating the timestamp for data creation

if firebase_admin._apps:
    firebase_admin.delete_app(firebase_admin.get_app()) 

cred = credentials.Certificate("path/to/serviceAccountKey.json")
firebase_admin.initialize_app(cred)

db = firestore.client()

# Case #1: Retrieving patient information based on queries

# Case #1a: A question is asked related to retrieving all patient information 
# based on specific identifiers (i.e.: name and date of birth)
# Note: these are for variables stored directly in the patient database
# (ex: no array or mapped values)
patient_name = "Jane Doe";
patient_dob = "02/05/1961";
condition = "Hypertension"

collection_ref = db.collection("Patient Information").where("name", "==", patient_name).where("dob", "==", patient_dob);

patient_docs = collection_ref.stream()

for patient_doc in patient_docs:
    print(f"{patient_doc.id} => {patient_doc.to_dict()}")
    
    # Case #1b: A question is asked to retrieve only select patient information
    # based on specific identifiers (ex: retrieiving medical conditions)
    # if you want to introduce multiple for loops, you need to redefine 
    # patient_docs and set it = collection_ref.stream() again. Otherwise, put
    # it in the same for loop

    medical_conditions = patient_doc.get("medical_conditions")  # Retrieve only the 'medication_conditions' field
    print(f"Medical Conditions: {medical_conditions}")
    
    # Case #1c: Retrieving a specific medication based on a certain condition
    # (i.e. what medications am I required to take for condition A?)
    
    medForCond = patient_doc.get(f"medical_conditions.{condition}.medications")
    # Join medications into a single string, separated by commas
    medications_str = ', '.join(medForCond)
    print(f"Medications for {condition}: {medications_str}")
    
med_collection = db.collection("Medication Information")
med_docs = med_collection.stream()

for med_doc in med_docs:
    data = med_doc.to_dict()
    inventory = data.get("inventory")
    if isinstance(inventory, int):
        print("Inventory is an int")
    elif isinstance(inventory, float):
        print("Inventory is a float")
    else:
        print(f"Other type: {type(inventory)}")
    
# Case #2: Retrieving medication information based on queries
medication_name = "Acetaminophen"

# Reference the document by its name
medication_ref = db.collection("Medication Information").document(medication_name)

# Get the document snapshot
medication_doc = medication_ref.get()

inventory = {"inventory": 11};

# Check if the document exists before accessing its data
if medication_doc.exists:
    print(medication_doc.to_dict())  # Print the document data
    medication_ref.update(inventory)
else:
    print(f"Medication '{medication_name}' not found in Firestore.")


now = datetime.now()

# Convert to local timezone (optional)
local_tz = pytz.timezone("America/New_York")  # Change to your timezone
now_local = now.astimezone(local_tz)

# Format timestamp in military time (24-hour format)
formatted_timestamp = now_local.strftime("%Y-%m-%d %H:%M:%S %Z") 
    
# Case #3: Adding/updating information 

patient_data = {
    "name": patient_name,
    "dob": patient_dob,
    "medication_schedule": [
  {
    "condition": "Hypertension",
    "dosage": "10 mg",
    "medication": "Lisinopril",
    "time": "7:30 EST",
    "taken": True,
    "time_taken": "",
    "compartment_id": 1
  },
  {
    "condition": "Hypertension",
    "dosage": "5 mg",
    "medication": "Amlodipine",
    "time": "20:00 EST",
    "taken": True,
    "time_taken": "",
    "compartment_id": 2
  },
  {
    "condition": "Osteoarthritis",
    "dosage": "500 mg",
    "medication": "Acetaminophen",
    "time": "8:00 EST",
    "taken": True,
    "time_taken": "",
    "compartment_id": 3
  },
  {
    "condition": "Osteoarthritis",
    "dosage": "500 mg",
    "medication": "Acetaminophen",
    "time": "14:00 EST",
    "taken": True,
    "time_taken": "",
    "compartment_id": 3
  },
  {
    "condition": "Osteoarthritis",
    "dosage": "500 mg",
    "medication": "Acetaminophen",
    "time": "20:00 EST",
    "taken": True,
    "time_taken": "",
    "compartment_id": 3
  },
  {
    "condition": "Mild Pain",
    "dosage": "200 mg",
    "medication": "Ibuprofen",
    "time": "10:00 EST",
    "taken": True,
    "time_taken": "",
    "compartment_id": 4
  }
]

}


# Function to add or update patient data
def add_or_update_patient(patient_name, patient_dob, patient_data):
    # Query to find the patient by name and dob
    query_ref = db.collection("Patient Information").where("name", "==", patient_name).where("dob", "==", patient_dob)

    pdocs = query_ref.stream()

    # Check if a patient exists
    patient_found = False
    for pdoc in pdocs:
        patient_found = True
        patient_ref = db.collection("Patient Information").document(pdoc.id)
        # Update the existing patient document
        print(f"Patient found. Updating the document: {pdoc.id}")
        patient_ref.update(patient_data)
        print("Patient document updated.")
        break
    
    if not patient_found:
        # If patient does not exist, create a new document
        print("Patient not found. Adding new patient document.")
        db.collection("Patient Information").add(patient_data)
        print("New patient document added.")

add_or_update_patient(patient_name, patient_dob, patient_data)

medication_data = {
    "active_ingredient": "Amlodipine besylate 5 mg",
    "brand_name": "Norvasc",
    "common_side_effects": "Swelling of the ankles or feet, headache, dizziness, fatigue, nausea, flushing",
    "compartment_id": 2,
    "dosage": {
        "Angina": "5-10mg, once daily",
        "Hypertension": "5mg, once daily (adjustable up to 10mg based on response)"
    },
    "drug_interactions": "Simvastatin, CYP3A4 inhibitors (e.g., Ketoconazole), Rifampin, Phenytoin",
    "expiration_date": "Please refer to the product packaging for the specific expiration date.",
    "inactive_ingredients": [
        "Microcrystalline cellulose",
        "Calcium phosphate dibasic",
        "Sodium starch glycolate",
        "Magnesium stearate",
        "Colloidal silicon dioxide",
        "Titanium dioxide"
    ],
    "inventory": 12,
    "lot_number": "Please refer to the product packaging for the specific lot number.",
    "manufacturer_information": {
        "name": "Pfizer Inc.",
        "contact": {
            "phone": "1-800-438-1985",
            "international_phone": "1-212-733-2323"
        }
    },
    "medical_conditions": "Hypertension, Angina",
    "medication": "Amlodipine",
    "onset_of_action": "6 to 12 hours for initial blood pressure reduction, peak effect in 6 to 9 days",
    "other_interactions": "Grapefruit juice, Simvastatin, CYP3A4 inhibitors",
    "severe_side_effects": "Severe dizziness, swelling of the hands/ankles/feet, difficulty breathing, low blood pressure",
    "storage": "Store at room temperature, 20-25ºC. Protect from light and moisture. Keep out of reach of children.",
    "usage_instructions": {
        "activity": "Avoid grapefruit and grapefruit juice as it may affect drug absorption.",
        "food": "Can be taken with or without food.",
        "time": "Take once daily at the same time each day.",
        "water": "Take with a full glass of water."
    },
    "warnings": [
        "May cause dizziness—avoid sudden movements from sitting or lying down.",
        "Monitor blood pressure regularly.",
        "Possible swelling in ankles or feet—report any severe swelling or difficulty breathing.",
        "Avoid excessive alcohol consumption as it can lower blood pressure further."
    ]
}



med_name = "Amlodipine"

# Function to add or update patient data
def add_or_update_medication(medication_name, medication_data):
    # Reference the document with the medication name as the document ID
    med_ref = db.collection("Medication Information").document(medication_name)
    
    # Check if the document already exists
    if med_ref.get().exists:
        print(f"Medication '{medication_name}' found. Updating the document.")
        med_ref.update(medication_data)  # Update existing document
        print("Medication document updated.")
    else:
        print(f"Medication '{medication_name}' not found. Adding new medication document.")
        med_ref.set(medication_data)  # Correct: Set the medication data
        print("New medication document added.")
        
add_or_update_medication(med_name, medication_data)
    

# add brand_name and interaction for each medication_schedule drug array




    


    