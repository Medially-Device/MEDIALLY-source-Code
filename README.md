# Smart Pillbox and Medication Assistant

A modular AI-powered system designed to reduce medication errors and support healthcare workers in Long-Term Care (LTC) facilities. This smart pillbox integrates **computer vision**, **natural language processing**, **secure cloud-based records**, and **automated pill dispensing** into a cost-effective and user-friendly device.

---

## 🔧 Project Structure

```
├── ComputerVision/      # Facial recognition and pill intake verification
├── LLM/                 # Language model reasoning and voice interaction
├── Firebase/            # Web interface and real-time database integration
├── Mechanical/          # CAD models, electronics, and system drawings
```

---

## 💡 Key Features

### ✅ Personalized Medication Assistant
- Uses an embedded LLM to answer user questions about medications (e.g., instructions, side effects, interactions).
- Real-time patient-specific responses using **Retrieval-Augmented Generation (RAG)** from Firebase.

### 👁️‍🗨️ Computer Vision Module
- **Facial recognition** (via DeepFace + DLIB) ensures correct user access.
- **Pose estimation + LSTM classifier** confirms pill ingestion using webcam input.

### 🌐 Firebase Integration
- Real-time database with secure login and access control.
- Web portal for staff to manage patient profiles, medication schedules, and system logs.

### 🛠 Mechanical System
- 3D-printed modular pillbox with electronic locking compartments.
- Integrated camera, servo-driven compartments, and physical interface.
- Powered by BeagleBone AI and Raspberry Pi with ROS2 communication.

---

## 🚀 How It Works

1. **At scheduled time**, user is notified via audio prompt.
2. **Facial recognition** is used to authenticate the user.
3. The appropriate compartment unlocks and provides **verbal instructions**.
4. The system uses pose-based video analysis to verify pill ingestion.
5. All interactions are logged in **Firebase** and accessible via the web portal.
6. **Voice assistant** allows the user to ask medication-related questions at any time.

---

## 📁 Repository Directory Overview

### `/ComputerVision`
- `Record_Predict.py`: Main pipeline for facial recognition and pose-based pill verification.
- `Model_Training.py`: LSTM model training for pill intake classification.
- `images/`: Landmark visuals, architecture diagrams, confusion matrices.

### `/LLM`
- `llm_pipeline.py`: Speech-to-text, intent classification, question answering.
- `intent_classifier/`: BioBERT fine-tuned model.
- `prompt_engineering/`: Prompt templates and query logic.
- `audio/`: gTTS integration and emergency alert triggers.

### `/Firebase`
- `firebase_config.js`: Web-based Firebase setup.
- `index.html`: Web interface for managing patients and medications.
- `rules/`: Firebase Firestore security rules and access policies.

### `/Mechanical`
- `cad_models/`: SolidWorks and STL files.
- `electronics/`: Wiring diagrams, Raspberry Pi and BeagleBone pinouts.
- `assembly_drawings/`: Exploded views and part specifications.
- `bom.csv`: Bill of Materials and cost analysis.

---

## 🧪 Testing Summary

- ✅ Facial recognition: 90% accuracy, 0 false positives in trials.
- ✅ Pill intake detection: 85% LSTM accuracy.
- ✅ LLM voice query handling: 84–94% accuracy across modules.
- ✅ Full system tested under real-world constraints (2-day supply, 4-medication compartments).

---

## 🔐 Security & Privacy

- All user data is encrypted and managed using **Google Firebase**.
- Complies with **PHIPA**, **GDPR**, **CCPA**, and **AIDA**.
- Access controlled via **Google Sign-In** and **2FA**.

---

## 🤝 Contributors

- Hannan Habibovic  
- Joshua Visser  
- Zohaib Saghir  
- Jasmun Banwait  

University of Guelph – ENGG*41x0 Final Capstone Project, Winter 2025
