# 💊 Smart Pillbox – LLM Module for a Smart Medical Assistant

## 🧠 Overview

**Smart Pillbox LLM Module**  
This repository contains the **Large Language Model (LLM)** component of a **Smart Pillbox** system designed to support **Long-Term Care (LTC)** healthcare workers and patients.

The LLM module functions as an **intelligent voice assistant**, capable of:

- 🗣️ Answering patient questions about medications and health
- 💡 Providing medication guidance
- 🧭 Assisting with timing and procedure for medication intake
- 🤝 Integrating with a pill dispensing system

## 🎯 Purpose

This module is a core component of a larger **Smart Medical Assistant** system, which aims to:

- 📈 Improve **medication adherence**
- ⏱️ Reduce **nurse workload** in long-term care environments
- 🧓 Enhance **patient autonomy and safety**

## 🔁 LLM Module Workflow

The **Smart Pillbox LLM Module** follows a structured pipeline that turns voice input into medically grounded answers, personalized to the patient.

### 🧩 Typical Workflow

#### 🎙️ 1. Voice Input
- The patient or nurse **presses a talk button** on the device.
- A **microphone** (e.g., headset) records the user’s speech.
- Upon release, audio is captured for processing.
- A **ROS2 Speech-to-Text Node** transcribes the audio using an STT engine or API.

#### ✍️ 2. Query Formulation
- The transcribed text is optionally processed by a **sentence fixer**:
  - A lightweight prompt-based LLM refines and clarifies phrasing.
  - Ensures proper formatting and domain-specific structure.

#### 🧠 3. Central LLM Reasoning
- The text is sent to the **LLM Node**, which:
  - Runs **LLaMA 3 (8B parameters, 8192-token context)** via the **Groq API**.
  - Offloads computation to the cloud to preserve edge device resources.
  - Uses **LangChain** for:
    - Prompt engineering
    - Maintaining short-term memory across dialogue turns

#### 🧭 4. Intent Classification
A fine-tuned **BioBERT (base-cased v1.1)** model classifies the query into one of four categories:

- **Patient History Questions** – queries about individual medical records or logs
- **General Medical Questions** – general drug/condition-related queries
- **Emergency Requests** – urgent help, falls, or health emergencies
- **Out-of-Domain (General/Other)** – casual or unrelated conversation

#### 🧰 5. Query Handling & Retrieval-Augmented Generation (RAG)

- **🩺 Patient History**
  - Retrieves relevant data from **Firebase** (e.g., last doses, vitals)
  - Uses **384-dim embeddings** (MiniLM-L6-v2) and **HNSW vector search**
  - Fuses top matches into the LLM prompt for personalized answers

- **💊 Medical/Medication Questions**
  - Searches a **medical knowledge base** for relevant info
  - Limits search space to reduce computation and cost
  - Factual results are combined into the LLM prompt

- **🚨 Emergency Requests**
  - Bypasses LLM; issues a **predefined emergency voice response**
  - Triggers an **email/notification alert** to caregivers with timestamp

- **🌐 Out-of-Domain Queries**
  - Routed to **OpenAI GPT-3.5 Turbo API**
  - Limited to **non-medical/general responses** (e.g., jokes, weather)
  - Response capped to **1000 tokens** (input + output)

#### 🧾 6. Answer Generation
- For Patient History and Medical queries:
  - LLaMA 3 generates a contextual answer based on embedded data and prompts
  - **LangChain memory** supports continuity in multi-turn conversations
- For Out-of-Domain queries:
  - **GPT-3.5 Turbo** provides a brief friendly response
- For Emergencies:
  - A predefined script is used to ensure instant feedback

#### 🔊 7. Voice Output & Logging
- Final answer is passed to a **Text-to-Speech Node (gTTS)** for audio playback
- Audio is played through the device to the patient
- Both **user query and system response** are logged to **Firebase QA Logs**, enabling:
  - 📈 Patient engagement tracking
  - 🛡️ Clinical audit trail
  - 🔁 Transparent care review by staff

---

### ✅ Key Capabilities

- 🗣️ Natural, voice-based patient interaction
- 🩺 Personalized, medically informed responses
- ⚡ Lightweight edge-cloud integration using **Groq**, **Firebase**, and **LangChain**
- 📊 Logging and traceability for safety and transparency


## ⚙️ Installation and Setup

To set up the **LLM Module** on a **Raspberry Pi** (or any Linux machine) with **ROS 2**, follow these steps:

### ✅ Prerequisites

- Install **ROS 2 Humble** (or **Foxy**) on your device.
- Ensure **Python 3** is available and up to date.
- Install required **system dependencies**:
  - Audio libraries (if using a microphone/speaker)
  - Camera drivers (if using the vision module)
- Ensure a **working internet connection** for access to:
  - **Groq API** (for LLaMA 3)
  - **OpenAI GPT API** (fallback)
  - **Firebase** (for patient data and logs)

> 🛠️ *In this project:*
> - The **LLM Module** runs on a **Raspberry Pi**
> - The **vision component** runs on a **BeagleBone AI**
> - You may alternatively run the full system on a **single Linux device** if all hardware is locally attached

---

### 📥 Step 1: Clone the Repository

Clone this repository into your ROS 2 workspace:


cd ~/ros2_ws/src
git clone https://github.com/YourUser/SmartPillbox-LLM-Module.git


- It is recommended to use a Python virtual environment. From the root of your workspace or inside the bluetooth_recorder folder, run:


- pip install -r bluetooth_recorder/bluetooth_recorder/requirements.txt
- This will install packages like:

- transformers

- torch

- langchain

- firebase_admin

## 🚀 Build and Launch (ROS 2)

Once the dependencies are installed and the packages are added to your ROS 2 workspace, you can build and run the system.

### 🛠️ Step 1: Compile ROS 2 Packages and Interfaces

From the root of your workspace:


cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select medially_interfaces bluetooth_recorder
source install/setup.bash

###🚦 Step 2: Launch the LLM Module
Use the provided ROS 2 launch file to start all required nodes:

ros2 launch bluetooth_recorder bluetooth_recorder_launch.py
This launch file will automatically start all necessary components in the correct order (with delay buffers for model loading), including:

- 🗣️ Speech-to-Text Node

- 🧠 LLM Service Node

- 📦 Intent Classification Node

- 🧾 Query Handler Clients

- 🔊 Text-to-Speech Node

- 💊 Dispensing-related Nodes (if integrated)

If successful, you’ll see console log output for each node as they initialize.


#### Repository Structure
- The repository is organized into two main packages, reflecting the ROS2 project structure. Below is a breakdown of the folders and key scripts, with emphasis on the LLM-related components:

smart-pillbox-LLM-module/
├── bluetooth_recorder/            # Main ROS2 package for the Smart Pillbox assistant (LLM & device logic)
│   ├── package.xml                # ROS2 package configuration (defines dependencies)
│   ├── setup.py                   # Setup script for the Python package
│   ├── launch/
│   │   └── bluetooth_recorder_launch.py   # Launch file to start all nodes of the system (LLM + dispensing)
│   ├── bluetooth_recorder/        # Python module directory for the package
│   │   ├── **speech_to_text_node.py**       # Node: Listens to microphone input (via button press) and performs Speech-to-Text 
│   │   ├── **llm_sentence_fixer_client.py** # Node: Takes raw transcriptions and refines them into well-formed questions (calls LLM service with "sentence_fixer" role)
│   │   ├── **bio_bert_intent_node.py**     # Node: Intent classification using BioBERT – categorizes the query into 4 types
│   │   ├── **LLM_service_node_v6.py**      # Node: The core LLM service provider. Handles LLMRequest service calls, uses LLaMA 3 via API and GPT-3.5 for responses
│   │   ├── **Patient_history_client_v2.py** # Node: Handles patient history queries. Retrieves relevant patient data from Firebase and uses LLM service to generate answers (RAG)
│   │   ├── **Medication_client.py**        # Node: Handles general medical questions. Searches medical knowledge base and calls LLM service to answer.
│   │   ├── **Emergency_question_client.py**# Node: Handles emergency queries. Sends alert emails to staff and provides a predefined urgent response 
│   │   ├── **Irrelevant_questions.py**     # Node: (Irrelevant_Node) possibly filters or handles queries deemed irrelevant/off-topic (to avoid unnecessary processing).
│   │   ├── **text_speech_node.py**         # Node: Text-to-Speech output using gTTS – converts answer text to audio and plays it:contentReference[oaicite:37]{index=37}.
│   │   ├── text_speech_v2.py              # (Updated version of the TTS node with improvements; launch uses this one.)
│   │   ├── **Chat_summary_node.py**        # Node: Monitors the ongoing conversation and generates summaries using a local HuggingFace model (to manage long-term context via LangChain memory).
│   │   ├── **vectorization_database_Patient_History.py** # Utility/Node: Pre-computes or loads the vector index for patient history data (using MiniLM embeddings and HNSW index).
│   │   ├── **vectorization_database_medication_v2.py**   # Utility/Node: Prepares the vector index for medical knowledge data.
│   │   ├── **Firebase_upload.py**         # Node: Handles logging of Q&A pairs and events to Firebase (runs as Firebase_node).
│   │   ├── **Reminder_check.py**          # Node: Periodically checks medication schedule (from Firebase) and sends reminders or alerts if doses are missed.
│   │   ├── **Medication_dispense.py** (and _v2.py, _v3.py) # Scripts for controlling the pill dispenser mechanism (servo motor control logic for releasing pills).
│   │   ├── **Servo_motor.py** and **servo_angle.py**      # Low-level control for the servo motors (e.g., functions to rotate servo to certain angles to open specific compartments).
│   │   ├── **Button.py**                  # Helper script to interface with the physical push-button (GPIO input on the Pi) that triggers voice recording.
│   │   ├── **recording_v2.py**            # Node: Manages audio recording from the microphone when the button is pressed, saving audio to file or stream for STT node.
│   │   ├── **bluetooth_recorder_node.py** # (Possibly deprecated by recording_v2) Initial node for recording via Bluetooth audio.
│   │   ├── **BeagleY_Facial.py**          # Node: Interfaces with the BeagleBone AI’s camera feed to perform facial recognition (DLIB/DeepFace) for identity verification.
│   │   ├── **BeagleY_AI_Client.py**       # (Related to BeagleBone) Could be a client to offload some AI tasks to the BeagleBone (named “BeagleY-AI” in the project:contentReference[oaicite:38]{index=38}).
│   │   ├── requirements.txt             # Python requirements file listing all needed libraries for this package.
│   │   └── (other utility scripts and ROS node implementations as needed)
│   ├── resource/
│   │   └── bluetooth_recorder            # Resource marker file for ROS2 (empty file required for Python packages).
│   └── test/                             # Test files for linting (flake8, etc.) – not directly relevant to functionality.
├── medially_interfaces/           # ROS2 package for custom service definitions (interfaces)
│   ├── CMakeLists.txt             # CMake config for building interfaces
│   ├── package.xml                # Package info (depends on rosidl interfaces)
│   └── srv/
│       ├── **LLMRequest.srv**        # Service definition for LLM queries. Contains: `string role`, `string input_text` -> `string output_text`. Used by LLM_service_node and its clients.
│       └── **FacialRecognition.srv** # Service definition for facial recognition requests (used between Pi and BeagleBone for verifying identity before dispensing).
└── (project root files)
    └── README.md                 # Documentation 

