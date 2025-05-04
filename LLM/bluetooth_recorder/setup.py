from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bluetooth_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Optionally, install srv files if needed
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='medially',
    maintainer_email='medially@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
'Intent_node = bluetooth_recorder.bio_bert_intent_node:main',
'llm_client_sentence_node = bluetooth_recorder.llm_sentence_fixer_client:main',
'Chat_summary_node = bluetooth_recorder.Chat_summary_node:main',
'Medication_vectorization_node_v2 = bluetooth_recorder.vectorization_database_medication_v2:main',
'medication_node = bluetooth_recorder.Medication_client:main',
'Patient_history_node_v2 = bluetooth_recorder.Patient_history_client_v2:main',
'Reminder_node = bluetooth_recorder.Reminder_check:main',
'Dispenser_node = bluetooth_recorder.Medication_dispense:main',
'Emergency_node = bluetooth_recorder.Emergency_question_client:main',
'Firebase_node = bluetooth_recorder.Firebase_upload:main',
'text_speech_node = bluetooth_recorder.text_speech_node:main',
'text_speech_node_v2 = bluetooth_recorder.text_speech_v2:main',
'Recording_node_v2 = bluetooth_recorder.recording_v2:main',
'speech_text_node_v2 = bluetooth_recorder.speech_text_v2:main',
'Dispenser_node_v2 = bluetooth_recorder.Medication_dispense_v2:main',
'BeagleY_Facial_node = bluetooth_recorder.BeagleY_Facial:main',
'Irrelevant_Node = bluetooth_recorder.Irrelevant_questions:main',
'llm_service_sentence_node_v6 = bluetooth_recorder.LLM_service_node_v6:main',
        ],
    },
)
