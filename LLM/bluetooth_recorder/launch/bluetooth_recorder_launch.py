#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Phase 1: LLM_service_node_v5 (starts immediately)
    llm_service = Node(
        package='bluetooth_recorder',
        executable='llm_service_sentence_node_v6',
        name='llm_service_sentence_node_v6',
        output='screen'
    )

    # Phase 2: Medication vectorization node (starts after 5 seconds)
    medication_vectorization = Node(
        package='bluetooth_recorder',
        executable='Medication_vectorization_node_v2',
        name='Medication_vectorization_node_v2',
        output='screen'
    )

    # Phase 3: Patient history vectorization node (starts after 10 seconds)
    patient_vectorization = Node(
        package='bluetooth_recorder',
        executable='Patient_vectorization_node',
        name='Patient_vectorization_node',
        output='screen'
    )

    # Phase 4: Chat summary node (starts after 15 seconds)
    chat_summary = Node(
        package='bluetooth_recorder',
        executable='Chat_summary_node',
        name='Chat_summary_node',
        output='screen'
    )

    # Phase 5: Speech to text node (starts after 20 seconds)
    speech_to_text = Node(
        package='bluetooth_recorder',
        executable='speech_text_node_v2',
        name='speech_text_node_v2',
        output='screen'
    )

    # Phase 6: Bio bert intent node (starts after 25 seconds)
    bio_bert_intent = Node(
        package='bluetooth_recorder',
        executable='Intent_node',
        name='Intent_node',
        output='screen'
    )

    # Phase 7: Other nodes launched in pairs of two

    # Pair 1:
    llm_client_sentence = Node(
        package='bluetooth_recorder',
        executable='llm_client_sentence_node',
        name='llm_client_sentence_node',
        output='screen'
    )
    medication_client = Node(
        package='bluetooth_recorder',
        executable='medication_node',
        name='medication_node',
        output='screen'
    )
    Irrelevant_Node = Node(
        package='bluetooth_recorder',
        executable='Irrelevant_Node',
        name='Irrelevant_Node',
        output='screen'
    )

    # Pair 2:
    patient_history_client = Node(
        package='bluetooth_recorder',
        executable='Patient_history_node_v2',
        name='Patient_history_node_v2',
        output='screen'
    )
    text_speech_v2 = Node(
        package='bluetooth_recorder',
        executable='text_speech_node_v2',
        name='text_speech_node_v2',
        output='screen'
    )

    # Pair 3:
    emergency_node = Node(
        package='bluetooth_recorder',
        executable='Emergency_node',
        name='Emergency_node',
        output='screen'
    )
    firebase_upload = Node(
        package='bluetooth_recorder',
        executable='Firebase_node',
        name='Firebase_node',
        output='screen'
    )

    # Pair 4 (Last Pair: Reminder_check and Medication_Release)
    reminder_check = Node(
        package='bluetooth_recorder',
        executable='Reminder_node',
        name='Reminder_node',
        output='screen'
    )
    medication_release = Node(
        package='bluetooth_recorder',
        executable='Dispenser_node_v2',
        name='Dispenser_node_v2',
        output='screen'
    )

    beagleY_Facial = Node(
        package='bluetooth_recorder',
        executable='BeagleY_Facial_node',
        name='BeagleY_Facial_node',
        output='screen'
    )

    # Phase 8: bluetooth_recorder_node (starts after all other nodes)
    bluetooth_recorder_node = Node(
        package='bluetooth_recorder',
        executable='Recording_node_v2',
        name='Recording_node_v2',
        output='screen'
    )

    ld = LaunchDescription()

    # Phase 1
    ld.add_action(llm_service)
    # Phase 2
    ld.add_action(TimerAction(period=20.0, actions=[medication_vectorization]))
    # Phase 3
    ld.add_action(TimerAction(period=40.0, actions=[patient_vectorization]))
    # Phase 4
    ld.add_action(TimerAction(period=60.0, actions=[chat_summary]))
    # Phase 5
    ld.add_action(TimerAction(period=80.0, actions=[speech_to_text]))
    # Phase 6
    ld.add_action(TimerAction(period=100.0, actions=[bio_bert_intent]))

    # Phase 7: Launch other nodes in pairs
    ld.add_action(TimerAction(period=120.0, actions=[llm_client_sentence, medication_client, Irrelevant_Node]))  # Pair 1
    ld.add_action(TimerAction(period=140.0, actions=[patient_history_client, text_speech_v2]))    # Pair 2
    ld.add_action(TimerAction(period=160.0, actions=[emergency_node, firebase_upload]))           # Pair 3
    ld.add_action(TimerAction(period=180.0, actions=[reminder_check, medication_release]))         # Pair 4

    ld.add_action(TimerAction(period=200.0, actions=[beagleY_Facial]))
    # Phase 8: Launch bluetooth_recorder_node last
    ld.add_action(TimerAction(period=220.0, actions=[bluetooth_recorder_node]))

    return ld

if __name__ == '__main__':
    generate_launch_description()
