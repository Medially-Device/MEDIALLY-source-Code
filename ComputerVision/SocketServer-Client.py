import socket
import sys
import cv2
import time
import numpy as np
import queue
from threading import Thread
import mediapipe as mp
import keras
import argparse

from Record_Predict import run_video_capture, verify_face, VideoStream, Processing_and_Prediction, GetSegment

# Set the parameters as needed (e.g., resolution, video length, model path)
resolution = '640x480'  # Example resolution
desired_video_length = 10  # Example video length in seconds
modelpath = 'pill_classifier.keras' 

#Initialize Model
model = keras.saving.load_model(modelpath)



# Server details
HOST = "192.168.2.135"  # Replace with the host's IP address
PORT = 12345  # Must match the server's port

def start_client(model):
    """Starts the client, listens for commands, and executes the specified function when triggered."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((HOST, PORT))
        print(f"Connected to server at {HOST}:{PORT}")
        
        while True:
            data = client_socket.recv(1024).decode().strip()
            if not data:
                continue
            
            print(f"Received: {data}")
            
            if data == "START":
                print("Starting video capture...")
                try:
                    
                    # Call the run_video_capture function directly
                    person_found, pill_taken, face_recognized=run_video_capture(resolution, desired_video_length, model)

                    result_message = f"Person Found: {person_found}, Pill_Taken: {pill_taken}, Face_Recognized: {face_recognized}\n, "
                    client_socket.sendall(result_message.encode())
                    print(result_message)

                except Exception as e:
                    output = f"Error executing video capture: {e}"
                    print(output)
                    client_socket.sendall(output.encode())
                    continue
                
                # If everything goes well, send success message
                success_msg = "Video capture completed successfully. Closing Connection."
                client_socket.sendall(success_msg.encode())
                print(success_msg)

                # Close the connection after completion
                client_socket.close()
                print("Connection closed.")
                sys.exit()  # Gracefully exit the script

if __name__ == "__main__":
    start_client(model)