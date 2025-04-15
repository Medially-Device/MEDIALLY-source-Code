import cv2
import time
import numpy as np
from threading import Thread
import mediapipe as mp
import keras

from deepface import DeepFace
import tempfile
import os



video_driver_id = 3

class VideoStream:
    """Handles video streaming from the webcam."""
    def __init__(self, resolution=(640, 480), framerate=36):
        self.stream = cv2.VideoCapture(video_driver_id)
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.stream.set(3, resolution[0])
        self.stream.set(4, resolution[1])
        self.stream.set(cv2.CAP_PROP_FPS, framerate)
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False
        self.timestamp = time.time()

    def start(self):
        """Starts the thread that reads frames from the video stream."""
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        """Continuously updates the frame from the video stream."""
        while True:
            if self.stopped:
                self.stream.release()
                return
            self.grabbed, self.frame = self.stream.read()
            self.timestamp = time.time()  # Update the timestamp

    def read(self):
        """Returns the most recent frame."""
        return self.frame

    def stop(self):
        """Stops the video stream and closes resources."""
        self.stopped = True


def GetSegment(resW, resH, desired_video_length, segment_number, save=False, videostream=None):
    frames_per_segment = 36 * desired_video_length

    if save:
        filename = f"video_segment_{segment_number}.mp4"
        out = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'mp4v'), 36, (resW, resH))
        print(f"Saving new video segment: {filename}")

    raw_video = []
    for num_frames in range(frames_per_segment):
        frame = videostream.read()  # Use the passed videostream object
        if num_frames % 3 == 0:
            raw_video.append(frame)

        if save:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            cv2.putText(frame, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            out.write(frame)

    if save:
        out.release()
    
    return raw_video

def verify_face(frame):
    """Performs face verification using DeepFace."""
    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".jpg")
    cv2.imwrite(temp_file.name, frame)
    
    try:
        verification_result = DeepFace.verify(temp_file.name, 'User1.jpg', detector_backend='OpenFace')
        os.unlink(temp_file.name)  # Cleanup temp file
        return verification_result["verified"]  # Returns True or False
    except Exception as e:
        print("Error in face verification:", e)
        os.unlink(temp_file.name)
        return False


def Processing_and_Prediction(frames, model, pill_taken_status, person_found_status, face_recognized):

    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(model_complexity=0)

    landmarks_list = []
    print("Getting landmarks...")
    
    for frame in frames:

        if pill_taken_status["pill_taken"]:
            print("Pill Taken, stopping early...")
            break
            
        frame_resized = cv2.resize(frame, (320, 240))
        frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)

        if not face_recognized["face_recognized"]:
            face_recognized["face_recognized"] = DeepFace.verify(frame_resized, 'User1.jpg', detector_backend='OpenFace')
            
        
        results = pose.process(frame_rgb)
        
        if results.pose_landmarks:
            person_found_status["person_found"] = True
            landmarks = []
            for lm in results.pose_landmarks.landmark:
                landmarks.append([lm.x, lm.y, lm.z, lm.visibility])
            landmarks_list.append(landmarks)
        else:
            # No landmarks found in this frame
            continue
        
    landmarks = np.array(landmarks_list)
    print(f"Landmarks shape: {landmarks.shape}")
    if len(landmarks_list) == 0:
        print("No pose landmarks detected in segment.")
        return
    

    # Padding and reshaping logic
    if landmarks.ndim == 3:
        landmarks[:, :, :3] -= np.mean(landmarks[:, :, :3], axis=1, keepdims=True)
        landmarks[:, :, :3] /= np.std(landmarks[:, :, :3], axis=1, keepdims=True)

    padded_shape = (200, 33, 4)  # For landmarks
    padded_landmarks = np.full(padded_shape, 0, dtype=landmarks.dtype)
    length_to_copy = min(landmarks.shape[0], 200)
    padded_landmarks[:length_to_copy, ...] = landmarks[:length_to_copy, ...]

    reshaped_landmarks = padded_landmarks.reshape(1, 200, 33 * 4)
    print(f"Reshaped landmarks shape: {reshaped_landmarks.shape}")

    # Make prediction
    try:
        y_pred = (model.predict(reshaped_landmarks) > 0.5).astype(int)
        print("Prediction result: ", y_pred)
        if y_pred[0][0] == 1:
            pill_taken_status["pill_taken"] = True
    except Exception as e:
        print("Error during prediction: ", e)


def run_video_capture(resolution, desired_video_length, model, num_segments=4):
    resW, resH = map(int, resolution.split('x'))
    
    # Initialize video stream
    videostream = VideoStream(resolution=(resW, resH), framerate=18).start()
    print("\nStarting to Record Video")
    time.sleep(1)

    frames = {}
    processing_thread = {}

    segment_number = 1
    pill_taken_status = {"pill_taken": False}
    person_found_status = {"person_found": False}
    face_recognized = {"face_recognized": False}

    while segment_number <= num_segments:
        print(f"\nRecording Video: {segment_number}")
        # Pass the videostream to GetSegment
        frames[segment_number] = GetSegment(resW, resH, desired_video_length, segment_number, save=True, videostream=videostream)
        print("Segment captured, now processing...")

        # Start processing frames in a separate thread
        processing_thread[segment_number] = Thread(target=Processing_and_Prediction, args=(frames[segment_number], model, pill_taken_status, person_found_status, face_recognized))
        processing_thread[segment_number].start()

        if pill_taken_status["pill_taken"]:
            print("Pill Taken, stopping early...")
            break

        segment_number += 1
        save = False

    # Wait for all threads to finish
    for thread in processing_thread.values():
        thread.join()

    # Release resources
    cv2.destroyAllWindows()
    videostream.stop()

    return person_found_status["person_found"], pill_taken_status["pill_taken"], face_recognized["face_recognized"]



if __name__ == "__main__":

    resolution = '640x480'
    desired_video_length = 10  # Desired video segment length in seconds
    modelpath = 'pill_classifier.keras'  # Path to your trained model

    # Load model beforehand (only once)
    model = keras.saving.load_model(modelpath)

    # Run the video capture and processing
    print("\nStarting the main video capture and processing...")
    person_found, pill_taken, face_recognized = run_video_capture(resolution, desired_video_length, model)

    print("Pill Taken: ", pill_taken)
    print("Person Found: ", person_found)
    print("Face Recognized: ", face_recognized)