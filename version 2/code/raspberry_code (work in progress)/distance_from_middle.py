import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import pickle

class FaceTracker:
    def __init__(self, encoding_file="encodings.pickle", tolerance=0.1, cv_scaler=4):
        # Load known encodings
        print("[INFO] loading encodings...")
        with open(encoding_file, "rb") as f:
            data = pickle.loads(f.read())
        self.known_face_encodings = data["encodings"]
        self.known_face_names = data["names"]

        # Camera setup
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (1920, 1080)}
        ))
        self.picam2.start()

        # Parameters
        self.tolerance = tolerance
        self.cv_scaler = cv_scaler

        # State
        self.face_locations = []
        self.face_names = []
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0

    def process_frame(self, frame):
        # Downscale for faster detection
        resized_frame = cv2.resize(frame, (0, 0), fx=(1/self.cv_scaler), fy=(1/self.cv_scaler))
        rgb_resized_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)

        self.face_locations = face_recognition.face_locations(rgb_resized_frame)
        face_encodings = face_recognition.face_encodings(rgb_resized_frame, self.face_locations, model='large')

        self.face_names = []
        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = "Unknown"
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = self.known_face_names[best_match_index]
            self.face_names.append(name)

        return frame

    def get_face_distance(self, frame):
        """
        Returns a list of tuples: (name, position, distance_px, distance_norm)
        - position: LEFT / RIGHT / CENTER
        - distance_px: pixel distance from vertical center
        - distance_norm: normalized deviation [0..0.5]
        """
        results = []
        frame_height, frame_width = frame.shape[:2]
        middle_x = frame_width // 2

        if len(self.face_locations) == 0:
            return results

        for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):
            # Scale back up
            top *= self.cv_scaler
            right *= self.cv_scaler
            bottom *= self.cv_scaler
            left *= self.cv_scaler

            face_center_x = (left + right) / 2.0
            distance_px = abs(face_center_x - middle_x)
            normalized_x = face_center_x / frame_width
            x_deviation = 0.5 - normalized_x
            distance_norm = abs(x_deviation)

            # Classify position
            if abs(x_deviation) < self.tolerance:
                position = "CENTER"
            elif x_deviation > 0:
                position = "LEFT"
            else:
                position = "RIGHT"

            results.append((name, position, distance_px, distance_norm))

        return results

    def calculate_fps(self):
        self.frame_count += 1
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 1:
            self.fps = self.frame_count / elapsed_time
            self.frame_count = 0
            self.start_time = time.time()
        return self.fps

        finally:
            cv2.destroyAllWindows()
            self.picam2.stop()

"""
import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import pickle

# ----- SETTINGS -----
tolerance = 0.1  # LEFT / RIGHT "center" window (10% of frame width)

# ----- LOAD FACE ENCODINGS -----
print("[INFO] loading encodings...")
with open("encodings.pickle", "rb") as f:
    data = pickle.loads(f.read())
known_face_encodings = data["encodings"]
known_face_names = data["names"]

# ----- CAMERA SETUP -----
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"format": 'XRGB8888', "size": (1920, 1080)}
))
picam2.start()

# ----- TRACKING STATE -----
cv_scaler = 4  # must be an integer scale factor

face_locations = []
face_encodings = []
face_names = []
frame_count = 0
start_time = time.time()
fps = 0

def process_frame(frame):
    global face_locations, face_encodings, face_names
    
    # Downscale for faster face detection
    resized_frame = cv2.resize(frame, (0, 0), fx=(1/cv_scaler), fy=(1/cv_scaler))
    rgb_resized_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
    
    face_locations = face_recognition.face_locations(rgb_resized_frame)
    face_encodings = face_recognition.face_encodings(rgb_resized_frame, face_locations, model='large')
    
    face_names = []
    for face_encoding in face_encodings:
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
        name = "Unknown"
        
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = known_face_names[best_match_index]
        face_names.append(name)
    
    return frame

def draw_results(frame):
    global face_locations, face_names, tolerance, cv_scaler

    frame_height, frame_width = frame.shape[:2]
    middle_x = frame_width // 2

    # Draw middle vertical line (center y-axis)
    cv2.line(frame, (middle_x, 0), (middle_x, frame_height), (0, 255, 255), 2)

    # If no faces, nothing to do
    if len(face_locations) == 0:
        return frame

    # Loop over all faces
    for (top, right, bottom, left), name in zip(face_locations, face_names):
        # Scale locations back up
        top    *= cv_scaler
        right  *= cv_scaler
        bottom *= cv_scaler
        left   *= cv_scaler
        
        # Draw face box
        cv2.rectangle(frame, (left, top), (right, bottom), (244, 42, 3), 3)
        
        # ----- LEFT / RIGHT / CENTER + DISTANCE FROM CENTER AXIS -----
        face_center_x = (left + right) / 2.0

        # distance from vertical center axis (in pixels)
        distance_px = abs(face_center_x - middle_x)

        # normalized [0..0.5] (0 = center, 0.5 = at extreme edge)
        normalized_x = face_center_x / frame_width
        x_deviation = 0.5 - normalized_x        # signed: + left, - right
        distance_norm = abs(x_deviation)        # unsigned

        # classify position with tolerance
        if abs(x_deviation) < tolerance:
            position = "CENTER"
        elif x_deviation > 0:
            position = "LEFT"
        else:
            position = "RIGHT"

        # print info in terminal
        print(
            f"{name}: pos={position}, "
            f"dist_from_center_px={distance_px:.1f}, "
            f"dist_norm={distance_norm:.3f}"
        )

        # ----- ON-SCREEN LABEL: name + pixel distance from center -----
        label = f"{name}  dx={distance_px:.1f}px"
        cv2.rectangle(frame, (left - 3, top - 35), (right + 3, top), (244, 42, 3), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, label, (left + 6, top - 8), font, 0.8, (255, 255, 255), 1)
    
    return frame

def calculate_fps():
    global frame_count, start_time, fps
    frame_count += 1
    elapsed_time = time.time() - start_time
    if elapsed_time > 1:
        fps = frame_count / elapsed_time
        frame_count = 0
        start_time = time.time()
    return fps

try:
    while True:
        frame = picam2.capture_array()
        processed_frame = process_frame(frame)
        display_frame = draw_results(processed_frame)
        current_fps = calculate_fps()
        
        cv2.putText(display_frame, f"FPS: {current_fps:.1f}",
                    (display_frame.shape[1] - 150, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('Video', display_frame)
        
        if cv2.waitKey(1) == ord("q"):
            break

finally:
    cv2.destroyAllWindows()
    picam2.stop()
"""