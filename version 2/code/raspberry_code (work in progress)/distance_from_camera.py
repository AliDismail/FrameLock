import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import pickle

class FaceDistanceTracker:
    def __init__(self,
                 encoding_file="encodings.pickle",
                 tolerance=0.1,
                 target_face_height_norm=0.25,
                 distance_tolerance_norm=0.03,
                 reference_distance_m=1.0,
                 cv_scaler=4):
        # Settings
        self.tolerance = tolerance
        self.TARGET_FACE_HEIGHT_NORM = target_face_height_norm
        self.DISTANCE_TOLERANCE_NORM = distance_tolerance_norm
        self.REFERENCE_DISTANCE_M = reference_distance_m
        self.cv_scaler = cv_scaler

        # Load encodings
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

        # State
        self.face_locations = []
        self.face_names = []

    def process_frame(self, frame):
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
        Returns a list of tuples: (name, distance_m, distance_state)
        - distance_m: approximate distance in meters
        - distance_state: FAR / CLOSE / OK
        """
        results = []
        frame_height, frame_width = frame.shape[:2]

        for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):
            # Scale back up
            top *= self.cv_scaler
            bottom *= self.cv_scaler

            # Face height
            face_height = bottom - top
            normalized_height = face_height / frame_height
            nh_safe = max(normalized_height, 1e-6)

            # Approximate distance
            distance_m = self.REFERENCE_DISTANCE_M * (self.TARGET_FACE_HEIGHT_NORM / nh_safe)

            # Distance state
            if nh_safe < (self.TARGET_FACE_HEIGHT_NORM - self.DISTANCE_TOLERANCE_NORM):
                distance_state = "FAR"
            elif nh_safe > (self.TARGET_FACE_HEIGHT_NORM + self.DISTANCE_TOLERANCE_NORM):
                distance_state = "CLOSE"
            else:
                distance_state = "OK"

            results.append((name, distance_m, distance_state))

        return results

    def cleanup(self):
        cv2.destroyAllWindows()
        self.picam2.stop()


"""
import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import pickle
import RPi.GPIO as GPIO   # GPIO for LEDs

# ----- SETTINGS -----
tolerance = 0.1             # LEFT / RIGHT "center" window (10% of frame width)

# Distance tuning:
# At this normalized face height, the person is at a "good" distance.
TARGET_FACE_HEIGHT_NORM = 0.25    # adjust based on what looks right
DISTANCE_TOLERANCE_NORM = 0.03    # band around target where no distance LED lights

# Rough distance calibration:
# If at TARGET_FACE_HEIGHT_NORM the person is, say, 1.0 meter away:
REFERENCE_DISTANCE_M = 1.0        # adjust after testing

# LED pins
LEFT_LED_PIN  = 17   # lights when face is LEFT
RIGHT_LED_PIN = 27   # lights when face is RIGHT
FAR_LED_PIN   = 22   # lights when face is FAR
NEAR_LED_PIN  = 23   # lights when face is CLOSE

# ----- GPIO SETUP -----
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_LED_PIN,  GPIO.OUT)
GPIO.setup(RIGHT_LED_PIN, GPIO.OUT)
GPIO.setup(FAR_LED_PIN,   GPIO.OUT)
GPIO.setup(NEAR_LED_PIN,  GPIO.OUT)

# start with all LEDs off
GPIO.output(LEFT_LED_PIN,  GPIO.LOW)
GPIO.output(RIGHT_LED_PIN, GPIO.LOW)
GPIO.output(FAR_LED_PIN,   GPIO.LOW)
GPIO.output(NEAR_LED_PIN,  GPIO.LOW)

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

    # Draw middle vertical line
    cv2.line(frame, (middle_x, 0), (middle_x, frame_height), (0, 255, 255), 2)

    # DEFAULT: turn all LEDs off at start of each frame
    GPIO.output(LEFT_LED_PIN,  GPIO.LOW)
    GPIO.output(RIGHT_LED_PIN, GPIO.LOW)
    GPIO.output(FAR_LED_PIN,   GPIO.LOW)
    GPIO.output(NEAR_LED_PIN,  GPIO.LOW)

    for (top, right, bottom, left), name in zip(face_locations, face_names):
        # Scale locations back up
        top    *= cv_scaler
        right  *= cv_scaler
        bottom *= cv_scaler
        left   *= cv_scaler
        
        # Draw face box
        cv2.rectangle(frame, (left, top), (right, bottom), (244, 42, 3), 3)
        
        # ----- LEFT / RIGHT / CENTER -----
        face_center_x = (left + right) / 2.0
        normalized_x = face_center_x / frame_width       # 0.0 (left edge) -> 1.0 (right edge)
        x_deviation = round(0.5 - normalized_x, 3)       # 0 means centered

        if abs(x_deviation) < tolerance:
            position = "CENTER"
        elif x_deviation > 0:
            position = "LEFT"
        else:
            position = "RIGHT"

        # ----- DISTANCE ESTIMATION -----
        face_height = bottom - top
        normalized_height = face_height / frame_height   # 0.0 (tiny) -> 1.0 (full height)

        # Avoid division by zero
        nh_safe = max(normalized_height, 1e-6)

        # Approximate distance using simple inverse scaling:
        # distance ≈ REFERENCE_DISTANCE_M * (TARGET_FACE_HEIGHT_NORM / normalized_height)
        distance_m = REFERENCE_DISTANCE_M * (TARGET_FACE_HEIGHT_NORM / nh_safe)

        # Distance LEDs with tolerance band
        # If within [TARGET_FACE_HEIGHT_NORM - tol, TARGET_FACE_HEIGHT_NORM + tol] → "good distance" → no LED.
        if nh_safe < (TARGET_FACE_HEIGHT_NORM - DISTANCE_TOLERANCE_NORM):
            distance_state = "FAR"
        elif nh_safe > (TARGET_FACE_HEIGHT_NORM + DISTANCE_TOLERANCE_NORM):
            distance_state = "CLOSE"
        else:
            distance_state = "OK"

        # Debug in terminal (no words on screen, only here)
        print(
            f"{name}: x_dev={x_deviation}, pos={position}, "
            f"face_h_norm={normalized_height:.3f}, dist_state={distance_state}, "
            f"approx_dist={distance_m:.2f} m"
        )

        # ----- LED CONTROL: LEFT / RIGHT -----
        if position == "LEFT":
            GPIO.output(LEFT_LED_PIN,  GPIO.HIGH)
            GPIO.output(RIGHT_LED_PIN, GPIO.LOW)
        elif position == "RIGHT":
            GPIO.output(LEFT_LED_PIN,  GPIO.LOW)
            GPIO.output(RIGHT_LED_PIN, GPIO.HIGH)
        else:  # CENTER
            GPIO.output(LEFT_LED_PIN,  GPIO.LOW)
            GPIO.output(RIGHT_LED_PIN, GPIO.LOW)

        # ----- LED CONTROL: FAR / CLOSE with tolerance -----
        if distance_state == "FAR":
            GPIO.output(FAR_LED_PIN,  GPIO.HIGH)
            GPIO.output(NEAR_LED_PIN, GPIO.LOW)
        elif distance_state == "CLOSE":
            GPIO.output(FAR_LED_PIN,  GPIO.LOW)
            GPIO.output(NEAR_LED_PIN, GPIO.HIGH)
        else:  # OK distance
            GPIO.output(FAR_LED_PIN,  GPIO.LOW)
            GPIO.output(NEAR_LED_PIN, GPIO.LOW)

        # ----- ON-SCREEN LABEL: ONLY name + distance -----
        label = f"{name} {distance_m:.2f} m"
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
    GPIO.cleanup()   # release all LED pins
"""
