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

### code to test
if __name__ == "__main__":
    tracker = FaceDistanceTracker()

    try:
        while True:
            # Capture frame from Picamera2
            frame = tracker.picam2.capture_array()

            # Run face recognition
            tracker.process_frame(frame)

            # Get distance info
            distances = tracker.get_face_distance(frame)

            # Draw bounding boxes and info
            for (top, right, bottom, left), (name, distance_m, distance_state) in zip(tracker.face_locations, distances):
                # Scale back up for display
                top *= tracker.cv_scaler
                right *= tracker.cv_scaler
                bottom *= tracker.cv_scaler
                left *= tracker.cv_scaler

                # Draw rectangle around face
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

                # Text overlay with name and distance
                text = f"{name} {distance_state} {distance_m:.2f}m"
                cv2.putText(frame, text, (left, top - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Print in terminal
                print(f"Name: {name}, Distance: {distance_m:.2f} m, State: {distance_state}")

            # Show frame
            cv2.imshow("Face Distance Tracker", frame)

            # Quit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        tracker.cleanup()





