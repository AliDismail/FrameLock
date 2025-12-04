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

    def close(self):
        cv2.destroyAllWindows()
        oself.picam2.stop()
"""  
### code to test the function
if __name__ == "__main__":
    tracker = FaceTracker()

    try:
        while True:
            # Capture frame from Picamera2
            frame = tracker.picam2.capture_array()

            # Run face recognition
            tracker.process_frame(frame)

            # Get distance info
            distances = tracker.get_face_distance(frame)

            # Draw bounding boxes and info
            for (top, right, bottom, left), (name, position, dist_px, dist_norm) in zip(tracker.face_locations, distances):
                # Scale back up for display
                top *= tracker.cv_scaler
                right *= tracker.cv_scaler
                bottom *= tracker.cv_scaler
                left *= tracker.cv_scaler

                # Draw rectangle
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

                # Info text
                text = f"{name} {position} norm:{dist_norm:.3f}"
                cv2.putText(frame, text, (left, top - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Also print in terminal
                print(f"Name: {name}, Pos: {position}, px: {dist_px:.1f}, norm: {dist_norm:.3f}")

            # Show FPS
            fps = tracker.calculate_fps()
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Display output window
            cv2.imshow("Face Tracker", frame)

            # Quit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        tracker.close()
"""


