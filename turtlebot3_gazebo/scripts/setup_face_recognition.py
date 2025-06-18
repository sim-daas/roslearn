#!/usr/bin/env python3

import os
import cv2
import face_recognition
import numpy as np

def setup_known_faces():
    """Setup script to capture and save known faces"""
    faces_dir = os.path.expanduser("~/known_faces")
    if not os.path.exists(faces_dir):
        os.makedirs(faces_dir)
    
    print(f"Face Recognition Setup")
    print(f"Known faces directory: {faces_dir}")
    print("Press 'c' to capture a face, 'q' to quit")
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    face_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Find faces in the frame
        rgb_frame = frame[:, :, ::-1]
        face_locations = face_recognition.face_locations(rgb_frame)
        
        # Draw rectangles around faces
        for (top, right, bottom, left) in face_locations:
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        
        cv2.imshow('Face Recognition Setup - Press "c" to capture, "q" to quit', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c') and face_locations:
            # Save the first detected face
            name = input("Enter name for this face: ")
            if name:
                filename = os.path.join(faces_dir, f"{name}.jpg")
                cv2.imwrite(filename, frame)
                print(f"Saved face as {filename}")
                face_count += 1
        elif key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print(f"Setup complete. Saved {face_count} faces.")

if __name__ == '__main__':
    setup_known_faces()
