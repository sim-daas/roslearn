#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
import cv2
import face_recognition
import numpy as np
import os
import threading
import time

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')
        
        # Subscriber for detection state
        self.detstate_sub = self.create_subscription(
            String, '/detstate', self.detstate_callback, 10)
        
        # Service client for lid control
        self.lid_control_client = self.create_client(SetBool, '/control_lid')
        
        # Face recognition setup
        self.known_face_encodings = []
        self.known_face_names = []
        self.load_known_faces()
        
        # Camera setup
        self.cap = None
        self.face_recognition_active = False
        self.recognition_thread = None
        self.thread_running = False
        
        # Recognition parameters
        self.face_locations = []
        self.face_encodings = []
        self.face_names = []
        self.process_this_frame = True
        
        # Lid control timing
        self.lid_open_time = None
        self.lid_open_duration = 10.0  # Keep lid open for 10 seconds
        self.lid_is_open = False
        
        self.get_logger().info('Face Recognition Node initialized')
        self.get_logger().info(f'Loaded {len(self.known_face_names)} known faces')

    def load_known_faces(self):
        """Load known faces from a directory"""
        # Create faces directory if it doesn't exist
        faces_dir = os.path.expanduser("~/known_faces")
        if not os.path.exists(faces_dir):
            os.makedirs(faces_dir)
            self.get_logger().warn(f'Created directory {faces_dir}. Please add known face images here.')
            return
        
        # Load face images
        for filename in os.listdir(faces_dir):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                image_path = os.path.join(faces_dir, filename)
                try:
                    # Load image and get face encoding
                    image = face_recognition.load_image_file(image_path)
                    face_encodings = face_recognition.face_encodings(image)
                    
                    if face_encodings:
                        # Use filename (without extension) as name
                        name = os.path.splitext(filename)[0]
                        self.known_face_encodings.append(face_encodings[0])
                        self.known_face_names.append(name)
                        self.get_logger().info(f'Loaded face: {name}')
                    else:
                        self.get_logger().warn(f'No face found in {filename}')
                        
                except Exception as e:
                    self.get_logger().error(f'Error loading {filename}: {str(e)}')

    def detstate_callback(self, msg):
        """Handle detection state messages"""
        if msg.data == "GOOD DISTANCE":
            if not self.face_recognition_active:
                self.start_face_recognition()
        else:
            if self.face_recognition_active:
                self.stop_face_recognition()

    def start_face_recognition(self):
        """Start face recognition process"""
        if self.thread_running:
            return
            
        self.get_logger().info('Starting face recognition...')
        self.face_recognition_active = True
        self.thread_running = True
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)  # Use laptop camera
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            self.face_recognition_active = False
            self.thread_running = False
            return
        
        # Set camera properties for better performance
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Start recognition thread
        self.recognition_thread = threading.Thread(target=self.face_recognition_loop)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()

    def stop_face_recognition(self):
        """Stop face recognition process"""
        if not self.thread_running:
            return
            
        self.get_logger().info('Stopping face recognition...')
        self.face_recognition_active = False
        self.thread_running = False
        
        # Close lid when stopping face recognition
        if self.lid_is_open:
            self.call_lid_control_service(False)
            self.lid_is_open = False
            self.lid_open_time = None
        
        if self.cap:
            self.cap.release()
            self.cap = None
        
        # Close OpenCV windows
        cv2.destroyAllWindows()

    def face_recognition_loop(self):
        """Main face recognition loop"""
        while self.thread_running and self.face_recognition_active:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # Create a copy for display
            display_frame = frame.copy()
            
            # Resize frame for faster processing and ensure correct data type
            small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
            # Convert BGR to RGB and ensure uint8 format
            rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
            rgb_small_frame = np.ascontiguousarray(rgb_small_frame, dtype=np.uint8)
            
            # Check if lid should be closed due to timeout
            current_time = time.time()
            if self.lid_is_open and self.lid_open_time and (current_time - self.lid_open_time) > self.lid_open_duration:
                self.get_logger().info('Lid timeout reached, closing lid')
                self.call_lid_control_service(False)
                self.lid_is_open = False
                self.lid_open_time = None
            
            # Only process every other frame to save time
            if self.process_this_frame:
                try:
                    # Find face locations and encodings
                    self.face_locations = face_recognition.face_locations(rgb_small_frame)
                    
                    # Only compute encodings if faces are found
                    if self.face_locations:
                        self.face_encodings = face_recognition.face_encodings(rgb_small_frame, self.face_locations)
                    else:
                        self.face_encodings = []
                    
                    self.face_names = []
                    known_face_detected = False
                    unknown_face_detected = False
                    
                    for face_encoding in self.face_encodings:
                        # Check if face matches known faces
                        matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                        name = "Unknown"
                        confidence = 0.0
                        
                        # Use the known face with the smallest distance
                        if self.known_face_encodings:
                            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                            best_match_index = np.argmin(face_distances)
                            if matches[best_match_index] and face_distances[best_match_index] < 0.6:  # Add distance threshold
                                name = self.known_face_names[best_match_index]
                                confidence = 1.0 - face_distances[best_match_index]
                                known_face_detected = True
                                self.get_logger().info(f'Recognized known face: {name} (confidence: {confidence:.2f})')
                            else:
                                unknown_face_detected = True
                                self.get_logger().info('Unknown face detected')
                        else:
                            unknown_face_detected = True
                        
                        self.face_names.append(name)
                    
                    # Handle lid control based on face detection
                    if unknown_face_detected:
                        # Immediately close lid if unknown face is detected
                        if self.lid_is_open:
                            self.get_logger().info('Unknown face detected, closing lid immediately')
                            self.call_lid_control_service(False)
                            self.lid_is_open = False
                            self.lid_open_time = None
                    elif known_face_detected:
                        # Open lid and reset timer for known face
                        if not self.lid_is_open:
                            self.get_logger().info('Known face detected, opening lid')
                            self.call_lid_control_service(True)
                            self.lid_is_open = True
                        # Reset or set the timer
                        self.lid_open_time = current_time
                        
                except Exception as e:
                    self.get_logger().error(f'Face recognition error: {str(e)}')
                    self.face_locations = []
                    self.face_names = []
            
            # Draw face rectangles and names on display frame
            for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):
                # Scale back up face locations since the frame we detected on was scaled to 1/4 size
                top *= 4
                right *= 4
                bottom *= 4
                left *= 4
                
                # Choose color based on recognition
                color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
                
                # Draw rectangle around face
                cv2.rectangle(display_frame, (left, top), (right, bottom), color, 2)
                
                # Draw label with name
                cv2.rectangle(display_frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(display_frame, name, (left + 6, bottom - 6), font, 0.6, (255, 255, 255), 1)
            
            # Add status text
            status_text = f"Face Recognition Active - Known faces: {len(self.known_face_names)}"
            cv2.putText(display_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show faces detected count
            faces_text = f"Faces detected: {len(self.face_locations)}"
            cv2.putText(display_frame, faces_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show lid status and timer
            lid_status = "OPEN" if self.lid_is_open else "CLOSED"
            cv2.putText(display_frame, f"Lid: {lid_status}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            if self.lid_is_open and self.lid_open_time:
                time_remaining = max(0, self.lid_open_duration - (current_time - self.lid_open_time))
                cv2.putText(display_frame, f"Time remaining: {time_remaining:.1f}s", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Show the frame
            cv2.imshow('Face Recognition - Press ESC to close', display_frame)
            
            # Check for ESC key to manually close
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC key
                self.stop_face_recognition()
                break
            
            self.process_this_frame = not self.process_this_frame
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.03)  # ~30 FPS

    def call_lid_control_service(self, open_lid):
        """Call the lid control service"""
        if not self.lid_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Lid control service not available')
            return
        
        request = SetBool.Request()
        request.data = open_lid
        
        future = self.lid_control_client.call_async(request)
        future.add_done_callback(self.lid_control_callback)

    def lid_control_callback(self, future):
        """Handle lid control service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Lid control service called successfully')
            else:
                self.get_logger().warn(f'Lid control service failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Lid control service call failed: {str(e)}')

    def destroy_node(self):
        """Clean up resources"""
        self.stop_face_recognition()
        cv2.destroy_all_windows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
