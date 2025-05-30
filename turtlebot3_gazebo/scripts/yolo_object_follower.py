#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time

class YOLOObjectFollower(Node):
    def __init__(self):
        super().__init__('yolo_object_follower')
        
        # Initialize YOLO model
        self.model = YOLO('yolov8n.pt')
        
        # Target object class (person=0, chair=56, bottle=39, etc.)
        self.target_class = 0  # person - change this to follow different objects
        self.target_names = {0: 'person', 56: 'chair', 39: 'bottle', 67: 'cell phone'}
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers and Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detection_pub = self.create_publisher(Image, '/detection_image', 10)
        
        # Control parameters
        self.image_width = 640
        self.image_height = 480
        self.center_tolerance = 80  # pixels - increased for more stable centering
        self.min_area = 3000  # minimum bounding box area to follow - reduced
        
        # State management
        self.robot_state = "SEARCHING"  # SEARCHING, FOLLOWING, APPROACHING
        self.last_detection_time = time.time()
        self.search_timeout = 2.0  # seconds without detection before searching
        self.target_lost_count = 0
        
        # PID-like control gains - adjusted for smoother movement
        self.angular_gain = 0.002  # Reduced for smoother turning
        self.linear_gain = 1.0     # Base speed multiplier
        self.max_linear_speed = 0.6   # Increased 3x from 0.22
        self.max_angular_speed = 2.0   # Match teleop angular speeds
        self.search_angular_speed = 0.5  # Faster search rotation
        
        # Target size control
        self.target_area_ratio = 0.12  # Desired object size in image (12% of image)
        self.approach_threshold = 0.08  # Stop approaching when object is this size
        self.too_close_threshold = 0.20  # Back away if object is larger than this
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.current_detection = None
        
        self.get_logger().info(f'YOLO Object Follower initialized. Following: {self.target_names.get(self.target_class, "unknown")}')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width = cv_image.shape[:2]
            
            # Run YOLO detection
            results = self.model(cv_image, verbose=False)
            
            # Process detections
            target_detected = False
            best_detection = None
            best_confidence = 0.0
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Get class, confidence, and bounding box
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        
                        if class_id == self.target_class and confidence > 0.4:  # lowered threshold
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            area = (x2 - x1) * (y2 - y1)
                            
                            # Check if this is the best detection
                            if area > self.min_area and confidence > best_confidence:
                                best_detection = {
                                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                                    'confidence': confidence,
                                    'area': area
                                }
                                best_confidence = confidence
                                target_detected = True
            
            # Update detection state
            if target_detected and best_detection:
                self.current_detection = best_detection
                self.last_detection_time = time.time()
                self.target_lost_count = 0
                if self.robot_state == "SEARCHING":
                    self.robot_state = "FOLLOWING"
                    self.get_logger().info("Target found! Switching to FOLLOWING mode")
            else:
                self.target_lost_count += 1
                if self.target_lost_count > 10:  # Lost target for multiple frames
                    self.current_detection = None
                    if self.robot_state != "SEARCHING":
                        self.robot_state = "SEARCHING"
                        self.get_logger().info("Target lost! Switching to SEARCHING mode")
            
            # Draw detections
            annotated_image = self.draw_all_detections(cv_image, results, best_detection)
            
            # Publish annotated image
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            self.detection_pub.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in image processing: {str(e)}')

    def control_loop(self):
        """Main control loop called by timer"""
        twist = Twist()
        
        if self.robot_state == "SEARCHING":
            # Rotate slowly to search for target
            twist.angular.z = self.search_angular_speed
            twist.linear.x = 0.0
            
        elif self.robot_state == "FOLLOWING" and self.current_detection:
            # Follow the detected target
            twist = self.calculate_following_velocity(self.current_detection)
            
        else:
            # Stop if no valid state
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)

    def calculate_following_velocity(self, detection):
        x1, y1, x2, y2 = detection['bbox']
        
        # Calculate center of bounding box
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        
        # Calculate image center
        image_center_x = self.image_width / 2
        
        # Calculate errors
        error_x = center_x - image_center_x
        
        # Calculate area ratio (for distance estimation)
        bbox_area = detection['area']
        area_ratio = bbox_area / (self.image_width * self.image_height)
        
        # Create twist message
        twist = Twist()
        
        # Angular velocity (turn towards object)
        if abs(error_x) > self.center_tolerance:
            twist.angular.z = -self.angular_gain * error_x
            twist.angular.z = max(min(twist.angular.z, self.max_angular_speed), -self.max_angular_speed)
        else:
            twist.angular.z = 0.0
        
        # Simplified linear velocity based on object size
        if area_ratio < self.approach_threshold:
            # Object too far, move forward at fixed speed
            twist.linear.x = 0.45  # Increased 3x from 0.15
            
        elif area_ratio > self.too_close_threshold:
            # Object too close, move backward
            twist.linear.x = -0.3  # Increased 3x from -0.1
            
        else:
            # Object at good distance, stop forward movement
            twist.linear.x = 0.0
        
        # Only move forward/backward if object is reasonably centered
        if abs(error_x) > self.center_tolerance * 1.5:
            twist.linear.x = twist.linear.x * 0.5  # Reduce speed when not centered
        
        self.get_logger().info(f'State: {self.robot_state}, Center: ({center_x:.1f}, {center_y:.1f}), '
                              f'Error_x: {error_x:.1f}, Area_ratio: {area_ratio:.3f}, '
                              f'Linear: {twist.linear.x:.2f}, Angular: {twist.angular.z:.2f}')
        
        return twist

    def draw_all_detections(self, cv_image, results, target_detection):
        annotated_image = cv_image.copy()
        
        # Draw all detections for visualization
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    
                    if confidence > 0.3:  # Show all detections with decent confidence
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        class_name = self.model.names[class_id]
                        
                        color = (0, 255, 0) if class_id == self.target_class else (0, 0, 255)
                        thickness = 3 if class_id == self.target_class else 2
                        
                        cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
                        cv2.putText(annotated_image, f'{class_name}: {confidence:.2f}', 
                                  (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw target detection with special markers
        if target_detection:
            self.draw_target_detection(annotated_image, target_detection)
        
        # Draw robot state
        cv2.putText(annotated_image, f'STATE: {self.robot_state}', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Draw crosshair at image center
        cv2.line(annotated_image, (self.image_width//2 - 30, self.image_height//2), 
                (self.image_width//2 + 30, self.image_height//2), (255, 0, 0), 2)
        cv2.line(annotated_image, (self.image_width//2, self.image_height//2 - 30), 
                (self.image_width//2, self.image_height//2 + 30), (255, 0, 0), 2)
        
        return annotated_image

    def draw_target_detection(self, image, detection):
        x1, y1, x2, y2 = detection['bbox']
        confidence = detection['confidence']
        area_ratio = detection['area'] / (self.image_width * self.image_height)
        
        # Draw bounding box
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        
        # Draw center point
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        cv2.circle(image, (center_x, center_y), 8, (0, 255, 0), -1)
        
        # Draw target indicator
        target_name = self.target_names.get(self.target_class, "target")
        cv2.putText(image, f'FOLLOWING {target_name.upper()}: {confidence:.2f}', 
                   (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw size indicator
        size_status = "GOOD DISTANCE"
        if area_ratio < self.approach_threshold:
            size_status = "TOO FAR"
        elif area_ratio > self.too_close_threshold:
            size_status = "TOO CLOSE"
        
        cv2.putText(image, f'SIZE: {size_status} ({area_ratio:.3f})', 
                   (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOObjectFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
