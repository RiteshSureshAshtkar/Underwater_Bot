#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import os
from inference import get_model
import supervision as sv

class RoboflowNode(Node):
    def __init__(self):
        super().__init__('roboflow_inference_node')
        
        # ROS2 subscribers
        self.image_subscription = self.create_subscription(
            Image,
            '/tethys/camera',
            self.image_callback,
            10)
        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/tethys/camera_info',
            self.camera_info_callback,
            10)
        
        # ROS2 publishers
        self.image_pub = self.create_publisher(Image, '/roboflow_inference/image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/roboflow_inference/camera_info', 10)
        self.midpoints_pub = self.create_publisher(Point, '/roboflow_inference/gate_center', 10)
        
        self.bridge = CvBridge()
        
        # Add a timer to check if we're receiving messages
        self.timer = self.create_timer(5.0, self.check_status)
        self.message_count = 0
        self.camera_info_count = 0
        
        # Roboflow setup
        try:
            os.environ["ROBOFLOW_API_KEY"] = "Jexa9ERAEPPPNX8Vlpgl"
            os.environ["ROBOFLOW_OFFLINE"] = "1"
            self.get_logger().info("Loading Roboflow model...")
            self.model = get_model(model_id="gates-detection-msoof/1")
            self.get_logger().info("Model loaded successfully!")
            
            self.box_annotator = sv.BoxAnnotator()
            self.label_annotator = sv.LabelAnnotator()
            
        except Exception as e:
            self.get_logger().error(f"Failed to load Roboflow model: {e}")
            self.model = None
        
        self.get_logger().info("Roboflow Inference Node started, waiting for images...")
        self.get_logger().info(f"Subscribed to: {self.image_subscription.topic_name}")
        self.get_logger().info(f"Subscribed to: {self.camera_info_subscription.topic_name}")
    
    def check_status(self):
        """Timer callback to check if we're receiving messages"""
        self.get_logger().info(f"Status check: Received {self.message_count} images, {self.camera_info_count} camera info messages")
        
        # Check if topics exist
        topic_names = self.get_topic_names_and_types()
        camera_topics = [name for name, _ in topic_names if 'camera' in name or 'tethys' in name]
        self.get_logger().info(f"Available camera/tethys topics: {camera_topics}")

    def camera_info_callback(self, msg):
        """Callback for camera info messages - simply republish them"""
        self.camera_info_count += 1
        
        # Create a new message with updated frame_id and timestamp
        camera_info_msg = CameraInfo()
        camera_info_msg.header=msg.header  # or keep original: msg.header.frame_id
        
        # Copy all camera calibration data
        camera_info_msg.height = msg.height
        camera_info_msg.width = msg.width
        camera_info_msg.distortion_model = msg.distortion_model
        camera_info_msg.d = msg.d
        camera_info_msg.k = msg.k
        camera_info_msg.r = msg.r
        camera_info_msg.p = msg.p
        camera_info_msg.binning_x = msg.binning_x
        camera_info_msg.binning_y = msg.binning_y
        camera_info_msg.roi = msg.roi
        
        # Publish the camera info
        self.camera_info_pub.publish(camera_info_msg)
        
        if self.camera_info_count % 10 == 0:  # Log every 10th message to avoid spam
            self.get_logger().info(f"Republished camera info #{self.camera_info_count}")

    def image_callback(self, msg):
        self.message_count += 1
        self.get_logger().info(f"Received image #{self.message_count}: {msg.width}x{msg.height}")
        
        
        if self.model is None:
            self.get_logger().error("Model not loaded, skipping inference")
            return
        
        try:
            # Convert ROS Image -> OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(f"Converted to OpenCV: {frame.shape}")
            cv2.imshow("hi",frame)
            cv2.waitKey(1)
            
            # Run inference
            self.get_logger().info("Running inference...")
            results = self.model.infer(frame)[0]
            detections = sv.Detections.from_inference(results)
            self.get_logger().info(f"Found {len(detections)} detections")
            
            # Annotate
            annotated = self.box_annotator.annotate(scene=frame, detections=detections)
            annotated = self.label_annotator.annotate(scene=annotated, detections=detections)
            cv2.imshow("hi1",annotated)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header=msg.header
            self.image_pub.publish(annotated_msg)
            
            # Process detections
            for xyxy, class_id, conf in zip(detections.xyxy, detections.class_id, detections.confidence):
                x_min, y_min, x_max, y_max = xyxy
                cx = int((x_min + x_max) / 2)
                cy = int((y_min + y_max) / 2)
                
                # Publish midpoint
                midpoint = Point()
                midpoint.x = float(cx)
                midpoint.y = float(cy)
                midpoint.z = float(conf)
                self.midpoints_pub.publish(midpoint)
                
                self.get_logger().info(f"Gate detected at midpoint: ({cx}, {cy}), conf={conf:.2f}")
                
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = RoboflowNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
