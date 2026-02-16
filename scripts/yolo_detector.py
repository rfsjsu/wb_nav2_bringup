#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')  
        
        # Parameters
        self.declare_parameter('model', 'yolov8l-cls.pt') # Can change the model to (yolov8n, yolov8m, yolov8l)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.6)
        
        model_name = self.get_parameter('model').value
        device = self.get_parameter('device').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        
        # Initialize YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_name}')
        self.model = YOLO(model_name)
     
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe Turtlebot camera
        self.image_sub = self.create_subscription(
            Image,
            '/rgbd_camera/image',
            self.image_callback,
            10
        )
        
        # Publisher for visualization
        self.vis_pub = self.create_publisher(
            Image,
            '/yolo/detections_image',
            10
        )
        
        self.frame_count = 0
        self.get_logger().info('YOLO Detector Node Started...')
        self.get_logger().info('Subscribed to: /rgbd_camera/image')
        self.get_logger().info('Publishing to: /yolo/detections_image')
    
    def image_callback(self, msg):
        try:
            self.frame_count += 1
            
            # Log every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Processing frame {self.frame_count}')
            
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.model(
                cv_image,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False
            )
            
            # Log detections to view in ternminal
            for result in results:
                if len(result.boxes) > 0:
                    for box in result.boxes:
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        class_name = self.model.names[cls_id]
                        self.get_logger().info(
                            f'Detected: {class_name} (confidence: {conf:.2f})'
                        )
            
            # Create annotated image
            annotated_image = results[0].plot()
            
            # Convert back to ROS Image message
            vis_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            vis_msg.header = msg.header
            
            # Publish visualization
            self.vis_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()