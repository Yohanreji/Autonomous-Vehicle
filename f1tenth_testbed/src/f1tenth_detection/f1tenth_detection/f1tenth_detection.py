#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from std_msgs.msg import Bool

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('f1tenth_detection')

        # Load YOLO model
        self.model = YOLO("/home/f1tenth/F1tenth_Project/src/f1tenth_detection/models/best.pt")
        self.bridge = CvBridge()

        # Publishers for detected objects
        self.zebra_crossing_pub = self.create_publisher(Bool, '/f1tenth_detection/zebra_crossing_detected', 10)
        self.stop_sign_pub = self.create_publisher(Bool, '/f1tenth_detection/stop_sign_detected', 10)
        self.traffic_light_pub = self.create_publisher(Bool, '/f1tenth_detection/traffic_light_detected', 10)
        self.person_pub = self.create_publisher(Bool, '/f1tenth_detection/person_detected', 10)

        # Subscription to ZED camera topic
        self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',  # Adjust the topic if needed
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

           
            #Confidence thresholds for objects
            stop_sign_threshold = 0.6
            zebra_crossing_threshold = 0.1
            person_threshold = 0.80
            
            # Run YOLOv8 inference
            results = self.model.predict(cv_image)

            # Initialize detection flags
            zebra_detected = False
            stop_sign_detected = False
            person_detected = False
            traffic_light_detected = False

            # Check for detections
            for result in results[0].boxes:
                class_id = int(result.cls)
                confidence = result.conf

                # Apply specific thresholds for each class
                if class_id == 11:  # Stop sign
                    if confidence >= stop_sign_threshold:
                        stop_sign_detected = True
                elif class_id == 0:  # Person
                    if confidence >= person_threshold:
                        person_detected = True
                elif class_id == 80:  # Zebra crossing 
                    if confidence >= zebra_crossing_threshold:
                        zebra_detected = True

            # Publish detection results
            self.zebra_crossing_pub.publish(Bool(data=zebra_detected))
            self.stop_sign_pub.publish(Bool(data=stop_sign_detected))
            self.person_pub.publish(Bool(data=person_detected))
            self.traffic_light_pub.publish(Bool(data=traffic_light_detected))

            # Annotate the image with bounding boxes
            annotated_image = results[0].plot() if len(results) > 0 else cv_image

            # Display the images in OpenCV windows
            cv2.imshow("YOLOv8 Detection", annotated_image)
            key = cv2.waitKey(1) & 0xFF  # Ensure OpenCV updates properly

            # Save the image as a fallback in case the UI doesn't open
            cv2.imwrite("/tmp/detection_output.jpg", annotated_image)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
