#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
import time

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # Detection result subscriptions
        self.create_subscription(Bool, '/f1tenth_detection/zebra_crossing_detected', self.zebra_crossing_callback, 10)
        self.create_subscription(Bool, '/f1tenth_detection/person_detected', self.person_callback, 10)
        self.create_subscription(Bool, '/f1tenth_detection/stop_sign_detected', self.stop_sign_callback, 10)

        # Publisher for driving commands
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/speed', 10)

        # Internal state variables
        self.zebra_crossing_detected = False
        self.person_detected = False
        self.stop_sign_detected = False

        # Flags for action control
        self.is_stopped = False
        self.stop_timer_start = None
        self.cooldown_active = False
        self.cooldown_start = None

        # State variable
        self.curr_state = 0  # Default state: Normal (0)

        # Timer to publish state continuously
        self.timer_ = self.create_timer(0.1, self.publish_continuous)

    def zebra_crossing_callback(self, msg):
        self.zebra_crossing_detected = msg.data
        self.make_decision()

    def person_callback(self, msg):
        self.person_detected = msg.data
        self.make_decision()

    def stop_sign_callback(self, msg):
        self.stop_sign_detected = msg.data
        self.make_decision()

    def handle_stopping(self):
        if self.is_stopped:
            elapsed_time = time.time() - self.stop_timer_start
            if elapsed_time >= 5.0:
                self.get_logger().info("Resuming after stop.")
                self.is_stopped = False
                self.curr_state = 0  # Back to normal state

                # Start cooldown after resuming
                self.get_logger().info("Starting cooldown for stop sign.")
                self.cooldown_active = True
                self.cooldown_start = time.time()

    def handle_cooldown(self):
        if self.cooldown_active:
            cooldown_elapsed = time.time() - self.cooldown_start
            if cooldown_elapsed >= 3.0:
                self.get_logger().info("Cooldown ended. Can detect stop sign again.")
                self.cooldown_active = False

    def make_decision(self):
        # Handle stop sign detection
        if self.stop_sign_detected and not self.is_stopped and not self.cooldown_active:
            self.get_logger().info("Stop sign detected! Stopping for 5 seconds.")
            self.curr_state = 1  # Stop sign state
            self.publish_continuous()
            self.is_stopped = True
            self.stop_timer_start = time.time()
            return

        # Handle zebra crossing detection
        if self.zebra_crossing_detected:
            self.get_logger().info("Zebra crossing detected! Switching to state 2.")
            if self.person_detected:
                # Handle Person Detection
                self.get_logger().info("Person detected! Switching to state 3.")
                self.curr_state = 3
                self.publish_continuous()
            else:
                self.curr_state = 2
                self.publish_continuous()
            return        
            
        
        # Ensure stopping logic is handled correctly
        self.handle_stopping()
        self.handle_cooldown()

        # Default action: Normal state
        if not self.is_stopped:
            self.curr_state = 0  # Normal driving state
            self.get_logger().info("No obstacles detected. Continuing.")

    def publish_continuous(self):
        """ Continuously publishes the current state """
        self.publish_drive_command(self.curr_state)

    def publish_drive_command(self, state):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(state)  # Convert state to float for publishing
        self.get_logger().info(f"Publishing State: {state}")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
