import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import cv2
from object_tracker.process_frame import process_frames

class ImageViewer(Node):
    def __init__(self):
        super().__init__('x500_mono_cam')
        self.cap = cv2.VideoCapture(0)
        self.origin = 0
        self.lock = False
        self.dist = 0
        self.direction = False
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.int_publisher_ = self.create_publisher(Int32MultiArray, '/hand_distance', 10)
        cv2.namedWindow('Processed Frame', cv2.WINDOW_NORMAL)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame_, self.origin, self.lock, self.dist, self.direction = process_frames(frame, self.origin, self.lock, self.dist, self.direction)
            int_msg = Int32MultiArray()
            int_msg.data = [int(self.dist), int(self.lock), int(self.direction)]
            self.int_publisher_.publish(int_msg)
            self.get_logger().info(f'Publishing distance: {int_msg.data}')
            self.get_logger().info(f'dist: {self.dist}')
            self.get_logger().info(f'locked: {self.lock}')
            cv2.imshow('Processed Frame', frame_)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()