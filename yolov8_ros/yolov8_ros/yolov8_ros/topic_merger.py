import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import DetectionArray  # Import your message type
from std_msgs.msg import Header

class DetectionMerger(Node):
    def __init__(self):
        super().__init__('topic_merger')

        # Create subscribers for the two topics
        self.subscriber1 = self.create_subscription(
            DetectionArray,
            'hand',
            self.listener_callback1,
            10
        )

        self.subscriber2 = self.create_subscription(
            DetectionArray,
            'ycbdataset',
            self.listener_callback2,
            10
        )

        # Create a publisher for the merged topic
        self.publisher = self.create_publisher(DetectionArray, 'detections', 10)

        # Store the latest received messages
        self.detections1 = None
        self.detections2 = None

    def listener_callback1(self, msg):
        self.detections1 = msg
        self.merge_and_publish()

    def listener_callback2(self, msg):
        self.detections2 = msg
        self.merge_and_publish()

    def merge_and_publish(self):
        if self.detections1 is not None and self.detections2 is not None:
            # Create a new merged message
            merged_msg = DetectionArray()
            merged_msg.header.stamp = self.detections1.header.stamp
            merged_msg.header.frame_id = self.detections1.header.frame_id
            
            # Combine detections from both messages
            merged_msg.detections = self.detections1.detections + self.detections2.detections

            # Publish the merged message
            self.publisher.publish(merged_msg)

            # Reset the stored messages
            self.detections1 = None
            self.detections2 = None

def main(args=None):
    rclpy.init(args=args)
    node = DetectionMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()
