import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

 
        self.cv_bridge = CvBridge()

        #sub and pub name
        self.declare_parameter('input_topic', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('output_topic', '/image_converted')

       
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # sub
        self.image_subscriber = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )

        # pub
        self.image_publisher = self.create_publisher(
            Image,
            output_topic,
            10
        )

        self.get_logger().info(f"Node started. Subscribing to {input_topic}, and publishing to {output_topic}")

    def image_callback(self, msg: Image):
        try:
            # use cv_bridge transform（bgra8 to OpenCV（bgra8）
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return

        # transform BGRA8 to BGR8（remove alpha channel）
        bgr_image = cv_image[:, :, :3]

        try:
            # transform by cv_bridge OpenCV（bgr8）to ROS message
            bgr_image_msg = self.cv_bridge.cv2_to_imgmsg(bgr_image, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert BGR image to ROS Image message: {str(e)}")
            return

        
        bgr_image_msg.header.stamp = msg.header.stamp
        bgr_image_msg.header.frame_id = msg.header.frame_id
        self.image_publisher.publish(bgr_image_msg)
        self.get_logger().info('Published converted image')


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
