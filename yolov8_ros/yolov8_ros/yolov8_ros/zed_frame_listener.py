#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration

class ZedFrameListener(Node):

    def __init__(self):
        super().__init__('zed_frame_listener')

        # create tf2 Buffer and TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # lookup_transform
        self.timer = self.create_timer(0.1, self.lookup_transform)

    def lookup_transform(self):
        try:
            #  'zed_left_camera_frame' 到 'map' 
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map',  # aim
                                                        'zed_left_camera_optical_frame',  # orginal
                                                        now,  # 
                                                        Duration(seconds=1.0))  # 

            # print the message
            self.get_logger().info(f"Translation: x={transform.transform.translation.x:.4f}, "
                                   f"y={transform.transform.translation.y:.4f}, "
                                   f"z={transform.transform.translation.z:.4f}")
            self.get_logger().info(f"Rotation (quaternion): x={transform.transform.rotation.x:.4f}, "
                                   f"y={transform.transform.rotation.y:.4f}, "
                                   f"z={transform.transform.rotation.z:.4f}, "
                                   f"w={transform.transform.rotation.w:.4f}")

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not find the transform: {e}")
            return

def main(args=None):
   
    rclpy.init(args=args)

    
    node = ZedFrameListener()

    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        pass
    finally:
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
