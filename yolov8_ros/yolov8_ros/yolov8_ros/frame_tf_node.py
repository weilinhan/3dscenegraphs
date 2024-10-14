import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_matrix
import numpy as np
from yolov8_msgs.msg import DetectionArray

class DetectionTransformer(Node):
    def __init__(self):
        super().__init__('frame_tf_3d')
        
        # sub
        self.subscription = self.create_subscription(
            DetectionArray,
            'detections_3d',
            self.listener_callback,
            10)
        
        # pub
        self.publisher = self.create_publisher(
            DetectionArray,
            'detections_3d_transformed',
            10)
        
      
        self.quaternion = (-0.5000, 0.5000, -0.5000, 0.5000)
        self.translation = (0.0, 0.0, 0.00)  
        
        
        self.transformation_matrix = quaternion_matrix(self.quaternion)
        self.transformation_matrix[:3, 3] = self.translation

    def listener_callback(self, msg):
        transformed_msg = DetectionArray()
        transformed_msg.header = msg.header
        transformed_msg.header.frame_id = 'zed_left_camera_frame'
        
        # 
        for detection in msg.detections:
            # 
            center = np.array([detection.bbox3d.center.position.x,
                               detection.bbox3d.center.position.y,
                               detection.bbox3d.center.position.z, 1])
            transformed_center = np.dot(self.transformation_matrix, center)[:3]
            
            # 
            detection.bbox3d.center.position.x = transformed_center[0]
            detection.bbox3d.center.position.y = transformed_center[1]
            detection.bbox3d.center.position.z = transformed_center[2]
            
            # 
            detection.bbox3d.frame_id = 'zed_left_camera_frame'
            
            transformed_msg.detections.append(detection)
        
        # 
        self.publisher.publish(transformed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DetectionTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

