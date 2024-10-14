import cv2
import random
import numpy as np
from typing import Tuple
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState
import message_filters
from cv_bridge import CvBridge
from ultralytics.utils.plotting import Annotator, colors
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
import networkx as nx
import matplotlib.pyplot as plt
import networkx as nx
import math



class DebugNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("class_position")

        self._class_to_color = {}  
        self.cv_bridge = CvBridge()  
        self.previous_detections = None  
        self.fig = None
        self.ax = None
        self.graph = nx.Graph()
        # params
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT)
    def get_color_for_class(self, class_name: str) -> Tuple[int, int, int]:
        if class_name not in self._class_to_color:
            self._class_to_color[class_name] = (random.randint(0, 255),
                                                random.randint(0, 255),
                                                random.randint(0, 255))
        return self._class_to_color[class_name]
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Configuring...")

        # QoS
        self.image_qos_profile = QoSProfile(
            reliability=self.get_parameter("image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # pub
        self._dbg_pub = self.create_publisher(Image, "class_position", 10)
        self._bb_markers_pub = self.create_publisher(MarkerArray, "dgb_bb_markers_1class", 10)
        self._kp_markers_pub = self.create_publisher(MarkerArray, "dgb_kp_markers_1class", 10)

        super().on_configure(state)
        self.get_logger().info(f"[{self.get_name()}] Configured")

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Activating...")

        self.image_sub = message_filters.Subscriber(self, Image, "image_raw", qos_profile=self.image_qos_profile)
        self.detections_sub = message_filters.Subscriber(self, DetectionArray, "detections", qos_profile=10)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.detections_sub), 10, 0.5)
        self._synchronizer.registerCallback(self.detections_cb)

        super().on_activate(state)
        self.get_logger().info(f"[{self.get_name()}] Activated")

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Deactivating...")

        self.destroy_subscription(self.image_sub.sub)
        self.destroy_subscription(self.detections_sub.sub)

        del self._synchronizer  
        super().on_deactivate(state)
        self.get_logger().info(f"[{self.get_name()}] Deactivated")

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Cleaning up...")

        self.destroy_publisher(self._dbg_pub)
        self.destroy_publisher(self._bb_markers_pub)
        self.destroy_publisher(self._kp_markers_pub)

        super().on_cleanup(state)
        self.get_logger().info(f"[{self.get_name()}] Cleaned up")

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Shutting down...")
        super().on_cleanup(state)
        self.get_logger().info(f"[{self.get_name()}] Shutted down")
        return TransitionCallbackReturn.SUCCESS

    def is_colliding(self, bbox1, bbox2):
        return (bbox1.center.position.x - bbox1.size.x / 2 <= bbox2.center.position.x + bbox2.size.x / 2 and
                bbox1.center.position.x + bbox1.size.x / 2 >= bbox2.center.position.x - bbox2.size.x / 2 and
                bbox1.center.position.y - bbox1.size.y / 2 <= bbox2.center.position.y + bbox2.size.y / 2 and
                bbox1.center.position.y + bbox1.size.y / 2 >= bbox2.center.position.y - bbox2.size.y / 2 and
                bbox1.center.position.z - bbox1.size.z / 2 <= bbox2.center.position.z + bbox2.size.z / 2 and
                bbox1.center.position.z + bbox1.size.z / 2 >= bbox2.center.position.z - bbox2.size.z / 2)

    def evaluate_static_relation(self, bbox1, bbox2, height_threshold=0.3):
        
        cos_theta = math.cos(math.radians(45))  # 0.707
        sin_theta = math.sin(math.radians(45))  # 0.707


        def rotate_around_x(y, z):
                """
                rotation
                """
                y_prime = y * cos_theta - z * sin_theta
                z_prime = y * sin_theta + z * cos_theta
                return y_prime, z_prime


        bbox1_center_x = bbox1.center.position.x
        bbox1_center_y, bbox1_center_z = rotate_around_x(bbox1.center.position.y, bbox1.center.position.z)

        bbox2_center_x = bbox2.center.position.x
        bbox2_center_y, bbox2_center_z = rotate_around_x(bbox2.center.position.y, bbox2.center.position.z)


        relations = {}
       

        if bbox1.center.position.x + bbox1.size.x / 2 < bbox2.center.position.x - bbox2.size.x / 2:
            relations['left_of'] = True
        elif bbox1.center.position.x - bbox1.size.x / 2 > bbox2.center.position.x + bbox2.size.x / 2:
            relations['right_of'] = True

        if bbox1.center.position.y + bbox1.size.y / 2 < bbox2.center.position.y - bbox2.size.y / 2:
            relations['behind'] = True
        elif bbox1.center.position.y - bbox1.size.y / 2 > bbox2.center.position.y + bbox2.size.y / 2:
            relations['in_front'] = True
        
        z_distance = abs(bbox1.center.position.z - bbox2.center.position.z)  
        if bbox1.center.position.z + bbox1.size.z / 2 > bbox2.center.position.z - bbox2.size.z / 2 and z_distance > height_threshold:
            relations['below'] = True  
        elif bbox1.center.position.z - bbox1.size.z / 2 <  bbox2.center.position.z + bbox2.size.z / 2 and z_distance > height_threshold:
            relations['above'] = True  
        
        if (bbox1.center.position.x - bbox1.size.x / 2 < bbox2.center.position.x + bbox2.size.x / 2 and
            bbox1.center.position.x + bbox1.size.x / 2 > bbox2.center.position.x - bbox2.size.x / 2 and
            bbox1.center.position.y - bbox1.size.y / 2 < bbox2.center.position.y + bbox2.size.y / 2 and
            bbox1.center.position.y + bbox1.size.y / 2 > bbox2.center.position.y - bbox2.size.y / 2):
            relations['contact'] = True

        elif (bbox1.center.position.x - bbox1.size.x / 2 >= bbox2.center.position.x - bbox2.size.x / 2 and
            bbox1.center.position.x + bbox1.size.x / 2 <= bbox2.center.position.x + bbox2.size.x / 2 and
            bbox1.center.position.y - bbox1.size.y / 2 >= bbox2.center.position.y - bbox2.size.y / 2 and
            bbox1.center.position.y + bbox1.size.y / 2 <= bbox2.center.position.y + bbox2.size.y / 2):
            relations['inside'] = True

        elif (bbox1.center.position.x - bbox1.size.x / 2 <= bbox2.center.position.x - bbox2.size.x / 2 and
            bbox1.center.position.x + bbox1.size.x / 2 >= bbox2.center.position.x + bbox2.size.x / 2 and
            bbox1.center.position.y - bbox1.size.y / 2 <= bbox2.center.position.y - bbox2.size.y / 2 and
            bbox1.center.position.y + bbox1.size.y / 2 >= bbox2.center.position.y + bbox2.size.y / 2):
            relations['surround'] = True

        return relations

    def evaluate_dynamic_relation(self, prev_bbox1, curr_bbox1, prev_bbox2, curr_bbox2, distance_threshold=0.02) -> str:

        prev_distance = np.linalg.norm(np.array([prev_bbox1.center.position.x, prev_bbox1.center.position.y, prev_bbox1.center.position.z]) -
                                        np.array([prev_bbox2.center.position.x, prev_bbox2.center.position.y, prev_bbox2.center.position.z]))
        curr_distance = np.linalg.norm(np.array([curr_bbox1.center.position.x, curr_bbox1.center.position.y, curr_bbox1.center.position.z]) -
                                        np.array([curr_bbox2.center.position.x, curr_bbox2.center.position.y, curr_bbox2.center.position.z]))

        is_colliding = self.is_colliding(curr_bbox1, curr_bbox2)

        velocity_bbox1 = np.array([curr_bbox1.center.position.x - prev_bbox1.center.position.x,
                                curr_bbox1.center.position.y - prev_bbox1.center.position.y,
                                curr_bbox1.center.position.z - prev_bbox1.center.position.z])

        velocity_bbox2 = np.array([curr_bbox2.center.position.x - prev_bbox2.center.position.x,
                                curr_bbox2.center.position.y - prev_bbox2.center.position.y,
                                curr_bbox2.center.position.z - prev_bbox2.center.position.z])

        moving_bbox1 = np.linalg.norm(velocity_bbox1) > distance_threshold  
        moving_bbox2 = np.linalg.norm(velocity_bbox2) > distance_threshold  
        movement_relation = None
        #if not is_colliding:
        #    if abs(curr_distance - prev_distance) < distance_threshold:
        #        movement_relation = 'stable' 
        #    elif curr_distance < prev_distance:
        #        movement_relation = 'getting_closer'  
        #    else:   
        #        movement_relation = 'moving_apart'  
        #else:
        #    movement_relation = 'static'  
        if is_colliding:
        
            if not moving_bbox1 and not moving_bbox2 and abs(curr_distance - prev_distance) < distance_threshold:
                return 'halting_together'

        
            elif (not moving_bbox1 or not moving_bbox2) and abs(curr_distance - prev_distance) > distance_threshold:
                return 'fixed_moving_together'

        
            elif moving_bbox1 and moving_bbox2:
                return 'moving_together'

        else:
          
            if abs(curr_distance - prev_distance) < distance_threshold:
                movement_relation = 'stable'
            elif curr_distance < prev_distance:
                movement_relation = 'getting_closer'
            else:
                movement_relation = 'moving_apart'


        return movement_relation  

    def detections_cb(self, img_msg: Image, detection_msg: DetectionArray) -> None:
        detections = detection_msg.detections  
        relations = []  
        bb_marker_array = MarkerArray()  
        kp_marker_array = MarkerArray() 

        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)

        if self.previous_detections is None or len(self.previous_detections) != len(detections):
            self.previous_detections = detections 
            return 

        for i in range(len(detections)):
            bbox1 = detections[i].bbox3d  
            for j in range(i + 1, len(detections)):
                bbox2 = detections[j].bbox3d  

                dynamic_relation = None
                try:
                    prev_bbox1 = self.previous_detections[i].bbox3d
                    prev_bbox2 = self.previous_detections[j].bbox3d
                    dynamic_relation = self.evaluate_dynamic_relation(prev_bbox1, bbox1, prev_bbox2, bbox2)
                    self.get_logger().info(f"relation between {detections[i].class_name} and {detections[j].class_name}: {dynamic_relation}")
                except ValueError:
                    self.get_logger().error(f"Error finding previous detection for {detections[i].class_name} or {detections[j].class_name}")

                if dynamic_relation and dynamic_relation != 'static':
                    relations.append((i, j, dynamic_relation))
                  

                static_relations = self.evaluate_static_relation(bbox1, bbox2)
                self.get_logger().info(f"Static relations between {detections[i].class_name} and {detections[j].class_name}: {static_relations}")
                relations.append((i, j, static_relations))

            color = self.get_color_for_class(detections[i].class_name)
            cv_image = self.draw_box(cv_image, detections[i], color)
            cv_image = self.draw_keypoints(cv_image, detections[i])

            if detections[i].bbox3d.frame_id:
                marker = self.create_bb_marker(detections[i], color)
                marker.header.stamp = img_msg.header.stamp
                marker.id = len(bb_marker_array.markers)
                bb_marker_array.markers.append(marker)

        self.previous_detections = detection_msg.detections

        graph = self.build_graph(detections, relations)

        nx.write_gml(graph, "/home/weilinhan/Desktop/ros2_ws/src/yolov8_ros/save/graph.gml")  

        self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding=img_msg.encoding))
        self._bb_markers_pub.publish(bb_marker_array)
        self._kp_markers_pub.publish(kp_marker_array)
        self.draw_and_show_graph(graph)
    def draw_and_show_graph(self, graph):
        plt.close(self.fig)
        self.fig, self.ax = plt.subplots(figsize=(10,8))

        #plt.figure(figsize=(10, 8))

        pos = nx.spring_layout(graph)  
        labels = nx.get_node_attributes(graph, 'class_name')  
        edge_labels = nx.get_edge_attributes(graph, 'relation')  

        
        node_colors = []
        for node, data in graph.nodes(data=True):
            class_name = data['class_name']
            color = self.get_color_for_class(class_name) 
            color_normalized = [c / 255.0 for c in color]  
            node_colors.append(color_normalized)  

        
        nx.draw(graph, pos, labels=labels, with_labels=True, node_color=node_colors, node_size=5000, font_size=10)
        
        
        nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels, font_color='red', font_size=8)

        plt.title("GNN Graph Visualization")
        #plt.show()  

   
    def draw_box(self, cv_image: np.ndarray, detection: Detection, color: Tuple[int]) -> np.ndarray:
        box_msg: BoundingBox2D = detection.bbox
        min_pt = (round(box_msg.center.position.x - box_msg.size.x / 2.0),
                  round(box_msg.center.position.y - box_msg.size.y / 2.0))
        max_pt = (round(box_msg.center.position.x + box_msg.size.x / 2.0),
                  round(box_msg.center.position.y + box_msg.size.y / 2.0))

        cv2.rectangle(cv_image, min_pt, max_pt, color, 2)  
        label = f"{detection.class_name} ({detection.score:.2f})"
        cv2.putText(cv_image, label, (min_pt[0], min_pt[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)  
        return cv_image

    def draw_keypoints(self, cv_image: np.ndarray, detection: Detection) -> np.ndarray:
        keypoints_msg = detection.keypoints
        for kp in keypoints_msg.data:
            cv2.circle(cv_image, (int(kp.point.x), int(kp.point.y)), 5, (0, 255, 0), -1)  
        return cv_image

    def create_bb_marker(self, detection: Detection, color: Tuple[int]) -> Marker:
        bbox3d = detection.bbox3d

        marker = Marker()
        marker.header.frame_id = bbox3d.frame_id

        marker.ns = "yolov8_3d"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False

        marker.pose.position.x = bbox3d.center.position.x
        marker.pose.position.y = bbox3d.center.position.y
        marker.pose.position.z = bbox3d.center.position.z

        marker.scale.x = bbox3d.size.x
        marker.scale.y = bbox3d.size.y
        marker.scale.z = bbox3d.size.z

        marker.color.r = color[0] / 255.0
        marker.color.g = color[1] / 255.0
        marker.color.b = color[2] / 255.0
        marker.color.a = 0.8 

        marker.lifetime = Duration(seconds=1.0).to_msg() 
        return marker

    def build_graph(self, detections, relations):
        G = nx.Graph()

       
        for i, detection in enumerate(detections):
           
            bbox_info = {
                'center': {
                    'x': detection.bbox3d.center.position.x,
                    'y': detection.bbox3d.center.position.y,
                    'z': detection.bbox3d.center.position.z
                },
                'size': {
                    'x': detection.bbox3d.size.x,
                    'y': detection.bbox3d.size.y,
                    'z': detection.bbox3d.size.z
                },
                'frame_id': detection.bbox3d.frame_id
            }

            
            G.add_node(i, class_name=detection.class_name, bbox=bbox_info)

       
        for (i, j, relation) in relations:
            G.add_edge(i, j, relation=relation)

        return G


def main():
    rclpy.init()
    node = DebugNode()  
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)  
    node.destroy_node()
    rclpy.shutdown()