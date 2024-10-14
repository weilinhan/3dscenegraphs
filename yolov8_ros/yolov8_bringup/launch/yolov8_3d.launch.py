from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #
    # ARGS
    #
    model_type = LaunchConfiguration("model_type")
    model_type_cmd = DeclareLaunchArgument(
        "model_type",
        default_value="YOLO",
        choices=["YOLO", "NAS"],
        description="Model type form Ultralytics (YOLO, NAS")
        
    tracker = LaunchConfiguration("tracker")
    tracker_cmd = DeclareLaunchArgument(
        "tracker",
        default_value="bytetrack.yaml",
        description="Tracker name or path"
    )

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="Device to use (GPU/CPU)"
    )

    enable = LaunchConfiguration("enable")
    enable_cmd = DeclareLaunchArgument(
        "enable",
        default_value="True",
        description="Whether to start YOLOv8 enabled"
    )

    threshold = LaunchConfiguration("threshold")
    threshold_cmd = DeclareLaunchArgument(
        "threshold",
        default_value="0.5",
        description="Minimum probability of a detection to be published"
    )

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/camera/rgb/image_raw",
        description="Name of the input image topic"
    )

    image_reliability = LaunchConfiguration("image_reliability")
    image_reliability_cmd = DeclareLaunchArgument(
        "image_reliability",
        default_value="2",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)"
    )

    input_depth_topic = LaunchConfiguration("input_depth_topic")
    input_depth_topic_cmd = DeclareLaunchArgument(
        "input_depth_topic",
        default_value="/camera/depth/image_raw",
        description="Name of the input depth topic"
    )

    depth_image_reliability = LaunchConfiguration("depth_image_reliability")
    depth_image_reliability_cmd = DeclareLaunchArgument(
        "depth_image_reliability",
        default_value="2",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input depth image topic (0=system default, 1=Reliable, 2=Best Effort)"
    )

    input_depth_info_topic = LaunchConfiguration("input_depth_info_topic")
    input_depth_info_topic_cmd = DeclareLaunchArgument(
        "input_depth_info_topic",
        default_value="/camera/depth/camera_info",
        description="Name of the input depth info topic"
    )

    depth_info_reliability = LaunchConfiguration("depth_info_reliability")
    depth_info_reliability_cmd = DeclareLaunchArgument(
        "depth_info_reliability",
        default_value="2",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input depth info topic (0=system default, 1=Reliable, 2=Best Effort)"
    )

    depth_image_units_divisor = LaunchConfiguration("depth_image_units_divisor")
    depth_image_units_divisor_cmd = DeclareLaunchArgument(
        "depth_image_units_divisor",
        default_value="1000",
        description="Divisor used to convert the raw depth image values into meters"
    )

    target_frame = LaunchConfiguration("target_frame")
    target_frame_cmd = DeclareLaunchArgument(
        "target_frame",
        default_value="base_link",
        description="Target frame to transform the 3D boxes"
    )

    maximum_detection_threshold = LaunchConfiguration("maximum_detection_threshold")
    maximum_detection_threshold_cmd = DeclareLaunchArgument(
        "maximum_detection_threshold",
        default_value="0.3",
        description="Maximum detection threshold in the z axis"
    )

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="yolo",
        description="Namespace for the nodes"
    )

    #
    # NODES
    #
    detector_node_cmd = Node(
        package="yolov8_ros",
        executable="yolov8_node",
        name="yolov8_node",
        namespace=namespace,
        parameters=[{
        
            "model": "ycblab.pt",
            "device": device,
            "enable": enable,
            "threshold": threshold,
            "image_reliability": image_reliability,
        }],
        remappings=[("image_raw", input_image_topic)]
    )
    
    hand_node_cmd = Node(
    	package="yolov8_ros",
        executable="hand_node",
        name="hand_node",
        namespace=namespace,
        parameters=[{
           "model": "handycb.pt",
           "device": device,
           "enable": enable,
           "threshold": threshold,
           "image_reliability": image_reliability,
        }],
        remappings=[("image_raw", input_image_topic)]
    )
	
    tracking_node_cmd = Node(
        package="yolov8_ros",
        executable="tracking_node",
        name="tracking_node",
        namespace=namespace,
        parameters=[{
            "tracker": tracker,
            "image_reliability": image_reliability
        }],
        remappings=[("image_raw", input_image_topic)]
    )
    
    topic_merger_cmd = Node(
        package="yolov8_ros",
        executable="topic_merger",
        name="topic_merger",
        namespace=namespace,
        parameters=[{
            "tracker": tracker,
            "image_reliability": image_reliability
        }],
        remappings=[("image_raw", input_image_topic)]
    )

    

    detect_3d_node_cmd = Node(
        package="yolov8_ros",
        executable="detect_3d_node",
        name="detect_3d_node",
        namespace=namespace,
        parameters=[{
            "target_frame": target_frame,
            "maximum_detection_threshold": maximum_detection_threshold,
            "depth_image_units_divisor": depth_image_units_divisor,
            "depth_image_reliability": depth_image_reliability,
            "depth_info_reliability": depth_info_reliability
        }],
        remappings=[
            ("depth_image", input_depth_topic),
            ("depth_info", input_depth_info_topic),
            ("detections", "tracking")
        ]
    )

    frame_tf_node_cmd = Node(
        package="yolov8_ros",
        executable="frame_tf_node",
        name="frame_tf_node",
        namespace=namespace,
        parameters=[{"image_reliability": image_reliability}],
        remappings=[
            ("image_raw", input_image_topic),
            ("detections", "detections_3d")
        ]
    )    



    debug_node_cmd = Node(
        package="yolov8_ros",
        executable="debug_node",
        name="debug_node",
        namespace=namespace,
        parameters=[{"image_reliability": image_reliability}],
        remappings=[
            ("image_raw", input_image_topic),
            ("detections", "detections_3d")
        ]
    )
    
   

    # New debug_node_position
    debug_node_position_cmd = Node(
        package="yolov8_ros",
        executable="class_position",
        name="class_position",
        namespace=namespace,
        parameters=[{"image_reliability": image_reliability}],
        remappings=[
            ("image_raw", input_image_topic),
            ("detections", "detections_3d")
        ]
    )
    
    
    
    ld = LaunchDescription()

 
    ld.add_action(tracker_cmd)
    ld.add_action(device_cmd)
    ld.add_action(enable_cmd)
    ld.add_action(threshold_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(image_reliability_cmd)
    ld.add_action(input_depth_topic_cmd)
    ld.add_action(depth_image_reliability_cmd)
    ld.add_action(input_depth_info_topic_cmd)
    ld.add_action(depth_info_reliability_cmd)
    ld.add_action(depth_image_units_divisor_cmd)
    ld.add_action(target_frame_cmd)
    ld.add_action(maximum_detection_threshold_cmd)
    ld.add_action(namespace_cmd)

    ld.add_action(detector_node_cmd)
    ld.add_action(tracking_node_cmd)
    ld.add_action(detect_3d_node_cmd)
    ld.add_action(debug_node_cmd)
    ld.add_action(debug_node_position_cmd)  # Added the new node
    ld.add_action(hand_node_cmd)
    ld.add_action(topic_merger_cmd)
    ld.add_action(frame_tf_node_cmd)
   
    return ld

