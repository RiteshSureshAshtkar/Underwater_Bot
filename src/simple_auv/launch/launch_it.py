from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
import os

def generate_launch_description():
    world_file = os.path.join(
        os.getenv('HOME'), 'ros2_simulation_ws', 'src','simple_auv', 'worlds','water_world.sdf'
    )
    launch_file_dir = os.path.join(get_package_share_directory('simple_auv'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_simple_auv = get_package_share_directory('simple_auv')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    rviz = LaunchConfiguration('rviz', default='false')
    
    # Set environment variables for Gazebo resources
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory('simple_auv'),
                         'models'))
    
    # Gazebo launch
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            world_file
        ]),
        TextSubstitution(text=' -r -v')],  # Removed extra -v1 which might cause issues
        'on_exit_shutdown': 'true'}.items()
    )
    
    # ROS2-Gazebo bridge for camera - Fixed topic names and frame IDs
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Image topic - matches the sensor topic name from SDF
            'camera@sensor_msgs/msg/Image@gz.msgs.Image',
            # Camera info topic
            'camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen',
        name='camera_bridge',
        remappings=[
            ('camera', '/tethys/camera'),
            ('camera/camera_info', '/tethys/camera_info')
        ]
    )
    
    # TF static transform publisher to fix frame ID issues
    camera_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
        name='camera_tf_publisher'
    )
    
    # AUV control bridges
    control_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/tethys/joint/propeller_joint/cmd_force@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/tethys/joint/horizontal_fins_joint/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/tethys/joint/vertical_fins_joint/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/tethys/pose@geometry_msgs/msg/Pose@gz.msgs.Pose'
            '/model/mesh/thruster_cmd@std_msgs/msg/Float64@gz.msgs.Double'
        ],
        output='screen',
        name='control_bridge'
    )
    
    # Optional RViz launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(rviz)
    )
    
    # Launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    ))
    
    # Add actions
    ld.add_action(set_env_vars_resources)
    ld.add_action(gazebo_cmd)
    ld.add_action(camera_bridge)
    ld.add_action(camera_tf_publisher)
    ld.add_action(control_bridge)
    ld.add_action(rviz_node)
    
    return ld
