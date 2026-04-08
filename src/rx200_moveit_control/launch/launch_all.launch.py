import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ----------------------------------------------------------
    # Launch Arguments
    # ----------------------------------------------------------
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='actual',
        description='actual or fake'
    )

    default_gripper_state = DeclareLaunchArgument(
        'default_gripper_state',
        default_value='true',
        description='Initial state of the gripper'
    )

    use_camera_state = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Launch RealSense camera'
    )

    robot_type = LaunchConfiguration('robot_type')
    use_camera = LaunchConfiguration('use_camera')

    hardware_type = PythonExpression(
        ["'", robot_type, "' if '", robot_type, "' == 'actual' else 'fake'"]
    )

    # ----------------------------------------------------------
    # URDF
    # ----------------------------------------------------------
    urdf_path = PathJoinSubstitution([
        FindPackageShare('rx200_xsarm_descriptions'),
        'urdf',
        'rx200.urdf.xacro'
    ])

    robot_description = Command([
        "xacro ",
        urdf_path,
        " robot_model:=rx200",
        " robot_name:=rx200",
        " use_world_frame:=true",
        " hardware_type:=", hardware_type
    ])

    # ----------------------------------------------------------
    # MoveIt + Robot State Publisher
    # ----------------------------------------------------------
    interbotix_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('interbotix_xsarm_moveit'),
                'launch',
                'xsarm_moveit.launch.py'
            )
        ),
        launch_arguments={
            'robot_model': 'rx200',
            'robot_name': 'rx200',
            'hardware_type': hardware_type,
            'use_moveit_rviz': 'true',
            'robot_description': robot_description,
            'rviz_frame': 'rx200/base_link',
        }.items()
    )

    # ----------------------------------------------------------
    # RealSense Camera (MATCHES URDF)
    # ----------------------------------------------------------
    rs_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ]),
        launch_arguments={

            'camera_name': 'camera',
            'camera_namespace': 'camera',

            # Resolution
            'rgb_camera.color_profile': '640,480,30',
            'depth_module.depth_profile': '640,480,30',

            # Enable depth → color alignment
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'initial_reset': 'false',
            'publish_tf': 'true',

            # ✨ MATCH EXACTLY YOUR URDF JOINTS ✨
            'base_frame_id': 'rx200/camera_link',
            'color_frame_id': 'rx200/camera_color_frame',
            'color_optical_frame_id': 'rx200/camera_color_optical_frame',
            'depth_frame_id': 'rx200/camera_depth_frame',
            'depth_optical_frame_id': 'rx200/camera_depth_optical_frame',

        }.items(),
        condition=IfCondition(use_camera)
    )

    # ----------------------------------------------------------
    # Pointcloud Filter
    # ----------------------------------------------------------
    pc_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'launch',
                'pc_filter.launch.py',
            ])
        ]),
        launch_arguments={
            'filter_ns': 'pc_filter',
            'filter_params': PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_perception'),
                'config',
                'filter_params.yaml'
            ]),
            'enable_pipeline': 'false',
            'cloud_topic': '/camera/camera/depth/color/points',
            'use_pointcloud_tuner_gui': 'false',
        }.items(),
        condition=IfCondition(use_camera)
    )

    # ----------------------------------------------------------
    # MoveIt Client, GUI, Vision Node
    # ----------------------------------------------------------
    moveit_client_node = Node(
        package='rx200_moveit_control',
        executable='rx200_moveit_client',
        name='rx200_moveit_client',
        output='screen',
        parameters=[{
            'start_state_gripper': LaunchConfiguration('default_gripper_state'),
            'use_sim_time': PythonExpression(["'", hardware_type, "' == 'fake'"])
        }]
    )

    keyboard_gui_node = Node(
        package='rx200_moveit_control',
        executable='keyboard_gui',
        name='keyboard_gui',
        output='screen'
    )

    vision_node = Node(
        package='rx200_moveit_control',
        executable='vision',
        name='vision',
        output='screen'
    )

    # ----------------------------------------------------------
    # RETURN
    # ----------------------------------------------------------
    return LaunchDescription([
        robot_type_arg,
        default_gripper_state,
        use_camera_state,

        interbotix_moveit_launch,
        rs_camera_launch,
        pc_filter_launch,

        moveit_client_node,
        keyboard_gui_node,
        vision_node
    ])
