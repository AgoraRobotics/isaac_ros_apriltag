import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    urdf_launch_dir = os.path.join(
        get_package_share_directory('depthai_descriptions'), 'launch')
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    name = LaunchConfiguration('name').perform(context)
    rgb_topic_name = name+'/rgb/image_raw'
    log_level = 'info'

    params_file = LaunchConfiguration("params_file")
    camera_model = LaunchConfiguration('camera_model',  default='OAK-D')

    parent_frame = LaunchConfiguration(
        'parent_frame',  default='oak_camera')
    cam_pos_x = LaunchConfiguration('cam_pos_x',     default='0.0')
    cam_pos_y = LaunchConfiguration('cam_pos_y',     default='0.0')
    cam_pos_z = LaunchConfiguration('cam_pos_z',     default='0.0')
    cam_roll = LaunchConfiguration('cam_roll',      default='0.0')
    cam_pitch = LaunchConfiguration('cam_pitch',     default='0.0')
    cam_yaw = LaunchConfiguration('cam_yaw',       default='0.0')
    use_composition = LaunchConfiguration(
        'rsp_use_composition', default='true')
    imu_from_descr = LaunchConfiguration('imu_from_descr', default='false')
    pass_tf_args_as_params = LaunchConfiguration(
        'pass_tf_args_as_params', default='false')
    override_cam_model = LaunchConfiguration(
        'override_cam_model', default='false')

    tf_params = {}
    if (pass_tf_args_as_params.perform(context) == 'true'):
        cam_model = ''
        if override_cam_model.perform(context) == 'true':
            cam_model = camera_model.perform(context)
        tf_params = {'camera': {
            'i_publish_tf_from_calibration': True,
            'i_tf_tf_prefix': name,
            'i_tf_camera_model': cam_model,
            'i_tf_base_frame': name,
            'i_tf_parent_frame': parent_frame.perform(context),
            'i_tf_cam_pos_x': cam_pos_x.perform(context),
            'i_tf_cam_pos_y': cam_pos_y.perform(context),
            'i_tf_cam_pos_z': cam_pos_z.perform(context),
            'i_tf_cam_roll': cam_roll.perform(context),
            'i_tf_cam_pitch': cam_pitch.perform(context),
            'i_tf_cam_yaw': cam_yaw.perform(context),
            'i_tf_imu_from_descr': imu_from_descr.perform(context),
        }
        }

    return [
        
        ComposableNodeContainer(
                package='rclcpp_components',
                name='oak_container',
                namespace='',
                executable='component_container_mt',
                output='screen'
        ),
                
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urdf_launch_dir, 'urdf_launch.py')),
            launch_arguments={'tf_prefix': name,
                              'camera_model': camera_model,
                              'base_frame': name,
                              'parent_frame': parent_frame,
                              'cam_pos_x': cam_pos_x,
                              'cam_pos_y': cam_pos_y,
                              'cam_pos_z': cam_pos_z,
                              'cam_roll': cam_roll,
                              'cam_pitch': cam_pitch,
                              'cam_yaw': cam_yaw,
                              'use_composition': use_composition,
                              'use_base_descr': pass_tf_args_as_params}.items()),

        LoadComposableNodes(
            target_container="oak_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=name,
                        remappings=[('/oak/rgb/image_raw', '/image_raw'),
                                    ('/oak/rgb/camera_info', '/camera_info')],
                        extra_arguments=[{"use_intra_process_comms": True}],
                        parameters=[params_file, tf_params],
                    )
            ],
        ),
        
        #### When using FastDDS this is necessary!!! ####
        
        # LoadComposableNodes(
        #     target_container="oak_container",
        #     composable_node_descriptions=[ComposableNode(
        #         package='isaac_ros_image_proc',
        #         plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        #         name='image_format_node',
        #         remappings=[
        #                 ('image', 'image_raw')],
        #         parameters=[{
        #                 'encoding_desired': 'rgb8',
        #         }]
        #         ),
        #     ],
        # ),
        
        LoadComposableNodes(
                target_container="oak_container",
                composable_node_descriptions=[ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                    name='rectify',
                    parameters=[{
                        'output_width': 1280,
                        'output_height': 720,
                    }],
                ),
            ],
        ),

        LoadComposableNodes(
                target_container="oak_container",
                composable_node_descriptions=[ComposableNode(
                    package='isaac_ros_apriltag',
                    plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                    name='apriltag',
                    remappings=[
                        ('image', 'image_rect')],
                ),
            ],
        ),
        
        LoadComposableNodes(
                target_container="oak_container",
                composable_node_descriptions=[ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                    name='resize',
                    remappings=[
                        ('/image', '/image_raw'),],
                    parameters=[{
                        'output_width': 640,
                        'output_height': 480,
                    }],
                ),
            ],
        ),
        
        # LoadComposableNodes(
        #     target_container="oak_container",
        #     composable_node_descriptions=[
        #         ComposableNode(
        #             package="depthai_filters",
        #             plugin="depthai_filters::SpatialBB",
        #             name="spatial_bb_node",
        #             remappings=[
        #                     ('stereo/camera_info', name+'/stereo/camera_info'),
        #                     ('nn/spatial_detections', name+'/nn/spatial_detections'),
        #                     ('rgb/preview/image_raw', name+'/rgb/preview/image_raw'),
        #             ],
        #             extra_arguments=[{"use_intra_process_comms": True}],
        #             parameters=[params_file],
        #         ),
        #     ],
        # ),
    
    ]

def generate_launch_description():

    oak_params_path = os.path.join(
        get_package_share_directory('isaac_ros_apriltag'),
        'config',
        'oak_params.yaml'
    )

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file", default_value=oak_params_path),
        DeclareLaunchArgument("parent_frame", default_value="oak_camera"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("rsp_use_composition", default_value='true'),
        DeclareLaunchArgument("pass_tf_args_as_params", default_value='false', description='Enables TF publishing from camera calibration file.'),
        DeclareLaunchArgument("imu_from_descr", default_value='false', description='Enables IMU publishing from URDF.'),
        DeclareLaunchArgument("override_cam_model", default_value='false', description='Overrides camera model from calibration file.'),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
