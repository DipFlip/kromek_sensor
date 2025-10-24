from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'integration_seconds',
            default_value='1',
            description='Integration time in seconds (how long to collect data before publishing)'
        ),
        DeclareLaunchArgument(
            'num_bins',
            default_value='100',
            description='Number of histogram bins (1-4096). Lower values reduce data size, higher values give better energy resolution'
        ),
        DeclareLaunchArgument(
            'gain',
            default_value='-1',
            description='Detector gain (20-250). -1 = read from device and don\'t change. Higher gain = more sensitive but more noise'
        ),
        DeclareLaunchArgument(
            'lld',
            default_value='-1',
            description='Lower Level Discriminator - energy threshold (0-4095). -1 = read from device. Filters out low-energy noise'
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='-1.0',
            description='Publishing rate in Hz. -1 = use 1/integration_seconds. Can be higher for real-time updates'
        ),
        DeclareLaunchArgument(
            'publish_histogram',
            default_value='true',
            description='Publish energy histogram on /kromek/raw'
        ),
        DeclareLaunchArgument(
            'publish_sum',
            default_value='true',
            description='Publish total counts on /kromek/sum'
        ),
        DeclareLaunchArgument(
            'publish_device_status',
            default_value='false',
            description='Publish detector settings (gain/bias/LLD) on /kromek/status'
        ),

        # Launch the node
        Node(
            package='kromek_ros2_hidapi',
            executable='kromek_hidapi_node',
            name='kromek_hidapi_node',
            output='screen',
            parameters=[{
                'integration_seconds': LaunchConfiguration('integration_seconds'),
                'num_bins': LaunchConfiguration('num_bins'),
                'gain': LaunchConfiguration('gain'),
                'lld': LaunchConfiguration('lld'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'publish_histogram': LaunchConfiguration('publish_histogram'),
                'publish_sum': LaunchConfiguration('publish_sum'),
                'publish_device_status': LaunchConfiguration('publish_device_status'),
            }],
            respawn=True,
            respawn_delay=2.0
        )
    ])
