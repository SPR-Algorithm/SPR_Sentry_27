import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    return LaunchDescription([
        # 启动 status_wz_publisher 节点
        Node(
            package='commander',
            executable='pass_hole',
            name='status_wz_publisher',
            namespace='red_standard_robot1',
            remappings=[
                ('/tf', '/red_standard_robot1/tf'),
                ('/tf_static', '/red_standard_robot1/tf_static')
            ],
            parameters=[{
                'polygons_json': ParameterValue(LaunchConfiguration('polygons_json', default='[[[10, 10], [10.78, -10.35], [-12.98, -10.85], [-12.68, 10.4]]]'), value_type=str)
            }],
            output='screen'
        ),
 
    ])
