from launch import LaunchDescription           
from launch_ros.actions import Node     
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution       

def generate_launch_description():             
    init_x_launch_arg = DeclareLaunchArgument('init_x', default_value=TextSubstitution(text='0.0'))
    init_y_launch_arg = DeclareLaunchArgument('init_y', default_value=TextSubstitution(text='0.0'))
    init_yaw_launch_arg = DeclareLaunchArgument('init_yaw', default_value=TextSubstitution(text='0.0'))
    move_strategy_config_launch_arg = DeclareLaunchArgument('move_strategy_config', 
                                                            default_value=TextSubstitution(text='/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/commander/commander/config1.yaml'))
    pcd_map_launch_arg = DeclareLaunchArgument('pcd_map', 
                                                default_value=TextSubstitution(text='/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_localization/fast_lio/PCD/test1.pcd'))
    init_location_launch_arg = DeclareLaunchArgument('init_location', default_value=TextSubstitution(text='False'))
    return LaunchDescription([                 
        Node(                                  
            package='commander',          
            executable='time_perception', 
        ),
        # Node(                                  
        #     package='commander',          
        #     executable='event_perception', 
        # ),
        init_x_launch_arg,
        init_y_launch_arg,
        init_yaw_launch_arg,
        move_strategy_config_launch_arg,
        pcd_map_launch_arg,
        init_location_launch_arg,
        Node(                                  
            package='commander',          
            executable='commander', 
            parameters=[{
                'init_x' : LaunchConfiguration('init_x'),
                'init_y' : LaunchConfiguration('init_y'),
                'init_yaw' : LaunchConfiguration('init_yaw'),
                'move_strategy_config' : LaunchConfiguration('move_strategy_config'),
                'pcd_map' : LaunchConfiguration('pcd_map'),
                'init_location' : LaunchConfiguration('init_location'),
            }]
        ),
    ])
