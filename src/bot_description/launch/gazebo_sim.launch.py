import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # get share path
    urdf_package_path = get_package_share_directory('bot_description')

    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'bot/bot.urdf.xacro')
    # default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')

    # declare a urdf parameter
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_xacro_path), description='load model file path'
    )

    # obtain the content through the file path(action_declare_arg_mode_path), then convert it into a parameter value object and pass it into robot_state_publisher
    substitutions_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type=str)

    # robot state publisher
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value,
                     'use_sim_time': True}]
    )

    # gazebo
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[('world', default_gazebo_world_path), ('verbose', 'true')]
    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'bot']
    )

    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller bot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )
    # 力控制器和两轮差速控制器均可以控制机器人的行动
    # 不推荐一个机器人有多个控制器控制一个地方的行动
    # action_load_effort_controller = launch.actions.ExecuteProcess(
    #     cmd='ros2 control load_controller bot_effort_controller --set-state active'.split(' '),
    #     output='screen'
    # )

    action_load_bot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller bot_diff_drive_controller --set-state active'.split(' '),
        output='screen'
    )



    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_bot_diff_drive_controller],
            )
        ),
    ])