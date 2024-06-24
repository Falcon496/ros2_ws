import os
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    teleop_node = Node(
                package='gz_sim',
                executable='teleop_node',
                output='screen',
                #別ターミナルで起動する設定
                prefix="xterm -e"
                )


    pkg_share_dir = get_package_share_directory('gz_sim')
    model_path = os.path.join(pkg_share_dir, "models")
    #ignition gazeboがモデルにアクセスできるように設定
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        model_path])
  
    #ワールドのsdfファイルを設定(worldタグのあるsdfファイル)
    world = os.path.join(pkg_share_dir, "models", "worlds", "nav_slam.sdf")
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='world')
    #ignition gazeboの起動設定
    ign_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                          'launch', 'ign_gazebo.launch.py')]),
        launch_arguments=[('ign_args', [' -r -v 3 ' +
                          world
                         ])])
    
    #ロボットをスポーンさせる設定
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-entity', 'LidarRobo',
                   '-name', 'LidarRobo',
                   #ロボットのsdfファイルを指定
                   '-file', PathJoinSubstitution([
                        pkg_share_dir,
                        "models", "LidarRobo", "model.sdf"]),
                    #ロボットの位置を指定
                   '-allow_renaming', 'true',
                   '-x', '-5.515',
                   '-y', '0.365',
                   '-z', '0.04',
                   '-Y', str(np.pi/2)],
        )
    
    #フィールドをスポーンさせる設定
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
            #フィールドのsdfファイルを指定
        arguments=['-file', PathJoinSubstitution([
                        pkg_share_dir,
                        "models", "field", "model.sdf"]),
                   '-allow_renaming', 'false'],
        )


    #ros_ign_bridgeの起動設定
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            #brigdeの設定ファイルを指定
            'config_file': os.path.join(pkg_share_dir, 'config', 'nav_slam.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'qos_overrides./odom.publisher.durability': 'transient_local',
        },{'use_sim_time': use_sim_time}],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen'
    )
    
    #rviz2の設定フィルのパスを取得
    rviz_config_dir = os.path.join(
        pkg_share_dir,
        'config',
        'nav_rviz.rviz')
    #rviz2の起動設定
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    
    #ロボットのsdfファイルのパスを取得
    sdf = os.path.join(
        get_package_share_directory('gz_sim'),
        'models', 'LidarRobo', 'model.sdf')
    
    #xacroでsdfファイルをurdfに変換
    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)
    
    #robot_state_publsherの起動設定
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': doc.toxml()}]) # type: ignore
    
    #laserscan_mergerの起動オプション設定
    laserscan_merger_params_file = LaunchConfiguration('laserscan_merger_params_file')
    declare_laserscan_merger_params_cmd = DeclareLaunchArgument(
        'laserscan_merger_params_file',
        default_value=os.path.join(get_package_share_directory("gz_sim"),
                                   'params', 'laserscan_merge.yaml'),
        description='Path to param config in yaml format')
    # laserscan_mergerの起動設定
    laserscan_multi_merger = Node(
        parameters=[laserscan_merger_params_file],
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name="laserscan_multi_merger",
        output='both')

    #slam_toolboxの起動オプション設定
    slam_params_file = LaunchConfiguration('slam_params_file')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("gz_sim"),
                                   'params', 'slam_param.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    #slam_toolboxの起動設定
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')



    return LaunchDescription([
        ign_resource_path,
        ign_gz,
        ignition_spawn_entity,
        ignition_spawn_world,
                             
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),

        bridge,
        robot_state_publisher,
        teleop_node,
        rviz2,
        declare_laserscan_merger_params_cmd,
        laserscan_multi_merger,
        declare_slam_params_file_cmd,
        start_async_slam_toolbox_node,
    ])

