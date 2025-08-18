import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージのパスを取得 (hokuyo_navigation2 に修正)
    hokuyo_navigation2_pkg_path = get_package_share_directory('hokuyo_navigation2')
    icart_mini_driver_pkg_path = get_package_share_directory('icart_mini_driver')

    # URDFファイルのパス (hokuyo_navigation2 パッケージ内の urdf ディレクトリを参照)
    urdf_path = os.path.join(hokuyo_navigation2_pkg_path, 'urdf', 'arno.xacro')

    # use_sim_time のLaunch引数を宣言
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # ロボットのdescription
    robot_description = {"robot_description": Command(['xacro ', urdf_path])} # use_ros2_control をtrueに設定

    # robot_state_publisher ノード
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # ジョイントステートパブリッシャー (必要に応じて)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': True, 'use_sim_time': use_sim_time}]
    )

    # icart_mini_driver_ros2 のLaunchファイルをインクルード
    icart_mini_driver_launch_path = os.path.join(
        icart_mini_driver_pkg_path, 'launch', 'icart_mini_bringup_launch.py'
    )
    icart_mini_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(icart_mini_driver_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return launch.LaunchDescription([
        declare_use_sim_time_cmd,
        joint_state_publisher_node,
        robot_state_publisher_node,
        icart_mini_driver_launch,
    ])