import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 自分のパッケージのshareディレクトリのパスを取得
    pkg_share = get_package_share_directory('rakingmotion_realsens')

    # RealsenseノードのLaunchファイルパスを取得
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # RVizの設定ファイルパスを取得
   # rviz_config_path = os.path.join(pkg_share, 'config', 'raking_view.rviz')

    # 1. RealSenseノードの起動
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path)
    )

    # 2. 自作ノードの起動
    raking_motion_node = Node(
        package='rakingmotion_realsens',
        executable='realsense_closest_node',
        name='realsense_closest_node',
        output='screen'
        # トピック名がコードと異なる場合は、ここでリマッピングも可能
        # remappings=[
        #     ('/camera/depth/camera_info', '/camera/camera/depth/camera_info'),
        #     ('/camera/depth/image_rect_raw', '/camera/camera/depth/image_rect_raw')
        # ]
    )

    # 3. RViz2の起動
    #  rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_path], # 保存したRViz設定ファイルをロード
    #     output='screen'
    # ) 

    return LaunchDescription([
        realsense_node,
        raking_motion_node,
         # rviz_node 
    ])