from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission_config = LaunchConfiguration("mission_config_path")
    pub_world_pose = LaunchConfiguration("pub_world_pose")
    enable_offboard_control = LaunchConfiguration("enable_offboard_control")
    auto_start_mission = LaunchConfiguration("auto_start_mission")
    takeoff_altitude = LaunchConfiguration("takeoff_altitude")

    default_config = PathJoinSubstitution(
        [FindPackageShare("drone_mission"), "config" ,"ground_mission.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_config_path",
                default_value=default_config,
                description="任务 YAML 配置文件路径。",
            ),
            DeclareLaunchArgument(
                "pub_world_pose",
                default_value="false",
                description="是否发布转换到 world_body 下的 /current_world_body_pos。",
            ),
            DeclareLaunchArgument(
                "enable_offboard_control",
                default_value="false",
                description="是否启动会切 OFFBOARD 并解锁的任务控制节点。",
            ),
            DeclareLaunchArgument(
                "auto_start_mission",
                default_value="false",
                description="任务控制器初始化完成后是否自动开始任务。",
            ),
            DeclareLaunchArgument(
                "takeoff_altitude",
                default_value="0.5",
                description="默认起飞高度，单位 m。",
            ),
            Node(
                package="drone_control",
                executable="tf_bridge_node",
                name="tf_bridge_node",
                output="screen",
                parameters=[{"pub_world_pose": pub_world_pose}],
            ),
            Node(
                package="drone_control",
                executable="mission_controller_node",
                name="mission_controller_node",
                output="screen",
                condition=IfCondition(enable_offboard_control),
                parameters=[{
                    "mission_config_path": mission_config,
                    "auto_start_mission": auto_start_mission,
                    "takeoff_altitude": takeoff_altitude,
                }],
            ),
            Node(
                package="drone_control",
                executable="route_comm_node",
                name="route_comm_node",
                output="screen",
                parameters=[{
                    "mission_config_path": mission_config,
                    "enable_offboard_control": enable_offboard_control,
                }],
            ),
        ]
    )
