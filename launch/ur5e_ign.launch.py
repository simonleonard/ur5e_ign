from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess
)

from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context, *args, **kwargs):
    
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ur5e_ign"),
         "config",
         "controller_manager.yaml", ]
    )

    initial_positions_file = PathJoinSubstitution(
        [FindPackageShare("ur5e_ign"), "config", "initial_positions.yaml"]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur5e_ign"), "rviz", "view_robot.rviz"]
    )

    # Combine the robot and gripper description commands into one
    combined_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=ur",
            " ",
            "ur_type:=ur5e",
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "use_fake_hardware:=false",
            " ",
            "sim_ignition:=true",
            " ",
            "initial_positions_file:=",
            initial_positions_file,
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )

    # Create the combined robot description
    robot_description = {"robot_description": combined_description_content}
    
    # The actual simulation is a ROS2-control system interface.
    # Start that with the usual ROS2 controller manager mechanisms.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, initial_joint_controllers],
        # prefix="screen -d -m gdb -command=/home/stefan/.gdb_debug_config --ex run --args",  # noqa E501
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("motion_control_handle/target_frame", "target_frame"),
            ("cartesian_motion_controller/target_frame", "target_frame"),
            ("cartesian_compliance_controller/target_frame", "target_frame"),
            ("cartesian_force_controller/target_wrench", "target_wrench"),
            ("cartesian_compliance_controller/target_wrench", "target_wrench"),
            ("cartesian_force_controller/ft_sensor_wrench", "ft_sensor_wrench"),
            ("cartesian_compliance_controller/ft_sensor_wrench", "ft_sensor_wrench"),
        ],
    )

    ur_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )

    ur_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )
    
    # Convenience function for easy spawner construction
    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name] + [a for a in args],
            remappings=[
                ("~/robot_description", "/robot_description")
            ]
        )

    # Active controllers
    active_list = ["joint_state_broadcaster", "cartesian_motion_controller"]
    active_spawners = [controller_spawner(controller) for controller in active_list]

    # Inactive controllers
    inactive_list = [
        #"cartesian_compliance_controller",
        #"cartesian_force_controller",
        #"cartesian_motion_controller",
        "joint_trajectory_controller",
        #"motion_control_handle",
        #"invalid_cartesian_compliance_controller",
        #"invalid_cartesian_force_controller",
    ]
    state = "--inactive"
    inactive_spawners = [
        controller_spawner(controller, state) for controller in inactive_list
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=[]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "-f", "world"],
    )

    rqt_joint_trajectory_controller = Node(
        package="rqt_gui",
        executable="rqt_gui",
        output="screen",
    )

    ignition_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -v 3 -r empty.sdf"}.items(),
    )

    ignition_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            combined_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
        ],
    )

    teleop_term = ExecuteProcess(
        cmd=["xterm", "-e ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/teleop_cmd"],
        shell=True,
    )

    
    nodes_to_start = [
        robot_state_publisher,
        rviz_node,
        ignition_launch_description,
        ignition_spawn_robot,
        teleop_term,
        #rqt_joint_trajectory_controller,
    ] + active_spawners + inactive_spawners

    return nodes_to_start


def generate_launch_description():
    # Declarations
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e", 
            description="Type/Series of used Robot",
        ),
        DeclareLaunchArgument(
            "safety_limits", 
            default_value="true",
            description="Enable the safety limits controller if true",
        ), 
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        ),
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_robot_driver",
            description="Package with the controller's configuration in 'config' folder. The package can be ur_robot_driver but I copied it because I want to include the gripper",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the conroller's configuration.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with custom/default robot URDF/XACRO files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="The URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]),
            description="The calibration configuration of the actual robot used.", 
            # I might need to run the calibration code recommended on the ur_robot_driver page.
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
