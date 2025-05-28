import xacro
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):

    urdf_path = PathJoinSubstitution([FindPackageShare("warthog_description"), "urdf", "warthog.urdf.xacro"])

    robot_description = {"robot_description": xacro.process_file(urdf_path.perform(context)).toxml()}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name='joint_state_publisher',
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[robot_description], 
    )


    # spawn_urdf_node = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     arguments=[
    #         "-name", "warthog_description",
    #         "-topic", "robot_description",
    #         "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0"  # Initial spawn position
    #     ],
    #     output="screen",
    #     parameters=[
    #         {'use_sim_time': True},
    #     ]
    # )
    
    gazebo_world_path = PathJoinSubstitution([
        FindPackageShare("warthog_description"),  # o el paquete donde tengas el mundo
        "worlds",
        "groundPlane.sdf"
    ])

    gazebo_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[gazebo_world_path.perform(context)],
        output="screen"
    )
    return [
        gazebo_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        # spawn_urdf_node
    ]

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
