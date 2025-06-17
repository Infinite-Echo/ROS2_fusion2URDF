import os
import adsk, adsk.core, adsk.fusion, traceback

def generate_robot_description_pkg(export_path: str, robot_name: str, app: adsk.core.Application, classic: bool = False):
    package_name = f'{robot_name}_description'
    pkg_dir = f'{export_path}/{package_name}'
    os.makedirs(pkg_dir, exist_ok=True)
    create_launch_file(export_path=export_path, robot_name=robot_name, package_name=package_name, classic=classic)
    create_cmake_file(export_path=export_path, package_name=package_name)
    create_rviz_config_file(export_path=export_path, robot_name=robot_name, package_name=package_name)
    create_world_file(export_path=export_path, package_name=package_name)
    create_package_xml_file(export_path=export_path, package_name=package_name)

def create_rviz_config_file(export_path: str, robot_name: str, package_name: str):
    rviz_config_str = '''Panels:
  - Class: rviz_common/Displays
    Help Height: 0
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /TF1
        - /TF1/Frames1
        - /TF1/Tree1
      Splitter Ratio: 0.5833333134651184
    Tree Height: 865
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: nav2_rviz_plugins/Navigation 2
    Name: Navigation 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: ""
      Mass Properties:
        Inertia: false
        Mass: false
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
    - Class: nav2_rviz_plugins/GoalTool
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: false
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.7853981852531433
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 0.7853981852531433
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1016
  Hide Left Dock: false
  Hide Right Dock: false
  Navigation 2:
    collapsed: false
  QMainWindow State: 000000ff00000000fd00000004000000000000016a0000039efc020000000bfb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d0000039e000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb00000018004e0061007600690067006100740069006f006e002000320000000237000001a40000013900fffffffb0000001e005200650061006c00730065006e0073006500430061006d00650072006100000002c6000000c10000000000000000fb0000001e005200650061006c00730065006e0073006500430061006d006500720061010000033a000000a10000000000000000000000010000010f0000039efc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d0000039e000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d00650100000000000004500000000000000000000004c50000039e00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1866
  X: 54
  Y: 27
'''
    os.makedirs(f'{export_path}/{package_name}/rviz', exist_ok=True)
    with open(f'{export_path}/{package_name}/rviz/{robot_name}.rviz', 'w') as file:
        file.write(rviz_config_str)

def create_package_xml_file(export_path: str, package_name: str):
    package_xml_string = '''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>'''
    with open(f'{export_path}/{package_name}/package.xml', 'w') as file:
        file.write(package_xml_string.format(package_name=package_name))

def create_cmake_file(export_path: str, package_name: str):
    cmake_file_string = '''cmake_minimum_required(VERSION 3.8)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY src launch rviz worlds
  DESTINATION share/${{PROJECT_NAME}}
)

ament_package()'''
    with open(f'{export_path}/{package_name}/CMakeLists.txt', 'w') as file:
        file.write(cmake_file_string.format(package_name=package_name))

def create_world_file(export_path: str, package_name: str):
    world_file_string = '''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="base_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!--light-->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
      <pose>0 0 0 0 -0 0</pose>
    </model>

  </world>
</sdf>'''
    os.makedirs(f'{export_path}/{package_name}/worlds', exist_ok=True)
    with open(f'{export_path}/{package_name}/worlds/base_world.sdf', 'w') as file:
        file.write(world_file_string)

def create_launch_file(export_path: str, robot_name: str, package_name: str, classic: bool = False):
    if classic:
        launch_file_string = create_gazebo_classic_launch_file()
    else:
        launch_file_string = create_gazebo_launch_file()
    os.makedirs(f'{export_path}/{package_name}/launch', exist_ok=True)
    with open(f'{export_path}/{package_name}/launch/display.launch.py', 'w') as file:
        file.write(launch_file_string.format(robot_name=robot_name, package_name=package_name))

def create_gazebo_classic_launch_file() -> str:
    launch_file_str = '''import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="{package_name}").find(
        "{package_name}"
    )
    default_model_path = os.path.join(pkg_share, "src/urdf/{robot_name}.xacro")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/{package_name}_config.rviz")

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {{"robot_description": Command(["xacro ", LaunchConfiguration("model")])}}
        ],
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "{robot_name}", "-topic", "robot_description"],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            launch.actions.ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    "--pause",
                ],
                output="screen",
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            spawn_entity,
            rviz_node,
        ]
    )
    '''
    return launch_file_str

def create_gazebo_launch_file() -> str:
    #Previously called Gazebo Ignition
    launch_file_str = '''import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_share = get_package_share_directory("{package_name}")
    default_model_path = os.path.join(pkg_share, "src/urdf/{robot_name}.xacro")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    default_world_path = os.path.join(pkg_share, "worlds/base_world.sdf")

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={{"gz_args": "-r " + default_world_path}}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_desc = Command(["xacro ", LaunchConfiguration("model")])
    params = {{"use_sim_time": True, "robot_description": robot_desc}}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[params],
        arguments=[],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[params],
    )

    # Visualize in RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_share, "rviz", "{robot_name}.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "{robot_name}",
            "-x",
            "0.0",
            "-z",
            "1.0",
            "-Y",
            "0.0",
            "-topic",
            "/robot_description",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Clock (IGN -> ROS2)
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            # Joint states (IGN -> ROS2)
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            # CMD VEL (ROS2 -> IGN)
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # ODOM (IGN -> ROS2)
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            # LIDAR (IGN -> ROS2)
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            # # LIDAR POINTCLOUD (IGN -> ROS2)
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            # TF (IGN -> ROS2)
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
        ],
        remappings=[
            ("/world/base_world/model/{robot_name}/joint_state", "joint_states"),
            ("/model/{robot_name}/cmd_vel", "cmd_vel"),
            ("/model/{robot_name}/odometry", "odom"),
            ("/model/{robot_name}/tf", "tf"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=default_world_path,
                description="Absolute path to world file",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            gz_sim,
            spawn,
            bridge,
            robot_state_publisher,
            rviz,
        ]
    )
    '''
    return launch_file_str