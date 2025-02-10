import os

def generate_robot_description_pkg(export_path: str, robot_name: str):
    package_name = f'{robot_name}_description'
    pkg_dir = f'{export_path}/{package_name}'
    os.makedirs(pkg_dir, exist_ok=True)
    create_launch_file(export_path=export_path, robot_name=robot_name, package_name=package_name)
    create_cmake_file(export_path=export_path, package_name=package_name)
    create_package_xml_file(export_path=export_path, package_name=package_name)

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
  DIRECTORY src launch rviz
  DESTINATION share/${{PROJECT_NAME}}
)

ament_package()'''
    with open(f'{export_path}/{package_name}/CMakeLists.txt', 'w') as file:
        file.write(cmake_file_string.format(package_name=package_name))


def create_launch_file(export_path: str, robot_name: str, package_name: str):
    launch_file_string = '''import launch
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
    os.makedirs(f'{export_path}/{package_name}/launch', exist_ok=True)
    with open(f'{export_path}/{package_name}/launch/display.launch.py', 'w') as file:
        file.write(launch_file_string.format(robot_name=robot_name, package_name=package_name))