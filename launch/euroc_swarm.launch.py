"""Three robots from EuRoC MH_01-03: replay -> OpenCV 5 front end -> RTAB-Map stereo odometry -> Swarm-SLAM.

ros2 launch launch/euroc_swarm.launch.py run:=<name> [rate:=0.5 skip:=2 robots:=3]
Outputs go to ~/runs/<run>/.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CV5_PYTHON = os.path.expanduser("~/cv5env/bin/python")
DATA = os.path.expanduser("~/data/euroc")
SEQUENCES = ["MH_01_easy", "MH_02_easy", "MH_03_medium"]


def setup(context):
    run = context.launch_configurations["run"]
    rate = context.launch_configurations["rate"]
    skip = context.launch_configurations["skip"]
    robots = int(context.launch_configurations["robots"])
    out = os.path.expanduser(f"~/runs/{run}")
    os.makedirs(out, exist_ok=True)
    # `python -m` from cwd=REPO finds our package; don't touch PYTHONPATH, ROS puts rclpy there.
    py = dict(cwd=REPO, output="screen")

    # Images are stamped in frame "camera_link" itself, i.e. the base frame IS the optical frame.
    # With a rotated camera_link->camera_optical TF, Swarm-SLAM's inter-robot loop closures came
    # out in the optical frame while odometry was in camera_link, so whole robots were merged
    # rotated by exactly that 120 deg (run euroc3_r025_b). Its own KITTI launch uses identity too.
    actions = [ExecuteProcess(cmd=[CV5_PYTHON, "-m", "coordinator.recorder", "--robots", str(robots),
                                   "--out", f"{out}/events.jsonl"], **py)]
    for i in range(robots):
        ns, mav0 = f"/r{i}", f"{DATA}/{SEQUENCES[i]}/mav0"
        cam = f"{ns}/stereo_camera"
        actions += [
            ExecuteProcess(cmd=[CV5_PYTHON, "-m", "frontend.node", "--robot", str(i),
                                "--calib", f"{REPO}/calib/euroc_cam{i}.yaml",
                                "--log", f"{out}/quality_r{i}.jsonl", "--frames-dir", f"{out}/frames/r{i}"], **py),
            Node(package="rtabmap_odom", executable="stereo_odometry", namespace=ns,
                 parameters=[{"frame_id": "camera_link", "publish_tf": False, "approx_sync": False,
                              "wait_for_transform": 0.2, "qos": 1, "qos_camera_info": 1}],
                 remappings=[("left/image_rect", f"{cam}/left/image_rect_color"),
                             ("right/image_rect", f"{cam}/right/image_rect_color"),
                             ("left/camera_info", f"{cam}/left/camera_info"),
                             ("right/camera_info", f"{cam}/right/camera_info"),
                             ("odom", f"{ns}/odom")],
                 # Same odometry flags as cslam_experiments' KITTI stereo launch.
                 arguments=["--Vis/MinInliers", "8", "--Odom/KeyFrameThr", "0.0",
                            "--Odom/VisKeyFrameThr", "0.0", "--Odom/ScanKeyFrameThr", "0.0",
                            "--ros-args", "--log-level", "warn"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory("cslam_experiments"), "launch", "cslam", "cslam_stereo.launch.py")),
                launch_arguments={"config_path": f"{REPO}/config/", "config_file": "cslam_stereo.yaml",
                                  "robot_id": str(i), "namespace": ns, "max_nb_robots": str(robots)}.items()),
            ExecuteProcess(cmd=[CV5_PYTHON, "-m", "frontend.euroc_replay", "--robot", str(i),
                                "--mav0", mav0, "--rate", rate, "--skip", skip,
                                "--stamps", f"{out}/stamps_r{i}.csv",
                                "--perturb", context.launch_configurations["perturb"]], **py),
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("run", default_value="test"),
        DeclareLaunchArgument("rate", default_value="0.5"),
        DeclareLaunchArgument("skip", default_value="2"),
        DeclareLaunchArgument("robots", default_value="3"),
        DeclareLaunchArgument("perturb", default_value="", description="failure injection, see euroc_replay.py"),
        OpaqueFunction(function=setup),
    ])
