import importlib.util
import os
from pathlib import Path


def test_ros2_demo(tmp_path):
    os.environ['GCOPTER_ARTIFACT_DIR'] = str(tmp_path)
    source = Path(__file__).resolve().parents[1] / 'scripts' / 'verify_demo.py'
    spec = importlib.util.spec_from_file_location('verify_demo', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.ARTIFACTS.mkdir(parents=True, exist_ok=True)
    module.rclpy.init()
    try:
        import json
        report = {'scenarios': module.scenario_tests(), 'map_types': module.map_tests()}
        (tmp_path / 'verification.json').write_text(json.dumps(report, indent=2))
    finally:
        module.rclpy.shutdown()


def test_invalid_parameters():
    import subprocess
    from ament_index_python.packages import get_package_prefix
    binary = Path(get_package_prefix('gcopter')) / 'lib/gcopter/global_planning'
    result = subprocess.run([str(binary), '--ros-args', '-p', 'VoxelWidth:=0.0'],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 1
    assert 'Invalid planner parameters' in result.stderr


def test_launch_shutdown(tmp_path):
    import signal
    import subprocess
    import time
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    env = dict(os.environ, ROS_DOMAIN_ID='84')
    # Separate domain for the launch lifecycle check; initialize only after setting it.
    previous = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = '84'
    proc = None
    rclpy.init()
    node = rclpy.create_node('launch_lifecycle_test')
    states = []
    sub = node.create_subscription(String, '/planner/status', lambda m: states.append(m.data),
                                  QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        with (tmp_path / 'launch.log').open('w') as output:
            proc = subprocess.Popen(['ros2', 'launch', 'gcopter', 'global_planning.launch.py',
                                     'rviz:=false', 'plot:=false'], env=env,
                                    stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 20
            while 'MAP_READY' not in states and time.monotonic() < deadline:
                assert proc.poll() is None
                rclpy.spin_once(node, timeout_sec=.1)
            assert 'MAP_READY' in states
            # Signal launch itself: it must shut down all its children.
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=15)
            assert proc.returncode == 0
            deadline = time.monotonic() + 5
            while node.count_publishers('/planner/status') and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            assert node.count_publishers('/planner/status') == 0
            assert node.count_publishers('/voxel_map') == 0
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        if previous is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous
