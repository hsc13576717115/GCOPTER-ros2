#!/usr/bin/env python3
"""Real ROS 2 process/topic tests. No fake planner or trajectory playback shortcuts."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

os.environ.setdefault('ROS_DOMAIN_ID', '83')
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String, Float64
from visualization_msgs.msg import Marker

LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
ARTIFACTS = Path(os.environ.get('GCOPTER_ARTIFACT_DIR', 'artifacts')).resolve()


def stop_process(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
            raise AssertionError('Child process did not shut down on SIGINT')


def cloud(points, padded=False):
    msg = PointCloud2()
    msg.header.frame_id = 'odom'
    msg.height = 1
    msg.width = len(points)
    offsets = (8, 0, 4) if padded else (0, 4, 8)
    msg.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                  for n, o in zip(('x', 'y', 'z'), offsets)]
    msg.point_step = 16 if padded else 12
    if padded and len(points) and len(points) % 2 == 0:
        msg.height = 2
        msg.width //= 2
    msg.row_step = msg.width * msg.point_step + (8 if padded else 0)
    msg.is_bigendian = padded
    msg.is_dense = True
    data = bytearray(msg.row_step * msg.height)
    for i, point in enumerate(points):
        row, col = divmod(i, msg.width)
        for value, offset in zip(point, offsets):
            struct.pack_into('>f' if padded else '<f', data,
                             row * msg.row_step + col * msg.point_step + offset, value)
    msg.data = bytes(data)
    return msg


class Demo:
    def __init__(self, name, attach=False):
        self.node = rclpy.create_node('gcopter_verifier_' + name)
        self.statuses = []
        self.markers = {}
        self.values = {n: [] for n in ('speed', 'total_thrust', 'tilt_angle', 'body_rate')}
        self.subscriptions = [self.node.create_subscription(String, '/planner/status', self.on_status, LATCHED)]
        for topic in ('route', 'waypoints', 'mesh', 'edge', 'trajectory', 'spheres'):
            self.subscriptions.append(self.node.create_subscription(
                Marker, '/visualizer/' + topic, lambda m, t=topic: self.on_marker(t, m), LATCHED))
        for topic in self.values:
            self.subscriptions.append(self.node.create_subscription(
                Float64, '/visualizer/' + topic, lambda m, t=topic: self.values[t].append(m.data), 100))
        self.goals = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.maps = None if attach else self.node.create_publisher(PointCloud2, '/voxel_map', LATCHED)
        self.process = None
        self.log = None
        self.dilate = .5 if attach else .25
        self.zmax = 5. if attach else 3.
        if not attach:
            ARTIFACTS.mkdir(parents=True, exist_ok=True)
            self.log = (ARTIFACTS / (name + '.log')).open('w')
            binary = Path(get_package_prefix('gcopter')) / 'lib/gcopter/global_planning'
            self.process = subprocess.Popen([str(binary), '--ros-args', '-p', 'MapBound:=[-5.0,5.0,-5.0,5.0,0.0,3.0]',
                '-p', 'DilateRadius:=0.25', '-p', 'TimeoutRRT:=0.5', '-p', 'PlanningSeed:=42'],
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.wait(lambda: self.goals.get_subscription_count() > 0, 15, 'goal subscription')

    def on_status(self, msg):
        self.statuses.append(msg.data)
        print(msg.data, flush=True)

    def on_marker(self, topic, msg):
        if msg.action == Marker.DELETEALL:
            self.markers[topic] = {}
        elif msg.action == Marker.ADD:
            assert all(math.isfinite(v) for p in msg.points for v in (p.x, p.y, p.z))
            self.markers.setdefault(topic, {})[(msg.ns, msg.id)] = msg

    def wait(self, condition, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.05)
            if self.process is not None:
                assert self.process.poll() is None, 'Planner exited; see log'
            if condition():
                return
        raise AssertionError(f'Timeout: {label}; statuses={self.statuses[-6:]}')

    def wait_status(self, prefix, start=0, timeout=15):
        self.wait(lambda: any(x.startswith(prefix) for x in self.statuses[start:]), timeout, prefix)

    def send(self, point):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(point[0]), float(point[1])
        z = (point[2] - self.dilate) / (self.zmax - 2 * self.dilate)
        assert 0 <= z <= 1
        msg.pose.orientation.z = z
        msg.pose.orientation.w = math.sqrt(1-z*z)
        self.goals.publish(msg)

    def reject(self, point, expected='REJECTED'):
        start = len(self.statuses)
        self.send(point)
        self.wait_status(expected, start)

    def set_map(self, points, padded=False):
        start = len(self.statuses)
        self.maps.publish(cloud(points, padded))
        self.wait_status('MAP_READY', start)

    def run(self, a=(-3, 0, 1.5), b=(3, 0, 1.5)):
        start = len(self.statuses)
        before = {n: len(v) for n,v in self.values.items()}
        self.send(a)
        self.wait_status('START_ACCEPTED', start)
        self.send(b)
        self.wait(lambda: any(s.startswith(('SUCCESS', 'FAILED')) for s in self.statuses[start:]), 30, 'planning result')
        assert not any(s.startswith('FAILED') for s in self.statuses[start:]), self.statuses[start:]
        self.wait_status('COMPLETED', start, 90)
        self.wait(lambda: ('spheres', 0) in self.markers.get('spheres', {}) and
                  math.dist((self.markers['spheres'][('spheres',0)].points[0].x,
                             self.markers['spheres'][('spheres',0)].points[0].y,
                             self.markers['spheres'][('spheres',0)].points[0].z), b) <= .001, 3, 'final marker')
        for topic in ('route', 'trajectory', 'mesh', 'edge'):
            assert any(m.points for m in self.markers.get(topic, {}).values()), topic
        for name, values in self.values.items():
            samples = values[before[name]:]
            assert len(samples) >= 5 and all(math.isfinite(x) for x in samples), name
        assert any(s.startswith('VALIDATED') for s in self.statuses[start:])
        return self.statuses[start:]

    def close(self):
        try:
            if self.process is not None:
                stop_process(self.process)
                assert self.process.returncode == 0, self.process.returncode
        finally:
            if self.log:
                self.log.close()
            self.node.destroy_node()


def obstacle_points(wall=False):
    return [(x*.25+.125, y*.25+.125, z*.25+.125)
            for x in (-1, 0) for y in (range(-20,20) if wall else range(-3,3)) for z in range(12)]


def scenario_tests():
    records = {}
    for scene in ('obstacle', 'empty', 'unreachable'):
        demo = Demo(scene)
        try:
            demo.reject((-3,0,1.5), 'REJECTED map not ready')
            invalid = cloud([(0,0,1)])
            invalid.fields = []
            start = len(demo.statuses)
            demo.maps.publish(invalid)
            demo.wait_status('INVALID_MAP', start)
            points = [] if scene=='empty' else obstacle_points(scene=='unreachable')
            demo.set_map(points, padded=scene=='obstacle')
            demo.reject((-5.01,0,1.5))
            if points:
                demo.reject((0,0,1.5))
            if scene=='unreachable':
                start=len(demo.statuses);demo.send((-3,0,1.5));demo.wait_status('START_ACCEPTED',start)
                demo.reject((-3,0,1.5),'REJECTED identical')
                demo.send((3,0,1.5));demo.wait_status('FAILED path search',start)
                records[scene]=demo.run((-3,-2,1.5),(-3,2,1.5))
            else:
                records[scene]=demo.run()
                records[scene+'_repeat']=demo.run((3,0,1.5),(-3,0,1.5))
            # A subscriber created after completion must receive retained map and trajectory.
            late={}
            subscriptions=[demo.node.create_subscription(PointCloud2,'/voxel_map',lambda m:late.update(map=True),LATCHED),
                demo.node.create_subscription(Marker,'/visualizer/trajectory',lambda m:late.update(trajectory=True) if m.action==Marker.ADD and m.points else None,LATCHED)]
            demo.wait(lambda:all(late.get(k) for k in ('map','trajectory')),5,'late subscription')
            for sub in subscriptions:
                demo.node.destroy_subscription(sub)
        finally:
            demo.close()
    return records


def map_tests():
    node=rclpy.create_node('map_verifier')
    results={}
    binary=Path(get_package_prefix('mockamap'))/'lib/mockamap/mockamap_node'
    for kind in range(1,5):
        received=[]
        topic='/verify_map_'+str(kind)
        sub=node.create_subscription(PointCloud2,topic,received.append,LATCHED)
        with (ARTIFACTS/f'map_{kind}.log').open('w') as log:
            proc=subprocess.Popen([str(binary),'--ros-args','-r','mock_map:='+topic,'-p',f'type:={kind}',
                '-p','x_length:=10','-p','y_length:=10','-p','z_length:=3','-p','resolution:=0.5'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                end=time.monotonic()+20
                while not received and time.monotonic()<end:
                    assert proc.poll() is None, f'map type {kind} exited'
                    rclpy.spin_once(node,timeout_sec=.1)
                assert received and received[0].width>0 and received[0].header.frame_id=='odom',kind
                results[str(kind)]=received[0].width*received[0].height
            finally:
                stop_process(proc)
                assert proc.returncode==0,proc.returncode
                node.destroy_subscription(sub)
    node.destroy_node()
    return results


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--attach',action='store_true',help='Validate an already running original-map demo')
    parser.add_argument('--start',nargs=3,type=float,default=[-15,-15,1.5])
    parser.add_argument('--goal',nargs=3,type=float,default=[15,15,1.5])
    args=parser.parse_args()
    ARTIFACTS.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    try:
        if args.attach:
            demo=Demo('attached',attach=True)
            try:
                demo.wait(lambda: any(s.startswith(('MAP_READY', 'SUCCESS', 'COMPLETED', 'FAILED')) for s in demo.statuses), 15, 'initialized planner')
                result={'first':demo.run(args.start,args.goal),'second':demo.run(args.goal,args.start)}
            finally:
                demo.close()
        else:
            result={'scenarios':scenario_tests(),'map_types':map_tests()}
        (ARTIFACTS/('attached_results.json' if args.attach else 'verification.json')).write_text(json.dumps(result,indent=2))
        print('PASS: complete planning, playback, finite telemetry and recovery',flush=True)
    finally:
        rclpy.shutdown()


if __name__=='__main__':
    main()
