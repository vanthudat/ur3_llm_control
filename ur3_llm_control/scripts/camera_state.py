#!/usr/bin/env python3
"""Publish only fresh, synchronized RGB-D observations; never use world model poses."""
import json
import time
import uuid
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from ur3_llm_control.perception import detect_blocks, detect_wrist_blocks
from ur3_llm_control.environment import ZONES, distance


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec*1e-9


def array(message, dtype, channels):
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype(dtype).newbyteorder(endian)
    width = message.width * channels
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    return rows[:, :width*dtype.itemsize].copy().view(dtype).reshape(
        (message.height, message.width, channels) if channels > 1 else (message.height, message.width))


class CameraState(Node):
    def __init__(self):
        super().__init__('camera_state')
        self.depth = None
        self.info = None
        self.rgb_received = 0.0
        self.depth_received = 0.0
        self.last_stamp = None
        self.stream_id = str(uuid.uuid4())
        self.frame_id = 0
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(String, '/environment_state', 1)
        self.debug = self.create_publisher(Image, '/table_camera/detections', 10)
        self.wrist_publisher = self.create_publisher(String, '/wrist_camera/observation', 10)
        self.create_subscription(CameraInfo, '/table_camera/camera_info', self.on_info, image_qos)
        self.create_subscription(Image, '/table_camera/depth_image', self.on_depth, image_qos)
        self.create_subscription(Image, '/table_camera/image', self.on_rgb, image_qos)
        self.create_subscription(Image, '/wrist_camera/image', self.on_wrist_rgb, image_qos)
        self.create_subscription(Image, '/wrist_camera/depth_image', self.on_wrist_depth, image_qos)
        self.rgb = None
        self.wrist_rgb = None
        self.wrist_depth = None
        self.wrist_last_stamp = None
        self.wrist_stream_id = str(uuid.uuid4())
        self.wrist_frame_id = 0

    def on_wrist_rgb(self, message):
        self.wrist_rgb = message
        self.process_wrist()

    def on_wrist_depth(self, message):
        if message.encoding == '32FC1':
            self.wrist_depth = message
            self.process_wrist()

    def process_wrist(self):
        message = self.wrist_rgb
        depth_message = self.wrist_depth
        if message is None or depth_message is None:
            return
        if message.encoding not in ('rgb8', 'bgr8'):
            return
        if (message.width != depth_message.width or message.height != depth_message.height or
                abs(stamp(message) - stamp(depth_message)) > 0.001 or
                stamp(message) == self.wrist_last_stamp):
            return
        rgb = array(message, 'u1', 3)
        if message.encoding == 'bgr8':
            rgb = rgb[:, :, ::-1].copy()
        depth = array(depth_message, 'f4', 1)
        counts = detect_wrist_blocks(rgb, depth)
        self.wrist_frame_id += 1
        self.wrist_last_stamp = stamp(message)
        observation = {
            'source': 'wrist_camera_rgb',
            'stamp': stamp(message),
            'observed_at': time.time(),
            'stream_id': self.wrist_stream_id,
            'frame_id': self.wrist_frame_id,
            'objects_visible': list(counts),
            'detections': counts,
        }
        output = String()
        output.data = json.dumps(observation)
        self.wrist_publisher.publish(output)

    def on_info(self, message):
        self.info = message

    def on_depth(self, message):
        if message.encoding == '32FC1':
            self.depth = message
            self.depth_received = time.time()
            self.process()

    def on_rgb(self, message):
        self.rgb = message
        self.rgb_received = time.time()
        self.process()

    def process(self):
        message = self.rgb
        if message is None or self.depth is None or self.info is None:
            return
        if abs(stamp(message)-stamp(self.depth)) > 0.001:
            return
        observed_at = min(self.rgb_received, self.depth_received)
        if time.time() - observed_at > 2.0 or stamp(message) == self.last_stamp:
            return
        if message.width != self.depth.width or message.height != self.depth.height:
            return
        if message.encoding not in ('rgb8', 'bgr8'):
            return
        k = self.info.k
        if k[0] <= 0 or k[4] <= 0:
            return
        rgb = array(message, 'u1', 3)
        if message.encoding == 'bgr8':
            rgb = rgb[:, :, ::-1].copy()
        depth = array(self.depth, 'f4', 1)
        objects = detect_blocks(rgb, depth, (k[0], k[4], k[2], k[5]))
        zones = {zone: [name for name, p in objects.items() if distance(p, center) < 0.055
                        and abs(p[2]-0.740) < 0.040] for zone, center in ZONES.items()}
        complete = len(objects) == 5
        zone_status = {zone: ('occupied' if names else 'free') if complete else 'unknown'
                       for zone, names in zones.items()}
        self.frame_id += 1
        self.last_stamp = stamp(message)
        scene = {'source': 'rgbd_camera', 'stamp': stamp(message),
                 'observed_at': observed_at, 'stream_id': self.stream_id,
                 'frame_id': self.frame_id, 'objects': objects,
                 'zones': zones, 'zone_status': zone_status, 'complete': complete}
        output = String()
        output.data = json.dumps(scene)
        self.publisher.publish(output)
        for index, (name, point) in enumerate(objects.items()):
            cv2.putText(rgb, f'{name}: {point[0]:.3f},{point[1]:.3f},{point[2]:.3f}',
                        (10, 22+index*22), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1)
        debug = Image()
        debug.header = message.header
        debug.height, debug.width = rgb.shape[:2]
        debug.encoding, debug.step = 'rgb8', rgb.shape[1]*3
        debug.data = rgb.tobytes()
        self.debug.publish(debug)
        self.rgb = None


def main():
    rclpy.init()
    node = CameraState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
