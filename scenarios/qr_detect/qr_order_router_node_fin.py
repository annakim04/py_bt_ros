#!/usr/bin/env python3
# qr_order_router_node.py

import json
from pathlib import Path
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped  
import yaml


class QrOrderRouter(Node):
    def __init__(self):
        super().__init__('qr_order_router')

        # YAML 경로 파라미터
        default_config_path = str(Path.home() / 'qr_configs' / 'area_config.yaml')
        self.declare_parameter('area_config_path', default_config_path)
        config_path = self.get_parameter(
            'area_config_path'
        ).get_parameter_value().string_value

        self.get_logger().info(f'[QR ROUTER] Using area config: {config_path}')

        # YAML에서 읽어올 변수, 로드 호출
        self.seoul_areas = []
        self.gyeonggi_areas = []
        self.warehouse_by_region = {}

        self._load_area_config(config_path)

        # 지역별 고정 좌표 Translation + Yaw(deg) 
        self.region_pose_params = {
            'GYEONGGI': {'x': -4.190, 'y': 2.063, 'z': 0.0, 'yaw_deg': 0.0},   
            'SEOUL': {'x': -0.235, 'y': 2.350, 'z': 0.0, 'yaw_deg': 90.0},     
        }

        # /qr_list sub (qr_detector_node subscribe)
        self.qr_list_sub = self.create_subscription(
            String,
            'qr_list',
            self.qr_list_callback,
            10
        )

        # /qr_warehouse pub (물류창고 이름(Str)) 
        self.warehouse_pub = self.create_publisher(
            String,
            '/qr_warehouse',
            10
        )

        # /qr_warehouse_pose pub (물류창고 장소(PoseStamped))
        self.warehouse_pose_pub = self.create_publisher(
            PoseStamped,
            'qr_warehouse_pose',
            10
        )

        self.get_logger().info('[QR ROUTER] Node initialized.')

    # ---------------- YAML 로드 ----------------
    def _load_area_config(self, path_str: str):
        path = Path(path_str)

        if not path.exists():
            self.get_logger().error(f'[QR ROUTER] Config file not found: {path}')
            return

        try:
            with path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'[QR ROUTER] Failed to load YAML: {e}')
            return

        self.seoul_areas = data.get('seoul_areas', [])
        self.gyeonggi_areas = data.get('gyeonggi_areas', [])
        self.warehouse_by_region = data.get('warehouse_by_region', {})

        self.get_logger().info(f'[QR ROUTER] Loaded {len(self.seoul_areas)} seoul_areas.')
        self.get_logger().info(f'[QR ROUTER] Loaded {len(self.gyeonggi_areas)} gyeonggi_areas.')
        self.get_logger().info(f'[QR ROUTER] warehouse_by_region: {self.warehouse_by_region}')

    # ---------------- 주소(도/시) -> region 문자열 ----------------
    def classify_region_by_area_name(self, area_name: str) -> str:
        if area_name in self.seoul_areas:
            return 'SEOUL'
        if area_name in self.gyeonggi_areas:
            return 'GYEONGGI'
        return 'UNKNOWN'

    # ---------------- 콜백: /qr_list 수신 ----------------
    def qr_list_callback(self, msg: String):
        raw = msg.data
        self.get_logger().info(f'[QR ROUTER] Received qr_list: {raw}')

        # 1) JSON 문자열 -> 파이썬 list
        try:
            data_list = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().error('[QR ROUTER] Failed to parse qr_list as JSON. Ignoring.')
            return

        if not isinstance(data_list, list):
            self.get_logger().error('[QR ROUTER] Parsed JSON is not a list. Ignoring.')
            return

        # 2) index 1 -> area_name 정보 읽기
        if len(data_list) <= 1:
            self.get_logger().warn('[QR ROUTER] List length <= 1. No area_name at index 1.')
            return

        area_name = str(data_list[1])
        self.get_logger().info(f'[QR ROUTER] area_name (index 1): {area_name}')

        # 3) area_name -> region
        region = self.classify_region_by_area_name(area_name)

        # 4) region -> 물류창고 이름
        warehouse_name = self.warehouse_by_region.get(region, 'UNKNOWN_WAREHOUSE')

        self.get_logger().info(
            f'[QR ROUTER] area_name="{area_name}", region={region}, warehouse={warehouse_name}'
        )

        # /qr_warehouse pub
        out_msg = String()
        out_msg.data = warehouse_name
        self.warehouse_pub.publish(out_msg)

        # region 해당하는 /qr_warehouse_pose pub
        pose_params = self.region_pose_params.get(region)

        if pose_params is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'  # 필요하면 'odom' 으로 변경

            # 위치
            pose_msg.pose.position.x = pose_params['x']
            pose_msg.pose.position.y = pose_params['y']
            pose_msg.pose.position.z = pose_params['z']

            # ---- yaw(deg) -> quaternion 변환 ----
            yaw_deg = pose_params.get('yaw_deg', 0.0)
            yaw_rad = math.radians(yaw_deg)
            half_yaw = yaw_rad * 0.5

            # (Z축 기준 회전 quaternion (roll=pitch=0 가정))
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = math.sin(half_yaw)
            pose_msg.pose.orientation.w = math.cos(half_yaw)
            # -----------------------------------

            self.warehouse_pose_pub.publish(pose_msg)

            self.get_logger().info(
                f'[QR ROUTER] Published warehouse pose for region={region}: '
                f"pos=({pose_params['x']}, {pose_params['y']}, {pose_params['z']}), "
                f"yaw={yaw_deg} deg"
            )

    def destroy_node(self):
        self.get_logger().info('[QR ROUTER] Shutting down.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QrOrderRouter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
