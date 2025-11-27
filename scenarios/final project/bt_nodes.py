import math
import rclpy
from rclpy.node import Node as RosNode

from modules.base_bt_nodes import (
    Node,
    Status,
    Sequence,
    BTNodeList as BaseBTNodeList
)
from modules.base_bt_nodes_ros import (
    ActionWithROSAction,
)

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


# ================================
# 좌표 정의 (네 맵에 맞게 수정 필요!)
# ================================
# 예시 값이니까, 실제 map 좌표로 꼭 바꿔줘!
CHARGE_X,  CHARGE_Y,  CHARGE_YAW  = 0.0, 0.0, 0.0   # 충전 장소
PICKUP_X,  PICKUP_Y,  PICKUP_YAW  = 1.0, 0.0, 0.0   # 택배 수령 장소
DELIV_X,   DELIV_Y,   DELIV_YAW   = 2.0, 0.0, 0.0   # 택배 배달 장소


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """2D yaw(라디안) -> Quaternion (z, w만 사용)"""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _build_nav_goal(x: float, y: float, yaw: float, agent) -> NavigateToPose.Goal:
    """주어진 (x, y, yaw)로 Nav2 NavigateToPose.Goal 생성 헬퍼"""
    goal = NavigateToPose.Goal()

    ps = PoseStamped()
    ps.header.frame_id = "map"
    # agent 안에 ros_bridge가 있다고 가정 (WaitForGoal에서 쓰던 패턴)
    ps.header.stamp = agent.ros_bridge.node.get_clock().now().to_msg()

    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    ps.pose.orientation = yaw_to_quaternion(yaw)

    goal.pose = ps
    return goal


# ============================================
# 1) MoveToCharge : 충전 장소로 이동
# ============================================
class MoveToCharge(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        return _build_nav_goal(CHARGE_X, CHARGE_Y, CHARGE_YAW, agent)

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        # Nav2 결과를 세부적으로 보고 싶으면 여기서 처리
        return Status.SUCCESS


# ============================================
# 2) MoveToPickup : 택배 수령 장소로 이동
# ============================================
class MoveToPickup(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        return _build_nav_goal(PICKUP_X, PICKUP_Y, PICKUP_YAW, agent)

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        return Status.SUCCESS


# ============================================
# 3) MoveToDelivery : 택배 배달 장소로 이동
# ============================================
class MoveToDelivery(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        return _build_nav_goal(DELIV_X, DELIV_Y, DELIV_YAW, agent)

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        return Status.SUCCESS


# ============================================
# 4) ReceiveParcel : 택배 수령 (더미, 항상 성공)
# ============================================
class ReceiveParcel(Node):
    def __init__(self, name, agent, wait_sec: float = 1.0):
        super().__init__(name)
        self.wait_sec = wait_sec
        self._start_time = None

    async def run(self, agent, blackboard):
        import time

        # 처음 들어왔을 때 시작 시간 기록
        if self._start_time is None:
            self._start_time = time.time()
            self.status = Status.RUNNING
            return self.status

        # wait_sec 만큼 기다린 뒤 SUCCESS 반환
        if time.time() - self._start_time >= self.wait_sec:
            self._start_time = None
            print("[BT] ReceiveParcel: 택배 수령 완료 (더미 성공)")
            self.status = Status.SUCCESS
            return self.status

        self.status = Status.RUNNING
        return self.status


# ============================================
# 5) DropoffParcel : 택배 배달 (더미, 항상 성공)
# ============================================
class DropoffParcel(Node):
    def __init__(self, name, agent, wait_sec: float = 1.0):
        super().__init__(name)
        self.wait_sec = wait_sec
        self._start_time = None

    async def run(self, agent, blackboard):
        import time

        if self._start_time is None:
            self._start_time = time.time()
            self.status = Status.RUNNING
            return self.status

        if time.time() - self._start_time >= self.wait_sec:
            self._start_time = None
            print("[BT] DropoffParcel: 택배 배달 완료 (더미 성공)")
            self.status = Status.SUCCESS
            return self.status

        self.status = Status.RUNNING
        return self.status


# ============================================
# BT Node Registration
# ============================================
class BTNodeList:
    # Sequence / Fallback 등 기본 CONTROL_NODES는 그대로 재사용
    CONTROL_NODES = BaseBTNodeList.CONTROL_NODES

    # 우리가 새로 정의한 Action/Leaf 노드들 이름
    ACTION_NODES = [
        "MoveToCharge",
        "MoveToPickup",
        "MoveToDelivery",
        "ReceiveParcel",
        "DropoffParcel",
    ]

    CONDITION_NODES = []
    DECORATOR_NODES = []
