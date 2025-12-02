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
from std_msgs.msg import String


def deg(d: float) -> float:
    return math.radians(d)

# ================================
# 좌표 정의 (네 맵에 맞게 수정 필요!)
# ================================
# 예시 값이니까, 실제 map 좌표로 꼭 바꿔줘!
CHARGE_X,  CHARGE_Y,  CHARGE_YAW  = -4.198, 0.2, deg(89.274)   # 충전 장소
PICKUP_X,  PICKUP_Y,  PICKUP_YAW  = -6.326, 3.209, deg(-78.415)   # 택배 수령 장소
DELIV_X,   DELIV_Y,   DELIV_YAW   = -4.19, 2.063, deg(-6.810)   # 택배 배달 장소


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
        # WaitForQRPose 노드가 저장해둔 PoseStamped 를 가져옴
        pose: PoseStamped = blackboard.get("qr_target_pose", None)

        if pose is None:
            agent.ros_bridge.node.get_logger().warn(
                "[MoveToDelivery] qr_target_pose is None (QR 인식 정보가 아직 없음)"
            )
            return None  # Goal이 None이면 이 액션 노드는 FAILURE 취급됨

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        return Status.SUCCESS



# ============================================
# 4) ReceiveParcel : 택배 수령 (더미, 항상 성공)
# ============================================
class ReceiveParcel(Node):
    """
    /limo/button 토픽이 'pressed' 가 되었을 때 SUCCESS.
    일정 시간 동안 안 눌리면 FAILURE (수령 안 함).
    그 전까지는 RUNNING.
    """
    TIMEOUT_SEC = 30.0  # 원하는 시간으로 수정

    def __init__(self, name, agent):
        super().__init__(name)
        self.agent = agent

        self._button_state = "release"
        self._node: RosNode = agent.ros_bridge.node

        self._start_time = self._node.get_clock().now()  # 시작 시각 저장

        self._sub = self._node.create_subscription(
            String,
            "/limo/button",
            self._callback,
            10
        )

    def _callback(self, msg: String):
        self._button_state = msg.data.strip()
        self._node.get_logger().info(
            f"[ReceiveParcel] button state: {self._button_state}"
        )

    async def run(self, agent, blackboard):
        # 1) 버튼이 pressed 되면 수령 성공
        if self._button_state.lower() == "pressed":
            self.status = Status.SUCCESS
            self._node.get_logger().info(
                "[ReceiveParcel] 버튼 pressed → 택배 수령 완료"
            )
            return self.status

        # 2) 아직 안 눌렸으면, 타임아웃 체크
        now = self._node.get_clock().now()
        elapsed = (now.sec + now.nanosec * 1e-9) - (self._start_time.sec + self._start_time.nanosec * 1e-9)

        if elapsed > self.TIMEOUT_SEC:
            # 일정 시간 동안 안 눌렸으면 FAILURE → "수령 안 함"으로 판단
            self.status = Status.FAILURE
            self._node.get_logger().warn(
                f"[ReceiveParcel] {self.TIMEOUT_SEC}초 동안 입력 없음 → 수령 실패"
            )
        else:
            # 아직 타임아웃 전이면 계속 기다림
            self.status = Status.RUNNING

        return self.status




# ============================================
# 5) DropoffParcel : 택배 배달 (더미, 항상 성공)
# ============================================
class DropoffParcel(Node):
    """
    /limo/button 토픽이 'release' 인 상태여야 SUCCESS.
    (즉, 손을 떼서 놓은 상태)
    """
    def __init__(self, name, agent):
        super().__init__(name)
        self.agent = agent

        self._button_state = "release"
        self._node: RosNode = agent.ros_bridge.node

        self._sub = self._node.create_subscription(
            String,
            "/limo/button",
            self._callback,
            10
        )

    def _callback(self, msg: String):
        self._button_state = msg.data.strip()
        self._node.get_logger().info(
            f"[DropoffParcel] button state: {self._button_state}"
        )

    async def run(self, agent, blackboard):
        # 버튼이 release 상태여야 배달 완료
        if self._button_state.lower() == "release":
            self.status = Status.SUCCESS
            self._node.get_logger().info(
                "[DropoffParcel] 버튼 release → 택배 배달 완료"
            )
        else:
            self.status = Status.RUNNING

        return self.status


# ============================================
# 6) WaitForQRPose : /qr_warehouse_pose 들어올 때까지 기다리는 노드
# ============================================
class WaitForQRPose(Node):
    """
    /qr_warehouse_pose (PoseStamped)를 한 번 받을 때까지 RUNNING.
    받으면 blackboard['qr_target_pose'] 에 저장하고 SUCCESS.
    """

    def __init__(self, name, agent):
        super().__init__(name)
        self.agent = agent
        self._pose = None

        # agent.ros_bridge.node 를 rclpy Node 처럼 사용
        self._node: RosNode = agent.ros_bridge.node
        self._sub = self._node.create_subscription(
            PoseStamped,
            "/qr_warehouse_pose",
            self._callback,
            10
        )

    def _callback(self, msg: PoseStamped):
        self._pose = msg
        self._node.get_logger().info(
            f"[WaitForQRPose] got pose from QR: "
            f"x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}"
        )

    async def run(self, agent, blackboard):
        # 아직 pose가 안 들어왔으면 계속 RUNNING
        if self._pose is None:
            self.status = Status.RUNNING
            return self.status

        # 받은 pose를 블랙보드에 저장
        blackboard["qr_target_pose"] = self._pose
        self._node.get_logger().info("[WaitForQRPose] stored pose to blackboard")

        # 한 번만 쓰고 초기화 (원하면 안 지워도 됨)
        self._pose = None

        self.status = Status.SUCCESS
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
        "WaitForQRPose",
    ]

    CONDITION_NODES = []
    DECORATOR_NODES = []

    #limo/button
    #pressed, release 둘중에 값 하나를 받음
