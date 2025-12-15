import math
import time
import rclpy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose, Spin
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatus

from modules.base_bt_nodes import (
    BTNodeList,
    Status,
    Node,
    Sequence,
    Fallback,
    ReactiveFallback,
    ReactiveSequence,
)

from modules.base_bt_nodes_ros import (
    ConditionWithROSTopics,
    ActionWithROSAction,
)

# =========================================================
# Helper
# =========================================================
def deg(d: float) -> float:
    return math.radians(d)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _create_nav_goal(node, x, y, yaw=None, pose_stamped=None):
    goal = NavigateToPose.Goal()

    if pose_stamped:
        goal.pose = pose_stamped
    else:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0

        if yaw is not None:
            ps.pose.orientation = yaw_to_quaternion(yaw)
        else:
            ps.pose.orientation.w = 1.0

        goal.pose = ps

    return goal


# =========================================================
# Coordinates
# =========================================================
CHARGE_X, CHARGE_Y, CHARGE_YAW = -4.198, 0.200, deg(0.0)
PICKUP_X, PICKUP_Y, PICKUP_YAW = -6.326, 3.209, deg(90.0)

NAV_ACTION_NAME = "/limo/navigate_to_pose"

# =========================================================
# Topics
# =========================================================
PARCEL_AVAILABLE_TOPIC = "/parcel_available"
RECEIVE_BUSY_TOPIC = "/receive_busy"
DROPOFF_BUSY_TOPIC = "/dropoff_busy"


# =========================================================
# Decorators (원본 유지)
# =========================================================
class RetryUntilSuccessful(Node):
    def __init__(self, name, child, num_attempts=1):
        super().__init__(name)
        self.child = child
        self.max_attempts = int(num_attempts)
        self.attempts = 0
        self.is_running = False
        self.type = "Decorator"

    async def run(self, agent, blackboard):
        if not self.is_running:
            self.attempts = 0
            self.is_running = True

        result = await self.child.run(agent, blackboard)

        if result == Status.SUCCESS:
            self.is_running = False
            return Status.SUCCESS

        if result == Status.FAILURE:
            self.attempts += 1
            if self.attempts < self.max_attempts:
                return Status.RUNNING
            self.is_running = False
            return Status.FAILURE

        return Status.RUNNING

    def reset(self):
        super().reset()
        if hasattr(self.child, "reset"):
            self.child.reset()

    def halt(self):
        self.is_running = False
        self.attempts = 0
        if hasattr(self.child, "halt"):
            self.child.halt()


class Timeout(Node):
    def __init__(self, name, child, duration=10.0):
        super().__init__(name)
        self.child = child
        self.duration = float(duration)
        self.start_time = None
        self.is_running = False
        self.type = "Decorator"

    async def run(self, agent, blackboard):
        if not self.is_running or self.start_time is None:
            self.start_time = time.time()
            self.is_running = True
            print(f"[{self.name}] Timer Started ({self.duration}s)")

        elapsed = time.time() - self.start_time

        if elapsed > self.duration:
            print(f"[{self.name}] TIMEOUT")
            if hasattr(self.child, "halt"):
                self.child.halt()
            self.is_running = False
            return Status.FAILURE

        result = await self.child.run(agent, blackboard)

        if result in (Status.SUCCESS, Status.FAILURE):
            self.is_running = False
            return result

        return Status.RUNNING

    def reset(self):
        super().reset()
        if hasattr(self.child, "reset"):
            self.child.reset()

    def halt(self):
        self.is_running = False
        self.start_time = None
        if hasattr(self.child, "halt"):
            self.child.halt()


# =========================================================
# Condition Nodes
# =========================================================
class ParcelAvailable(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            [(Bool, PARCEL_AVAILABLE_TOPIC, "parcel_available")],
        )

    def _predicate(self, agent, blackboard):
        return self._cache.get(
            "parcel_available", Bool(data=False)
        ).data


class OtherRobotReceiving(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            [(Bool, RECEIVE_BUSY_TOPIC, "recv_busy")],
        )

    async def run(self, agent, blackboard):
        msg = self._cache.get("recv_busy")
        if msg is None:
            return Status.FAILURE
        return Status.SUCCESS if msg.data else Status.FAILURE


class OtherRobotDropping(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            [(Bool, DROPOFF_BUSY_TOPIC, "drop_busy")],
        )

    def _predicate(self, agent, blackboard):
        return self._cache.get(
            "drop_busy", Bool(data=False)
        ).data


class ReceiveParcel(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            [(String, "/limo/button_status", "button_state")],
        )

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache:
            return False

        if self._cache["button_state"].data.strip().lower() == "pressed":
            del self._cache["button_state"]
            return True

        return False


class DropoffParcel(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            [(String, "/limo/button_status", "button_state")],
        )

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache:
            return False

        if self._cache["button_state"].data.strip().lower() == "released":
            del self._cache["button_state"]
            return True

        return False


class WaitForQRPose(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            [(PoseStamped, "/qr_warehouse_pose", "qr_pose")],
        )

    def _predicate(self, agent, blackboard):
        if "qr_pose" not in self._cache:
            return False

        blackboard["qr_target_pose"] = self._cache["qr_pose"]
        del self._cache["qr_pose"]
        return True


# =========================================================
# Action Nodes
# =========================================================
class MoveToCharge(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            (NavigateToPose, NAV_ACTION_NAME),
        )

    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(
            self.ros.node, CHARGE_X, CHARGE_Y, CHARGE_YAW
        )


class MoveToPickup(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            (NavigateToPose, NAV_ACTION_NAME),
        )

        self.receive_busy_pub = agent.ros_bridge.node.create_publisher(
            Bool, RECEIVE_BUSY_TOPIC, 10
        )
        self.busy_sent = False

    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(
            self.ros.node, PICKUP_X, PICKUP_Y, PICKUP_YAW
        )

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            if not self.busy_sent:
                self.receive_busy_pub.publish(Bool(data=True))
                self.busy_sent = True
                print("[MoveToPickup] 🔴 /receive_busy = true")
            return Status.SUCCESS

        return Status.FAILURE


class MoveToDelivery(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            (NavigateToPose, NAV_ACTION_NAME),
        )

        self.receive_busy_pub = agent.ros_bridge.node.create_publisher(
            Bool, RECEIVE_BUSY_TOPIC, 10
        )
        self.busy_cleared = False

    def _build_goal(self, agent, blackboard):
        qr_pose = blackboard.get("qr_target_pose")
        if qr_pose is None:
            return None

        if not self.busy_cleared:
            self.receive_busy_pub.publish(Bool(data=False))
            self.busy_cleared = True
            print("[MoveToDelivery] 🟢 /receive_busy = false")

        return _create_nav_goal(
            self.ros.node, 0, 0, pose_stamped=qr_pose
        )

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            if "qr_target_pose" in blackboard:
                del blackboard["qr_target_pose"]
            return Status.SUCCESS

        return Status.FAILURE


class MoveToPickupWaiting(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            (NavigateToPose, NAV_ACTION_NAME),
        )

    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] ⏸ Moving to Waiting Area")
        return _create_nav_goal(
            self.ros.node, WAIT_X, WAIT_Y, WAIT_YAW
        )

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.RUNNING
        return Status.RUNNING

class MoveToWaitingDrop(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            (NavigateToPose, NAV_ACTION_NAME),
        )

    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] ⏸ Moving to Waiting Area")
        return _create_nav_goal(
            self.ros.node, WAIT_X, WAIT_Y, WAIT_YAW
        )

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.RUNNING
        return Status.RUNNING

class SpinInPlace(ActionWithROSAction): #리모 로봇이 제자리 회전하는 노드
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        # 서버(/limo/spin)에 Spin 액션을 요청하도록 설정
        super().__init__(final_name, agent, (Spin, "/limo/spin"))

    def _build_goal(self, agent, blackboard):
        goal = Spin.Goal()
        goal.target_yaw = 1.57  # 90도 회전 (서버 설정에 따름)
        return goal


# =========================================================
# Registration (🔥 원본 유지)
# =========================================================
CUSTOM_ACTION_NODES = [
    "MoveToCharge",
    "MoveToPickup",
    "MoveToDelivery",
    "MoveToWaiting",
]

CUSTOM_CONDITION_NODES = [
    "ReceiveParcel",
    "DropoffParcel",
    "WaitForQRPose",
    "ParcelAvailable",
    "OtherRobotReceiving",
    "OtherRobotDropping",
]

CUSTOM_DECORATOR_NODES = [
    "RetryUntilSuccessful",
    "Timeout",
]

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

# ⚠️ 네가 강조한 부분: 절대 수정 안 함
if hasattr(BTNodeList, "DECORATOR_NODES"):
    BTNodeList.DECORATOR_NODES.extend(CUSTOM_DECORATOR_NODES)
else:
    BTNodeList.CONTROL_NODES.extend(CUSTOM_DECORATOR_NODES)
