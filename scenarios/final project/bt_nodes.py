# scenarios/limo_delivery/bt_nodes.py

import math
import rclpy
import py_trees
from py_trees.decorators import Retry, Timeout  # [수정] Timeout 추가

# ROS 2 Messages & Action
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

# Base Modules
from modules.base_bt_nodes import BTNodeList, Status
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction

# =========================================================
# 헬퍼 함수
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

# [좌표 설정]
CHARGE_X,  CHARGE_Y,  CHARGE_YAW  = -4.198, 0.200, deg(0.0)
PICKUP_X,  PICKUP_Y,  PICKUP_YAW  = -6.326, 3.209, deg(90.0)

# [Action Server 이름]
NAV_ACTION_NAME = "/limo/navigate_to_pose"


# =========================================================
# Decorators
# =========================================================
class RetryUntilSuccessful(Retry):
    def __init__(self, name, child, num_attempts=1):
        super(RetryUntilSuccessful, self).__init__(
            name=name,
            child=child,
            num_failures=int(num_attempts)
        )

# [추가] 일정 시간 동안 성공 못하면 실패 처리하는 데코레이터
class Timeout(Timeout):
    def __init__(self, name, child, duration=10.0):
        super(Timeout, self).__init__(
            name=name, 
            child=child, 
            duration=float(duration)
        )

# =========================================================
# Condition Nodes
# =========================================================
class ReceiveParcel(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, [(String, "/limo/button", "button_state")])

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache: return False
        return self._cache["button_state"].data.strip().lower() == "pressed"

class DropoffParcel(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, [(String, "/limo/button", "button_state")])

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache: return False
        state = self._cache["button_state"].data.strip().lower()
        return state in ["released", "release"]

class WaitForQRPose(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, [(PoseStamped, "/qr_warehouse_pose", "qr_pose")])

    def _predicate(self, agent, blackboard):
        if "qr_pose" in self._cache:
            # 오래된 데이터 방지를 위해 타임스탬프 체크를 할 수도 있으나,
            # 여기서는 데이터가 들어오면 바로 성공으로 처리합니다.
            blackboard["qr_target_pose"] = self._cache["qr_pose"]
            return True
        return False

# =========================================================
# Action Nodes
# =========================================================
class MoveToCharge(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, NAV_ACTION_NAME))
    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(self.ros.node, CHARGE_X, CHARGE_Y, CHARGE_YAW)

class MoveToPickup(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, NAV_ACTION_NAME))
    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(self.ros.node, PICKUP_X, PICKUP_Y, PICKUP_YAW)

class MoveToDelivery(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, NAV_ACTION_NAME))
    def _build_goal(self, agent, blackboard):
        qr_pose = blackboard.get("qr_target_pose")
        if qr_pose is None: return None
        return _create_nav_goal(self.ros.node, 0, 0, pose_stamped=qr_pose)
    
    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            if "qr_target_pose" in blackboard: del blackboard["qr_target_pose"]
            return Status.SUCCESS
        return Status.FAILURE

# =========================================================
# Node Registration
# =========================================================
CUSTOM_ACTION_NODES = ['MoveToCharge', 'MoveToPickup', 'MoveToDelivery']
CUSTOM_CONDITION_NODES = ['ReceiveParcel', 'DropoffParcel', 'WaitForQRPose']
# [수정] Timeout 추가
CUSTOM_DECORATOR_NODES = ['RetryUntilSuccessful', 'Timeout']

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

if hasattr(BTNodeList, 'DECORATOR_NODES'):
    BTNodeList.DECORATOR_NODES.extend(CUSTOM_DECORATOR_NODES)
else:
    BTNodeList.CONTROL_NODES.extend(CUSTOM_DECORATOR_NODES)
