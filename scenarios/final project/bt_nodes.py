# scenarios/final_project/bt_nodes.py

import math
import time
import rclpy

# ROS 2 Messages & Action
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

# Base Modules
from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback
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
# 1. 커스텀 데코레이터 직접 구현
# =========================================================

class RetryUntilSuccessful(Node):
    def __init__(self, name, child, num_attempts=1):
        super().__init__(name)
        self.child = child
        self.max_attempts = int(num_attempts)
        self.attempts = 0
        self.type = "Decorator"

    async def run(self, agent, blackboard):
        if self.status != Status.RUNNING:
            self.attempts = 0

        result = await self.child.run(agent, blackboard)

        if result == Status.SUCCESS:
            self.status = Status.SUCCESS
            return Status.SUCCESS
        
        elif result == Status.FAILURE:
            self.attempts += 1
            if self.attempts % 10 == 0: # 10번마다 로그 출력
                print(f"[{self.name}] Retrying... ({self.attempts}/{self.max_attempts})")
            
            if self.attempts < self.max_attempts:
                self.status = Status.RUNNING
                return Status.RUNNING
            else:
                self.status = Status.FAILURE
                return Status.FAILURE
        
        self.status = Status.RUNNING
        return Status.RUNNING

    def reset(self):
        super().reset()
        self.child.reset()
        self.attempts = 0


class Timeout(Node):
    def __init__(self, name, child, duration=10.0):
        super().__init__(name)
        self.child = child
        self.duration = float(duration)
        self.start_time = None
        self.type = "Decorator"

    async def run(self, agent, blackboard):
        if self.status != Status.RUNNING or self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time
        
        # [디버깅] 남은 시간 출력 (1초 간격으로 찍힘)
        # if int(elapsed) % 5 == 0: 
        #     print(f"[{self.name}] Time elapsed: {elapsed:.1f}/{self.duration}")

        if elapsed > self.duration:
            print(f"[{self.name}] TIMEOUT! ({self.duration}s passed). Failing child.")
            if hasattr(self.child, 'halt'):
                self.child.halt()
            self.status = Status.FAILURE
            return Status.FAILURE

        result = await self.child.run(agent, blackboard)
        self.status = result
        return result

    def reset(self):
        super().reset()
        self.child.reset()
        self.start_time = None


# =========================================================
# 2. Condition Nodes
# =========================================================
class ReceiveParcel(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, [(String, "/limo/button", "button_state")])
    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache: return False
        
        data = self._cache["button_state"].data.strip().lower()
        if data == "pressed":
            # 버튼 확인 후 캐시 삭제 (한번 누르면 소모됨)
            del self._cache["button_state"]
            return True
        return False

class DropoffParcel(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, [(String, "/limo/button", "button_state")])
    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache: return False
        
        state = self._cache["button_state"].data.strip().lower()
        if state in ["released", "release"]:
            del self._cache["button_state"]
            return True
        return False

class WaitForQRPose(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, [(PoseStamped, "/qr_warehouse_pose", "qr_pose")])
    
    def _predicate(self, agent, blackboard):
        if "qr_pose" in self._cache:
            print("[WaitForQRPose] QR Code Detected! Saving to blackboard.")
            blackboard["qr_target_pose"] = self._cache["qr_pose"]
            
            # [핵심 수정] 사용한 데이터는 즉시 삭제 (안 그러면 계속 Success 뜸)
            del self._cache["qr_pose"] 
            return True
        return False

# =========================================================
# 3. Action Nodes
# =========================================================
class MoveToCharge(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, NAV_ACTION_NAME))
    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] Moving to Charging Station...")
        return _create_nav_goal(self.ros.node, CHARGE_X, CHARGE_Y, CHARGE_YAW)

class MoveToPickup(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, NAV_ACTION_NAME))
    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] Moving to Pickup Point...")
        return _create_nav_goal(self.ros.node, PICKUP_X, PICKUP_Y, PICKUP_YAW)

class MoveToDelivery(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, NAV_ACTION_NAME))
    def _build_goal(self, agent, blackboard):
        qr_pose = blackboard.get("qr_target_pose")
        if qr_pose is None: 
            print(f"[{self.name}] ERROR: No QR Pose in blackboard!")
            return None
        print(f"[{self.name}] Moving to Delivery Point (from QR)...")
        return _create_nav_goal(self.ros.node, 0, 0, pose_stamped=qr_pose)
    
    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            if "qr_target_pose" in blackboard: del blackboard["qr_target_pose"]
            return Status.SUCCESS
        return Status.FAILURE

# =========================================================
# 4. Node Registration
# =========================================================
CUSTOM_ACTION_NODES = ['MoveToCharge', 'MoveToPickup', 'MoveToDelivery']
CUSTOM_CONDITION_NODES = ['ReceiveParcel', 'DropoffParcel', 'WaitForQRPose']
CUSTOM_DECORATOR_NODES = ['RetryUntilSuccessful', 'Timeout']

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

if hasattr(BTNodeList, 'DECORATOR_NODES'):
    BTNodeList.DECORATOR_NODES.extend(CUSTOM_DECORATOR_NODES)
else:
    BTNodeList.CONTROL_NODES.extend(CUSTOM_DECORATOR_NODES)
