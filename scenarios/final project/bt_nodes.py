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
# 1. 커스텀 데코레이터 (상태 유지 기능 강화)
# =========================================================

class RetryUntilSuccessful(Node):
    def __init__(self, name, child, num_attempts=1):
        super().__init__(name)
        self.child = child
        self.max_attempts = int(num_attempts)
        self.attempts = 0
        self.is_running = False # 실행 중인지 체크하는 플래그
        self.type = "Decorator"

    async def run(self, agent, blackboard):
        # 처음 실행이면 카운트 초기화
        if not self.is_running:
            self.attempts = 0
            self.is_running = True

        result = await self.child.run(agent, blackboard)

        if result == Status.SUCCESS:
            self.is_running = False
            self.status = Status.SUCCESS
            return Status.SUCCESS
        
        elif result == Status.FAILURE:
            self.attempts += 1
            if self.attempts % 10 == 0: 
                print(f"[{self.name}] Retrying... ({self.attempts}/{self.max_attempts})")
            
            if self.attempts < self.max_attempts:
                self.status = Status.RUNNING
                return Status.RUNNING
            else:
                self.is_running = False
                self.status = Status.FAILURE
                return Status.FAILURE
        
        self.status = Status.RUNNING
        return Status.RUNNING

    def reset(self):
        # [핵심] 강제 reset이 들어와도 실행 중이면 카운트를 유지함
        super().reset()
        if hasattr(self.child, 'reset'):
            self.child.reset()
        # self.attempts = 0  <-- 이걸 지워서 상태 유지

    def halt(self):
        # 진짜로 중단될 때만 초기화
        self.is_running = False
        self.attempts = 0
        if hasattr(self.child, 'halt'):
            self.child.halt()


class Timeout(Node):
    def __init__(self, name, child, duration=10.0):
        super().__init__(name)
        self.child = child
        self.duration = float(duration)
        self.start_time = None
        self.is_running = False # 실행 중인지 체크하는 플래그
        self.type = "Decorator"

    async def run(self, agent, blackboard):
        # [핵심] 실행 중이 아니었다면 시간 시작
        if not self.is_running or self.start_time is None:
            self.start_time = time.time()
            self.is_running = True
            print(f"[{self.name}] ⏳ Timer Started. Limit: {self.duration}s")

        elapsed = time.time() - self.start_time
        
        # 로그 출력 (1초 단위)
        if int(elapsed * 10) % 10 == 0:
             print(f"[{self.name}] ... {elapsed:.1f}s / {self.duration}s")

        if elapsed > self.duration:
            print(f"[{self.name}] 🚨 TIMEOUT! ({elapsed:.1f}s). Force FAILURE.")
            if hasattr(self.child, 'halt'):
                self.child.halt()
            self.is_running = False # 종료 처리
            self.status = Status.FAILURE
            return Status.FAILURE

        result = await self.child.run(agent, blackboard)
        
        if result == Status.SUCCESS:
            print(f"[{self.name}] Child Succeeded!")
            self.is_running = False
            self.status = Status.SUCCESS
            return Status.SUCCESS
        
        if result == Status.FAILURE:
            self.is_running = False
            self.status = Status.FAILURE
            return Status.FAILURE

        self.status = Status.RUNNING
        return Status.RUNNING

    def reset(self):
        # [핵심] 프레임워크가 매 틱마다 reset을 호출해도 시간은 초기화하지 않음
        super().reset()
        if hasattr(self.child, 'reset'):
            self.child.reset()
        # self.start_time = None <-- 이걸 지워서 시간 유지!

    def halt(self):
        # 트리가 강제로 이 노드를 끌 때만 시간 초기화
        self.is_running = False
        self.start_time = None
        if hasattr(self.child, 'halt'):
            self.child.halt()


# =========================================================
# 2. Condition Nodes (수정됨: 인자 충돌 방지)
# =========================================================
class ReceiveParcel(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [(String, "/limo/button", "button_state")])

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache: return False
        
        data = self._cache["button_state"].data.strip().lower()
        if data == "pressed":
            del self._cache["button_state"]
            return True
        return False

class DropoffParcel(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [(String, "/limo/button", "button_state")])

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache: return False
        
        state = self._cache["button_state"].data.strip().lower()
        if state in ["released", "release"]:
            del self._cache["button_state"]
            return True
        return False

class WaitForQRPose(Node):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name)
        self.ros = agent.ros_bridge
        self.qr_cache = None
        
        self.sub = self.ros.node.create_subscription(
            PoseStamped, 
            "/qr_warehouse_pose", 
            self.listener_callback, 
            10
        )
        self.type = "Action" 

    def listener_callback(self, msg):
        self.qr_cache = msg

    async def run(self, agent, blackboard):
        if self.qr_cache is not None:
            print("[WaitForQRPose] ✅ QR Code Detected! Saving to blackboard.")
            blackboard["qr_target_pose"] = self.qr_cache
            self.qr_cache = None 
            self.status = Status.SUCCESS
            return Status.SUCCESS
        
        self.status = Status.RUNNING
        return Status.RUNNING

    def reset(self):
        super().reset()
        self.qr_cache = None


# =========================================================
# 3. Action Nodes (수정됨: 인자 충돌 방지)
# =========================================================
class MoveToCharge(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, (NavigateToPose, NAV_ACTION_NAME))

    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] ⚡ Moving to Charging Station...")
        return _create_nav_goal(self.ros.node, CHARGE_X, CHARGE_Y, CHARGE_YAW)

class MoveToPickup(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, (NavigateToPose, NAV_ACTION_NAME))

    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] 📦 Moving to Pickup Point...")
        return _create_nav_goal(self.ros.node, PICKUP_X, PICKUP_Y, PICKUP_YAW)

class MoveToDelivery(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, (NavigateToPose, NAV_ACTION_NAME))

    def _build_goal(self, agent, blackboard):
        qr_pose = blackboard.get("qr_target_pose")
        if qr_pose is None: 
            print(f"[{self.name}] ❌ ERROR: No QR Pose in blackboard!")
            return None
        print(f"[{self.name}] 🚚 Moving to Delivery Point (from QR)...")
        return _create_nav_goal(self.ros.node, 0, 0, pose_stamped=qr_pose)
    
    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            if "qr_target_pose" in blackboard: del blackboard["qr_target_pose"]
            return Status.SUCCESS
        return Status.FAILURE

# =========================================================
# 4. Node Registration
# =========================================================
CUSTOM_ACTION_NODES = ['MoveToCharge', 'MoveToPickup', 'MoveToDelivery', 'WaitForQRPose']
CUSTOM_CONDITION_NODES = ['ReceiveParcel', 'DropoffParcel']
CUSTOM_DECORATOR_NODES = ['RetryUntilSuccessful', 'Timeout']

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

if hasattr(BTNodeList, 'DECORATOR_NODES'):
    BTNodeList.DECORATOR_NODES.extend(CUSTOM_DECORATOR_NODES)
else:
    BTNodeList.CONTROL_NODES.extend(CUSTOM_DECORATOR_NODES)
