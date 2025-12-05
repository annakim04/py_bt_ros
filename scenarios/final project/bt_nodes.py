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

# [좌표 설정] - 사용자 환경에 맞게 수정 필요
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
        self.is_running = False 
        self.type = "Decorator"

    async def run(self, agent, blackboard):
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
        super().reset()
        if hasattr(self.child, 'reset'):
            self.child.reset()

    def halt(self):
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
        self.is_running = False 
        self.type = "Decorator"

    async def run(self, agent, blackboard):
        # [핵심] 처음 시작할 때만 타이머 시작
        if not self.is_running or self.start_time is None:
            self.start_time = time.time()
            self.is_running = True
            print(f"[{self.name}] ⏳ Timer Started. Limit: {self.duration}s")

        elapsed = time.time() - self.start_time
        
        # 1초마다 남은 시간 출력
        if int(elapsed * 10) % 10 == 0:
             print(f"[{self.name}] ... {elapsed:.1f}s / {self.duration}s")

        # 시간 초과 체크
        if elapsed > self.duration:
            print(f"[{self.name}] 🚨 TIMEOUT! ({elapsed:.1f}s). Force FAILURE.")
            if hasattr(self.child, 'halt'):
                self.child.halt()
            self.is_running = False 
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
        # [핵심] 외부 reset 신호에도 시간은 유지
        super().reset()
        if hasattr(self.child, 'reset'):
            self.child.reset()

    def halt(self):
        self.is_running = False
        self.start_time = None
        if hasattr(self.child, 'halt'):
            self.child.halt()


# =========================================================
# 2. Condition Nodes (인자 충돌 방지 적용)
# =========================================================
class ReceiveParcel(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [(String, "/limo/button", "button_state")])

    def _predicate(self, agent, blackboard):
        # 1. 데이터가 아예 안 들어온 경우
        if "button_state" not in self._cache:
            # 너무 자주 뜨면 주석 처리 가능
            print(f"[{self.name}] ⏳ Waiting for /limo/button data...") 
            return False
        
        # 2. 데이터가 들어온 경우: 내용 확인
        msg = self._cache["button_state"]
        raw_data = msg.data.strip().lower()
        
        # [디버깅] 현재 버튼 상태 출력
        print(f"[{self.name}] 🔘 Button State: '{raw_data}'")
        
        if raw_data == "pressed":
            print(f"[{self.name}] ✅ Button PRESSED! Moving to next step.")
            # 버튼 확인 후 캐시 삭제 (한번 누르면 소모됨)
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

# [수정된 WaitForQRPose] - QoS 호환성 해결 버전
# [수정된 WaitForQRPose]
class WaitForQRPose(Node):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name)
        self.ros = agent.ros_bridge

        # 콜백에서 받은 Pose를 저장할 변수
        self.qr_pose = None
        
        self.sub = self.ros.node.create_subscription(
            PoseStamped, 
            "/qr_warehouse_pose",   # publisher와 동일하게 절대 토픽명
            self.listener_callback, 
            10
        )
        self.type = "Action" 

    def listener_callback(self, msg: PoseStamped):
        # 👉 여기서 x, y 모두 찍기
        x = msg.pose.position.x
        y = msg.pose.position.y
        print(f"[WaitForQRPose] ✉ DATA RECEIVED! x={x:.3f}, y={y:.3f}")
        print(f"[WaitForQRPose]    frame_id={msg.header.frame_id}")
        # run()에서 사용할 Pose 저장
        self.qr_pose = msg

    async def run(self, agent, blackboard):
        # 콜백에서 Pose가 들어왔으면 SUCCESS
        if self.qr_pose is not None:
            print("[WaitForQRPose] ✅ QR Pose Detected! Saving to blackboard.")
            # Blackboard에 저장해서 MoveToDelivery에서 사용
            blackboard["qr_target_pose"] = self.qr_pose

            # 여기서 바로 성공 처리
            self.status = Status.SUCCESS
            return Status.SUCCESS

        # 아직 Pose 못 받았으면 계속 기다림
        print("[WaitForQRPose] ⏳ Waiting for QR Pose...")
        self.status = Status.RUNNING
        return Status.RUNNING

    def reset(self):
        # ⚠️ 중요: Timeout이 매 틱마다 reset()을 호출하더라도
        #         qr_pose는 지우지 않아야 함 (그래야 run()이 볼 수 있음)
        super().reset()
        # self.qr_pose = None  # ← 이 줄은 절대 쓰지 말 것!
        # 필요하면 halt()에서만 지워도 됨

    def halt(self):
        # 트리에서 완전히 빠질 때만 Pose 제거
        self.qr_pose = None
        if hasattr(super(), "halt"):
            super().halt()


# =========================================================
# 3. Action Nodes (인자 충돌 방지 적용)
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
