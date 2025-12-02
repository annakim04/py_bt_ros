import math
import rclpy
from rclpy.node import Node as RosNode

# =========================================================
# 1. [핵심] py_trees 라이브러리 Import
# =========================================================
# XML에 있는 RetryUntilSuccessful 기능을 구현하기 위해 필요합니다.
import py_trees
from py_trees.decorators import Retry

# ROS 2 Messages
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

# py_bt_ros 기본 모듈 Import
# (사용하시는 환경의 경로와 일치해야 합니다)
from modules.base_bt_nodes import BTNodeList, Status
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction


# =========================================================
# 2. 헬퍼 함수 및 좌표 정의
# =========================================================

def deg(d: float) -> float:
    return math.radians(d)

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Yaw(rad) -> Quaternion 변환"""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def _create_nav_goal(node, x, y, yaw=None, pose_stamped=None):
    """Nav2 Goal 메시지 생성 헬퍼"""
    goal = NavigateToPose.Goal()
    
    if pose_stamped:
        # QR 코드 등에서 받은 전체 PoseStamped가 있다면 사용
        goal.pose = pose_stamped
    else:
        # 직접 지정한 좌표(x, y, yaw) 사용
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

# --- [사용자 맵 좌표 설정] ---
# 충전 장소 (Charge)
CHARGE_X,  CHARGE_Y,  CHARGE_YAW  = -4.198, 0.2, deg(89.274)
# 택배 수령 장소 (Pickup)
PICKUP_X,  PICKUP_Y,  PICKUP_YAW  = -6.326, 3.209, deg(-78.415)


# =========================================================
# 3. Custom Decorator (RetryUntilSuccessful)
# =========================================================

class RetryUntilSuccessful(Retry):
    """
    XML 태그: <RetryUntilSuccessful num_attempts="60">
    설명: 자식 노드가 성공할 때까지 지정된 횟수만큼 재시도합니다.
    (py_trees의 Retry 클래스를 상속받아 구현)
    """
    def __init__(self, name, child, num_attempts=1):
        # XML의 'num_attempts'는 문자열로 올 수 있으므로 int 변환 필요
        # 부모 클래스(Retry)는 'num_failures'라는 인자를 사용함
        super(RetryUntilSuccessful, self).__init__(
            name=name,
            child=child,
            num_failures=int(num_attempts)
        )


# =========================================================
# 4. Condition Nodes (센서/토픽 확인)
# =========================================================

class ReceiveParcel(ConditionWithROSTopics):
    """
    역할: 택배 수령 버튼이 눌렸는지 확인 (토픽: /limo/button)
    성공조건: data == "pressed"
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (String, "/limo/button", "button_state") # (Type, Topic, CacheKey)
        ])

    def _predicate(self, agent, blackboard):
        # 데이터가 아직 안 들어왔으면 False
        if "button_state" not in self._cache:
            return False
        
        # 데이터 확인
        msg = self._cache["button_state"]
        state = msg.data.strip().lower()
        
        return state == "pressed"


class DropoffParcel(ConditionWithROSTopics):
    """
    역할: 택배 하차 확인 (버튼이 떼어졌는지 확인)
    성공조건: data == "release"
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (String, "/limo/button", "button_state")
        ])

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache:
            return False
        
        msg = self._cache["button_state"]
        state = msg.data.strip().lower()
        
        return state == "release"


class WaitForQRPose(ConditionWithROSTopics):
    """
    역할: QR 코드 좌표 수신 대기 (/qr_warehouse_pose)
    동작: 데이터가 들어오면 블랙보드('qr_target_pose')에 저장하고 Success 반환
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (PoseStamped, "/qr_warehouse_pose", "qr_pose")
        ])

    def _predicate(self, agent, blackboard):
        if "qr_pose" in self._cache:
            # 블랙보드에 저장 (MoveToDelivery 노드가 사용함)
            blackboard["qr_target_pose"] = self._cache["qr_pose"]
            return True
        return False


# =========================================================
# 5. Action Nodes (Nav2 이동)
# =========================================================

class MoveToCharge(ActionWithROSAction):
    """충전 장소로 이동"""
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, "/navigate_to_pose"))

    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(self.ros.node, CHARGE_X, CHARGE_Y, CHARGE_YAW)

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        return Status.FAILURE


class MoveToPickup(ActionWithROSAction):
    """택배 수령 장소로 이동"""
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, "/navigate_to_pose"))

    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(self.ros.node, PICKUP_X, PICKUP_Y, PICKUP_YAW)

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        return Status.FAILURE


class MoveToDelivery(ActionWithROSAction):
    """
    택배 배달 장소로 이동
    (WaitForQRPose가 블랙보드에 저장해둔 좌표 사용)
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, "/navigate_to_pose"))

    def _build_goal(self, agent, blackboard):
        # 블랙보드에서 목표 좌표 가져오기
        qr_pose = blackboard.get("qr_target_pose")
        
        if qr_pose is None:
            print("[MoveToDelivery] Error: QR Pose not found in blackboard!")
            return None # None 리턴 시 Action 실패 처리됨

        return _create_nav_goal(self.ros.node, 0, 0, pose_stamped=qr_pose)

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        return Status.FAILURE


# =========================================================
# 6. Node Registration (XML 파서에 등록)
# =========================================================

# 1) 정의한 노드 이름 리스트
CUSTOM_ACTION_NODES = [
    'MoveToCharge',
    'MoveToPickup',
    'MoveToDelivery',
]

CUSTOM_CONDITION_NODES = [
    'ReceiveParcel',
    'DropoffParcel',
    'WaitForQRPose',
]

CUSTOM_DECORATOR_NODES = [
    'RetryUntilSuccessful'
]

# 2) BTNodeList 확장 (Module 로딩 시 자동 실행)
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

# [중요] Decorator 등록
# XML 파서가 <RetryUntilSuccessful> 태그를 인식하려면 CONTROL_NODES나 ACTION_NODES에 있어야 함
if hasattr(BTNodeList, 'CONTROL_NODES'):
    BTNodeList.CONTROL_NODES.extend(CUSTOM_DECORATOR_NODES)
else:
    # 만약 베이스 코드에 CONTROL_NODES 리스트가 없다면 Action 리스트에 추가
    BTNodeList.ACTION_NODES.extend(CUSTOM_DECORATOR_NODES)
