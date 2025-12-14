import math
import time
import rclpy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose, Spin
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveFallback, ReactiveSequence
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction


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

CHARGE_X,  CHARGE_Y,  CHARGE_YAW  = -4.198, 0.200, deg(0.0) #배달 픽업 장소 좌표
PICKUP_X,  PICKUP_Y,  PICKUP_YAW  = -6.326, 3.209, deg(90.0) #배달 수령 장소 좌표

NAV_ACTION_NAME = "/limo/navigate_to_pose" #Nav2에 목적지를 보내는 토픽

#커스텀 노드
#자식 노드가 실패하더라도 즉시 실패를 반환하지 않고, 지정된 횟수만큼 재시도 기회를 주는 노드
class RetryUntilSuccessful(Node):
    def __init__(self, name, child, num_attempts=1):
        super().__init__(name)
        self.child = child #RetryUntilSuccessful 노드가 감싸고 있는 실제 실행 노드
        self.max_attempts = int(num_attempts) # 최대 재시도 횟수 설정
        self.attempts = 0 #현재 시도한 횟수 카운터
        self.is_running = False #현재 재시도 진행 중인지 확인하는 플래그
        self.type = "Decorator" #노드 타입 명시

    async def run(self, agent, blackboard):
        #처음 실행할 때 카운터를 0으로 초기화
        if not self.is_running:
            self.attempts = 0
            self.is_running = True

        result = await self.child.run(agent, blackboard) #자식 노드를 실행하고 결과를 받아옴
       
        #자식 노드가 성공한 경우 성공 반환
        if result == Status.SUCCESS:
            self.is_running = False #재시도 루프 종료
            self.status = Status.SUCCESS
            return Status.SUCCESS
       
        #자식 노드가 실패한 경우 실패 반환
        elif result == Status.FAILURE:
            self.attempts += 1 #실패 카운트 증가
            #로그를 너무 자주 뜨지 않게 하기 위해 10번에 한번만 출력
            if self.attempts % 10 == 0:
                print(f"[{self.name}] Retrying... ({self.attempts}/{self.max_attempts})")
            #아직 재시도 기회가 남은 경우 실행 중 상태를 반환
            if self.attempts < self.max_attempts:
                self.status = Status.RUNNING
                return Status.RUNNING
            else:
                #재시도 기회 전부 사용한 경우
                self.is_running = False #루프 종료
                self.status = Status.FAILURE
                return Status.FAILURE
        #자식 노드가 실행하고 있는 경우 실행 중 반환
        self.status = Status.RUNNING
        return Status.RUNNING
    #0.1초마다 돌아오는 다음 생각(Tick)을 준비하는 함수
    #자식 노드는 다시 실행할 수 있게 하지만, 지금까지의 실패 횟수는 기억하며 재시도 로직 진행
    def reset(self): #행동 트리가 강제로 초기화할 때 호출됨
        super().reset()
        if hasattr(self.child, 'reset'):
            self.child.reset()
            #self.attempts를 초기화하지 않음 (상태 유지 필요 시)

    #더 중요한 일이 생겨서 이 작업을 아예 중단할 때 호출
    #나중에 다시 돌아왔을 때는 실패 카운트를 0으로 완전히 초기화
    def halt(self): # 행동 트리가 강제로 중단할 때 호출됨
        self.is_running = False
        self.attempts = 0
        if hasattr(self.child, 'halt'):
            self.child.halt()

#자식 노드가 정해진 시간 안에 성공하지 못하면,강제로 실패 처리해 무한 대기를 방지하는 노드
class Timeout(Node):
    def __init__(self, name, child, duration=10.0):
        super().__init__(name)
        self.child = child # Timeout 노드가 감싸고 있는 실제 실행 노드
        self.duration = float(duration) #제한 시간 (초 단위)
        self.start_time = None #타이머 시작 시간
        self.is_running = False #현재 타이머가 돌아가는 중인지 확인하는 플래그
        self.type = "Decorator" #노드 타입 명시

    async def run(self, agent, blackboard):
        #처음 실행될 때 한 번만 타이머 시작
        #is_running이 False거나 start_time이 없으면 새로 시간을 잰다.
        if not self.is_running or self.start_time is None:
            self.start_time = time.time()
            self.is_running = True
            print(f"[{self.name}] Timer Started. Limit: {self.duration}s")

        #경과 시간 계산
        elapsed = time.time() - self.start_time
       
        # 1초마다 남은 시간 출력
        if int(elapsed * 10) % 10 == 0:
            print(f"[{self.name}] ... {elapsed:.1f}s / {self.duration}s")

        # 시간 초과 체크
        if elapsed > self.duration:
            print(f"[{self.name}] TIMEOUT! ({elapsed:.1f}s). Force FAILURE.")
            #자식 노드에게 중단 명령(halt)을 내림
            if hasattr(self.child, 'halt'):
                self.child.halt()
            self.is_running = False
            self.status = Status.FAILURE
            return Status.FAILURE

        #아직 시간이 남았다면, 자식 노드를 실행
        result = await self.child.run(agent, blackboard)
       
        #자식 노드가 시간 안에 성공한 경우
        if result == Status.SUCCESS:
            print(f"[{self.name}] Child Succeeded!")
            self.is_running = False #타이머 종료
            self.status = Status.SUCCESS
            return Status.SUCCESS
       
        #자식 노드가 스스로 실패했을 때
        if result == Status.FAILURE:
            self.is_running = False #타이머 종료
            self.status = Status.FAILURE
            return Status.FAILURE
       
        #자식 노드가 RUNNING인 경우
        self.status = Status.RUNNING
        return Status.RUNNING

    #0.1초마다 돌아오는 다음 생각(Tick)을 준비하는 함수
    def reset(self):
        #외부 reset 신호에도 시간은 유지
        super().reset()
        if hasattr(self.child, 'reset'):
            self.child.reset()
        #여기서 self.start_time을 지우면 0.1초마다 시간이 초기화되어 영원히 Timeout이 안 됨

    #더 중요한 일이 생겨서 이 작업을 아예 중단할 때 호출
     #나중에 다시 돌아왔을 때는 0초로 초기화
    def halt(self): # 행동 트리가 강제로 중단할 때 호출됨
        self.is_running = False
        self.start_time = None
        if hasattr(self.child, 'halt'):
            self.child.halt()


#컨디션 노드
class ReceiveParcel(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent,
                         [(String, "/limo/button_status", "button_state")])

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache:
            return False

        state = self._cache["button_state"].data.strip().lower()
        print(f"[{self.name}] 🔘 Button State = {state}")

        if state == "pressed":
            # 상태 플래그 저장
            blackboard["parcel_received"] = True
            return True

        return False

class DropoffParcel(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent,
                         [(String, "/limo/button_status", "button_state")])

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache:
            return False

        state = self._cache["button_state"].data.strip().lower()
        print(f"[{self.name}] 🔘 Button State = {state}")

        if state == "released":
            blackboard["parcel_dropped"] = True
            return True

        return False

class WaitForQRPose(ConditionWithROSTopics): #배달 장소 인식 여부를 판단하는 노드
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [(PoseStamped, "/qr_warehouse_pose", "qr_pose")])

    def _predicate(self, agent, blackboard):
        #데이터가 아예 안 들어온 경우
        if "qr_pose" not in self._cache:
            print(f"[{self.name}] Waiting for QR data")
            return False
       
        #이동할 x,y 좌표를 데이터를 안전하게 넘겨주기 위해 캐시에 저장
        '''
        행동 트리(MoveToDelivery)가 블랙보드에 있는 좌표 (X=10, Y=20)을 읽으려고 합니다.

        X=10을 읽는 그 0.0001초 찰나의 순간에!

        ROS 콜백이 끼어들어서 새로운 좌표 (X=50, Y=90)을 덮어써버립니다.

        행동 트리는 X는 옛날 것(10), Y는 새것(90)을 읽어서 (10, 90)이라는 존재하지 않는 이상한 좌표로 이동해버릴 수 있습니다.
        '''
        msg = self._cache["qr_pose"]
       
        # 이동할 x,y 좌표를 출력
        x = msg.pose.position.x
        y = msg.pose.position.y
        print(f"[{self.name}] ✉ DATA RECEIVED! x={x:.3f}, y={y:.3f}")

        print(f"[{self.name}] QR Pose Detected! Saving to blackboard.")
        #모든 노드가 볼 수 있게 블랙 보드에 이동할 x,y좌표를 저장함
        blackboard["qr_target_pose"] = msg

        # 4. 사용한 데이터 삭제 (한 번만 인식하고 넘어가기 위해)
        del self._cache["qr_pose"]
        return True

class IsButtonPressed(ConditionWithROSTopics): #택배 운송 여부를 판단하는 노
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [(String, "/limo/button_status", "button_state")])

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache:
            return False # 데이터 없으면 안 눌린 것으로 간주
       
        data = self._cache["button_state"].data.strip().lower()
        # pressed 상태면 True, 아니면 False (데이터를 지우지 않음!)
        return (data == "pressed")

#액션 노드
class MoveToCharge(ActionWithROSAction): #충전 장소로 이동하는 액션 노드
    #Nav2에 보낼 요청 생성
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, (NavigateToPose, NAV_ACTION_NAME))

    #백보드에 저장한 x,y좌표로 이동하는 요청을 Nav2에 전송
    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] Moving to Charging Station")
        return _create_nav_goal(self.ros.node, CHARGE_X, CHARGE_Y, CHARGE_YAW)

class MoveToPickup(ActionWithROSAction): #수령 장소로 이동하는 액션 노드
    #Nav2에 보낼 요청 생성
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, (NavigateToPose, NAV_ACTION_NAME))

    #백보드에 저장한 x,y좌표로 이동하는 요청을 Nav2에 전송
    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] Moving to Pickup Point")
        return _create_nav_goal(self.ros.node, PICKUP_X, PICKUP_Y, PICKUP_YAW)

class MoveToDelivery(ActionWithROSAction): #배달 장소로 이동하는 액션 노드
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, (NavigateToPose, NAV_ACTION_NAME))

    #백보드에 저장한 x,y좌표를 Nav2에 전송
    def _build_goal(self, agent, blackboard):
        qr_pose = blackboard.get("qr_target_pose")
        if qr_pose is None:
            print(f"[{self.name}] ERROR: No QR Pose in blackboard")
            return None
        print(f"[{self.name}] Moving to Delivery Point (from QR)")
        return _create_nav_goal(self.ros.node, 0, 0, pose_stamped=qr_pose)
   
    #Nav2에 전송을 성공 실패 여부 확인
    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            if "qr_target_pose" in blackboard: del blackboard["qr_target_pose"]
            return Status.SUCCESS
        return Status.FAILURE

class SpinInPlace(ActionWithROSAction): #리모 로봇이 제자리 회전하는 노드
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        # 서버(/limo/spin)에 Spin 액션을 요청하도록 설정
        super().__init__(final_name, agent, (Spin, "/limo/spin"))

    def _build_goal(self, agent, blackboard):
        goal = Spin.Goal()
        goal.target_yaw = 1.57  # 90도 회전 (서버 설정에 따름)
        return goal

CUSTOM_ACTION_NODES = ['MoveToCharge', 'MoveToPickup', 'MoveToDelivery', 'SpinInPlace']
CUSTOM_CONDITION_NODES = ['ReceiveParcel', 'DropoffParcel','WaitForQRPose','IsButtonPressed']
CUSTOM_DECORATOR_NODES = ['RetryUntilSuccessful', 'Timeout']

#내가 만든 커스텀 노드들을 행동 트리 시스템에 등록하는 절차
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

#행동 트리 라이브러리의 버전에 따라 노드를 관리하는 방식이 다를 수 있기 때문
#최신의 경우: "Sequence나 Fallback은 Control에 넣고, Retry나 Timeout은 Decorator에 넣는데 만약 DECORATOR_NODES가 있는데 CONTROL_NODES에 넣으면 에러가 날 수도 있음
#옛날의 경우: 전부 Control에 넣는데 이 경우 DECORATOR_NODES라는 리스트 자체가 아예 존재하지 않음
if hasattr(BTNodeList, 'DECORATOR_NODES'):
    BTNodeList.DECORATOR_NODES.extend(CUSTOM_DECORATOR_NODES)
else:
    BTNodeList.CONTROL_NODES.extend(CUSTOM_DECORATOR_NODES)
