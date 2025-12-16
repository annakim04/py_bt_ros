import math
import time
import rclpy
import json

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose, Spin
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatus

from std_msgs.msg import String, Bool, UInt8

from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveFallback, ReactiveSequence
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction


# =========================================================
# UI 문구 (화면에 그대로 보여질 문자열)
# =========================================================
UI_TITLE = "[DELIVERY STATUS]"

UI_DELIVERY_START      = "DELIVERY START"
UI_ARRIVED_PICKUP      = "Arrived PICK-UP Location"
UI_IN_TRANSIT          = "In TRANSIT"
UI_PACKAGE_NOT_DETECTED   = "Package NOT Detected"
UI_PICKUP_FAILED       = "PICK-UP FAILED"
UI_DELIVERY_COMPLETED  = "DELIVERY COMPLETED"
UI_BACK_TO_PICKUP      = "Back to PICK-UP Location"

# =========================================================
# UI 토픽
# =========================================================
TOPIC_DELIVERY_STATUS    = "/delivery/status"
TOPIC_DELIVERY_ARRIVED   = "/delivery/arrived"
TOPIC_DELIVERY_DELIVERED = "/delivery/delivered"

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
CHARGE_X, CHARGE_Y, CHARGE_YAW = 0.776, 4.081, deg(-74.4)
PICKUP_X, PICKUP_Y, PICKUP_YAW = -0.194, 0.0007, deg(-5.2)
WAIT_X, WAIT_Y, WAIT_YAW = 2.979, 2.922, deg(11.8)

NAV_ACTION_NAME = "/limo/navigate_to_pose"

# =========================================================
# Topics
# =========================================================
PARCEL_AVAILABLE_TOPIC = "/parcel_available"
RECEIVE_BUSY_TOPIC = "/receive_busy"
DROPOFF_BUSY_TOPIC = "/dropoff_busy"

class DeliveryPublishMixin:
    def _ensure_publishers(self):
        if getattr(self, "_delivery_pub_ready", False):
            return
        node = self.ros.node
        self.pub_status    = node.create_publisher(String, TOPIC_DELIVERY_STATUS, 10)
        self.pub_arrived   = node.create_publisher(Bool,   TOPIC_DELIVERY_ARRIVED, 10)
        self.pub_delivered = node.create_publisher(Bool,   TOPIC_DELIVERY_DELIVERED, 10)
        self._delivery_pub_ready = True

    def publish_status(self, state: str, destination: str = "N/A"):
        self._ensure_publishers()
        msg = String()
        msg.data = json.dumps({
            "state": state,
            "destination": destination
        }, ensure_ascii=False)
        self.pub_status.publish(msg)

    def publish_arrived(self, v: bool):
        self._ensure_publishers()
        self.pub_arrived.publish(Bool(data=v))

    def publish_delivered(self, v: bool):
        self._ensure_publishers()
        self.pub_delivered.publish(Bool(data=v))

class AlwaysFailure(Node):
    def __init__(self, node_name, agent, name=None):
        # 프레임워크가 보내주는 인자(node_name, agent, name)를 모두 받아야 합니다.
        final_name = name if name else node_name
        super().__init__(final_name)
        self.type = "Action"  # 액션 타입 명시

    async def run(self, agent, blackboard):
        return Status.FAILURE        


# =========================================================
# Decorators (원본 유지)
# =========================================================
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


class Timeout(Node, DeliveryPublishMixin):
    def __init__(self, name, child, duration=10.0):
        super().__init__(name)
        self.child = child
        self.duration = float(duration)
        self.start_time = None
        self.is_running = False
        self.type = "Decorator"
        self.ros = None

    async def run(self, agent, blackboard):
        if self.ros is None:           
            self.ros = agent.ros_bridge

        if not self.is_running or self.start_time is None:
            self.start_time = time.time()
            self.is_running = True
            print(f"[{self.name}] Timer Started ({self.duration}s)")

        elapsed = time.time() - self.start_time

        if int(elapsed * 10) % 10 == 0:
            print(f"[{self.name}] ... {elapsed:.1f}s / {self.duration}s")

        if elapsed > self.duration:
            print(f"[{self.name}] TIMEOUT")

            if hasattr(self.child, "halt"):
                self.child.halt()
            self.is_running = False
            self.status = Status.FAILURE
            return Status.FAILURE

        result = await self.child.run(agent, blackboard)
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
        self._last_value = False  # 마지막 상태 기억

    def _predicate(self, agent, blackboard):
        if "parcel_available" in self._cache:
            new_value = self._cache["parcel_available"].data

            # 값이 바뀌었을 때만 교체
            if new_value != self._last_value:
                print(
                    f"[{self.name}] parcel_available changed: "
                    f"{self._last_value} → {new_value}"
                )
                self._last_value = new_value

        # 메시지가 없으면 마지막 값 유지
        return self._last_value

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

    async def run(self, agent, blackboard):
        msg = self._cache.get("drop_busy")
        if msg is None:
            return Status.FAILURE
        return Status.SUCCESS if msg.data else Status.FAILURE



class ReceiveParcel(ConditionWithROSTopics, DeliveryPublishMixin): #택배 수령 여부를 판단하는 노드########
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [(UInt8, "/limo/button_status", "button_state")])
        self.ros = agent.ros_bridge  #######################!!!!!!!!!!!!!!!!
        self.receive_busy_pub = agent.ros_bridge.node.create_publisher(
            Bool, RECEIVE_BUSY_TOPIC, 10
        )
        self.cleared = False        
        
    def _predicate(self, agent, blackboard):
        #데이터가 아예 안 들어온 경우
        if "button_state" not in self._cache:
            print(f"[{self.name}] Waiting for /limo/button data") 
            return False
        
        # 데이터가 들어온 경우 내용 확인함
        msg = self._cache["button_state"]
        raw_data = msg.data
        
        #현재 버튼 상태 출력
        print(f"[{self.name}] Button State: '{raw_data}'")
        
        if raw_data == 1:
            print(f"[{self.name}] Button PRESSED! Moving to next step.")
            if not self.cleared:
                self.receive_busy_pub.publish(Bool(data=False))
                self.cleared = True
                print("[ReceiveParcel] /receive_busy = false (picked up)")
            
            
            # 버튼 확인 후 캐시 삭제 (한번 누르면 소모)
            del self._cache["button_state"]
            return True
            
        return False



class DropoffParcel(ConditionWithROSTopics):#택배 배달 여부를 판단하는 노드  
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [(UInt8, "/limo/button_status", "button_state")])
        self.ros = agent.ros_bridge#####################
        self.dropoff_busy_pub = agent.ros_bridge.node.create_publisher(
            Bool, DROPOFF_BUSY_TOPIC, 10
        )
        self.cleared = False        

    def _predicate(self, agent, blackboard):
        #데이터가 아예 안 들어온 경우
        if "button_state" not in self._cache:
            print(f"[{self.name}] Waiting for /limo/button data") 
            return False
        
        state = self._cache["button_state"].data
        if state == 0:
            print(f"[{self.name}] Button released! Delivery completed.")

            if not self.cleared:
                self.dropoff_busy_pub.publish(Bool(data=False))
                self.cleared = True
                print("[DropoffParcel] /dropoff_busy = false (completed)")


            # 버튼 확인 후 캐시 삭제 (한번 누르면 소모)
            del self._cache["button_state"]
            return True
        
        return False


class WaitForQRPose(ConditionWithROSTopics): #배달 장소 인식 여부를 판단하는 노드
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [(PoseStamped, "/qr_warehouse_pose", "qr_pose"),(String, "/qr_warehouse", "warehouse_name"),],)

    def _predicate(self, agent, blackboard):
        #데이터가 아예 안 들어온 경우
        if "qr_pose" not in self._cache:
            print(f"[{self.name}] Waiting for QR data")
            return False
        
        '''
        warehouse = "_"
        if "warehouse_name" in self._cache:
            warehouse = self._cache["warehouse_name"].data.strip()
            # 한 번만 쓰고 싶으면 소모
            del self._cache["warehouse_name"]
        '''
        if "warehouse_name" not in self._cache:
            print(f"[{self.name}] Waiting for /qr_warehouse String")
            return False

        # 3) 둘 다 있으면 읽어서 저장
        warehouse = self._cache["warehouse_name"].data.strip()
        del self._cache["warehouse_name"]



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
        blackboard["qr_destination"] = warehouse

        # 4. 사용한 데이터 삭제 (한 번만 인식하고 넘어가기 위해)
        del self._cache["qr_pose"]
        return True

class IsButtonPressed(ConditionWithROSTopics): #택배 운송 여부를 판단하는 노드
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [(UInt8, "/limo/button_status", "button_state")])

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache: 
            return False # 데이터 없으면 안 눌린 것으로 간주
        
        data = self._cache["button_state"].data
        # pressed 상태면 True, 아니면 False (데이터를 지우지 않음!)
        return (data == 1)

class MoveToCharge(ActionWithROSAction, DeliveryPublishMixin):
    #Nav2에 보낼 요청 생성
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, (NavigateToPose, NAV_ACTION_NAME))
        self.ros = agent.ros_bridge

    #백보드에 저장한 x,y좌표로 이동하는 요청을 Nav2에 전송
    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] Moving to Charging Station")
        self.publish_status(UI_PACKAGE_NOT_DETECTED)
        return _create_nav_goal(self.ros.node, CHARGE_X, CHARGE_Y, CHARGE_YAW)


class MoveToPickup(ActionWithROSAction, DeliveryPublishMixin): ##############
    #Nav2에 보낼 요청 생성
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, (NavigateToPose, NAV_ACTION_NAME))
        self.ros = agent.ros_bridge###############

        self.receive_busy_pub = agent.ros_bridge.node.create_publisher(
            Bool, RECEIVE_BUSY_TOPIC, 10
        )
        self.busy_sent = False

    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] Moving to Pickup Point")
        self.publish_status(UI_DELIVERY_START)
        return _create_nav_goal(self.ros.node, PICKUP_X, PICKUP_Y, PICKUP_YAW)


    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            if not self.busy_sent:
                self.receive_busy_pub.publish(Bool(data=True))
                self.busy_sent = True
                print("[MoveToPickup] /receive_busy = true")
            return Status.SUCCESS

        return Status.FAILURE


class MoveToDelivery(ActionWithROSAction, DeliveryPublishMixin):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, (NavigateToPose, NAV_ACTION_NAME))
        self.ros = agent.ros_bridge

        self.dropoff_busy_pub = agent.ros_bridge.node.create_publisher(
            Bool, DROPOFF_BUSY_TOPIC, 10
        )

        self.busy_cleared = False

    def _build_goal(self, agent, blackboard):
        qr_pose = blackboard.get("qr_target_pose")
        if qr_pose is None: 
            print(f"[{self.name}] ERROR: No QR Pose in blackboard")
            return None
        dest = blackboard.get("qr_destination", "_")
        self.publish_status(UI_IN_TRANSIT, dest)

        if not self.busy_cleared:
            self.dropoff_busy_pub.publish(Bool(data=False))
            self.busy_cleared = True
            print("[MoveToDelivery] /droppoff_busy = false")

        print(f"[{self.name}] Moving to Delivery Point (from QR)")
        return _create_nav_goal(self.ros.node, 0, 0, pose_stamped=qr_pose)

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            if not self.busy_cleared:
                self.dropoff_busy_pub.publish(Bool(data=True))
                self.busy_cleared = True
                print("[MoveToDelivery] /dropoff_busy = true (arrived)")            
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
        print(f"[{self.name}] Moving to Waiting Area(for pickup)")
        return _create_nav_goal(
            self.ros.node, WAIT_X, WAIT_Y, WAIT_YAW
        )

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        return Status.RUNNING

class MoveToWaitingDrop(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            (NavigateToPose, NAV_ACTION_NAME),
        )

    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] Moving to Waiting Area(for dropoff)")
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
        goal.target_yaw = 1.57  # 90도 회전 
        return goal


# =========================================================
# Registration
# =========================================================
CUSTOM_ACTION_NODES = ["MoveToCharge", "MoveToPickup", "MoveToDelivery", "MoveToPickupWaiting", "MoveToWaitingDrop", "SpinInPlace", "AlwaysFailure"]

CUSTOM_CONDITION_NODES = ["ReceiveParcel", "DropoffParcel", "WaitForQRPose", "ParcelAvailable", "OtherRobotReceiving", "OtherRobotDropping", "IsButtonPressed"]

CUSTOM_DECORATOR_NODES = ["RetryUntilSuccessful", "Timeout"]

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)


if hasattr(BTNodeList, "DECORATOR_NODES"):
    BTNodeList.DECORATOR_NODES.extend(CUSTOM_DECORATOR_NODES)
else:
    BTNodeList.CONTROL_NODES.extend(CUSTOM_DECORATOR_NODES)     
