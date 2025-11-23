import math
from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback

# BT Node List
CUSTOM_ACTION_NODES = [
    'MoveToGoal',
    'CaptureImage',
    'ReturnHome',

    # --- 새로 추가하는 goal 설정 노드들 ---
    'SetPickupGoal',
    'SetRedDropGoal',
    'SetGreenDropGoal',
    'SetBlueDropGoal',
]

CUSTOM_CONDITION_NODES = [
    'InitNavGoal',

    # --- 새로 추가하는 색 관련 조건 노드들 ---
    'WaitForParcelColor',
    'IsParcelRed',
    'IsParcelGreen',
    'IsParcelBlue',
]

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus
from std_msgs.msg import String  # 색 정보 토픽
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction, ActionWithROSService



class InitNavGoal(ConditionWithROSTopics):
    """
    /bt/goal_pose 와 /amcl_pose 를 받아서
    - blackboard['goal_pose']
    - blackboard['initial_pose']
    를 세팅하는 조건 노드.
    둘 다 한 번 이상 들어오면 SUCCESS.
    """

    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""

        # /bt/goal_pose: PoseStamped (예: RViz2에서 Publish)
        goal_topic = "/bt/goal_pose"

        # Nav2 기본 amcl_pose 는 "/amcl_pose"
        amcl_topic = f"{ns}/amcl_pose" if ns else "/amcl_pose"

        super().__init__(name, agent, [
            (PoseStamped, goal_topic, 'goal'),
            (PoseWithCovarianceStamped, amcl_topic, 'amcl'),
        ])

    def _predicate(self, agent, blackboard):
        cache = self._cache

        if 'goal' not in cache or 'amcl' not in cache:
            # 아직 둘 중 하나라도 안 들어왔으면 조건 불만족
            return False

        goal_msg: PoseStamped = cache['goal']
        amcl_msg: PoseWithCovarianceStamped = cache['amcl']

        # goal_pose 그대로 저장
        blackboard['goal_pose'] = goal_msg

        # amcl_pose 를 PoseStamped 로 변환해서 initial_pose 로 저장
        init = PoseStamped()
        init.header.frame_id = amcl_msg.header.frame_id
        init.header.stamp = amcl_msg.header.stamp
        init.pose = amcl_msg.pose.pose
        blackboard['initial_pose'] = init

        # 필요하면 debug 로그
        self.ros.node.get_logger().info(
            f"[InitNavGoal] goal=({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}), "
            f"initial=({init.pose.position.x:.2f}, {init.pose.position.y:.2f})"
        )

        return True


class MoveToGoal(ActionWithROSAction):
    """
    블랙보드의 'goal_pose'(PoseStamped)를 가져와
    Nav2 /navigate_to_pose 액션에 전달하는 노드.
    """

    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        action_name = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, bb):
        if 'goal_pose' not in bb:
            self.ros.node.get_logger().error("[MoveToGoal] goal_pose not in blackboard")
            return None

        goal_pose: PoseStamped = bb['goal_pose']
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['move_result'] = 'succeeded'
            self.ros.node.get_logger().info("[MoveToGoal] Nav2 succeeded")
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            bb['move_result'] = 'canceled'
            self.ros.node.get_logger().warn("[MoveToGoal] Nav2 canceled")
            return Status.FAILURE
        else:
            bb['move_result'] = f'aborted({status_code})'
            self.ros.node.get_logger().error(f"[MoveToGoal] Nav2 aborted with status {status_code}")
            return Status.FAILURE


class CaptureImage(ActionWithROSService):
    """
    /capture_image (std_srvs/Trigger) 서비스를 호출해서
    사진을 저장하는 노드.
    """

    def __init__(self, name, agent):
        super().__init__(name, agent, (Trigger, "/capture_image"))

    def _build_request(self, agent, blackboard):
        return Trigger.Request()

    def _interpret_response(self, response, agent, blackboard):
        if response.success:
            blackboard['capture_result'] = response.message
            self.ros.node.get_logger().info(f"[CaptureImage] {response.message}")
            return Status.SUCCESS
        else:
            self.ros.node.get_logger().error(f"[CaptureImage] failed: {response.message}")
            return Status.FAILURE


class ReturnHome(ActionWithROSAction):
    """
    블랙보드의 'initial_pose'(PoseStamped)를 사용해
    다시 원래 위치로 돌아가는 Nav2 액션 노드.
    """

    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        action_name = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, bb):
        if 'initial_pose' not in bb:
            self.ros.node.get_logger().error("[ReturnHome] initial_pose not in blackboard")
            return None

        init_pose: PoseStamped = bb['initial_pose']
        goal = NavigateToPose.Goal()
        goal.pose = init_pose
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['return_result'] = 'succeeded'
            self.ros.node.get_logger().info("[ReturnHome] returned to initial pose")
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            bb['return_result'] = 'canceled'
            self.ros.node.get_logger().warn("[ReturnHome] Nav2 canceled")
            return Status.FAILURE
        else:
            bb['return_result'] = f'aborted({status_code})'
            self.ros.node.get_logger().error(f"[ReturnHome] Nav2 aborted with status {status_code}")
            return Status.FAILURE


from std_msgs.msg import String
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction, ActionWithROSService
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus


class WaitForParcelColor(ConditionWithROSTopics):
    """
    /parcel_color (std_msgs/String)을 구독해서
    - blackboard['parcel_color'] 에 현재 색을 기록하고
    - RED / GREEN / BLUE 중 하나면 SUCCESS,
      없거나 UNKNOWN이면 FAILURE 를 반환하는 조건 노드.
    """

    def __init__(self, name, agent):
        # 네임스페이스와 무관하게 /parcel_color 를 본다고 가정
        topic = "/parcel_color"
        super().__init__(name, agent, [
            (String, topic, 'parcel_color')
        ])

    def _predicate(self, agent, blackboard):
        cache = self._cache

        if 'parcel_color' not in cache:
            # 아직 메시지가 안 들어왔으면 조건 불만족
            return False

        msg: String = cache['parcel_color']
        color = msg.data.strip().upper()

        # 블랙보드에 저장
        blackboard['parcel_color'] = color

        # 디버그용 로그
        self.ros.node.get_logger().info(
            f"[WaitForParcelColor] parcel_color = '{color}'"
        )

        # 실제 색이 아직 인식 안되었으면 실패로 처리
        if color not in ("RED", "GREEN", "BLUE"):
            return False

        return True


class IsParcelRed(ConditionWithROSTopics):
    """
    /parcel_color == 'RED' 인지 확인하는 Condition 노드.
    """

    def __init__(self, name, agent):
        topic = "/parcel_color"
        super().__init__(name, agent, [
            (String, topic, 'parcel_color')
        ])

    def _predicate(self, agent, blackboard):
        cache = self._cache
        if 'parcel_color' not in cache:
            return False

        msg: String = cache['parcel_color']
        color = msg.data.strip().upper()

        # 항상 블랙보드 동기화
        blackboard['parcel_color'] = color

        self.ros.node.get_logger().info(
            f"[IsParcelRed] parcel_color = '{color}'"
        )

        return color == "RED"


class IsParcelGreen(ConditionWithROSTopics):
    """
    /parcel_color == 'GREEN' 인지 확인하는 Condition 노드.
    """

    def __init__(self, name, agent):
        topic = "/parcel_color"
        super().__init__(name, agent, [
            (String, topic, 'parcel_color')
        ])

    def _predicate(self, agent, blackboard):
        cache = self._cache
        if 'parcel_color' not in cache:
            return False

        msg: String = cache['parcel_color']
        color = msg.data.strip().upper()
        blackboard['parcel_color'] = color

        self.ros.node.get_logger().info(
            f"[IsParcelGreen] parcel_color = '{color}'"
        )

        return color == "GREEN"


class IsParcelBlue(ConditionWithROSTopics):
    """
    /parcel_color == 'BLUE' 인지 확인하는 Condition 노드.
    """

    def __init__(self, name, agent):
        topic = "/parcel_color"
        super().__init__(name, agent, [
            (String, topic, 'parcel_color')
        ])

    def _predicate(self, agent, blackboard):
        cache = self._cache
        if 'parcel_color' not in cache:
            return False

        msg: String = cache['parcel_color']
        color = msg.data.strip().upper()
        blackboard['parcel_color'] = color

        self.ros.node.get_logger().info(
            f"[IsParcelBlue] parcel_color = '{color}'"
        )

        return color == "BLUE"


# === 택배 픽업/드랍존 좌표 설정 ===
# (x, y, yaw[rad])
DELIVERY_POINTS = {
    "PICKUP":     (0.0, 0.0, 0.0),   # 집하 위치
    "DROP_RED":   (1.0, 0.0, 0.0),   # 빨강 드랍존
    "DROP_GREEN": (0.0, 1.0, 0.0),   # 초록 드랍존
    "DROP_BLUE":  (-1.0, 0.0, 0.0),  # 파랑 드랍존
}


def _make_pose_stamped(x, y, yaw, frame_id="map"):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    # stamp는 굳이 안 넣어도 됨 (0이면 Nav2가 그냥 써줌)

    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = 0.0

    # yaw -> quaternion (roll=pitch=0 가정)
    ps.pose.orientation.z = math.sin(yaw * 0.5)
    ps.pose.orientation.w = math.cos(yaw * 0.5)
    return ps


class SetPickupGoal(Node):
    """
    DELIVERY_POINTS['PICKUP'] 좌표를 goal_pose로 설정하는 노드.
    집하 장소로 이동하기 전에 사용.
    """
    def __init__(self, name, agent):
        super().__init__(name, agent)

    def tick(self, agent, blackboard):
        x, y, yaw = DELIVERY_POINTS["PICKUP"]
        pose = _make_pose_stamped(x, y, yaw)
        blackboard['goal_pose'] = pose

        agent.ros.node.get_logger().info(
            f"[SetPickupGoal] goal_pose = ({x:.2f}, {y:.2f}, {yaw:.2f})"
        )
        return Status.SUCCESS


class SetRedDropGoal(Node):
    """
    빨간 택배 드랍존(DROP_RED) 좌표를 goal_pose로 설정하는 노드.
    """
    def __init__(self, name, agent):
        super().__init__(name, agent)

    def tick(self, agent, blackboard):
        x, y, yaw = DELIVERY_POINTS["DROP_RED"]
        pose = _make_pose_stamped(x, y, yaw)
        blackboard['goal_pose'] = pose

        agent.ros.node.get_logger().info(
            f"[SetRedDropGoal] goal_pose = ({x:.2f}, {y:.2f}, {yaw:.2f})"
        )
        return Status.SUCCESS


class SetGreenDropGoal(Node):
    """
    초록 택배 드랍존(DROP_GREEN) 좌표를 goal_pose로 설정하는 노드.
    """
    def __init__(self, name, agent):
        super().__init__(name, agent)

    def tick(self, agent, blackboard):
        x, y, yaw = DELIVERY_POINTS["DROP_GREEN"]
        pose = _make_pose_stamped(x, y, yaw)
        blackboard['goal_pose'] = pose

        agent.ros.node.get_logger().info(
            f"[SetGreenDropGoal] goal_pose = ({x:.2f}, {y:.2f}, {yaw:.2f})"
        )
        return Status.SUCCESS


class SetBlueDropGoal(Node):
    """
    파랑 택배 드랍존(DROP_BLUE) 좌표를 goal_pose로 설정하는 노드.
    """
    def __init__(self, name, agent):
        super().__init__(name, agent)

    def tick(self, agent, blackboard):
        x, y, yaw = DELIVERY_POINTS["DROP_BLUE"]
        pose = _make_pose_stamped(x, y, yaw)
        blackboard['goal_pose'] = pose

        agent.ros.node.get_logger().info(
            f"[SetBlueDropGoal] goal_pose = ({x:.2f}, {y:.2f}, {yaw:.2f})"
        )
        return Status.SUCCESS

