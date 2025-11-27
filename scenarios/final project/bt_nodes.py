import math
from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback

# BT Node List
CUSTOM_ACTION_NODES = [
    'SetWp1Goal',
    'SetWp2Goal',
    'SetWp3Goal',
    'SetWp4Goal',
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

from std_msgs.msg import String
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction, ActionWithROSService
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus


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


WAYPOINTS = {
    "WP1": (0.0, 0.0, 0.0),      # 첫 번째 좌표
    "WP2": (1.0, 0.0, 0.0),      # 두 번째 좌표
    "WP3": (1.0, 1.0, 1.5708),   # 세 번째 좌표
    "WP4": (0.0, 1.0, 3.1416),   # 네 번째 좌표
}


class SetWp1Goal(Node):
    """WAYPOINTS['WP1']를 goal_pose에 세팅"""
    def __init__(self, name, agent):
        super().__init__(name)
        self.agent = agent

    def tick(self, agent, blackboard):
        x, y, yaw = WAYPOINTS["WP1"]
        pose = _make_pose_stamped(x, y, yaw)
        blackboard["goal_pose"] = pose

        agent.ros.node.get_logger().info(
            f"[SetWp1Goal] goal_pose = ({x:.2f}, {y:.2f}, {yaw:.2f})"
        )
        return Status.SUCCESS


class SetWp2Goal(Node):
    """WAYPOINTS['WP2']를 goal_pose에 세팅"""
    def __init__(self, name, agent):
        super().__init__(name)
        self.agent = agent

    def tick(self, agent, blackboard):
        x, y, yaw = WAYPOINTS["WP2"]
        pose = _make_pose_stamped(x, y, yaw)
        blackboard["goal_pose"] = pose

        agent.ros.node.get_logger().info(
            f"[SetWp2Goal] goal_pose = ({x:.2f}, {y:.2f}, {yaw:.2f})"
        )
        return Status.SUCCESS


class SetWp3Goal(Node):
    """WAYPOINTS['WP3']를 goal_pose에 세팅"""
    def __init__(self, name, agent):
        super().__init__(name)
        self.agent = agent

    def tick(self, agent, blackboard):
        x, y, yaw = WAYPOINTS["WP3"]
        pose = _make_pose_stamped(x, y, yaw)
        blackboard["goal_pose"] = pose

        agent.ros.node.get_logger().info(
            f"[SetWp3Goal] goal_pose = ({x:.2f}, {y:.2f}, {yaw:.2f})"
        )
        return Status.SUCCESS


class SetWp4Goal(Node):
    """WAYPOINTS['WP4']를 goal_pose에 세팅"""
    def __init__(self, name, agent):
        super().__init__(name)
        self.agent = agent

    def tick(self, agent, blackboard):
        x, y, yaw = WAYPOINTS["WP4"]
        pose = _make_pose_stamped(x, y, yaw)
        blackboard["goal_pose"] = pose

        agent.ros.node.get_logger().info(
            f"[SetWp4Goal] goal_pose = ({x:.2f}, {y:.2f}, {yaw:.2f})"
        )
        return Status.SUCCESS

