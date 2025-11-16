# py_bt_ros/scenarios/nav_scenario/bt_nodes.py

from modules.base_bt_nodes import (
    Node,
    Status,
    Sequence,
    BTNodeList as BaseBTNodeList
)
from modules.base_bt_nodes_ros import (
    ActionWithROSAction,
    ActionWithROSService,
)

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger


# ============================================
# 1) WaitForGoal : /bt/goal_pose 받으면 SUCCESS
# ============================================
class WaitForGoal(Node):
    def __init__(self, name, agent, goal_topic="/bt/goal_pose"):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.goal_msg = None

        # goal pose subscribe
        self.ros.node.create_subscription(
            PoseStamped,
            goal_topic,
            self._cb,
            10
        )

    def _cb(self, msg):
        self.goal_msg = msg

    async def run(self, agent, blackboard):
        if self.goal_msg is None:
            self.status = Status.RUNNING
            return self.status

        # save goal pose
        blackboard["goal_pose"] = self.goal_msg

        # save initial pose once
        if "initial_pose" not in blackboard:
            init = PoseStamped()
            init.header.frame_id = "map"
            init.pose.position.x = 0.0
            init.pose.position.y = 0.0
            init.pose.orientation.w = 1.0
            blackboard["initial_pose"] = init

        self.status = Status.SUCCESS
        return self.status


# ============================================
# 2) MoveToGoal : Nav2 Action 요청
# ============================================
class MoveToGoal(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        pose: PoseStamped = blackboard.get("goal_pose")
        if pose is None:
            return None

        # Nav2가 원하는 Goal 구조 (중요!)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        return Status.SUCCESS


# ============================================
# 3) CaptureImage : Trigger 서비스 호출
# ============================================
class CaptureImage(ActionWithROSService):
    def __init__(self, name, agent, service_name="/capture_image"):
        super().__init__(name, agent, (Trigger, service_name))

    def _build_request(self, agent, blackboard):
        return Trigger.Request()

    def _interpret_response(self, response, agent, blackboard):
        return Status.SUCCESS


# ============================================
# 4) ReturnToStart : Nav2 Action 복귀
# ============================================
class ReturnToStart(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        pose: PoseStamped = blackboard.get("initial_pose")
        if pose is None:
            return None

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        return Status.SUCCESS


# ============================================
# BT Node Registration (중요)
# ============================================
class BTNodeList:
    CONTROL_NODES = BaseBTNodeList.CONTROL_NODES    # Sequence 포함
    ACTION_NODES = [
        "WaitForGoal",
        "MoveToGoal",
        "CaptureImage",
        "ReturnToStart",
    ]
    CONDITION_NODES = []
    DECORATOR_NODES = []
