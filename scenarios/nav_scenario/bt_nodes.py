import rclpy
from rclpy.node import Node as RosNode
from rclpy.subscription import Subscription

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

# WaitForGoal : /bt/goal_pose 받으면 SUCCESS
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped

# WaitForGoal 
class WaitForGoal(Node):
    def __init__(self, name, agent, 
                 goal_topic="/bt/goal_pose", 
                 pose_topic="/amcl_pose"): 
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.goal_msg = None
        self.current_pose_msg = None # 현재 위치 저장용 변수

        self.ros.node.create_subscription(
            PoseStamped,
            goal_topic,
            self._goal_cb,
            10
        )

        self.ros.node.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self._pose_cb,
            10
        )

    def _goal_cb(self, msg):
        self.goal_msg = msg # 새 목표 수신

    def _pose_cb(self, msg):
        self.current_pose_msg = msg # 현재 위치 계속 업데이트

    async def run(self, agent, blackboard):
        if self.goal_msg is None or self.current_pose_msg is None:
            self.status = Status.RUNNING
            return self.status

        # Blackboard에 목표 저장
        blackboard["goal_pose"] = self.goal_msg

        # Blackboard에 현재 위치를 'initial_pose'로 저장
        if "initial_pose" not in blackboard:
            init = PoseStamped()
            init.header.frame_id = "map"
            init.header.stamp = self.ros.node.get_clock().now().to_msg()
            init.pose = self.current_pose_msg.pose.pose 
            blackboard["initial_pose"] = init

        self.goal_msg = None

        self.status = Status.SUCCESS
        return self.status

# MoveToGoal : Nav2 Action 요청
class MoveToGoal(ActionWithROSAction):


from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped





# ============================================
# 4) ReturnToStart : Nav2 Action 복귀
# ============================================
class ReturnToStart(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        pose: PoseStamped = blackboard.get("goal_pose")
        if pose is None:
            return None

        # Nav2가 원하는 Goal 구조
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
        "ReturnToStart",
    ]
    CONDITION_NODES = []
    DECORATOR_NODES = []
