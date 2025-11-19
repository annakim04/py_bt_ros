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



from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped





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
        "ReturnToStart",
    ]
    CONDITION_NODES = []
    DECORATOR_NODES = []