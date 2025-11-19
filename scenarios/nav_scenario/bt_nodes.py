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
# 3) CaptureImage : Trigger 서비스 호출
# ============================================
class CaptureImage(ActionWithROSService):
    def __init__(self, name, agent, service_name="/capture_image"):
        super().__init__(name, agent, (Trigger, service_name))

    def _build_request(self, agent, blackboard):
        return Trigger.Request()

    def _interpret_response(self, response, agent, blackboard):
        return Status.SUCCESS





class BTNodeList:
    CONTROL_NODES = BaseBTNodeList.CONTROL_NODES    # Sequence 포함
    ACTION_NODES = [
        "CaptureImage",
    ]
    CONDITION_NODES = []
    DECORATOR_NODES = []