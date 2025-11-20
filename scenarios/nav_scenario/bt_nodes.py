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

# 1) WaitForGoal : /bt/goal_pose 받으면 SUCCESS
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped

# 1) WaitForGoal : /bt/goal_pose 받고, /amcl_pose 저장
class WaitForGoal(Node):
    def __init__(self, name, agent, 
                 goal_topic="/bt/goal_pose", 
                 pose_topic="/amcl_pose"): 
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.goal_msg = None
        self.current_pose_msg = None # 현재 위치 저장용 변수

        # Goal Pose 구독
        self.ros.node.create_subscription(
            PoseStamped,
            goal_topic,
            self._goal_cb,
            10
        )
        
        # AMCL Pose (현재 위치) 구독
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

        # Blackboard에 *실제* 현재 위치를 'initial_pose'로 저장
        if "initial_pose" not in blackboard:
            init = PoseStamped()
            init.header.frame_id = "map"
            init.header.stamp = self.ros.node.get_clock().now().to_msg()
            # (0,0 하드코딩 대신, /amcl_pose에서 받은 실제 Pose를 사용)
            init.pose = self.current_pose_msg.pose.pose 
            blackboard["initial_pose"] = init
        
        self.goal_msg = None

        self.status = Status.SUCCESS
        return self.status

# MoveToGoal : Nav2 Action 요청
class MoveToGoal(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        pose: PoseStamped = blackboard.get("goal_pose")
        if pose is None:
            return None

        # Nav2가 원하는 Goal 구조
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        return Status.SUCCESS


