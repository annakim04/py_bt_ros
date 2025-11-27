import rclpy
from rclpy.node import Node
from rclpy.action import (
    ActionServer,
    ActionClient,
    GoalResponse,
    CancelResponse,
)
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

result = NavigateToPose.Result()

if status == GoalStatus.STATUS_SUCCEEDED:
    self.get_logger().info("Nav2 navigation succeeded.")
    result.error_code = 0        # NONE = 0
    result.error_msg = ""
    goal_handle.succeed()
else:
    self.get_logger().warn(f"Nav2 navigation failed. status={status}")
    result.error_code = status   # 대충 상태 코드 넣거나, 너가 원하는 값
    result.error_msg = "Nav2 failed"
    goal_handle.abort()

return result



class LimoNavigateServer(Node):
    """
    BT <-> Nav2 사이를 중계하는 액션 서버.

    - 이 노드가 띄우는 액션 서버:
        {ns}/navigate_to_pose   (BT에서 여기로 goal 보냄)

    - 내부에서 사용하는 Nav2 액션 서버:
        nav2_action_name        (기본: '/navigate_to_pose')

    동작:
      1) BT에서 goal(NavigateToPose.Goal)을 받으면
      2) Nav2의 /navigate_to_pose 에 똑같이 goal을 forward
      3) Nav2 결과를 기다렸다가 그대로 BT 쪽에 결과 전달
    """

    def __init__(self, ns: str = "/limo", nav2_action_name: str = "/navigate_to_pose"):
        super().__init__("limo_navigate_server")

        # BT가 사용할 네임스페이스
        self.ns = ns.rstrip("/")
        self.action_name = f"{self.ns}/navigate_to_pose"

        # Nav2 NavigateToPose 액션 클라이언트
        self.nav2_action_name = nav2_action_name
        self.nav2_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

        # BT에서 사용할 액션 서버
        self.server = ActionServer(
            self,
            NavigateToPose,
            self.action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info(f"[LimoNavigateServer] BT action server: {self.action_name}")
        self.get_logger().info(f"[LimoNavigateServer] Nav2 action client: {self.nav2_action_name}")

    # --- goal / cancel 콜백 ---

    def goal_cb(self, goal_request: NavigateToPose.Goal):
        # 간단히 모두 ACCEPT (필요하면 frame_id 체크 등 추가 가능)
        self.get_logger().info("Received new goal from BT.")
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        # BT에서 cancel 요청 오면 Nav2 쪽에도 cancel 전달할 것
        self.get_logger().info("Received cancel request from BT.")
        return CancelResponse.ACCEPT

    # --- 핵심 실행 콜백 ---

    async def execute_cb(self, goal_handle):
        """
        BT에서 NavigateToPose.Goal을 받았을 때 호출되는 부분.
        Nav2 액션 서버에 그대로 goal을 포워딩하고 결과를 기다린다.
        """
        bt_goal: NavigateToPose.Goal = goal_handle.request

        # Nav2 서버 준비될 때까지 대기
        self.get_logger().info("Waiting for Nav2 NavigateToPose action server...")
        if not self.nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 NavigateToPose server not available!")
            goal_handle.abort()
            result = NavigateToPose.Result()
            result.result = Empty()
            return result

        self.get_logger().info(
            f"Sending goal to Nav2: "
            f"({bt_goal.pose.pose.position.x:.2f}, "
            f"{bt_goal.pose.pose.position.y:.2f})"
        )

        # Nav2로 goal 전송 (feedback 콜백에서 BT에 피드백 전달)
        nav2_send_future = self.nav2_client.send_goal_async(
            bt_goal,
            feedback_callback=lambda fb: self._on_nav2_feedback(fb, goal_handle),
        )

        nav2_goal_handle = await nav2_send_future

        if not nav2_goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the goal.")
            goal_handle.abort()
            result = NavigateToPose.Result()
            result.result = Empty()
            return result

        self.get_logger().info("Nav2 goal accepted. Waiting for result...")

        # BT에서 cancel 요청이 들어오면 Nav2에도 cancel
        if goal_handle.is_cancel_requested:
            self.get_logger().info("BT requested cancel. Cancelling Nav2 goal...")
            cancel_future = nav2_goal_handle.cancel_goal_async()
            await cancel_future
            goal_handle.canceled()
            result = NavigateToPose.Result()
            result.result = Empty()
            return result

        # Nav2 결과 기다리기
        nav2_result_future = nav2_goal_handle.get_result_async()
        nav2_result = await nav2_result_future

        # Nav2 결과 상태에 따라 BT goal 상태 설정
        status = nav2_result.status
        # status 값은 rclpy.action.GoalStatus.STATUS_SUCCEEDED 등 사용 가능
        # 여기서는 단순히 SUCCEEDED만 성공 처리
        from action_msgs.msg import GoalStatus

        result = NavigateToPose.Result()
        result.result = Empty()

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 navigation succeeded.")
            goal_handle.succeed()
        else:
            self.get_logger().warn(f"Nav2 navigation failed. status={status}")
            goal_handle.abort()

        return result

    # --- Nav2 피드백 -> BT 피드백 포워딩 ---

    def _on_nav2_feedback(self, nav2_feedback_msg, bt_goal_handle):
        """
        Nav2 액션 서버에서 넘어온 피드백을
        그대로 BT 쪽 액션 서버 피드백으로 전달.
        """
        feedback = nav2_feedback_msg.feedback  # type: NavigateToPose.Feedback
        bt_goal_handle.publish_feedback(feedback)


def main():
    rclpy.init()
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--ns", type=str, default="/limo", help="BT 쪽에서 사용할 네임스페이스 (ex. /limo)")
    parser.add_argument(
        "--nav2_action",
        type=str,
        default="/navigate_to_pose",
        help="Nav2 NavigateToPose 액션 이름",
    )
    args = parser.parse_args()

    node = LimoNavigateServer(ns=args.ns, nav2_action_name=args.nav2_action)

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
