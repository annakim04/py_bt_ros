#!/usr/bin/env python3
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
from std_msgs.msg import Empty


class LimoNavigateServer(Node):
    """
    BT <-> Nav2 사이를 중계하는 액션 서버.

    - BT가 사용하는 액션 서버: {ns}/navigate_to_pose
    - 내부 Nav2 액션: /navigate_to_pose

    BT에서 NavigateToPose.Goal을 받으면 그대로 Nav2로 forward 하고
    Nav2 성공/실패 상태를 BT로 넘겨준다.
    """

    def __init__(self, ns="/limo", nav2_action_name="/navigate_to_pose"):
        super().__init__("limo_navigate_server")

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

    # -------------------------------------------------------------------------
    # goal / cancel 콜백
    # -------------------------------------------------------------------------
    def goal_cb(self, goal_request: NavigateToPose.Goal):
        self.get_logger().info("Received new goal from BT.")
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Received cancel request from BT.")
        return CancelResponse.ACCEPT

    # -------------------------------------------------------------------------
    # 핵심 execute 콜백
    # -------------------------------------------------------------------------
    async def execute_cb(self, goal_handle):
        """
        BT → this server → Nav2 로 goal을 포워딩하고, 결과를 받아 BT에 전달
        """
        bt_goal: NavigateToPose.Goal = goal_handle.request

        # 1) Nav2 서버 준비 대기
        self.get_logger().info("Waiting for Nav2 NavigateToPose action server...")
        if not self.nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 NavigateToPose server not available!")

            goal_handle.abort()
            result = NavigateToPose.Result()
            result.result = Empty()
            return result

        self.get_logger().info(
            f"Sending goal to Nav2: ({bt_goal.pose.pose.position.x:.2f}, "
            f"{bt_goal.pose.pose.position.y:.2f})"
        )

        # 2) Nav2로 goal 전송
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

        # 3) BT에서 cancel 요청 체크
        if goal_handle.is_cancel_requested:
            self.get_logger().info("BT requested cancel. Cancelling Nav2 goal...")
            cancel_future = nav2_goal_handle.cancel_goal_async()
            await cancel_future

            goal_handle.canceled()
            result = NavigateToPose.Result()
            result.result = Empty()
            return result

        # 4) Nav2 결과 기다림
        nav2_result_future = nav2_goal_handle.get_result_async()
        nav2_result = await nav2_result_future

        status = nav2_result.status
        result = NavigateToPose.Result()
        result.result = Empty()

        # 5) 성공 / 실패 판정
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 navigation succeeded.")
            goal_handle.succeed()

        else:
            self.get_logger().warn(f"Nav2 navigation failed. status={status}")
            goal_handle.abort()

        return result

    # -------------------------------------------------------------------------
    # Nav2 피드백 포워딩
    # -------------------------------------------------------------------------
    def _on_nav2_feedback(self, nav2_feedback_msg, bt_goal_handle):
        """
        Nav2 NavigateToPose 피드백을 그대로 BT에게 전달
        """
        feedback = nav2_feedback_msg.feedback  # NavigateToPose.Feedback
        bt_goal_handle.publish_feedback(feedback)


def main():
    rclpy.init()

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--ns", type=str, default="/limo")
    parser.add_argument("--nav2_action", type=str, default="/navigate_to_pose")
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
