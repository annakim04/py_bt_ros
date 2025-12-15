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
    import asyncio # 파일 맨 위에 이거 추가해야 함!

    async def execute_cb(self, goal_handle):
        """
        [수정된 버전] 이동 중에도 취소 요청을 실시간으로 감시합니다.
        """
        bt_goal = goal_handle.request

        # 1. Nav2 서버 연결 확인
        if not self.nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 server not available!")
            goal_handle.abort()
            return NavigateToPose.Result()

        self.get_logger().info("Forwarding goal to Nav2...")

        # 2. Nav2에게 목표 전송
        nav2_send_future = self.nav2_client.send_goal_async(
            bt_goal,
            feedback_callback=lambda fb: self._on_nav2_feedback(fb, goal_handle)
        )
        nav2_goal_handle = await nav2_send_future

        if not nav2_goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the goal.")
            goal_handle.abort()
            return NavigateToPose.Result()

        self.get_logger().info("Nav2 Moving... (Monitoring for Cancel)")

        # 3. [핵심 수정] 결과 대기 + 취소 감시 루프
        nav2_result_future = nav2_goal_handle.get_result_async()

        # Nav2가 끝날 때까지 0.1초마다 깨어나서 확인
        while not nav2_result_future.done():
            
            # (1) 행동 트리가 취소를 요청했는지 확인!
            if goal_handle.is_cancel_requested:
                self.get_logger().info("✋ BT Cancel Requested! Stopping Nav2...")
                
                # Nav2에게도 취소 명령 전송
                cancel_future = nav2_goal_handle.cancel_goal_async()
                await cancel_future
                
                # BT에게 취소 완료 보고
                goal_handle.canceled()
                
                # 빈 결과 반환하며 종료
                result = NavigateToPose.Result()
                result.result = Empty()
                return result

            # (2) 아직 안 끝났으면 0.1초 대기 (CPU 양보)
            await asyncio.sleep(0.1)

        # 4. Nav2 이동 종료 후 결과 처리
        nav2_result = nav2_result_future.result()
        status = nav2_result.status
        
        result = NavigateToPose.Result()
        result.result = Empty()

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 Arrived Succeeded.")
            goal_handle.succeed()
        else:
            self.get_logger().warn(f"Nav2 Failed/Cancelled. Status: {status}")
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
