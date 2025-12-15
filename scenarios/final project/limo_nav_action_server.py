import asyncio  # [중요] 비동기 대기(sleep)를 위해 필수
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
from rclpy.executors import MultiThreadedExecutor

class LimoNavigateServer(Node):
    """
    [LIMO 네비게이션 중계 서버]
    BT <-> Nav2 사이를 중계하며, '취소 요청(Cancel)'을 실시간으로 처리합니다.
    """

    def __init__(self, ns="/limo", nav2_action_name="/navigate_to_pose"):
        super().__init__("limo_navigate_server")

        self.ns = ns.rstrip("/")
        
        # 1. 행동 트리(BT)용 액션 서버
        self.action_name = f"{self.ns}/navigate_to_pose"

        # 2. Nav2용 액션 클라이언트
        self.nav2_action_name = nav2_action_name
        self.nav2_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

        # 3. 서버 생성
        self.server = ActionServer(
            self,
            NavigateToPose,
            self.action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info(f"[Bridge] Ready: {self.action_name} <-> {self.nav2_action_name}")

    # -------------------------------------------------------------------------
    # Goal / Cancel 수신 콜백
    # -------------------------------------------------------------------------
    def goal_cb(self, goal_request: NavigateToPose.Goal):
        self.get_logger().info("Received new goal from BT.")
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Received cancel request from BT.")
        return CancelResponse.ACCEPT

    # -------------------------------------------------------------------------
    # [핵심] 실행 콜백 (취소 감시 로직 포함)
    # -------------------------------------------------------------------------
    async def execute_cb(self, goal_handle):
        bt_goal = goal_handle.request

        # 1. Nav2 서버 연결 확인
        if not self.nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 server not available!")
            goal_handle.abort()
            result = NavigateToPose.Result()
            result.result = Empty()
            return result

        # 2. Nav2에게 목표 전송
        self.get_logger().info("Forwarding goal to Nav2...")
        nav2_send_future = self.nav2_client.send_goal_async(
            bt_goal,
            feedback_callback=lambda fb: self._on_nav2_feedback(fb, goal_handle)
        )
        nav2_goal_handle = await nav2_send_future

        if not nav2_goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the goal.")
            goal_handle.abort()
            result = NavigateToPose.Result()
            result.result = Empty()
            return result

        self.get_logger().info("Nav2 Moving... (Monitoring Cancel Request)")

        # 3. 결과 대기 + 취소 감시 루프
        nav2_result_future = nav2_goal_handle.get_result_async()

        # [중요] Nav2가 끝날 때까지 0.1초마다 깨어나서 취소 여부를 확인
        while not nav2_result_future.done():
            
            # (A) 행동 트리가 취소를 요청했는지 확인
            if goal_handle.is_cancel_requested:
                self.get_logger().info("✋ BT Cancel Requested! Stopping Nav2...")
                
                # Nav2에게 정지 명령 전송
                cancel_future = nav2_goal_handle.cancel_goal_async()
                await cancel_future
                
                # BT에게 '취소됨' 보고
                goal_handle.canceled()
                
                # 작업 종료
                result = NavigateToPose.Result()
                result.result = Empty()
                return result

            # (B) 아직 안 끝났으면 0.1초 대기 (CPU 양보)
            await asyncio.sleep(0.1)

        # 4. Nav2 이동이 정상적으로 끝난 경우 (취소 없이 도착)
        nav2_result = nav2_result_future.result()
        status = nav2_result.status
        
        result = NavigateToPose.Result()
        result.result = Empty()

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Nav2 Arrived Successfully.")
            goal_handle.succeed()
        else:
            self.get_logger().warn(f"❌ Nav2 Failed or Stopped. Status: {status}")
            goal_handle.abort()

        return result

    # -------------------------------------------------------------------------
    # 피드백 중계
    # -------------------------------------------------------------------------
    def _on_nav2_feedback(self, nav2_feedback_msg, bt_goal_handle):
        feedback = nav2_feedback_msg.feedback
        bt_goal_handle.publish_feedback(feedback)


def main():
    rclpy.init()

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--ns", type=str, default="/limo")
    parser.add_argument("--nav2_action", type=str, default="/navigate_to_pose")
    args = parser.parse_args()

    node = LimoNavigateServer(ns=args.ns, nav2_action_name=args.nav2_action)

    # 멀티스레드 실행기 (취소와 실행을 동시에 처리하기 위해 필수)
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
