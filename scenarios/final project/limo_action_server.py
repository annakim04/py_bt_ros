import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from nav2_msgs.action import Spin
from geometry_msgs.msg import Twist

class LimoRobotServer(Node):
    """
    [서버 2] BT <-> Robot Hardware (cmd_vel) 직접 제어 서버
    Nav2를 거치지 않고 단순 동작(회전 등)을 수행합니다.
    """
    def __init__(self, ns="/limo"):
        super().__init__("limo_robot_server")
        self.ns = ns.rstrip("/")
        
        # BT용 서버 주소: /limo/spin
        self.spin_action_name = f"{self.ns}/spin"
        
        # 로봇 바퀴 제어용 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.spin_server = ActionServer(
            self, Spin, self.spin_action_name,
            execute_callback=self.spin_execute_cb,
            goal_callback=self.spin_goal_cb,
            cancel_callback=self.spin_cancel_cb
        )
        self.get_logger().info(f"[Robot Server] Ready: {self.spin_action_name} -> /cmd_vel")

    def spin_goal_cb(self, goal): return GoalResponse.ACCEPT
    
    def spin_cancel_cb(self, goal):
        self.get_logger().info("Spin Cancel Requested! Stopping robot.")
        return CancelResponse.ACCEPT

    async def spin_execute_cb(self, goal_handle):
        self.get_logger().info("Executing Spin...")
        
        twist = Twist()
        twist.angular.z = 0.5  # 회전 속도
        
        duration = 2.0  # 회전 시간
        start_time = time.time()
        feedback = Spin.Feedback()

        while (time.time() - start_time) < duration:
            # 취소 요청(버튼 누름) 체크
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return Spin.Result()

            self.cmd_vel_pub.publish(twist)
            
            feedback.angular_distance_traveled = (time.time() - start_time) * 0.5
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        # 종료 후 정지
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        goal_handle.succeed()
        return Spin.Result()

def main():
    rclpy.init()
    node = LimoRobotServer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()