import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

from move_base_skill.action import MoveBaseSkill

class TestMoveBaseSkill(Node):
    def __init__(self):
        super().__init__('test_move_base_skill')
        self.skill_client = ActionClient(self, MoveBaseSkill, '/skill/move_base')

    def send_test_goal(self):
        goal_msg = MoveBaseSkill.Goal()
        goal_msg.goal_type = MoveBaseSkill.Goal.POSE

        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 0.3
        pose1.pose.position.y = 0.3
        pose1.pose.position.z = 0.
        pose1.pose.orientation.x = 0.
        pose1.pose.orientation.y = 0.
        pose1.pose.orientation.z = 0.
        pose1.pose.orientation.w = 1.

        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.pose.position.x = 0.6
        pose2.pose.position.y = 0.
        pose2.pose.position.z = 0.
        pose2.pose.orientation.x = 0.
        pose2.pose.orientation.y = 0.
        pose2.pose.orientation.z = 0.
        pose2.pose.orientation.w = 1.

        goal_msg.poses = [pose1, pose2]
        self.get_logger().info(f'Sending goal {goal_msg}')

        self.skill_client.wait_for_server()
        return self.skill_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    move_base_skill_client = TestMoveBaseSkill()

    future = move_base_skill_client.send_test_goal()
    rclpy.spin_until_future_complete(move_base_skill_client, future)

    move_base_skill_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
