from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from task_manager_nav2.basic_navigator import BasicNavigator

from geometry_msgs.msg import PoseWithCovarianceStamped


class GoalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, "send_goal_nav2", self.listener_callback, 10
        )
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

    def listener_callback(self, msg):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.pose.pose.position.x
        goal_pose.pose.position.y = msg.pose.pose.position.y
        goal_pose.pose.orientation.w = 1.0
        limit_for_task_duration = 600.0
        self.navigator.goToPose(goal_pose)
        i = 0
        while not self.navigator.isNavComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.navigator.info(
                    f"Current position x: {feedback.current_pose.pose.position.x} y:  {feedback.current_pose.pose.position.x}"
                )
                if Duration.from_msg(feedback.navigation_time) > Duration(
                    seconds=limit_for_task_duration
                ):
                    self.navigator.cancelNav()

        result = self.navigator.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            self.navigator.info("Goal succeeded!")
        elif result == GoalStatus.STATUS_CANCELED:
            self.navigator.info("Goal was canceled!")
        elif result == GoalStatus.STATUS_ABORTED:
            self.navigator.info("Goal failed!")
        else:
            self.navigator.info("Goal has an invalid return status!")


def main(args=None):
    rclpy.init()
    goal_subscriber = GoalSubscriber()
    rclpy.spin(goal_subscriber)


if __name__ == "__main__":
    main()

