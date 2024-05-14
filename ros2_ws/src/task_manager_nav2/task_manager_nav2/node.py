from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from task_manager_nav2.basic_navigator import BasicNavigator

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String



class GoalSubscriber(Node):

    def __init__(self):
        super().__init__("nav2_task_manager")

        self.init_params()
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, self.goal_topic, self.goto_callback, 10
        )
        self.subscription = self.create_subscription(
            String, self.cancel_goal_topic, self.cancel_goal_callback, 10
        )        

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

    def init_params(self):
        self.declare_parameter(
            "limit_for_task_duration", 600.0, ParameterDescriptor(description="The maximum duration (in seconds) of a task before it is cancelled.")
        )      
        self.limit_for_task_duration = (
            self.get_parameter("limit_for_task_duration").get_parameter_value().double_value
        )
        self.declare_parameter(
            "goal_topic", "send_goal_nav2", ParameterDescriptor(description="topic to send tasks to nav2") 
        )          
        self.goal_topic = (
            self.get_parameter("goal_topic").get_parameter_value().string_value
        )      
        self.declare_parameter(
            "cancel_goal_topic", "cancel_goal_nav2", ParameterDescriptor(description="topic to send tasks to nav2") 
        )          
        self.cancel_goal_topic = (
            self.get_parameter("cancel_goal_topic").get_parameter_value().string_value
        )             

    def cancel_goal_callback(self, msg):
        self.navigator.cancelNav()  
        self.navigator.info("Goal canceled!")           

    def goto_callback(self, msg):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.pose.pose.position.x
        goal_pose.pose.position.y = msg.pose.pose.position.y
        goal_pose.pose.orientation.w = 1.0
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
                    seconds=self.limit_for_task_duration
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

