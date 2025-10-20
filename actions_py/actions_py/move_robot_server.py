import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from my_robot_interfaces.action import MoveRobot
from rclpy.action.server import ServerGoalHandle,GoalResponse,CancelResponse
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading



class MoveRobotServer(Node):
    def __init__(self):
        super().__init__("move_robot")
        self.goal_handle_:ServerGoalHandle = None
        self.goal_lock_=threading.Lock()
        self.robot_position_=50
        self.move_robot_server_=ActionServer(self,
                                             MoveRobot,
                                             "move_robot",
                                             goal_callback=self.goal_callback,
                                             cancel_callback=self.cancel_callback,
                                             execute_callback=self.execute_callback,
                                             callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action Server Has Started !")
        self.get_logger().info("Robot_position" + str(self.robot_position_))

    ## GOal validation work
    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Recieved a new goal")
        if goal_request.position  not in range(0,100) or goal_request.velocity <=0:
            self.get_logger().warn("Invalid Position/Velocity, Rejected Goal")
            return GoalResponse.REJECT
        ## New goal must be valid to go into Preemption
        if self.goal_handle_ is not None and self.goal_handle_.is_active:
            self.goal_handle_.abort()
            
        self.get_logger().warn("Valid Position/Velocity, Accepted Goal")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle:ServerGoalHandle):
        self.get_logger().error("Recieved a Cancel Request.")
        return CancelResponse.ACCEPT
        
    def execute_callback(self, goal_handle_:ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_=goal_handle_
        feedback=MoveRobot.Feedback()
        result= MoveRobot.Result()
        ## Get request from goal
        goal_position= goal_handle_.request.position
        velocity = goal_handle_.request.velocity
        self.get_logger().info("Executing the goal !")
        # Execute Action
        while rclpy.ok():
            ## cheking if goal aborted
            if not goal_handle_.is_active:
                result.position=self.robot_position_
                result.message=" Preempted by another goal !!"
                return result
            
            ## checking and handling goal cancelled
            if goal_handle_.is_cancel_requested:
                result.position= self.robot_position_
                if goal_position == self.robot_position_:
                    result.message="Sucess"
                    goal_handle_.succeed()
                else:
                    result.message=" Cancelled the goal !"
                    goal_handle_.canceled()
                return result

            diff= goal_position-self.robot_position_
            if diff == 0:
                result.position=self.robot_position_
                result.message="Reached to goal position i.e "+ str(self.robot_position_)
                goal_handle_.succeed()
                return result
            elif diff>0:
                if diff >=velocity:
                    self.robot_position_+=velocity
                else:
                    self.robot_position_+=diff
            else: ## if difference is neg i.e goal is before the current position
                if abs(diff) >= velocity:
                    self.robot_position_ -= velocity
                else:
                    self.robot_position_ -= abs(diff)

            self.get_logger().info(str(self.robot_position_))
            feedback.current_position= self.robot_position_
            goal_handle_.publish_feedback(feedback)
            time.sleep(1.0)
        goal_handle_.succeed()
        result.position=self.robot_position_
        result.message="Reached to goal position i.e "+ str(goal_position)
        return result
    
def main(args=None):
    rclpy.init(args=args)
    node=MoveRobotServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__=="__main__":
    main()