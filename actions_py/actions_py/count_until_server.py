import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.action import GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading

import time
from my_robot_interfaces.action import CountUntil

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle: ServerGoalHandle = None
        self.goal_lock= threading.Lock()
        self.count_until_server_=ActionServer(self, CountUntil,
                                               "count_until",
                                               goal_callback=self.goal_callback,
                                               cancel_callback= self.cancel_callback,
                                               execute_callback=self.execute_callback,
                                               callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action Server Has Started !!")

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Recieved the goal")

        # #policy of : Refuse new goal if current goal is active 
        # if  self.goal_handle is not None and self.goal_handle.is_active:
        #     self.get_logger().info("A goal is already active, rejecting the new goal.")
        #     return GoalResponse.REJECT
        # validate the goal Request
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the goal.")
            return GoalResponse.REJECT
        
        # ## Policy of :: Preempt current goal if new goal is received
        # with self.goal_lock:
        #     if self.goal_handle is not None and self.goal_handle.is_active:
        #         self.get_logger().info("Abrot current goal and accept new goal")
        #         self.goal_handle.abort()
                
        self.get_logger().info("Accepting the Goal.")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().warn("Received a Cancel Request")
        return CancelResponse.ACCEPT ## or Reject to reject the cancell request

    def execute_callback(self, goal_handle:ServerGoalHandle):
        with self.goal_lock:
            self.goal_handle = goal_handle

        ## get request from goal
        target_number=goal_handle.request.target_number
        period=goal_handle.request.period

        ##Execute Action
        counter=0
        self.get_logger().info("Executing the goal !")
        feedback= CountUntil.Feedback()
        result= CountUntil.Result()
        for i in range(target_number):
            # ## For preempt the current goal and process new goal
            # if not goal_handle.is_active:
            #     result.reached_number=counter
            #     return result
            ## Normal working loop
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.reached_number=counter
                return result
            counter+=1
            self.get_logger().info(str(counter))
            feedback.current_number= counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)
        ## Once done set the goal state
        goal_handle.succeed()

        ## send the result
        result.reached_number= counter
        return result



def main(args=None):
    rclpy.init(args=args)
    node=CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__=="__main__":
    main()
