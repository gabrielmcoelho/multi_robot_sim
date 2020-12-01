#! /usr/bin/env python

import threading
import time
import rospy
import actionlib
import actionlib_tutorials.msg

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from multi_robots_security_system.msg import RobotPatrolAction, RobotPatrolGoal, RobotPatrolResult

client_servers = ['/robot1/web', '/robot2/web'] 
robot_servers = ['/robot1/patrol', '/robot2/patrol'] 

# Provides an action server interface to interact with the client
class ActionServerController(object):
    
    def __init__(self, robot, client_server_name, robot_number):
        self.robot = robot
        self.action_server = actionlib.SimpleActionServer(client_server_name, RobotPatrolAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()
        self.result = RobotPatrolResult()
        self.robot_number = robot_number
        print(client_server_name + ' server initialized!')
      
    def execute_cb(self, goal):
        self.waiting_response = True
        self.success = True

        if(len(goal.patrol_poses.poses) == 0):
            print('Robot ' + str(self.robot_number) + ' received action: Stop')
        elif(len(goal.patrol_poses.poses) == 1):
            print('Robot ' + str(self.robot_number) + ' received action: Investigate')
        elif(len(goal.patrol_poses.poses) > 1):
            print('Robot ' + str(self.robot_number) + ' received action: Patrol')
        self.robot.setGoal(goal.patrol_poses)

        while(True):
            if(self.waiting_response):
                time.sleep(0.5)
                if(self.action_server.is_new_goal_available()):
                    self.action_server.set_aborted(self.result)
                    return
            else:
                if(self.success):
                    self.action_server.set_succeeded(self.result)
                else:
                    self.action_server.set_aborted(self.result)
                return


    def on_goal_finish(self, status, result):
        self.waiting_response = False
        self.result.status = result.status
        if status == GoalStatus.SUCCEEDED:
            self.success = True
        else:
            self.success = False

# Communicates with the robot
class RobotController(threading.Thread):
    receive_goal = False
    robotPatrolGoal = RobotPatrolGoal()
    def __init__(self, robot_server_name, robot_number):
        super(RobotController,self).__init__()
        self.robot_server_name = robot_server_name
        self.robot = actionlib.SimpleActionClient(robot_server_name, RobotPatrolAction)
        self.robot_number = robot_number
        self.clientInterface = None

    def run(self):
        while(True):
            if self.receive_goal:
                self.receive_goal = False

                self.robot.wait_for_server()

                print('sending action to /robot' + str(self.robot_number) + '/patrol server')

                self.robot.send_goal(self.robotPatrolGoal, self.on_goal_result)
            else:
                time.sleep(0.5)

        return

    def setGoal(self, poses):
        self.robotPatrolGoal.patrol_poses = poses
        self.receive_goal = True
    
    def on_goal_result(self, status, result):
        print('Robot ' + str(self.robot_number) + ' finished its goal!')
        print('Goal status: ' + str(status))
        print('Robot ' + str(self.robot_number) + ' is currently ' + result.status)
        # If a client interface is provided an it has a method called 'on_goal_finish', invoke it
        if(self.clientInterface and callable(getattr(self.clientInterface, 'on_goal_finish', None))):
            self.clientInterface.on_goal_finish(status, result)


        
if __name__ == '__main__':
    rospy.init_node('security_system')
    robots = []
    clients = []
    for i in range(len(robot_servers)):
        robots.append(RobotController(robot_servers[i], i+1))
        robots[i].start()
    # The client interface implementation can be adjusted to your goals. 
    # In this case, we are using one action server for each robot
    for i in range(len(client_servers)):
        clients.append(ActionServerController(robots[i], client_servers[i], i+1))
        robots[i].clientInterface = clients[i]
    rospy.spin()