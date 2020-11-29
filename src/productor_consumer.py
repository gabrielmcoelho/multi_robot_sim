#! /usr/bin/env python

import threading

import time

import rospy

import actionlib

import actionlib_tutorials.msg

from move_base_msgs.msg import MoveBaseAction
from multi_robots_security_system.msg import SecurityAction, SecurityGoal, SecurityResult, RobotPatrolAction, RobotPatrolGoal, RobotPatrolResult

robots_status = ['available', 'available'] 

class RobotClientController(object):
    
    securityResult = SecurityResult()
    def __init__(self, robots):
        self.robots = robots
        self._securityActionServer = actionlib.SimpleActionServer('/security_system', SecurityAction, execute_cb=self.execute_security_action, auto_start = False)
        self._securityActionServer.start()
      
    def execute_security_action(self, goal):
        if len(goal.patrol_poses.poses) > 0:
            print('client requested patrol action')
        else:
            print('client requested robot to stop')
        self.robots[goal.robotIndex].setGoal(goal.patrol_poses)

    def on_goal_result(self, result, robotIndex):
        self.securityResult.status = result.status
        self.securityResult.robotIndex = robotIndex
        self._securityActionServer.set_succeeded(self.securityResult)

class RobotController(threading.Thread):
    receive_goal = False
    robotPatrolGoal = RobotPatrolGoal()
    def __init__(self, robot_server_name, robot_number):
        super(RobotController,self).__init__()
        self.robot_server_name = robot_server_name
        self.robot = actionlib.SimpleActionClient(robot_server_name, RobotPatrolAction)
        self.robot_number = robot_number

    def run(self):
        while(True):
            if self.receive_goal:
                self.receive_goal = False

                print('waiting for /robot' + self.robot_number + '/patrol server')

                self.robot.wait_for_server()

                print('sending patrol action to /robot' + self.robot_number + '/patrol server')

                self.robot.send_goal(self.robotPatrolGoal, self.on_goal_result)
            else:
                time.sleep(0.5)

        return

    def setGoal(self, poses):
        self.robotPatrolGoal.patrol_poses = poses
        self.receive_goal = True
    
    def on_goal_result(self, status, result):
        print('goal result!')
        print(status)
        print(result)
        server.on_goal_result(result, self.robot_number - 1)


        
if __name__ == '__main__':
    rospy.init_node('security_system')
    robots = []
    for i in range(len(robots_status)):
            robot_server_name = '/robot' + str(i+1) +'/patrol'
            robots.append(RobotController(robot_server_name, i+1))
            robots[i].start()
    server = RobotClientController(robots)
    rospy.spin()