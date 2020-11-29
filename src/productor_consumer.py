#! /usr/bin/env python

import threading

import time

import rospy

import actionlib

import actionlib_tutorials.msg

from move_base_msgs.msg import MoveBaseAction
from multi_robots_security_system.msg import SecurityAction, SecurityGoal, SecurityFeedback, RobotPatrolAction, RobotPatrolGoal, RobotPatrolFeedback

robots_status = ['available', 'available'] 

class RobotClientController(object):
    # create messages that are used to publish feedback
    # _feedback = RobotPatrolFeedback()

    def __init__(self, robots):
        self.robots = robots
        self._securityActionServer = actionlib.SimpleActionServer('/security_system', SecurityAction, execute_cb=self.execute_security_action, auto_start = False)
        self._securityActionServer.start()
      
    def execute_security_action(self, goal):
        print('execute callback on client controller!')
        self.robots[goal.robotIndex].setGoal(goal.patrol_poses)
        # # helper variables
        # r = rospy.Rate(1)
        # success = True
        
        # # append the seeds for the fibonacci sequence
        # self._feedback.sequence = []
        # self._feedback.sequence.append(0)
        # self._feedback.sequence.append(1)
        
        # # publish info to the console for the user
        # rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # # start executing the action
        # for i in range(1, goal.order):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()
          
        # if success:
        #     self._result.sequence = self._feedback.sequence
        #     rospy.loginfo('%s: Succeeded' % self._action_name)
        #     self._as.set_succeeded(self._result)

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

                print('waiting for server')             
                self.robot.wait_for_server()

                print('manda pros states bb')

                # print(self.robot_server_name + " Goal foi enviado! (" + str(self.robotPatrolGoal.x) + "," + str(self.robotPatrolGoal.y) + ")")

                # self.robot.send_goal(self.robotPatrolGoal)

                # self.robot.wait_for_result()

                # result = self.robot.get_result()
                # print(self.robot_server_name + " Resultado recebido!!")
                # robots_status[self.robot_number-1] = 0

            else:
                time.sleep(0.5)

        return

    def setGoal(self, poses):
        print('ele sabe que tem q enviar pro states :)')
        if len(poses.poses) > 0:
            self.robotPatrolGoal.patrol_poses = poses
            self.receive_goal = True
            if(len(poses.poses) == 1):
                robots_status[self.robot_number-1] = 'going to investigate'
            else:
                robots_status[self.robot_number-1] = 'patrolling'
        return

        
if __name__ == '__main__':
    rospy.init_node('security_system')
    robots = []
    for i in range(len(robots_status)):
            robot_server_name = '/robot' + str(i+1) +'/patrol'
            robots.append(RobotController(robot_server_name, i+1))
            robots[i].start()
    server = RobotClientController(robots)
    rospy.spin()