#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
import time
from std_msgs.msg import String
from multi_robots_security_system.msg import RobotPatrolAction, RobotPatrolGoal, RobotPatrolResult

# define state Ocioso
class Ocioso(smach.State):
    request = False

    def __init__(self):      
        smach.State.__init__(self, outcomes=['indoInvestigar', 'patrulhando'], output_keys=['poseObj', 'index'])
        self.robot = None
        self.action = None

    def execute(self, userdata):
        rospy.loginfo('Executing state Ocioso')
        self.robot.setRobotStatus('available')

        while(True): 
            if(self.action == 'investigate'):
                userdata.poseObj = self.data
                return 'indoInvestigar'
            elif(self.action == 'patrol'):
                userdata.poseObj = self.data
                userdata.index = 0
                return 'patrulhando'
            else:
                time.sleep(0.5)

    def goInvestigate(self, poseObj):
        self.action = 'investigate'
        self.request = True
        self.data = poseObj

    def startPatrol(self, poseObj):
        self.action = 'patrol'
        self.request = True
        self.data = poseObj
               

# define state IndoInvestigar
class IndoInvestigar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['investigando', 'ocioso'], input_keys=['poseObj'], output_keys=['poseObj', 'index'])
        self.move_base = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)
        self.new_goal = MoveBaseGoal()
        self.robot = None

    def execute(self, userdata):
        rospy.loginfo('Executing state IndoInvestigar')
        self.robot.setRobotStatus('going_to_investigate')

        self.new_goal.target_pose.header = userdata.poseObj.header
        self.new_goal.target_pose.pose = userdata.poseObj.poses[0]
        
        self.move_base.wait_for_server()

        self.move_base.send_goal(self.new_goal)

        self.move_base.wait_for_result()

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            self.robot.setResult()
            return 'investigando'
        else:
            if self.action == 'investigate':
                userdata.poseObj = self.data
                return 'indoInvestigar'
            elif self.action == 'patrol':
                userdata.poseObj = self.data
                userdata.index = 0
                return 'patrulhando'
            self.robot.setRobotStatus('available')
            return 'ocioso'
    
    def cancelMovement(self, data, nextAction):
        self.move_base.cancel_all_goals()
        self.data = data
        self.action = nextAction

        


# define state Investigando
class Investigando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['indoInvestigar', 'patrulhando', 'ocioso'], output_keys=['poseObj', 'index'])
        self.stop = False
        self.robot = None

    def execute(self, userdata):
        rospy.loginfo('Executing state Investigando')
        self.robot.setRobotStatus('investigating')
        
        while(True): 
            if(self.stop):
                self.stop = False
                if(self.action == 'investigate'):
                    userdata.poseObj = self.data
                    return 'indoInvestigar'
                elif(self.action == 'patrol'):
                    userdata.poseObj = self.data
                    userdata.index = 0
                    return 'patrulhando'
                return 'ocioso'
            else:
                time.sleep(0.5)

        return 'retornarCozinha'
    
    def stopInvestigating(self, data, nextAction):
        self.stop = True
        self.data = data
        self.action = nextAction

# define state Patrulhando
class Patrulhando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['patrulhando', 'indoInvestigar', 'ocioso'], input_keys=['poseObj', 'index'], output_keys=['poseObj', 'index'])
        self.move_base = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)
        self.new_goal = MoveBaseGoal()
        self.robot = None
        self.action = None

    def execute(self, userdata):
        rospy.loginfo('Executing state Patrulhando')
        self.robot.setRobotStatus('patrolling')

        self.new_goal.target_pose.header = userdata.poseObj.header
        self.new_goal.target_pose.pose = userdata.poseObj.poses[userdata.index]
        
        self.move_base.wait_for_server()

        print('goal has been sent!')

        self.move_base.send_goal(self.new_goal)

        self.move_base.wait_for_result()

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            userdata.poseObj = userdata.poseObj
            userdata.index += 1
            userdata.index = userdata.index % len(userdata.poseObj.poses)
            return 'patrulhando'
        else:
            if self.action == 'investigate':
                userdata.poseObj = self.data
                return 'indoInvestigar'
            self.robot.setRobotStatus('available')
            return 'ocioso'
    
    def stopPatrolling(self, data, nextAction):
        self.move_base.cancel_all_goals()
        self.data = data
        self.action = nextAction
        

class RobotPatrol():


    def __init__(self, availableState, patrollingState, goingToInvestigateState, investigatingState):
        self.status = 'available'
        self.patrolCoordinates = []
        self.shouldPatrol = False
        print('ta aqui o server poha')
        self.patrolActionServer = actionlib.SimpleActionServer("robot2/patrol", RobotPatrolAction, execute_cb=self.execute_patrol, auto_start = False)
        self.patrolActionServer.start()
        self.availableState = availableState
        self.patrollingState = patrollingState
        self.goingToInvestigateState = goingToInvestigateState
        self.investigatingState = investigatingState
        self.robotPatrolResult = RobotPatrolResult()


    def execute_patrol(self, goal):
        if(len(goal.patrol_poses.poses) == 0):
            data = None
            nextAction = None
        elif(len(goal.patrol_poses.poses) == 1):
            data = goal.patrol_poses
            nextAction = 'investigate'
        elif(len(goal.patrol_poses.poses) > 1):
            data = goal.patrol_poses
            nextAction = 'patrol'

        if self.status == 'available':
            if(nextAction == None):
                rospy.loginfo('robot2 is already available!')
            elif(nextAction == 'investigate'):
                self.shouldPatrol = True
                self.availableState.goInvestigate(data)
            elif(nextAction == 'patrol'):
                self.shouldPatrol = True
                self.availableState.startPatrol(data)
        elif self.status == 'going_to_investigate':
            self.goingToInvestigateState.cancelMovement(data, nextAction)
        elif self.status == 'investigating':
            self.investigatingState.stopInvestigating(data, nextAction)
        elif self.status == 'patrolling':
            if(nextAction == 'investigate'):
                self.patrollingState.stopPatrolling(data, nextAction)
            else:
                rospy.loginfo('robot2 is already patrolling!')

        while(True):
            if not self.shouldPatrol:
                self.patrolActionServer.set_succeeded(self.robotPatrolResult)
                return
            else:
                time.sleep(0.5)

        

    def setResult(self):
        self.robotPatrolResult.status = 'investigating'
        self.shouldPatrol = False

    def setRobotStatus(self, status):
        self.status = status

       

def main():
    rospy.init_node('state_machine_robot2')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['FIM'])

    availableState = Ocioso()
    patrollingState = Patrulhando()
    goingToInvestigateState = IndoInvestigar()
    investigatingState = Investigando()

    robot = RobotPatrol(availableState, patrollingState, goingToInvestigateState, investigatingState)

    availableState.robot = robot
    patrollingState.robot = robot
    goingToInvestigateState.robot = robot
    investigatingState.robot = robot

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('OCIOSO', availableState, 
                               transitions={'indoInvestigar': 'INDO_INVESTIGAR', 'patrulhando': 'PATRULHANDO'})

        smach.StateMachine.add('INDO_INVESTIGAR', goingToInvestigateState, 
                               transitions={'investigando': 'INVESTIGANDO', 'ocioso': 'OCIOSO'},
                               remapping={'goal':'goal'})

        smach.StateMachine.add('INVESTIGANDO', investigatingState, 
                               transitions={'indoInvestigar': 'INDO_INVESTIGAR', 'patrulhando': 'PATRULHANDO', 'ocioso': 'OCIOSO'})

        smach.StateMachine.add('PATRULHANDO', patrollingState, 
                               transitions={'patrulhando': 'PATRULHANDO', 'indoInvestigar': 'INDO_INVESTIGAR', 'ocioso': 'OCIOSO'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('sm_robot2', sm, '/SM_ROBOT2')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()