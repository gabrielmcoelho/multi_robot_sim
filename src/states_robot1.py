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
from multi_robot_sim.msg import RobotDeliveryAction, RobotDeliveryGoal, RobotDeliveryResult
from multi_robots_security_system.msg import RobotPatrolAction, RobotPatrolGoal, RobotPatrolFeedback


# define state Ocioso
class Ocioso(smach.State):
    request = False

    def __init__(self):      
        smach.State.__init__(self, outcomes=['indoInvestigar', 'patrulhando'],
                             output_keys=['goal'])    
        robot.setRobotStatus('available')    


    def execute(self, userdata):
        rospy.loginfo('Executing state Ocioso')

        while(True): 1
                    userdata.pose = self.data
                    return 'indoInvestigar'
                elif(mode == 'patrol'):
                    userdata.poses = self.data
                    return 'patrulhando'
            else:
                time.sleep(0.5)

    def goInvestigate(self, pose):
        self.mode = 'investigate'
        self.request = True
        self.data = pose

    def startPatrol(self, poses):
        self.mode = 'patrol'
        self.request = True
        self.data = poses
               

# define state IndoInvestigar
class IndoInvestigar(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['investigando', 'ocioso'],
                             input_keys=['goal'])
        self.move_base = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
        self.new_goal = MoveBaseGoal()
        self.robot = robot
        robot.setRobotStatus('going_to_investigate')

    def execute(self, userdata):
        rospy.loginfo('Executing state IndoInvestigar')

        self.new_goal.target_pose = userdata.pose
        
        self.move_base.wait_for_server()

        self.move_base.send_goal(self.new_goal)

        self.move_base.wait_for_result()

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            self.robot.setResult('investigando')
            return 'investigando'
        else:
            if self.action == 'investigate':
                userdata.pose = self.data
                return 'indoInvestigar'
            elif self.action == 'patrol':
                userdata.poses = self.data
                return 'patrulhando'
            self.robot.setResult('ocioso')
            return 'ocioso'
    
    def cancelMovement(self, data, nextAction):
        self.move_base.cancel_all_goals()
        self.data = data
        self.action = nextAction

        


# define state Investigando
class Investigando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['indoInvestigar', 'patrulhando', 'ocioso'])
        self.stop = False
        robot.setRobotStatus('investigating')

    def execute(self, userdata):
        rospy.loginfo('Executing state Investigando')
        
         while(True): 
            if(self.stop):
                self.stop = False
                if(self.action == 'investigate'):
                    userdata.pose = self.data
                    return 'indoInvestigar'
                elif(self.action == 'patrol'):
                    userdata.poses = self.data
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
        smach.State.__init__(self, outcomes=['indoInvestigar', 'ocioso'])
        self.move_base = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
        self.new_goal = MoveBaseGoal()
        robot.setRobotStatus('patrolling')

    def execute(self, userdata):
        rospy.loginfo('Executing state Patrulhando')
        
        self.new_goal.target_pose = userdata.poses[userdata.index]
        
        self.move_base.wait_for_server()

        self.move_base.send_goal(self.new_goal)

        self.move_base.wait_for_result()

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            userdata.index += 1
            userdata.index = userdata.index % len(userdata.poses)
            return 'patrulhando'
        else:
            if self.action == 'investigate':
                userdata.pose = self.data
                return 'indoInvestigar'
            self.robot.setResult('ocioso')
            return 'ocioso'
    
    def stopPatrolling(self, data, nextAction):
        self.move_base.cancel_all_goals()
        self.data = data
        self.action = nextAction
        

class RobotPatrol():


    def __init__(self, availableState, goingToInvestigateState, investigatingState, patrollingState):
        self.status = 'available'
        self.patrolCoordinates = []
        self.shouldPatrol = False
        self.patrolActionServer = actionlib.SimpleActionServer("robot1/patrol", RobotPatrolAction, execute_cb=self.execute_patrol, auto_start = False)
        self.patrolActionServer.start()
        self.availableState = availableState
        self.goingToInvestigateState = goingToInvestigateState
        self.investigatingState = investigatingState
        self.patrollingState = patrollingState
        self.robotPatrolResult = RobotPatrolResult()


    def execute_patrol(self, goal):
        success = True
        if(len(goal.patrol_poses.poses) > 0):
            if self.status == 'available':
                self.shouldPatrol = True
                if(len(goal.patrol_poses.poses) == 1):
                    self.robotState.goInvestigate(goal.patrol_poses.poses[0])
                else:
                    self.robotState.startPatrol(goal.patrol_poses.poses)
            elif self.status == 'going_to_investigate':
                if(len(goal.patrol_poses.poses) == 1):
                    self.patrollingState.cancelMovement(goal.patrol_poses.poses[0], 'investigate')
                else:
                    self.patrollingState.cancelMovement(goal.patrol_poses.poses, 'patrol')
            elif self.status == 'investigating':
                if(len(goal.patrol_poses.poses) == 1):
                    self.patrollingState.stopInvestigating(goal.patrol_poses.poses[0], 'investigate')
                else:
                    self.patrollingState.stopInvestigating(goal.patrol_poses.poses, 'patrol')
            elif self.status == 'patrolling':
                if(len(goal.patrol_poses.poses) == 1):
                    self.patrollingState.stopPatrolling(goal.patrol_poses.poses[0], 'investigate')
                else:
                    rospy.loginfo('robot1 is already patrolling!')

        while(True):
            if not self.shouldPatrol:
                if success:
                    self.patrolActionServer.set_succeeded(self.robotPatrolResult)
                return
            else:
                if self.patrolActionServer.is_preempt_requested():
                    if self.status == 'going_to_investigate':
                        self.patrollingState.cancelMovement(None, None)
                    elif self.status == 'investigating':
                        self.patrollingState.stopInvestigating(None, None)
                    elif self.status == 'patrolling':
                        self.patrollingState.stopPatrolling(None, None)
                    rospy.loginfo('robot1/patrol Preempted')
                    success = False
                    self.shouldPatrol = False
                    self.patrolActionServer.set_preempted()
                time.sleep(0.5)

        

    def setResult(self):
        self.shouldPatrol = False
        return

    def setRobotStatus(self, status):
        self.status = status
        return

       

def main():
    rospy.init_node('state_machine_robot1')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['FIM'])

    availableState = Ocioso()
    goingToInvestigateState = IndoInvestigar()
    investigatingState = Investigando()
    patrollingState = Patrulhando()
    robot = RobotPatrol(availableState, goingToInvestigateState, investigatingState, patrollingState)

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
                               transitions={'indoInvestigar': 'INDO_INVESTIGAR', 'ocioso': 'OCIOSO'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('sm_robot1', sm, '/SM_ROBOT1')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()