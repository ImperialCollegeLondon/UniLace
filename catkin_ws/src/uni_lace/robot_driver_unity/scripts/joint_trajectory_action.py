#!/usr/bin/env python

import rospy
import actionlib
import std_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg

class JointTrajectoryAction(object):
    def __init__(self, name):
        self.action_name = name
        # self.action_name = "joint_trajectory_action"
        self._action_server = actionlib.ActionServer(self.action_name, 
            control_msgs.msg.FollowJointTrajectoryAction, goal_cb=self.goalCB, cancel_cb=self.cancelCB, auto_start = False)
        # TODO: get joint names

        # self.action_goal = actionlib.ServerGoalHandle ActionServer<control_msgs::FollowJointTrajectoryAction>

        self.pub_trajectory_command_ = rospy.Publisher("joint_path_command", trajectory_msgs.msg.JointTrajectory, queue_size=1)
        self.sub_controller_state_ = rospy.Subscriber("controller_states", std_msgs.msg.Bool, self.controllerStateCB)
        # self.sub_robot_status_ = rospy.Subscriber("robot_status", , self.robotStatusCB)

        self._has_active_goal_ = False
        self.controller_alive_ = False
        self.has_moved_once_ = False
        self.active_goal_ = actionlib.ServerGoalHandle()

        self._action_server.start()
    
    def controllerStateCB(self, state):
        if not state.data and self._has_active_goal_:
            self._has_active_goal_ = False
            self.active_goal_.set_succeeded()

    def goalCB(self, gh):
        rospy.loginfo("{}: Received new goal".format(self.action_name))
        # TODO: check if controller is alive
        if gh.get_goal().trajectory.points:
            if self._has_active_goal_:
                rospy.logwarn("%s Received new goal, canceling current goal", self.action_name)
                self.abortGoal()
            else:
                gh.set_accepted()
                self.active_goal_ = gh
                self._has_active_goal_ = True
                self.pub_trajectory_command_.publish(gh.get_goal().trajectory)
        else:
            rospy.logerr("{}: Joint trajectory action failed on empty trajectory".format(self.action_name))
            self.active_goal_.set_aborted()

    def cancelCB(self, gh):
        rospy.logwarn("{}: Received action cancel request".format(self.action_name))
        if self.active_goal_ == gh:
            rospy.loginfo("{}: Received action cancel request".format(self.action_name))
            empty = trajectory_msgs.msg.JointTrajectory()
            self.pub_trajectory_command_.publish(empty)

            self.active_goal_.set_canceled()
            self._has_active_goal_ = False
        else:
            rospy.logwarn("{}: Active goal and goal cancel do not match, ignoring cancel request".format(self.action_name))

    def abortGoal(self):
        empty = trajectory_msgs.msg.JointTrajectory()
        self.pub_trajectory_command_.publish(empty)

        self.active_goal_.set_aborted()
        self._has_active_goal_ = False

def main():
    rospy.init_node('joint_trajectory_action', anonymous=True)
    # rospy.loginfo(rospy.get_name())
    JointTrajectoryAction(rospy.get_name())
    rospy.spin()

if __name__ == '__main__':
    main()