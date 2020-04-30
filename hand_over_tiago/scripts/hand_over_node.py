#! /usr/bin/env python

import rospy
import numpy

import actionlib
from actionlib import SimpleActionClient

from geometry_msgs.msg import WrenchStamped, Point
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest
from hand_over_msgs.msg import HandOverAction, HandOverGoal, HandOverFeedback, HandOverResult, TakeOverAction
from hand_over_msgs.msg import MeasureForceAction, MeasureForceGoal, MeasureForceFeedback, MeasureForceResult
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


class HandOver(object):

    def __init__(self, name, wrenchtopic="/wrist_ft"):

        self._threshold = rospy.get_param('~threshold')
        self._approach_motion = rospy.get_param('~approach_motion')
        self._retreat_delay = rospy.get_param('~retreat_delay')
        self._retreat_motion = rospy.get_param('~retreat_motion')
       
        self._result = HandOverResult()
        self._client = {}
        self._movement_finished = {}
        self.force_variation = {}
        self.force_bias = {}
        self.previous_force = {}
        self._r = rospy.Rate(100)

        self.force_variation = numpy.array([0.0, 0.0, 0.0])
        self.force_bias = numpy.array([0.0, 0.0, 0.0])
        self.previous_force = numpy.array([0.0, 0.0, 0.0])

        self.sub_ft = rospy.Subscriber(wrenchtopic, WrenchStamped, self.handle_wrench)

        self._apply_planning_scene_diff = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        self._apply_planning_scene_diff.wait_for_service()

        self._as_hand = actionlib.SimpleActionServer("handover", HandOverAction,
                                                     execute_cb=self.execute_hand, auto_start=False)

        self._as_take = actionlib.SimpleActionServer("handover_take", TakeOverAction,
                                                     execute_cb=self.execute_take, auto_start=False)

        self._as_measure = actionlib.SimpleActionServer("measure", MeasureForceAction,
                                                        execute_cb=self.execute_measure, auto_start=False)

        self._as_hand.start()
        self._as_take.start()
        self._as_measure.start()

    def execute_hand(self, goal):
        feedback = HandOverFeedback()
        self._result = HandOverResult()
        self._result.success = False

        # Approach
        feedback.phase = HandOverFeedback.PHASE_APPROACH
        self._as_hand.publish_feedback(feedback)
        tiago_play_motion(self._approach_motion) # TODO check return value

        feedback.phase = HandOverFeedback.PHASE_WAITING_FOR_CONTACT
        self._as_hand.publish_feedback(feedback)
        if self.wait_for_force_handover(self._as_hand, self._threshold):
            self._result.success = True

            # Open gripper
            feedback.phase = HandOverFeedback.PHASE_EXECUTING
            self._as_hand.publish_feedback(feedback)
            open_gripper()

            # TODO un-attach object in gripper, if present

            rospy.sleep(self._retreat_delay)

            # Retreat
            feedback.phase = HandOverFeedback.PHASE_RETREAT
            self._as_hand.publish_feedback(feedback)
            retreat_result = tiago_play_motion(self._retreat_motion, wait_duration=15.0)
            if retreat_result == False or retreat_result.error_code != retreat_result.SUCCEEDED:
                self._result.success = False
            
        # Callback Finished
        if self._result.success:
            rospy.loginfo('Succeeded')
            self._as_hand.set_succeeded(self._result)
        else:
            self._as_hand.set_aborted(self._result)

    def execute_take(self, goal):
        feedback = HandOverFeedback()
        self._result = HandOverResult()
        self._result.success = False

        # Approach
        feedback.phase = HandOverFeedback.PHASE_APPROACH
        self._as_take.publish_feedback(feedback)
        approach_result = tiago_play_motion(self._approach_motion) # TODO check return value
        # if approach_result == False or approach_result.error_code != approach_result.SUCCEEDED:

        # Open gripper
        feedback.phase = HandOverFeedback.PHASE_EXECUTING
        self._as_take.publish_feedback(feedback)
        open_gripper()

        feedback.phase = HandOverFeedback.PHASE_WAITING_FOR_CONTACT
        self._as_take.publish_feedback(feedback)
        if self.wait_for_force_handover(self._as_take, self._threshold):
            self._result.success = True

            # Close gripper
            feedback.phase = HandOverFeedback.PHASE_EXECUTING
            self._as_take.publish_feedback(feedback)
            close_gripper()

            if len(goal.object.object.primitives) != 0 or len(goal.object.object.meshes) != 0 or len(goal.object.object.planes) != 0:
                rospy.loginfo("AttachedCollisionObject given, attaching now")
                scene = PlanningScene()
                scene.is_diff = True
                scene.robot_state.is_diff = True
                scene.robot_state.attached_collision_objects = [goal.object]
                planning_scene_diff_req = ApplyPlanningSceneRequest()
                planning_scene_diff_req.scene = scene
                rospy.loginfo(self._apply_planning_scene_diff.call(planning_scene_diff_req))
            else:
                rospy.loginfo("No AttachedCollisionObject given, so not attaching")

            rospy.sleep(self._retreat_delay)

            # Retreat
            feedback.phase = HandOverFeedback.PHASE_RETREAT
            self._as_take.publish_feedback(feedback)
            retreat_result = tiago_play_motion(self._retreat_motion, wait_duration=15.0)
            if retreat_result == False or retreat_result.error_code != retreat_result.SUCCEEDED:
                self._result.success = False

        # Callback Finished
        if self._result.success:
            rospy.loginfo('Succeeded')
            self._as_take.set_succeeded(self._result)
        else:
            self._as_take.set_aborted(self._result)

    def execute_measure(self, goal):
        self._result = MeasureForceResult()
        self._result.success = False

        if self.wait_for_force(self._threshold):
            self._result.success = True
            
        # Callback Finished
        if self._result.success:
            rospy.loginfo('Succeeded')
            self._as_measure.set_succeeded(self._result)
        else:
            self._as_measure.set_aborted(self._result)

    def handle_wrench(self, msg):
        current_force = numpy.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_variation = (current_force - self.previous_force)
        self.previous_force = current_force

    def wait_for_force(self, threshold, timeout=None):
        feedback = MeasureForceFeedback()

        # wait for condition
        if timeout is not None:
            timeout_time = rospy.Time.now() + rospy.Duration(timeout)

        current_val = 0.0

        self.force_bias = self.previous_force
        rospy.loginfo('waiting for force with bias %f %f %f', self.force_bias[0], self.force_bias[1], self.force_bias[2])
        N = 7
        while current_val < threshold:
            feedback.current_value = current_val
            self._as_measure.publish_feedback(feedback)

            current_val = ((N-1)*current_val+numpy.linalg.norm(self.previous_force-self.force_bias))/N
            rospy.logdebug('current_val: %f', current_val)
            # check that preempt has not been requested by the client
            if self._as_measure.is_preempt_requested():
                rospy.loginfo('Preempted')
                self._result.success = False
                self._as_measure.set_preempted(self._result)
                return False

            if timeout is not None:
                if rospy.Time.now() > timeout_time:
                    rospy.loginfo("timeout waiting for force")
                    return False

            self._r.sleep()

        rospy.loginfo("wait_for_force ended with: current_val: %f threshold: %f", current_val, threshold)
        return True

    def wait_for_force_handover(self, action_server, threshold, timeout=None):
        feedback = HandOverFeedback()

        # wait for condition
        if timeout is not None:
            timeout_time = rospy.Time.now() + rospy.Duration(timeout)

        current_val = 0.0

        self.force_bias = self.previous_force
        rospy.loginfo('waiting for force with bias %f %f %f', self.force_bias[0], self.force_bias[1], self.force_bias[2])
        N = 7
        while current_val < threshold:
            feedback.phase = HandOverFeedback.PHASE_WAITING_FOR_CONTACT
            # feedback.current_value = current_val
            action_server.publish_feedback(feedback)

            current_val = ((N-1)*current_val+numpy.linalg.norm(self.previous_force-self.force_bias))/N
            rospy.logdebug('current_val: %f', current_val)
            # check that preempt has not been requested by the client
            if action_server.is_preempt_requested():
                rospy.loginfo('Preempted')
                self._result.success = False
                action_server.set_preempted(self._result)
                return False

            if timeout is not None:
                if rospy.Time.now() > timeout_time:
                    rospy.loginfo("timeout waiting for force")
                    return False

            self._r.sleep()

        rospy.loginfo("wait_for_force ended with: current_val: %f threshold: %f", current_val, threshold)
        return True


def tiago_play_motion(motion, wait_duration=5.0):
    rospy.loginfo('Doing PlayMotion: ' + motion)
    pm = SimpleActionClient('/play_motion', PlayMotionAction)
    pm.wait_for_server()
    pmg = PlayMotionGoal()
    pmg.motion_name = motion
    pm.send_goal(pmg)
    if pm.wait_for_result(rospy.Duration.from_sec(wait_duration)):
        pm_result = pm.get_result()
        rospy.loginfo(pm_result)
        return pm_result
    else:
        rospy.logwarn("play motion timed out")
        return False 


def open_gripper():
    return tiago_play_motion('open_gripper', 1.0)


def close_gripper():
    return tiago_play_motion('close_gripper', 1.0)


if __name__ == '__main__':

    rospy.init_node('hand_over')
    HandOver("handover")
    rospy.loginfo("Everything started, spinning ...")
    rospy.spin()
