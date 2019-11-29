#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import sys
import rospy
import actionlib
import tf2_ros
import math
from tf2_ros import Buffer, TransformListener
from control_msgs.msg import PointHeadAction, PointHeadGoal
import geometry_msgs
from geometry_msgs.msg import PointStamped
from uwds3_msgs.msg import SceneNodeArrayStamped
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class BehaviorManager(object):
    def __init__(self):
        rospy.loginfo("[supervision] Starting the robot behavior manager...")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        rospy.sleep(0.5)

        self.point_head_action_srv = rospy.get_param("~point_head_action_srv", "head_controller/point_head_action")
        self.head_action_client = actionlib.SimpleActionClient(self.point_head_action_srv, PointHeadAction)

        self.lookat_min_duration = rospy.get_param("~lookat_min_duration", 0.2)
        self.lookat_max_velocity = rospy.get_param("~lookat_max_velocity", 0.1)

        self.max_linear_velocity = rospy.get_param("~linear_velocity_ratio", 0.15)
        self.max_angular_velocity = rospy.get_param("~angular_velocity_ratio", 0.25)
        assert(self.max_linear_velocity != 0.0)
        assert(self.max_angular_velocity != 0.0)

        self.pointing_frame = rospy.get_param("~pointing_frame", "")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_footprint")

        self.look_at_min_height = rospy.get_param("~look_at_min_height", 1.0)

        self.last_look_at_point = None
        self.last_tracked_frame = None

        self.enable_tracking = True

        rospy.loginfo("[supervision] Waiting '{}' action server...".format(self.point_head_action_srv))
        if not self.head_action_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Not able to connect to '{}' action server ! Check that you are connected to the robot, if so check the ROS_MASTER_URI and ROS_IP. If everything is ok, verify that this node do NOT run on a particular namespace.".format(self.point_head_action_srv))
            sys.exit()

        self.cmd_vel_publisher = rospy.Publisher("mobile_base_controller/cmd_vel", Twist, queue_size=1)

        self.joy_topic = rospy.get_param("~joy_topic", "joy")
        rospy.loginfo("[supervision] Subscribing to '/{}' topic...".format(self.joy_topic))
        self.track_subscriber = rospy.Subscriber(self.joy_topic, Joy, self.joystick_callback, queue_size=1)

        self.tracks_topic = rospy.get_param("~tracks_topic", "tracks")
        rospy.loginfo("[supervision] Subscribing to '/{}' topic...".format(self.tracks_topic))
        self.track_subscriber = rospy.Subscriber(self.tracks_topic, SceneNodeArrayStamped, self.observation_callback, queue_size=1)

        rospy.loginfo("[supervision] Behavior manager ready !")

    def observation_callback(self, tracks_msg):
        person_to_lookat = None
        min_person_dist = 10000
        rospy.logdebug("[supervision] Tracks received, triggering callback...")
        for track in tracks_msg.nodes:
            if track.label == "person" or track.label == "face":
                if track.is_located is True:
                    success, t, _ = self.get_transform_from_tf2(self.pointing_frame, track.id)
                    if success is True:
                        dist = math.sqrt(math.pow(t[0], 2)+ math.pow(t[1], 2)+ math.pow(t[2], 2))
                        if min_person_dist > dist:
                            person_to_lookat = track
                            min_person_dist = dist

        track_to_lookat = person_to_lookat
        if track_to_lookat is not None:
            if self.enable_tracking is True:
                look_at_point = PointStamped()
                look_at_point.header = track_to_lookat.position.header
                look_at_point.header.stamp = rospy.Time(0)
                look_at_point.point = track_to_lookat.position.pose.pose.position
                self.last_look_at_point = look_at_point
                self.last_tracked_frame = track_to_lookat.id
                self.look_at(look_at_point, timeout=self.lookat_min_duration)

    def joystick_callback(self, joy_msg):
        if joy_msg.buttons[7] == 1.0: # security check
            if joy_msg.buttons[0] > 0.0 or joy_msg.buttons[1] > 0.0 or joy_msg.buttons[2] > 0.0 or joy_msg.buttons[3] > 0.0 or joy_msg.buttons[4] > 0.0:
                # head control mode
                self.enable_tracking = False

                if joy_msg.buttons[4] == 1.0:
                    command = Twist()
                    command.angular.z = 0.25
                    self.cmd_vel_publisher.publish(command)

                elif joy_msg.buttons[1] == 1.0:
                    # look_at ground
                    if self.last_look_at_point is not None and self.last_tracked_frame is not None:
                        look_at_point = self.last_look_at_point
                        self.look_at(look_at_point, look_at_height=0.0, timeout=3.0)

                elif joy_msg.buttons[0] == 1.0:
                    # look_at left
                    if self.last_look_at_point is not None and self.last_tracked_frame is not None:
                        success, t, _ = self.get_transform_from_tf2(self.pointing_frame, self.last_tracked_frame)
                        if success is True:
                            look_at_point = PointStamped()
                            look_at_point.header.frame_id = self.pointing_frame
                            if self.last_tracked_frame == self.base_frame_id:
                                look_at_point.header.frame_id = self.base_frame_id
                                look_at_point.point.x = 2.0
                                look_at_point.point.y = 1.0
                                look_at_point.point.z = self.look_at_min_height
                            else:
                                look_at_point.point.x = t[0] - 1.0
                                look_at_point.point.y = t[1]
                                look_at_point.point.z = t[2]
                            self.look_at(look_at_point, timeout=3.0)
                elif joy_msg.buttons[2] == 1.0:
                    # look_at right
                    if self.last_look_at_point is not None and self.last_tracked_frame is not None:
                        success, t, _ = self.get_transform_from_tf2(self.pointing_frame, self.last_tracked_frame)
                        if success is True:
                            look_at_point = PointStamped()
                            look_at_point.header.frame_id = self.pointing_frame
                            if self.last_tracked_frame == self.base_frame_id:
                                look_at_point.header.frame_id = self.base_frame_id
                                look_at_point.point.x = 2.0
                                look_at_point.point.y = -1.0
                                look_at_point.point.z = self.look_at_min_height
                            else:
                                look_at_point.point.x = t[0] + 1.0
                                look_at_point.point.y = t[1]
                                look_at_point.point.z = t[2]
                            self.look_at(look_at_point, timeout=3.0)
                elif joy_msg.buttons[3] == 1.0:
                    # look forward
                    look_at_point = PointStamped()
                    look_at_point.header.frame_id = self.base_frame_id
                    look_at_point.point.x = 2.0
                    look_at_point.point.y = 0.0
                    look_at_point.point.z = self.look_at_min_height
                    self.last_look_at_point = look_at_point
                    self.last_tracked_frame = self.base_frame_id
                    self.look_at(look_at_point, timeout=3.0)

                self.enable_tracking = True
            else:
                command = Twist()
                command.linear.x = joy_msg.axes[1] * self.max_linear_velocity
                command.angular.z = joy_msg.axes[0] * self.max_angular_velocity
                self.cmd_vel_publisher.publish(command)
                #self.enable_tracking = True

    def look_at(self, look_at_point, look_at_height=1.35, timeout=None):
        if look_at_point.header.frame_id != "":
            if look_at_point.header.frame_id == "odom" or look_at_point.header.frame_id == "map" or look_at_point.header.frame_id == "base_footprint":
                look_at_point.point.z = look_at_height
            goal = PointHeadGoal(target=look_at_point,
                                 pointing_axis=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.0),
                                 pointing_frame=self.pointing_frame,
                                 min_duration=rospy.Duration(self.lookat_min_duration),
                                 max_velocity=self.lookat_max_velocity)
            rospy.logdebug("[supervision] Send goal : \n{}".format(goal))
            self.head_action_client.send_goal(goal)
            rospy.logdebug("[supervision] Wait for goal result...")
            if timeout is None:
                self.head_action_client.wait_for_result()
            else:
                self.head_action_client.wait_for_result(rospy.Duration(timeout))

    def get_transform_from_tf2(self, source_frame, target_frame, time=None):
        try:
            if time is not None:
                trans = self.tf_buffer.lookup_transform(source_frame, target_frame, time)
            else:
                trans = self.tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            rx = trans.transform.rotation.x
            ry = trans.transform.rotation.y
            rz = trans.transform.rotation.z
            rw = trans.transform.rotation.w

            return True, [x, y, z], [rx, ry, rz, rw]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("[supervision] Exeption occured: {}".format(e))
            return False, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    rospy.init_node("behavior_manager")
    core = BehaviorManager().run()
