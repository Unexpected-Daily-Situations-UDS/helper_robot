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


class BehaviorManager(object):
    def __init__(self):
        rospy.loginfo("[supervision] Starting the robot behavior manager...")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        rospy.sleep(0.5)

        self.point_head_action_srv = rospy.get_param("~point_head_action_srv", "point_head_action")
        self.head_action_client = actionlib.SimpleActionClient(self.point_head_action_srv, PointHeadAction)

        self.lookat_min_duration = rospy.get_param("~lookat_min_duration", 0.2)
        self.lookat_max_velocity = rospy.get_param("~lookat_max_velocity", 0.1)

        self.pointing_frame = rospy.get_param("~pointing_frame", "")

        self.look_at_min_height = rospy.get_param("~look_at_height", 1.35)

        rospy.loginfo("[supervision] Waiting '{}' action server...".format(self.point_head_action_srv))
        if not self.head_action_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Not able to connect to '{}' action server ! Check that you are connected to the robot, if so check the ROS_MASTER_URI and ROS_IP.".format(self.point_head_action_srv))
            sys.exit()

        self.tracks_topic = rospy.get_param("~tracks_topic", "tracks")
        rospy.loginfo("[supervision] Subscribing to '/{}' topic...".format(self.tracks_topic))
        self.track_subscriber = rospy.Subscriber(self.tracks_topic, SceneNodeArrayStamped, self.observation_callback, queue_size=1)

        rospy.loginfo("[supervision] Behavior manager ready !")

    def observation_callback(self, tracks_msg):
        track_to_lookat = None
        min_dist = 10000
        rospy.logdebug("[supervision] Tracks received, triggering callback...")
        for track in tracks_msg.nodes:
            if track.label == "person" or track.label == "face":
                if track.is_located is True:
                    success, t, _ = self.get_transform_from_tf2(self.pointing_frame, track.id)#, time=tracks_msg.header.stamp)
                    if success is True:
                        dist = math.sqrt(math.pow(t[0], 2)+ math.pow(t[1], 2)+ math.pow(t[2], 2))
                        if min_dist > dist:
                            track_to_lookat = track
                            min_dist = dist

        if track_to_lookat is not None:
            self.look_at(track_to_lookat)

    def look_at(self, scene_node):
        if scene_node.position.header.frame_id != "":
            look_at_point = PointStamped()
            look_at_point.header = scene_node.position.header
            look_at_point.header.stamp = rospy.Time(0)
            look_at_point.point = scene_node.position.pose.pose.position
            if scene_node.position.header.frame_id == "odom" or scene_node.position.header.frame_id == "map":
                if look_at_point.point.z < self.look_at_min_height:
                    look_at_point.point.z = self.look_at_min_height
            goal = PointHeadGoal(target=look_at_point,
                                 pointing_axis=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.0),
                                 pointing_frame=self.pointing_frame,
                                 min_duration=rospy.Duration(self.lookat_min_duration),
                                 max_velocity=self.lookat_max_velocity)
            rospy.logdebug("[supervision] Send goal : \n{}".format(goal))
            self.head_action_client.send_goal(goal)
            rospy.logdebug("[supervision] Wait for goal result...")
            self.head_action_client.wait_for_result(rospy.Duration(self.lookat_min_duration))

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
