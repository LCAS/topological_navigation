#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 14 11:10:28 2015

@author: cdondrup
"""

import socket
import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Header, String
import yaml
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
import actionlib
import actionlib_msgs
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from strands_navigation_msgs.msg import ExecutePolicyModeAction, ExecutePolicyModeGoal
from strands_navigation_msgs.srv import LoadTopoNavTestScenario, LoadTopoNavTestScenarioResponse, RunTopoNavTestScenario, RunTopoNavTestScenarioResponse, GetRouteTo
from roslib.packages import find_resource, get_pkg_subdir
import time
from strands_navigation_msgs.srv import GetTopologicalMap, GetTopologicalMapRequest
from tf import TransformListener
from topological_navigation.load_maps_from_yaml import YamlMapLoader
from scitos_teleop.msg import action_buttons

HOST = 'localhost'
PORT = 4000


class ScenarioServer(object):
    _id = 0
    _loaded = False
    _robot_poses = []

    def __init__(self, name):
        rospy.loginfo("Starting scenario server")
        robot = rospy.get_param("~robot", False)
        conf_file = find_resource("topological_navigation", "scenario_server.yaml")[0]
        rospy.loginfo("Reading config file: %s ..." % conf_file)
        with open(conf_file,'r') as f:
            conf = yaml.load(f)
        self._robot_start_node = conf["robot_start_node"]
        self._robot_goal_node = conf["robot_goal_node"]
        self._timeout = conf["success_metrics"]["nav_timeout"]
        rospy.loginfo(" ... done")
        self._insert_maps()
        self.tf = TransformListener()
        rospy.Service("~load", LoadTopoNavTestScenario, self.load_scenario)
        self.reset_pose = self.reset_pose_robot if robot else self.reset_pose_sim
        rospy.Service("~reset", Empty, self.reset)
        rospy.Service("~start", RunTopoNavTestScenario, self.start)
        rospy.loginfo("All done")

    def _insert_maps(self):
        map_dir = rospy.get_param("~map_dir", "")
        rospy.loginfo("Inserting maps into datacentre...")
        if map_dir == "":
            rospy.logwarn("No 'map_dir' given. Will not insert maps into datacentre and assume they are present already.")
            return
        y = YamlMapLoader()
        y.insert_maps(y.read_maps(map_dir))
        rospy.loginfo("... done")

    def robot_callback(self, msg):
        self._robot_poses.append(msg)

    def _connect_port(self, port):
        """ Establish the connection with the given MORSE port"""
        sock = None

        for res in socket.getaddrinfo(HOST, port, socket.AF_UNSPEC, socket.SOCK_STREAM):
            af, socktype, proto, canonname, sa = res
            try:
                sock = socket.socket(af, socktype, proto)
            except socket.error:
                sock = None
                continue
            try:
                sock.connect(sa)
            except socket.error:
                sock.close()
                sock = None
                continue
            break

        return sock

    def _translate(self, msg, target_tf):
        t = self.tf.getLatestCommonTime(target_tf, msg.header.frame_id)
        msg.header.stamp=t
        new_pose=self.tf.transformPose(target_tf,msg)
        return new_pose

    def _clear_costmaps(self):
        try:
            s = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
            s.wait_for_service()
            s()
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def _init_nav(self, pose):
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.sleep(1.0)
        initialPose = PoseWithCovarianceStamped()
        initialPose.header = pose.header
        initialPose.pose.pose = pose.pose
        p_cov = np.array([0.0]*36)
        initialPose.pose.covariance = tuple(p_cov.ravel().tolist())
        pub.publish(initialPose)

    def _get_set_object_pose_command(self, agent, id, pose):
        new_pose = self._translate(
            msg=pose,
            target_tf="/world"
        )
        return 'id%d simulation set_object_pose ["%s", "[%f, %f, 0.1]", "[%f, %f, %f, %f]"]\n' \
        % (id, agent, new_pose.pose.position.x, new_pose.pose.position.y, \
        new_pose.pose.orientation.w, new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z)

    def reset_pose_robot(self):
        while(self._robot_start_node != rospy.wait_for_message("/current_node", String).data):
            rospy.loginfo("+++ Please push the robot to %s +++" % self._robot_start_node)
            rospy.sleep(1)

        while(not rospy.wait_for_message("/teleop_joystick/action_buttons", action_buttons).A):
            rospy.loginfo("+++ Robot at %s please confirm correct positioning with A button on joypad +++" % self._robot_start_node)
            rospy.sleep(1)

    def reset_pose_sim(self):
        sock = self._connect_port(PORT)
        if not sock:
            raise Exception("Could not create socket connection to morse")

        sock.send(
            self._get_set_object_pose_command(
                "robot",
                self._id,
                self._robot_start_pose
            )
        )
        self._id += 1

        sock.close()

    def reset(self, req):
        if not self._loaded:
            rospy.logfatal("No scenario loaded!")
            return EmptyResponse()

        rospy.loginfo("Resetting robot position...")

        self.reset_pose()

        self._init_nav(self._robot_start_pose)
        self._clear_costmaps()
        rospy.loginfo("... done")
        return EmptyResponse()

    def graceful_fail(self):
        closest_node = rospy.wait_for_message("/closest_node", String).data
        rospy.loginfo("Closest node: %s" % closest_node)
        if closest_node != self._robot_start_node:
            rospy.loginfo("Using policy execution from %s to %s" % (closest_node, self._robot_start_node))
            s = rospy.ServiceProxy("/get_simple_policy/get_route_to", GetRouteTo)
            s.wait_for_service()
            policy = s(self._robot_start_node)
            self.client.send_goal(ExecutePolicyModeGoal(route=policy.route))
            self.client.wait_for_result(timeout=rospy.Duration(self._timeout))
            return self.client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED
        else:
            rospy.loginfo("Using topo nav from %s to %s" % (closest_node, self._robot_start_node))
            rospy.loginfo("Starting topo nav client...")
            client = actionlib.SimpleActionClient("/topological_navigation", GotoNodeAction)
            client.wait_for_server()
            rospy.loginfo(" ... done")
            client.send_goal(GotoNodeGoal(target=self._robot_start_node))
            client.wait_for_result(timeout=rospy.Duration(self._timeout))
            return client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED

    def start(self, req):
        rospy.loginfo("Starting test ...")
        if not self._loaded:
            rospy.logfatal("No scenario loaded!")
            return RunTopoNavTestScenarioResponse(False, False)

        self._robot_poses = []
        grace_res = False
        rospy.Subscriber("/robot_pose", Pose, self.robot_callback)
        rospy.loginfo("Sending goal to policy execution ...")
        self.client.send_goal(ExecutePolicyModeGoal(route=self._policy.route))
        t = time.time()
        rospy.loginfo("... waiting for result ...")
        self.client.wait_for_result(timeout=rospy.Duration(self._timeout))
        elapsed = time.time() - t
        res = self.client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED
        rospy.loginfo("... policy execution finished")
        self.client.cancel_all_goals()
        if not res:
            rospy.loginfo("Attemting graceful death ...")
            grace_res = self.graceful_fail()
            rospy.loginfo("... dead")
        distance = self.get_distance_travelled(self._robot_poses)
        rospy.loginfo("... test done")
        return RunTopoNavTestScenarioResponse(
            nav_success=res,
            graceful_fail=grace_res,
            human_success=False,
            min_distance_to_human=0,
            distance_travelled=distance,
            travel_time=elapsed,
            mean_speed=distance/elapsed
        )

    def get_distance_travelled(self, poses):
        distance = 0.0
        for idx in range(len(poses))[1:]:
            distance += np.sqrt(
                [((poses[idx].position.x - poses[idx-1].position.x)**2) \
                + ((poses[idx].position.y - poses[idx-1].position.y)**2)]
            )
        return distance

    def load_scenario(self, req):
        # No try except to have exception break the test
        s = rospy.ServiceProxy("/topological_map_manager/switch_topological_map", GetTopologicalMap)
        s.wait_for_service()
        topo_map = s(GetTopologicalMapRequest(pointset=req.pointset)).map

        self._robot_start_pose = None
        found_end_node = False
        for node in topo_map.nodes:
            if node.name == self._robot_start_node:
                self._robot_start_pose = PoseStamped(
                        header=Header(stamp=rospy.Time.now(), frame_id="/map"),
                        pose=node.pose
                )
            elif node.name == self._robot_goal_node:
                found_end_node = True

        if self._robot_start_pose == None:
            raise Exception("Topological map '%s' does not contain start node '%s'." % (req.pointset, self._robot_start_node))
        if not found_end_node:
            raise Exception("Topological map '%s' does not contain goal node '%s'." % (req.pointset, self._robot_goal_node))

        rospy.loginfo("Starting policy execution client...")
        # Has to be done here because the policy execution server waits for a topo map.
        self.client = actionlib.SimpleActionClient("/topological_navigation/execute_policy_mode", ExecutePolicyModeAction)
        self.client.wait_for_server()
        rospy.loginfo(" ... started")

        self._loaded = True
        self.reset(None)

        # No try except to have exception break the test
        rospy.loginfo("Getting route ...")
        s = rospy.ServiceProxy("/get_simple_policy/get_route_to", GetRouteTo)
        s.wait_for_service()
        self._policy = s(self._robot_goal_node)
        rospy.loginfo(" ... done")

        return LoadTopoNavTestScenarioResponse()

if __name__ == "__main__":
    rospy.init_node("scenario_server")
    s = ScenarioServer(rospy.get_name())
    rospy.spin()
