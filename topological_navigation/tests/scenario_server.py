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
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped, Point
import actionlib
import actionlib_msgs
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from topological_navigation_msgs.msg import ExecutePolicyModeAction, ExecutePolicyModeGoal
from topological_navigation_msgs.srv import LoadTopoNavTestScenario, LoadTopoNavTestScenarioResponse, RunTopoNavTestScenario, RunTopoNavTestScenarioResponse, GetRouteTo
from roslib.packages import find_resource
import time
from topological_navigation_msgs.srv import GetTopologicalMap, GetTopologicalMapRequest
from tf import TransformListener
import tf.transformations as trans
from topological_navigation.load_maps_from_yaml import YamlMapLoader
from scitos_teleop.msg import action_buttons
from scitos_msgs.srv import EnableMotors, ResetBarrierStop, ResetMotorStop

HOST = 'localhost'
PORT = 4000
PKG = "topological_navigation"


class ScenarioServer(object):
    _id = 0
    _loaded = False
    _robot_poses = []
    _distances = [0,0]

    def __init__(self, name):
        rospy.loginfo("Starting scenario server")
        self.robot = rospy.get_param("~robot", False)
        conf_file = find_resource("topological_navigation", "scenario_server.yaml")[0]
        rospy.loginfo("Reading config file: %s ..." % conf_file)
        with open(conf_file,'r') as f:
            conf = yaml.load(f)
        self._robot_start_node = conf["robot_start_node"]
        self._robot_goal_node = conf["robot_goal_node"]
        self._obstacle_node_prefix = conf["obstacle_node_prefix"]
        self._obstacle_types = conf["obstacle_types"]
        self._timeout = conf["success_metrics"]["nav_timeout"]
        rospy.loginfo(" ... done")
        self._insert_maps()
        self.tf = TransformListener()
        rospy.Service("~load", LoadTopoNavTestScenario, self.load_scenario)
        self.reset_pose = self.reset_pose_robot if self.robot else self.reset_pose_sim
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
            s = rospy.ServiceProxy("move_base/clear_costmaps", Empty)
            s.wait_for_service()
            s()
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def _init_nav(self, pose):
        pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
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

    def robot_start_dist(self, msg):
        self._distances = self._get_pose_distance(msg, self._robot_start_pose.pose)

    def reset_pose_robot(self):
        rospy.loginfo("Enabling freerun ...")
        try:
            s = rospy.ServiceProxy("enable_motors", EnableMotors)
            s.wait_for_service()
            s(False)
        except (rospy.ROSInterruptException, rospy.ServiceException) as e:
            rospy.logwarn(e)
        rospy.loginfo("... enabled")

        while self._robot_start_node != rospy.wait_for_message("current_node", String).data and not rospy.is_shutdown():
            rospy.loginfo("+++ Please push the robot to '%s' +++" % self._robot_start_node)
            rospy.sleep(1)
        rospy.loginfo("+++ Robot at '%s' +++" % self._robot_start_node)

        while not rospy.is_shutdown():
            sub = rospy.Subscriber("robot_pose", Pose, self.robot_start_dist)
            rospy.loginfo("+++ Please confirm correct positioning with A button on joypad: distance %.2fm %.2fdeg +++" %(self._distances[0], self._distances[1]))
            if self._robot_start_node != rospy.wait_for_message("current_node", String).data:
                self.reset_pose()
                return
            try:
                if rospy.wait_for_message("teleop_joystick/action_buttons", action_buttons, timeout=1.).A:
                    break
            except rospy.ROSException:
                pass
        sub.unregister()
        sub = None
        rospy.loginfo("+++ Position confirmed +++")

        rospy.loginfo("Enabling motors, resetting motor stop and barrier stopped ...")
        try:
            s = rospy.ServiceProxy("enable_motors", EnableMotors)
            s.wait_for_service()
            s(True)
            s = rospy.ServiceProxy("reset_motorstop", ResetMotorStop)
            s.wait_for_service()
            s()
            s = rospy.ServiceProxy("reset_barrier_stop", ResetBarrierStop)
            s.wait_for_service()
            s()
        except (rospy.ROSInterruptException, rospy.ServiceException) as e:
            rospy.logwarn(e)
        rospy.loginfo("... enabled and reset")

    def _send_socket(self, sock, agent, pose):
        sock.send(
            self._get_set_object_pose_command(
                agent,
                self._id,
                pose
            )
        )
        res = sock.recv(4096)
        self._id += 1
        if "FAILURE" in res:
            raise Exception(res)

    def _get_quaternion_distance(self, q1, q2):
        q1 = (q1.x, q1.y, q1.z, q1.w)
        q2 = (q2.x, q2.y, q2.z, q2.w)
        return (trans.euler_from_quaternion(q1)[2] - trans.euler_from_quaternion(q2)[2]) * 180/np.pi

    def _get_euclidean_distance(self, p1, p2):
        return np.sqrt(np.power(p2.x - p1.x, 2) + np.power(p2.y - p1.y, 2))

    def _get_pose_distance(self, p1, p2):
        e = self._get_euclidean_distance(p1.position, p2.position)
        q = self._get_quaternion_distance(p1.orientation, p2.orientation)
        return e, q

    def reset_pose_sim(self):
        sock = self._connect_port(PORT)
        if not sock:
            raise Exception("Could not create socket connection to morse")

        # Clean up whole scene
        # Create pose outside of map
        clear_pose = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id="/map"),
            pose=Pose(position=Point(x=20, y=20, z=0.01))
        )

        # Moving obstacles to clear_pose
        for obstacle in self._obstacle_types:
            for idx in range(10):
                self._send_socket(sock, obstacle+str(idx), clear_pose)

        # Setting robot and obstacles to correct position
        self._send_socket(sock, "robot", self._robot_start_pose)

        if len(self._obstacle_poses) > 0:
            for idx, d in enumerate(self._obstacle_poses):
                try:
                    obstacle = max([x for x in self._obstacle_types if x in d["name"]], key=len)
                except ValueError:
                    rospy.logwarn("No obstacle specified for obstacle node '%s', will use '%s'." % (d["name"], self._obstacle_types[0]))
                    obstacle = self._obstacle_types[0]
                rospy.loginfo("Adding obstacle %s%i" % (obstacle,idx))
                self._send_socket(sock, obstacle+str(idx), d["pose"])
        else:
            rospy.logwarn("No nodes starting with '%s' found in map. Assuming test without obstacles." % self._obstacle_node_prefix)

        sock.close()

        while not rospy.is_shutdown():
            rpose = rospy.wait_for_message("robot_pose", Pose)
            rospy.loginfo("Setting initial amcl pose ...")
            self._init_nav(self._robot_start_pose)
            dist = self._get_pose_distance(rpose, self._robot_start_pose.pose)
            if dist[0] < 0.1 and dist[1] < 10:
                break
            rospy.sleep(0.2)
#            self._robot_start_pose.header.stamp = rospy.Time.now()
        rospy.loginfo(" ... pose set.")

    def reset(self, req):
        if not self._loaded:
            rospy.logfatal("No scenario loaded!")
            return EmptyResponse()

        self.client.cancel_all_goals()

        rospy.loginfo("Resetting robot position...")

        self.reset_pose()

        self._clear_costmaps()
        rospy.loginfo("... reset successful")
        return EmptyResponse()

    def graceful_fail(self):
        res = False
        closest_node = rospy.wait_for_message("closest_node", String).data
        rospy.loginfo("Closest node: %s" % closest_node)
        if closest_node != self._robot_start_node:
            rospy.loginfo("Using policy execution from %s to %s" % (closest_node, self._robot_start_node))
            s = rospy.ServiceProxy("get_simple_policy/get_route_to", GetRouteTo)
            s.wait_for_service()
            policy = s(self._robot_start_node)
            self.client.send_goal(ExecutePolicyModeGoal(route=policy.route))
            self.client.wait_for_result(timeout=rospy.Duration(self._timeout))
            res = self.client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED
            self.client.cancel_all_goals()
        else:
            rospy.loginfo("Using topo nav from %s to %s" % (closest_node, self._robot_start_node))
            rospy.loginfo("Starting topo nav client...")
            client = actionlib.SimpleActionClient("topological_navigation", GotoNodeAction)
            client.wait_for_server()
            rospy.loginfo(" ... done")
            client.send_goal(GotoNodeGoal(target=self._robot_start_node))
            client.wait_for_result(timeout=rospy.Duration(self._timeout))
            res = client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED
            client.cancel_all_goals()
        return res

    def start(self, req):
        rospy.loginfo("Starting test ...")

        if not self._loaded:
            rospy.logfatal("No scenario loaded!")
            return RunTopoNavTestScenarioResponse(False, False)

        self._robot_poses = []
        grace_res = False
        sub = rospy.Subscriber("robot_pose", Pose, self.robot_callback)
        rospy.loginfo("Sending goal to policy execution ...")
        print self._policy.route
        self.client.send_goal(ExecutePolicyModeGoal(route=self._policy.route))
        t = time.time()
        rospy.loginfo("... waiting for result ...")
        print self._timeout
        nav_timeout = not self.client.wait_for_result(timeout=rospy.Duration(self._timeout))
        elapsed = time.time() - t
        res = self.client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED
        rospy.loginfo("... policy execution finished")
        self.client.cancel_all_goals()
        if not res:
            rospy.loginfo("Attempting graceful death ...")
            grace_res = self.graceful_fail()
            rospy.loginfo("... dead")
        sub.unregister()
        sub = None
        distance = self.get_distance_travelled(self._robot_poses)
        rospy.loginfo("... test done")
        return RunTopoNavTestScenarioResponse(
            nav_success=res,
            nav_timeout=nav_timeout,
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
            distance += self._get_euclidean_distance(poses[idx].position, poses[idx-1].position)
        return distance

    def load_scenario(self, req):
        # No try except to have exception break the test
        s = rospy.ServiceProxy("/topological_map_manager/switch_topological_map", GetTopologicalMap)
        s.wait_for_service()
        topo_map = s(GetTopologicalMapRequest(pointset=req.pointset)).map

        self.pointset = req.pointset

        self._robot_start_pose = None
        self._obstacle_poses = []
        found_end_node = False
        for node in topo_map.nodes:
            if node.name == self._robot_start_node:
                self._robot_start_pose = PoseStamped(
                    header=Header(stamp=rospy.Time.now(), frame_id="/map"),
                    pose=node.pose
                )
            elif node.name == self._robot_goal_node:
                found_end_node = True
            elif node.name.startswith(self._obstacle_node_prefix):
                self._obstacle_poses.append({
                    "name": node.name.lower(),
                    "pose": PoseStamped(
                        header=Header(stamp=rospy.Time.now(), frame_id="/map"),
                        pose=node.pose
                    )
                })

        if self._robot_start_pose == None:
            raise Exception("Topological map '%s' does not contain start node '%s'." % (req.pointset, self._robot_start_node))
        if not found_end_node:
            raise Exception("Topological map '%s' does not contain goal node '%s'." % (req.pointset, self._robot_goal_node))

        rospy.loginfo("Starting policy execution client...")
        # Has to be done here because the policy execution server waits for a topo map.
        self.client = actionlib.SimpleActionClient("topological_navigation/execute_policy_mode", ExecutePolicyModeAction)
        self.client.wait_for_server()
        rospy.loginfo(" ... started")

        self._loaded = True
        self.reset(None)

        # No try except to have exception break the test
        rospy.loginfo("Getting route ...")
        s = rospy.ServiceProxy("get_simple_policy/get_route_to", GetRouteTo)
        s.wait_for_service()
        self._policy = s(self._robot_goal_node)
        rospy.loginfo(" ... done")

        return LoadTopoNavTestScenarioResponse()

if __name__ == "__main__":
    rospy.init_node("scenario_server")
    s = ScenarioServer(rospy.get_name())
    rospy.spin()
