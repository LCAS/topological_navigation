#!/usr/bin/env python
import rospy
import threading
import numpy as np
from topological_localization.particle_filter import TopologicalParticleFilter
from topological_localization.prediction_model import PredictionModel
from topological_localization.srv import LocalizeAgent, LocalizeAgentRequest, LocalizeAgentResponse, StopLocalize, StopLocalizeRequest, StopLocalizeResponse
from topological_localization.msg import ProbabilityDistributionStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from strands_navigation_msgs.msg import TopologicalMap
from std_msgs.msg import String

class TopologicalLocalization():

    def __init__(self):
        # agents currently tracking
        self.agents = []
        # observation subscribers for each agent
        self.obs_subscribers = []
        # # publishers localization result for each agent
        self.res_publishers = []
        # # publishers viz result for each agent
        self.viz_publishers = []
        # thread that loop predictions at fixed rate for each agent
        self.prediction_threads = []

        # these will contain info about the topology
        self.topo_map = None
        self.node_diffs2D = []
        self.node_distances = []
        self.connected_nodes = []
        self.node_names = []
        self.node_coords = []

        # contains a list of threading.Event for stopping the localization of each agent
        self.stopping_events = []

        # to avoid inconsistencies when registering/unregistering agents concurrently
        self.internal_lock = threading.Lock()

        # declare services
        rospy.Service("~localize_agent", LocalizeAgent, self._localize_agent_handler)
        rospy.Service("~stop_localize", StopLocalize, self._stop_localize_handler)

        rospy.Subscriber("topological_map", TopologicalMap, self._topo_map_cb)

        rospy.loginfo("Waiting for topological map...")
        while self.topo_map is None:
            rospy.sleep(0.5)

        rospy.loginfo("DONE")

    def _localize_agent_handler(self, request):
        # lock resources
        rospy.loginfo("Received request to localize new agent {}".format(request.name))
        self.internal_lock.acquire()

        ## set default values ##
        # default name is unknown if requested is ''
        name = (request.name, 'unknown')[request.name == '']
        # default particles number is 300 if requested is 0
        n_particles = (request.n_particles, 300)[request.n_particles <= 0]
        initial_spread_policy = request.initial_spread_policy
        prediction_model = request.prediction_model
        do_prediction = request.do_prediction
        # default prediction rate is 0.5 if requested is 0.
        prediction_rate = (request.prediction_rate, 0.5)[
            request.prediction_rate <= 0.]
        # default speed decay is 1 if requested is 0
        prediction_speed_decay = (request.prediction_speed_decay, 1.0)[
            request.prediction_speed_decay <= 0]

        if name in self.agents:
            rospy.logwarn("Agent {} already being localized".format(name))
            # release resources
            self.internal_lock.release()
            return LocalizeAgentResponse(False)

        # Initialize the prediction model
        if prediction_model == LocalizeAgentRequest.PRED_CTMC:
            pm = PredictionModel(
                pred_type=PredictionModel.CTMC,
                node_coords=self.node_coords,
                node_diffs2D=self.node_diffs2D,
                node_distances=self.node_distances,
                connected_nodes=self.connected_nodes
            )
        elif prediction_model == LocalizeAgentRequest.PRED_IDENTITY:
            pm = PredictionModel(
                pred_type=PredictionModel.IDENTITY
            )
        else:
            rospy.logerr(
                "Prediction model {} unknown".format(prediction_model))
            # release resources
            self.internal_lock.release()
            return LocalizeAgentResponse(False)


        # Initialize publishers and messages
        cn_pub = rospy.Publisher("{}/current_node".format(name), String, queue_size=10, latch=True)
        pd_pub = rospy.Publisher("{}/current_prob_dist".format(name), ProbabilityDistributionStamped, queue_size=10, latch=True)
        self.res_publishers.append((cn_pub, pd_pub))
        strmsg = String()
        pdmsg = ProbabilityDistributionStamped()
        cnviz_pub = rospy.Publisher("{}/current_node_viz".format(name), Marker, queue_size=10)
        parviz_pub = rospy.Publisher("{}/particles_viz".format(name), MarkerArray, queue_size=10)
        self.viz_publishers.append((cnviz_pub, parviz_pub))
        nodemkrmsg = Marker()
        nodemkrmsg.header.frame_id = "/map"
        nodemkrmsg.type = nodemkrmsg.SPHERE
        nodemkrmsg.pose.position.z = 6
        nodemkrmsg.pose.orientation.w = 1
        nodemkrmsg.scale.x = 0.5
        nodemkrmsg.scale.y = 0.5
        nodemkrmsg.scale.z = 0.5
        nodemkrmsg.color.a = 1
        nodemkrmsg.color.r = 0
        nodemkrmsg.color.g = 0
        nodemkrmsg.color.b = 1
        nodemkrmsg.id = 0
        ptcsarrmsg = MarkerArray()
        for i in range(n_particles):
            ptcmkrmsg = Marker()
            ptcmkrmsg.header.frame_id = "/map"
            ptcmkrmsg.type = ptcmkrmsg.SPHERE
            ptcmkrmsg.pose.position.z = 0
            ptcmkrmsg.pose.orientation.w = 1
            ptcmkrmsg.scale.x = 0.1
            ptcmkrmsg.scale.y = 0.1
            ptcmkrmsg.scale.z = 0.1
            ptcmkrmsg.color.a = 0.6
            ptcmkrmsg.color.r = 1
            ptcmkrmsg.color.g = 0
            ptcmkrmsg.color.b = 0
            ptcmkrmsg.id = i
            ptcsarrmsg.markers.append(ptcmkrmsg)

        # get the how to spread the particles initially
        # if request.initial_spread_policy == LocalizeAgentRequest.CLOSEST_NODE:
        #     sigma = -1
        # elif request.initial_spread_policy == LocalizeAgentRequest.SPREAD_RADIUS:
        #     sigma = request.initial_spread_radius
        # elif request.initial_spread_policy == LocalizeAgentRequest.SPREAD_UNIFORM:
        #     sigma = np.inf
        # else:
        #     sigma = -1

        # Initialize a new instance of particle_filter
        pf = TopologicalParticleFilter(
            num=n_particles,
            prediction_model=pm,
            prediction_speed_decay=prediction_speed_decay,
            node_coords=self.node_coords,
            node_distances=self.node_distances,
            connected_nodes=self.connected_nodes,
            node_diffs2D=self.node_diffs2D,
            node_names=self.node_names
        )

        # function to publish current node and particles distribution
        def __publish(node, particles):
            nodes, counts = np.unique(particles, return_counts=True)

            node_names = [self.node_names[_node] for _node in nodes]
            probs = counts / np.sum(counts)
            
            strmsg.data = self.node_names[node]
            pdmsg.header.stamp = rospy.get_rostime()
            pdmsg.nodes = node_names
            pdmsg.probabilities = probs

            cn_pub.publish(strmsg)
            pd_pub.publish(pdmsg)

            # publish viz stuff
            for i, p in enumerate(particles):
                ptcsarrmsg.markers[i].header.stamp = rospy.get_rostime()
                ptcsarrmsg.markers[i].pose.position.x = self.node_coords[p][0] + \
                    ptcsarrmsg.markers[i].scale.x * np.random.randn(1, 1)
                ptcsarrmsg.markers[i].pose.position.y = self.node_coords[p][1] + \
                    ptcsarrmsg.markers[i].scale.y * np.random.randn(1, 1)
            nodemkrmsg.pose.position.x = self.node_coords[node][0]
            nodemkrmsg.pose.position.y = self.node_coords[node][1]

            cnviz_pub.publish(nodemkrmsg)
            parviz_pub.publish(ptcsarrmsg)


        # send pose observation to particle filter
        def __pose_obs_cb(msg):
            node, particles = pf.receive_pose_obs(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.covariance[0], # variance of x
                msg.pose.covariance[7], # variance of y
                (rospy.get_rostime().to_sec(), msg.header.stamp.to_sec())[
                    msg.header.stamp.to_sec() > 0]
            )
            __publish(node, particles)


        # send probability dist observation to particle filter
        def __prob_dist_obs_cb(msg):
            nodes = [np.where(self.node_names == nname)[0][0] for nname in msg.nodes]
            node, particles = pf.receive_prob_dist_obs(
                nodes, 
                msg.probabilities,
                (rospy.get_rostime().to_sec(), msg.header.stamp.to_sec())[
                    msg.header.stamp.to_sec() > 0]
            )
            __publish(node, particles)

        # subscribe to topics receiving observation
        self.obs_subscribers.append((
            rospy.Subscriber("{}/pose_obs".format(name), PoseWithCovarianceStamped, __pose_obs_cb),
            rospy.Subscriber("{}/prob_dist_obs".format(name),
                             ProbabilityDistributionStamped, __prob_dist_obs_cb)
        ))
        
        thr = None
        stop_event = None
        if do_prediction:
            # threaded function that performs predictions at a constant rate
            stop_event = threading.Event()
            def __prediction_loop():
                rate = rospy.Rate(prediction_rate)
                while not stop_event.is_set():
                    node, particles = pf.predict(
                        timestamp_secs=rospy.get_rostime().to_sec()
                    )
                    if not (node is None or particles is None): 
                        __publish(node, particles)
                    
                    rate.sleep()
            
            thr = threading.Thread(target=__prediction_loop)
            thr.start()

        self.stopping_events.append(stop_event)
        self.prediction_threads.append(thr)
        self.agents.append(name)


        rospy.loginfo("n_particles:{}, initial_spread_policy:{}, prediction_model:{}, do_prediction:{}, prediction_rate:{}. prediction_speed_decay:{}".format(
            n_particles, initial_spread_policy, prediction_model, do_prediction, prediction_rate, prediction_speed_decay
        ))

        # release resources
        self.internal_lock.release()

        rospy.loginfo("DONE")

        return LocalizeAgentResponse(True)

    def _stop_localize_handler(self, request):
        rospy.loginfo("Unregistering agent {} for localization".format(request.name))
        self.internal_lock.acquire()
        # default name is unknown if requested is ''
        name = (request.name, 'unknown')[request.name == '']
        if name in self.agents:
            agent_idx = self.agents.index(name)
            # stop prediction loop
            if self.stopping_events[agent_idx] is not None:
                self.stopping_events[agent_idx].set()
            if self.prediction_threads[agent_idx] is not None:
                self.prediction_threads[agent_idx].join()

            # unregister topic pubs/subs
            for sub in self.obs_subscribers[agent_idx]:
                sub.unregister()
            for pub in self.res_publishers[agent_idx]:
                pub.unregister()
            for pub in self.viz_publishers[agent_idx]:
                pub.unregister()

            # cleanup all the corresponding variables
            del self.stopping_events[agent_idx]
            del self.prediction_threads[agent_idx]
            del self.obs_subscribers[agent_idx]
            del self.res_publishers[agent_idx]
            del self.viz_publishers[agent_idx]
            del self.agents[agent_idx]

            self.internal_lock.release()
            rospy.loginfo("DONE")
            return StopLocalizeResponse(True)
        else:
            rospy.logwarn("The agent {} is already not being localized.".format(name))
            self.internal_lock.release()
            return StopLocalizeResponse(False)

    def _topo_map_cb(self, msg):
        """This function receives the Topological Map"""
        self.topo_map = msg

        # save and compute topological map informations
        self.node_names = np.array([node.name for node in self.topo_map.nodes])
        self.node_coords = np.array(
            [[node.pose.position.x, node.pose.position.y] for node in self.topo_map.nodes])

        self.node_diffs2D = []
        self.node_distances = []
        self.connected_nodes = []
        for i, _ in enumerate(self.node_names):
            self.node_diffs2D.append(self.node_coords - self.node_coords[i])
            self.connected_nodes.append(np.array([np.where(self.node_names == edge.node)[
                                        0][0] for edge in self.topo_map.nodes[i].edges]))

        self.node_diffs2D = np.array(self.node_diffs2D)
        self.connected_nodes = np.array(self.connected_nodes)
        
        self.node_distances = np.sqrt(np.sum(self.node_diffs2D ** 2, axis=2))
        
        # print("self.node_diffs2D", self.node_diffs2D.shape)
        # print("self.node_distances", self.node_distances.shape)
        # print("self.connected_nodes", self.connected_nodes.shape)

    def close(self):
        # stop all the threads
        for thr, stop_event in zip(self.prediction_threads, self.stopping_events):
            stop_event.set()
            thr.join()


if __name__ == "__main__":
    rospy.init_node("topological_localization")

    localization_node = TopologicalLocalization()

    rospy.spin()

    localization_node.close()
