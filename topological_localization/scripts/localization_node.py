import rospy
import threading
from topological_localization.particle_filter import TopologicalParticleFilter
from topological_localization.prediction_model import PredictionModel
from topological_localization.srv import LocalizeAgent, LocalizeAgentResponse, StopLocalize, StopLocalizeResponse
from topological_localization.msg import ProbabilityDistributionStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from strands_navigation_msgs.msg import TopologicalMap
from std_msgs.msg import String

class TopologicalLocalization():

    def __init__(self):
        # agents currently tracking
        self.agents = []
        # request info for each agent
        self.requests = []
        # particles filters for each agent
        self.particle_filters = []
        # observation subscribers for each agent
        self.obs_subscribers = []
        # # publishers localization result for each agent
        # self.res_publishers = []
        # # publishers viz result for each agent
        # self.viz_publishers = []
        # thread that loop predictions at fixed rate for each agent
        self.prediction_threads = []

        # these will contain info about the topology
        self.topo_map = None
        self.node_diffs2D = []
        self.node_distances = []
        self.connected_nodes = []
        self.node_names = []
        self.node_coords = []

        # declare services
        rospy.Service("~localize_agent", LocalizeAgent, self._localize_agent_handler)
        rospy.Service("~stop_localize", StopLocalize, self._stop_localize_handler)

        rospy.Subscriber("topological_map", TopologicalMap, self._topo_map_cb)

        rospy.loginfo("Waiting for topological map...")
        while self.topo_map is None:
            rospy.sleep(0.5)

        rospy.loginfo("DONE")

    def _localize_agent_handler(self, request):
        rospy.loginfo("Received request to localize new agent {}".format(request.name))
        if request.name in self.agents:
            rospy.logwarn("Agent {} already being localized".format(request.name))
            return LocalizeAgentResponse(False)

        # Initialize the prediction model
        if request.prediction_model == LocalizeAgent.PRED_CTMT:
            pm = PredictionModel(
                pred_type=PredictionModel.CTMC,
                node_coords=self.node_coords,
                node_diffs2D=self.node_diffs2D,
                node_distances=self.node_distances,
                connected_nodes=self.connected_nodes
            )
        elif request.prediction_model == LocalizeAgent.PRED_IDENTITY:
            pm = PredictionModel(
                pred_type=PredictionModel.IDENTITY
            )
        else:
            rospy.logerr(
                "Prediction model {} unknown".format(request.prediction_model))
            return LocalizeAgentResponse(False)

        # Initialize publishers and messages
        cn_pub = rospy.Publisher("{}/current_node".format(request.name), String, queue_size=10, latch=True)
        pd_pub = rospy.Publisher("{}/prob_dist_topoloc".format(request.name), ProbabilityDistributionStamped, queue_size=10, latch=True)
        # self.res_publishers.append((cn_pub, pd_pub))
        strmsg = String()
        pdmsg = ProbabilityDistributionStamped()
        cnviz_pub = rospy.Publisher("{}/current_node_viz".format(request.name), Marker, queue_size=10)
        parviz_pub = rospy.Publisher("{}/particles_viz".format(request.name), MarkerArray, queue_size=10)
        # self.viz_publishers.append((cnviz_pub, parviz_pub))
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
        for i in range(request.n_particles):
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
        if request.initial_spread_policy == LocalizeAgent.CLOSEST_NODE:
            sigma = -1
        elif request.initial_spread_policy == LocalizeAgent.SPREAD_RADIUS:
            sigma = request.initial_spread_radius
        elif request.initial_spread_policy == LocalizeAgent.SPREAD_UNIFORM:
            sigma = np.inf
        else:
            sigma = -1

        # Initialize a new instance of particle_filter
        pf = TopologicalParticleFilter(
            num=request.n_particles,
            prediction_model=pm,
            prediction_speed_decay=request.prediction_speed_decay,
            node_coords=self.node_coords,
            node_distances=self.node_distances,
            connected_nodes=self.connected_nodes,
            node_diffs2D=self.node_diffs2D
        )

        # function to publish current node and particles distribution
        def __publish(node, particles):
            nodes, counts = np.unique(particles, return_counts=True)

            node_names = [self.node_names[node] for node in nodes]
            probs = counts / np.sum(counts)
            
            strmsg.data = node
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
                msg.header.stamp.to_sec()
            )
            __publish(node, particles)


        # send probability dist observation to particle filter
        def __prob_dist_obs_cb(msg):
            nodes = [self.node_names.index(nname) for nname in msg.nodes]
            node, particles = pf.receive_prob_dist_obs(
                nodes, 
                msg.probabilities,
                msg.header.stamp.to_sec()
            )
            __publish(node, particles)

        # subscribe to topics receiving observation
        self.obs_subscribers.append((
            rospy.Subscriber("{}/pose_obs".format(request.name), PoseWithCovarianceStamped, __pose_obs_cb),
            rospy.Subscriber("{}/prob_dist_obs".format(request.name),
                             ProbabilityDistributionStamped, __prob_dist_obs_cb)
        ))
        
        thr = None
        if request.do_prediction:
            # threaded function that performs predictions at a constant rate
            def __prediction_loop():
                rate = rospy.Rate(request.prediction_rate())
                while not rospy.is_shutdown():
                    node, particles = pf.predict(
                        timestamp_secs=rospy.get_rostime().to_sec()
                    )
                    __publish(node, particles)
                    
                    rate.sleep()
            
            thr = threading.Thread(target=__prediction_loop)
            thr.start()

        self.prediction_threads.append(thr)
        self.agents.append(request.name)
        self.particle_filters.append(pf)
        self.requests.append(request)

        return LocalizeAgentResponse(True)

    def _stop_localize_handler(self, request):
        # TODO stop pfs
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
            self.node_distances.append(
                np.sqrt(np.sum(self.node_diffs2D[i] ** 2, axis=1)))
            self.connected_nodes.append([np.where(self.node_names == edge.node)[
                                        0][0] for edge in self.topo_map.nodes[i].edges])

        self.node_diffs2D = np.array(self.node_diffs2D)
        self.node_distances = np.array(self.node_distances)
        self.connected_nodes = np.array(self.connected_nodes)

    def close(self):
        # TODO stop all the threads here
        for thr in self.prediction_threads:
            thr.join()


if __name__ == "__main__":
    rospy.init_node("topological_localization")

    localization_node = TopologicalLocalization()

    rospy.spin()

    localization_node.close()
