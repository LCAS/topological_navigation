#!/usr/bin/env python
import rospy
import threading
import numpy as np
from bayesian_topological_localisation.particle_filter import TopologicalParticleFilter
from bayesian_topological_localisation.prediction_model import PredictionModel
from bayesian_topological_localisation.srv import LocaliseAgent, LocaliseAgentRequest, \
    LocaliseAgentResponse, StopLocalise, StopLocaliseRequest, StopLocaliseResponse, \
    UpdatePoseObservation, UpdatePoseObservationRequest, UpdatePoseObservationResponse, \
    UpdateLikelihoodObservation, UpdateLikelihoodObservationRequest, UpdateLikelihoodObservationResponse, \
    UpdatePriorLikelihoodObservation, UpdatePriorLikelihoodObservationRequest, UpdatePriorLikelihoodObservationResponse, \
    Predict, PredictRequest, PredictResponse
from bayesian_topological_localisation.msg import DistributionStamped, PoseObservation, LikelihoodObservation
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from strands_navigation_msgs.msg import TopologicalMap
from std_msgs.msg import String

class TopologicalLocalisation():

    def __init__(self):
        # agents currently tracking
        self.agents = []
        # observation subscribers for each agent
        self.obs_subscribers = []
        # # publishers localisation result for each agent
        self.res_publishers = []
        # # publishers viz result for each agent
        self.viz_publishers = []
        # services for updating the state estimation
        self.upd_services = []
        # thread that loop predictions at fixed rate for each agent
        self.prediction_threads = []

        # these will contain info about the topology
        self.topo_map = None
        self.node_diffs2D = []
        self.node_distances = []
        self.connected_nodes = []
        self.node_names = []
        self.node_coords = []

        # contains a list of threading.Event for stopping the localisation of each agent
        self.stopping_events = []

        # to avoid inconsistencies when registering/unregistering agents concurrently
        self.internal_lock = threading.Lock()

        # declare services
        rospy.Service("~localise_agent", LocaliseAgent, self._localise_agent_handler)
        rospy.Service("~stop_localise", StopLocalise, self._stop_localise_handler)

        rospy.Subscriber("topological_map", TopologicalMap, self._topo_map_cb)

        rospy.loginfo("Waiting for topological map...")
        while self.topo_map is None:
            rospy.sleep(0.5)

        rospy.loginfo("DONE")

    def _localise_agent_handler(self, request):
        # lock resources
        rospy.loginfo("Received request to localise new agent {}".format(request.name))
        self.internal_lock.acquire()

        # to stop executing in other threads/cbs
        stop_event = threading.Event()

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
            rospy.logwarn("Agent {} already being localised".format(name))
            # release resources
            self.internal_lock.release()
            return LocaliseAgentResponse(False)

        # Initialize the prediction model
        if prediction_model == LocaliseAgentRequest.PRED_CTMC:
            pm = PredictionModel(
                pred_type=PredictionModel.CTMC,
                node_coords=self.node_coords,
                node_diffs2D=self.node_diffs2D,
                node_distances=self.node_distances,
                connected_nodes=self.connected_nodes
            )
        elif prediction_model == LocaliseAgentRequest.PRED_IDENTITY:
            pm = PredictionModel(
                pred_type=PredictionModel.IDENTITY
            )
        else:
            rospy.logerr(
                "Prediction model {} unknown".format(prediction_model))
            # release resources
            self.internal_lock.release()
            return LocaliseAgentResponse(False)


        # Initialize publishers and messages
        cn_pub = rospy.Publisher("{}/estimated_node".format(name), String, queue_size=10, latch=True)
        pd_pub = rospy.Publisher("{}/current_prob_dist".format(name), DistributionStamped, queue_size=10, latch=True)
        self.res_publishers.append((cn_pub, pd_pub))
        cnviz_pub = rospy.Publisher("{}/estimated_node_viz".format(name), Marker, queue_size=10)
        parviz_pub = rospy.Publisher("{}/particles_viz".format(name), MarkerArray, queue_size=10)
        staparviz_pub = rospy.Publisher("{}/stateless_particles_viz".format(name), MarkerArray, queue_size=10)
        self.viz_publishers.append((cnviz_pub, parviz_pub, staparviz_pub))
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
        staptcsarrmsg = MarkerArray()
        for i in range(n_particles):
            staptcmkrmsg = Marker()
            staptcmkrmsg.header.frame_id = "/map"
            staptcmkrmsg.type = staptcmkrmsg.SPHERE
            staptcmkrmsg.pose.position.z = 0
            staptcmkrmsg.pose.orientation.w = 1
            staptcmkrmsg.scale.x = 0.1
            staptcmkrmsg.scale.y = 0.1
            staptcmkrmsg.scale.z = 0.1
            staptcmkrmsg.color.a = 0.6
            staptcmkrmsg.color.r = 1
            staptcmkrmsg.color.g = 1
            staptcmkrmsg.color.b = 0
            staptcmkrmsg.id = i
            staptcsarrmsg.markers.append(staptcmkrmsg)

        # get the how to spread the particles initially
        # if request.initial_spread_policy == LocaliseAgentRequest.CLOSEST_NODE:
        #     sigma = -1
        # elif request.initial_spread_policy == LocaliseAgentRequest.SPREAD_RADIUS:
        #     sigma = request.initial_spread_radius
        # elif request.initial_spread_policy == LocaliseAgentRequest.SPREAD_UNIFORM:
        #     sigma = np.inf
        # else:
        #     sigma = -1

        # Initialize a new instance of particle_filter
        pf = TopologicalParticleFilter(
            num=n_particles,
            prediction_model=pm,
            initial_spread_policy=initial_spread_policy,
            prediction_speed_decay=prediction_speed_decay,
            node_coords=self.node_coords,
            node_distances=self.node_distances,
            connected_nodes=self.connected_nodes,
            node_diffs2D=self.node_diffs2D,
            node_names=self.node_names
        )

        def __prepare_pd_msg(particles, timestamp=None):
            pdmsg = DistributionStamped()
            nodes, counts = np.unique(particles, return_counts=True)

            probs = np.zeros((self.node_names.shape[0]))
            probs[nodes] = counts.astype(float) / np.sum(counts)

            if timestamp is None:
                timestamp = rospy.get_rostime()
            pdmsg.header.stamp = timestamp
            pdmsg.nodes = self.node_names.tolist()
            pdmsg.values = np.copy(probs).tolist()

            return pdmsg

        def __prepare_cn_msg(node):
            strmsg = String()
            strmsg.data = self.node_names[node]
            return strmsg

        # function to publish current node and particles distribution
        def __publish(node, particles):
            if not stop_event.is_set():
                cn_pub.publish(__prepare_cn_msg(node))
                pd_pub.publish(__prepare_pd_msg(particles))

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

        # function to publish stateless particles distribution viz
        def __publish_stateless_viz(particles):
            if not stop_event.is_set():
                # publish viz stuff
                for i, p in enumerate(particles):
                    staptcsarrmsg.markers[i].header.stamp = rospy.get_rostime()
                    staptcsarrmsg.markers[i].pose.position.x = self.node_coords[p][0] + \
                        staptcsarrmsg.markers[i].scale.x * np.random.randn(1, 1)
                    staptcsarrmsg.markers[i].pose.position.y = self.node_coords[p][1] + \
                        staptcsarrmsg.markers[i].scale.y * np.random.randn(1, 1)

                staparviz_pub.publish(staptcsarrmsg)

        ## topic callbacks ##
        # send pose observation to particle filter
        def __pose_obs_cb(msg):
            if np.isfinite(msg.pose.pose.pose.position.x) and \
                    np.isfinite(msg.pose.pose.pose.position.y) and \
                    np.isfinite(msg.pose.pose.covariance[0]) and \
                    np.isfinite(msg.pose.pose.covariance[7]):
                node, particles = pf.receive_pose_obs(
                    msg.pose.pose.pose.position.x,
                    msg.pose.pose.pose.position.y,
                    msg.pose.pose.covariance[0], # variance of x
                    msg.pose.pose.covariance[7], # variance of y
                    # (rospy.get_rostime().to_sec(), msg.pose.header.stamp.to_sec())[
                        # msg.pose.header.stamp.to_sec() > 0]
                    rospy.get_rostime().to_sec(),
                    identifying=msg.identifying
                )
                __publish(node, particles)
            else:
                rospy.logwarn(
                    "Received non-admissible pose observation <{}, {}, {}, {}>, discarded".format(msg.pose.pose.pose.position.x, msg.pose.pose.pose.position.y, msg.pose.pose.covariance[0], msg.pose.pose.covariance[7]))

        # send likelihood observation to particle filter
        def __likelihood_obs_cb(msg):
            if len(msg.likelihood.nodes) == len(msg.likelihood.values):
                try:
                    nodes = [np.where(self.node_names == nname)[0][0] for nname in msg.likelihood.nodes]
                except IndexError:
                    rospy.logwarn(
                        "Received non-admissible node name {}, likelihood discarded".format(msg.likelihood.nodes))
                else:
                    values = np.array(msg.likelihood.values)
                    if np.isfinite(values).all() and (values >= 0.).all() and np.sum(values) > 0:
                        node, particles = pf.receive_likelihood_obs(
                            nodes, 
                            msg.likelihood.values,
                            # (rospy.get_rostime().to_sec(), msg.likelihood.header.stamp.to_sec())[
                                # msg.likelihood.header.stamp.to_sec() > 0]
                            rospy.get_rostime().to_sec(),
                            identifying=msg.identifying
                        )
                        __publish(node, particles)
                    else:
                        rospy.logwarn(
                            "Received non-admissible likelihood observation {}, discarded".format(msg.likelihood.values))
            else:
                rospy.logwarn("Nodes array and values array sizes do not match {} != {}, discarding likelihood observation".format(
                    len(msg.likelihood.nodes), len(msg.likelihood.values)))

        # subscribe to topics receiving observation
        self.obs_subscribers.append((
            rospy.Subscriber("{}/pose_obs".format(name), PoseObservation, __pose_obs_cb),
            rospy.Subscriber("{}/likelihood_obs".format(name),
                             LikelihoodObservation, __likelihood_obs_cb)
        ))

        ## Services handlers ##
        # Get the pose observation and returns the localisation result
        def __update_pose_handler(request):
            if np.isfinite(request.pose.pose.pose.position.x) and \
                    np.isfinite(request.pose.pose.pose.position.y) and \
                    np.isfinite(request.pose.pose.covariance[0]) and \
                    np.isfinite(request.pose.pose.covariance[7]):
                node, particles = pf.receive_pose_obs(
                    request.pose.pose.pose.position.x,
                    request.pose.pose.pose.position.y,
                    request.pose.pose.covariance[0],  # variance of x
                    request.pose.pose.covariance[7],  # variance of y
                    # (rospy.get_rostime().to_sec(), request.pose.header.stamp.to_sec())[
                    #     request.pose.header.stamp.to_sec() > 0]
                    rospy.get_rostime().to_sec(),
                    identifying=request.identifying
                )
                __publish(node, particles)
                resp = UpdatePoseObservationResponse()
                resp.success = True
                resp.estimated_node = __prepare_cn_msg(node).data
                resp.current_prob_dist = __prepare_pd_msg(particles)
                return(resp)
            else:
                rospy.logwarn(
                    "Received non-admissible pose observation <{}, {}, {}, {}>, discarded".format(request.pose.pose.pose.position.x, request.pose.pose.pose.position.y, request.pose.pose.covariance[0], request.pose.pose.covariance[7]))
          
            # fallback negative response
            resp = UpdatePoseObservationResponse()
            resp.success = False
            return(resp)

        # get a likelihood observation and return localisation result
        def __update_likelihood_handler(request):
            if len(request.likelihood.nodes) == len(request.likelihood.values):
                try:
                    nodes = [np.where(self.node_names == nname)[0][0]
                             for nname in request.likelihood.nodes]
                except IndexError:
                    rospy.logwarn(
                        "Received non-admissible node name {}, likelihood discarded".format(request.likelihood.nodes))
                else:
                    values = np.array(request.likelihood.values)
                    # rospy.loginfo("Received likelihood: {}".format(zip(nodes, values)))
                    if np.isfinite(values).all() and (values >= 0.).all() and np.sum(values) > 0:
                        node, particles = pf.receive_likelihood_obs(
                            nodes,
                            request.likelihood.values,
                            # (rospy.get_rostime().to_sec(), request.likelihood.header.stamp.to_sec())[
                                # request.likelihood.header.stamp.to_sec() > 0]
                            rospy.get_rostime().to_sec(),
                            identifying=request.identifying
                        )
                        __publish(node, particles)
                        resp = UpdateLikelihoodObservationResponse()
                        resp.success = True
                        resp.estimated_node = __prepare_cn_msg(node).data
                        resp.current_prob_dist = __prepare_pd_msg(particles)
                        return(resp)
                    else:
                        rospy.logwarn(
                            "Received non-admissible likelihood observation {}, discarded".format(request.likelihood.values))

            else:
                rospy.logwarn("Nodes array and values array sizes do not match {} != {}, discarding likelihood observation".format(
                    len(request.likelihood.nodes), len(request.likelihood.values)))

            # fallback negative response
            resp = UpdateLikelihoodObservationResponse()
            resp.success = False
            return(resp)

        def __do_stateless_prediction(request):
            # get a copy of the particle filter to work with
            _pf = pf.copy()
            # if requested pred rate is lesseq than 0 use the global one
            _prediction_rate = (request.prediction_rate, prediction_rate)[
                request.prediction_rate <= 0.]
            pred_step_secs = 1. / _prediction_rate
            secs_left = max(0.0, request.secs_from_now)
            time = rospy.get_rostime()
            resp = PredictResponse()
            resp.success = True
            # sub-function to append predictions to the result message
            def ___append_prediction(node, particles, secs_passed, timestamp):
                if not (node is None or particles is None):
                    __publish_stateless_viz(particles)
                    resp.secs_from_now.append(secs_passed)
                    resp.estimated_node.append(__prepare_cn_msg(node).data)
                    resp.prob_dist.append(__prepare_pd_msg(particles, timestamp=timestamp))
                    return True
                else:
                    resp.success = False
                    rospy.logwarn(
                        "Cannot perform prediction, no observation received so far.")
                    return False
            # perform all the steps until reached limit time
            while secs_left > 0:
                node, particles = _pf.predict(
                    timestamp_secs=time.to_sec()
                )
                if request.return_history:
                    succ = ___append_prediction(
                            node, 
                            particles,
                            secs_passed=request.secs_from_now - secs_left,
                            timestamp=time)
                    if not succ:
                        break
                    
                time += rospy.Duration.from_sec(pred_step_secs)
                secs_left -= pred_step_secs

            # add last prediction at secs_left == 0
            node, particles = _pf.predict(
                timestamp_secs=time.to_sec()
            )
            _ = ___append_prediction(node, 
                        particles,
                        secs_passed=request.secs_from_now - secs_left,
                        timestamp=time)

            return resp

        def __do_stateless_update(request):
            # create a new PF and assign the prior distribution to start up with
            __pf = TopologicalParticleFilter(
                num=n_particles,
                prediction_model=pm,
                initial_spread_policy=initial_spread_policy,
                prediction_speed_decay=prediction_speed_decay,
                node_coords=self.node_coords,
                node_distances=self.node_distances,
                connected_nodes=self.connected_nodes,
                node_diffs2D=self.node_diffs2D,
                node_names=self.node_names
            )
            if len(request.likelihood.nodes) == len(request.likelihood.values) and \
                    len(request.prior.nodes) == len(request.prior.values):
                try:
                    lkl_nodes = [np.where(self.node_names == nname)[0][0]
                             for nname in request.likelihood.nodes]
                    pr_nodes = [np.where(self.node_names == nname)[0][0]
                             for nname in request.prior.nodes]
                except IndexError:
                    rospy.logwarn(
                        "Received non-admissible node name {}/{}, prior/likelihood discarded".format(request.prior.nodes, request.likelihood.nodes))
                else:
                    pr_values = np.array(request.prior.values)
                    # rospy.loginfo(
                    #     "Received prior: {}".format(zip(pr_nodes, pr_values)))
                    lkl_values = np.array(request.likelihood.values)
                    # rospy.loginfo(
                    #     "Received likelihood: {}".format(zip(lkl_nodes, lkl_values)))
                    if np.isfinite(lkl_values).all() and (lkl_values >= 0.).all() and np.sum(lkl_values) > 0 and \
                            np.isfinite(pr_values).all() and (pr_values >= 0.).all() and np.sum(pr_values) > 0:
                        ts = rospy.get_rostime().to_sec()
                        # send the prior (assignment bcs it's the first observation ever received)
                        _, _ = __pf.receive_likelihood_obs(
                            pr_nodes,
                            request.prior.values,
                            ts,
                            False
                        )
                        # send the lkl (prediction is not performed here because the ts is the same as prior ts)
                        node, particles = __pf.receive_likelihood_obs(
                            lkl_nodes,
                            request.likelihood.values,
                            ts,
                            False
                        )
                        resp = UpdatePriorLikelihoodObservationResponse()
                        resp.success = True
                        resp.estimated_node = __prepare_cn_msg(node).data
                        resp.current_prob_dist = __prepare_pd_msg(particles)
                        return(resp)
                    else:
                        rospy.logwarn(
                            "Received non-admissible prior/likelihood observation {}, discarded".format(request.prior.values, request.likelihood.values))

            else:
                rospy.logwarn("Nodes array and values array sizes do not match {} != {}/{} != {}, discarding prior/likelihood observation".format(
                    len(request.prior.nodes), len(request.prior.values), len(request.likelihood.nodes), len(request.likelihood.values)))

        # subscribe to services for update
        self.upd_services.append({
            rospy.Service("{}/update_pose_obs".format(name), UpdatePoseObservation, __update_pose_handler),
            rospy.Service("{}/update_likelihood_obs".format(name), UpdateLikelihoodObservation, __update_likelihood_handler),
            rospy.Service("{}/predict_stateless".format(name), Predict, __do_stateless_prediction),
            rospy.Service("{}/update_stateless".format(name), UpdatePriorLikelihoodObservation, __do_stateless_update)
        })
        
        thr = None
        if do_prediction:
            # threaded function that performs predictions at a constant rate
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

        return LocaliseAgentResponse(True)

    def _stop_localise_handler(self, request):
        rospy.loginfo("Unregistering agent {} for localisation".format(request.name))
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

            # unregister topic subs
            for sub in self.obs_subscribers[agent_idx]:
                sub.unregister()
            # shutting down services
            for srv in self.upd_services[agent_idx]:
                srv.shutdown()
            # unregister topic pubs
            for pub in self.res_publishers[agent_idx]:
                pub.unregister()
            for pub in self.viz_publishers[agent_idx]:
                pub.unregister()
            

            # cleanup all the related variables
            del self.stopping_events[agent_idx]
            del self.prediction_threads[agent_idx]
            del self.obs_subscribers[agent_idx]
            del self.res_publishers[agent_idx]
            del self.viz_publishers[agent_idx]
            del self.upd_services[agent_idx]
            del self.agents[agent_idx]

            self.internal_lock.release()
            rospy.loginfo("DONE")
            return StopLocaliseResponse(True)
        else:
            rospy.logwarn("The agent {} is already not being localised.".format(name))
            self.internal_lock.release()
            return StopLocaliseResponse(False)

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
    rospy.init_node("bayesian_topological_localisation")

    localisation_node = TopologicalLocalisation()

    rospy.spin()

    localisation_node.close()
