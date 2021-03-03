import numpy as np
import threading
import rospy
import math
from bayesian_topological_localisation.particle import Particle

class TopologicalParticleFilter():
    FOLLOW_OBS = 0      # use the distribution of the first observation
    SPREAD_UNIFORM = 1  # equally distributed along all nodes
    CLOSEST_NODE = 2    # assigned to closest node

    # if the entropy of the current distribution is smaller than this threshold,
    # stop jumping to close nodes that are unconnected
    DEFAULT_UNCONNECTED_JUMP_THRESHOLD = 0.8
    # if the Jensen-Shannon Distance btw prior and likelihood is greater than this threshold, 
    # reinitialize particles with the likelihood AND restart jumping to close unconnected nodes
    DEFAULT_REINIT_JSD_THRESHOLD = 0.90


    def __init__(self, num, prediction_model, initial_spread_policy, prediction_speed_decay, node_coords, node_distances, connected_nodes, node_diffs2D, node_names,
                 reinit_jsd_threshold=None, unconnected_jump_threshold=None):
        self.n_of_ptcl = num
        self.prediction_model = prediction_model
        self.initial_spread_policy = initial_spread_policy
        # speed decay when doing only prediction (it does eventually stop)
        self.prediction_speed_decay = prediction_speed_decay
        self.node_coords = node_coords
        self.node_distances = node_distances
        self.connected_nodes = connected_nodes
        self.node_diffs2D = node_diffs2D
        self.node_names = node_names

        # current particles
        self.particles = [None] * self.n_of_ptcl  # np.empty((self.n_of_ptcl))
        # particles after prediction phase
        self.predicted_particles = [None] * self.n_of_ptcl #np.empty((self.n_of_ptcl))
        # particles weight
        self.W = np.ones((self.n_of_ptcl))

        # time of last update
        self.time = [None] * self.n_of_ptcl
        # life time in current node
        self.life = np.zeros((self.n_of_ptcl))
        # last estimated node
        self.last_estimate = None
        # current estimate of speed
        self.current_speed = np.zeros((2))
        # num samples to use to estimate picker speed
        self.n_speed_samples = 10
        self.speed_samples = [] #[np.array([0.]*2)] * self.n_speed_samples
        # history of poses received
        self.last_pose = np.zeros((2))
        # timestamp of poses
        self.last_ts = np.zeros((1))
        # if to jump to only connected nodes 
        self.only_connected = False        

        self.print_debug = True

        self.reinit_jsd_threshold = reinit_jsd_threshold if reinit_jsd_threshold is not None else self.DEFAULT_REINIT_JSD_THRESHOLD
        self.unconnected_jump_threshold = unconnected_jump_threshold if unconnected_jump_threshold is not None else self.DEFAULT_UNCONNECTED_JUMP_THRESHOLD

        self.lock = threading.Lock()

    def _expand_distribution(self, prob, nodes):
        if type(nodes) == np.ndarray:
            _nodes = nodes.tolist()
        else:
            _nodes = nodes
        new_prob = np.zeros(self.node_coords.shape[0])
        for i in range(new_prob.shape[0]):
            try:
                _idx = _nodes.index(i)
            except:
                pass
            else:
                new_prob[i] = prob[_idx]

        return new_prob

    def _normalize(self, arr):
        arr = np.array(arr)
        row_sums = np.sum(arr)  # for 1d array
        if row_sums == 0:
            arr = np.ones(arr.shape)
            row_sums = np.sum(arr)
            if self.print_debug:
                rospy.logwarn("Array to normalise is zero, resorting to uniform")
        arr = arr.astype(float) / row_sums
        return arr

    def _normal_pdf(self, mu_x, mu_y, cov_x, cov_y, nodes):
        mean = np.array([mu_x, mu_y])                 # center of gaussian
        cov_x = np.max([cov_x, 0.2])
        cov_y = np.max([cov_y, 0.2])
        cov_M = np.matrix([[cov_x, 0.], [0., cov_y]])    # cov matrix
        det_M = cov_x * cov_y                           # det cov matrix
        diffs2D = np.matrix(self.node_coords[nodes] - np.array([mu_x, mu_y]))
        up = np.exp(- 0.5 * (diffs2D * cov_M.I * diffs2D.T).diagonal())
        probs = np.array(up / np.sqrt((2*np.pi)**2 * det_M))
        return self._normalize(probs.reshape((-1)))

    def _update_speed(self, obs_x=None, obs_y=None, timestamp_secs=None):
        if not (obs_x is None or obs_y is None):
            # compute speed w.r.t. last pose obs
            distance = np.array([obs_x, obs_y]) - self.last_pose
            time = timestamp_secs - self.last_ts

            if len(self.speed_samples) == self.n_speed_samples:
                self.speed_samples.pop(0)
            self.speed_samples.append(distance / time)

            self.current_speed = np.average(self.speed_samples, axis=0)

            self.last_pose = np.array([obs_x, obs_y])
            self.last_ts = timestamp_secs
        else:
            self.current_speed *= self.prediction_speed_decay

    def _initialize_wt_pose(self, obs_x, obs_y, cov_x, cov_y, timestamp_secs):
        nodes_prob = self._normal_pdf(obs_x, obs_y, cov_x, cov_y, range(self.node_coords.shape[0]))
        nodes_prob = self._normalize(nodes_prob)
        _particles_nodes = np.random.choice(range(self.node_coords.shape[0]), self.n_of_ptcl, p=nodes_prob)
        # sample velocity (x,y components) as gaussian sample with mean 0.0 and a covariance
        _particles_vels = np.random.normal(0.0, 0.05, (self.n_of_ptcl, 2))
        # sample time (seconds) as exponential sample 
        _particles_lifes = np.random.exponential(scale=1.0, size=self.n_of_ptcl)

        self.particles = [
            Particle(node, vel, life, timestamp_secs)
            for node, vel, life in zip(_particles_nodes, _particles_vels, _particles_lifes)
        ]
        
        for idx in range(len(self.particles)):
            self.predicted_particles[idx] = self.particles[idx].__copy__()
        self.time = np.ones((self.n_of_ptcl)) * timestamp_secs
        # self.life = np.zeros((self.n_of_ptcl))
        # self.last_pose = np.array([obs_x, obs_y])
        self.last_ts = timestamp_secs
        self.W = np.ones((self.n_of_ptcl))

    def _initialize_wt_likelihood(self, nodes, likelihood, timestamp_secs):
        probs = self._normalize(np.array(likelihood))
        _particles_nodes = np.random.choice(nodes, self.n_of_ptcl, p=probs)
        # sample velocity (x,y components) as gaussian sample with mean 0.0 and a covariance
        _particles_vels = np.random.normal(0.0, 0.05, (self.n_of_ptcl, 2))
        # sample time (seconds) as exponential sample
        _particles_lifes = np.random.uniform(high=1, size=self.n_of_ptcl)

        self.particles = [
            Particle(node, vel, life, timestamp_secs)
            for node, vel, life in zip(_particles_nodes, _particles_vels, _particles_lifes)
        ]

        for idx in range(len(self.particles)):
            self.predicted_particles[idx] = self.particles[idx].__copy__()
        self.time = np.ones((self.n_of_ptcl)) * timestamp_secs
        # self.life = np.zeros((self.n_of_ptcl))
        self.W = np.ones((self.n_of_ptcl))
        
    def _initialize_uniform(self, timestamp_secs):
        prob = 1.0 / self.n_of_ptcl
        self._initialize_wt_likelihood(
            np.arange(self.node_coords.shape[0]), 
            np.ones((self.node_coords.shape[0])) * prob,
            timestamp_secs
        )

    # if p is a prob dist
    def _compute_entropy(self, p):
        prob_p = p + 1e-5 # to avoid getting -inf for log(0)
        entropy = np.sum(prob_p * np.log2(prob_p))
        entropy = -entropy

        return entropy

    #  p and q are prob dist, returns the KL divergence
    def _compute_kl(self, p, q):
        prob_p = p + 1e-5 # to avoid getting -inf for log(0)
        prob_q = q + 1e-5 # to avoid getting -inf for log(0)

        kld = np.sum(prob_p * np.log2(prob_p / prob_q))

        return kld

    # distance between two distributions, symmetric
    # this is a value between 0 and 1
    def _compute_jensen_shannon_distance(self, p, q):
        m = (p + q) / 2

        divergence = (self._compute_kl(p, m) + self._compute_kl(q, m)) / 2

        distance = np.sqrt(divergence)

        return distance

    def _predict(self, timestamp_secs):

        for particle_idx in range(self.n_of_ptcl):

            p = self.particles[particle_idx]

            _new_p = self.prediction_model.predict(
                particle = p,
                timestamp_secs = timestamp_secs
            )

            self.predicted_particles[particle_idx] = _new_p

    # weighting with normal distribution with var around the observation
    def _weight_pose(self, obs_x, obs_y, cov_x, cov_y, timestamp_secs, identifying):
        _nodes = np.array([p.node for p in self.predicted_particles])
        _vels = np.array([p.vel for p in self.predicted_particles])
        idx_sort = np.argsort(_nodes)
        nodes, indices_start, counts = np.unique(
            _nodes[idx_sort], return_index=True, return_counts=True)
        indices_groups = np.split(idx_sort, indices_start[1:])
        nodes = nodes.tolist()

        #### DEBUG
        # p_entropy = self._compute_entropy(self._normalize(counts))
        # rospy.loginfo("Entropy of prior: {}".format(p_entropy))
        ####

        # weight pose
        _all_nodes = np.arange(self.node_coords.shape[0])
        prob_dist = self._normal_pdf(obs_x, obs_y, cov_x, cov_y, _all_nodes)

        self.W = np.zeros((self.n_of_ptcl))
        for _, (node, indices) in enumerate(zip(nodes, indices_groups)):
            self.W[indices] = prob_dist[node]

        # weight speed
        if len(self.speed_samples) == self.n_speed_samples:
            def __gaussian(x, mu, sig):
                return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))
            
            norm_current_sped = np.linalg.norm(self.current_speed)
            unit_current_speed = self.current_speed / \
                norm_current_sped
            for p_i, _vel in enumerate(_vels):
                # weight angle
                _norm_vel = np.linalg.norm(_vel)
                _unit_vel = _vel / _norm_vel
                dot_product = np.dot(unit_current_speed, _unit_vel)
                angle = np.arccos(dot_product)
                self.W[p_i] += (np.cos(angle) + 1) / 8.
                # weght norm
                _n_w = __gaussian(_norm_vel, norm_current_sped, norm_current_sped * 0.5)
                self.W[p_i] += _n_w / 4.
            # print("After")

        # assign velocity of gps
        # if len(self.speed_samples) == self.n_speed_samples:
        #     for i in range(len(self.predicted_particles)):
        #         self.predicted_particles[i].vel = self.current_speed

        # print("current_speed {}".format(self.current_speed))
        # print("avg ptcl speed {}, median {}, max {}, min {}".format(
        #     np.average(_vels, axis=0), np.median(_vels, axis=0), np.max(_vels, axis=0), np.min(_vels, axis=0)))


        # compute distributions distance
        js_distance = self._compute_jensen_shannon_distance(
            self._expand_distribution(self._normalize(counts), nodes), prob_dist)
        #### DEBUG 
        # o_entropy = self._compute_entropy(prob_dist)
        # rospy.loginfo("Entropy of pose observation: {}".format(o_entropy))

        # rospy.loginfo("Jensen-Shannon distance: {}".format(js_distance))
        ####

        # it measn the particles are "disjoint" from this obs
        if identifying and js_distance > self.reinit_jsd_threshold:
            if self.print_debug:
                rospy.logwarn("Reinitializing particles, JS distance between prior and likelihood {} is greater than {}".format(
                    js_distance, self.reinit_jsd_threshold))
            self._initialize_wt_pose(obs_x, obs_y, cov_x, cov_y, timestamp_secs)
            self.only_connected = False # we are not really sure now anymore

    # weighting wih a given likelihood distribution
    def _weight_likelihood(self, nodes_dist, likelihood, timestamp_secs, identifying):
        _nodes = np.array([p.node for p in self.predicted_particles])
        idx_sort = np.argsort(_nodes)
        nodes, indices_start, counts = np.unique(
            _nodes[idx_sort], return_index=True, return_counts=True)
        indices_groups = np.split(idx_sort, indices_start[1:])
        nodes = nodes.tolist()

        # p_entropy = self._compute_entropy(self._normalize(counts))
        # rospy.loginfo("Entropy of prior: {}".format(p_entropy))

        self.W = np.zeros((self.n_of_ptcl))
        for _, (node, indices) in enumerate(zip(nodes, indices_groups)):
            if node in nodes_dist:
                self.W[indices] = likelihood[nodes_dist.index(node)]

        ##### DEBUG observation compute entropy
        # o_entropy = self._compute_entropy(self._normalize(likelihood))
        # rospy.loginfo("Entropy of likel observation: {}".format(o_entropy))
        ####

        # compute distributions distance
        js_distance = self._compute_jensen_shannon_distance(
            self._expand_distribution(self._normalize(counts), nodes), self._expand_distribution(self._normalize(likelihood), nodes_dist))
        # if self.print_debug and identifying: rospy.loginfo("Jensen-Shannon distance: {}".format(js_distance))

        # it measn the particles are "disjoint" from this obs
        if identifying and js_distance > self.reinit_jsd_threshold:
            if self.print_debug:
                rospy.logwarn("Reinitializing particles, JS distance between prior and likelihood {} is greater than {}".format(
                    js_distance, self.reinit_jsd_threshold))
            self._initialize_wt_likelihood(nodes_dist, likelihood, timestamp_secs)
            self.only_connected = False # we are not really sure now anymore

    # produce the node estimate based on topological mass from particles and their weight
    def _estimate_node(self, use_weight=True):
        _nodes = [p.node for p in self.predicted_particles]
        nodes, indices_start, counts = np.unique(
            _nodes, return_index=True, return_counts=True)
        masses = []
        if use_weight:
            for (_, index_start, count) in zip(nodes, indices_start, counts):
                masses.append(self.W[index_start] * count)
        else:
            masses = counts
        # if self.print_debug:
        #     print("Masses: {}".format(zip(self.node_names[nodes], masses, counts)))
        self.last_estimate = self.predicted_particles[indices_start[np.argmax(
            masses)]]
        # if self.print_debug:
        #     print("Node estimate: {} {}".format(self.node_names[self.last_estimate.node], self.last_estimate))

    def _add_noise(self, particle):
        # noise to the node, bernoulli
        if np.random.random() < 0.001:
            if self.only_connected:
                closeby_nodes = self.connected_nodes[particle.node]
            else:
                closeby_nodes = np.where((self.node_distances[particle.node]<=3))[0]
            particle.node = np.random.choice(closeby_nodes)
        
        particle.vel += np.random.normal(0.0, 0.0005)
        particle.life = max(0, particle.life + np.random.uniform(low=-0.1, high=0.1))

    def _resample(self, use_weight=True):
        if use_weight:
            prob = self._normalize(self.W)
            particles_idxs = np.random.choice(
                np.arange(len(self.particles)), self.n_of_ptcl, p=prob)
        else:
            particles_idxs = np.arange(len(self.particles))
        
        for pi, idx in enumerate(particles_idxs):
            self.particles[pi] = self.predicted_particles[idx].__copy__()

        # add noise to the state of the new particles
        for p in self.particles:
            # if self.print_debug: print("clean", str(p))
            self._add_noise(p)
            # if self.print_debug: print("noisy", str(p))

        # compute entropy of the new distribution
        nodes, indices_start, counts = np.unique(
            [p.node for p in self.particles], return_index=True, return_counts=True)
        p_entropy = self._compute_entropy(self._normalize(counts))
        # if self.print_debug: 
        #     rospy.loginfo("Final entropy : {} ({})".format(p_entropy, self.only_connected))

        if not self.only_connected and p_entropy < self.unconnected_jump_threshold:
            self.only_connected = True
            if self.print_debug:
                rospy.logwarn("Stop jumping to unconnected nodes, entropy of current particles distribution {} smaller than {}.".format(p_entropy, self.unconnected_jump_threshold))

    def set_JSD_upper_bound(self, bound):
        self.reinit_jsd_threshold = bound

    def set_entropy_lower_bound(self, bound):
        self.unconnected_jump_threshold = bound

    def predict(self, timestamp_secs):
        """Performs a prediction step, estimates the new node and resamples the particles based on the prediction model only."""
        self.lock.acquire()

        if self.last_estimate is None: # never received an observation
            particles = None
            p_estimate = None
        else:
            self._update_speed()

            self._predict(timestamp_secs)

            self._estimate_node(use_weight=False)

            self._resample(use_weight=False)

            particles = [None] * len(self.particles)
            for idx in range(len(self.particles)):
                particles[idx] = self.particles[idx].__copy__()
            p_estimate = self.last_estimate.__copy__()

        self.lock.release()

        return p_estimate, particles

    def receive_pose_obs(self, obsx, obsy, covx, covy, timestamp_secs, identifying):
        """Performs a full bayesian optimization step of the particles by integrating the new pose observation"""
        self.lock.acquire()

        if self.last_estimate is None:  # never received an observation before
            # if the observation can be a false positive do not initialize the PF with it
            if identifying:
                self._initialize_wt_pose(obsx, obsy, covx, covy, timestamp_secs)
            else:
                self._initialize_uniform(timestamp_secs)
            
            use_weight = False
        else:
            if identifying:
                self._update_speed(obsx, obsy, timestamp_secs)
            else:
                self._update_speed()

            self._predict(timestamp_secs)

            self._weight_pose(obsx, obsy, covx, covy,
                              timestamp_secs, identifying)
            
            use_weight = True

        self._estimate_node(use_weight=use_weight)

        self._resample(use_weight=use_weight)

        particles = [None] * len(self.particles)
        for idx in range(len(self.particles)):
            particles[idx] = self.particles[idx].__copy__()
        p_estimate = self.last_estimate.__copy__()

        self.lock.release()

        return p_estimate, particles

    def receive_likelihood_obs(self, nodes, likelihood, timestamp_secs, identifying):
        """Performs a full bayesian optimization step of the particles by integrating the new likelihood distribution observation"""
        self.lock.acquire()
        
        if self.last_estimate is None:  # never received an observation before
            # if the observation can be a false positive do not initialize the PF with it
            if identifying:
                self._initialize_wt_likelihood(
                    nodes, likelihood, timestamp_secs)
            else:
                self._initialize_uniform(timestamp_secs)
            
            use_weight = False
        else:
            self._update_speed()

            self._predict(timestamp_secs)

            self._weight_likelihood(
                nodes, likelihood, timestamp_secs, identifying)

            use_weight = True

        self._estimate_node(use_weight=use_weight)

        self._resample(use_weight=use_weight)

        particles = [None] * len(self.particles)
        for idx in range(len(self.particles)):
            particles[idx] = self.particles[idx].__copy__()
        p_estimate = self.last_estimate.__copy__()

        self.lock.release()

        return p_estimate, particles

    
    def copy(self):
        """Factory function that produces a copy of the current object"""
        # create a new PF object
        copy_obj = TopologicalParticleFilter(num=self.n_of_ptcl,
                                             prediction_model=self.prediction_model,
                                             initial_spread_policy=self.initial_spread_policy,
                                             prediction_speed_decay=self.prediction_speed_decay,
                                             node_coords=self.node_coords,
                                             node_distances=self.node_distances,
                                             connected_nodes=self.connected_nodes,
                                             node_diffs2D=self.node_diffs2D,
                                             node_names=self.node_names)

        # get all the class variables
        variables = [attr for attr in dir(copy_obj) if not callable(
            getattr(copy_obj, attr)) and not attr.startswith("__")]
        
        # copy all the variable values, excluding some
        exclude_variables = ["lock", "particles", "predicted_particles"]
        for var in variables:
            if var in exclude_variables:
                continue

            if isinstance(getattr(self, var), np.ndarray):
                setattr(copy_obj, var, np.copy(getattr(self, var)))
            else:
                setattr(copy_obj, var, getattr(self, var))

        # we dont want to print stuff related to ac opy object
        copy_obj.print_debug = False

        # copy the particles
        for idx in range(len(self.particles)):
            copy_obj.predicted_particles[idx] = self.predicted_particles[idx].__copy__()
            copy_obj.particles[idx] = self.particles[idx].__copy__()

        return copy_obj
