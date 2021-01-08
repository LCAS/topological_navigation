import numpy as np
import threading
import rospy

class TopologicalParticleFilter():
    FOLLOW_OBS = 0      # use the distribution of the first observation
    SPREAD_UNIFORM = 1  # equally distributed along all nodes
    CLOSEST_NODE = 2    # assigned to closest node

    REINIT_THRESHOLD = 1e-6     # if prob of current dist with observation is smaller than this constant, reinitialize particles

    def __init__(self, num, prediction_model, initial_spread_policy, prediction_speed_decay, node_coords, node_distances, connected_nodes, node_diffs2D, node_names):
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
        self.particles = np.empty((self.n_of_ptcl))
        # previous timestep particles
        self.prev_particles = np.empty((self.n_of_ptcl))
        # particles after prediction phase
        self.predicted_particles = np.empty((self.n_of_ptcl))
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
        self.n_speed_samples = 20
        self.speed_samples = [np.array([0.]*2)] * self.n_speed_samples
        # history of poses received
        self.last_pose = np.zeros((2))
        # timestamp of poses
        self.last_ts = np.zeros((1))
        

        self.lock = threading.Lock()

    def _normalize(self, arr):
        row_sums = arr.sum()  # for 1d array
        if row_sums == 0:
            arr = np.ones(arr.shape)
            row_sums = arr.sum()
            rospy.logwarn("Array to normalise is zero, resorting to uniform")
        arr = arr / row_sums
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
        return probs.reshape((-1))

    def _update_speed(self, obs_x=None, obs_y=None, timestamp_secs=None):
        if not (obs_x is None or obs_y is None):
            # compute speed w.r.t. last pose obs
            distance = np.array([obs_x, obs_y]) - self.last_pose
            time = timestamp_secs - self.last_ts

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
        self.particles = np.random.choice(range(self.node_coords.shape[0]), self.n_of_ptcl, p=nodes_prob)
        
        self.prev_particles = np.copy(self.particles)
        self.predicted_particles = np.copy(self.particles)
        self.time = np.ones((self.n_of_ptcl)) * timestamp_secs
        self.life = np.zeros((self.n_of_ptcl))
        self.last_pose = np.array([obs_x, obs_y])
        self.last_ts = timestamp_secs

    def _initialize_wt_likelihood(self, nodes, likelihood, timestamp_secs):
        probs = self._normalize(np.array(likelihood))
        self.particles = np.random.choice(nodes, self.n_of_ptcl, p=probs)

        self.prev_particles = np.copy(self.particles)
        self.predicted_particles = np.copy(self.particles)
        self.time = np.ones((self.n_of_ptcl)) * timestamp_secs
        self.life = np.zeros((self.n_of_ptcl))

    def _predict(self, timestamp_secs, only_connected=False):
        # update life time only where the agent has actually been there
        traversed_particles_mask = (self.particles == self.last_estimate) | (self.life > 0.)
        self.life[traversed_particles_mask] += timestamp_secs - \
            self.time[traversed_particles_mask]

        self.time[:] = timestamp_secs

        for particle_idx in range(self.n_of_ptcl):

            p_node = self.particles[particle_idx]

            transition_p, t_nodes = self.prediction_model.predict(
                node=p_node,
                speed=self.current_speed,
                time=self.life[particle_idx],
                only_connected=only_connected
            )
            # print(transition_p)
            self.predicted_particles[particle_idx] = np.random.choice(
                t_nodes, p=transition_p)

    # weighting with normal distribution with var around the observation
    def _weight_pose(self, obs_x, obs_y, cov_x, cov_y):
        idx_sort = np.argsort(self.predicted_particles)
        nodes, indices_start, _ = np.unique(
            self.predicted_particles[idx_sort], return_index=True, return_counts=True)
        indices_groups = np.split(idx_sort, indices_start[1:])
        nodes = nodes.tolist()

        prob_dist = self._normal_pdf(obs_x, obs_y, cov_x, cov_y, nodes)

        self.W = np.zeros((self.n_of_ptcl))
        for _, (node, indices) in enumerate(zip(nodes, indices_groups)):
            self.W[indices] = prob_dist[nodes.index(node)]

        # it measn the particles are disjoint from this obs
        if np.sum(prob_dist) < TopologicalParticleFilter.REINIT_THRESHOLD:
            rospy.logwarn("Reinitializing particles, observation probability is less than {}".format(TopologicalParticleFilter.REINIT_THRESHOLD))
            self._initialize_wt_pose(obs_x, obs_y, cov_x, cov_y, timestamp_secs=rospy.get_rostime().to_sec())

    # weighting wih a given likelihood distribution
    def _weight_likelihood(self, nodes_dist, likelihood):
        idx_sort = np.argsort(self.predicted_particles)
        nodes, indices_start, _ = np.unique(
            self.predicted_particles[idx_sort], return_index=True, return_counts=True)
        indices_groups = np.split(idx_sort, indices_start[1:])

        self.W = np.zeros((self.n_of_ptcl))
        for _, (node, indices) in enumerate(zip(nodes, indices_groups)):
            if node in nodes_dist:
                self.W[indices] = likelihood[nodes_dist.index(node)]

        # TODO if W doesn't sum up to 1 then re-initialize the particles with this obs
        # it measn the particles are disjoint from this obs

    # produce the node estimate based on topological mass from particles and their weight
    def _estimate_node(self, use_weight=True):
        nodes, indices_start, counts = np.unique(
            self.predicted_particles, return_index=True, return_counts=True)
        masses = []
        if use_weight:
            for (_, index_start, count) in zip(nodes, indices_start, counts):
                masses.append(self.W[index_start] * count)
        else:
            masses = counts
        # print("Masses: {}".format(zip(self.node_names[nodes], masses, counts)))
        self.last_estimate = nodes[np.argmax(masses)]
        # print("Last estimate: {}".format(self.node_names[self.last_estimate]))

    def _resample(self, use_weight=True):
        # pass
        self.prev_particles = np.copy(self.particles)
        if use_weight:
            prob = self._normalize(self.W)
            self.particles = np.random.choice(
                self.predicted_particles, self.n_of_ptcl, p=prob)
        else:
            self.particles = np.copy(self.predicted_particles)

        ## reset life time if particle jumped in another node
        for particle_idx in range(self.n_of_ptcl):
            if self.particles[particle_idx] != self.prev_particles[particle_idx]:
                self.life[particle_idx] = 0

    def predict(self, timestamp_secs):
        """Performs a prediction step, estimates the new node and resamples the particles based on the prediction model only."""
        self.lock.acquire()

        if self.last_estimate is None: # never received an observation
            particles = None
            node = None
        else:
            self._update_speed()

            self._predict(timestamp_secs, only_connected=True)

            self._estimate_node(use_weight=False)

            self._resample(use_weight=False)

            particles = np.copy(self.particles)
            node = self.last_estimate

        self.lock.release()

        return node, particles

    def receive_pose_obs(self, obsx, obsy, covx, covy, timestamp_secs):
        """Performs a full bayesian optimization step of the particles by integrating the new pose observation"""
        self.lock.acquire()

        if self.last_estimate is None:  # never received an observation before
            self._initialize_wt_pose(obsx, obsy, covx, covy, timestamp_secs)
            
            use_weight = False
        else:
            self._update_speed(obsx, obsy, timestamp_secs)

            self._predict(timestamp_secs)

            self._weight_pose(obsx, obsy, covx, covy)
            
            use_weight = True

        self._estimate_node(use_weight=use_weight)

        self._resample(use_weight=use_weight)

        particles = np.copy(self.particles)
        node = self.last_estimate

        self.lock.release()

        return node, particles

    def receive_likelihood_obs(self, nodes, likelihood, timestamp_secs):
        """Performs a full bayesian optimization step of the particles by integrating the new likelihood distribution observation"""
        self.lock.acquire()
        
        if self.last_estimate is None:  # never received an observation before
            self._initialize_wt_likelihood(nodes, likelihood, timestamp_secs)
            
            use_weight = False
        else:
            self._update_speed()

            self._predict(timestamp_secs)

            self._weight_likelihood(nodes, likelihood)

            use_weight = True

        self._estimate_node(use_weight=use_weight)

        self._resample(use_weight=use_weight)

        particles = np.copy(self.particles)
        node = self.last_estimate

        self.lock.release()

        return node, particles

    
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
        exclude_variables = ["lock"]
        for var in variables:
            if var in exclude_variables:
                continue

            if isinstance(getattr(self, var), np.ndarray):
                setattr(copy_obj, var, np.copy(getattr(self, var)))
            else:
                setattr(copy_obj, var, getattr(self, var))

        return copy_obj
