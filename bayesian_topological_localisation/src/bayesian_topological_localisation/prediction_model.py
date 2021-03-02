import numpy as np

class PredictionModel:

    CTMC = 0 # continuous-time markov chain
    IDENTITY = 1 # remain in the same node

    def __init__(self, pred_type=0, node_coords=[], node_diffs2D=[], node_distances=[], connected_nodes=[], unconnected_distance=5.0):
        self.pred_type = pred_type
        self.node_coords = np.array(node_coords)
        self.node_diffs2D = np.array(node_diffs2D)
        self.node_distances = np.array(node_distances)
        self.connected_nodes = np.array(connected_nodes)
        self.unconnected_distance = unconnected_distance

    # always predict the node it's already in
    def _identity(self, node):
        return [1.], [node]

    # transition probability governed by the continuous markov model
    def _ctmc(self, particle, timestamp_secs, only_connected=True):
        ### part one, update life time in node
        _new_life = particle.life + max(0.01, timestamp_secs - particle.last_time_secs)

        ## part two update node
        _prob = []
        _nodes = []

        # check if speed and next node have same direction
        same_direction_mask = np.any(particle.vel * self.node_diffs2D[particle.node, self.connected_nodes[particle.node]] >= 0, axis=1)
        pos_connected_nodes = self.connected_nodes[particle.node][same_direction_mask]
        neg_connected_nodes = self.connected_nodes[particle.node][~same_direction_mask]

        # project speed vector on the edges toward successive nodes
        diffs_norm = np.dot(
            self.node_diffs2D[particle.node, pos_connected_nodes], self.node_diffs2D[particle.node, pos_connected_nodes].transpose()).diagonal()
        speed_proj = (np.dot(self.node_diffs2D[particle.node, pos_connected_nodes], particle.vel) /
                        diffs_norm).reshape((-1, 1)) * self.node_diffs2D[particle.node, pos_connected_nodes]
        speed_proj = np.sqrt(
            np.dot(speed_proj, speed_proj.transpose()).diagonal())

        # probability of moving to trans_node
        # compute lambda considering speed and distance between nodes
        # lambda = (0.6931 * speed) / dist, so that p(transitioning) = 0.5 when the position is halfway between current and next particle.node
        # because exp(-lambda * tau) = 0.5 => ln(0.5) = -0.6931 = - lambda * tau, where tau is time in the particle.node
        lambda_p = 2.0 * np.log(0.5) * speed_proj / \
            self.node_distances[particle.node][pos_connected_nodes]

        w_p = speed_proj
        sum_w_p = np.sum(w_p)

        # this is te bernoully probabily of jumping/not-jumping
        bern_p_jump = np.sum(w_p * (1.0 - np.exp(_new_life * lambda_p)))/ sum_w_p

        # now first we decide if we jump:
        if np.random.random() < bern_p_jump:
            # we jump...

            nodes_p_jump = lambda_p / np.sum(lambda_p)

            _new_node_idx = np.random.choice(
                np.arange(len(pos_connected_nodes)), p=nodes_p_jump)
            _new_node = pos_connected_nodes[_new_node_idx]
            _jump_prob = nodes_p_jump[_new_node_idx]
        else:
            # we do not jump...

            _new_node = particle.node
            _jump_prob = 1.0 - bern_p_jump


        # # probability of connected nodes in the direction of speed
        # _prob += list((p_move / div_coef).tolist())
        # _nodes += list(pos_connected_nodes)

        # # probability of connected nodes in the opposite direction of speed
        # _prob += [0.] * len(neg_connected_nodes)
        # _nodes += list(neg_connected_nodes)

        # # probability of remaining in current particle.node
        # _prob.append(1.0 - sum(_prob))
        # _nodes.append(particle.node)


        # _new_node_idx = np.random.choice(np.arange(len(_nodes)), p=_prob)
        # _new_node = _nodes[_new_node_idx]

        ## part three: update vel
        _this_step_vel = _jump_prob * (self.node_coords[_new_node] - self.node_coords[particle.node]) / max(
            0.1, timestamp_secs - particle.last_time_secs)  # max(0.01, timestamp_secs - particle.last_time_secs)
        _new_vel = particle.vel + (_this_step_vel -  # np.clip(_this_step_vel, -1, 1) -
                                 particle.vel) / particle.n_steps_vel

        # print("From {} to {} with prob {} vel {}({}) time {}".format(particle.node, _new_node,
                                                                #  _prob[_new_node_idx], _this_step_vel, _new_vel, timestamp_secs - particle.last_time_secs))

        ## set all the new values
        _new_particle = particle.__copy__()
        if _new_node == particle.node:
            _new_particle.life = _new_life
        else:
            _new_particle.life = 0.0
            _new_particle.node = _new_node
        _new_particle.vel = _new_vel
        _new_particle.last_time_secs = timestamp_secs


        return _new_particle

    # returns the prediction in form of a probability distribution over nodes given time and speed
    def predict(self, particle, timestamp_secs, only_connected=False):
        if self.pred_type == PredictionModel.CTMC:
            return self._ctmc(particle=particle, timestamp_secs=timestamp_secs)
        else:
            return self._identity(node)
