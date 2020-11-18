import numpy as np

class PredictionModel:

    CTMC = 0 # continuous-time markov chain
    IDENTITY = 1 # remain in the same node

    def __init__(self, pred_type=0, node_coords=[], node_diffs2D=[], node_distances=[], connected_nodes=[]):
        self.pred_type = pred_type
        self.node_coords = np.array(node_coords)
        self.node_diffs2D = np.array(node_diffs2D)
        self.node_distances = np.array(node_distances)
        self.connected_nodes = np.array(connected_nodes)

    # always predict the node it's already in
    def _identity(self, node):
        return [1.], [node]

    # transition probability governed by the continuous markov model
    def _ctmc(self, node, speed, time, only_connected=False):
        prob = []
        nodes = []

        # check if speed and next node have same direction
        same_direction_mask = np.any(speed * self.node_diffs2D[node, self.connected_nodes[node]] >= 0, axis=1)
        pos_connected_nodes = self.connected_nodes[node][same_direction_mask]
        neg_connected_nodes = self.connected_nodes[node][~same_direction_mask]

        # trans probability goes mostly to connected nodes and one len(self.connected_nodes)th goes to the unconnected neighboors
        div_coef = len(pos_connected_nodes)
        # if (not only_connected) and len(unconnected_neighboors_nodes):
        #     div_coef += 1

        #     # probability of nodes that are not connected but close enough
        #     # project speed vector on the edges toward successive nodes
        #     diffs_norm = np.dot(self.node_diffs2D[unconnected_neighboors_nodes],
        #                         self.node_diffs2D[unconnected_neighboors_nodes].transpose()).diagonal()
        #     speed_proj = (np.dot(self.node_diffs2D[unconnected_neighboors_nodes], speed) / diffs_norm).reshape(
        #         (-1, 1)) * self.node_diffs2D[unconnected_neighboors_nodes]
        #     speed_proj = np.sqrt(
        #         np.dot(speed_proj, speed_proj.transpose()).diagonal())
        #     if lambda_p <= 0:
        #         _lambda_p = - np.log(0.5) * speed_proj / \
        #             self.node_distances[unconnected_neighboors_nodes]
        #     else:
        #         _lambda_p = np.ones(speed_proj.shape) * lambda_p
        #     p_move = 1. - np.exp(- time * _lambda_p)

        #     prob += list(((p_move / div_coef) /
        #                     len(unconnected_neighboors_nodes)).tolist())
        #     nodes += list(unconnected_neighboors_nodes)

        # project speed vector on the edges toward successive nodes
        diffs_norm = np.dot(
            self.node_diffs2D[node, pos_connected_nodes], self.node_diffs2D[node, pos_connected_nodes].transpose()).diagonal()
        speed_proj = (np.dot(self.node_diffs2D[node, pos_connected_nodes], speed) /
                        diffs_norm).reshape((-1, 1)) * self.node_diffs2D[node, pos_connected_nodes]
        speed_proj = np.sqrt(
            np.dot(speed_proj, speed_proj.transpose()).diagonal())

        # probability of moving to trans_node
        # compute lambda considering speed and distance between nodes
        # lambda = (0.6931 * speed) / dist, so that p(transitioning) = 0.5 when the position is halfway between current and next node
        # because exp(-lambda * tau) = 0.5 => ln(0.5) = -0.6931 = - lambda * tau, where tau is time in the node
        # if lambda_p <= 0:
        _lambda_p = - np.log(0.5) * speed_proj / \
            self.node_distances[node][pos_connected_nodes]
        # else:
        #     _lambda_p = np.ones(speed_proj.shape) * lambda_p
        time = max(min(time, 60. * 5), 0.) # to avoid overflow error 

        p_move = 1. - np.exp(- time * _lambda_p)

        prob += list((p_move / div_coef).tolist())
        nodes += list(pos_connected_nodes)

        prob += [0.] * len(neg_connected_nodes)
        nodes += list(neg_connected_nodes)

        # set probability of remaining
        prob.append(1.0 - sum(prob))
        nodes.append(node)

        # return probability and list of nodes it can jump to
        return prob, nodes

    # returns the prediction in form of a probability distribution over nodes given time and speed
    def predict(self, node, time=None, speed=None):
        if self.pred_type == PredictionModel.CTMC:
            return self._ctmc(node=node, speed=speed, time=time)
        else:
            return self._identity(node)
