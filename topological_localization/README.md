Topological Localization
======================

This package provides support for localizing agents in a topological map, based on `topological_navigation`. 

It uses particle filter for performing state estimation. It uses continuous-time Hidden Markov Models as the prediction model.

The code exploits the fact that generally a topological map nodes are sparsely connected - particles can only jump from a node to another if they are connected through an edge - to perform the computation efficiently.  

# How to use

## Launching topological localization

This node assumes that `topological_navigation` is running, in particular check that 
1. the topological map is loaded in the DB 
2. the `topological_map_manager` is running.

Launch localization node with:

```
roslaunch topological_localization topological_localization.launch
```

## Registering a new agent to localize

The node provide the service `/topological_localization/localize_agent` To register a new agent.

The srv is:

```
# constants for selecting the policy on how to spread the particles when the localization starts 
uint8 CLOSEST_NODE=0       # all particles to closest node 
uint8 SPREAD_UNIFORM=1     # particles spread uniformly around all ndoes
uint8 FOLLOW_OBS=2         # use the distribution of the first observation

# constants for the prediction model to use
uint8 PRED_CTMC=0          # continuous-time hidden markov model
uint8 PRED_IDENTITY=1      # each particle remains in the same node it was previously

string name                         # identifier of the agent to track
uint64 n_particles                  # number of particle to use 
uint8 initial_spread_policy         # policy value, one of the above constants
uint8 prediction_model              # prediction model, one of above constants 
bool do_prediction                  # if to perform prediction steps when observations are not available
float64 prediction_rate             # rate at which predictions are performed
float64 prediction_speed_decay      # the last estimated speed from observations will be multiplied by this factor at every estimation step until a new observation is received and the speed is estimated again. If 1.0, speed remains constant.

---

bool success                        # true if the registration of new agent localization was successful
```

## Stop localizing agent

Call the service `/topological_localization/stop_localize`, with srv:
```
string name         # identifier for the agent

---

bool success        # true if the localization is stopped 
```

## Sending observations

the localization node needs observations in order to localize and agent - e.g. `robot_1` - on the map. There are two ways an observation can be provided:

### pose with covariance 
To be published to topic `/robot_1/pose_obs`. The message has to provide a pose (x, y) and a variance for the pose as a covariance matrix: 
```
[var_x,   0  ,  0, 0, 0, 0,
   0  , var_y,  0, 0, 0, 0,
   0  ,   0  ,  0, 0, 0, 0, 
   0  ,   0  ,  0, 0, 0, 0, 
   0  ,   0  ,  0, 0, 0, 0, 
   0  ,   0  ,  0, 0, 0, 0]
```

### probability distribution
To be published to topic `/robot_1/prob_dist_obs`. The message has to contain a list of nodes names with a list of probabilties, one for each node. The message does not need to contain all the nodes in the map, the one that have a probability non-zero are sufficient.

## Getting localization result

The node position for an agent - e.g. `robot_1` - that is being localized is published latch to topic `/robot_1/current_node`. The package also provides the probability distribution of the agent location published latch to topic `/robot_1/current_prob_dist` as a list of nodes and with their corresponding probability.

## Visualizations

In order to visualize the localization result - for agent `robot_1` for example - on rviz add:
- a `Marker` attached to topic `/robot_1/current_node_viz`
- a `MarkerArray` attached to topic `/robot_1/particles_viz` 

## TODO
- [x] implement use of `prediction_speed_decay`, now always constant speed
- [ ] implement use of `initial_spread_policy`, now just uses the first observation
- [ ] change the $\lambda$ paramenter is computed in the ctmm prediction model, particles jumps too much if distance between nodes is unequal
- [x] correctly stop the threads on shutdown request
- [x] default particles number if not provided
- [ ] allow to use different state estimation methods than just particle filters. A simple one is needed for localizing robots from the metric localization, i.e. replace this https://github.com/LCAS/topological_navigation/blob/master/topological_navigation/scripts/localisation.py .
