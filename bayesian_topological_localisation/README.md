Bayesian Topological Localisation
======================

This package provides support for localising agents in a topological map, based on `topological_navigation`. 

It uses particle filter for performing state estimation. This code is based on https://eprints.lincoln.ac.uk/45627/1/topoNBS.pdf. 

The code exploits the fact that generally a topological map nodes are sparsely connected - particles can only jump from a node to another if they are connected through an edge - to perform the computation efficiently.  

# How to use

## Launching topological localisation

This node assumes that `topological_navigation` is running, in particular check that 
1. the topological map is loaded in the DB 
2. the `topological_map_manager` is running.

Launch localisation node with:

```
roslaunch bayesian_topological_localisation topological_localisation.launch
```

## Registering a new agent to localise

The node provide the service `/bayesian_topological_localisation/localise_agent` To register a new agent.

The srv is:

```
string name                         # identifier of the agent to track
uint64 n_particles                  # number of particle to use 
bool do_prediction                  # if to perform prediction steps when observations are not available
float64 prediction_rate             # rate at which predictions are performed

---

bool success                        # true if the registration of new agent localisation was successful
```

## Stop localising agent

Call the service `/bayesian_topological_localisation/stop_localise`, with srv:
```
string name         # identifier for the agent

---

bool success        # true if the localisation is stopped 
```

## Sending observations

The localisation node needs observations in order to localise and agent - e.g. `robot_1` - on the map.  
There are two ways an observation can be provided listed below. Each type of observation can be provided through a topic or by calling a service. If calling the service you will get the localisation result straight away in your service response. Each time you send an observation you have to set the parameter `identifying` (`False` by default) which indicates whether the observation uniquely identifies the target (e.g. GPS identifier) or not (e.g. LIDAR). 

### 1. Pose with covariance 
- **topic modality**: To be published to topic `/robot_1/pose_obs` (msg type `geometry_msgs/PoseWithCovarianceStamped`). The message has to provide a pose (x, y) and a variance for the pose as a covariance matrix: 
   ```
   [var_x,   0  ,  0, 0, 0, 0,
      0  , var_y,  0, 0, 0, 0,
      0  ,   0  ,  0, 0, 0, 0, 
      0  ,   0  ,  0, 0, 0, 0, 
      0  ,   0  ,  0, 0, 0, 0, 
      0  ,   0  ,  0, 0, 0, 0]
   ```
- **service modality**: Call `/robot_1/update_pose_obs` (srv type `bayesian_topological_localisation/UpdatePoseObservation`), the request parameter `pose` is a `geometry_msgs/PoseWithCovarianceStamped` and has to be filled as for the topic modality.

### 2. Likelihood
- **topic modality**: To be published to topic `/robot_1/likelihood_obs` (msg type `bayesian_topological_localisation/DistributionStamped`). The message has to contain a list of nodes names with a list of values for the likelihood, one for each node. The message does not need to contain all the nodes in the map, the one that have a likelihood non-zero are sufficient.
  
- **service modality**: Call `/robot_1/update_likelihood_obs` (srv type `bayesian_topological_localisation/UpdateLikelihoodObservation`), the request parameter `likelihood` is a `bayesian_topological_localisation/DistributionStamped` and has to be filled as for the topic modality.
  
## Getting localisation result

The node position for an agent - e.g. `robot_1` - that is being localised is published latch to topic `/robot_1/estimated_node`. The package also provides the probability distribution of the agent location published latch to topic `/robot_1/current_prob_dist` as a list of nodes and with their corresponding probability.

## Visualizations

In order to visualize the localisation result - for agent `robot_1` for example - on rviz add:
- a `Marker` attached to topic `/robot_1/estimated_node_viz`
- a `MarkerArray` attached to topic `/robot_1/particles_viz` 

## Stateless utilities
The node provides some services to perform inference into the future, the services work on a copy of the particle filter they therefore do not affect to current state of the localisation.

### Prediction the future distribution
Call `/robot_1/predict_stateless` (srv type `bayesian_topological_localisation/Predict`) specifying
   - `secs_from_now`: how many seconds into the future you want to predict 
   - `prediction_rate`: the rate of prediction, if `secs_from_now = 10` and `prediction_rate = 1` the node will perform 10 prediction steps to give the final prediction
   - `return_history`: (default `False`) if `True` returns the prediction result at all prediction steps, rather than just the final prediction at `secs_from_now`.

### Perform an update step from given prior and likelihood
Call `/robot_1/update_stateless` (srv type `bayesian_topological_localisation/UpdatePriorLikelihoodObservation`) specifying
   - `prior`: The prior distribution of nodes 
   - `likelihood`: the likelihood ditribution.
Returns the posterior distribution computed from prior and likelihood and the estimated node.

## Monitoring Particles Distribution
The PF implements two mechanisms for preventing the particles distribution to diverge.

1. **monitoring entropy**: at each step, the entropy of the distribution over the topological map is computed, whenever this falls below a threshold the particles do not jump to node that are not connected anymore. The threshould (default value: 0.6) can be set with the service `/bayesian_topological_localisation/set_entropy_lower_bound`.
2. **monitoring observation distance**: every time the PF receives a new observation we compute the [Jensen-Shannon Distance](https://scipy.github.io/devdocs/generated/scipy.spatial.distance.jensenshannon.html) between the current distribution and the normalised observation. If this is higher than a threshold and the observation is *identifying* (see "Sending observations" section above) then 1) we re-initialise the particles with the distribution of the observation; and 2) we restart jumping to non connected nodes (if the entropy monitor was triggered before). The threshould (default value: 0.975) can be set with the service `/bayesian_topological_localisation/set_JSD_upper_bound`.
