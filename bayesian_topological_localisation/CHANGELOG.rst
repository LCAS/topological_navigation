^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bayesian_topological_localisation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2022-01-25)
------------------
* Merge pull request `#117 <https://github.com/magnucha/topological_navigation/issues/117>`_ from LCAS/noetic
  Basic navigation works in ros noetic but not all scripts are converted to python 3
* python 3 compatible for most parts of toponav but not all!
* Merge pull request `#111 <https://github.com/magnucha/topological_navigation/issues/111>`_ from adambinch/remove_strands_dependencies
  Removing strands navigation dependencies from topological navigation.
* revert
* strands dependencies removed from `bayesian_topological_localisation`
* Merge branch 'master' of github.com:LCAS/topological_navigation into node_names
* Merge pull request `#78 <https://github.com/magnucha/topological_navigation/issues/78>`_ from francescodelduchetto/toponav2-restrictions
  Toponav2 restrictions
* adding files lost somehow in the commits
* adding lost files after the merge; fix few changes
* Merge branch 'master' of github.com:LCAS/topological_navigation into francescodelduchetto-toponav2-restrictions
* Merge branch 'master' of github.com:LCAS/topological_navigation into toponav2_launch
* Merge branch 'master' of github.com:LCAS/topological_navigation into faster_route_search2
* removing bayesian_toponav which shouldn't be here
* Merge branch 'toponav2-devel-restrictions' of github.com:francescodelduchetto/topological_navigation into toponav2-devel
* Merge branch 'master' of https://github.com/adambinch/topological_navigation into adam-master
* Contributors: Adam Binch, MikHut, adambinch, francescodelduchetto

2.3.0 (2021-07-15)
------------------
* Merge branch 'master' of github.com:LCAS/topological_navigation into tmap_to_tmap2
* Merge pull request `#82 <https://github.com/LCAS/topological_navigation/issues/82>`_ from adambinch/fix_conflicts
  Fix conflicts
* Merge branch 'master' of github.com:LCAS/topological_navigation into fix_conflicts
  # Conflicts:
  #	topological_navigation/scripts/execute_policy_server.py
  #	topological_navigation/scripts/navigation.py
* Merge pull request `#62 <https://github.com/LCAS/topological_navigation/issues/62>`_ from francescodelduchetto/master
  New features in bayesian_topological_localisation node
* remove 2 factor from probability of jumping; remove weighting for the speed components of particles because it pratically works best without; clip factor to avoid division with zero
* Delete README.md.orig
* Merge branch 'master' of github.com:francescodelduchetto/topological_navigation
* removing some prints; removing un-used arguments
* Update README.md
* Merge pull request `#3 <https://github.com/LCAS/topological_navigation/issues/3>`_ from francescodelduchetto/particles-states
  Particles states
* prediction model split in two steps: predict if to jump and predict where to jump
* improvements on estimation of velocity of particles
* reduce noise and initialization values for particles
* adding topic with entire state of the particles
* fixing some parameters
* particle has speed in the state and it's estimated from the particle's movement itself
* services for setting thresholds
* Fixing error in update_stateless for prior distribution not initialising the particles
* change JSD threshold
* adding measure of entropy, Jensen-Shannon distance for deciding when to start/stop jumping to close nodes and when reinitialize the particles
* fix
* fix
* Merge pull request `#2 <https://github.com/LCAS/topological_navigation/issues/2>`_ from francescodelduchetto/future_pred
  Future pred
* Adding argument to discriminate between observations that identifies the target and those that don't, the identifying ones allow will re0initialise the ptcl distribution when the observation is disjoint from the prior
* Adding a service  for merging a prior dist with a likelihood dist without affecting the ongoing localization
* adding possibility of returning the entire history of stateless prediction; adding ad-hoc prediction rate argument for stateless prediction service
* Adding stateless prediction to predict future states of the localisation, without affecting the current estimate
* Merge pull request `#49 <https://github.com/LCAS/topological_navigation/issues/49>`_ from francescodelduchetto/master
  Fix shared resources handling on callbacks
* Merge pull request `#1 <https://github.com/LCAS/topological_navigation/issues/1>`_ from francescodelduchetto/detached_master
  fix use of stop threading event
* fix use of stop threading event
* add stop thread event also to publishing function to avoid publish to closed topic
* Merge branch 'master' of github.com:francescodelduchetto/topological_navigation
* changing order of topics unregisters and service shutdowns to avoid error in queued callbacks
* Contributors: Adam Binch, Jaime Pulido Fentanes, Your Name, adambinch, francescodelduchetto

* Merge branch 'master' of github.com:LCAS/topological_navigation into tmap_to_tmap2
* Merge pull request `#82 <https://github.com/LCAS/topological_navigation/issues/82>`_ from adambinch/fix_conflicts
* Merge branch 'master' of github.com:LCAS/topological_navigation into fix_conflicts
* Merge pull request `#62 <https://github.com/LCAS/topological_navigation/issues/62>`_ from francescodelduchetto/master
  New features in bayesian_topological_localisation node
* remove 2 factor from probability of jumping; remove weighting for the speed components of particles because it pratically works best without; clip factor to avoid division with zero
* Merge branch 'master' of github.com:francescodelduchetto/topological_navigation
* Merge pull request `#3 <https://github.com/LCAS/topological_navigation/issues/3>`_ from francescodelduchetto/particles-states
  Particles states
* prediction model split in two steps: predict if to jump and predict where to jump
* improvements on estimation of velocity of particles
* reduce noise and initialization values for particles
* adding topic with entire state of the particles
* fixing some parameters
* particle has speed in the state and it's estimated from the particle's movement itself
* services for setting thresholds
* Fixing error in update_stateless for prior distribution not initialising the particles
* change JSD threshold
* adding measure of entropy, Jensen-Shannon distance for deciding when to start/stop jumping to close nodes and when reinitialize the particles
* Merge pull request `#2 <https://github.com/LCAS/topological_navigation/issues/2>`_ from francescodelduchetto/future_pred
  Future pred
* Adding argument to discriminate between observations that identifies the target and those that don't, the identifying ones allow will re0initialise the ptcl distribution when the observation is disjoint from the prior
* Adding a service  for merging a prior dist with a likelihood dist without affecting the ongoing localization
* adding possibility of returning the entire history of stateless prediction; adding ad-hoc prediction rate argument for stateless prediction service
* Adding stateless prediction to predict future states of the localisation, without affecting the current estimate
* Merge pull request `#49 <https://github.com/LCAS/topological_navigation/issues/49>`_ from francescodelduchetto/master
  Fix shared resources handling on callbacks
* Merge pull request `#1 <https://github.com/LCAS/topological_navigation/issues/1>`_ from francescodelduchetto/detached_master
  fix use of stop threading event
* add stop thread event also to publishing function to avoid publish to closed topic
* Merge branch 'master' of github.com:francescodelduchetto/topological_navigation
* changing order of topics unregisters and service shutdowns to avoid error in queued callbacks
* Contributors: Adam Binch, Jaime Pulido Fentanes, adambinch, francescodelduchetto

2.2.0 (2020-11-25)
------------------
* Merge pull request `#48 <https://github.com/LCAS/topological_navigation/issues/48>`_ from Jailander/new-pkg-version
  Making package version number compatible with other packages for release
* Making package version number compatible with other packages for release
* Merge pull request `#43 <https://github.com/LCAS/topological_navigation/issues/43>`_ from francescodelduchetto/master
  Topological localization package
* Update README.md
* Re-initialize particles when the weighting from pose is too little wrt particles; allow small chance of jumping to unconnected nodes
* get current time when receiving message instead of message time to avoid problems when time received is not accurate enough
* current_node is now estimated_node, because it's more clear wrt what it contains
* implemented services for sending observations and getting localisation result back
* handle with warning when observation is completely disjoint from prediction
* renaming to bayesian_topological_localisation
* Contributors: Jaime Pulido Fentanes, francescodelduchetto, jailander

2.1.0 (2020-04-20)
------------------

2.0.0 (2020-04-08 23:43)
------------------------

1.1.1 (2020-04-08 22:56)
------------------------
