^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bayesian_topological_localisation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
