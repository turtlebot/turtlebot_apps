^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.3 (2015-03-23)
------------------
* use env instead arg for map closes `#134 <https://github.com/turtlebot/turtlebot_apps/issues/134>`_
* Contributors: roycho111

2.3.2 (2015-01-21)
------------------

2.3.1 (2014-12-30)
------------------

2.3.0 (2014-12-30)
------------------
* base_frame and odom_frame are configurable
* Added comment about track_unknown_space also to global_planner_param.yaml
* Update move_base_params.yaml
  Removed base_global_planner parameter which was defined twice
* - track_unknown_space parameter added to obstacle layer in costmap_common_param.yaml
  - set allow_unknown global path planning to false and added note about needed track_unkown_space param
* - Added default values for navfn comments
  - Added global planner param file
  - Added comment for global planner in move base param file
  - Removed base_local_planner.yaml
* Added navfn param file to move_base.launch.xml
* Added param file for navfn global planner
* restructure of custom param
* fixed typo in move_base.launch.xml
* added comments about dummy file
* Load custom param file for move_base
* set laser topic from launch file
* set gmapping  minimum score to 200
* fixed few parameters
* bugfix and param files clean up
* bugfix move_base.launch.xml
* updated turtlebot navigation params to use layer plugins
* frame_id parameters can now be entered by user from launch file
* Set frame_id parameters as arguments in amcl launch file
* turtlebot navigation environment variables (map).
* minimumScore in gmapping
* typo bugfix for dwa-local-planner -> dwa_local_planner run_depends.
* Run depend fix for dwa_local_planner
* constrain translational acceleration to match the smoother's profile, `#93 <https://github.com/turtlebot/turtlebot_apps/issues/93>`_.
* improved obstacle avoidance, better turning arcs.
* don't set translational min vel to zero else negligible rotations will be accepted.
* update dwa local planner with experimentally verified kobuki limits and accelerations.
* local costmap and obstacle layer should be referencing odom, not map, fixes `#90 <https://github.com/turtlebot/turtlebot_apps/issues/90>`_.
* Merge branch 'indigo' of https://github.com/turtlebot/turtlebot_apps into indigo
* there is actually one (not zero) vy_sample when planning, fixes `#89 <https://github.com/turtlebot/turtlebot_apps/issues/89>`_.
* 0.0 minimum velocity so it generates local paths that spin in place.
* switch to the dwa planner.
* dwa planner configuration for the turtlebot.
* Removed dependencies that move_base now brings in, closes `#56 <https://github.com/turtlebot/turtlebot_apps/issues/56>`_.
* Contributors: AlexReimann, Alexander Reimann, Daniel Stonier, Jihoon Lee, Mehdi Tlili, kentsommer

2.2.4 (2013-10-14)
------------------

2.2.3 (2013-09-27)
------------------

2.2.2 (2013-09-26)
------------------

2.2.1 (2013-09-23)
------------------

2.2.0 (2013-08-30)
------------------
* On gmapping, provide a realistic range for the kinect and reduce the map_update_interval (now use default value).
* Add bugtracker and repo info URLs.
* Rename include launchers to *.launch.xml.
* Changelogs at package level.
* URL for the pull request that will make the navi modules workaround redundant.
* Dependencies reviewed

2.1.x - hydro, unstable
=======================

2.1.1 (2013-08-09)
------------------
* Add few namespace-related changes
* Add safety controller to navigation demos
* Rationalize the use of velocity smoother: remap properly robot_cmd_vel, add comments, and avoid meaningless topic names"
* Adapt turtlebot_navigation configuration to hydro navi stack

2.1.0 (2013-07-19)
------------------
* Catkinized


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/turtlebot_apps/ChangeList
