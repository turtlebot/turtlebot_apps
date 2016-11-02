^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot_follower
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.7 (2016-11-01)
------------------
* fixed bad printf format
* clean up cmake dependencies
* Included depth_image_proc in cmake deps
* Contributors: Mr-Yellow, Rohan Agrawal, Tully Foote

2.3.6 (2016-06-29)
------------------
* add dependency on depth_image_proc
* Contributors: Tully Foote

2.3.5 (2016-06-28)
------------------

2.3.4 (2016-06-28)
------------------
* Switch to use depth/image_rect instead of depth/points for efficiency.
  It takes ~ 1/4th the cpu power to do this version.
  This has the added benefit of not requiring PCL as a dependency.
* Add support for joystick override of follower
  This allows joystick button 5 to toggle follower on and off.
  It simply adds another input to the mux which sends zero
  velocity with a slightly higher priority than the follower.
* the sim directory no longer exists
  It was implicitly removed in `#142 <https://github.com/turtlebot/turtlebot_apps/issues/142>`_
* Added arguments in launch file
* Add simulation launch in turtlebot_follower
* Contributors: Jihoon Lee, Nadya Ampilogova, Tully Foote

2.3.3 (2015-03-23)
------------------

2.3.2 (2015-01-21)
------------------

2.3.1 (2014-12-30)
------------------

2.3.0 (2014-12-30)
------------------
* remove inappropriate / fixes `#127 <https://github.com/turtlebot/turtlebot_apps/issues/127>`_
* sim to simulation param
* follower in simulation
* Contributors: Jihoon Lee

2.2.4 (2013-10-14)
------------------

2.2.3 (2013-09-27)
------------------

2.2.2 (2013-09-26)
------------------
* Use service files to turtlebot_msgs.

2.2.1 (2013-09-23)
------------------

2.2.0 (2013-08-30)
------------------
* Rename include launchers to *.launch.xml.
* Changelogs at package level.

2.1.x - hydro, unstable
=======================

2.1.1 (2013-08-09)
------------------
* Fix namespace issues
* Rationalize the use of velocity smoother: remap properly robot_cmd_vel, add comments, and avoid meaningless topic names

2.1.0 (2013-07-19)
------------------
* Catkinized
* Unexistent include dir removed from package description


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/turtlebot_apps/ChangeList
