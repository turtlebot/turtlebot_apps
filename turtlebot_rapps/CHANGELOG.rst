=========
Changelog
=========

2.3.1 (2014-12-30)
------------------

2.3.0 (2014-12-30)
------------------
* uncomment move_base `#129 <https://github.com/turtlebot/turtlebot_apps/issues/129>`_
* disable map_store `#126 <https://github.com/turtlebot/turtlebot_apps/issues/126>`_
* sim to simulation param
* fixing image topic for simulation
* sim turtlebot make a map works
* add video teleop with sim
* follower in simulation
* it uses rect_color
* remove map_store dependency
* updates
* updates
* update make a map rapp launch
* update interface
* typo fix
* update type info for interfaces
* ps3_teleop uses relative path
* relative path on all rapps
* rename teleop to video_teleop
* turtlebot teleop uses video_teleop virtual app. and use public interface
* remove deprecated attribute. pairing_clients from panorama
* Update/fix comments
* Update map_nav launch for use with environment hooks
* Add missing required node dependency
* Change map_serve to pull from environment variable
* Turtlebot_rapps/teleop to implement rocon_apps/teleop
* Typo fix
* Moved to generic 'virtual_joystick'
* Remove map_server for make_a_map - not needed
* Rolled android and qt make_a_map and map_nav into one
* Remove capabilities dependancy for follower, graveyard capabilities version.
* Give map_nav and make_a_map default map for Qt
* Small typo fix to re-add map-store
* Moved to seperate android/qt launchers and rapps to remove error as android requires mongodb
* Updated package.xml to include new and refactored rapps
* Refactored android_xxx to remove android
* turtlebot_core_apps -> turtlebot_rapps, `#86 <https://github.com/turtlebot/turtlebot_apps/issues/86>`_
* Contributors: Daniel Stonier, Jihoon Lee, Kent Sommer, dwlee, kentsommer

2.2.4 (2013-10-14)
------------------
* Use 27018 port (27017 is the default) in all apps avoid error 48
  conflicts.

2.2.3 (2013-09-27)
------------------

2.2.2 (2013-09-26)
------------------

2.2.1 (2013-09-23)
------------------
* Set of fixes to make it work with Android apps.

2.2.0 (2013-08-30)
------------------
* Remove redundant rapp information.
* Add auto shutdown functionality on auto_docking.
* Add bugtracker and repo info URLs.
* Rename include launchers to *.launch.xml.
* Changelogs at package level.
* Use a private warehouse port.

2.1.x - hydro, unstable
=======================

2.1.1 (2013-08-09)
------------------
* Fix namespaces for auto docking app
* Add various changes to enable android apps (w/ and w/o pairing mode)
* Add rapp files for map manager app
* Update interface files for ps3 & xbox360 rapps
* Update panorama rapp files
* Add linux.ros. to platform name
* Rename all .app extensions as .rapp
* Remove our beloved chirp app. Don't worry; it's now included on rocon_apps

2.1.0 (2013-07-19)
------------------
* Catkinized
* map_store is now catkinized
* Remove legacy code
* Prepare for new app manager


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/turtlebot_apps/ChangeList
