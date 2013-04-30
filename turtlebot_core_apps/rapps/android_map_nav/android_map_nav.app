display: Map Nav
description: Drive a turtlebot around a pre-made map from an Android device.
platform: turtlebot
launch: turtlebot_core_apps/android_map_nav.launch
interface: turtlebot_core_apps/android_teleop.interface
icon: turtlebot_core_apps/map_nav_bubble_icon.png
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: org.ros.android.map_nav.MainActivity
   app: 
     gravityMode: 0
     base_control_topic: /cmd_vel
