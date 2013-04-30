display: Make a Map
description: Make a map by driving a Turtlebot from an Android device.
platform: turtlebot
launch: turtlebot_core_apps/android_make_a_map.launch
interface: turtlebot_core_apps/android_teleop.interface
icon: turtlebot_core_apps/make_a_map_bubble_icon.png
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: org.ros.android.make_a_map.MainActivity
   app: 
     gravityMode: 0
     base_control_topic: /cmd_vel 
