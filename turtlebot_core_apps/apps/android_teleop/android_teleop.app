display: Android Teleop
description: Drive a turtlebot from Android with a touch joystick and video feed.
platform: turtlebot
launch: turtlebot_core_apps/android_teleop.launch
interface: turtlebot_core_apps/android_teleop.interface
icon: turtlebot_core_apps/android_teleop_bubble_icon.png
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.teleop.Teleop
   app: 
     gravityMode: 0
     base_control_topic: /cmd_vel
