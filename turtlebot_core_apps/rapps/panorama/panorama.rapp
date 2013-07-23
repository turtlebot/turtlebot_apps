display:     Panorama
description: Turtlebot makes a 360 degree panorama image 
platform:    turtlebot
launch:      turtlebot_core_apps/panorama.launch
interface:   turtlebot_core_apps/panorama.interface 
icon:        turtlebot_core_apps/panorama_bubble_icon.png
clients:
  - type: android
    manager:
      api-level: 9
      intent-action: com.ros.turtlebot.apps.panorama.PanoramaActivity
    app:
      pano_img_topic: /turtlebot_panorama/panorama/compressed
