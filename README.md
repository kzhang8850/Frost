# STARK Industries - Frost
Frost, an integrated experiment in the fields of computer vision, sensor fusion, and artificial intelligence, is an autonomous robot that can launch snowballs at enemies. Just arm it with snowballs, and watch as it dominates snowball fights on its own, with an accuracy equivalent to or better than a human.

For more information about Frost, check out our website, linked above.

This full codebase cannot be run without Frost - Autonomous Snowball Launcher, which is not out for distribution. It requires a LIDAR, a Kinect camera, and an actuated catapult.

However, it is possible to run components. 

1. To run the body detection module:
      - Just have a webcam or similar on hand, modify the code slightly to interface with said webcam, and then run                   body_detection.py, which will find and track bodies in the camera's field of view.
2. To run the LIDAR module:
      - Buy or make a LIDAR, then hook it up properly with the correct wiring (depending on what LIDAR you have or how you             obtained it, this may require changing code). Then run processor.py, which will show a visualizer for your LIDAR               output. You may have to comment out all other parts of the code that do not pertain to the LIDAR.
