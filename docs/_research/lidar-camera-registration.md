---
layout: page
title: "Real-Time LiDAR–Panoramic Camera Semantic Registration on Spot"
permalink: /research/lidar-camera-registration
---

# Real-Time LiDAR–Panoramic Camera Semantic Registration on a Legged Robot

**CERLAB Roboteam, CMU · 2025**

Hardware-side counterpart of my [semantic exploration work](/research/semantic-exploration):
give a real Boston Dynamics Spot dense, semantically-colored 3D perception by fusing an
Ouster LiDAR with an Insta360 X4 panoramic camera.

## Highlights

- Real-time projection pipeline registering full-rate LiDAR sweeps into the panoramic
  image: **10 FPS at full LiDAR rate, <100 ms latency, <10 px reprojection error**.
- Spatial consistency maintained under the high-frequency vibration of a walking
  quadruped — the failure mode that kills most naive LiDAR–camera rigs on legged
  platforms.
- Designed and 3D-printed a rigid sensor mount so the extrinsic calibration actually
  survives deployment; calibrated the fisheye/panoramic model (lesson learned: never use
  a checkerboard with QR codes on it — they corrupt corner detection).
- Output: dense semantically-colored point clouds consumed by downstream mapping and
  viewpoint planning.

*Stack: ROS, C++, Ouster SDK, OpenCV fisheye calibration, Spot SDK.*
