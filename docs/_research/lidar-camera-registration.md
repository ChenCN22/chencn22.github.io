---
layout: page
title: "Real-Time LiDAR–Panoramic Camera Semantic Registration on Spot"
permalink: /research/lidar-camera-registration
---


**CERLAB Roboteam, CMU · 2025 – 2026**

<figure style="margin:1rem 0;"><img src="/assets/img/lidar/hardware-spot.jpg" alt="Spot carrying an Ouster OS-1 LiDAR and an Insta360 X4 camera in a machine shop" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">The sensor suite: Ouster OS-1 LiDAR and Insta360 X4 dual-fisheye camera on a Boston Dynamics Spot, in the machine shop used for the real-robot experiments.</figcaption></figure>

The sensor side of my [semantic exploration work](/research/semantic-exploration): fuse an
Ouster LiDAR with an Insta360 X4 panoramic camera so a walking Spot gets dense,
semantically-colored 3D perception.

- **10 FPS at full LiDAR rate · <100 ms latency · <10 px reprojection error**
- Stays aligned under gait vibration — a 3D-printed rigid mount keeps the extrinsics honest.
- Feeds the 3D semantic object map used by the exploration planner.

## Calibration: target-less, from a single scene

Render the accumulated LiDAR scan as an intensity panorama, match it to the fisheye frame
for an initial guess, then refine by maximizing mutual information over the overlap.

<div style="display:grid;grid-template-columns:1fr 1fr;gap:.8rem;margin:1rem 0;">
  <figure style="margin:0;"><img src="/assets/img/lidar/calib-fisheye.jpg" alt="Raw fisheye frame used for calibration" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">1 · Raw Insta360 fisheye frame (omnidirectional camera model).</figcaption></figure>
  <figure style="margin:0;"><img src="/assets/img/lidar/calib-lidar-intensity.jpg" alt="LiDAR intensity panorama" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">2 · The same scene as a LiDAR reflectivity panorama rendered from the accumulated scan.</figcaption></figure>
</div>
<figure style="margin:0 0 1rem;"><img src="/assets/img/lidar/calib-superglue.jpg" alt="SuperGlue matches between the fisheye frame and the LiDAR intensity image" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">3 · Learned feature matches between camera and LiDAR intensity images give the initial extrinsic guess (282 matches here).</figcaption></figure>
<figure style="margin:0 0 1rem;"><img src="/assets/img/lidar/calib-fov.jpg" alt="Overlap region between fisheye FoV and LiDAR scan band" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">4 · The overlap between the fisheye field of view (arcs) and the LiDAR elevation/azimuth band, cropped before the information-theoretic refinement.</figcaption></figure>


## What it produces

<figure style="margin:1rem 0;">
<video controls muted playsinline preload="metadata" poster="/assets/img/lidar/projection-live-poster.jpg" style="width:100%;border-radius:6px;">
  <source src="/assets/img/lidar/projection-live.mp4" type="video/mp4">
</video>
<figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Live registration in rviz: every LiDAR sweep is projected into the fisheye image (points colored by depth) while the accumulated cloud builds on the left — running at full LiDAR rate.</figcaption>
</figure>

<figure style="margin:1rem 0;"><img src="/assets/img/lidar/fusion-mask.jpg" alt="Instance segmentation mask on the dual-fisheye frame" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">The same projection in reverse: detection masks on the fisheye frame are lifted onto the LiDAR points that fall inside them, which is how objects enter the 3D semantic map.</figcaption></figure>

*Stack: ROS, C++, Ouster SDK, OpenCV fisheye calibration, direct visual–LiDAR calibration
(koide3), Spot SDK.*
