---
layout: page
title: "Real-Time LiDAR–Panoramic Camera Semantic Registration on Spot"
permalink: /research/lidar-camera-registration
---


**CERLAB Roboteam, CMU · 2025 – 2026**

<figure style="margin:1rem 0;"><img src="/assets/img/lidar/hardware-spot.jpg" alt="Spot carrying an Ouster OS-1 LiDAR and an Insta360 X4 camera in a machine shop" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">The sensor suite: Ouster OS-1 LiDAR and Insta360 X4 dual-fisheye camera on a Boston Dynamics Spot, in the machine shop used for the real-robot experiments.</figcaption></figure>

The sensor side of my [semantic exploration work](/research/semantic-exploration): fuse an
Ouster OS1 LiDAR with an Insta360 X4 panoramic camera so a walking Spot gets dense,
semantically-colored 3D perception.

- **10 FPS at full LiDAR rate · <100 ms latency · <10 px reprojection error**
- Stays aligned under gait vibration — a 3D-printed rigid mount, iterated until the registration held.
- Feeds the 3D semantic object map used by the exploration planner.

## Drivers

Everything on the robot is C++ / ROS 2: the Ouster OS1 driver and the fusion node. For the camera I
cut the official Insta360 X4 C++ SDK down to a ROS 2 node and trimmed its encode/decode path, which
is where most of the camera latency was. We started with the Spot CAM, whose driver was unstable;
after testing fixes against the Python API docs we switched the lab to the X4.

## Intrinsics

Both cameras were calibrated in OpenCV, with the camera model picked empirically by reprojection
error. The Spot CAM panorama is stitched from several views and Boston Dynamics does not publish
the projection, so it has no single intrinsic matrix. The X4 is treated as an omnidirectional
(equirectangular) camera, which is what makes the rest of the pipeline clean.

## Extrinsics: target-less, from a single scene

Render the accumulated LiDAR scan as an intensity panorama, match it to the fisheye frame for an
initial guess, then refine by maximizing mutual information over the overlap. The extrinsics and the
registration transform are optimized jointly. Built on a modified version of Kenji Koide's
[direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration).

<div style="display:grid;grid-template-columns:1fr 1fr;gap:.8rem;margin:1rem 0;">
  <figure style="margin:0;"><img src="/assets/img/lidar/calib-fisheye.jpg" alt="Raw fisheye frame used for calibration" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">1 · Raw Insta360 fisheye frame (omnidirectional camera model).</figcaption></figure>
  <figure style="margin:0;"><img src="/assets/img/lidar/calib-lidar-intensity.jpg" alt="LiDAR intensity panorama" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">2 · The same scene as a LiDAR reflectivity panorama rendered from the accumulated scan.</figcaption></figure>
</div>
<figure style="margin:0 0 1rem;"><img src="/assets/img/lidar/calib-superglue.jpg" alt="SuperGlue matches between the fisheye frame and the LiDAR intensity image" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">3 · Learned feature matches between camera and LiDAR intensity images give the initial extrinsic guess (282 matches here).</figcaption></figure>
<figure style="margin:0 0 1rem;"><img src="/assets/img/lidar/calib-fov.jpg" alt="Overlap region between fisheye FoV and LiDAR scan band" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">4 · The overlap between the fisheye field of view (arcs) and the LiDAR elevation/azimuth band, cropped before the information-theoretic refinement.</figcaption></figure>


## Time synchronisation

The X4 has no hardware sync input (the OS1 does), so alignment is done in software on host time.
I measured the camera's average latency relative to the OS1 and the fusion node compensates for
it with a priority queue, matching each LiDAR sweep to the right frame.

## Odometry

FAST-LIO runs on the OS1's built-in IMU and drifts less than Spot's own state estimate, and it
handles the IMU bias; its output is published as the robot's IMU/odometry for the mapping stack.

## What it produces

<figure style="margin:1rem 0;">
<video controls muted playsinline preload="metadata" poster="/assets/img/lidar/projection-live-poster.jpg" style="width:100%;border-radius:6px;">
  <source src="/assets/img/lidar/projection-live.mp4" type="video/mp4">
</video>
<figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Live registration in rviz: every LiDAR sweep is projected into the fisheye image (points colored by depth) while the accumulated cloud builds on the left — running at full LiDAR rate.</figcaption>
</figure>

<figure style="margin:1rem 0;"><img src="/assets/img/lidar/fusion-mask.jpg" alt="Instance segmentation mask on the dual-fisheye frame" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">The same projection in reverse: detection masks on the fisheye frame are lifted onto the LiDAR points that fall inside them, which is how objects enter the 3D semantic map.</figcaption></figure>

A ROS node applies the transforms to publish registered RGB-D / coloured point clouds for 3D
detection and segmentation. For the object map, 2D detections and segmentation masks are projected
into the cloud through the same transforms, then DBSCAN clusters the hits into dense per-object
point clouds.

*Stack: C++, ROS 2, Ouster SDK, Insta360 SDK, OpenCV calibration, direct_visual_lidar_calibration
(koide3, modified), FAST-LIO, Spot SDK.*
