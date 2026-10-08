---
layout: page
title: "Real-Time LiDAR–Panoramic Camera Semantic Registration on Spot"
permalink: /research/lidar-camera-registration
---

# Real-Time LiDAR–Panoramic Camera Semantic Registration on a Legged Robot

**CERLAB Roboteam, CMU · 2025 – 2026**

<figure style="margin:1rem 0;"><img src="/assets/img/lidar/hardware-spot.jpg" alt="Spot carrying an Ouster OS-1 LiDAR and an Insta360 X4 camera in a machine shop" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">The sensor suite: Ouster OS-1 LiDAR and Insta360 X4 dual-fisheye camera on a Boston Dynamics Spot, in the machine shop used for the real-robot experiments.</figcaption></figure>

Hardware-side counterpart of my [semantic exploration work](/research/semantic-exploration):
give a real Boston Dynamics Spot dense, semantically-colored 3D perception by fusing an
Ouster LiDAR with an Insta360 X4 panoramic camera. This is the pipeline that turns 2D
detections into the 3D object map the exploration planner reasons about.

## Highlights

- Real-time projection pipeline registering full-rate LiDAR sweeps into the panoramic
  image: **10 FPS at full LiDAR rate, <100 ms latency, <10 px reprojection error**.
- Spatial consistency maintained under the high-frequency vibration of a walking
  quadruped — the failure mode that kills most naive LiDAR–camera rigs on legged
  platforms.
- Designed and 3D-printed a rigid sensor mount so the extrinsic calibration actually
  survives deployment.
- Output: dense semantically-colored point clouds consumed by downstream mapping and
  viewpoint planning.

## Calibration: target-less, from a single scene

Extrinsics between a 360° camera and a spinning LiDAR are hard to get with a checkerboard
alone, so the pipeline uses a **direct visual–LiDAR calibration**: render the accumulated
LiDAR scan as an intensity panorama, match it against the fisheye frame to get an initial
guess, then refine by maximizing mutual information over the overlapping field of view.

<div style="display:grid;grid-template-columns:1fr 1fr;gap:.8rem;margin:1rem 0;">
  <figure style="margin:0;"><img src="/assets/img/lidar/calib-fisheye.jpg" alt="Raw fisheye frame used for calibration" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">1 · Raw Insta360 fisheye frame (omnidirectional camera model).</figcaption></figure>
  <figure style="margin:0;"><img src="/assets/img/lidar/calib-lidar-intensity.jpg" alt="LiDAR intensity panorama" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">2 · The same scene as a LiDAR reflectivity panorama rendered from the accumulated scan.</figcaption></figure>
</div>
<figure style="margin:0 0 1rem;"><img src="/assets/img/lidar/calib-superglue.jpg" alt="SuperGlue matches between the fisheye frame and the LiDAR intensity image" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">3 · Learned feature matches between camera and LiDAR intensity images give the initial extrinsic guess (282 matches here).</figcaption></figure>
<figure style="margin:0 0 1rem;"><img src="/assets/img/lidar/calib-fov.jpg" alt="Overlap region between fisheye FoV and LiDAR scan band" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">4 · The overlap between the fisheye field of view (arcs) and the LiDAR elevation/azimuth band, cropped before the information-theoretic refinement.</figcaption></figure>

Lesson learned the hard way: never calibrate the fisheye intrinsics with a checkerboard that
has QR codes printed on it — they corrupt corner detection.

## What it produces

<figure style="margin:1rem 0;"><img src="/assets/img/lidar/fusion-mask.jpg" alt="Instance segmentation mask on the dual-fisheye frame" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Instance-segmentation masks on the raw dual-fisheye frame; LiDAR points that project into a mask are assigned to that object.</figcaption></figure>

<figure style="margin:1rem 0;"><img src="/assets/img/lidar/fusion-topview.jpg" alt="Top-down view of the RGB-colored LiDAR map with detected machines" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Online fusion during a real run: the registered LiDAR map colored from the panoramic camera, with detected machines boxed and the traveled path.</figcaption></figure>

<figure style="margin:1rem 0;"><img src="/assets/img/pose/panorama.jpg" alt="Panoramic frame with the five semantic targets detected" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Panoramic frame from the same deployment with the five semantic targets detected.</figcaption></figure>

<figure style="margin:1rem 0;">
<video controls muted playsinline preload="metadata" poster="/assets/img/lidar/fusion-demo-poster.jpg" style="width:100%;border-radius:6px;">
  <source src="/assets/img/lidar/fusion-demo.mp4" type="video/mp4">
</video>
<figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">25 s of the real-robot run with fused mapping: RGB-colored map (top view), third-person and first-person views; the posture close-up is slowed to 0.6×.</figcaption>
</figure>

*Stack: ROS, C++, Ouster SDK, OpenCV fisheye calibration, direct visual–LiDAR calibration
(koide3), Spot SDK.*
