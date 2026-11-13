---
layout: page
title: "Real-Time LiDAR–Panoramic Camera Registration on Spot"
permalink: /research/lidar-camera-registration
---

**CERLAB Roboteam, CMU · 2025 – 2026 · solo project (hardware, calibration, software)**

<figure style="margin:1rem 0;"><img src="/assets/img/lidar/fusion-overlay.jpg" alt="Front and back fisheye frames with the LiDAR sweep projected on top, points coloured by depth" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">The result in one frame: every LiDAR point lands on the pixel that sees it, on both fisheye lenses, while the robot walks. Points are coloured by depth.</figcaption></figure>

<div style="background:#f6f6f6;border-left:4px solid #c41230;padding:.8rem 1rem;margin:1rem 0;font-size:.95em;">
<strong>In brief.</strong> A quadruped robot that inspects a factory needs to know not only <em>where</em> things are (LiDAR) but <em>what</em> they are (camera). I built the sensor rig and the software that keeps a 360° camera and a LiDAR aligned on a walking Boston Dynamics Spot: a target-less calibration method, a ROS 2 fusion node, and the time synchronisation between them. The output is a coloured, semantically labelled 3D map at the full LiDAR rate, which became the perception layer of our <a href="/research/semantic-exploration">semantic exploration system</a>.
<br><br>
<strong>Code:</strong> <a href="https://github.com/ChenCN22/lidar_camera_calibration">lidar_camera_calibration</a> (target-less extrinsic calibration, any LiDAR–camera pair) · <a href="https://github.com/ChenCN22/lidar_camera_fusion">lidar_camera_fusion</a> (ROS 2 fusion node for the Insta360 + Ouster rig)
</div>

## Introduction

Industrial inspection is repetitive, sometimes dangerous, and still mostly done by people walking
around with a clipboard. A legged robot can take over the walking: **Spot** is a 32 kg quadruped from
Boston Dynamics that climbs stairs, crosses cluttered shop floors and carries up to 14 kg of
payload on a rail along its back. Out of the box, though, Spot only perceives enough to avoid
obstacles. To *inspect* — to find a lathe, a valve or a conveyor and look at the right part of it —
it needs two things its stock sensors do not give: dense 3D geometry and colour images that cover
the whole surroundings.

A LiDAR provides the geometry and a panoramic camera the appearance, but each is useless to the
other until they share one coordinate frame and one clock. That alignment is the **registration**
problem, and on a walking robot it is harder than on a car: the body shakes with every step, the
camera has no hardware trigger, and a fisheye lens does not fit the pinhole model most calibration
tools assume.

**Objective.** Build a sensor suite and software stack that keeps an Ouster OS1 LiDAR and an
Insta360 X4 dual-fisheye camera registered on Spot in real time, so that every LiDAR point can be
coloured and labelled from the image, and every image detection can be placed in 3D.

## Methods

<figure style="margin:1rem 0;"><img src="/assets/img/lidar/hardware-spot.jpg" alt="Spot carrying an Ouster OS-1 LiDAR and an Insta360 X4 camera in a machine shop" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">The sensor suite: Ouster OS1-128 LiDAR and Insta360 X4 camera on a 3D-printed mount, on Spot's payload rail, in the machine shop used for the real-robot experiments.</figcaption></figure>

The pipeline has five parts: drivers, camera intrinsics, LiDAR–camera extrinsics, time
synchronisation, and the fusion node that applies all of it at run time.

**Hardware and drivers.** The LiDAR and camera sit on a rigid 3D-printed mount that I revised
until the calibration held under gait vibration. Everything on the robot is C++ and
ROS 2. The Ouster driver is stock; for the camera I cut the official Insta360 X4 C++ SDK down to a
ROS 2 node and trimmed its encode/decode path, which was most of the camera latency. The lab had
started with Spot's own panoramic camera, whose driver was unstable and whose stitched panorama has
no published projection model; after testing fixes against the Python API I recommended the switch
to the X4, which the lab adopted.

**Intrinsics.** Both fisheye lenses were calibrated in OpenCV. The camera model was chosen
empirically by reprojection error; the omnidirectional (Mei) model fit the X4's wide fisheye lenses best
and keeps the later projection math simple.

**Extrinsics, target-less and from a single scene.** Checkerboards are awkward with a LiDAR, so the
calibration uses the scene itself. The accumulated LiDAR scan is rendered as a reflectivity
panorama from the LiDAR's viewpoint; learned feature matching (SuperGlue) between that panorama and
the fisheye frame gives an initial transform; the transform is then refined by minimising the
normalised information distance (a mutual-information measure) between LiDAR reflectivity and
image intensity over the overlapping field of view. The method builds on Kenji Koide's
[direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration),
which I adapted to the dual-fisheye rig and wrapped into a reproducible Docker workflow (the
released repository).

<div style="display:grid;grid-template-columns:1fr 1fr;gap:.8rem;margin:1rem 0;">
  <figure style="margin:0;"><img src="/assets/img/lidar/calib-fisheye.jpg" alt="Raw fisheye frame used for calibration" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">1 · Raw Insta360 fisheye frame (omnidirectional camera model).</figcaption></figure>
  <figure style="margin:0;"><img src="/assets/img/lidar/calib-lidar-intensity.jpg" alt="LiDAR intensity panorama" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">2 · The same scene as a LiDAR reflectivity panorama rendered from the accumulated scan.</figcaption></figure>
</div>
<figure style="margin:0 0 1rem;"><img src="/assets/img/lidar/calib-superglue.jpg" alt="SuperGlue matches between the fisheye frame and the LiDAR intensity image" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">3 · Learned feature matches between camera and LiDAR intensity images give the initial extrinsic guess (282 matches here).</figcaption></figure>
<figure style="margin:0 0 1rem;"><img src="/assets/img/lidar/calib-fov.jpg" alt="Overlap region between fisheye FoV and LiDAR scan band" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">4 · The overlap between the fisheye field of view (arcs) and the LiDAR elevation/azimuth band, cropped before the information-theoretic refinement.</figcaption></figure>

**Time synchronisation.** The OS1 accepts a hardware sync signal; the X4 does not. Alignment is
therefore done in software on host time: I measured the camera's latency relative to the LiDAR,
and the fusion node holds incoming frames in a priority queue and pairs each LiDAR sweep with the
frame closest in corrected time.

**Odometry.** FAST-LIO runs on the OS1's built-in IMU. It drifts less than Spot's own state
estimate and handles the IMU bias, so its output is published as the robot's odometry for the
mapping stack.

**Fusion node.** At run time one ROS 2 node applies the extrinsics and the time offset in both
directions: LiDAR points are projected into the front and back fisheye images to publish coloured
point clouds and depth overlays, and 2D detection masks are lifted onto the LiDAR points that fall
inside them, then clustered (DBSCAN) into per-object point clouds for the 3D semantic map.

## Results

<figure style="margin:1rem 0;">
<video controls muted playsinline preload="metadata" poster="/assets/img/lidar/fusion-live-poster.jpg" style="width:100%;max-width:620px;display:block;margin:0 auto;border-radius:6px;">
  <source src="/assets/img/lidar/fusion-live.mp4" type="video/mp4">
</video>
<figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Live run in RViz while Spot walks through the shop. Top: each LiDAR sweep projected into the front and back fisheye frames. Bottom: the coloured point cloud built from the same transforms, rendering at 31 fps.</figcaption></figure>

On the robot the stack runs at the LiDAR's full 10 Hz with under 100 ms end-to-end latency, and
the projected points fall within 10 px of the image features they belong to, across both lenses.
The registration held through walking and the body pitch and roll motions used by the
exploration planner. Each mount revision shows up in the calibration log as a shift of a few
centimetres in the recovered LiDAR-to-camera translation, which the target-less method recovered
from three or four 30-second recordings each time.

<figure style="margin:1rem 0;"><img src="/assets/img/lidar/fusion-mask.jpg" alt="Instance segmentation mask on the dual-fisheye frame" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">The projection in reverse: detection masks on the fisheye frame are lifted onto the LiDAR points inside them. This is how objects enter the 3D semantic map.</figcaption></figure>

The map this produces is what the [semantic exploration system](/research/semantic-exploration)
plans on. In the real-robot experiments reported in our ICRA 2027 submission
([arXiv:2609.19460](https://arxiv.org/abs/2609.19460)), Spot located and inspected five machines
in an unknown shop using this perception layer.

## Discussion

The target-less calibration was the right call. Carrying a calibration target around a shop is
impractical, and the mutual-information refinement converged from the SuperGlue initial guess in
every session we ran. The limitation is that it is an *offline* local optimisation: it gives no
guarantee of a global optimum and it does not notice when the extrinsics drift. The mount, not the
software, is what keeps the registration valid, which is why it took several revisions.
An online, self-checking calibration is the natural next step and is the direction I want to take
in a PhD.

Software time synchronisation worked because the camera's latency was stable, but a camera with a
hardware trigger would remove a source of error we currently only estimate. The omnidirectional
model also leaves residual error at the extreme edge of the fisheye image, which is part of the
10 px figure.

The practical lesson was about sequencing: the drivers and the mount had to be solved before any
calibration result meant anything, and most of the project time went there rather than into the
algorithm.

## My role

This project was mine end to end: the sensor selection and the recommendation to replace Spot's
camera, the 3D-printed mount and its revisions, the Insta360 ROS 2 driver, the intrinsic and
extrinsic calibration and its Docker workflow, the time synchronisation, the fusion node, and the integration with FAST-LIO and the exploration
stack. Prof. Kenji Shimada advised; my labmate Xiaoyang Zhan was the consumer of the map on the
planning side.

*Stack: C++, ROS 2 Humble, Ouster SDK, Insta360 SDK, OpenCV, direct_visual_lidar_calibration
(koide3, modified), SuperGlue, FAST-LIO, Spot SDK, Docker.*
