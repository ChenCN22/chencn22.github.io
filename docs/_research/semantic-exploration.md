---
layout: page
title: "5-DoF Semantic Exploration with VLM-Guided Active Inspection"
permalink: /research/semantic-exploration
---


**CERLAB, CMU · 2025 – 2026 · my main M.S. research project**

📄 **Paper:** X. Zhan\*, **S. Chen\***, K. Shimada, "Pose-aware Legged Robot Semantic
Exploration with Omnidirectional Perception in Confined Unknown Environments,"
[arXiv:2609.19460](https://arxiv.org/abs/2609.19460), Sep 2026 — *submitted to ICRA 2027,
under review.* \*Equal contribution.
🔗 **Project page (videos & figures):** [shawn207.github.io/projects/pose](https://shawn207.github.io/projects/pose/)


<figure style="margin:1rem 0;"><img src="/assets/img/pose/frontpage.jpg" alt="Real-world run in a machine shop: (a) Spot pitched −30° to see the top of a band saw, (b) the same viewpoint in the point cloud, (c) fisheye frame, (d) semantic map with object boxes, viewpoints and path, (e) panorama with the five target machines detected." style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Real-world run in a machine shop: (a) Spot pitched −30° to see the top of a band saw, (b) the same viewpoint in the point cloud, (c) fisheye frame, (d) semantic map with object boxes, viewpoints and path, (e) panorama with the five target machines detected.</figcaption></figure>
## Video

<div style="position:relative;padding-bottom:56.25%;height:0;overflow:hidden;max-width:100%;margin:0 0 1rem;">
  <iframe src="https://www.youtube.com/embed/1NR4InKZl2I" title="Real-world run: Spot exploring an unknown machine shop" style="position:absolute;top:0;left:0;width:100%;height:100%;border:0;" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

*Real-world run on Spot in a machine shop (playback 0.6×–2.5×). More figures on the
[project page](https://shawn207.github.io/projects/pose/).*

## The problem

Real inspection targets — a lathe, a valve, a conveyor — have *semantics*, and their most
informative view is rarely the one a flat-bodied robot gets for free: a ground robot's
limited vertical field of view leaves the **upper surfaces** of tall equipment unseen.
A quadruped can tilt its body (pitch and roll → 5-DoF viewpoints) to fix that, but every
extra posture costs mission time. POSE is about getting that trade-off right.

## What the system does

1. **Frontier exploration** with an incremental roadmap planner and TSP-ordered tours.
2. **Semantic mapping** — instance masks back-projected into 3D object boxes.
3. **5-DoF viewpoint sampling** — body pitch and roll added to the planar pose; postures
   chosen by expected coverage gain, executed aim-aligned.
4. **VLM-assisted pruning** — observation history + a bird's-eye-view map let the model
   drop redundant visits; a geometric fallback means the stack never blocks.
5. Semantic and frontier viewpoints merged in one global planner.

<figure style="margin:1rem 0;"><img src="/assets/img/pose/frontpage.jpg" alt="Real-world run in a machine shop: (a) Spot pitched −30° to see the top of a band saw, (b) the same viewpoint in the point cloud, (c) fisheye frame, (d) semantic map with object boxes, viewpoints and path, (e) panorama with the five target machines detected." style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Real-world run in a machine shop: (a) Spot pitched −30° to see the top of a band saw, (b) the same viewpoint in the point cloud, (c) fisheye frame, (d) semantic map with object boxes, viewpoints and path, (e) panorama with the five target machines detected.</figcaption></figure>
## Video

<div style="position:relative;padding-bottom:56.25%;height:0;overflow:hidden;max-width:100%;margin:0 0 1rem;">
  <iframe src="https://www.youtube.com/embed/1NR4InKZl2I" title="Real-world run: Spot exploring an unknown machine shop" style="position:absolute;top:0;left:0;width:100%;height:100%;border:0;" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

*Real-world run on Spot in a machine shop (playback 0.6×–2.5×). More figures on the
[project page](https://shawn207.github.io/projects/pose/).*

## The problem

Real inspection targets — a lathe, a valve, a conveyor — have *semantics*, and their most
informative view is rarely the one a flat-bodied robot gets for free: a ground robot's
limited vertical field of view leaves the **upper surfaces** of tall equipment unseen.
A quadruped can tilt its body (pitch and roll → 5-DoF viewpoints) to fix that, but every
extra posture costs mission time. POSE is about getting that trade-off right.

## What the system does

Spot, carrying an omnidirectional camera–LiDAR suite, explores an unknown industrial scene
end-to-end:

1. **Frontier-based exploration** with an incremental roadmap planner (information-gain
   scoring, TSP-ordered global tours) drives coverage of the unknown map.
2. **Semantic mapping** — 2D instance-segmentation detections are back-projected using the
   **per-instance visible-pixel masks** (not just bounding boxes), DBSCAN-filtered, and
   fused into 3D semantic object boxes in the occupancy map.
3. **5-DoF viewpoint sampling** — candidate viewpoints around each mapped object add the
   robot's **body pitch and roll** to its planar pose; postures are selected from the
   partial object map by **expected coverage gain**.
4. **Aim-aligned execution** — approach and body reorientation are aligned with the
   viewing aim, so the robot spends fewer postures (and less time) per object.
5. **VLM-assisted viewpoint pruning** — an object-centric strategy in which a
   vision-language model, given the persistent observation history and a bird's-eye-view
   (BEV) map, prunes redundant inspection visits. A geometric answer remains the automatic
   fallback on timeout or API error, so the autonomy stack never blocks on a network call.
6. The resulting semantic viewpoints are **merged with geometric exploration viewpoints**
   in one global exploration planner.


<figure style="margin:1rem 0;"><img src="/assets/img/pose/methodology.jpg" alt="System overview: fusion &amp; mapping → 5-DoF viewpoint sampling with an object-centric VLM session → merged with geometric frontiers in a global TSP planner → aim-aligned posture execution." style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">System overview: fusion &amp; mapping → 5-DoF viewpoint sampling with an object-centric VLM session → merged with geometric frontiers in a global TSP planner → aim-aligned posture execution.</figcaption></figure>

<figure style="margin:1rem 0;"><img src="/assets/img/pose/gain-score.jpg" alt="(a) Candidate viewpoints around a mapped object, colored by visibility score; (b) a tilted posture (−30° pitch, −20° roll) sees ~5× more unobserved object voxels than the flat one." style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">(a) Candidate viewpoints around a mapped object, colored by visibility score; (b) a tilted posture (−30° pitch, −20° roll) sees ~5× more unobserved object voxels than the flat one.</figcaption></figure>

<figure style="margin:1rem 0;"><img src="/assets/img/pose/session.png" alt="What the VLM sees when pruning viewpoints: a BEV map crop with numbered candidates, the re-projected RGB view with mapped voxels tinted green, and a compact structured prompt." style="max-width:760px;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">What the VLM sees when pruning viewpoints: a BEV map crop with numbered candidates, the re-projected RGB view with mapped voxels tinted green, and a compact structured prompt.</figcaption></figure>
The VLM's reasoning is overlaid on the queried frame and streamed to rviz, which makes its
decisions auditable in real time.

## Results

**Simulation** — three Isaac Sim industrial scenes, vs. a planar (3-DoF) baseline and other exploration planners:

- **+8–10 percentage points** of final target-surface coverage over the planar baseline;
- **17–32% less exploration time**;
- **53–73% fewer postures** than competing 5-DoF methods;
- the **highest mean object-coverage AUC** among the evaluated baselines.


<figure style="margin:1rem 0;"><img src="/assets/img/pose/sim-env.jpg" alt="Trajectories of the four planners in the three Isaac Sim scenes (stars = tilted inspection postures, squares = target objects). POSE inspects every object with far fewer postures." style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Trajectories of the four planners in the three Isaac Sim scenes (stars = tilted inspection postures, squares = target objects). POSE inspects every object with far fewer postures.</figcaption></figure>
**Real world (qualitative demonstration)** — the full system ran on a Boston Dynamics Spot
with the omnidirectional camera–LiDAR suite in a university machine shop, detecting and
reconstructing five target machines. The sensor side of that deployment is on the
[LiDAR–camera registration](/research/lidar-camera-registration) page.


<figure style="margin:1rem 0;"><img src="/assets/img/pose/panorama.jpg" alt="Panoramic frame from the real run with the five semantic targets detected: two turret mills, a lathe, and two band saws." style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Panoramic frame from the real run with the five semantic targets detected: two turret mills, a lathe, and two band saws.</figcaption></figure>
## Engineering notes

- The VLM bridge is a standalone ROS node speaking plain `PoseStamped` topics — the C++
  planner gained zero dependencies and runs without it.
- A reproducible harness scores every run against a ground-truth scan (coverage over time,
  time-to-coverage, VLM cost) at real-time factor 1.0, with Isaac Sim and the planner in
  separate Docker containers. One cold-start deadlock is written up
  [here]({% post_url 2026-07-21-exploration-cold-start-debugging %}).

## Next steps

Richer VLM context (object-map crops, multi-frame queries), and porting the validated
planner onto the Isaac Sim 5.0 + ROS2 branch.

*Stack: ROS Noetic + Isaac Sim (this branch), Isaac Sim 5.0 + ROS2 (parallel branch),
C++ (planner), Python (VLM bridge, simulator tooling), Docker.*
