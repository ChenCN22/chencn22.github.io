---
layout: page
title: "5-DoF Semantic Exploration with VLM-Guided Active Inspection"
permalink: /research/semantic-exploration
---

**CERLAB, CMU · 2025 – 2026 · my main M.S. research project · co-first author**

📄 **Paper:** X. Zhan\*, **S. Chen\***, K. Shimada, "Pose-aware Legged Robot Semantic
Exploration with Omnidirectional Perception in Confined Unknown Environments,"
[arXiv:2609.19460](https://arxiv.org/abs/2609.19460), Sep 2026 — *submitted to ICRA 2027,
under review.* \*Equal contribution.
🔗 **Project page (videos & figures):** [shawn207.github.io/projects/pose](https://shawn207.github.io/projects/pose/)

<figure style="margin:1rem 0;"><img src="/assets/img/pose/frontpage.jpg" alt="Real-world run in a machine shop: (a) Spot pitched −30° to see the top of a band saw, (b) the same viewpoint in the point cloud, (c) fisheye frame, (d) semantic map with object boxes, viewpoints and path, (e) panorama with the five target machines detected." style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Real-world run in a machine shop: (a) Spot pitched −30° to see the top of a band saw, (b) the same viewpoint in the point cloud, (c) fisheye frame, (d) semantic map with object boxes, viewpoints and path, (e) panorama with the five target machines detected.</figcaption></figure>

<div style="background:#f6f6f6;border-left:4px solid #c41230;padding:.8rem 1rem;margin:1rem 0;font-size:.95em;">
<strong>In brief.</strong> A robot sent to inspect an unfamiliar factory floor has to find the machines first, then look at the parts that matter, and the top of a tall machine is exactly what a ground robot cannot see. POSE lets a legged robot tilt its body to get those views, decides <em>when</em> the extra posture is worth the time, and asks a vision-language model to skip visits that would show nothing new. In simulation it covers more of the target surfaces in less time than planar exploration; on a real Spot it found and inspected five machines in an unknown shop.
</div>

## Introduction

Routine inspection of industrial sites (checking a lathe's chuck, a valve's handwheel, the belt of
a conveyor) is work that keeps people in loud, hazardous spaces. Legged robots such as Boston
Dynamics' Spot can already walk those spaces autonomously, but walking is not inspecting. In a
building the robot has never seen, it must discover *where* the equipment is and *what* it is, and
then collect views that actually show the surfaces an inspector would check.

That last step is where a ground robot falls short. Its camera sits low and its vertical field of
view is limited, so the upper surfaces of tall equipment stay unseen no matter how it drives around
them. A quadruped can fix this by pitching and rolling its body, which turns a 3-DoF (planar)
viewpoint into a 5-DoF one. Every tilt, however, costs mission time, and tilting at every object
from every side would make exploration far slower than it needs to be.

**Objective.** Build an exploration system for a legged robot that (1) maps the semantics of an
unknown space while exploring it, (2) chooses body postures only where they add coverage of the
target objects, and (3) does both without blowing up mission time, verified in simulation against
planar and 5-DoF baselines and demonstrated on a real robot.

## Video

<div style="position:relative;padding-bottom:56.25%;height:0;overflow:hidden;max-width:100%;margin:0 0 1rem;">
  <iframe src="https://www.youtube.com/embed/1NR4InKZl2I" title="Real-world run: Spot exploring an unknown machine shop" style="position:absolute;top:0;left:0;width:100%;height:100%;border:0;" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

*Real-world run on Spot in a machine shop (playback 0.6×–2.5×). More figures on the
[project page](https://shawn207.github.io/projects/pose/).*

## Methods

<figure style="margin:1rem 0;"><img src="/assets/img/pose/methodology.jpg" alt="System overview: fusion &amp; mapping → 5-DoF viewpoint sampling with an object-centric VLM session → merged with geometric frontiers in a global TSP planner → aim-aligned posture execution." style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">System overview: fusion &amp; mapping → 5-DoF viewpoint sampling with an object-centric VLM session → merged with geometric frontiers in a global TSP planner → aim-aligned posture execution.</figcaption></figure>

The system runs as a loop of five stages on a Spot carrying an Ouster LiDAR and an Insta360
panoramic camera (the sensor side is on the
[LiDAR–camera registration](/research/lidar-camera-registration) page).

1. **Semantic mapping.** Instance masks from the panoramic image are back-projected through the
   registered LiDAR points into 3D object boxes, giving a semantic occupancy map that grows as the
   robot explores.
2. **Frontier exploration.** Unexplored boundaries of the map become geometric viewpoints, connected
   by an incremental roadmap and ordered into a tour by a travelling-salesman (TSP) solver.
3. **5-DoF viewpoint sampling.** Around each mapped object, candidate viewpoints are sampled with
   body pitch and roll added to the planar pose. Each candidate is scored by the expected number of
   still-unobserved object voxels it would reveal, so a tilt is proposed only where the flat view
   misses something.
4. **VLM-assisted pruning.** Before the robot commits to the candidates, a per-object session with a
   vision-language model receives the observation history, a bird's-eye-view map crop with the
   candidates numbered, and the re-projected views with already-mapped voxels tinted green. It
   returns a keep-or-drop verdict per candidate with a one-line reason. If the verdict is late or
   missing, a geometric fallback is used, so the planner never blocks on the model.
5. **Global planning and aim-aligned execution.** Semantic and frontier viewpoints are merged in one
   TSP planner; postures are executed in a receding-horizon fashion with the body already aimed at
   the object as it arrives, which avoids reorienting after each stop.

<figure style="margin:1rem 0;"><img src="/assets/img/pose/gain-score.jpg" alt="(a) Candidate viewpoints around a mapped object, colored by visibility score; (b) a tilted posture (−30° pitch, −20° roll) sees ~5× more unobserved object voxels than the flat one." style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">(a) Candidate viewpoints around a mapped object, coloured by visibility score; (b) a tilted posture (−30° pitch, −20° roll) sees about 5× more unobserved object voxels than the flat one.</figcaption></figure>

<figure style="margin:1rem 0;"><img src="/assets/img/pose/session.png" alt="What the VLM sees when pruning viewpoints: a BEV map crop with numbered candidates, the re-projected RGB view with mapped voxels tinted green, and a compact structured prompt." style="max-width:760px;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">What the VLM sees when pruning viewpoints: a BEV map crop with numbered candidates, the re-projected RGB view with mapped voxels tinted green, and a compact structured prompt. Its reasoning is overlaid on the frame and streamed to RViz, so every decision is auditable live.</figcaption></figure>

**Evaluation.** Three Isaac Sim industrial scenes with ground-truth scans, run at real-time factor
1.0 with the simulator and the planner in separate Docker containers. Every run is scored
automatically on target-surface coverage over time, time to coverage, number of postures and VLM
cost, against a planar (3-DoF) baseline and other 5-DoF exploration planners. The real-world
experiment is a qualitative demonstration on Spot.

## Results

**Simulation**, three scenes, versus the planar baseline and competing planners:

- **+8–10 percentage points** of final target-surface coverage over the planar baseline;
- **17–32% less exploration time**;
- **53–73% fewer postures** than competing 5-DoF methods;
- the **highest mean object-coverage AUC** among the evaluated baselines.

<figure style="margin:1rem 0;"><img src="/assets/img/pose/sim-env.jpg" alt="Trajectories of the four planners in the three Isaac Sim scenes (stars = tilted inspection postures, squares = target objects). POSE inspects every object with far fewer postures." style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Trajectories of the four planners in the three Isaac Sim scenes (stars = tilted inspection postures, squares = target objects). POSE inspects every object with far fewer postures.</figcaption></figure>

**Real world.** The full stack ran on Spot in a university machine shop it had not seen before. The
robot explored the space, detected five target machines (two turret mills, a lathe, two band saws),
tilted to inspect their upper surfaces and reconstructed them in the semantic map, with the VLM
session pruning repeat visits online.

<figure style="margin:1rem 0;"><img src="/assets/img/pose/panorama.jpg" alt="Panoramic frame from the real run with the five semantic targets detected: two turret mills, a lathe, and two band saws." style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Panoramic frame from the real run with the five semantic targets detected: two turret mills, a lathe, and two band saws.</figcaption></figure>

## Discussion

The coverage-versus-time trade-off is the result that matters. The gain-scored sampling is what
keeps the posture count low: tilting is proposed only where the flat view provably misses object
voxels, and the VLM then removes the candidates that history shows to be redundant. Together they
explain why POSE needs 53–73% fewer postures than other 5-DoF planners while covering more surface
than the planar one.

Two design choices paid off in practice. Keeping the VLM in a separate ROS node with a geometric
fallback meant the C++ planner gained no dependencies and never stalled on a slow API call, which
also made the real-robot run possible. Overlaying the model's reasoning on the queried frame made
its mistakes visible immediately, which was how we tuned the prompt.

The limitations are clear. All quantitative numbers come from simulation; the real-world run shows
the system works end to end but was not benchmarked against baselines or repeated enough for
statistics. The VLM adds latency and API cost that scale with the number of objects, and its
context is still a single frame plus a map crop. We are extending it toward richer context
(object-map crops, multi-frame queries) and toward instruction-driven inspection, where the robot is
asked to locate a named component or answer a question about the scene, and porting the validated
planner to the Isaac Sim 5.0 + ROS 2 branch.

## My role

Equal-contribution first author with Xiaoyang Zhan. Xiaoyang led the frontier exploration planner
and the global TSP tour. I built the sensor fusion and semantic mapping stack on the real robot,
the 5-DoF viewpoint sampling and aim-aligned, receding-horizon posture execution, the VLM session
and pruning (prompt design, the ROS bridge and the fallback), the Isaac Sim benchmark harness that
produced the simulation results, and the real-robot deployment. Prof. Kenji Shimada advised. One
cold-start deadlock from the benchmark harness is written up
[here]({% post_url 2026-07-21-exploration-cold-start-debugging %}).

*Stack: ROS Noetic + Isaac Sim (this branch), Isaac Sim 5.0 + ROS 2 (parallel branch),
C++ (planner), Python (VLM bridge, simulator tooling), Docker.*
