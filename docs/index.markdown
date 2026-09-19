---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

# layout: home
layout: page
title:
perlink: /
---

# Hi, I'm Kris (Shiyu Chen 陈时宇)

M.S. student in Mechanical Engineering (Research) @ Carnegie Mellon University,
[Computational Engineering and Robotics Lab (CERLAB)](https://cerlab11.andrew.cmu.edu/),
advised by Prof. Kenji Shimada.

I work on **autonomous robotic exploration and active perception** — teaching legged robots
not just to map a space, but to *understand* what is in it and decide where to look next.

📄 [Curriculum Vitae](/assets/ChenShiyuCV.pdf) ·
💻 [GitHub](https://github.com/ChenCN22) ·
✉️ [shiyuche@andrew.cmu.edu](mailto:shiyuche@andrew.cmu.edu)

### Research Interests

Autonomous exploration & next-best-view planning · semantic mapping ·
LiDAR–camera fusion · VLM-in-the-loop inspection · embodied intelligence

## Current Research

- [**5-DoF Semantic Exploration with VLM-Guided Active Inspection**](/research/semantic-exploration) —
  a Spot robot explores an unknown industrial scene, maps the machines it recognizes, and
  tilts its body (pitch and roll on top of planar motion — 5-DoF viewpoints chosen by
  expected coverage gain, with VLM-assisted viewpoint pruning) to observe the surfaces a
  level gaze cannot see. **+8–10 pp** target-surface coverage and **17–32% less exploration
  time** than the planar baseline in simulation; demonstrated on a real Spot in a machine shop.
  *Co-first author · [arXiv:2609.19460](https://arxiv.org/abs/2609.19460) · submitted to
  ICRA 2027.* (NVIDIA Isaac Sim, ROS, C++/Python)
- [**Real-Time LiDAR–Panoramic Camera Semantic Registration**](/research/lidar-camera-registration) —
  fusing Ouster LiDAR with Insta360 panoramic imagery on a real Boston Dynamics Spot:
  10 FPS dense semantically-colored point clouds with <100 ms latency under gait vibration.

## Selected Past Projects

- [PID Control of a DC Motor in Noisy Environments](/projects/dc-motor) — Wavelet MRPID vs.
  CNN-attention PID (ICCSM 2024)
- Cable-driven soft robotic arm for minimally invasive surgery (Univ. of Cincinnati) —
  published in *ASME J. Medical Diagnostics*, 2026
- Deep-learning defect detection for industrial inspection (CISDI / Chongqing University)

## Latest Posts

 <ul>
  {% for post in site.posts limit:5 %}
    <li style="margin-bottom: 0.6rem;">
      <a href="{{ post.url | relative_url }}">{{ post.title }}</a>
      <span style="color:#777; font-size:0.9em;">
        — {{ post.date | date: "%Y-%m-%d" }}
      </span>
    </li>
  {% endfor %}
</ul>

[All posts →](/posts/)
