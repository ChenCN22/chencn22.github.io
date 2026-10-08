---
layout: page
title:
permalink: /
---

<div class="hero">
  <div class="txt">
    <h1>Shiyu (Kris) Chen</h1>
    <div class="mail">shiyuche at andrew dot cmu dot edu</div>
    <p>I'm an M.S. (Research) student in Mechanical Engineering at <a href="https://www.cmu.edu/">Carnegie Mellon University</a>, working with <a href="https://www.meche.engineering.cmu.edu/directory/bios/shimada-kenji.html">Prof. Kenji Shimada</a> at <a href="https://cerlab11.andrew.cmu.edu/">CERLAB</a> on legged-robot autonomy.</p>
    <p>I want robots to inspect real industrial spaces the way an experienced technician would: take a plain-language request, use the map they already have, decide where and how to look, and report what they found.</p>
    <div class="btns">
      <a class="btn" href="/assets/ChenShiyuCV.pdf">CV</a>
      <a class="btn" href="https://arxiv.org/abs/2609.19460">arXiv</a>
      <a class="btn" href="https://github.com/ChenCN22">GitHub</a>
      <a class="btn" href="/about/">About</a>
    </div>
  </div>
  <div class="pic"><img src="/assets/img/avatar.jpg" alt="Shiyu Chen"><small>Pittsburgh, 2026</small></div>
</div>

## Research interests

<div class="chips">
  <span class="chip">Instruction-driven semantic inspection with legged robots</span>
  <span class="chip">Vision-language-action models for viewpoint &amp; posture selection</span>
  <span class="chip">Learned observation-value models for active perception</span>
  <span class="chip">Closed-loop inspection: perceive, reason, re-plan, report</span>
</div>

## Research

<div class="card">
  <img src="/assets/img/pose/posture.jpg" alt="Spot tilting to inspect a band saw">
  <div class="b">
    <div class="tag">Active perception · ICRA 2027 submission</div>
    <h3><a href="/research/semantic-exploration">5-DoF Semantic Exploration with VLM-Guided Active Inspection</a></h3>
    <p>Spot tilts its body to see the surfaces a level camera misses — +8–10 pp coverage, 17–32 % less time than planar exploration; demonstrated on a real robot.</p>
    <div class="links"><a href="/research/semantic-exploration">page</a><a href="https://arxiv.org/abs/2609.19460">paper</a><a href="https://www.youtube.com/watch?v=1NR4InKZl2I">video</a></div>
  </div>
</div>

<div class="card">
  <img src="/assets/img/lidar/projection-live-poster.jpg" alt="LiDAR points projected onto the fisheye image">
  <div class="b">
    <div class="tag">Perception · hardware</div>
    <h3><a href="/research/lidar-camera-registration">Real-Time LiDAR–Panoramic Camera Registration on Spot</a></h3>
    <p>Target-less calibration and a 10 FPS fusion pipeline that keeps LiDAR and a 360° camera aligned while the robot walks.</p>
    <div class="links"><a href="/research/lidar-camera-registration">page</a></div>
  </div>
</div>

<div class="card">
  <img src="/assets/img/shoe-tile.jpg" alt="In-shoe 3D reconstruction">
  <div class="b">
    <div class="tag">3D sensing · ongoing</div>
    <h3><a href="/research/in-shoe-reconstruction">3D Reconstruction of Shoe Interiors for Footwear Fit</a></h3>
    <p>Measuring the inside of a closed shoe to assess fit for special foot shapes; probe, fixture and ground-truth test plan — target error ≤ 1 mm.</p>
    <div class="links"><a href="/research/in-shoe-reconstruction">page</a></div>
  </div>
</div>

## News

<ul class="news">
  <li><span class="d">2026/09</span><span>Our paper on pose-aware legged-robot semantic exploration is on <a href="https://arxiv.org/abs/2609.19460">arXiv</a> (co-first author) and submitted to ICRA 2027.</span></li>
  <li><span class="d">2026/09</span><span>Full system demonstrated on a real Spot in a CMU machine shop — <a href="https://www.youtube.com/watch?v=1NR4InKZl2I">video</a>.</span></li>
  <li><span class="d">2026/09</span><span>Started a new project with Prof. Shimada on 3D reconstruction of shoe interiors.</span></li>
  <li><span class="d">2025/08</span><span>Joined CMU MechE as an M.S. (Research) student at CERLAB.</span></li>
</ul>

## Publications

<div class="pub">X. Zhan*, <span class="me">S. Chen*</span>, K. Shimada. Pose-aware Legged Robot Semantic Exploration with Omnidirectional Perception in Confined Unknown Environments. <span class="v">arXiv 2609.19460, 2026 · submitted to ICRA 2027 · *equal contribution</span></div>
<div class="pub">Y. Wang, C. Liu, K. Jiang, B. Wu, <span class="me">S. Chen</span>, J. Dong, A. Ashfaq. Innovative Flexible Robotic Arm for Enhanced Precision in Transaortic Surgical Myectomy. <span class="v">ASME J. Medical Diagnostics, 2026</span></div>
<div class="pub">L. Liu, <span class="me">S. Chen</span>, Z. Tang. Intelligent Diagnosis of Bearing Faults Based on SDP Image Fusion. <span class="v">ICICML, 2023</span></div>

## Notes

<ul class="posts-mini">
{% for post in site.posts limit:4 %}
  <li><a href="{{ post.url | relative_url }}">{{ post.title }}</a> <span>— {{ post.date | date: "%Y-%m-%d" }}</span></li>
{% endfor %}
</ul>

[All notes →](/posts/)
