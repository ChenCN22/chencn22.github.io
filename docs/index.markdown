---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

# layout: home
layout: page
title: 
perlink: /
---

<!-- <nav style="margin-bottom: 2rem; font-size: 1.2rem;">
  <a href="/posts">Posts</a> • 
  <a href="/projects/">Projects</a> • 
  <a href="/research/">Research</a>
</nav> -->


# Hi, I'm Kris (Shiyu Chen)  
Graduate student in Mechanical Engineering @ CMU  
I'm currently a research student @ [CERLAB](https://cerlab11.andrew.cmu.edu/)

Focus: Robotics, Control Systems, and AI Vision


## Latest Posts
 <ul>
  {% for post in site.posts limit:3 %}
    <li style="margin-bottom: 0.6rem;">
      <a href="{{ post.url | relative_url }}">{{ post.title }}</a>
      <span style="color:#777; font-size:0.9em;">
        — {{ post.date | date: "%Y-%m-%d" }}
      </span>
    </li>
  {% endfor %}
</ul>


### (Under construction) Projects 
Ongoing:
- [Diffusion Models for CT Segmentation](/research/diffusion)

Previous:
- [PID Control of DC Motor](/projects/dc-motor)

### Research Interests
Robotics • Computer Vision • 3D Preception 


