---
layout: page
title: "All Posts"
permalink: /posts/
---

# All Posts by Category:

{% for cat in site.categories %}
  {% assign key = cat[0] %}
  {% assign display = site.category_display_names[key] | default: key | capitalize %}

## {{ display }}

<ul>
  {% assign posts = cat[1] | sort: 'date' | reverse %}
  {% for post in posts %}
    <li>
      <a href="{{ post.url | relative_url }}">{{ post.title }}</a>
      <span style="color:#777; font-size:0.9em;"> — {{ post.date | date: "%Y-%m-%d" }}</span>
    </li>
  {% endfor %}
</ul>

---
{% endfor %}
