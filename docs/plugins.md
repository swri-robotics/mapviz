---
layout: default
title: Plugins
nav_order: 2
has_children: true
---

# Plugins
{: .no_toc }

<table>
<tr>
<th>Plugin Name</th>
<th>Description</th>
</tr>
{% for plugin in site.plugins %}
<tr>
<td><a href="{{ site.baseurl }}{{ plugin.url }}" class="mb-1 mt-1 v-align-middle">{{ plugin.title | markdownify | remove: '<p>' | remove: '</p>' }}</a></td>
<td>{{ plugin.description | markdownify | remove: '<p>' | remove: '</p>' }}</td>
</tr>
{%- comment -%}
{%- if plugin.image and plugin.image != "" -%}
![]({{ site.baseurl | append: '/assets/images/' }}{{ plugin.image }})
{%- endif -%}
{%- endcomment -%}
{%- endfor -%}
</table>
