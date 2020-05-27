---
layout: default
title: Plugins
nav_order: 2
---

# Plugins
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

{% for plugin in site.plugins %}
## {{ plugin.name }}

{{ plugin.description }}

{% if plugin.image and plugin.image != "" %}
![]({{ site.baseurl | append: '/assets/images/' }}{{ plugin.image }})
{% endif %}

### Parameters
{: .no_toc }

{% for param in plugin.parameters %}
*   {{ param.name }}

    {{ param.description }}
{% endfor %}

---

{% endfor %}
