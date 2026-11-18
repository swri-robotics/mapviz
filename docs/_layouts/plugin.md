---
layout: default
---

{% capture content %}

# {{ page.title }}

{{ page.description }}

{% if page.image and page.image != "" %}
![]({{ site.baseurl | append: '/assets/images/' }}{{ page.image }})
{% endif %}

{% comment %}Longer explanation shown on the plugin's own page only, not in
the table on the Plugins page.{% endcomment %}
{% if page.details %}
{{ page.details }}
{% endif %}

## Parameters

{% if page.parameters %}
  {% for param in page.parameters %}
| {{ param.name }} | {{ param.description | replace: "|","/"}} |
  {%- endfor -%}
{% else %}
  No parameters.
{% endif %}

{% endcapture %}

{{ content | markdownify }}
