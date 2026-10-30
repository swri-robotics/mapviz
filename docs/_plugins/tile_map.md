---
title: "Tile Map"
description: "Projects a geo-referenced multi-resolution image tile map into the scene.  Map tiles can be obtained from [Bing Maps](https://www.bing.com/mapspreview) or any [WMTS Tile Service](http://www.opengeospatial.org/standards/wmts).  Pre-defined services that access [Stamen Design](http://maps.stamen.com/) (terrain, watercolor, and toner) are provided.  Custom or local WMTS map servers can also be specified.  Map data is cached to disk which enables some limited use completely offline."
image: "satellite.png"
parameters:
  - name: "Source"
    description: "The name of source of the tile data."
  - name: "Base URL"
    description: "A template URL used to obtain map tiles.  When obtaining map tiles, parameters labeled `{level}`, `{x}`, and `{y}` in the URL will be replaced with appropriate values.  For example, `http://tile.stamen.com/terrain/{level}/{x}/{y}.png` is appropriate for retrieving terrain tiles from Stamen Design."
  - name: "API Key"
    description: "When the `Carto`, `Bing Maps (terrain)` or a `Stamen` source is selected, this field holds that service's API key instead of a URL; enter the key and click the `Save` button in order for tiles to be available.  Without a key, Carto serves an \"API KEY REQUIRED\" image for every tile.  To get a Carto key, sign up or log in at [carto.com/basemaps/apikey](https://carto.com/basemaps/apikey) and create a key in the dashboard (non-commercial use is free up to 5M requests per month, commercial use up to 1M).  Carto requires maps to credit \"© OpenStreetMap contributors, © CARTO\".  You can get a Bing Maps Key from the [Microsoft Developer Network](https://msdn.microsoft.com/en-us/library/ff428642.aspx), and a Stadia Maps key (used by the Stamen sources) from [Stadia Maps](https://client.stadiamaps.com/).  Keys are saved in the Mapviz config as `carto_api_key`, `bing_api_key` and `stadia_api_key`."
  - name: "Max Zoom"
    description: "The maximum zoom level that will be used when requesting tiles."
---
