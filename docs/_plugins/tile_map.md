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
    description: "When the `Bing Maps (terrain)` source is selected, you must enter a Bing Maps access key here and click the `Save` button in order for tiles to be available.  You can get a Bing Maps Key from the [Microsoft Developer Network](https://msdn.microsoft.com/en-us/library/ff428642.aspx)."
  - name: "Max Zoom"
    description: "The maximum zoom level that will be used when requesting tiles."
---
