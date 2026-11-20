#!/usr/bin/env python3
# Copyright (c) 2026, Southwest Research Institute® (SwRI®)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
r"""
Build an elevation set that tile_map can use without a network connection.

An elevation set is a .geo file and a folder of tiles, laid out like the
imagery multires_image shows, with heights stored in the Terrarium encoding.
tile_map's terrain shading and contour lines read it when it covers the view.

    # Download AWS Terrain Tiles for an area (no key needed):
    make_elevation_set terrarium --bounds 29.40 -98.70 29.50 -98.55 \\
        --output ~/.map_tiles --name swri_elevation

    # Convert your own elevation models (any GeoTIFF GDAL can read):
    make_elevation_set dem USGS_1M_14_x55y327.tif --output ~/.map_tiles --name swri_1m

Needs GDAL's Python bindings (apt install python3-gdal) and numpy.
"""

import argparse
import concurrent.futures
import math
import os
import shutil
import sys
import time
import urllib.request

import numpy as np

try:
    from osgeo import gdal
except ImportError:  # Reported by main(), so the module can still be imported.
    gdal = None

TERRARIUM_URL = 'https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png'
TERRARIUM_MAX_ZOOM = 15
# Boxes (south, west, north, east) around the areas the Terrain Tiles'
# regional data sets cover, from the data sources page, and the credit each
# asks for, from the attribution page.  A box may reach a little past its area
# along a border, so a credit may show near its area but never goes missing
# inside it.  tile_map has the same table (src/terrain.cpp); keep the two in
# step.
TERRARIUM_UNITED_STATES = (
    (31.3, -125.0, 49.0, -104.4), (29.5, -106.6, 31.3, -104.4), (28.9, -104.4, 49.0, -95.2),
    (25.8, -100.6, 29.0, -97.0), (24.3, -97.0, 31.0, -79.9), (31.0, -95.2, 49.4, -89.5),
    (31.0, -89.5, 47.5, -84.8), (31.0, -84.8, 45.9, -82.4), (31.0, -82.4, 42.3, -79.8),
    (31.0, -79.8, 43.4, -76.3), (31.0, -76.3, 45.1, -73.3), (31.0, -73.3, 45.4, -69.0),
    (43.0, -71.1, 47.5, -66.9), (51.0, -180.0, 71.6, -141.0), (54.6, -141.0, 60.4, -129.9),
    (51.0, 172.0, 53.1, 180.0), (18.8, -160.4, 22.4, -154.6), (17.6, -67.4, 18.6, -64.4),
    (13.1, 144.5, 20.7, 146.2), (-14.7, -171.2, -11.0, -168.0),
)
TERRARIUM_REGIONS = (
    ((
        (60.0, -180.0, 90.0, 180.0),
    ),
     'ArcticDEM terrain data DEM(s) were created from DigitalGlobe, Inc., '
     'imagery and funded under National Science Foundation awards 1043681, 1559691, and 1542736'),
    ((
        (-39.3, 129.0, -10.8, 150.1),
    ),
     'Australia terrain data © Commonwealth of Australia (Geoscience '
     'Australia) 2017'),
    ((
        (46.3, 9.5, 47.8, 13.0), (46.3, 12.9, 49.1, 17.2),
    ),
     'Austria terrain data © offene Daten Österreichs – Digitales '
     'Geländemodell (DGM) Österreich'),
    ((
        (48.3, -141.1, 83.2, -95.1), (48.0, -95.1, 83.2, -52.5), (46.2, -90.0, 48.0, -84.0),
        (45.3, -84.0, 46.2, -81.5), (41.6, -83.5, 43.4, -78.9), (43.2, -82.6, 46.2, -74.3),
        (45.0, -74.3, 48.0, -52.5), (43.3, -67.0, 45.0, -59.5),
    ),
     'Canada terrain data contains information licensed under the Open '
     'Government Licence – Canada'),
    ((
        (36.0, -25.0, 71.5, 24.2), (60.5, 24.2, 70.1, 31.6), (59.7, 24.2, 60.5, 28.0),
        (53.9, 24.2, 59.7, 28.3), (34.8, 19.0, 42.0, 28.3), (42.0, 22.0, 48.3, 29.8),
        (34.5, 32.2, 35.8, 34.7), (35.7, 14.1, 36.1, 14.6),
    ),
     'Europe terrain data produced using Copernicus data and information '
     'funded by the European Union - EU-DEM layers'),
    ((
        (22.0, -118.5, 32.75, -114.7), (22.0, -114.8, 32.5, -111.0),
        (14.5, -111.0, 31.4, -108.2), (14.5, -108.2, 31.8, -106.4), (14.5, -106.4, 30.9, -104.4),
        (14.5, -104.4, 29.9, -102.3), (14.5, -102.3, 29.9, -100.6), (14.5, -100.6, 28.7, -99.5),
        (14.5, -99.5, 27.7, -98.5), (14.5, -98.5, 26.4, -97.1), (17.8, -97.1, 25.9, -86.7),
        (14.5, -97.1, 17.8, -92.2), (16.0, -92.2, 17.8, -90.4),
    ),
     'Mexico terrain data source: INEGI, Continental relief, 2016'),
    ((
        (-47.4, 166.3, -34.3, 178.7),
    ),
     'New Zealand terrain data Copyright 2011 Crown copyright (c) Land '
     'Information New Zealand and the New Zealand Government (All rights reserved)'),
    ((
        (57.9, 4.4, 63.0, 12.9), (63.0, 7.0, 66.0, 14.6), (66.0, 11.0, 68.6, 18.2),
        (68.6, 15.0, 71.3, 31.2),
    ),
     'Norway terrain data © Kartverket'),
    ((
        (49.8, -5.8, 55.9, 1.8), (54.0, -8.2, 55.4, -5.4), (54.6, -7.7, 60.9, -0.7),
    ),
     'United Kingdom terrain data © Environment Agency copyright and/or '
     'database right 2015. All rights reserved'),
)
# The regional data sets are used from zoom 7 (Canada) or 9 and 10 (the
# rest); coarser zooms use only the global ones.
TERRARIUM_REGIONAL_ZOOM = 7
TERRARIUM_ATTRIBUTION_URL = (
    'https://github.com/tilezen/joerd/blob/master/docs/attribution.md')
TERRARIUM_SOURCES_URL = (
    'https://github.com/tilezen/joerd/blob/master/docs/data-sources.md')
# The data sets the Terrain Tiles are compiled from, their licenses and the
# credit each asks for, as the Terrain Tiles attribution page gives them.
# The tiles don't say which source each pixel came from, so a set's
# ATTRIBUTION.md lists them all.
TERRARIUM_SOURCES = (
    ('USGS 3DEP (formerly NED), United States', 'public domain',
     '3DEP data courtesy of the U.S. Geological Survey'),
    ('SRTM, global', 'public domain in the United States',
     'SRTM data courtesy of the U.S. Geological Survey'),
    ('GMTED2010, global', 'public domain',
     'GMTED2010 data courtesy of the U.S. Geological Survey'),
    ('ETOPO1, global including bathymetry', 'public domain; not to be used for navigation',
     'Global ETOPO1 terrain data U.S. National Oceanic and Atmospheric Administration'),
    ('ArcticDEM', 'released without limitation',
     'ArcticDEM terrain data DEM(s) were created from DigitalGlobe, Inc., imagery and '
     'funded under National Science Foundation awards 1043681, 1559691, and 1542736'),
    ('Geoscience Australia 5 m LiDAR DEM, Australia', 'CC BY 4.0',
     '© Commonwealth of Australia (Geoscience Australia) 2017.'),
    ('data.gv.at DGM, Austria', 'CC BY 3.0 Austria',
     '© offene Daten Österreichs – Digitales Geländemodell (DGM) Österreich.'),
    ('CDEM, Canada', 'Open Government Licence – Canada',
     'Contains information licensed under the Open Government Licence – Canada.'),
    ('EU-DEM, Europe', 'Copernicus data policy',
     'Produced using Copernicus data and information funded by the European Union - '
     'EU-DEM layers.'),
    ('INEGI Continental relief, Mexico', 'INEGI free use of information',
     'Source: INEGI, Continental relief, 2016'),
    ('LINZ 8 m DEM, New Zealand', 'CC BY 3.0 New Zealand',
     'Copyright 2011 Crown copyright (c) Land Information New Zealand and the New Zealand '
     'Government. All rights reserved'),
    ('Kartverket DTM, Norway', 'CC BY 4.0', '© Kartverket'),
    ('Environment Agency LIDAR composite DTM, United Kingdom', 'Open Government Licence v3',
     '© Environment Agency copyright and/or database right 2015. All rights reserved.'),
)
USER_AGENT = 'mapviz-tile_map-make_elevation_set/1.0'

EARTH_CIRCUMFERENCE = 40075016.685578488
MERCATOR_HALF_WIDTH = EARTH_CIRCUMFERENCE / 2.0
METERS_PER_DEGREE = EARTH_CIRCUMFERENCE / 360.0
UTM_BANDS = 'CDEFGHJKLMNPQRSTUVWX'

# Meters in one unit of height, by the names GDAL reports.
HEIGHT_UNITS = {
    'm': 1.0, 'meter': 1.0, 'meters': 1.0, 'metre': 1.0, 'metres': 1.0,
    'ft': 0.3048, 'foot': 0.3048, 'feet': 0.3048, 'international foot': 0.3048,
    'us-ft': 1200.0 / 3937.0, 'ftus': 1200.0 / 3937.0, 'us survey foot': 1200.0 / 3937.0,
    'us_survey_foot': 1200.0 / 3937.0, 'foot_us': 1200.0 / 3937.0,
}


def encode(heights):
    """
    Encode heights in meters as Terrarium RGBA pixels.

    Height is R * 256 + G + B / 256 - 32768.  Cells that aren't finite get
    alpha 0, which marks them as having no data.
    """
    heights = np.asarray(heights, dtype=np.float64)
    valid = np.isfinite(heights)
    value = np.where(valid, heights, 0.0) + 32768.0
    steps = np.clip(np.round(value * 256.0), 0, 256 * 65536 - 1).astype(np.int64)
    rgba = np.zeros(heights.shape + (4,), dtype=np.uint8)
    rgba[..., 0] = steps >> 16
    rgba[..., 1] = (steps >> 8) & 255
    rgba[..., 2] = steps & 255
    rgba[..., 3] = np.where(valid, 255, 0)
    return rgba


def decode(pixels):
    """Heights in meters from Terrarium RGB or RGBA pixels; NaN where alpha is 0."""
    pixels = np.asarray(pixels)
    heights = (pixels[..., 0].astype(np.float64) * 256.0 + pixels[..., 1] +
               pixels[..., 2] / 256.0 - 32768.0)
    if pixels.shape[-1] == 4:
        heights = np.where(pixels[..., 3] == 0, np.nan, heights)
    return heights


def tile_column(longitude, zoom):
    """Return the web mercator tile column that a longitude falls in."""
    n = 2 ** zoom
    return min(n - 1, max(0, int(math.floor((longitude + 180.0) / 360.0 * n))))


def tile_row(latitude, zoom):
    """Return the web mercator tile row that a latitude falls in."""
    n = 2 ** zoom
    lat = math.radians(min(85.0511, max(-85.0511, latitude)))
    y = (1.0 - math.asinh(math.tan(lat)) / math.pi) / 2.0 * n
    return min(n - 1, max(0, int(math.floor(y))))


def terrarium_tiles(bounds, zoom):
    """Return the tiles covering (south, west, north, east): first and last column and row."""
    south, west, north, east = bounds
    return (tile_column(west, zoom), tile_column(east, zoom),
            tile_row(north, zoom), tile_row(south, zoom))


def download_tile(zoom, x, y, attempts=3):
    """One Terrarium tile's pixels, as a (256, 256, 3) array."""
    url = TERRARIUM_URL.format(z=zoom, x=x, y=y)
    request = urllib.request.Request(url, headers={'User-Agent': USER_AGENT})
    for attempt in range(attempts):
        try:
            with urllib.request.urlopen(request, timeout=60) as reply:
                data = reply.read()
            break
        except OSError:
            if attempt + 1 == attempts:
                raise
            time.sleep(2.0 ** attempt)
    path = f'/vsimem/terrarium_{zoom}_{x}_{y}.png'
    gdal.FileFromMemBuffer(path, data)
    try:
        dataset = gdal.Open(path)
        pixels = np.dstack([dataset.GetRasterBand(b + 1).ReadAsArray()
                            for b in range(dataset.RasterCount)])
    finally:
        dataset = None
        gdal.Unlink(path)
    return pixels


def mosaic_terrarium(bounds, zoom, fetch=download_tile, workers=8, progress=None):
    """
    Download the Terrarium tiles covering an area and join them.

    Returns the heights and a GDAL geotransform in web mercator (EPSG:3857).
    """
    x0, x1, y0, y1 = terrarium_tiles(bounds, zoom)
    columns = x1 - x0 + 1
    rows = y1 - y0 + 1
    heights = np.full((rows * 256, columns * 256), np.nan, dtype=np.float32)

    def get(tile):
        x, y = tile
        return tile, decode(fetch(zoom, x, y))

    tiles = [(x, y) for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)]
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        for done, ((x, y), tile) in enumerate(pool.map(get, tiles), start=1):
            row = (y - y0) * 256
            col = (x - x0) * 256
            heights[row:row + 256, col:col + 256] = tile
            if progress:
                progress(done, len(tiles))

    tile_meters = EARTH_CIRCUMFERENCE / 2 ** zoom
    transform = (-MERCATOR_HALF_WIDTH + x0 * tile_meters, tile_meters / 256.0, 0.0,
                 MERCATOR_HALF_WIDTH - y0 * tile_meters, 0.0, -tile_meters / 256.0)
    return heights, transform


def utm_zone(latitude, longitude):
    """Return the UTM zone number and latitude band letter for a point."""
    zone = int(math.floor((longitude + 180.0) / 6.0)) + 1
    band = UTM_BANDS[min(len(UTM_BANDS) - 1, max(0, int(math.floor((latitude + 80.0) / 8.0))))]
    return min(60, max(1, zone)), band


def target_projection(projection, latitude, longitude):
    """
    Where to put the set: GDAL's name for the coordinate system and the .geo keys.

    wgs84 sets are in longitude and latitude; utm sets in the zone around the
    point given, on the WGS 84 datum.
    """
    if projection == 'wgs84':
        return 'EPSG:4326', {'projection': 'wgs84'}
    zone, band = utm_zone(latitude, longitude)
    epsg = (32600 if latitude >= 0.0 else 32700) + zone
    return f'EPSG:{epsg}', {'projection': 'utm', 'utm_zone': zone, 'utm_band': band}


def pixel_size(projection, meters, latitude):
    """Pixel width and height in the set's coordinates for a size in meters."""
    if projection == 'wgs84':
        return (meters / (METERS_PER_DEGREE * math.cos(math.radians(latitude))),
                meters / METERS_PER_DEGREE)
    return meters, meters


def warp(source, srs, resolution=None, bounds=None):
    """
    Reproject a GDAL dataset (or list of them) into `srs`.

    Returns the heights as float32, NaN where there is no data, and the
    geotransform.  `bounds` (south, west, north, east) crops the result.
    """
    options = {
        'format': 'MEM', 'dstSRS': srs, 'resampleAlg': 'bilinear',
        'dstNodata': float('nan'), 'outputType': gdal.GDT_Float32, 'multithread': True,
    }
    if resolution is not None:
        options['xRes'], options['yRes'] = resolution
    if bounds is not None:
        south, west, north, east = bounds
        options['outputBounds'] = (west, south, east, north)
        options['outputBoundsSRS'] = 'EPSG:4326'
    result = gdal.Warp('', source, options=gdal.WarpOptions(**options))
    if result is None:
        raise RuntimeError('GDAL could not reproject the elevation data')
    heights = result.GetRasterBand(1).ReadAsArray().astype(np.float32)
    return heights, result.GetGeoTransform()


def downsample(heights):
    """Halve a grid, averaging the cells that have data in each 2 x 2 block."""
    rows, columns = heights.shape
    padded = np.full((rows + rows % 2, columns + columns % 2), np.nan, dtype=np.float64)
    padded[:rows, :columns] = heights
    blocks = padded.reshape(padded.shape[0] // 2, 2, padded.shape[1] // 2, 2)
    valid = np.isfinite(blocks)
    count = valid.sum(axis=(1, 3))
    total = np.where(valid, blocks, 0.0).sum(axis=(1, 3))
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.where(count > 0, total / count, np.nan).astype(np.float32)


def layer_count(width, height, tile_size):
    """How many layers multires_image would look for (see TileSet::Load())."""
    largest = max(width, height)
    if largest <= tile_size:
        return 1
    return int(math.ceil(math.log2(largest / tile_size))) + 1


def write_png(path, rgba):
    """Write RGBA pixels as a PNG."""
    rows, columns, _ = rgba.shape
    memory = gdal.GetDriverByName('MEM').Create('', columns, rows, 4, gdal.GDT_Byte)
    for band in range(4):
        memory.GetRasterBand(band + 1).WriteArray(rgba[..., band])
    gdal.GetDriverByName('PNG').CreateCopy(path, memory, options=['ZLEVEL=9'])


def write_set(heights, transform, keys, output, name, tile_size=256, attribution=''):
    """
    Write heights as an elevation set: output/name.geo and the output/name folder.

    `transform` is the GDAL geotransform of the heights, in the coordinates
    `keys` describe ({'projection': 'wgs84'} or utm with zone and band).
    Tiles with no data at all are left out.  Returns the number of tiles
    written and their total size in bytes.
    """
    gdal.SetConfigOption('GDAL_PAM_ENABLED', 'NO')
    rows, columns = heights.shape
    folder = os.path.join(output, name)
    if os.path.isdir(folder):
        shutil.rmtree(folder)
    os.makedirs(folder)

    written = 0
    size = 0
    layer = heights
    for index in range(layer_count(columns, rows, tile_size)):
        directory = os.path.join(folder, f'layer{index}')
        os.makedirs(directory)
        layer_rows, layer_columns = layer.shape
        for row in range(int(math.ceil(layer_rows / tile_size))):
            for column in range(int(math.ceil(layer_columns / tile_size))):
                tile = np.full((tile_size, tile_size), np.nan, dtype=np.float32)
                part = layer[row * tile_size:(row + 1) * tile_size,
                             column * tile_size:(column + 1) * tile_size]
                tile[:part.shape[0], :part.shape[1]] = part
                if not np.isfinite(tile).any():
                    continue
                path = os.path.join(directory, f'tile{row:05d}x{column:05d}.png')
                write_png(path, encode(tile))
                written += 1
                size += os.path.getsize(path)
        layer = downsample(layer)

    def corner(x, y):
        return (transform[0] + x * transform[1] + y * transform[2],
                transform[3] + x * transform[4] + y * transform[5])

    lines = [
        '# Elevation for mapviz tile_map, made by make_elevation_set.',
        f'image_path: {name}',
        f'image_width: {columns}',
        f'image_height: {rows}',
        f'tile_size: {tile_size}',
        'extension: png',
        'encoding: terrarium',
        'datum: wgs84',
    ]
    lines += [f'{key}: {value}' for key, value in keys.items()]
    if attribution:
        lines.append('attribution: "{}"'.format(attribution.replace('"', '\\"')))
    lines.append('tiepoints:')
    for x, y in ((0, 0), (columns, 0), (0, rows), (columns, rows)):
        cx, cy = corner(x, y)
        lines.append(f'  - point: [{x}, {y}, {cx!r}, {cy!r}]')
    with open(os.path.join(output, name + '.geo'), 'w', encoding='utf-8') as geo:
        geo.write('\n'.join(lines) + '\n')
    return written, size


def write_attribution(output, name, credit, sources, processing):
    """
    Write name/ATTRIBUTION.md: where the heights came from and what was done to them.

    It goes with the set when the set is copied or shipped, as the data's
    licenses ask.  `sources` is a list of lines describing the source data,
    and `processing` a list of what was done to it.
    """
    lines = [
        f'# Elevation set {name}',
        '',
        "Made by make_elevation_set (mapviz tile_map) for tile_map's terrain shading and "
        'contour lines.',
        '',
        '## Credit shown on the map',
        '',
        credit or '(none)',
        '',
        '## Sources',
        '',
    ]
    lines += sources
    lines += ['', '## Changes made to the source data', '']
    lines += [f'- {step}' for step in processing]
    path = os.path.join(output, name, 'ATTRIBUTION.md')
    with open(path, 'w', encoding='utf-8') as out:
        out.write('\n'.join(lines) + '\n')
    return path


def overlaps(bounds, areas):
    """Whether a (south, west, north, east) box overlaps any of `areas`."""
    south, west, north, east = bounds
    return any(south <= area[2] and north >= area[0] and west <= area[3] and east >= area[1]
               for area in areas)


def terrarium_attribution(bounds, zoom):
    """Return the credit tile_map would show for Terrain Tiles over `bounds` at `zoom`."""
    regional = zoom >= TERRARIUM_REGIONAL_ZOOM
    united_states = regional and overlaps(bounds, TERRARIUM_UNITED_STATES)
    credits = [
        'Elevation: Mapzen Terrain Tiles',
        ('3DEP, SRTM' if united_states else 'SRTM') +
        ' and GMTED2010 courtesy of the U.S. Geological Survey',
        'ETOPO1 terrain data U.S. NOAA',
    ]
    if regional:
        credits += [credit for areas, credit in TERRARIUM_REGIONS if overlaps(bounds, areas)]
    return '; '.join(credits)


def terrarium_sources(zoom):
    """Return the ATTRIBUTION.md lines for a set made from the Terrain Tiles."""
    lines = [
        f'AWS Terrain Tiles (Terrarium encoding, zoom {zoom}), compiled by Mapzen from the '
        "data sets below.  The tiles don't record which data set each height came from; "
        'which ones cover an area is shown on the data sources page (' +
        TERRARIUM_SOURCES_URL + ').  Data '
        'sets with a license other than public domain need their credit wherever the data '
        'is shown or passed on (see ' + TERRARIUM_ATTRIBUTION_URL + ').',
        '',
        '| Data set | License | Credit |',
        '| --- | --- | --- |',
    ]
    lines += [f'| {name} | {terms} | {credit} |' for name, terms, credit in TERRARIUM_SOURCES]
    lines += [
        '',
        'The U.S. Geological Survey asks for credit when its data is used for '
        'redistribution, resale, presentation or publication.',
    ]
    return lines


def height_scale(datasets, units):
    """Meters per unit of height in the source files."""
    if units != 'auto':
        return HEIGHT_UNITS[units]
    scales = set()
    for dataset in datasets:
        unit = (dataset.GetRasterBand(1).GetUnitType() or 'm').strip().lower()
        if unit not in HEIGHT_UNITS:
            raise ValueError(
                f'{dataset.GetDescription()}: unknown height unit "{unit}"; set --units')
        scales.add(HEIGHT_UNITS[unit])
    if len(scales) > 1:
        raise ValueError('The files use different height units; convert them separately')
    return scales.pop() if scales else 1.0


def center_of(bounds=None, datasets=()):
    """Latitude and longitude of the middle of the area, for picking a UTM zone."""
    if bounds is not None:
        south, west, north, east = bounds
        return 0.5 * (south + north), 0.5 * (west + east)
    info = gdal.Info(datasets[0], format='json')
    lon, lat = info['wgs84Extent']['coordinates'][0][0]
    lon2, lat2 = info['wgs84Extent']['coordinates'][0][2]
    return 0.5 * (lat + lat2), 0.5 * (lon + lon2)


def describe(heights, transform, keys, written, size, output, name):
    """Print a summary of the set written."""
    rows, columns = heights.shape
    finite = heights[np.isfinite(heights)]
    print(f'Wrote {os.path.join(output, name)}.geo: {columns} x {rows} pixels, '
          f'{keys["projection"]}, {written} tiles, {size / 1e6:.1f} MB')
    if finite.size:
        print(f'Heights from {finite.min():.1f} to {finite.max():.1f} m')
    else:
        print('Warning: no heights in the area', file=sys.stderr)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Build an elevation set for tile_map to use offline.',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = parser.add_subparsers(dest='source', required=True)

    def common(command):
        command.add_argument('--output', required=True, help='folder to write the set into')
        command.add_argument('--name', required=True,
                             help='name of the .geo file and its tile folder')
        command.add_argument('--projection', choices=('wgs84', 'utm'), default='wgs84',
                             help='coordinates of the set (default wgs84)')
        command.add_argument('--resolution', type=float,
                             help='pixel size in meters (default: that of the source)')
        command.add_argument('--tile-size', type=int, default=256, choices=(256, 512))
        command.add_argument('--attribution',
                             help='credit line to show on the map (default: '
                                  'for terrarium, the credits of the data sets that cover '
                                  '--bounds; for dem, none)')

    terrarium = sub.add_parser('terrarium', help='download AWS Terrain Tiles for an area')
    terrarium.add_argument('--bounds', type=float, nargs=4, required=True,
                           metavar=('SOUTH', 'WEST', 'NORTH', 'EAST'))
    terrarium.add_argument('--zoom', type=int, default=TERRARIUM_MAX_ZOOM,
                           help=f'tile level to download, up to {TERRARIUM_MAX_ZOOM}')
    terrarium.add_argument('--max-tiles', type=int, default=2500,
                           help='refuse areas needing more tiles than this (default 2500)')
    common(terrarium)

    dem = sub.add_parser('dem', help='convert elevation GeoTIFFs (or anything GDAL reads)')
    dem.add_argument('files', nargs='+')
    dem.add_argument('--bounds', type=float, nargs=4,
                     metavar=('SOUTH', 'WEST', 'NORTH', 'EAST'), help='crop to this area')
    dem.add_argument('--units', choices=('auto', 'm', 'ft', 'us-ft'), default='auto',
                     help='height unit of the files (default: what they say, else meters)')
    common(dem)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if gdal is None:
        print('make_elevation_set needs GDAL: apt install python3-gdal', file=sys.stderr)
        return 1
    gdal.UseExceptions()
    os.makedirs(args.output, exist_ok=True)

    if args.source == 'terrarium':
        if not 0 <= args.zoom <= TERRARIUM_MAX_ZOOM:
            print(f'--zoom must be 0 to {TERRARIUM_MAX_ZOOM}', file=sys.stderr)
            return 1
        x0, x1, y0, y1 = terrarium_tiles(args.bounds, args.zoom)
        count = (x1 - x0 + 1) * (y1 - y0 + 1)
        if count > args.max_tiles:
            print(f'The area needs {count} tiles at zoom {args.zoom}; use a lower --zoom, '
                  f'a smaller area or --max-tiles', file=sys.stderr)
            return 1
        print(f'Downloading {count} Terrarium tiles at zoom {args.zoom}...')

        def progress(done, total):
            if done % 50 == 0 or done == total:
                print(f'  {done}/{total}')

        heights, transform = mosaic_terrarium(args.bounds, args.zoom, progress=progress)
        source = gdal.GetDriverByName('MEM').Create(
            '', heights.shape[1], heights.shape[0], 1, gdal.GDT_Float32)
        source.SetGeoTransform(transform)
        source.SetProjection('EPSG:3857')
        source.GetRasterBand(1).WriteArray(heights)
        latitude, longitude = center_of(args.bounds)
        meters = args.resolution or (
            EARTH_CIRCUMFERENCE * math.cos(math.radians(latitude)) / 2 ** (args.zoom + 8))
        srs, keys = target_projection(args.projection, latitude, longitude)
        heights, transform = warp(
            source, srs, pixel_size(args.projection, meters, latitude), args.bounds)
        attribution = (terrarium_attribution(args.bounds, args.zoom)
                       if args.attribution is None else args.attribution)
        sources = terrarium_sources(args.zoom)
        processing = [
            f'Tiles covering {args.bounds} (south, west, north, east) downloaded from '
            f'{TERRARIUM_URL} and joined.',
            f'Reprojected from web mercator to {srs} with bilinear resampling, at '
            f'{meters:.2f} m pixels, and cropped to the area.',
            'Re-encoded as Terrarium PNG tiles (1/256 m steps) in layers of halving '
            'resolution, each pixel the mean of the four below it.',
        ]
    else:
        datasets = [gdal.Open(path) for path in args.files]
        try:
            scale = height_scale(datasets, args.units)
        except ValueError as error:
            print(error, file=sys.stderr)
            return 1
        latitude, longitude = center_of(args.bounds, datasets)
        srs, keys = target_projection(args.projection, latitude, longitude)
        resolution = None
        if args.resolution:
            resolution = pixel_size(args.projection, args.resolution, latitude)
        heights, transform = warp(datasets, srs, resolution, args.bounds)
        heights *= scale
        attribution = args.attribution or ''
        sources = [f'- {os.path.basename(path)}' for path in args.files]
        processing = [
            f'Reprojected to {srs} with bilinear resampling'
            + (f', at {args.resolution:g} m pixels' if args.resolution else '')
            + (', and cropped to the area' if args.bounds else '') + '.',
        ]
        if scale != 1.0:
            processing.append(f'Heights converted to meters (multiplied by {scale:.10g}).')
        processing.append(
            'Re-encoded as Terrarium PNG tiles (1/256 m steps) in layers of halving '
            'resolution, each pixel the mean of the four below it.')

    written, size = write_set(
        heights, transform, keys, args.output, args.name, args.tile_size, attribution)
    describe(heights, transform, keys, written, size, args.output, args.name)
    credits = write_attribution(args.output, args.name, attribution, sources, processing)
    print(f'Wrote {credits}; keep it with the set if you pass the set on')
    if attribution:
        print(f'Credit shown on the map: {attribution}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
