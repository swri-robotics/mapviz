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
"""Tests for scripts/make_elevation_set.py, without network access."""

import importlib.util
import math
import os

import numpy as np
import pytest
import yaml

SCRIPT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'make_elevation_set.py')
spec = importlib.util.spec_from_file_location('make_elevation_set', SCRIPT)
mes = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mes)

if mes.gdal is not None:
    mes.gdal.UseExceptions()

needs_gdal = pytest.mark.skipif(mes.gdal is None, reason='GDAL Python bindings not installed')


def test_encoding_round_trips_to_a_256th_of_a_meter():
    heights = np.array([[0.0, -12.5, 8848.86], [233.719, -32768.0, np.nan]])
    decoded = mes.decode(mes.encode(heights))
    assert np.isnan(decoded[1, 2])
    finite = np.isfinite(heights)
    assert np.allclose(decoded[finite], heights[finite], atol=1.0 / 512.0)


def test_encoding_matches_the_terrarium_formula():
    rgba = mes.encode(np.array([[100.0]]))
    r, g, b, a = (int(v) for v in rgba[0, 0])
    assert r * 256 + g + b / 256.0 - 32768.0 == 100.0
    assert a == 255


def test_decode_without_alpha_has_no_gaps():
    rgb = mes.encode(np.array([[5.0, np.nan]]))[..., :3]
    decoded = mes.decode(rgb)
    assert decoded[0, 0] == 5.0
    assert np.isfinite(decoded[0, 1])


def test_tiles_cover_the_bounds():
    x0, x1, y0, y1 = mes.terrarium_tiles((29.44, -98.63, 29.47, -98.59), 15)
    assert (x0, x1) == (mes.tile_column(-98.63, 15), mes.tile_column(-98.59, 15))
    # Rows count down from the north.
    assert y0 == mes.tile_row(29.47, 15) and y1 == mes.tile_row(29.44, 15)
    assert x0 <= x1 and y0 <= y1
    assert mes.tile_column(-180.0, 3) == 0 and mes.tile_column(180.0, 3) == 7


def test_utm_zones():
    assert mes.utm_zone(29.45, -98.61) == (14, 'R')
    assert mes.utm_zone(-33.9, 151.2) == (56, 'H')
    assert mes.utm_zone(84.0, 0.0)[1] == 'X'


def test_downsample_averages_the_cells_with_data():
    heights = np.array([[1.0, 3.0, 5.0], [np.nan, 5.0, np.nan], [7.0, 7.0, 9.0]])
    half = mes.downsample(heights)
    assert half.shape == (2, 2)
    assert half[0, 0] == pytest.approx(3.0)
    assert half[0, 1] == pytest.approx(5.0)
    assert half[1, 0] == pytest.approx(7.0)
    assert half[1, 1] == pytest.approx(9.0)
    assert np.isnan(mes.downsample(np.full((2, 2), np.nan))[0, 0])


def test_layer_count_matches_multires_image():
    assert mes.layer_count(200, 100, 256) == 1
    assert mes.layer_count(300, 100, 256) == 2
    assert mes.layer_count(1024, 1000, 256) == 3
    assert mes.layer_count(1025, 10, 256) == 4


@needs_gdal
def test_mosaic_joins_tiles_in_web_mercator():
    bounds = (29.44, -98.63, 29.47, -98.59)
    x0, x1, y0, y1 = mes.terrarium_tiles(bounds, 15)

    def fetch(zoom, x, y):
        return mes.encode(np.full((256, 256), float((x - x0) * 10 + (y - y0))))[..., :3]

    heights, transform = mes.mosaic_terrarium(bounds, 15, fetch=fetch, workers=2)
    assert heights.shape == ((y1 - y0 + 1) * 256, (x1 - x0 + 1) * 256)
    assert heights[0, 0] == 0.0
    assert heights[-1, -1] == (x1 - x0) * 10 + (y1 - y0)
    assert heights[0, 256] == 10.0
    tile_meters = mes.EARTH_CIRCUMFERENCE / 2 ** 15
    assert transform[1] == pytest.approx(tile_meters / 256.0)
    assert transform[0] == pytest.approx(-mes.MERCATOR_HALF_WIDTH + x0 * tile_meters)


@needs_gdal
def test_written_set_has_tiles_and_corner_tiepoints(tmp_path):
    rows, columns = 300, 600
    heights = np.add.outer(np.arange(rows), np.arange(columns)).astype(np.float32)
    heights[:, 512:] = np.nan
    transform = (-98.6, 0.0001, 0.0, 29.5, 0.0, -0.0001)
    written, size = mes.write_set(
        heights, transform, {'projection': 'wgs84'}, str(tmp_path), 'site', 256, 'Me "too"')
    assert size > 0

    with open(tmp_path / 'site.geo') as geo:
        info = yaml.safe_load(geo)
    assert info['image_path'] == 'site'
    assert (info['image_width'], info['image_height']) == (columns, rows)
    assert info['encoding'] == 'terrarium' and info['extension'] == 'png'
    assert info['attribution'] == 'Me "too"'
    corners = {tuple(p['point'][:2]): p['point'][2:] for p in info['tiepoints']}
    assert corners[(0, 0)] == pytest.approx([-98.6, 29.5])
    assert corners[(columns, rows)] == pytest.approx([-98.6 + 0.06, 29.5 - 0.03])

    # Layer 0 is 3 x 2 tiles, but the column of tiles from 512 on has no data.
    layer0 = sorted(os.listdir(tmp_path / 'site' / 'layer0'))
    assert layer0 == ['tile00000x00000.png', 'tile00000x00001.png',
                      'tile00001x00000.png', 'tile00001x00001.png']
    assert sorted(os.listdir(tmp_path / 'site' / 'layer1')) == ['tile00000x00000.png']
    assert sorted(os.listdir(tmp_path / 'site' / 'layer2')) == ['tile00000x00000.png']
    assert written == 6

    tile = mes.gdal.Open(str(tmp_path / 'site' / 'layer0' / 'tile00001x00001.png'))
    pixels = np.dstack([tile.GetRasterBand(b + 1).ReadAsArray() for b in range(4)])
    decoded = mes.decode(pixels)
    assert decoded[0, 0] == pytest.approx(256 + 256)
    # Past the edge of the data: rows from 300 on.
    assert np.isnan(decoded[300 - 256, 0])


@needs_gdal
def test_dem_in_feet_and_utm_comes_out_in_meters_and_wgs84(tmp_path):
    path = str(tmp_path / 'dem.tif')
    dataset = mes.gdal.GetDriverByName('GTiff').Create(path, 100, 80, 1, mes.gdal.GDT_Float32)
    # 10 m pixels in UTM zone 14 north, near 29.45 N, 98.61 W.
    dataset.SetGeoTransform((536000.0, 10.0, 0.0, 3259000.0, 0.0, -10.0))
    dataset.SetProjection('EPSG:32614')
    band = dataset.GetRasterBand(1)
    band.SetUnitType('ft')
    band.SetNoDataValue(-9999.0)
    values = np.full((80, 100), 1000.0, dtype=np.float32)
    values[:, :10] = -9999.0
    band.WriteArray(values)
    dataset = None

    assert mes.main(['dem', path, '--output', str(tmp_path), '--name', 'dem_set',
                     '--attribution', 'Données © Exemple']) == 0
    with open(tmp_path / 'dem_set.geo', encoding='utf-8') as geo:
        info = yaml.safe_load(geo)
    assert info['projection'] == 'wgs84'
    assert info['attribution'] == 'Données © Exemple'
    with open(tmp_path / 'dem_set' / 'ATTRIBUTION.md', encoding='utf-8') as credits:
        text = credits.read()
    assert 'Données © Exemple' in text
    assert '- dem.tif' in text
    assert 'converted to meters' in text
    lon, lat = info['tiepoints'][0]['point'][2:]
    assert lat == pytest.approx(29.46, abs=0.02) and lon == pytest.approx(-98.62, abs=0.02)

    tile = mes.gdal.Open(str(tmp_path / 'dem_set' / 'layer0' / 'tile00000x00000.png'))
    pixels = np.dstack([tile.GetRasterBand(b + 1).ReadAsArray() for b in range(4)])
    decoded = mes.decode(pixels)
    finite = decoded[np.isfinite(decoded)]
    assert finite.size > 0
    assert finite == pytest.approx(304.8, abs=0.01)
    # The no-data strip along the west edge stays empty.
    assert np.isnan(decoded[40, 0])


@needs_gdal
def test_dem_to_utm_at_a_set_resolution(tmp_path):
    path = str(tmp_path / 'dem.tif')
    dataset = mes.gdal.GetDriverByName('GTiff').Create(path, 50, 50, 1, mes.gdal.GDT_Float32)
    dataset.SetGeoTransform((-98.62, 0.0002, 0.0, 29.46, 0.0, -0.0002))
    dataset.SetProjection('EPSG:4326')
    dataset.GetRasterBand(1).WriteArray(np.full((50, 50), 250.0, dtype=np.float32))
    dataset = None

    assert mes.main(['dem', path, '--output', str(tmp_path), '--name', 'utm_set',
                     '--projection', 'utm', '--resolution', '5']) == 0
    with open(tmp_path / 'utm_set.geo') as geo:
        info = yaml.safe_load(geo)
    assert (info['projection'], info['utm_zone'], info['utm_band']) == ('utm', 14, 'R')
    points = {tuple(p['point'][:2]): p['point'][2:] for p in info['tiepoints']}
    width = info['image_width']
    assert points[(width, 0)][0] - points[(0, 0)][0] == pytest.approx(5.0 * width)
    # About 0.01 degrees of longitude across: roughly 970 m, so about 194 pixels.
    assert math.isclose(width, 194, abs_tol=4)


def test_terrarium_attribution_lists_every_source(tmp_path):
    os.makedirs(tmp_path / 'site')
    path = mes.write_attribution(
        str(tmp_path), 'site', 'Elevation: Mapzen Terrain Tiles', mes.terrarium_sources(15),
        ['Reprojected.'])
    with open(path, encoding='utf-8') as credits:
        text = credits.read()
    assert 'Elevation: Mapzen Terrain Tiles' in text
    assert mes.TERRARIUM_ATTRIBUTION_URL in text
    for _, _, credit in mes.TERRARIUM_SOURCES:
        assert credit in text
    assert 'Contains information licensed under the Open Government Licence – Canada.' in text
    assert '- Reprojected.' in text


def test_terrarium_credit_names_the_regional_sources():
    # The same places as test_elevation_set.cpp's credit test, so the two copies
    # of the regional table stay in step.
    usgs = ('Elevation: Mapzen Terrain Tiles; 3DEP, SRTM and GMTED2010 courtesy of the U.S. '
            'Geological Survey; ETOPO1 terrain data U.S. NOAA')
    assert mes.terrarium_attribution((29.40, -98.70, 29.50, -98.55), 15) == usgs

    oslo = mes.terrarium_attribution((59.85, 10.60, 59.95, 10.85), 15)
    assert oslo.startswith('Elevation: Mapzen Terrain Tiles; SRTM and GMTED2010')
    assert 'EU-DEM layers' in oslo
    assert 'Norway terrain data © Kartverket' in oslo
    assert 'ArcticDEM' not in oslo
    assert 'Canada' not in oslo
    assert 'Kartverket' not in mes.terrarium_attribution((59.85, 10.60, 59.95, 10.85), 5)

    def credit(bounds, zoom):
        return mes.terrarium_attribution(bounds, zoom)

    # Along borders, the credit goes with the side of the border the view is on.
    toronto = credit((43.60, -79.50, 43.75, -79.30), 12)
    assert 'Canada' in toronto and '3DEP' not in toronto
    seattle = credit((47.55, -122.40, 47.65, -122.25), 12)
    assert '3DEP' in seattle and 'Canada' not in seattle
    munich = credit((48.10, 11.50, 48.20, 11.65), 12)
    assert 'EU-DEM' in munich and 'Austria' not in munich
    dublin = credit((53.30, -6.35, 53.40, -6.20), 12)
    assert 'EU-DEM' in dublin and 'United Kingdom' not in dublin
    monterrey = credit((25.60, -100.40, 25.75, -100.20), 12)
    assert 'INEGI' in monterrey and '3DEP' not in monterrey
    assert 'INEGI' not in credit((14.55, -90.60, 14.70, -90.45), 12)  # Guatemala City
    assert 'EU-DEM' not in credit((50.40, 30.40, 50.50, 30.60), 12)  # Kyiv

    vienna = mes.terrarium_attribution((48.1, 16.2, 48.3, 16.5), 12)
    assert 'Geländemodell' in vienna
    assert 'EU-DEM layers' in vienna

    assert mes.terrarium_attribution((-90.0, -180.0, 90.0, 180.0), 0) == (
        'Elevation: Mapzen Terrain Tiles; SRTM and GMTED2010 courtesy of the U.S. Geological '
        'Survey; ETOPO1 terrain data U.S. NOAA')


def test_attribution_can_be_set_for_either_source():
    terrarium = mes.parse_args(['terrarium', '--bounds', '1', '2', '3', '4', '--output', 'o',
                                '--name', 'n', '--attribution', 'EU-DEM credit'])
    assert terrarium.attribution == 'EU-DEM credit'
    assert mes.parse_args(['terrarium', '--bounds', '1', '2', '3', '4', '--output', 'o',
                           '--name', 'n']).attribution is None
    assert mes.parse_args(['dem', 'a.tif', '--output', 'o', '--name', 'n']).attribution is None


def test_unknown_height_unit_is_refused():
    class Band:

        def GetUnitType(self):
            return 'cubits'

    class Dataset:

        def GetRasterBand(self, index):
            return Band()

        def GetDescription(self):
            return 'odd.tif'

    with pytest.raises(ValueError):
        mes.height_scale([Dataset()], 'auto')
    assert mes.height_scale([Dataset()], 'us-ft') == pytest.approx(0.3048006096)
