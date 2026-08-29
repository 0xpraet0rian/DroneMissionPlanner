"""
Drone Mission Planner
Single pywebview window — Leaflet map + KML/KMZ import + grid/corridor/orbit/manual
mission planning + DJI WPML (.kmz) export for DJI Fly waypoint missions.

Theme and app skeleton match "Survey Photo Organizer" by the same author.

Copyright (C) 2026  praet0rian (mark0)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import os, sys, math, json, time, zipfile, threading, re, tempfile, shutil, base64
import urllib.request, urllib.parse
import http.server
import xml.etree.ElementTree as ET
import webview

APP_VERSION = '1.2'

# ── Geo helpers ─────────────────────────────────────────────────────────────

EARTH_R = 6_371_000.0

def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))

def destination_point(lat, lon, dist_m, bearing_deg):
    br = math.radians(bearing_deg)
    d = dist_m / EARTH_R
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    lat2 = math.asin(math.sin(lat1) * math.cos(d) + math.cos(lat1) * math.sin(d) * math.cos(br))
    lon2 = lon1 + math.atan2(
        math.sin(br) * math.sin(d) * math.cos(lat1),
        math.cos(d) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)

def bearing_deg(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def to_xy(lat, lon, ref_lat, ref_lon):
    x = (lon - ref_lon) * 111_320.0 * math.cos(math.radians(ref_lat))
    y = (lat - ref_lat) * 110_540.0
    return x, y

def from_xy(x, y, ref_lat, ref_lon):
    lat = ref_lat + y / 110_540.0
    lon = ref_lon + x / (111_320.0 * math.cos(math.radians(ref_lat)))
    return lat, lon

def fetch_elevations_m(coords):
    """Batch ground-elevation lookup (meters, SRTM-derived) for a list of
    (lat, lon) pairs via the free Open-Topo-Data API -- no key required. Used
    for terrain-following altitude: DJI Fly's consumer app doesn't honor the
    WPML aboveGroundLevel height mode (that's a Pilot 2 / FlightHub 2 /
    enterprise-drone feature), so third-party planners like Litchi and Maven
    get the same effect by computing per-waypoint altitude offsets themselves
    and exporting plain relativeToStartPoint heights -- that's the approach
    used here too. Raises on any failure so the caller can surface a clear
    error instead of silently treating a network hiccup as flat terrain."""
    elevations = []
    for i in range(0, len(coords), 100):
        chunk = coords[i:i + 100]
        locs = '|'.join(f'{lat:.6f},{lon:.6f}' for lat, lon in chunk)
        url = 'https://api.opentopodata.org/v1/srtm30m?locations=' + urllib.parse.quote(locs, safe='|,.-')
        with urllib.request.urlopen(url, timeout=15) as resp:
            data = json.loads(resp.read().decode('utf-8'))
        if data.get('status') != 'OK':
            raise RuntimeError('Elevation lookup failed: ' + str(data.get('error', data.get('status'))))
        for r in data.get('results', []):
            e = r.get('elevation')
            if e is None:
                raise RuntimeError('No elevation data for one or more points (outside SRTM coverage -- '
                                    'far north/south, or over open ocean)')
            elevations.append(e)
    return elevations

def footprint(altitude_m, sensor_w_mm, sensor_h_mm, focal_mm):
    """Ground footprint of one photo: altitude * sensorSize / focalLength (same
    formula YMapper uses, equivalent to 2*altitude*tan(FOV/2) but exact rather
    than reconstructed from a rounded diagonal-FOV spec)."""
    w = altitude_m * sensor_w_mm / focal_mm
    h = altitude_m * sensor_h_mm / focal_mm
    return w, h

def camera_fov(sensor_w_mm, sensor_h_mm, focal_mm):
    """Horizontal/vertical/diagonal FOV in degrees, for display only."""
    fov_h = 2 * math.degrees(math.atan(sensor_w_mm / (2 * focal_mm)))
    fov_v = 2 * math.degrees(math.atan(sensor_h_mm / (2 * focal_mm)))
    diag_mm = math.hypot(sensor_w_mm, sensor_h_mm)
    fov_d = 2 * math.degrees(math.atan(diag_mm / (2 * focal_mm)))
    return fov_h, fov_v, fov_d

def recommended_shutter_speed(altitude_m, sensor_w_mm, focal_mm, img_w_px, speed_ms):
    """Fastest shutter speed to avoid motion blur, from GSD/groundspeed. Same formula
    YMapper uses: gsd = altitude*sensorWidth/(imageWidth*focalLength); shutter = gsd/speed,
    snapped to the nearest standard camera shutter speed."""
    if speed_ms <= 0:
        return None
    gsd = (altitude_m * sensor_w_mm) / (img_w_px * focal_mm)
    ideal = gsd / speed_ms
    standard = [1/16000, 1/8000, 1/6400, 1/5000, 1/4000, 1/3200, 1/2500, 1/2000, 1/1600,
                1/1250, 1/1000, 1/800, 1/640, 1/500, 1/400, 1/320, 1/240, 1/200, 1/160,
                1/120, 1/100, 1/80, 1/60, 1/50, 1/40, 1/30, 1/25, 1/20, 1/15, 1/12.5,
                1/10, 1/8, 1/6.25, 1/5, 1/4, 1/3, 1/2]
    closest = min(standard, key=lambda s: abs(ideal - s))
    return round(1 / closest)

def polygon_area_m2(polygon_latlon):
    """Shoelace formula in local meters (same approach YMapper uses for its Area stat)."""
    if len(polygon_latlon) < 3:
        return 0.0
    ref_lat = polygon_latlon[0][0]
    ref_lon = polygon_latlon[0][1]
    pts = [to_xy(p[0], p[1], ref_lat, ref_lon) for p in polygon_latlon]
    area = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return abs(area) / 2.0

def _safe_filename(name):
    """Strip path separators/traversal from a filename that reaches Python via the
    JS bridge — the JS side already sanitizes it, but the export dialog writes to
    disk, so defend here too rather than trusting a single layer."""
    if not name:
        return None
    name = os.path.basename(str(name)).strip()
    name = re.sub(r'[\\/:*?"<>|]', '', name)
    return name or None

# ── KML / KMZ import — namespace-agnostic, tolerant of nested Folders/MultiGeometry ──

def _local(tag):
    return tag.split('}')[-1] if '}' in tag else tag

def _parse_coord_text(text):
    pts = []
    for tok in (text or '').strip().split():
        parts = tok.split(',')
        if len(parts) >= 2:
            try:
                lon, lat = float(parts[0]), float(parts[1])
                pts.append([lat, lon])
            except (ValueError, IndexError):
                pass
    return pts

def _placemark_name(pm):
    for child in pm:
        if _local(child.tag) == 'name' and child.text:
            return child.text.strip()
    return None

def parse_geometries(root):
    """Extract Polygons, LineStrings and Points from any KML tree, ignoring namespace
    variants and nesting (Folder/Document/MultiGeometry). Tolerant by design so it
    works with files from Google Earth, QGIS, DJI, GIS exports, etc."""
    polygons, lines, points = [], [], []
    for pm in root.iter():
        if _local(pm.tag) != 'Placemark':
            continue
        name = _placemark_name(pm)
        for geom in pm.iter():
            tagl = _local(geom.tag)
            if tagl == 'Polygon':
                outer = None
                for b in geom:
                    if _local(b.tag) == 'outerBoundaryIs':
                        for coord_el in b.iter():
                            if _local(coord_el.tag) == 'coordinates' and coord_el.text:
                                outer = _parse_coord_text(coord_el.text)
                                break
                        break
                if outer and len(outer) >= 3:
                    if outer[0] == outer[-1]:
                        outer = outer[:-1]
                    polygons.append({'name': name or f'Area {len(polygons) + 1}', 'coords': outer})
            elif tagl == 'LineString':
                for c in geom:
                    if _local(c.tag) == 'coordinates' and c.text:
                        coords = _parse_coord_text(c.text)
                        if len(coords) < 2:
                            continue
                        # CAD/GIS exports often save a closed area as a
                        # LineString returning to its start rather than a
                        # <Polygon>. Treat that as an area, or it would only
                        # be offered as a corridor and buffered like a road.
                        closed = (len(coords) >= 4
                                  and haversine_m(coords[0][0], coords[0][1], coords[-1][0], coords[-1][1]) < 2.0)
                        if closed:
                            outer = coords[:-1]
                            if len(outer) >= 3:
                                polygons.append({'name': name or f'Area {len(polygons) + 1}', 'coords': outer})
                                continue
                        lines.append({'name': name or f'Route {len(lines) + 1}', 'coords': coords})
            elif tagl == 'Point':
                for c in geom:
                    if _local(c.tag) == 'coordinates' and c.text:
                        coords = _parse_coord_text(c.text)
                        if coords:
                            points.append({'name': name or f'Point {len(points) + 1}',
                                           'lat': coords[0][0], 'lon': coords[0][1]})
    return polygons, lines, points

def parse_kml_kmz(path):
    path_lower = path.lower()
    if path_lower.endswith('.kmz'):
        with zipfile.ZipFile(path, 'r') as z:
            names = [n for n in z.namelist() if n.lower().endswith('.kml')]
            if not names:
                raise ValueError('No .kml file found inside this .kmz')
            main = next((n for n in names if os.path.basename(n).lower() == 'doc.kml'), names[0])
            with z.open(main) as f:
                tree = ET.parse(f)
    else:
        tree = ET.parse(path)
    root = tree.getroot()
    polygons, lines, points = parse_geometries(root)
    if not polygons and not lines and not points:
        raise ValueError('No polygons, routes or points found in this file — '
                          'it may be empty or use an unsupported geometry type (e.g. gx:Track).')
    return polygons, lines, points

# ── Drone / camera presets ───────────────────────────────────────────────────
# droneEnumValue=68 works for DJI Fly on any consumer drone (per YMapper);
# DJI's WPML spec only documents enterprise models. Camera specs come from
# YMapper's preset table, cross-checked against DJI's Mini 4/5 Pro spec pages.
# Footprint = altitude * sensorSize / focalLength.

# ── Gimbal pitch presets ──────────────────────────────────────────────────
# -90° = straight down (nadir), 0° = horizontal, DJI/WPML convention.
# Sources: James & Robson 2014 (Earth Surface Processes and Landforms 39:10) on
# doming error in pure-nadir DEMs, fixed by a 5-10° tilt; DJI Terra manual's
# -45° default oblique tilt for 3D missions; Pix4D's 45-80° oblique band and
# 80/70 overlap recommendation for double-grid missions; roofing-inspection
# guides' -70° for surface damage and -45/-60° orbit passes.
GIMBAL_PRESETS = {
    'nadir_2d': {
        'label': 'Flat 2D map / orthomosaic (farm, land survey)', 'pitch': -90, 'overlap': (75, 65),
        'note': 'Straight-down nadir — the standard for flat-terrain orthomosaics: '
                'minimal distortion, maximum ground coverage per photo.',
    },
    'vegetation': {
        'label': 'Vegetation / crop health survey', 'pitch': -90, 'overlap': (75, 65),
        'note': 'Nadir capture, same as flat 2D mapping. An ordinary RGB drone cannot compute '
                'true NDVI (needs a near-infrared sensor), but visible-light indices computed '
                'from the resulting orthomosaic — ExG = 2G-R-B, VARI = (G-R)/(G+R-B), GLI = '
                '(2G-R-B)/(2G+R+B) — correlate well with canopy vigor and are standard practice '
                'for RGB-only vegetation surveys.',
    },
    'topo_accurate': {
        'label': 'Topographic / elevation model (DEM, volumetrics, cut-fill)', 'pitch': -80, 'overlap': (80, 70),
        'note': 'A 5-10° tilt off nadir breaks the near-parallel imaging geometry that '
                'causes "doming" — a well-documented systematic vertical error in pure-nadir '
                'DEMs (James & Robson, 2014, Earth Surface Processes and Landforms). Pair '
                'with Crosshatch for a proper convergent network, and place ground control '
                'points under Site markup for survey-grade accuracy.',
    },
    '3d_model': {
        'label': '3D model / building / urban scene', 'pitch': -45, 'overlap': (80, 70), 'threeD': True,
        'note': "Matches DJI Terra's own default oblique tilt (-45°) for 3D reconstruction "
                'missions. Turns on "3D mapping (nadir + oblique)" automatically — a single '
                'oblique pass never images vertical surfaces like walls, so this flies the area '
                'twice, once nadir and once oblique, to actually get both.',
    },
    'facade': {
        'label': 'Building facade / vertical structure inspection', 'pitch': -50, 'overlap': (80, 70),
        'note': 'Typical oblique range for facade capture is 45-60° off nadir; -50° '
                'balances wall detail against sky/ground context in each frame.',
    },
    'roof': {
        'label': 'Roof inspection', 'pitch': -70, 'overlap': (80, 70),
        'note': 'Steeper oblique used by roofing-inspection guides to reveal surface damage '
                '(lifted shingles, dents, creases) that a straight-down shot hides. Pair with '
                'a -90° nadir pass for the overhead layout.',
    },
    'corridor': {
        'label': 'Corridor / linear infrastructure (roads, pipelines, rail)', 'pitch': -90, 'overlap': (75, 65),
        'note': 'Standard nadir for as-built/linear route documentation.',
    },
    'custom': {'label': 'Custom', 'pitch': -90, 'overlap': None, 'note': 'Set your own angle below.'},
}

# Rated minutes are manufacturer lab-ideal figures (windless, sea level). See the
# realistic/reserve factors below for how these become a safe per-battery budget.
DRONE_PRESETS = {
    'mini4pro': {'label': 'DJI Mini 4 Pro', 'droneEnumValue': 68, 'droneSubEnumValue': 0,
                 'defaultCamera': 'mini4pro_48',
                 'batteries': [{'label': 'Standard (2590mAh, 34 min rated)', 'minutes': 34},
                               {'label': 'Intelligent Flight Battery Plus (45 min rated, >249g takeoff weight)', 'minutes': 45}]},
    'mini5pro': {'label': 'DJI Mini 5 Pro', 'droneEnumValue': 68, 'droneSubEnumValue': 0,
                 'defaultCamera': 'mini5pro_48',
                 'batteries': [{'label': 'Standard (36 min rated)', 'minutes': 36},
                               {'label': 'Intelligent Flight Battery Plus (4680mAh, 52 min rated)', 'minutes': 52}]},
    'air3': {'label': 'DJI Air 3', 'droneEnumValue': 68, 'droneSubEnumValue': 0,
             'defaultCamera': 'air3_50',
             'batteries': [{'label': 'Standard (46 min rated)', 'minutes': 46}]},
    'air3s': {'label': 'DJI Air 3S', 'droneEnumValue': 68, 'droneSubEnumValue': 0,
              'defaultCamera': 'air3s_48',
              'batteries': [{'label': 'Standard (45 min rated)', 'minutes': 45}]},
    'mavic3': {'label': 'DJI Mavic 3', 'droneEnumValue': 68, 'droneSubEnumValue': 0,
               'defaultCamera': 'mavic3e',
               'batteries': [{'label': 'Standard (46 min rated)', 'minutes': 46}]},
    'mavic3pro': {'label': 'DJI Mavic 3 Pro', 'droneEnumValue': 68, 'droneSubEnumValue': 0,
                  'defaultCamera': 'mavic3e',
                  'batteries': [{'label': 'Standard (43 min rated)', 'minutes': 43}]},
    'custom': {'label': 'Custom / Advanced', 'droneEnumValue': 68, 'droneSubEnumValue': 0,
               'defaultCamera': 'mini4pro_48',
               'batteries': [{'label': 'Custom', 'minutes': 30}]},
}

# Real-world endurance is usually 70-80% of rated, and standard practice reserves
# 20-30% battery for RTH/contingency. Both editable in the UI, not baked in.
BATTERY_REALISTIC_FACTOR_DEFAULT = 0.75
BATTERY_RESERVE_FRACTION_DEFAULT = 0.30

# label: (sensor_w_mm, sensor_h_mm, focal_mm, img_w_px, img_h_px)
CAMERA_PRESETS = {
    'mini4pro_48': {'label': 'DJI Mini 4 Pro (48 MP)', 'sensor_w': 9.7, 'sensor_h': 7.3, 'focal': 6.88, 'img_w': 8064, 'img_h': 6048},
    'mini4pro_12': {'label': 'DJI Mini 4 Pro (12 MP)', 'sensor_w': 9.7, 'sensor_h': 7.3, 'focal': 6.88, 'img_w': 4032, 'img_h': 3024},
    'mini5pro_48': {'label': 'DJI Mini 5 Pro (48 MP)', 'sensor_w': 13.2, 'sensor_h': 8.8, 'focal': 9.0, 'img_w': 8192, 'img_h': 6144},
    'mini5pro_12': {'label': 'DJI Mini 5 Pro (12 MP)', 'sensor_w': 13.2, 'sensor_h': 8.8, 'focal': 9.0, 'img_w': 4098, 'img_h': 3072},
    'air3_50': {'label': 'DJI Air 3 (50 MP, wide)', 'sensor_w': 9.65, 'sensor_h': 7.24, 'focal': 6.72, 'img_w': 8064, 'img_h': 6048},
    'air3_12': {'label': 'DJI Air 3 (12 MP, wide)', 'sensor_w': 9.65, 'sensor_h': 7.24, 'focal': 6.72, 'img_w': 4032, 'img_h': 3024},
    'air3s_48': {'label': 'DJI Air 3S (48 MP, wide)', 'sensor_w': 13.2, 'sensor_h': 8.8, 'focal': 8.67, 'img_w': 8192, 'img_h': 6144},
    'air3s_12': {'label': 'DJI Air 3S (12 MP, wide)', 'sensor_w': 13.2, 'sensor_h': 8.8, 'focal': 8.67, 'img_w': 4032, 'img_h': 3024},
    'mavic2pro': {'label': 'DJI Mavic 2 Pro', 'sensor_w': 13.2, 'sensor_h': 8.8, 'focal': 10.3, 'img_w': 5472, 'img_h': 3648},
    'mavic3e': {'label': 'DJI Mavic 3 / 3E (Hasselblad)', 'sensor_w': 17.3, 'sensor_h': 13.0, 'focal': 12.3, 'img_w': 5280, 'img_h': 3956},
    'mavic3t_rgb': {'label': 'DJI Mavic 3T (RGB)', 'sensor_w': 6.4, 'sensor_h': 4.8, 'focal': 4.4, 'img_w': 8000, 'img_h': 6000},
    'mavicair2': {'label': 'DJI Mavic Air 2 (12 MP)', 'sensor_w': 6.35, 'sensor_h': 4.94, 'focal': 4.8, 'img_w': 4000, 'img_h': 3000},
    'mavicair2s': {'label': 'DJI Mavic Air 2S', 'sensor_w': 13.05, 'sensor_h': 8.82, 'focal': 9.18, 'img_w': 5472, 'img_h': 3648},
    'mini2': {'label': 'DJI Mini 2 (12 MP)', 'sensor_w': 6.3, 'sensor_h': 4.7, 'focal': 4.49, 'img_w': 4000, 'img_h': 3000},
    'custom': {'label': 'Custom (enter sensor/focal manually)', 'sensor_w': 9.7, 'sensor_h': 7.3, 'focal': 6.88, 'img_w': 8064, 'img_h': 6048},
}

DEFAULT_MISSION_CONFIG = {
    'drone': 'mini4pro',
    'droneEnumValue': 68, 'droneSubEnumValue': 0,
    'camera': 'mini4pro_48', 'sensor_w': 9.7, 'sensor_h': 7.3, 'focal': 6.88, 'img_w': 8064, 'img_h': 6048,
    'batteryMinutes': 34, 'realisticFactor': BATTERY_REALISTIC_FACTOR_DEFAULT,
    'reserveFraction': BATTERY_RESERVE_FRACTION_DEFAULT,
    'altitude': 80, 'speed': 8,
    'forwardOverlap': 75, 'sideOverlap': 65, 'rotationDeg': 0,
    'sideSpacingOverride': 0, 'forwardSpacingOverride': 0, 'crosshatch': False,
    'threeDMapping': False, 'obliqueGimbal': -45,
    'corridorWidth': 40,
    'orbitRadius': 30, 'orbitPoints': 12, 'orbitClockwise': True,
    'orbitRings': 1, 'orbitMinAltitude': 0, 'orbitMaxAltitude': 0,
    'orbitTurnMode': 'toPointAndPassWithContinuityCurvature',
    'overviewEnabled': False, 'overviewAltitude': 0, 'overviewGimbal': -60,
    'delayAtWaypoint': 0,
    # Distance/speed assumes instant acceleration, which is badly wrong for
    # tightly-spaced stop-and-rotate waypoints where cruise speed is never
    # reached. 1.4 m/s^2 is the measured small-quadcopter average from Xu et
    # al., 2021 (MDPI Drones); ignoring it skews flight time by up to 1.7x.
    'droneAccel': 1.4,
    # DJI Fly caps a mission at 200 waypoints, but the RC2's own mission UI is
    # reported to destabilise from ~70+ on closely-packed mapping missions.
    # 90 stays under both; editable since tolerances vary by firmware.
    'maxWaypointsPerFile': 90,
    # WPML distance/time photo triggers aren't reliably available on consumer
    # DJI Fly, so grid/corridor rows fly continuously at a speed derived from
    # this interval (forward_spacing / cameraInterval) while the camera's own
    # Timer mode fires the shutter. That timer MUST be set manually pre-flight;
    # no WPML field can do it on consumer hardware. 2.0s matches HOT's
    # drone-flightplan and suits every DJI camera even shooting RAW.
    'cameraInterval': 2.0,
    # 'turnOnly' (default): sparse waypoints, camera on its own interval timer
    # (see cameraInterval). 'full': a waypoint per photo -- no manual step, but
    # jitter-prone and can hit RC2 waypoint limits. Both mirror YMapper/Waypoint OS.
    'waypointMode': 'turnOnly',
    'flyToWaylineMode': 'safely', 'finishAction': 'goHome',
    'exitOnRCLost': 'executeLostAction', 'executeRCLostAction': 'goBack',
    'takeOffSecurityHeight': 20, 'globalTransitionalSpeed': 10,
    # Straight-line stop at every waypoint, not a curved pass-through — matches what a
    # proven-working DJI Fly exporter uses. Curved turns can clip corners outside a
    # drawn survey area and blur photos taken mid-turn.
    'headingMode': 'followWayline', 'turnMode': 'toPointAndStopWithDiscontinuityCurvature',
    'heightMode': 'relativeToStartPoint', 'gimbalPitch': -90, 'gimbalPreset': 'nadir_2d',
}

# ── Mission generators ───────────────────────────────────────────────────────

def coverage_spacing(cfg):
    """Resolve the actual side/forward spacing (m) that will be used: an explicit
    override in meters if the user set one, otherwise derived from real sensor/focal
    geometry + altitude + overlap%. Exposed separately so the UI can show a live
    estimate before committing to a generation."""
    fw, fh = footprint(cfg['altitude'], cfg['sensor_w'], cfg['sensor_h'], cfg['focal'])
    side_spacing = cfg.get('sideSpacingOverride') or max(2.0, fw * (1 - cfg['sideOverlap'] / 100))
    forward_spacing = cfg.get('forwardSpacingOverride') or max(2.0, fh * (1 - cfg['forwardOverlap'] / 100))
    return max(1.0, side_spacing), max(1.0, forward_spacing)

def _point_in_polygon(x, y, poly):
    """Ray-casting point-in-polygon test (PNPOLY). Per-sample rather than
    per-edge-intersection so a scan line landing exactly on a vertex doesn't
    silently drop a whole row of coverage."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside

def _in_any_polygon(x, y, polys):
    return any(_point_in_polygon(x, y, p) for p in polys) if polys else False

def _scanline_intervals(y, poly):
    """Exact x-intervals where the horizontal line at `y` lies inside the
    polygon (even-odd rule) -- the classic scanline fill: collect every edge
    crossing, sort, and pair them up. Half-open (y1 > y) != (y2 > y) test so
    a line exactly through a vertex counts each edge once instead of twice,
    and horizontal edges are skipped consistently."""
    xs = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
    xs.sort()
    return [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2)]

def _subtract_intervals(intervals, holes):
    """Cut hole intervals out of coverage intervals -- how a no-fly zone
    splits a scan row into separate flyable stretches, exactly at the zone's
    true boundary rather than at the nearest sample point."""
    out = list(intervals)
    for h0, h1 in holes:
        nxt = []
        for a, b in out:
            if h1 <= a or h0 >= b:
                nxt.append((a, b))
                continue
            if h0 > a:
                nxt.append((a, h0))
            if h1 < b:
                nxt.append((h1, b))
        out = nxt
    return out

def _offset_polygon(poly, dist):
    """Dilate a simple polygon outward by `dist` with mitered corners -- a
    real polygon offset, not a scale about the centroid.

    Needed because the coverage margin has to apply evenly all the way round
    the shape. Scaling from the centroid would push a long thin site out
    mostly along its length and barely across its width, and clamping rows
    onto the polygon's extreme edge (the previous approach) is worse still:
    at the very tip of a rotated shape the scan line catches a near-zero
    slice, which the margin then inflated into a short stub row jutting out
    of the site -- the stray corner waypoints reported on a real 174x28 m
    survey area.
    """
    if dist == 0 or len(poly) < 3:
        return list(poly)
    pts = list(poly)
    n = len(pts)
    # Outward normals are only well defined for a known winding; force CCW.
    area2 = sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
                for i in range(n))
    if area2 < 0:
        pts.reverse()
    normals = []
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy) or 1.0
        normals.append((dy / L, -dx / L))  # outward for CCW winding
    out = []
    for i in range(n):
        px, py = normals[i - 1]   # edge arriving at vertex i
        nx, ny = normals[i]       # edge leaving vertex i
        mx, my = px + nx, py + ny
        ml = math.hypot(mx, my)
        if ml < 1e-9:            # spike vertex, edges exactly opposed
            out.append((pts[i][0] + nx * dist, pts[i][1] + ny * dist))
            continue
        mx, my = mx / ml, my / ml
        cos_half = mx * px + my * py
        # Cap the miter so a sharp corner bevels instead of shooting a long
        # spike out of the shape.
        out.append((pts[i][0] + mx * (dist / max(0.35, cos_half)),
                    pts[i][1] + my * (dist / max(0.35, cos_half))))
    return out

def _sweep_coverage_rows(rpts, side_spacing, forward_spacing, exclude_polys=None, boundary_margin=0.0):
    """Boustrophedon (lawnmower) sweep built from EXACT scanline intersections,
    not a sampled point-in-polygon lattice. For each row, the row's true
    x-intervals inside the polygon are computed directly (classic scanline
    fill), so every row starts and ends precisely on the boundary (extended by
    boundary_margin -- the boundary is where the surveyed site ends, not a
    fence: a photo footprint reaching slightly past the edge is free coverage,
    and clipping to the nearest interior sample point is what used to leave
    rows ragged/short near corners). Photos inside each interval are then
    spread evenly at <= forward_spacing.

    The previous lattice implementation only snapped each row's FAR end
    outward (and to the bounding box, not the polygon), so on any shape whose
    edges aren't parallel to the sweep, rows started up to a full
    forward_spacing short of the boundary. Scanline makes both ends exact and
    is also asymptotically cheaper (O(rows*edges) instead of
    O(rows*samples*edges)).

    Rows are distributed evenly across the span rather than stepped from one
    edge (which piled all the leftover on the far side and read as lopsided
    coverage). The boundary margin is applied by genuinely dilating the
    polygon first (_offset_polygon), so rows in the margin band intersect a
    real shape. Clamping them onto the original polygon's extreme edge
    instead -- the previous approach -- caught a near-zero-width slice at the
    tip of a rotated shape and inflated it into a stub row sticking out of
    the site.

    exclude_polys are cut out per-row as exact interval subtraction, so a
    no-fly zone splits a row exactly at the zone boundary. Returns a list of
    ROWS (each a list of (x, y) sample points); generate_grid keeps only each
    row's endpoints in Turn Only mode -- see its comment for why."""
    # Everything below sweeps the DILATED shape, so the margin is already
    # baked into the geometry and rows need no special casing at the edges.
    area = _offset_polygon(rpts, boundary_margin) if boundary_margin > 0 else rpts
    ys = [p[1] for p in area]
    miny, maxy = min(ys), max(ys)
    span_y = maxy - miny
    # Rows sit at the centres of equal bands, not on the span's extremes. On
    # any rotated site the outermost point is a corner, so a row placed exactly
    # there caught a zero-width slice and emitted a stray waypoint outside the
    # area. Band centres cover the same span with no degenerate rows, whatever
    # the orientation.
    n_rows = max(1, math.ceil(span_y / side_spacing)) if span_y > 0 else 1
    row_spacing = span_y / n_rows if span_y > 0 else side_spacing

    rows = []
    reverse = False
    for row_i in range(n_rows):
        y = miny + (row_i + 0.5) * row_spacing
        intervals = _scanline_intervals(y, area)
        if exclude_polys:
            holes = []
            for hp in exclude_polys:
                holes.extend(_scanline_intervals(y, hp))
            intervals = _subtract_intervals(intervals, holes)
        segments = []
        for a, b in intervals:
            length = b - a
            if length < 0.5:
                # Degenerate sliver (a polygon tip, or what's left beside a
                # hole) -- one photo at its middle instead of two coincident
                # endpoints pretending to be a flyable stretch.
                segments.append([((a + b) / 2, y)])
                continue
            n_steps = max(1, math.ceil(length / forward_spacing))
            step = length / n_steps
            segments.append([(a + k * step, y) for k in range(n_steps + 1)])
        if reverse:
            segments.reverse()
            for seg in segments:
                seg.reverse()
        if segments:
            rows.append(segments)
        reverse = not reverse
    return rows

def _decompose_into_cells(rows):
    """Boustrophedon CELLULAR decomposition.

    A no-fly zone (or a concave boundary) splits one scan row into several
    disconnected stretches. Flying them in plain row order -- end of stretch A
    straight to start of stretch B -- sends the aircraft directly across the
    gap between them, which is exactly the zone it was supposed to avoid.
    Confirmed by direct test before this existed: an L-shaped area with one
    no-fly box produced 3 legs passing right through the box.

    The standard fix (and what real coverage-path planners do) is to treat the
    free space as separate CELLS: link each row's stretch to the stretch it
    overlaps in the previous row, and fly each connected cell as its own
    serpentine. Where a row's stretch splits in two, or two merge into one,
    that's a critical point -- the affected cells are closed and new ones
    opened, so no path ever spans a gap.

    Takes rows (list of rows; each a list of segments; each segment a list of
    same-y points) and returns a list of cells, each a list of segments in
    flight order.
    """
    def extent(seg):
        xs = [p[0] for p in seg]
        return min(xs), max(xs)

    def overlaps(a, b):
        # Touching-only counts as disconnected: two stretches that merely meet
        # at a hole's edge are on opposite sides of it.
        return a[0] < b[1] - 1e-9 and b[0] < a[1] - 1e-9

    open_cells = []   # each: {'segments': [...], 'extent': (lo, hi)}
    finished = []
    for row in rows:
        exts = [extent(s) for s in row]
        # Match this row's segments against the currently open cells.
        seg_to_cells = [[ci for ci, c in enumerate(open_cells) if overlaps(exts[si], c['extent'])]
                        for si in range(len(row))]
        cell_to_segs = [[si for si in range(len(row)) if ci in seg_to_cells[si]]
                        for ci in range(len(open_cells))]
        next_open = []
        used = set()
        for si, seg in enumerate(row):
            cands = seg_to_cells[si]
            # Continue an existing cell only on a clean 1:1 continuation --
            # a split (one cell feeding several segments) or a merge (several
            # cells feeding one segment) is a critical point, so those cells
            # are closed and fresh ones start here.
            if len(cands) == 1 and len(cell_to_segs[cands[0]]) == 1:
                cell = open_cells[cands[0]]
                cell['segments'].append(seg)
                cell['extent'] = exts[si]
                next_open.append(cell)
                used.add(cands[0])
            else:
                next_open.append({'segments': [seg], 'extent': exts[si]})
                for ci in cands:
                    if ci not in used:
                        finished.append(open_cells[ci])
                        used.add(ci)
        # Any open cell this row didn't touch at all has ended.
        for ci, c in enumerate(open_cells):
            if ci not in used and c not in next_open:
                finished.append(c)
        open_cells = next_open
    finished.extend(open_cells)

    # Serpentine within each cell: alternate direction row to row so the
    # aircraft turns at the end of each pass instead of deadheading back.
    cells = []
    for c in finished:
        segs = []
        for i, seg in enumerate(c['segments']):
            s = list(seg)
            # Normalise to a known direction first, then alternate -- the raw
            # segments already carry the sweep's own global alternation, which
            # doesn't survive being regrouped into cells.
            if s[0][0] > s[-1][0]:
                s.reverse()
            if i % 2 == 1:
                s.reverse()
            segs.append(s)
        if segs:
            cells.append(segs)
    return _order_cells(cells)

def _reverse_cell(cell):
    """Fly the same cell from its other end: rows bottom-to-top instead of
    top-to-bottom, each row's direction flipped to keep the serpentine."""
    return [list(reversed(seg)) for seg in reversed(cell)]

def _order_cells(cells):
    """Greedy nearest-neighbour ordering, picking each cell's traversal
    direction too.

    Cells come out of the decomposition in the order the sweep happened to
    create them, so the aircraft could finish one cell and then deadhead
    right across the site to start the next -- clearly visible as long
    diagonals in a rendered path. Choosing the nearest remaining cell, and
    whichever of its two ends is closer, keeps those transits short. Greedy
    rather than optimal: this is a small open-TSP instance, and the exact
    answer isn't worth the runtime for a handful of cells.
    """
    if len(cells) <= 1:
        return cells
    remaining = list(cells)
    # Start from the cell containing the lowest row, so the mission still
    # begins at a predictable edge of the site rather than somewhere random.
    start = min(remaining, key=lambda c: (c[0][0][1], c[0][0][0]))
    remaining.remove(start)
    ordered = [start]
    cur = start[-1][-1]
    while remaining:
        best = None
        for c in remaining:
            for cand in (c, _reverse_cell(c)):
                d = math.hypot(cand[0][0][0] - cur[0], cand[0][0][1] - cur[1])
                if best is None or d < best[0]:
                    best = (d, c, cand)
        _, original, chosen = best
        remaining.remove(original)
        ordered.append(chosen)
        cur = chosen[-1][-1]
    return ordered

def _segments_cross(a, b, c, d):
    """Proper segment intersection (shared endpoints / touching don't count)."""
    def orient(p, q, r):
        v = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        return 0 if abs(v) < 1e-12 else (1 if v > 0 else -1)
    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    return 0 not in (o1, o2, o3, o4) and o1 != o2 and o3 != o4

def _seg_enters_poly(a, b, poly):
    """True if the straight leg a->b passes through the polygon's interior."""
    if _point_in_polygon(a[0], a[1], poly) or _point_in_polygon(b[0], b[1], poly):
        return True
    n = len(poly)
    return any(_segments_cross(a, b, poly[i], poly[(i + 1) % n]) for i in range(n))

def _expand_poly(poly, margin):
    """Push every vertex outward from the centroid so the routed path clears
    the real zone by a margin instead of grazing its exact edge. A negative
    margin shrinks instead, which is how the interior-only test polygon is
    built (see _route_around_exclusions)."""
    if margin == 0 or len(poly) < 3:
        return list(poly)
    cx = sum(p[0] for p in poly) / len(poly)
    cy = sum(p[1] for p in poly) / len(poly)
    out = []
    for x, y in poly:
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy) or 1.0
        out.append((x + dx / d * margin, y + dy / d * margin))
    return out

def _detour_around(a, b, hull):
    """Vertices to route through so a->b goes AROUND a convex hull instead of
    across it. Both ways round are built (the hull vertices on either side of
    the a-b line, ordered along it) and the shorter one wins."""
    ax, ay = a
    ex, ey = b[0] - ax, b[1] - ay
    length = math.hypot(ex, ey) or 1.0
    def side(p):
        return ex * (p[1] - ay) - ey * (p[0] - ax)
    def along(p):
        return ((p[0] - ax) * ex + (p[1] - ay) * ey) / length
    best = None
    for group in ([v for v in hull if side(v) > 0], [v for v in hull if side(v) < 0]):
        if not group:
            continue
        ordered = sorted(group, key=along)
        path = [a] + ordered + [b]
        dist = sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                   for i in range(len(path) - 1))
        if best is None or dist < best[0]:
            best = (dist, ordered)
    return list(best[1]) if best else []

def _route_around_exclusions(points, exclude_polys, margin):
    """Insert transit waypoints so no straight leg ever crosses a no-fly zone.

    Splitting rows at zone boundaries is not enough on its own: the aircraft
    still flies from the end of one stretch to the start of the next, and from
    the last row of one cell to the first row of the next -- both of which can
    cut straight across the zone. Verified before this existed: five different
    area/zone layouts produced 2-7 zone-crossing legs each, even with cellular
    decomposition already in place.

    Each offending leg is re-routed around the offending zone's expanded
    convex hull. A hull is used rather than the raw polygon because it is
    guaranteed to contain the zone (so clearing the hull clears the zone) and
    is convex, which makes "go around one side" well defined.

    points: list of (x, y). Returns list of (x, y, is_detour), where
    is_detour marks inserted transit points that must not take a photo --
    they sit outside the surveyed area by construction.
    """
    if not exclude_polys or len(points) < 2:
        return [(p[0], p[1], False) for p in points]
    # Two polygons per zone, and the sizes matter:
    #  * TEST is the hull shrunk slightly, i.e. the zone's interior. It must
    #    NOT be expanded: rows legitimately end exactly ON the boundary, and
    #    such an endpoint would sit inside an expanded hull, so every leg from
    #    there reports a hit no re-routing can clear (42 endpoints ballooned to
    #    350 waypoints). Shrinking keeps boundary endpoints outside while still
    #    catching any leg that truly cuts through.
    #  * ROUTE is expanded by the full margin so transit points clear the zone.
    zones = []
    for p in exclude_polys:
        base = _convex_hull(list(p))
        if len(base) < 3:
            continue
        zones.append((_expand_poly(base, -0.05), _expand_poly(base, margin)))
    if not zones:
        return [(p[0], p[1], False) for p in points]

    def first_hit(p, q):
        for test_hull, route_hull in zones:
            if _seg_enters_poly(p, q, test_hull):
                return route_hull
        return None

    out = [(points[0][0], points[0][1], False)]
    for i in range(1, len(points)):
        leg = [(out[-1][0], out[-1][1]), points[i]]
        # A detour can itself clip a different zone, so re-check until clear.
        # The cap stops a pathological layout from looping forever; if it is
        # hit the leg is emitted as-is rather than silently dropping coverage.
        for _ in range(8):
            changed = False
            rebuilt = [leg[0]]
            for j in range(1, len(leg)):
                p, q = leg[j - 1], leg[j]
                route_hull = first_hit(p, q)
                if route_hull:
                    detour = _detour_around(p, q, route_hull)
                    if detour:
                        rebuilt.extend(detour)
                        changed = True
                rebuilt.append(q)
            leg = rebuilt
            if not changed:
                break
        for p in leg[1:-1]:
            # Skip a detour point that coincides with what's already there --
            # routing two consecutive legs around the same corner otherwise
            # emits the same vertex twice, which is a zero-length flight leg.
            if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) < 0.05:
                continue
            out.append((p[0], p[1], True))
        out.append((points[i][0], points[i][1], False))
    return out

def _sweep_coverage(rpts, side_spacing, forward_spacing, exclude_polys=None, boundary_margin=0.0):
    """Flat dense sample list -- the real per-photo positions, used for photo-
    count/area/interval estimates (and by the JS live-estimate mirror). The
    actual flight path generate_grid builds does NOT fly to each of these; see
    _sweep_coverage_rows."""
    rows = _sweep_coverage_rows(rpts, side_spacing, forward_spacing, exclude_polys, boundary_margin)
    return [pt for row in rows for seg in row for pt in seg]

def _polyline_offset(pts, dist):
    """Offset an open polyline sideways by `dist` (positive = left of travel).

    Offsetting each SAMPLE by its own segment's perpendicular -- what the
    corridor used to do -- breaks down at every bend: the samples either side
    of a vertex use different headings, so the offset path steps sideways
    there, leaving notches and outward spikes exactly at the route's corners.
    Offsetting the polyline itself with miter joins keeps the passes cleanly
    parallel. The miter is capped so a hairpin bend can't fling a pass far
    away from the route.
    """
    n = len(pts)
    if n < 2 or dist == 0:
        return list(pts)
    normals = []
    for i in range(n - 1):
        dx, dy = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        L = math.hypot(dx, dy) or 1.0
        normals.append((-dy / L, dx / L))
    out = []
    for i, pt in enumerate(pts):
        if i == 0:
            nx, ny = normals[0]
            out.append((pt[0] + nx * dist, pt[1] + ny * dist))
        elif i == n - 1:
            nx, ny = normals[-1]
            out.append((pt[0] + nx * dist, pt[1] + ny * dist))
        else:
            (ax, ay), (bx, by) = normals[i - 1], normals[i]
            mx, my = ax + bx, ay + by
            ml = math.hypot(mx, my)
            if ml < 1e-9:  # route doubles straight back on itself
                out.append((pt[0] + ax * dist, pt[1] + ay * dist))
                continue
            mx, my = mx / ml, my / ml
            cos_half = mx * ax + my * ay
            # Cap the miter at 2x the offset distance. Uncapped, a sharp bend
            # sends the outer pass's corner point far off the route (the
            # miter length is dist/cos(half-angle), which runs away as the
            # turn tightens); clamping bevels the corner instead, which is
            # what a flight path wants.
            miter = dist / max(0.5, cos_half)
            out.append((pt[0] + mx * miter, pt[1] + my * miter))
    return out

def _polyline_extend(pts, amount):
    """Push both ends outward along their own direction so a pass overshoots
    the route's end instead of stopping exactly on it -- same reasoning as the
    grid's boundary margin: a photo slightly past the end is free coverage,
    while stopping short leaves the end of the corridor thin."""
    if amount <= 0 or len(pts) < 2:
        return list(pts)
    out = list(pts)
    dx, dy = out[0][0] - out[1][0], out[0][1] - out[1][1]
    L = math.hypot(dx, dy) or 1.0
    out[0] = (out[0][0] + dx / L * amount, out[0][1] + dy / L * amount)
    dx, dy = out[-1][0] - out[-2][0], out[-1][1] - out[-2][1]
    L = math.hypot(dx, dy) or 1.0
    out[-1] = (out[-1][0] + dx / L * amount, out[-1][1] + dy / L * amount)
    return out

def _turn_points(pts, tol_deg=1.0):
    """Reduce a dense run of photo positions to just the points the aircraft
    actually has to steer at: the two ends plus every genuine corner.

    Turn Only mode flies each stretch as one continuous line between
    waypoints, so it only needs the turns -- but taking the first and last
    point ALONE is only correct when the stretch is straight. A grid row
    always is; a corridor pass is not: it follows the route's bends, so
    reducing it to two endpoints made the aircraft fly straight from the
    start of the route to its end, cutting every corner and abandoning the
    corridor completely. Keeping direction changes fixes that, and still
    yields exactly two points for a straight stretch.
    """
    if len(pts) < 3:
        return list(pts)
    tol = math.radians(tol_deg)
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        ax, ay = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
        bx, by = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        if math.hypot(ax, ay) < 1e-9 or math.hypot(bx, by) < 1e-9:
            continue
        turn = abs(math.atan2(ax * by - ay * bx, ax * bx + ay * by))
        if turn > tol:
            out.append(pts[i])
    out.append(pts[-1])
    return out

def _polyline_length(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))

def _polyline_samples(pts, spacing):
    """Evenly spaced points along a polyline (step <= spacing, both ends
    included)."""
    seglens = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1)]
    total = sum(seglens)
    if total <= 0:
        return [tuple(pts[0])]
    n = max(1, math.ceil(total / spacing))
    out, seg_i, acc = [], 0, 0.0
    for s in range(n + 1):
        target = total * s / n
        while seg_i < len(seglens) - 1 and acc + seglens[seg_i] < target:
            acc += seglens[seg_i]
            seg_i += 1
        L = seglens[seg_i] or 1e-9
        t = min(max((target - acc) / L, 0.0), 1.0)
        x1, y1 = pts[seg_i]
        x2, y2 = pts[seg_i + 1]
        out.append((x1 + (x2 - x1) * t, y1 + (y2 - y1) * t))
    return out

def _project_exclusions(exclusions_latlon, ref_lat, ref_lon, cf, sf):
    """Exclusion polygons come in as plain lat/lon like the boundary — reproject
    them into the same rotated local-xy frame the boundary/sweep use, so a
    single point-in-polygon check works for both."""
    out = []
    for excl in (exclusions_latlon or []):
        if len(excl) < 3:
            continue
        exy = [to_xy(p[0], p[1], ref_lat, ref_lon) for p in excl]
        out.append([(x * cf - y * sf, x * sf + y * cf) for x, y in exy])
    return out

def _grid_rows_for(polygon_latlon, cfg, exclusions_latlon=None):
    """The grid sweep's rows for a polygon, in its own rotated frame -- shared
    by grid_photo_points and the pass-count estimate so neither re-derives the
    projection/rotation setup independently."""
    ref_lat = sum(p[0] for p in polygon_latlon) / len(polygon_latlon)
    ref_lon = sum(p[1] for p in polygon_latlon) / len(polygon_latlon)
    pts_xy = [to_xy(p[0], p[1], ref_lat, ref_lon) for p in polygon_latlon]
    side_spacing, forward_spacing = coverage_spacing(cfg)
    rot = math.radians(cfg.get('rotationDeg', 0) or 0)
    cf, sf = math.cos(-rot), math.sin(-rot)
    rpts = [(x * cf - y * sf, x * sf + y * cf) for x, y in pts_xy]
    exclude_rpolys = _project_exclusions(exclusions_latlon, ref_lat, ref_lon, cf, sf)
    margin = side_spacing / 2.0
    rows = _sweep_coverage_rows(rpts, side_spacing, forward_spacing, exclude_rpolys, margin)
    return rows, rpts, exclude_rpolys, side_spacing, forward_spacing, margin

def _grid_row_count(polygon_latlon, cfg, exclusions_latlon=None):
    if len(polygon_latlon) < 3:
        return 0
    rows, _, _, _, _, _ = _grid_rows_for(polygon_latlon, cfg, exclusions_latlon)
    return len(rows)

def grid_photo_points(polygon_latlon, cfg, exclusions_latlon=None):
    """Dense per-photo sample points for a grid area -- the same geometry
    generate_grid uses to lay out its sparse row-turn flight path, flattened
    here for counting/estimating instead. Used for the photo-count estimate
    and for excluded_count (how many shots a no-fly zone would actually
    remove), since generate_grid's own output is no longer one entry per
    photo -- see its comment for why."""
    if len(polygon_latlon) < 3:
        return []
    rows, rpts, exclude_rpolys, side_spacing, forward_spacing, margin = _grid_rows_for(
        polygon_latlon, cfg, exclusions_latlon)
    pts = [pt for row in rows for seg in row for pt in seg]
    if cfg.get('crosshatch'):
        transposed = [(y, x) for x, y in rpts]
        transposed_excl = [[(y, x) for x, y in poly] for poly in exclude_rpolys]
        pts += _sweep_coverage(transposed, side_spacing, forward_spacing, transposed_excl, margin)
    return pts

def generate_grid(polygon_latlon, cfg, exclusions_latlon=None):
    if len(polygon_latlon) < 3:
        raise ValueError('A grid area needs at least 3 points')
    ref_lat = sum(p[0] for p in polygon_latlon) / len(polygon_latlon)
    ref_lon = sum(p[1] for p in polygon_latlon) / len(polygon_latlon)
    pts_xy = [to_xy(p[0], p[1], ref_lat, ref_lon) for p in polygon_latlon]

    side_spacing, forward_spacing = coverage_spacing(cfg)

    rot = math.radians(cfg.get('rotationDeg', 0) or 0)
    cf, sf = math.cos(-rot), math.sin(-rot)
    ci, si = math.cos(rot), math.sin(rot)
    rpts = [(x * cf - y * sf, x * sf + y * cf) for x, y in pts_xy]
    exclude_rpolys = _project_exclusions(exclusions_latlon, ref_lat, ref_lon, cf, sf)

    # The boundary marks where the site ends, not a fence: coverage may reach
    # half a row's spacing past it, which is free extra coverage. Treating the
    # edge as strict is what produced short, ragged rows near the corners.
    margin = side_spacing / 2.0
    rows = _sweep_coverage_rows(rpts, side_spacing, forward_spacing, exclude_rpolys, margin)
    if cfg.get('crosshatch'):
        # A second sweep at 90° catches gaps the first sweep's direction misses,
        # especially on concave/irregular site boundaries — appended as a second
        # pass rather than interleaved, so it always runs after the main grid.
        transposed = [(y, x) for x, y in rpts]
        transposed_excl = [[(y, x) for x, y in poly] for poly in exclude_rpolys]
        rows2 = _sweep_coverage_rows(transposed, side_spacing, forward_spacing, transposed_excl, margin)
        rows += [[[(x, y) for y, x in seg] for seg in row] for row in rows2]

    if not rows:
        raise ValueError('No coverage generated — the area may be too small for the current spacing/altitude')

    # Two waypoint modes, matching what YMapper and Waypoint OS both settled on.
    # "Full" stops at every photo: no manual pre-flight step, but jitter-prone
    # on a dense grid. "turnOnly" is the default -- see its branch below.
    # Both route the finished path around no-fly zones: splitting rows at a zone
    # boundary keeps photos out of it but says nothing about the legs BETWEEN
    # stretches, which is where the aircraft actually crossed. See
    # _route_around_exclusions.
    detour_margin = max(2.0, forward_spacing * 0.25)

    if cfg.get('waypointMode') == 'full':
        ordered = [pt for cell in _decompose_into_cells(rows) for seg in cell for pt in seg]
        waypoints = []
        for x, y, is_detour in _route_around_exclusions(ordered, exclude_rpolys, detour_margin):
            lx, ly = x * ci - y * si, x * si + y * ci
            lat, lon = from_xy(lx, ly, ref_lat, ref_lon)
            waypoints.append({'lat': lat, 'lon': lon, 'alt': cfg['altitude'], 'speed': cfg['speed'],
                               'gimbal': cfg.get('gimbalPitch', -90), 'heading_mode': 'followWayline',
                               # A detour point sits outside the surveyed area
                               # by construction, so it transits rather than
                               # shooting -- and doesn't pause either.
                               'photo': not is_detour,
                               'hover': 0 if is_detour else cfg.get('delayAtWaypoint', 0)})
        return waypoints

    # "Turn only" (default): a stop at every photo caused position-hold jitter
    # and RC2 mission-count instability in flight testing, and consumer DJI Fly
    # has no continuous-flight interval trigger to encode. So each row flies as
    # one continuous line and the camera's own interval timer (set manually
    # pre-flight) fires the shutter; cruise speed is derived FROM that interval
    # so photos land every forward_spacing metres. Same approach as HOT's
    # drone-flightplan. See cameraInterval in DEFAULT_MISSION_CONFIG.
    camera_interval = cfg.get('cameraInterval') or 2.0
    row_speed = max(0.5, forward_spacing / camera_interval)

    # Segment boundaries come from exact interval arithmetic, so a zone of any
    # width splits the row properly. Grouping them into connected cells makes
    # the aircraft finish one side of a zone before starting the other.
    ordered = []
    for cell in _decompose_into_cells(rows):
        for seg in cell:
            # A grid row is straight, so this yields the same two endpoints it
            # always did -- shared with the corridor so both mission types
            # follow one rule for what counts as a turn.
            ordered.extend(_turn_points(seg))
    waypoints = []
    for x, y, _is_detour in _route_around_exclusions(ordered, exclude_rpolys, detour_margin):
        lx, ly = x * ci - y * si, x * si + y * ci
        lat, lon = from_xy(lx, ly, ref_lat, ref_lon)
        waypoints.append({'lat': lat, 'lon': lon, 'alt': cfg['altitude'], 'speed': row_speed,
                           'gimbal': cfg.get('gimbalPitch', -90), 'heading_mode': 'followWayline',
                           'photo': False, 'hover': 0})
    return waypoints

def generate_3d_mapping(polygon_latlon, cfg, exclusions_latlon=None):
    """Nadir + oblique double-grid for full 3D reconstruction (DJI Terra/Pix4D
    method): a straight-down pass for top surfaces plus a second, 90°-rotated
    pass at an oblique angle so facades actually get imaged too."""
    nadir_cfg = dict(cfg)
    nadir_cfg['gimbalPitch'] = -90
    nadir_cfg['crosshatch'] = False
    nadir_pts = generate_grid(polygon_latlon, nadir_cfg, exclusions_latlon)

    oblique_cfg = dict(cfg)
    oblique_cfg['gimbalPitch'] = cfg.get('obliqueGimbal', -45)
    oblique_cfg['rotationDeg'] = (cfg.get('rotationDeg', 0) + 90) % 360
    oblique_cfg['crosshatch'] = False
    oblique_pts = generate_grid(polygon_latlon, oblique_cfg, exclusions_latlon)

    return nadir_pts + oblique_pts

def mission_photo_points(polygon_latlon, cfg, exclusions_latlon=None):
    """grid_photo_points, but also handles the 3D-mapping (nadir+oblique
    double-grid) case the same way generate_3d_mapping itself splits cfg."""
    if not cfg.get('threeDMapping'):
        return grid_photo_points(polygon_latlon, cfg, exclusions_latlon)
    nadir_cfg = dict(cfg)
    nadir_cfg['gimbalPitch'] = -90
    nadir_cfg['crosshatch'] = False
    oblique_cfg = dict(cfg)
    oblique_cfg['rotationDeg'] = (cfg.get('rotationDeg', 0) + 90) % 360
    oblique_cfg['crosshatch'] = False
    return (grid_photo_points(polygon_latlon, nadir_cfg, exclusions_latlon)
            + grid_photo_points(polygon_latlon, oblique_cfg, exclusions_latlon))

def corridor_photo_points(line_latlon, cfg, exclusions_latlon=None):
    """Per-photo sample points for a corridor, as a list of PASSES, each a
    list of flyable SEGMENTS -- the same shape _sweep_coverage_rows returns
    for a grid, so both mission types are consumed the same way.

    Each pass is a properly offset copy of the route polyline (see
    _polyline_offset for why per-sample perpendiculars were wrong), extended
    past both ends, then sampled at the photo spacing.
    """
    if len(line_latlon) < 2:
        return [], (0, 0), 1.0
    ref_lat, ref_lon = line_latlon[0][0], line_latlon[0][1]
    pts_xy = [to_xy(p[0], p[1], ref_lat, ref_lon) for p in line_latlon]
    # cf=1, sf=0 is an identity rotation -- corridor coverage doesn't rotate
    # into a scan frame the way grid does, so exclusions just need projecting
    # into the same plain local-xy frame the route itself uses.
    exclude_xypolys = _project_exclusions(exclusions_latlon, ref_lat, ref_lon, 1.0, 0.0)

    if _polyline_length(pts_xy) <= 0:
        raise ValueError('Route has zero length')

    side_spacing, forward_spacing = coverage_spacing(cfg)
    width = max(0.0, cfg.get('corridorWidth', 0))

    # Same margin philosophy as the grid: the corridor width marks where the
    # surveyed strip ends, not a fence the aircraft must stay inside of.
    # Covering slightly past it is free extra coverage, whereas putting the
    # outermost pass exactly on the edge leaves that edge with only half a
    # photo footprint over it.
    margin = side_spacing / 2.0
    half = width / 2.0 + margin
    span = 2.0 * half
    n_passes = 1 if width <= 0 else max(2, math.ceil(span / side_spacing) + 1)
    offsets = [0.0] if n_passes == 1 else [-half + i * (span / (n_passes - 1))
                                           for i in range(n_passes)]

    passes = []
    for pass_i, off in enumerate(offsets):
        line = _polyline_extend(_polyline_offset(pts_xy, off), margin)
        samples = _polyline_samples(line, forward_spacing)
        if pass_i % 2 == 1:
            samples.reverse()
        # Split at the exclusion zones directly rather than dropping points
        # and inferring the gap from spacing afterwards -- that heuristic
        # missed any zone narrower than 1.5x the photo spacing and let a leg
        # fly straight through it.
        segs, cur = [], []
        for pt in samples:
            if _in_any_polygon(pt[0], pt[1], exclude_xypolys):
                if cur:
                    segs.append(cur)
                    cur = []
            else:
                cur.append(pt)
        if cur:
            segs.append(cur)
        if segs:
            passes.append(segs)
    return passes, (ref_lat, ref_lon), forward_spacing

def generate_corridor(line_latlon, cfg, exclusions_latlon=None):
    if len(line_latlon) < 2:
        raise ValueError('A corridor route needs at least 2 points')
    passes, (ref_lat, ref_lon), forward_spacing = corridor_photo_points(
        line_latlon, cfg, exclusions_latlon)
    if not any(passes):
        raise ValueError('No coverage generated - the route may be too short, or fully inside a no-fly zone')

    # Corridors get the same no-fly-zone routing as grids: splitting a pass at
    # the zone boundary keeps photos out of it, but says nothing about the
    # straight leg between one stretch and the next, which is where the
    # aircraft actually crossed one. See _route_around_exclusions.
    exclude_xypolys = _project_exclusions(exclusions_latlon, ref_lat, ref_lon, 1.0, 0.0)
    detour_margin = max(2.0, forward_spacing * 0.25)

    # Same Full/Turn Only choice as generate_grid -- see its comment.
    if cfg.get('waypointMode') == 'full':
        ordered = [pt for pas in passes for seg in pas for pt in seg]
        waypoints = []
        for ox, oy, is_detour in _route_around_exclusions(ordered, exclude_xypolys, detour_margin):
            lat, lon = from_xy(ox, oy, ref_lat, ref_lon)
            waypoints.append({'lat': lat, 'lon': lon, 'alt': cfg['altitude'], 'speed': cfg['speed'],
                               'gimbal': cfg.get('gimbalPitch', -90), 'heading_mode': 'followWayline',
                               'photo': not is_detour,
                               'hover': 0 if is_detour else cfg.get('delayAtWaypoint', 0)})
        return waypoints

    # Same reasoning as generate_grid: each pass flies as one continuous line
    # at a speed derived from the camera's interval timer, rather than stopping
    # per photo. See cameraInterval in DEFAULT_MISSION_CONFIG.
    camera_interval = cfg.get('cameraInterval') or 2.0
    row_speed = max(0.5, forward_spacing / camera_interval)

    ordered = []
    for pas in passes:
        for seg in pas:
            ordered.extend(_turn_points(seg))
    waypoints = []
    for ox, oy, _is_detour in _route_around_exclusions(ordered, exclude_xypolys, detour_margin):
        lat, lon = from_xy(ox, oy, ref_lat, ref_lon)
        waypoints.append({'lat': lat, 'lon': lon, 'alt': cfg['altitude'], 'speed': row_speed,
                           'gimbal': cfg.get('gimbalPitch', -90), 'heading_mode': 'followWayline',
                           'photo': False, 'hover': 0})
    return waypoints

def generate_orbit(center_lat, center_lon, cfg):
    """Single ring, or stacked rings at different altitudes when orbitRings>1 —
    recommended for 3D reconstruction of a tall/complex object (tower, silo,
    monument), since one ring only sees it from a single elevation angle.
    Aim for >=30 photos per ring."""
    radius = cfg['orbitRadius']
    num_points = max(3, int(cfg.get('orbitPoints', 12)))
    clockwise = cfg.get('orbitClockwise', True)
    rings = max(1, int(cfg.get('orbitRings', 1)))

    if rings == 1:
        altitudes = [cfg['altitude']]
    else:
        lo = cfg.get('orbitMinAltitude') or cfg['altitude'] * 0.5
        hi = cfg.get('orbitMaxAltitude') or cfg['altitude']
        altitudes = [lo + (hi - lo) * i / (rings - 1) for i in range(rings)]

    waypoints = []
    for ring_i, altitude in enumerate(altitudes):
        ring_clockwise = clockwise if ring_i % 2 == 0 else not clockwise  # alternate direction, ring to ring
        for i in range(num_points):
            frac = i / num_points
            angle = frac * 360 if ring_clockwise else 360 - frac * 360
            lat, lon = destination_point(center_lat, center_lon, radius, angle)
            hdg = bearing_deg(lat, lon, center_lat, center_lon)
            hdg = hdg - 360 if hdg > 180 else hdg
            pitch = -math.degrees(math.atan2(altitude, radius))
            # Per DJI's WPML spec 'fixed' means "hold the current heading",
            # NOT "point at this angle" -- 'smoothTransition' is the mode that
            # actually reads waypointHeadingAngle, which is what facing the
            # orbit centre needs.
            waypoints.append({'lat': lat, 'lon': lon, 'alt': altitude, 'speed': cfg.get('speed', 5),
                               'gimbal': round(pitch), 'heading_mode': 'smoothTransition', 'heading_angle': round(hdg),
                               'photo': cfg.get('photo', True), 'hover': cfg.get('delayAtWaypoint', 0),
                               # A circular path made of stop-and-rotate segments (the grid/corridor
                               # default) looks like a stuttering polygon, not an orbit — DJI Fly's
                               # own continuity-curvature turn mode is a centripetal Catmull-Rom spline
                               # through the waypoints, which is what actually flies a smooth circle.
                               'turn_mode': cfg.get('orbitTurnMode', 'toPointAndPassWithContinuityCurvature')})
    return waypoints

def generate_overview(polygon_latlon, cfg, exclusions_latlon=None):
    """A single higher-altitude lap around the site boundary with a photo at every
    corner plus mid-edge points — a quick 'whole site in context' pass, meant to be
    flown in addition to a detailed grid, not instead of it."""
    if len(polygon_latlon) < 3:
        raise ValueError('An overview lap needs at least 3 points')
    alt = cfg.get('overviewAltitude') or (cfg['altitude'] * 1.5)
    speed = cfg.get('speed', 8)
    ref_lat, ref_lon = polygon_latlon[0][0], polygon_latlon[0][1]
    exclude_xypolys = _project_exclusions(exclusions_latlon, ref_lat, ref_lon, 1.0, 0.0)
    waypoints = []
    n = len(polygon_latlon)
    for i in range(n):
        a = polygon_latlon[i]
        b = polygon_latlon[(i + 1) % n]
        ax, ay = to_xy(a[0], a[1], ref_lat, ref_lon)
        if not _in_any_polygon(ax, ay, exclude_xypolys):
            waypoints.append({'lat': a[0], 'lon': a[1], 'alt': alt, 'speed': speed,
                               'gimbal': cfg.get('overviewGimbal', -60), 'heading_mode': 'followWayline',
                               'photo': True, 'hover': 0})
        mid_lat, mid_lon = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        mx, my = to_xy(mid_lat, mid_lon, ref_lat, ref_lon)
        if not _in_any_polygon(mx, my, exclude_xypolys):
            waypoints.append({'lat': mid_lat, 'lon': mid_lon, 'alt': alt, 'speed': speed,
                               'gimbal': cfg.get('overviewGimbal', -60), 'heading_mode': 'followWayline',
                               'photo': True, 'hover': 0})
    return waypoints

def estimate_coverage(polygon_latlon, cfg):
    """Exact photo-count estimate (runs the real sweep, same as generate_grid) plus
    bonus stats mirrored from YMapper's own info panel: polygon area, recommended
    shutter speed to avoid motion blur, and the time interval between photos along
    a pass — useful for sanity-checking a mission before committing to it."""
    if len(polygon_latlon) < 3:
        return {'side_spacing': 0, 'forward_spacing': 0, 'passes': 0, 'estimated_photos': 0,
                'area_m2': 0, 'shutter_speed': None, 'forward_interval_s': 0}
    side_spacing, forward_spacing = coverage_spacing(cfg)
    pts = grid_photo_points(polygon_latlon, cfg)
    # Pass count comes from the real sweep, not sqrt(area)/spacing -- that
    # approximation assumes a square footprint and ignores rotation entirely,
    # so it was well off for long/thin or rotated areas (exactly the shapes
    # where knowing the pass count matters most).
    passes = max(1, _grid_row_count(polygon_latlon, cfg))
    shutter = recommended_shutter_speed(cfg['altitude'], cfg['sensor_w'], cfg['focal'], cfg['img_w'], cfg['speed'])
    interval = forward_spacing / cfg['speed'] if cfg.get('speed') else 0
    return {'side_spacing': round(side_spacing, 1), 'forward_spacing': round(forward_spacing, 1),
            'passes': passes, 'estimated_photos': len(pts),
            'area_m2': round(polygon_area_m2(polygon_latlon)), 'shutter_speed': shutter,
            'forward_interval_s': round(interval, 1)}

# ── Battery-based mission splitting ──────────────────────────────────────────

def usable_battery_seconds(cfg):
    """Rated flight time -> safe usable-per-battery mission time budget. See the
    DRONE_PRESETS comment above for the sourcing on both factors."""
    rated_min = cfg.get('batteryMinutes') or 20
    realistic = cfg.get('realisticFactor', BATTERY_REALISTIC_FACTOR_DEFAULT)
    reserve = cfg.get('reserveFraction', BATTERY_RESERVE_FRACTION_DEFAULT)
    return max(60.0, rated_min * 60.0 * realistic * (1 - reserve))

def _is_stop_turn(mode):
    # Every turn mode except the smooth-flythrough one brings the aircraft to a
    # stop at the waypoint -- see build_waylines_wpml's turn-mode comments.
    return mode != 'toPointAndPassWithContinuityCurvature'

def leg_time_sec(dist_m, cruise_speed, accel, must_stop):
    """Time to cover one leg, modeling acceleration/deceleration instead of
    assuming instant cruise speed. When the aircraft stops at either end
    (must_stop), a leg shorter than the distance needed to reach cruise speed
    never actually gets there -- that's the triangular-profile branch, and it's
    the common case for a tightly-spaced grid survey in the default stop-and-
    rotate turn mode. See droneAccel's comment in DEFAULT_MISSION_CONFIG for
    where the default acceleration figure comes from."""
    if cruise_speed <= 0:
        return 0.0
    if not must_stop:
        return dist_m / cruise_speed
    accel = accel or 1.4
    d_half = cruise_speed ** 2 / (2 * accel)
    if dist_m >= 2 * d_half:
        return 2 * (cruise_speed / accel) + (dist_m - 2 * d_half) / cruise_speed
    return 2 * math.sqrt(dist_m / accel)

def split_mission_by_battery(waypoints, cfg):
    """Greedily group waypoints into standalone sub-missions, breaking a batch
    whenever EITHER limit is hit first: flight-time budget (swap battery, load
    the next one) or waypoint count. The waypoint-count limit exists because
    DJI Fly enforces a hard per-file cap (200 waypoints, confirmed current for
    the Mini 5 Pro) and real-world reports (independent of this app -- see
    maxWaypointsPerFile's comment in DEFAULT_MISSION_CONFIG) describe the RC2's
    own mission UI destabilizing well before that on mapping-style missions
    with many closely-packed points, which is exactly the shape of mission a
    grid survey produces."""
    if not waypoints:
        return []
    budget = usable_battery_seconds(cfg)
    max_wps = max(1, int(cfg.get('maxWaypointsPerFile', 90) or 90))
    accel = cfg.get('droneAccel', 1.4)
    default_turn = cfg.get('turnMode', 'toPointAndStopWithDiscontinuityCurvature')
    batches = []
    current = [waypoints[0]]
    elapsed = waypoints[0].get('hover', 0) or 0
    for i in range(1, len(waypoints)):
        prev_wp, wp = waypoints[i - 1], waypoints[i]
        speed = prev_wp.get('speed') or cfg.get('speed') or 5
        dist = haversine_m(prev_wp['lat'], prev_wp['lon'], wp['lat'], wp['lon'])
        must_stop = (_is_stop_turn(prev_wp.get('turn_mode', default_turn))
                     or _is_stop_turn(wp.get('turn_mode', default_turn)))
        leg = leg_time_sec(dist, speed, accel, must_stop) + (wp.get('hover', 0) or 0)
        if current and (elapsed + leg > budget or len(current) >= max_wps):
            batches.append(current)
            current = []
            elapsed = 0
        current.append(wp)
        elapsed += leg
    if current:
        batches.append(current)
    return batches

# ── Auto-optimal grid rotation ───────────────────────────────────────────────

def _convex_hull(points):
    """Andrew's monotone chain. points: list of (x, y). Returns hull in CCW order."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]

def optimal_rotation_deg(polygon_latlon, cfg=None):
    """Rotating calipers: the minimum-area bounding rectangle of a convex hull is
    always aligned with one of its edges, so test each edge angle and keep the
    smallest. Aligns the grid sweep to minimize wasted transit distance.

    For an actual rectangle (or anything close to one), the edge angle along
    its LONG side and the edge angle along its SHORT side both produce the
    exact same bounding-box area (rotating a rectangle 90 degrees doesn't
    change its area) -- so area alone is a tie between "few long passes" and
    "many short passes", and which one wins was effectively down to hull
    vertex order, not efficiency. That's the bug: it could just as easily
    align the sweep across the short axis, producing many more passes (and
    turns) than flying along the long axis for the identical bounding area.

    Once cfg is known this instead scores each candidate by estimated total
    FLIGHT TIME (not raw distance -- a short row and a short inter-row
    side-step both pay the same accel/decel/stop-and-rotate cost as a long
    one, via the same leg_time_sec model used for the real time estimate),
    and picks the genuinely faster orientation. Raw distance alone still
    under-penalizes lots of short passes (more distance can lose to fewer
    turns once turn overhead is counted), which is why this isn't just
    summing row lengths. Without cfg (e.g. an older caller), falls back to
    the plain min-area angle, tie-broken toward fewer passes."""
    if len(polygon_latlon) < 3:
        return 0.0
    ref_lat = sum(p[0] for p in polygon_latlon) / len(polygon_latlon)
    ref_lon = sum(p[1] for p in polygon_latlon) / len(polygon_latlon)
    pts = [to_xy(p[0], p[1], ref_lat, ref_lon) for p in polygon_latlon]
    hull = _convex_hull(pts)
    if len(hull) < 3:
        return 0.0

    candidates = []  # (edge_angle, rx_span, ry_span, area)
    n = len(hull)
    for i in range(n):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % n]
        edge_angle = math.atan2(y2 - y1, x2 - x1)
        c, s = math.cos(-edge_angle), math.sin(-edge_angle)
        rx = [x * c - y * s for x, y in hull]
        ry = [x * s + y * c for x, y in hull]
        rx_span, ry_span = max(rx) - min(rx), max(ry) - min(ry)
        candidates.append((edge_angle, rx_span, ry_span, rx_span * ry_span))

    best_area = min(c[3] for c in candidates)

    side_spacing = coverage_spacing(cfg)[0] if cfg else None
    if side_spacing and side_spacing > 0:
        speed = cfg.get('speed') or 8
        accel = cfg.get('droneAccel') or 1.4
        def time_estimate(c):
            _, rx_span, ry_span, _ = c
            passes = max(1, math.ceil(ry_span / side_spacing) + 1)
            row_time = leg_time_sec(rx_span, speed, accel, True)
            turn_time = leg_time_sec(side_spacing, speed, accel, True)
            return passes * row_time + max(0, passes - 1) * turn_time
        best = min(candidates, key=time_estimate)
    else:
        # No cfg given -- still break area ties toward the orientation with
        # fewer rows (larger rx_span = passes run along the long axis)
        # instead of picking whichever edge the hull happened to start on.
        near_min = [c for c in candidates if c[3] <= best_area * 1.001]
        best = max(near_min, key=lambda c: c[1])

    return round(math.degrees(best[0]) % 180, 1)

# ── DJI WPML export (wpmz/template.kml + wpmz/waylines.wpml inside a .kmz) ──────
# XML structure and field names verified against DJI's published WPML spec and
# cross-checked against a real open-source generator/parser round-trip.

def _esc(s):
    return (s or '').replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;').replace('"', '&quot;')

def _action_group_xml(wp, idx):
    actions = []
    aid = 0
    # Gimbal is set once via an action (not a per-placemark angle tag) and stays at
    # that pitch until changed again — the caller only passes a gimbal value here
    # when it actually changes from the previous waypoint (see build_waylines_wpml).
    if wp.get('_gimbal_change') is not None:
        actions.append(f'''
          <wpml:action>
            <wpml:actionId>{aid}</wpml:actionId>
            <wpml:actionActuatorFunc>gimbalEvenlyRotate</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:gimbalPitchRotateAngle>{wp['_gimbal_change']}</wpml:gimbalPitchRotateAngle>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>''')
        aid += 1
    if wp.get('hover'):
        actions.append(f'''
          <wpml:action>
            <wpml:actionId>{aid}</wpml:actionId>
            <wpml:actionActuatorFunc>hover</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:hoverTime>{wp['hover']}</wpml:hoverTime>
            </wpml:actionActuatorFuncParam>
          </wpml:action>''')
        aid += 1
    if wp.get('photo'):
        actions.append(f'''
          <wpml:action>
            <wpml:actionId>{aid}</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>''')
        aid += 1
    if not actions:
        return ''
    return f'''
        <wpml:actionGroup>
          <wpml:actionGroupId>{idx}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{idx}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{idx}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger>
            <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
          </wpml:actionTrigger>{''.join(actions)}
        </wpml:actionGroup>'''

def _mission_config_xml(cfg):
    # No payloadInfo/takeOffSecurityHeight — DJI Fly's parser is stricter than the
    # documented enterprise Cloud-API spec and rejects the fuller version.
    return f'''  <wpml:missionConfig>
    <wpml:flyToWaylineMode>{cfg['flyToWaylineMode']}</wpml:flyToWaylineMode>
    <wpml:finishAction>{cfg['finishAction']}</wpml:finishAction>
    <wpml:exitOnRCLost>{cfg['exitOnRCLost']}</wpml:exitOnRCLost>
    <wpml:executeRCLostAction>{cfg['executeRCLostAction']}</wpml:executeRCLostAction>
    <wpml:globalTransitionalSpeed>{cfg['globalTransitionalSpeed']}</wpml:globalTransitionalSpeed>
    <wpml:droneInfo>
      <wpml:droneEnumValue>{cfg['droneEnumValue']}</wpml:droneEnumValue>
      <wpml:droneSubEnumValue>{cfg['droneSubEnumValue']}</wpml:droneSubEnumValue>
    </wpml:droneInfo>
  </wpml:missionConfig>'''

def build_template_kml(cfg):
    # DJI Fly reads waylines.wpml directly — template.kml just needs to exist
    # alongside it with the mission config, no duplicate placemark list needed.
    now = int(time.time() * 1000)
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">
<Document>
  <wpml:author>DroneMissionPlanner</wpml:author>
  <wpml:createTime>{now}</wpml:createTime>
  <wpml:updateTime>{now}</wpml:updateTime>
{_mission_config_xml(cfg)}
</Document>
</kml>'''

def build_waylines_wpml(cfg, waypoints):
    placemarks = []
    prev_gimbal = None
    for i, wp in enumerate(waypoints):
        gimbal = wp.get('gimbal', cfg.get('gimbalPitch', -90))
        wp2 = dict(wp)
        if i == 0 or gimbal != prev_gimbal:
            wp2['_gimbal_change'] = gimbal
            prev_gimbal = gimbal
        ag = _action_group_xml(wp2, i)
        heading_mode = wp.get('heading_mode', cfg['headingMode'])
        heading_angle = wp.get('heading_angle', 0)
        # Absent from DJI's documented WPML spec and from real production
        # files, so likely inert -- but an independent WPML library pairs
        # smoothTransition with 1, and matching that costs nothing.
        heading_angle_enable = 1 if heading_mode in ('smoothTransition', 'towardPOI') else 0
        placemarks.append(f'''
      <Placemark>
        <Point>
          <coordinates>{wp['lon']},{wp['lat']}</coordinates>
        </Point>
        <wpml:index>{i}</wpml:index>
        <wpml:executeHeight>{wp['alt']}</wpml:executeHeight>
        <wpml:waypointSpeed>{wp.get('speed', cfg['speed'])}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>{heading_mode}</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>{heading_angle}</wpml:waypointHeadingAngle>
          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
          <wpml:waypointHeadingAngleEnable>{heading_angle_enable}</wpml:waypointHeadingAngleEnable>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>
        </wpml:waypointHeadingParam>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>{wp.get('turn_mode', cfg['turnMode'])}</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
        <wpml:useStraightLine>1</wpml:useStraightLine>{ag}
      </Placemark>''')
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">
<Document>
{_mission_config_xml(cfg)}
  <Folder>
    <wpml:templateId>0</wpml:templateId>
    <wpml:executeHeightMode>{cfg['heightMode']}</wpml:executeHeightMode>
    <wpml:waylineId>0</wpml:waylineId>
    <wpml:distance>0</wpml:distance>
    <wpml:duration>0</wpml:duration>
    <wpml:autoFlightSpeed>{cfg['speed']}</wpml:autoFlightSpeed>{''.join(placemarks)}
  </Folder>
</Document>
</kml>'''

def export_wpml_kmz(cfg, waypoints, out_path):
    tkml = build_template_kml(cfg)
    wpml = build_waylines_wpml(cfg, waypoints)
    with zipfile.ZipFile(out_path, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('wpmz/template.kml', tkml)
        z.writestr('wpmz/waylines.wpml', wpml)

# ── Upload straight to a DJI RC over MTP ─────────────────────────────────────
# ADB can't work here: the RC2's adbd deliberately refuses host handshakes as a
# firmware hardening measure, so it reports "offline" forever. MTP is what DJI
# supports, driven via the Shell.Application COM automation Explorer itself uses.
#
# DJI Fly only loads missions it created, so a dummy mission must already exist
# on the controller as a UUID folder holding "<uuid>.kmz". list_mission_slots()
# lists them with waypoint count and location (DJI Fly's own mission title isn't
# reachable over MTP) so the user picks which one to replace.

WAYPOINT_MTP_PATH = ['Internal shared storage', 'Android', 'data', 'dji.go.v5', 'files', 'waypoint']
_UUID_RE = re.compile(r'^[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}$')

def _shell_app():
    import win32com.client
    return win32com.client.Dispatch('Shell.Application')

def _find_device_folder(shell, name_hint):
    computer = shell.NameSpace(17)  # CSIDL_DRIVES, i.e. "This PC"
    if computer is None:
        raise RuntimeError('Could not access "This PC" via the Windows Shell.')
    seen = []
    for item in computer.Items():
        seen.append(item.Name)
        if name_hint.lower() in item.Name.lower():
            return shell.NameSpace(item), item.Name
    raise RuntimeError(
        f'No device with "{name_hint}" in its name under This PC. Found: '
        + (', '.join(seen) or '(nothing — is the controller connected, unlocked, and awake?)')
    )

def _descend(shell, folder, name):
    if folder is None:
        return None, []
    seen = []
    for item in folder.Items():
        seen.append(item.Name)
        if item.Name.lower() == name.lower():
            return shell.NameSpace(item), seen
    return None, seen

def find_waypoint_folder(shell, device_name_hint):
    folder, found_name = _find_device_folder(shell, device_name_hint)
    trail = [found_name]
    for step in WAYPOINT_MTP_PATH:
        folder, siblings = _descend(shell, folder, step)
        if folder is None:
            raise RuntimeError(
                f'Could not find "{step}" inside {" > ".join(trail)}. Found instead: '
                + (', '.join(siblings) or '(empty)')
            )
        trail.append(step)
    return folder

def _copy_from_mtp(shell, mtp_item, dest_dir):
    dest_folder = shell.NameSpace(dest_dir)
    if dest_folder is None:
        raise RuntimeError(f'Could not open local temp folder {dest_dir}')
    FOF_SILENT, FOF_NOCONFIRMATION, FOF_NOERRORUI = 4, 16, 512
    dest_folder.CopyHere(mtp_item, FOF_SILENT | FOF_NOCONFIRMATION | FOF_NOERRORUI)

def _peek_mission_kmz(local_kmz_path):
    """Waypoint count, an approximate center point, and (if present — DJI Fly itself
    doesn't set one) a KML <name> tag, read from a mission file already on the
    controller. Only used to help identify a mission slot in the upload picker."""
    try:
        with zipfile.ZipFile(local_kmz_path) as z:
            names = z.namelist()
            xml_name = (next((n for n in names if n.lower().endswith('waylines.wpml')), None)
                        or next((n for n in names if n.lower().endswith('template.kml')), None))
            if not xml_name:
                return {'waypoints': None, 'lat': None, 'lon': None, 'name': None}
            root = ET.fromstring(z.read(xml_name).decode('utf-8', errors='replace'))
    except Exception:
        return {'waypoints': None, 'lat': None, 'lon': None, 'name': None}

    lats, lons, count = [], [], 0
    for pm in root.iter():
        if _local(pm.tag) != 'Placemark':
            continue
        for c in pm.iter():
            if _local(c.tag) == 'coordinates' and c.text:
                parts = c.text.strip().split(',')
                if len(parts) >= 2:
                    try:
                        lons.append(float(parts[0]))
                        lats.append(float(parts[1]))
                        count += 1
                    except ValueError:
                        pass
                break
    name = next((el.text.strip() for el in root.iter()
                 if _local(el.tag) == 'name' and el.text and el.text.strip()), None)
    return {
        'waypoints': count or None,
        'lat': round(sum(lats) / len(lats), 5) if lats else None,
        'lon': round(sum(lons) / len(lons), 5) if lons else None,
        'name': name,
    }

def list_mission_slots(device_name_hint='DJI', progress=None):
    """Every UUID-named mission folder on the controller, each with the real
    identifying info actually available: modified time, plus waypoint count and
    an approximate center read from the mission file itself. No fabricated names —
    DJI Fly's own mission title isn't exposed anywhere MTP can reach (see the
    module comment above), so this shows what's real instead of guessing."""
    def report(msg):
        if progress:
            progress(msg)
    try:
        import win32com.client  # noqa: F401
    except ImportError:
        raise RuntimeError('pywin32 is required for this — install with: pip install pywin32')

    shell = _shell_app()
    report('Looking for the controller...')
    waypoint_folder = find_waypoint_folder(shell, device_name_hint)

    slots = []
    for item in waypoint_folder.Items():
        if _UUID_RE.match(item.Name):
            try:
                mod = item.ModifyDate
            except Exception:
                mod = ''
            slots.append((item.Name, mod))
    if not slots:
        raise RuntimeError(
            'No mission folders under waypoint/. Create a throwaway waypoint mission '
            'in DJI Fly on the controller first, then try again.'
        )
    slots.sort(key=lambda m: m[1], reverse=True)

    report(f'Reading {len(slots)} mission slot(s)...')
    tmp_dir = tempfile.mkdtemp(prefix='dmp_peek_')
    results = []
    try:
        for uuid, mod in slots[:50]:
            report(f'  reading {uuid}...')
            entry = {'uuid': uuid, 'modified': str(mod), 'waypoints': None,
                     'lat': None, 'lon': None, 'name': None}
            mission_folder, _ = _descend(shell, waypoint_folder, uuid)
            kmz_item = mission_folder.ParseName(f'{uuid}.kmz') if mission_folder else None
            if kmz_item is not None:
                try:
                    _copy_from_mtp(shell, kmz_item, tmp_dir)
                    local_path = os.path.join(tmp_dir, f'{uuid}.kmz')
                    for _ in range(20):
                        if os.path.exists(local_path):
                            break
                        time.sleep(0.1)
                    if os.path.exists(local_path):
                        entry.update(_peek_mission_kmz(local_path))
                        os.remove(local_path)
                except Exception:
                    pass  # this slot just shows without waypoint/location details
            results.append(entry)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
    return results

def _replace_file_in_mtp_folder(folder, filename, local_path, report):
    """Delete <filename> in an MTP folder if present, verifying it's actually
    gone before copying the replacement in, then verify the copy landed under
    the exact same name rather than MTP silently creating a renamed duplicate
    (e.g. "name (1).jpg") when a delete hadn't fully propagated yet — a known
    MTP failure mode that looks like nothing happened even though the upload
    "succeeded". Raises with a specific, actionable message if that happened."""
    existing = folder.ParseName(filename)
    if existing is not None:
        report(f'Removing the old {filename}...')
        try:
            existing.InvokeVerb('delete')
        except Exception:
            pass
        for _ in range(20):  # up to ~4s, polling rather than a blind fixed sleep
            if folder.ParseName(filename) is None:
                break
            time.sleep(0.2)
        else:
            report(f'Warning: {filename} may not have actually been removed before uploading.')

    report(f'Uploading {filename}...')
    FOF_SILENT, FOF_NOCONFIRMATION, FOF_NOERRORUI = 4, 16, 512
    folder.CopyHere(local_path, FOF_SILENT | FOF_NOCONFIRMATION | FOF_NOERRORUI)

    target = None
    for _ in range(30):  # up to ~6s
        target = folder.ParseName(filename)
        if target is not None:
            break
        time.sleep(0.2)
    if target is None:
        raise RuntimeError(
            f'{filename} was not found after uploading — the MTP copy likely failed silently.'
        )

    stem = os.path.splitext(filename)[0]
    dupes = [item.Name for item in folder.Items()
             if item.Name != filename and item.Name.lower().startswith(stem.lower() + ' (')]
    if dupes:
        raise RuntimeError(
            f'The device created a renamed copy ({dupes[0]}) instead of overwriting '
            f'{filename} — the old file is still what DJI Fly is using. This is a known MTP '
            'quirk when a delete hasn\'t fully propagated; try again, or reboot the '
            'controller and retry.'
        )

def upload_kmz_to_slot(local_kmz_path, target_uuid, device_name_hint='DJI', progress=None):
    def report(msg):
        if progress:
            progress(msg)
    try:
        import win32com.client  # noqa: F401
    except ImportError:
        raise RuntimeError('pywin32 is required for this — install with: pip install pywin32')
    if not _UUID_RE.match(target_uuid or ''):
        raise RuntimeError('Invalid mission slot.')

    shell = _shell_app()
    report('Looking for the controller...')
    waypoint_folder = find_waypoint_folder(shell, device_name_hint)
    mission_folder, _ = _descend(shell, waypoint_folder, target_uuid)
    if mission_folder is None:
        raise RuntimeError(f'Mission slot {target_uuid} is no longer on the controller.')

    tmp_dir = tempfile.mkdtemp(prefix='dmp_mtp_')
    try:
        tmp_path = os.path.join(tmp_dir, f'{target_uuid}.kmz')
        shutil.copy(local_kmz_path, tmp_path)
        _replace_file_in_mtp_folder(mission_folder, f'{target_uuid}.kmz', tmp_path, report)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
    return target_uuid

def upload_preview_to_slot(local_jpg_path, target_uuid, device_name_hint='DJI', progress=None):
    """Replace the mission's map-preview thumbnail — a sibling structure to the
    mission itself: waypoint/map_preview/<uuid>/<uuid>.jpg. Not every DJI Fly
    version necessarily has this folder, so failures here are meant to be caught
    and treated as non-fatal by the caller.

    Worth knowing even when the file replace verifiably succeeds: DJI Fly's
    mission-*list* view appears to cache the thumbnail bitmap independent of the
    file on disk (it only visibly regenerates the thumbnail when you actually
    open a mission in the waypoint editor), so a raw file overwrite from outside
    the app may not show up in the list without DJI Fly itself re-scanning —
    opening the mission, restarting DJI Fly, or rebooting the controller forces
    that. The mission content itself (the kmz) is unaffected by this either way."""
    def report(msg):
        if progress:
            progress(msg)
    shell = _shell_app()
    waypoint_folder = find_waypoint_folder(shell, device_name_hint)
    preview_root, siblings = _descend(shell, waypoint_folder, 'map_preview')
    if preview_root is None:
        raise RuntimeError('No "map_preview" folder next to waypoint/ — found: '
                            + (', '.join(siblings) or 'nothing'))
    preview_folder, _ = _descend(shell, preview_root, target_uuid)
    if preview_folder is None:
        raise RuntimeError(f'No preview folder for mission {target_uuid}.')

    tmp_dir = tempfile.mkdtemp(prefix='dmp_preview_')
    try:
        tmp_path = os.path.join(tmp_dir, f'{target_uuid}.jpg')
        shutil.copy(local_jpg_path, tmp_path)
        _replace_file_in_mtp_folder(preview_folder, f'{target_uuid}.jpg', tmp_path, report)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
    report('Preview file replaced on the controller (the mission LIST thumbnail may still show '
           'the old one until you open the mission or restart DJI Fly — see note above).')

# ── Python API exposed to JS ─────────────────────────────────────────────────

class Api:
    def __init__(self):
        self._window = None

    def set_window(self, w):
        self._window = w

    def get_presets(self):
        return {'drones': DRONE_PRESETS, 'cameras': CAMERA_PRESETS, 'gimbals': GIMBAL_PRESETS,
                'defaults': DEFAULT_MISSION_CONFIG, 'version': APP_VERSION}

    # ── App settings (tiny JSON file in the user's home dir) ──
    # Browser-side storage can't be trusted for anything that must survive
    # restarts here: the page's origin includes the local server's port, and
    # an origin change silently wipes localStorage. Used for the remembered
    # drone model and the first-run tutorial flag.
    def get_app_settings(self):
        return {'ok': True, 'settings': _load_app_settings()}

    def set_app_setting(self, key, value):
        s = _load_app_settings()
        s[str(key)] = value
        _save_app_settings(s)
        return {'ok': True}

    # ── Import ──
    def import_kml(self):
        try:
            path = self._window.create_file_dialog(
                webview.OPEN_DIALOG,
                file_types=('KML files (*.kml)', 'KMZ files (*.kmz)', 'All files (*.*)')
            )
        except Exception as e:
            return {'ok': False, 'msg': f'Dialog error: {e}'}
        if not path:
            return {'ok': False, 'msg': 'Cancelled'}
        try:
            polygons, lines, points = parse_kml_kmz(path[0])
            return {'ok': True, 'polygons': polygons, 'lines': lines, 'points': points,
                     'msg': f'Loaded {len(polygons)} area(s), {len(lines)} route(s), '
                            f'{len(points)} point(s) from {os.path.basename(path[0])}'}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    # ── Generators ──
    # exclusions is only ever filtered on the server side (never silently on the
    # client), and only counted as "dropped" when the caller actually asked for
    # zones to be applied -- the frontend decides whether to keep the filtered
    # result or ask for an unfiltered regenerate based on excluded_count, so it
    # can warn before anything gets silently skipped rather than after.
    def generate_grid(self, polygon, cfg, exclusions=None):
        try:
            fn = generate_3d_mapping if cfg.get('threeDMapping') else generate_grid
            wps = fn(polygon, cfg, exclusions)
            photos = len(mission_photo_points(polygon, cfg, exclusions))
            excluded = 0
            if exclusions:
                excluded = max(0, len(mission_photo_points(polygon, cfg, None)) - photos)
            return {'ok': True, 'waypoints': wps, 'excluded_count': excluded, 'estimated_photos': photos}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def generate_corridor(self, line, cfg, exclusions=None):
        try:
            wps = generate_corridor(line, cfg, exclusions)
            photos = sum(len(s) for p in corridor_photo_points(line, cfg, exclusions)[0] for s in p)
            excluded = 0
            if exclusions:
                excluded_photos = sum(len(s) for p in corridor_photo_points(line, cfg, None)[0] for s in p)
                excluded = max(0, excluded_photos - photos)
            return {'ok': True, 'waypoints': wps, 'excluded_count': excluded, 'estimated_photos': photos}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def generate_orbit(self, center, cfg):
        try:
            return {'ok': True, 'waypoints': generate_orbit(center[0], center[1], cfg)}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def generate_overview(self, polygon, cfg, exclusions=None):
        try:
            wps = generate_overview(polygon, cfg, exclusions)
            excluded = 0
            if exclusions:
                excluded = max(0, len(generate_overview(polygon, cfg, None)) - len(wps))
            return {'ok': True, 'waypoints': wps, 'excluded_count': excluded}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def estimate_grid(self, polygon, cfg):
        try:
            return {'ok': True, **estimate_coverage(polygon, cfg)}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def terrain_follow(self, waypoints, ref_lat, ref_lon):
        """Adjust each waypoint's altitude so real height above ground stays
        constant over sloped terrain, using the first waypoint (or wherever the
        caller says launch will happen) as the elevation reference point --
        same assumption Litchi/Maven make, since the planner can't know the
        actual takeoff spot in advance either."""
        if not waypoints:
            return {'ok': False, 'msg': 'No waypoints to adjust'}
        try:
            coords = [(ref_lat, ref_lon)] + [(wp['lat'], wp['lon']) for wp in waypoints]
            elevations = fetch_elevations_m(coords)
            ref_elev = elevations[0]
            altitudes = [round(wp['alt'] + (e - ref_elev), 1) for wp, e in zip(waypoints, elevations[1:])]
            min_alt = min(altitudes)
            warn = (f'Lowest adjusted altitude is {min_alt}m -- double check nothing dips too close to '
                    f'the ground; SRTM data is ~30m resolution and can miss small terrain features.'
                    if min_alt < 5 else None)
            return {'ok': True, 'altitudes': altitudes, 'warn': warn}
        except Exception as e:
            return {'ok': False, 'msg': f'Elevation lookup failed (needs internet access): {e}'}

    def optimal_rotation(self, polygon, cfg=None):
        try:
            return {'ok': True, 'rotation': optimal_rotation_deg(polygon, cfg)}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def estimate_battery_split(self, waypoints, cfg):
        try:
            batches = split_mission_by_battery(waypoints, cfg)
            budget = usable_battery_seconds(cfg)

            accel = cfg.get('droneAccel', 1.4)
            default_turn = cfg.get('turnMode', 'toPointAndStopWithDiscontinuityCurvature')

            def batch_time(b):
                t = 0.0
                for i in range(1, len(b)):
                    speed = b[i - 1].get('speed') or cfg.get('speed') or 5
                    dist = haversine_m(b[i - 1]['lat'], b[i - 1]['lon'], b[i]['lat'], b[i]['lon'])
                    must_stop = (_is_stop_turn(b[i - 1].get('turn_mode', default_turn))
                                 or _is_stop_turn(b[i].get('turn_mode', default_turn)))
                    t += leg_time_sec(dist, speed, accel, must_stop)
                    t += b[i].get('hover', 0) or 0
                return t

            return {'ok': True, 'batteries': len(batches), 'usable_minutes': round(budget / 60, 1),
                     'batches': [{'waypoints': len(b), 'minutes': round(batch_time(b) / 60, 1)} for b in batches]}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    # ── Export ──
    def export_wpml(self, cfg, waypoints, filename=None):
        if not waypoints:
            return {'ok': False, 'msg': 'No waypoints to export'}
        fname = _safe_filename(filename) or 'mission.kmz'
        try:
            path = self._window.create_file_dialog(
                webview.SAVE_DIALOG, save_filename=fname,
                file_types=('DJI WPML mission (*.kmz)',)
            )
        except Exception as e:
            return {'ok': False, 'msg': f'Dialog error: {e}'}
        if not path:
            return {'ok': False, 'msg': 'Cancelled'}
        out = path if isinstance(path, str) else path[0]
        if not out.lower().endswith('.kmz'):
            out += '.kmz'
        try:
            export_wpml_kmz(cfg, waypoints, out)
            return {'ok': True, 'msg': f'Exported {len(waypoints)} waypoints to {os.path.basename(out)}'}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def export_wpml_split(self, cfg, waypoints, filename=None):
        if not waypoints:
            return {'ok': False, 'msg': 'No waypoints to export'}
        try:
            batches = split_mission_by_battery(waypoints, cfg)
        except Exception as e:
            return {'ok': False, 'msg': str(e)}
        if len(batches) <= 1:
            return self.export_wpml(cfg, waypoints, filename)
        base = os.path.splitext(_safe_filename(filename) or 'mission.kmz')[0]
        try:
            folder = self._window.create_file_dialog(webview.FOLDER_DIALOG)
        except Exception as e:
            return {'ok': False, 'msg': f'Dialog error: {e}'}
        if not folder:
            return {'ok': False, 'msg': 'Cancelled'}
        out_dir = folder if isinstance(folder, str) else folder[0]
        try:
            written = []
            for i, batch in enumerate(batches, start=1):
                fname = f'{base}_part{i}_of_{len(batches)}.kmz'
                out_path = os.path.join(out_dir, fname)
                export_wpml_kmz(cfg, batch, out_path)
                written.append(fname)
            return {'ok': True, 'msg': f'Exported {len(batches)} battery-sized missions to {out_dir}: '
                                        + ', '.join(written)}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def export_gcps(self, gcps, filename=None):
        # Reference markers for correcting the orthomosaic in
        # Pix4D/Metashape/WebODM, not flight waypoints (they're never written
        # into the WPML export -- DJI Fly would try to fly to them).
        # Elevation is required by all three: georeferencing needs X/Y/Z.
        # Column order varies by tool (Pix4D wants Easting first, WebODM
        # Northing first) and every importer has you map columns anyway, so
        # this order is just the readable default. Surveyed/Notes are for
        # your own reference.
        if not gcps:
            return {'ok': False, 'msg': 'No ground control points to export'}
        fname = _safe_filename(filename) or 'gcps.csv'
        if not fname.lower().endswith('.csv'):
            fname += '.csv'
        try:
            path = self._window.create_file_dialog(
                webview.SAVE_DIALOG, save_filename=fname,
                file_types=('Ground control points (*.csv)',)
            )
        except Exception as e:
            return {'ok': False, 'msg': f'Dialog error: {e}'}
        if not path:
            return {'ok': False, 'msg': 'Cancelled'}
        out = path if isinstance(path, str) else path[0]
        if not out.lower().endswith('.csv'):
            out += '.csv'
        try:
            with open(out, 'w', encoding='utf-8', newline='') as f:
                f.write('Label,Latitude,Longitude,Elevation,Surveyed,Notes\n')
                for p in gcps:
                    label = str(p.get('label', '')).replace(',', ' ').replace('"', "'")
                    notes = str(p.get('notes', '') or '').replace(',', ';').replace('"', "'").replace('\n', ' ')
                    surveyed = 'yes' if p.get('surveyed') else 'no'
                    elevation = p.get('elevation', 0) or 0
                    f.write(f"{label},{p['lat']:.8f},{p['lon']:.8f},{elevation:.2f},{surveyed},{notes}\n")
            return {'ok': True, 'msg': f'Exported {len(gcps)} ground control point(s) to {os.path.basename(out)}'}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def _live_log(self, msg, is_err=False):
        """Push a log line to the picker's terminal-style panel as it happens,
        instead of only returning the full log once the call finishes — useful
        for both the user (proof it isn't hung) and for diagnosing exactly which
        step failed on a flaky MTP connection."""
        try:
            self._window.evaluate_js(f'appendUploadLog({json.dumps(msg)}, {json.dumps(bool(is_err))})')
        except Exception:
            pass  # log streaming is best-effort; the final response still carries it all

    def list_rc_missions(self, device_hint='DJI'):
        log = []
        def report(msg):
            log.append(msg)
            self._live_log(msg)
        try:
            slots = list_mission_slots(device_hint or 'DJI', progress=report)
            return {'ok': True, 'slots': slots, 'log': log}
        except Exception as e:
            self._live_log(str(e), is_err=True)
            return {'ok': False, 'msg': str(e), 'log': log}

    def upload_to_rc_slot(self, cfg, waypoints, target_uuid, device_hint='DJI', preview_data_url=None):
        if not waypoints:
            return {'ok': False, 'msg': 'No waypoints to upload', 'log': []}
        log = []
        def report(msg):
            log.append(msg)
            self._live_log(msg)
        tmp_dir = tempfile.mkdtemp(prefix='dmp_upload_')
        try:
            tmp_kmz = os.path.join(tmp_dir, 'mission.kmz')
            export_wpml_kmz(cfg, waypoints, tmp_kmz)
            uuid = upload_kmz_to_slot(tmp_kmz, target_uuid, device_hint or 'DJI', progress=report)

            if preview_data_url:
                try:
                    _, b64data = preview_data_url.split(',', 1)
                    tmp_jpg = os.path.join(tmp_dir, 'preview.jpg')
                    with open(tmp_jpg, 'wb') as f:
                        f.write(base64.b64decode(b64data))
                    upload_preview_to_slot(tmp_jpg, uuid, device_hint or 'DJI', progress=report)
                except Exception as e:
                    report(f'Preview thumbnail not updated (mission itself is fine): {e}')

            return {'ok': True, 'msg': f'Uploaded — open the mission on the controller '
                                        f'(slot {uuid}) in DJI Fly.', 'log': log}
        except Exception as e:
            self._live_log(str(e), is_err=True)
            return {'ok': False, 'msg': str(e), 'log': log}
        finally:
            shutil.rmtree(tmp_dir, ignore_errors=True)

    # ── Project save/load ──
    def save_project(self, project_json):
        try:
            path = self._window.create_file_dialog(
                webview.SAVE_DIALOG, save_filename='mission.json',
                file_types=('Mission project (*.json)',)
            )
        except Exception as e:
            return {'ok': False, 'msg': f'Dialog error: {e}'}
        if not path:
            return {'ok': False, 'msg': 'Cancelled'}
        out = path if isinstance(path, str) else path[0]
        if not out.lower().endswith('.json'):
            out += '.json'
        try:
            with open(out, 'w', encoding='utf-8') as f:
                f.write(project_json)
            return {'ok': True, 'msg': f'Saved project to {os.path.basename(out)}'}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def load_project(self):
        try:
            path = self._window.create_file_dialog(
                webview.OPEN_DIALOG, file_types=('Mission project (*.json)', 'All files (*.*)')
            )
        except Exception as e:
            return {'ok': False, 'msg': f'Dialog error: {e}'}
        if not path:
            return {'ok': False, 'msg': 'Cancelled'}
        try:
            with open(path[0], 'r', encoding='utf-8') as f:
                data = f.read()
            return {'ok': True, 'data': data, 'msg': f'Loaded {os.path.basename(path[0])}'}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}


# ── HTML UI ────────────────────────────────────────────────────────────────────

HTML = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Drone Mission Planner</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
:root{
  --bg:#0d0d0d; --bg2:#161616; --bg3:#1f1f1f; --bg4:#262626; --border:#2a2a2a; --border2:#333;
  --orange:#e07b00; --orange2:#ff9500; --orange-dim:#7a4400; --orange-glow:#e07b0022;
  --text:#d6d6d6; --text-dim:#8a8a8a; --text-faint:#5c5c5c; --sel:#e07b0033; --hi:#e07b0066;
  --green:#3fae4b; --red:#c0392b; --blue:#4488ff;
  --radius:6px; --radius-sm:4px;
}
*{box-sizing:border-box;margin:0;padding:0;}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--text);
     display:flex;flex-direction:column;height:100vh;overflow:hidden;font-size:13px;
     -webkit-font-smoothing:antialiased;}

/* ── Titlebar ── */
#titlebar{display:flex;align-items:center;gap:10px;padding:8px 14px;
  background:linear-gradient(180deg,#0e0e0e,#0a0a0a);border-bottom:1px solid var(--orange-dim);
  box-shadow:0 1px 0 #000, 0 2px 6px #0006;flex-shrink:0;}
#titlebar .logo{color:var(--orange);font-size:15px;font-weight:700;letter-spacing:.3px;
  display:flex;align-items:center;gap:7px;}
#titlebar .logo #app-version{font-size:10px;color:var(--text-faint);font-weight:600;
  letter-spacing:.4px;margin-left:2px;}
#titlebar .credit{margin-left:auto;font-size:10px;color:var(--text-faint);}
#titlebar .credit a{color:var(--orange-dim);text-decoration:none;}
#titlebar .credit a:hover{color:var(--orange);}

/* ── Toolbar ── */
#toolbar{display:flex;align-items:center;gap:4px;padding:7px 10px;
  background:var(--bg2);border-bottom:1px solid var(--border);flex-shrink:0;flex-wrap:wrap;}
.tgroup{display:flex;gap:4px;padding:2px;background:#00000030;border-radius:var(--radius);}
button{background:var(--bg3);color:var(--text);border:1px solid var(--border2);border-radius:var(--radius-sm);
  padding:6px 12px;cursor:pointer;font-size:12px;white-space:nowrap;font-weight:500;
  transition:background .12s,border-color .12s,transform .08s;}
button:hover{background:var(--bg4);border-color:#484848;}
button:active{transform:translateY(1px);}
button.primary{background:linear-gradient(180deg,var(--orange),#c66a00);border-color:var(--orange2);color:#fff;
  font-weight:600;box-shadow:0 1px 3px #0006;}
button.primary:hover{background:linear-gradient(180deg,var(--orange2),var(--orange));}
button.active{background:var(--orange);border-color:var(--orange2);color:#000;font-weight:700;}
button:disabled{opacity:.35;cursor:default;}
.sep{width:1px;align-self:stretch;background:var(--border2);margin:2px 4px;}
#status-bar{margin-left:auto;font-size:11px;color:var(--text-dim);white-space:nowrap;
  padding:4px 10px;background:#00000030;border-radius:var(--radius-sm);}

#main{display:flex;flex:1;overflow:hidden;}

/* ── Sidebar ── */
#sidebar{width:330px;flex-shrink:0;background:#0a0a0a;border-right:1px solid var(--border);
  display:flex;flex-direction:column;}
#tabs{display:flex;border-bottom:1px solid var(--border);background:#080808;}
.tab{flex:1;text-align:center;padding:10px 4px;font-size:10.5px;text-transform:uppercase;
  letter-spacing:1.2px;cursor:pointer;color:var(--text-faint);border-bottom:2px solid transparent;
  font-weight:600;transition:color .12s;}
.tab:hover{color:var(--text-dim);}
.tab.active{color:var(--orange);border-bottom-color:var(--orange);background:var(--orange-glow);}
.tab-badge{display:inline-block;margin-left:5px;background:var(--orange);color:#000;
  border-radius:8px;padding:0 5px;font-size:8.5px;font-weight:800;letter-spacing:0;vertical-align:1px;}
.tab-badge:empty{display:none;}
#tab-content{flex:1;overflow-y:auto;padding:12px;}

/* ── Section cards ── */
.panel-section{margin-bottom:12px;background:#000000a0;border:1px solid var(--border);
  border-radius:var(--radius);padding:11px 12px;}
.panel-section h4{font-size:10px;color:var(--orange);text-transform:uppercase;
  letter-spacing:1.3px;margin-bottom:9px;font-weight:700;display:flex;align-items:center;gap:6px;}
.panel-section h4::before{content:'';width:3px;height:11px;background:var(--orange);border-radius:2px;}
.panel-section.tight{padding:8px 10px;}
.panel-section.pending{border-color:var(--orange);background:linear-gradient(180deg,#1c1300,#140d00);
  box-shadow:0 0 0 1px var(--orange-glow), 0 2px 10px #00000060;}
.panel-section.pending h4::before{background:var(--orange2);box-shadow:0 0 6px var(--orange2);}

.field{margin-bottom:9px;}
.field:last-child{margin-bottom:0;}
.field label{display:block;font-size:10.5px;color:var(--text-dim);margin-bottom:4px;font-weight:500;}
.field input,.field select{width:100%;background:var(--bg3);border:1px solid var(--border2);color:var(--text);
  border-radius:var(--radius-sm);padding:6px 8px;font-size:12px;outline:none;
  transition:border-color .12s,background .12s;}
.field input:hover,.field select:hover{border-color:#484848;}
.field input:focus,.field select:focus{border-color:var(--orange);background:var(--bg4);}
.field-row{display:flex;gap:8px;}
.field-row .field{flex:1;}
.checkbox-row{display:flex;align-items:center;gap:7px;font-size:12px;margin-bottom:9px;
  color:var(--text);cursor:pointer;user-select:none;}
.checkbox-row:last-child{margin-bottom:0;}
.checkbox-row input{width:auto;accent-color:var(--orange);cursor:pointer;}
.hint{font-size:10.5px;color:var(--text-dim);line-height:1.5;margin-top:5px;}
.hint.warn{color:var(--orange2);}
.hint b{color:var(--text);}
.help-icon{display:inline-flex;align-items:center;justify-content:center;width:14px;height:14px;
  border-radius:50%;background:var(--bg4);color:var(--text-faint);font-size:9px;font-weight:700;
  cursor:help;margin-left:5px;vertical-align:middle;border:1px solid var(--border2);flex-shrink:0;}
.help-icon:hover{background:var(--orange-dim);color:#fff;border-color:var(--orange);}

/* ── Collapsible groups ── */
details{border:1px solid var(--border);border-radius:var(--radius-sm);margin-bottom:8px;
  background:#00000060;overflow:hidden;}
details:last-child{margin-bottom:0;}
details[open]{border-color:var(--border2);}
details.active-kind{border-color:var(--orange-dim);}
summary{cursor:pointer;font-size:11px;color:var(--text-dim);padding:8px 10px;font-weight:600;
  list-style:none;display:flex;align-items:center;justify-content:space-between;
  transition:color .12s,background .12s;}
summary:hover{color:var(--text);background:#ffffff08;}
summary::-webkit-details-marker{display:none;}
summary::after{content:'\25BE';font-size:10px;color:var(--text-faint);transition:transform .15s;}
details[open] summary::after{transform:rotate(180deg);}
details .details-body{padding:2px 10px 10px;}
.kind-badge{font-size:8.5px;background:var(--orange-dim);color:#fff;padding:1px 6px;
  border-radius:8px;text-transform:uppercase;letter-spacing:.5px;margin-left:6px;}

.mission-type-grid{display:grid;grid-template-columns:1fr 1fr;gap:7px;}
.mission-type-btn{display:flex;flex-direction:column;align-items:center;gap:6px;
  padding:11px 4px 9px;font-size:11.5px;background:var(--bg3);border-color:var(--border2);
  color:var(--text-dim);}
.mission-type-btn svg{width:26px;height:26px;color:var(--text-faint);transition:color .12s;}
.mission-type-btn:hover{background:var(--bg4);border-color:var(--orange-dim);color:var(--text);}
.mission-type-btn:hover svg{color:var(--orange);}
.mission-type-btn.active{background:var(--orange);border-color:var(--orange2);color:#000;font-weight:700;}
.mission-type-btn.active svg{color:#000;}

/* ── Waypoints table ── */
#wp-table{width:100%;border-collapse:collapse;font-size:11px;}
#wp-table th{text-align:left;color:var(--orange);font-weight:600;padding:6px 4px;
  border-bottom:1px solid var(--border2);position:sticky;top:0;background:#0a0a0a;
  font-size:10px;text-transform:uppercase;letter-spacing:.5px;}
#wp-table td{padding:4px;border-bottom:1px solid #1a1a1a;}
#wp-table tr:hover{background:var(--bg3);}
#wp-table tr.selected{background:var(--sel);}
#wp-table input{width:52px;background:var(--bg3);border:1px solid var(--border2);color:var(--text);
  border-radius:2px;padding:3px 4px;font-size:10.5px;}
#wp-table .del-btn{color:var(--red);cursor:pointer;font-weight:bold;opacity:.7;}
#wp-table .del-btn:hover{opacity:1;}
#wp-stats{padding:9px 12px;font-size:10.5px;color:var(--text-dim);border-top:1px solid var(--border);
  display:flex;flex-wrap:wrap;gap:12px;background:#080808;}
#wp-stats b{color:var(--orange);}

/* ── Replay ── */
#replay-panel{background:var(--bg3);border:1px solid var(--border);border-radius:var(--radius);
  padding:10px;margin-bottom:10px;}
#replay-panel .row{display:flex;align-items:center;gap:6px;margin-bottom:7px;}
#replay-panel .row:last-of-type{margin-bottom:0;}
#replay-panel button{padding:5px 10px;}
#replay-panel input[type=range]{flex:1;accent-color:var(--orange);}
#replay-panel select{background:var(--bg2);border:1px solid var(--border2);color:var(--text);
  border-radius:var(--radius-sm);padding:4px 6px;font-size:11px;}
#replay-info{font-size:10.5px;color:var(--text-dim);}
#replay-info b{color:var(--orange);}
.drone-marker{width:26px;height:26px;display:flex;align-items:center;justify-content:center;
  font-size:20px;filter:drop-shadow(0 0 4px #000);transition:transform .15s linear;}

/* ── Layers ── */
.layer-item{background:var(--bg3);border:1px solid var(--border);border-radius:var(--radius-sm);
  padding:8px 9px;margin-bottom:6px;transition:border-color .12s;}
.layer-item:hover{border-color:var(--border2);}
.layer-item .name{font-size:12px;font-weight:600;color:var(--text);margin-bottom:6px;
  display:flex;align-items:center;gap:5px;}
.layer-item .actions{display:flex;gap:4px;flex-wrap:wrap;}
.layer-item button{font-size:10.5px;padding:4px 8px;}
#layers-list{margin-top:6px;}
.empty-hint{font-size:11.5px;color:var(--text-faint);text-align:center;padding:26px 14px;line-height:1.6;}

/* ── Map area ── */
#content{flex:1;display:flex;flex-direction:column;overflow:hidden;position:relative;}
#map{flex:1;width:100%;}
/* Centered in the space to the RIGHT of the reserved 258px search column
   (12px margin + ~230px search bar/results width), not the full map width --
   a plain 50% center still reaches back far enough at the app's minimum
   window size (1000px, ~670px of map) to collide with #map-search /
   #map-search-results if both are visible at once (drawing while a search
   dropdown is open). This stays clear at any supported window width instead
   of relying on tuned pixel margins that only work at one size. */
#draw-hint{position:absolute;top:12px;left:calc(258px + (100% - 258px)/2);
  transform:translateX(-50%);z-index:900;
  background:#0a0a0ae8;border:1px solid var(--orange);border-radius:var(--radius);padding:8px 18px;
  font-size:12px;color:#fff;display:none;pointer-events:none;box-shadow:0 4px 16px #000a;
  max-width:min(600px, calc(100% - 290px));text-align:center;line-height:1.5;}
#draw-hint.visible{display:block;}
#map-search{position:absolute;top:12px;left:12px;z-index:900;display:flex;gap:6px;}
#map-search input{width:230px;background:#0a0a0ae8;border:1px solid var(--border2);color:var(--text);
  border-radius:var(--radius-sm);padding:7px 10px;font-size:12.5px;box-shadow:0 4px 16px #000a;}
#map-search input:focus{border-color:var(--orange);outline:none;}
#map-search button{padding:6px 10px;box-shadow:0 4px 16px #000a;}
#map-search-results{position:absolute;top:50px;left:12px;z-index:900;width:230px;max-height:260px;
  overflow-y:auto;background:#0a0a0ae8;border:1px solid var(--border2);border-radius:var(--radius);
  box-shadow:0 4px 16px #000a;display:none;}
#map-search-results.visible{display:block;}
#map-search-results .result-item{padding:8px 10px;font-size:11.5px;color:var(--text);cursor:pointer;
  border-bottom:1px solid var(--border);line-height:1.4;}
#map-search-results .result-item:last-child{border-bottom:none;}
#map-search-results .result-item:hover{background:var(--bg4);}
#map-search-results .result-empty{padding:10px;font-size:11.5px;color:var(--text-faint);}

.wp-marker{width:22px;height:22px;border-radius:50%;background:var(--orange);border:2px solid #000;
  color:#000;font-size:10px;font-weight:800;display:flex;align-items:center;justify-content:center;
  box-shadow:0 2px 6px #000a;cursor:pointer;}
.wp-marker.selected{background:var(--orange2);box-shadow:0 0 8px var(--orange2);}
.wp-marker.first{background:var(--green);}
.wp-marker.last{background:var(--red);}
.poi-marker{width:16px;height:16px;border-radius:50%;background:var(--blue);border:2px solid #000;
  box-shadow:0 2px 6px #000a;}

.leaflet-tooltip.wp-tooltip{background:#0a0a0af0;color:var(--text);border:1px solid var(--orange);
  border-radius:var(--radius-sm);font-size:11px;line-height:1.5;padding:6px 9px;box-shadow:0 4px 14px #000a;}
.leaflet-tooltip.wp-tooltip::before{border-top-color:var(--orange);}
.wp-tooltip b{color:var(--orange);}

#progress-overlay{position:fixed;inset:0;background:#000c;z-index:9999;display:none;
  flex-direction:column;align-items:center;justify-content:center;gap:14px;backdrop-filter:blur(2px);}
#progress-overlay.visible{display:flex;}
#progress-msg{color:#ccc;font-size:12px;}

.modal-overlay{position:fixed;inset:0;background:#000a;z-index:9998;display:none;
  align-items:center;justify-content:center;backdrop-filter:blur(2px);}
.modal-overlay.visible{display:flex;}
.modal-panel{width:560px;max-width:92vw;max-height:80vh;display:flex;flex-direction:column;
  background:var(--bg2);border:1px solid var(--orange-dim);border-radius:var(--radius);
  box-shadow:0 12px 40px #000a;}
.modal-panel h3{padding:14px 16px;font-size:13px;color:var(--orange);border-bottom:1px solid var(--border);
  display:flex;align-items:center;justify-content:space-between;}
.modal-panel h3 span.close{cursor:pointer;color:var(--text-dim);font-weight:normal;font-size:16px;}
#picker-battery-warn{display:none;background:#2a1400;color:var(--orange2);font-size:11.5px;
  line-height:1.5;padding:9px 16px;border-bottom:1px solid var(--orange-dim);}
#picker-battery-warn.visible{display:block;}
#picker-log{background:#000;color:#3fda4f;font-family:Consolas,'Courier New',monospace;font-size:11px;
  line-height:1.6;padding:10px 14px;max-height:160px;overflow-y:auto;border-bottom:1px solid var(--border);
  display:none;white-space:pre-wrap;}
#picker-log.visible{display:block;}
#picker-log .err{color:#ff5c5c;}
#picker-list,#drone-picker-list{overflow-y:auto;padding:10px 16px;flex:1;}
.slot-row{background:var(--bg3);border:1px solid var(--border);border-radius:var(--radius-sm);
  padding:9px 11px;margin-bottom:8px;display:flex;align-items:center;gap:10px;}
.slot-row:last-child{margin-bottom:0;}
.slot-info{flex:1;font-size:11.5px;line-height:1.6;}
.slot-info .uuid{color:var(--text-faint);font-size:10px;font-family:Consolas,monospace;}
.slot-info .meta b{color:var(--orange);}
#picker-footer{padding:10px 16px;border-top:1px solid var(--border);font-size:10.5px;color:var(--text-dim);}

::-webkit-scrollbar{width:7px;height:7px;}
::-webkit-scrollbar-track{background:transparent;}
::-webkit-scrollbar-thumb{background:var(--border2);border-radius:4px;}
::-webkit-scrollbar-thumb:hover{background:var(--orange-dim);}

/* Keyboard-visible focus for accessibility -- mouse clicks don't show it,
   tabbing does. */
button:focus-visible,input:focus-visible,select:focus-visible,.tab:focus-visible{
  outline:2px solid var(--orange2);outline-offset:1px;}

/* ── Splashscreen ── */
#splash{position:fixed;inset:0;z-index:10000;background:var(--bg);
  display:flex;flex-direction:column;align-items:center;justify-content:center;gap:0;
  transition:opacity .45s ease;}
#splash.fading{opacity:0;pointer-events:none;}
#splash-art{position:relative;width:150px;height:150px;display:flex;
  align-items:center;justify-content:center;margin-bottom:18px;}
#splash-art .ring{position:absolute;inset:0;border:1.5px solid var(--orange);
  border-radius:50%;opacity:0;animation:splash-ping 2.1s ease-out infinite;}
#splash-art .ring:nth-child(2){animation-delay:.7s;}
#splash-art .ring:nth-child(3){animation-delay:1.4s;}
@keyframes splash-ping{
  0%{transform:scale(.35);opacity:0;}
  12%{opacity:.55;}
  100%{transform:scale(1.15);opacity:0;}
}
#splash-drone{animation:splash-hover 2.6s ease-in-out infinite;filter:drop-shadow(0 6px 14px #000c);}
@keyframes splash-hover{0%,100%{transform:translateY(0);}50%{transform:translateY(-7px);}}
#splash-title{font-size:19px;font-weight:700;color:var(--orange);letter-spacing:.6px;}
#splash-sub{font-size:10.5px;color:var(--text-faint);margin-top:6px;letter-spacing:2.5px;
  text-transform:uppercase;}
#splash-bar{width:170px;height:2px;background:var(--bg3);border-radius:2px;margin-top:22px;
  overflow:hidden;position:relative;}
#splash-bar::after{content:'';position:absolute;top:0;left:-40%;width:40%;height:100%;
  background:linear-gradient(90deg,transparent,var(--orange),transparent);
  animation:splash-shimmer 1.1s linear infinite;}
@keyframes splash-shimmer{to{left:100%;}}

/* ── Interactive tutorial ── */
#tour-overlay{position:fixed;inset:0;z-index:9000;display:none;}
#tour-overlay.visible{display:block;}
#tour-spotlight{position:fixed;border-radius:9px;z-index:9001;pointer-events:none;
  box-shadow:0 0 0 200vmax rgba(0,0,0,.74);border:1.5px solid var(--orange);
  transition:top .3s ease,left .3s ease,width .3s ease,height .3s ease,opacity .3s ease;}
#tour-spotlight.hidden-target{opacity:0;}
#tour-card{position:fixed;z-index:9002;width:340px;max-width:calc(100vw - 24px);background:var(--bg2);
  border:1px solid var(--orange-dim);border-radius:var(--radius);
  box-shadow:0 12px 44px #000d;transition:top .3s ease,left .3s ease;}
#tour-card h3{padding:12px 15px 0;font-size:13px;color:var(--orange);font-weight:700;}
#tour-card .tour-body{padding:8px 15px 12px;font-size:11.8px;line-height:1.65;color:var(--text);}
#tour-card .tour-body b{color:var(--orange2);}
/* A dot per step overflowed the card once the tour grew past a handful of
   steps -- 14 dots plus three buttons simply don't fit, and the Next button
   was pushed outside the card's right edge. A compact "3 / 14" counter takes
   fixed space no matter how many steps there are. min-width:0 + flex-shrink
   let the buttons compress rather than overflow if the card is ever
   narrowed. */
#tour-card .tour-footer{display:flex;align-items:center;gap:6px;padding:10px 15px;
  border-top:1px solid var(--border);min-width:0;}
#tour-card .tour-count{margin-right:auto;font-size:10.5px;color:var(--text-faint);
  font-variant-numeric:tabular-nums;white-space:nowrap;}
#tour-card .tour-footer button{flex-shrink:1;min-width:0;padding:6px 10px;}
#tour-card .tour-skip{background:none;border:none;color:var(--text-faint);font-size:11px;
  padding:4px 6px;flex-shrink:0;}
#tour-card .tour-skip:hover{color:var(--text);background:none;border:none;}
</style>
</head>
<body>

<div id="splash">
  <div id="splash-art">
    <div class="ring"></div><div class="ring"></div><div class="ring"></div>
    <svg id="splash-drone" width="64" height="64" viewBox="0 0 24 24">
      <g stroke="#e07b00" stroke-width="1.5" stroke-linecap="round" fill="none">
        <line x1="12" y1="12" x2="4.5" y2="4.5"/><line x1="12" y1="12" x2="19.5" y2="4.5"/>
        <line x1="12" y1="12" x2="4.5" y2="19.5"/><line x1="12" y1="12" x2="19.5" y2="19.5"/>
        <circle cx="4.5" cy="4.5" r="2.6"/><circle cx="19.5" cy="4.5" r="2.6"/>
        <circle cx="4.5" cy="19.5" r="2.6"/><circle cx="19.5" cy="19.5" r="2.6"/>
      </g>
      <circle cx="12" cy="12" r="2.4" fill="#e07b00"/>
    </svg>
  </div>
  <div id="splash-title">Drone Mission Planner</div>
  <div id="splash-sub">DJI WPML mission builder</div>
  <div id="splash-bar"></div>
</div>

<div id="tour-overlay">
  <div id="tour-spotlight"></div>
  <div id="tour-card">
    <h3 id="tour-title"></h3>
    <div class="tour-body" id="tour-body"></div>
    <div class="tour-footer">
      <div class="tour-count" id="tour-count"></div>
      <button class="tour-skip" onclick="endTour()">Skip ✕</button>
      <button id="tour-back" onclick="tourStep(-1)">&#8592; Back</button>
      <button class="primary" id="tour-next" onclick="tourStep(1)">Next &#8594;</button>
    </div>
  </div>
</div>

<div id="titlebar">
  <div>
    <div class="logo">&#128225; Drone Mission Planner<span id="app-version"></span></div>
  </div>
  <button id="btn-tour" onclick="startTour()" style="margin-left:auto;font-size:11px;padding:4px 10px;" title="Interactive guided tour of the app">&#127891; Tutorial</button>
  <div class="credit" style="margin-left:12px;">by <a href="https://github.com/0xpraet0rian" target="_blank">praet0rian (mark0)</a></div>
</div>

<div id="toolbar">
  <div class="tgroup">
    <button id="btn-import" class="primary" onclick="importKml()">&#128193; Import KML/KMZ</button>
  </div>
  <div class="tgroup">
    <button id="btn-finish" onclick="finishDraw()" style="display:none;">&#10003; Finish</button>
    <button id="btn-cancel" onclick="cancelDraw()" style="display:none;">&#10005; Cancel</button>
  </div>
  <div class="sep"></div>
  <div class="tgroup">
    <button id="btn-undo" onclick="undoAction()" disabled title="Undo (Ctrl+Z)">&#8617; Undo</button>
    <button id="btn-redo" onclick="redoAction()" disabled title="Redo (Ctrl+Y)">&#8618; Redo</button>
  </div>
  <div class="sep"></div>
  <div class="tgroup" id="tgroup-export">
    <button onclick="clearMission()" title="Clear the current mission's waypoints">&#128465; Clear</button>
    <button class="primary" onclick="exportWpml()" title="Export the mission as a DJI WPML .kmz ready to fly">&#128190; Export WPML</button>
    <button onclick="exportWpmlSplit()" title="Split into multiple missions sized to your battery's usable endurance">&#128267; Export by Battery</button>
    <button onclick="openUploadPicker()" title="Pick a mission slot on a connected DJI RC/RC2 to replace, over MTP">&#128225; Upload to RC</button>
  </div>
  <div class="sep"></div>
  <div class="tgroup" id="tgroup-file">
    <button onclick="saveProject()" title="Save the whole project (mission, zones, GCPs, settings) to a file">Save</button>
    <button onclick="loadProject()" title="Load a saved project file">Load</button>
  </div>
  <div id="status-bar">Ready</div>
</div>

<div id="progress-overlay"><div id="progress-msg">Loading...</div></div>

<div id="picker-overlay" class="modal-overlay">
  <div class="modal-panel">
    <h3>Upload to RC <span class="close" onclick="closeUploadPicker()">&times;</span></h3>
    <div id="picker-battery-warn"></div>
    <div id="picker-log"></div>
    <div id="picker-list"></div>
    <div id="picker-footer">Pick which mission slot on the controller gets replaced. Nothing else on the controller is touched.</div>
  </div>
</div>

<div id="drone-picker-overlay" class="modal-overlay">
  <div class="modal-panel">
    <h3>Which drone do you fly? <span class="close" onclick="closeDronePicker()">&times;</span></h3>
    <div id="drone-picker-list"></div>
    <div id="picker-footer">Sets the right camera and battery defaults. Change this anytime under Setup &rarr; Aircraft &amp; camera.</div>
  </div>
</div>

<div id="tour-welcome-overlay" class="modal-overlay">
  <div class="modal-panel" style="width:430px;">
    <h3>&#128075; Welcome</h3>
    <div style="padding:14px 16px;font-size:12.5px;line-height:1.7;color:var(--text);">
      First time here? Take a quick <b>interactive tour</b> — it walks through drawing a mission,
      the parameters that matter, and getting the flight onto your controller. About two minutes.
    </div>
    <div style="display:flex;gap:8px;padding:0 16px 14px;">
      <button class="primary" style="flex:1;" onclick="closeTourWelcome(true)">&#127891; Start the tour</button>
      <button onclick="closeTourWelcome(false)">Maybe later</button>
    </div>
  </div>
</div>

<div id="main">
  <div id="sidebar">
    <div id="tabs">
      <div class="tab active" data-tab="setup" onclick="showTab('setup')">Setup</div>
      <div class="tab" data-tab="waypoints" onclick="showTab('waypoints')">Waypoints<span id="tab-wp-count" class="tab-badge"></span></div>
      <div class="tab" data-tab="layers" onclick="showTab('layers')">Layers</div>
    </div>
    <div id="tab-content"></div>
    <div id="wp-stats"></div>
  </div>
  <div id="content">
    <div id="draw-hint"></div>
    <div id="map-search">
      <input id="map-search-input" type="text" placeholder="Search a place or address…" onkeydown="if(event.key==='Enter') mapSearch()">
      <button onclick="mapSearch()" title="Search">🔍</button>
      <button onclick="mapGeolocate()" title="Go to my location">📍</button>
    </div>
    <div id="map-search-results"></div>
    <div id="map"></div>
  </div>
</div>

<script>
// ── State ──────────────────────────────────────────────────────────────────
var PRESETS = {drones:{}, cameras:{}, gimbals:{}, defaults:{}};
var cfg = {};
var waypoints = [];      // {lat,lon,alt,speed,gimbal,heading_mode,heading_angle,photo,hover}
var pois = [];
var importedLayers = []; // {kind:'polygon'|'line'|'point', name, coords/lat/lon}
var activeTab = 'setup';
var drawMode = null;     // 'area'|'route'|'orbit'|'manual'|'exclude'|'gcp'
var tempPoints = [];
var selectedWpIdx = null;
var pendingKind = null;      // 'grid'|'corridor'|'orbit' — the source shape for the current mission
var pendingGeom = null;
var pendingGenerated = false; // true once this pending shape has been generated at least once
var missionName = 'Mission';  // prompted at the start of each mission, used as the export filename prefix
// Grid/corridor waypoints no longer carry photo:true per shot (camera fires
// on its own interval timer during continuous flight -- see generate_grid's
// Python comment), so the dense photo-count estimate the server computed at
// generation time is tracked separately here for display purposes.
var lastEstimatedPhotos = 0;
var exclusionZones = []; // [{coords:[[lat,lon],...]}] — no-fly holes a grid mission skips
var gcpPoints = [];       // [{lat,lon,label}] — ground control points, exported separately, never flown to

// ── Undo / redo ──────────────────────────────────────────────────────────
// Whole-state JSON snapshots rather than a command log: far safer across this
// many mutation sites than tracking an inverse for each, at a little memory.
var HISTORY_LIMIT = 50;
var historyUndo = [];
var historyRedo = [];
function snapshotState(){
  return JSON.stringify({pendingKind:pendingKind, pendingGeom:pendingGeom, pendingGenerated:pendingGenerated,
    waypoints:waypoints, exclusionZones:exclusionZones, gcpPoints:gcpPoints, importedLayers:importedLayers,
    missionName:missionName});
}
// Call BEFORE a mutation, capturing the about-to-be-old state onto the undo
// stack -- any new change invalidates whatever was in the redo stack.
function pushHistory(){
  historyUndo.push(snapshotState());
  if(historyUndo.length>HISTORY_LIMIT) historyUndo.shift();
  historyRedo = [];
  updateUndoRedoButtons();
}
function restoreSnapshot(snap){
  var s = JSON.parse(snap);
  pendingKind=s.pendingKind; pendingGeom=s.pendingGeom; pendingGenerated=s.pendingGenerated;
  waypoints=s.waypoints; exclusionZones=s.exclusionZones; gcpPoints=s.gcpPoints;
  importedLayers=s.importedLayers; missionName=s.missionName;
  redrawPendingBoundary(); redrawExclusionZones(); redrawGCPs(); drawImportedLayers(false);
  renderWaypoints();
  if(activeTab==='setup') renderSetup();
  else if(activeTab==='waypoints') renderWaypointsTab();
  else if(activeTab==='layers') renderLayersTab();
  updateImportButton();
  refreshEstimate();
}
function undoAction(){
  if(!historyUndo.length) return;
  historyRedo.push(snapshotState());
  restoreSnapshot(historyUndo.pop());
  updateUndoRedoButtons();
}
function redoAction(){
  if(!historyRedo.length) return;
  historyUndo.push(snapshotState());
  restoreSnapshot(historyRedo.pop());
  updateUndoRedoButtons();
}
function updateUndoRedoButtons(){
  var u=document.getElementById('btn-undo'), r=document.getElementById('btn-redo');
  if(u) u.disabled = historyUndo.length===0;
  if(r) r.disabled = historyRedo.length===0;
}
document.addEventListener('keydown', function(e){
  if(!(e.ctrlKey || e.metaKey)) return;
  var tag = document.activeElement ? document.activeElement.tagName : '';
  // Don't hijack Ctrl+Z/Y while typing in a text field -- the browser's own
  // native input undo should keep working there instead.
  if(tag==='INPUT' || tag==='TEXTAREA' || (document.activeElement && document.activeElement.isContentEditable)) return;
  var key = e.key.toLowerCase();
  if(key==='z' && !e.shiftKey){ e.preventDefault(); undoAction(); }
  else if(key==='y' || (key==='z' && e.shiftKey)){ e.preventDefault(); redoAction(); }
});

// ── Client-side coverage estimate (mirrors the Python formulas exactly) ────
// footprint = altitude * sensorSize / focalLength — real sensor/lens physics, same
// formula YMapper itself uses, not a derived-FOV approximation.
function footprintWH(c){
  return [c.altitude*c.sensor_w/c.focal, c.altitude*c.sensor_h/c.focal];
}
function coverageSpacing(c){
  var fw=footprintWH(c);
  var side = (c.sideSpacingOverride>0) ? c.sideSpacingOverride : Math.max(2, fw[0]*(1-c.sideOverlap/100));
  var fwd  = (c.forwardSpacingOverride>0) ? c.forwardSpacingOverride : Math.max(2, fw[1]*(1-c.forwardOverlap/100));
  return [side, fwd];
}
function recommendedShutterSpeed(c){
  if(!c.speed) return null;
  var gsd = (c.altitude*c.sensor_w)/(c.img_w*c.focal);
  var ideal = gsd/c.speed;
  var standard=[1/16000,1/8000,1/6400,1/5000,1/4000,1/3200,1/2500,1/2000,1/1600,1/1250,1/1000,
    1/800,1/640,1/500,1/400,1/320,1/240,1/200,1/160,1/120,1/100,1/80,1/60,1/50,1/40,1/30,1/25,
    1/20,1/15,1/12.5,1/10,1/8,1/6.25,1/5,1/4,1/3,1/2];
  var closest=standard.reduce((a,b)=>Math.abs(ideal-b)<Math.abs(ideal-a)?b:a);
  return Math.round(1/closest);
}
function polygonAreaM2(polygon){
  if(!polygon || polygon.length<3) return 0;
  var refLat=polygon[0][0], refLon=polygon[0][1];
  var pts=polygon.map(p=>toXY(p[0],p[1],refLat,refLon));
  var area=0, n=pts.length;
  for(var i=0;i<n;i++){ var a=pts[i], b=pts[(i+1)%n]; area += a[0]*b[1]-b[0]*a[1]; }
  return Math.abs(area)/2;
}
function toXY(lat,lon,refLat,refLon){
  return [(lon-refLon)*111320*Math.cos(refLat*Math.PI/180), (lat-refLat)*110540];
}
// Port of the Python sweep geometry so the live estimate matches the real
// generator exactly instead of guessing off the bounding box, which badly
// overcounts for diagonal or thin polygons. pointInPolygonJS itself is kept
// for the exclusion-zone hit test on manual waypoint placement.
function pointInPolygonJS(x,y,poly){
  var inside=false, n=poly.length;
  for(var i=0,j=n-1;i<n;j=i++){
    var xi=poly[i][0], yi=poly[i][1], xj=poly[j][0], yj=poly[j][1];
    if(((yi>y)!==(yj>y)) && (x < (xj-xi)*(y-yi)/(yj-yi)+xi)) inside=!inside;
  }
  return inside;
}
function projectExclusionsJS(exclusions, refLat, refLon, cf, sf){
  return (exclusions||[]).filter(e=>e.length>=3).map(function(e){
    return e.map(p=>toXY(p[0],p[1],refLat,refLon)).map(p=>[p[0]*cf-p[1]*sf, p[0]*sf+p[1]*cf]);
  });
}
// Exact scanline row intervals -- direct mirror of the Python backend's
// _scanline_intervals/_subtract_intervals, so the live
// photo-count estimate is computed by the SAME geometry as the real
// generator rather than a sampled approximation of it. See
// _sweep_coverage_rows (Python) for the full reasoning.
function scanlineIntervalsJS(y, poly){
  var xs=[], n=poly.length;
  for(var i=0;i<n;i++){
    var x1=poly[i][0], y1=poly[i][1];
    var x2=poly[(i+1)%n][0], y2=poly[(i+1)%n][1];
    if((y1>y)!==(y2>y)) xs.push(x1 + (y-y1)*(x2-x1)/(y2-y1));
  }
  xs.sort(function(a,b){return a-b;});
  var out=[];
  for(var k=0;k+1<xs.length;k+=2) out.push([xs[k], xs[k+1]]);
  return out;
}
function subtractIntervalsJS(ivs, holes){
  var out=ivs;
  holes.forEach(function(h){
    var nxt=[];
    out.forEach(function(iv){
      if(h[1]<=iv[0] || h[0]>=iv[1]){ nxt.push(iv); return; }
      if(h[0]>iv[0]) nxt.push([iv[0], h[0]]);
      if(h[1]<iv[1]) nxt.push([h[1], iv[1]]);
    });
    out=nxt;
  });
  return out;
}
// Rows of SEGMENTS, mirroring Python's _sweep_coverage_rows. Segment count
// matters as well as point count: in Turn Only mode each segment costs two
// waypoints, so a zone that splits rows pushes the total past "passes x 2".
// Mirrors _offset_polygon in the Python backend -- a real mitered dilation,
// not a scale about the centroid. See that function for why.
function offsetPolygonJS(poly, dist){
  if(!dist || poly.length<3) return poly.slice();
  var pts=poly.slice(), n=pts.length, area2=0;
  for(var i=0;i<n;i++){ var j=(i+1)%n; area2 += pts[i][0]*pts[j][1] - pts[j][0]*pts[i][1]; }
  if(area2<0) pts.reverse();
  var normals=[];
  for(var i=0;i<n;i++){
    var a=pts[i], b=pts[(i+1)%n];
    var dx=b[0]-a[0], dy=b[1]-a[1], L=Math.hypot(dx,dy)||1;
    normals.push([dy/L, -dx/L]);   // outward for CCW winding
  }
  var out=[];
  for(var i=0;i<n;i++){
    var pv=normals[(i-1+n)%n], nx=normals[i];
    var mx=pv[0]+nx[0], my=pv[1]+nx[1], ml=Math.hypot(mx,my);
    if(ml<1e-9){ out.push([pts[i][0]+nx[0]*dist, pts[i][1]+nx[1]*dist]); continue; }
    mx/=ml; my/=ml;
    var cosHalf=mx*pv[0]+my*pv[1];
    var miter=dist/Math.max(0.35, cosHalf);
    out.push([pts[i][0]+mx*miter, pts[i][1]+my*miter]);
  }
  return out;
}
function sweepCoverageRowsJS(rpts, sideSpacing, forwardSpacing, excludePolys, boundaryMargin){
  var margin=boundaryMargin||0;
  // Dilate first, then sweep rows at swath CENTRES -- matches
  // _sweep_coverage_rows exactly; see its comments for why neither clamping
  // nor placing rows on the span's extremes works for a rotated site.
  var area=margin>0 ? offsetPolygonJS(rpts, margin) : rpts;
  var ys=area.map(p=>p[1]);
  var miny=Math.min.apply(null,ys), maxy=Math.max.apply(null,ys);
  var spanY=maxy-miny;
  var nRows=spanY>0 ? Math.max(1, Math.ceil(spanY/sideSpacing)) : 1;
  var rowSpacing=spanY>0 ? spanY/nRows : sideSpacing;
  var rows=[], reverse=false;
  for(var rowI=0; rowI<nRows; rowI++){
    var y=miny+(rowI+0.5)*rowSpacing;
    var intervals=scanlineIntervalsJS(y, area);
    if(excludePolys && excludePolys.length){
      var holes=[];
      excludePolys.forEach(function(hp){ holes=holes.concat(scanlineIntervalsJS(y, hp)); });
      intervals=subtractIntervalsJS(intervals, holes);
    }
    var segs=[];
    intervals.forEach(function(iv){
      var a=iv[0], b=iv[1], length=b-a;
      if(length<0.5){ segs.push([[(a+b)/2, y]]); return; }
      var nSteps=Math.max(1, Math.ceil(length/forwardSpacing));
      var step=length/nSteps, seg=[];
      for(var k=0;k<=nSteps;k++) seg.push([a+k*step, y]);
      segs.push(seg);
    });
    if(reverse){ segs.reverse(); segs.forEach(function(s){ s.reverse(); }); }
    if(segs.length) rows.push(segs);
    reverse=!reverse;
  }
  return rows;
}
function sweepCoverageJS(rpts, sideSpacing, forwardSpacing, excludePolys, boundaryMargin){
  var pts=[];
  sweepCoverageRowsJS(rpts, sideSpacing, forwardSpacing, excludePolys, boundaryMargin)
    .forEach(function(row){ row.forEach(function(seg){ pts=pts.concat(seg); }); });
  return pts;
}
function sweepSegmentCountJS(rpts, sideSpacing, forwardSpacing, excludePolys, boundaryMargin){
  var n=0;
  sweepCoverageRowsJS(rpts, sideSpacing, forwardSpacing, excludePolys, boundaryMargin)
    .forEach(function(row){ n += row.length; });
  return n;
}
function estimateGrid(polygon, c, exclusions){
  if(!polygon || polygon.length<3) return null;
  var refLat=polygon.reduce((s,p)=>s+p[0],0)/polygon.length;
  var refLon=polygon.reduce((s,p)=>s+p[1],0)/polygon.length;
  var pts=polygon.map(p=>toXY(p[0],p[1],refLat,refLon));
  var rot=(c.rotationDeg||0)*Math.PI/180, cf=Math.cos(-rot), sf=Math.sin(-rot);
  var rpts=pts.map(p=>[p[0]*cf-p[1]*sf, p[0]*sf+p[1]*cf]);
  var exPolys=projectExclusionsJS(exclusions, refLat, refLon, cf, sf);
  var sp=coverageSpacing(c);
  var side=sp[0], forward=sp[1];
  var margin=side/2; // mirrors generate_grid's boundary margin -- keeps this live estimate in sync with the real export
  var rows=sweepCoverageRowsJS(rpts, side, forward, exPolys, margin);
  var count=0, segCount=0;
  rows.forEach(function(row){ segCount+=row.length; row.forEach(function(s){ count+=s.length; }); });
  if(c.crosshatch && !c.threeDMapping){
    var transposed=rpts.map(p=>[p[1],p[0]]);
    var exTransposed=exPolys.map(poly=>poly.map(p=>[p[1],p[0]]));
    count += sweepCoverageJS(transposed, side, forward, exTransposed, margin).length;
    segCount += sweepSegmentCountJS(transposed, side, forward, exTransposed, margin);
  }
  if(c.threeDMapping){
    // Mirrors generate_3d_mapping: a second full pass rotated 90°, oblique gimbal.
    var rot2=((c.rotationDeg||0)+90)*Math.PI/180, cf2=Math.cos(-rot2), sf2=Math.sin(-rot2);
    var rpts2=pts.map(p=>[p[0]*cf2-p[1]*sf2, p[0]*sf2+p[1]*cf2]);
    var exPolys2=projectExclusionsJS(exclusions, refLat, refLon, cf2, sf2);
    count += sweepCoverageJS(rpts2, side, forward, exPolys2, margin).length;
    segCount += sweepSegmentCountJS(rpts2, side, forward, exPolys2, margin);
  }
  // passes = the real number of scan rows, straight from the sweep, rather
  // than re-deriving it from the bounding-box height (which ignored where the
  // rows actually landed).
  return {side:side.toFixed(1), forward:forward.toFixed(1),
          passes:Math.max(1, rows.length), segments:Math.max(1, segCount), photos:count};
}
function estimateCorridor(line, c){
  if(!line || line.length<2) return null;
  // Measure in the SAME local-xy projection the generator uses, not with
  // haversine: to_xy's constants and the haversine sphere differ by ~0.1%,
  // which is invisible until it lands either side of a ceil() boundary and
  // the estimate reports one photo per pass fewer than the mission actually
  // contains (seen at 640 m, correct at 350 m and 910 m).
  var lref=line[0];
  var lxy=line.map(function(q){ return toXY(q[0],q[1],lref[0],lref[1]); });
  var length=0;
  for(var i=1;i<lxy.length;i++) length+=Math.hypot(lxy[i][0]-lxy[i-1][0], lxy[i][1]-lxy[i-1][1]);
  var sp=coverageSpacing(c);
  var side=sp[0], forward=sp[1];
  var width=Math.max(0,c.corridorWidth||0);
  // Mirrors corridor_photo_points: passes span the corridor width PLUS a
  // half-line-spacing margin on each side, and each pass is extended by that
  // same margin past both ends of the route before being sampled. Without
  // the margin terms this under-counted every corridor.
  var margin=side/2;
  var span=width+2*margin;
  var passes = width<=0 ? 1 : Math.max(2, Math.ceil(span/side)+1);
  var passLen = length + 2*margin;
  var perPass = Math.max(1, Math.ceil(passLen/forward)) + 1;
  // Exclusion zones aren't subtracted here (unlike the grid estimate, which
  // has exact interval geometry to work with). They can only ever remove
  // photos, so this stays an upper bound -- the safe direction for the
  // battery/file-count warnings this feeds.
  return {side:side.toFixed(1), forward:forward.toFixed(1), passes:passes,
          segments:passes, photos:passes*perPass};
}
function refreshEstimate(){
  var el=document.getElementById('live-estimate');
  if(!el) return;
  if(pendingKind==='orbit'){
    var n=Math.max(3,parseInt(cfg.orbitPoints)||8);
    var rings=Math.max(1,parseInt(cfg.orbitRings)||1);
    var circumference=2*Math.PI*(cfg.orbitRadius||10);
    var ringsNote = rings>1 ? ' &middot; <b>'+rings+'</b> rings' : '';
    el.innerHTML = '<div style="font-size:16px;color:var(--orange);font-weight:700;margin:4px 0;">'+(n*rings)+' waypoints'+ringsNote+'</div>' +
      '<div class="hint">Radius <b>'+(cfg.orbitRadius||10)+'m</b> &middot; circumference ~'+Math.round(circumference)+'m &middot; '+
      'point spacing ~'+Math.round(circumference/n)+'m</div>';
    return;
  }
  var est=null, areaM2=0;
  if(pendingKind==='grid'){ est=estimateGrid(pendingGeom, cfg, exclusionZones.map(z=>z.coords)); areaM2=polygonAreaM2(pendingGeom); }
  else if(pendingKind==='corridor') est=estimateCorridor(pendingGeom, cfg);
  if(!est){ el.innerHTML=''; return; }
  var shutter=recommendedShutterSpeed(cfg);
  var isFull = cfg.waypointMode==='full';
  var camInterval = cfg.cameraInterval||2.0;
  var rowSpeed = Math.max(0.5, Number(est.forward)/camInterval);
  var interval = isFull ? (cfg.speed ? (Number(est.forward)/cfg.speed) : 0) : camInterval;
  var flightSec, estWpCount;
  if(isFull){
    // Full mode: a real stop at every photo, same physics as computeFlightSeconds
    // uses post-generation -- see leg_time_sec's Python comment.
    var legCount = Math.max(0, est.photos-1);
    var mustStop = isStopTurn(cfg.turnMode);
    flightSec = legCount * legTimeSec(Number(est.forward), cfg.speed||1, cfg.droneAccel, mustStop);
    estWpCount = est.photos;
  } else {
    // Turn Only: waypoints are sparse (row endpoints only, continuous cruise
    // between them at rowSpeed). Model it as continuous cruise plus one
    // accel/decel turn per pass, since a row reversal is always a real stop.
    var totalDist = Math.max(0, est.photos-1) * Number(est.forward);
    var turns = Math.max(0, (est.passes||1) - 1);
    var turnSec = turns * legTimeSec(Number(est.side)||Number(est.forward), rowSpeed, cfg.droneAccel, true);
    flightSec = totalDist/rowSpeed + turnSec;
    // Two waypoints per flyable STRETCH, not per pass: a no-fly zone splits
    // rows into several stretches, each costing its own pair. Slightly under-
    // counts when zones are present (the routing detour points can't be known
    // without running the full pass -- 42 vs 45 actual in testing); the
    // warning shown after generating uses the real waypoint array and is exact.
    estWpCount = (est.segments || est.passes || 1) * 2;
  }
  var usableSec = usableBatteryMinutes(cfg)*60;
  var battBatches = Math.max(1, Math.ceil(flightSec/usableSec));
  var wpBatches = Math.max(1, Math.ceil(estWpCount/(cfg.maxWaypointsPerFile||90)));
  var batches = Math.max(battBatches, wpBatches);
  var warnWhy = wpBatches>battBatches ? '~'+estWpCount+' waypoints is over the safe per-file limit' : '~'+batches+' batteries needed at this size';
  var warn = batches>1
    ? '<div class="hint warn">&#128267; '+warnWhy+' &mdash; use "Export by Battery" after generating to split automatically, or lower overlap/raise altitude to shrink it.</div>' : '';
  var camNote = (!isFull && (pendingKind==='grid' || pendingKind==='corridor')) ?
    '<div class="hint" style="color:var(--orange2);">&#128247; Before flying: set the camera to Timer/interval shooting at <b>'+camInterval.toFixed(1)+'s</b>, then fly this mission at <b>~'+rowSpeed.toFixed(1)+' m/s</b> &mdash; that combination is what actually gets you '+est.forward+'m photo spacing (see the &#63; on Camera interval under Advanced for why this can\'t be automated).</div>' : '';
  var extraStats = '<div class="hint">' +
    (areaM2 ? 'Area <b>'+(areaM2>=10000?(areaM2/10000).toFixed(2)+' ha':Math.round(areaM2)+' m&sup2;')+'</b> &middot; ' : '') +
    'Photo interval <b>~'+interval.toFixed(1)+'s</b>' +
    (shutter ? ' &middot; shutter &le; <b>1/'+shutter+'</b> to avoid blur' : '') +
    ' &middot; flight <b>~'+Math.round(flightSec/60)+'m</b>' +
    '</div>';
  el.innerHTML = '<div class="hint">Line spacing <b>'+est.side+'m</b> &middot; photo spacing <b>'+est.forward+'m</b> &middot; '+est.passes+' pass(es)</div>' +
    '<div style="font-size:16px;color:var(--orange);font-weight:700;margin:4px 0;">~'+est.photos+' photos (estimate)</div>' +
    extraStats + camNote + warn;
}

// zoomControl:false + added back at bottomleft -- Leaflet's default top-left
// zoom control would otherwise sit directly under #map-search (also top-left).
var map = L.map('map', {preferCanvas:true, zoomControl:false}).setView([45.35,22.28], 12);
L.control.zoom({position:'bottomleft'}).addTo(map);

// ── Base layers (switchable) ────────────────────────────────────────────────
// maxNativeZoom is where the tile provider's real imagery stops; Leaflet upscales
// past that instead of going blank. Esri satellite goes to z23, OSM/CARTO ~z19-20,
// OpenTopoMap z17.
var baseStreets = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  {maxZoom:22, maxNativeZoom:19, attribution:'&copy; OpenStreetMap'});
var baseSatellite = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {maxZoom:23, maxNativeZoom:23, attribution:'Tiles &copy; Esri'});
// Google's raw tile endpoint isn't an official/licensed API -- there's no key, no
// SLA, and it's outside Google's terms of service for map tile access, but it's
// the same well-known trick most hobby GIS/drone-planning tools use to get
// satellite imagery that's often higher native resolution than Esri's in a given
// area (varies by region -- neither source is uniformly better everywhere). It
// can be blocked or rate-limited without notice since it's unofficial.
var baseGoogleSat = L.tileLayer('https://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
  {subdomains:['mt0','mt1','mt2','mt3'], maxZoom:22, maxNativeZoom:21, attribution:'Imagery &copy; Google (unofficial tile access)'});
var baseSatelliteLabels = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}',
  {maxZoom:23, maxNativeZoom:19, attribution:'Esri', pane:'shadowPane'});
var baseTopo = L.tileLayer('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',
  {maxZoom:20, maxNativeZoom:17, attribution:'&copy; OpenTopoMap (CC-BY-SA)'});
var baseDark = L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
  {maxZoom:22, maxNativeZoom:20, attribution:'&copy; CARTO'});
baseStreets.addTo(map);

var importedGroup = L.layerGroup().addTo(map);
var tempGroup = L.layerGroup().addTo(map);
var wpGroup = L.layerGroup().addTo(map);
var snapGroup = L.layerGroup().addTo(map);
var exclusionGroup = L.layerGroup().addTo(map);
var gcpGroup = L.layerGroup().addTo(map);
var geoGroup = L.layerGroup().addTo(map); // "you are here" marker from the geolocate button, kept separate from tempGroup so it isn't wiped by cancelDraw()
// Keeps a hand-drawn boundary on the map after drawing finishes: cancelDraw()
// clears tempGroup, which otherwise made the traced shape vanish. Separate from
// tempGroup (in-progress) and importedGroup (KMZ shapes are already drawn).
var pendingBoundaryGroup = L.layerGroup().addTo(map);
function redrawPendingBoundary(){
  pendingBoundaryGroup.clearLayers();
  if(!pendingGeom) return;
  if(pendingKind==='grid'){
    L.polygon(pendingGeom, {color:'#4488ff', weight:2, fillOpacity:.06, dashArray:'6,4'}).addTo(pendingBoundaryGroup);
  } else if(pendingKind==='corridor'){
    L.polyline(pendingGeom, {color:'#4488ff', weight:2, dashArray:'6,4'}).addTo(pendingBoundaryGroup);
  } else if(pendingKind==='orbit'){
    L.circleMarker(pendingGeom, {radius:6, color:'#4488ff', weight:2, fillOpacity:.3}).addTo(pendingBoundaryGroup);
  }
}
var wpPathLayer = null;
var wpMarkers = {};

var satelliteHybrid = L.layerGroup([baseSatellite, baseSatelliteLabels]);
var googleSatHybrid = L.layerGroup([baseGoogleSat, baseSatelliteLabels]);
L.control.layers({
  'Street': baseStreets,
  'Satellite (Esri)': satelliteHybrid,
  'Satellite (Esri, no labels)': baseSatellite,
  'Satellite (Google)': googleSatHybrid,
  'Satellite (Google, no labels)': baseGoogleSat,
  'Topographic': baseTopo,
  'Dark': baseDark,
}, {
  'Imported KML/KMZ': importedGroup,
  'Flight path': wpGroup,
  'No-fly / exclusion zones': exclusionGroup,
  'Ground control points': gcpGroup,
}, {position:'topright', collapsed:true}).addTo(map);

// ── Map search / geolocate ──────────────────────────────────────────────────
// Nominatim (OpenStreetMap's free geocoder) -- no API key, but rate-limited to
// ~1 req/sec by its usage policy, which a single interactive search box stays
// well under.
function mapSearch(){
  var input = document.getElementById('map-search-input');
  var q = input.value.trim();
  var resultsEl = document.getElementById('map-search-results');
  if(!q){ resultsEl.classList.remove('visible'); return; }
  resultsEl.innerHTML = '<div class="result-empty">Searching…</div>';
  resultsEl.classList.add('visible');
  fetch('https://nominatim.openstreetmap.org/search?format=json&limit=6&q=' + encodeURIComponent(q))
    .then(function(r){ return r.json(); })
    .then(function(results){
      if(!results || !results.length){
        resultsEl.innerHTML = '<div class="result-empty">No results found.</div>';
        return;
      }
      resultsEl.innerHTML = '';
      results.forEach(function(r){
        var item = document.createElement('div');
        item.className = 'result-item';
        item.textContent = r.display_name;
        item.onclick = function(){
          var lat = parseFloat(r.lat), lon = parseFloat(r.lon);
          map.setView([lat, lon], 17);
          resultsEl.classList.remove('visible');
          resultsEl.innerHTML = '';
        };
        resultsEl.appendChild(item);
      });
    })
    .catch(function(){
      resultsEl.innerHTML = '<div class="result-empty">Search failed -- check your internet connection.</div>';
    });
}
function mapGeolocate(){
  if(!navigator.geolocation){
    alert('Geolocation is not available in this window.');
    return;
  }
  setStatus('Locating…');
  navigator.geolocation.getCurrentPosition(function(pos){
    var lat = pos.coords.latitude, lon = pos.coords.longitude;
    map.setView([lat, lon], 17);
    geoGroup.clearLayers();
    L.circleMarker([lat, lon], {radius:8, color:'#3b82f6', weight:2, fillColor:'#3b82f6', fillOpacity:.5})
      .bindTooltip('Your location').addTo(geoGroup);
    setStatus('Ready');
  }, function(err){
    // WebView2's own geolocation permission prompt (a bar at the top of the
    // window, not this app's UI) has to be accepted for this to work at all --
    // a denial or an unsupported host surfaces here as a plain error message
    // rather than this app silently doing nothing.
    setStatus('Ready');
    alert('Could not get your location: ' + err.message);
  }, {enableHighAccuracy:true, timeout:12000});
}

// ── Splashscreen ───────────────────────────────────────────────────────────
// Visible from the very first paint (static HTML, no JS needed to show it) so
// it genuinely covers the presets/settings load, then holds a minimum beat so
// it reads as a deliberate opening rather than a flicker.
var _splashT0 = Date.now();
var _splashHidden = false;
function hideSplash(){
  if(_splashHidden) return;
  _splashHidden = true;
  var el = document.getElementById('splash');
  if(!el) return;
  var wait = Math.max(0, 1400 - (Date.now() - _splashT0));
  setTimeout(function(){
    el.classList.add('fading');
    setTimeout(function(){ el.style.display='none'; }, 500);
  }, wait);
}
// Failsafe: never strand the user behind the splash if pywebviewready doesn't
// fire (e.g. opening the served page in a plain browser during development).
setTimeout(hideSplash, 6000);

// ── Interactive tutorial ───────────────────────────────────────────────────
// Spotlight walkthrough: dims everything except the current step's target
// (one element + a huge box-shadow, no canvas tricks), with a card explaining
// it. Steps can switch sidebar tabs so their target actually exists when
// measured. Replayable anytime from the Tutorial button in the titlebar;
// offered automatically exactly once on first launch (persisted via the
// Python-side settings file, NOT localStorage -- see get_app_settings).
var TOUR_STEPS = [
  {title:'Welcome to Drone Mission Planner',
   body:'Plan DJI waypoint missions on a real map and export them as WPML <b>.kmz</b> files your RC2 controller flies directly.<br><br>This tour takes about two minutes. Use the buttons or the <b>&#8592; &#8594;</b> arrow keys; Esc leaves at any point. Replay it anytime from the &#127891; Tutorial button up top.'},
  {title:'Four mission types', target:'.mission-type-grid', tab:'setup',
   body:'<b>Grid Survey</b> sweeps an area in parallel rows for mapping/photogrammetry. <b>Corridor</b> follows a path (road, river, dig transect). <b>Orbit</b> circles a point of interest. <b>Manual</b> places waypoints one by one.<br><br>Click one, then click corners on the map to define your shape.'},
  {title:'Drawing on the map', target:'#map', pad:-6,
   body:'While drawing: <b>click</b> places a point, <b>right-click</b> (or Backspace) undoes the last one, <b>double-click</b> or Enter finishes, Esc cancels.<br><br>The banner up top shows the traced distance live, and clicks snap onto imported KML/KMZ shapes when you have any.'},
  {title:'Find your site', target:'#map-search',
   body:'Search any place or address (free OpenStreetMap geocoder), or hit &#128205; to jump to your current location.'},
  {title:'Basemaps', target:'.leaflet-control-layers',
   body:'Switch between street, satellite (Esri and Google), topographic and dark basemaps here — plus toggles for each overlay (flight path, no-fly zones, GCPs, imports). Satellite coverage quality varies by region, so try both providers over your site.'},
  {title:'Bring your own shapes', target:'#btn-import',
   body:'Import <b>KML/KMZ</b> files from Google Earth, QGIS or another planner. Polygons can become grid areas, lines become corridor routes or raw waypoints — check the <b>Layers</b> tab after importing. A line that closes back on itself is detected and offered as an area.'},
  {title:'No-fly zones & ground control', target:'#sec-site-markup', tab:'setup',
   body:'Draw a <b>no-fly zone</b> inside a survey area and the flight path is cut around it — rows stop exactly at its edge and the aircraft is routed around rather than straight across it.<br><br><b>GCPs</b> mark where you\'ll place physical ground targets. After the flight, type in each one\'s surveyed coordinate and export a CSV for Pix4D / Metashape / WebODM.'},
  {title:'Flight parameters', target:'#sec-flight', tab:'setup',
   body:'<b>Altitude</b> is the big one — it sets the photo footprint and ground resolution (GSD), which drive photo spacing and flight time. The camera line under Aircraft &amp; camera updates live as you change it.'},
  {title:'Overlap, rotation & waypoint mode', target:'#sec-mission-specific', tab:'setup',
   body:'Overlap %s control photogrammetry quality. Grid rotation aligns rows to your area automatically (tweak or wind-align manually here).<br><br><b>Turn Only</b> mode needs the camera set to <b>interval shooting</b> manually before takeoff — the app shows the exact interval and speed to use. <b>Full</b> mode stops for every photo instead: simpler, but jittery on dense grids.'},
  {title:'Waypoints & layers', target:'#tabs',
   body:'After generating, the <b>Waypoints</b> tab lists every point — edit altitude, speed or gimbal per point, drag markers on the map, replay the flight at speed, or apply <b>terrain-following</b> altitude over sloped ground. <b>Layers</b> holds your imports.'},
  {title:'Live mission stats', target:'#wp-stats',
   body:'Total distance, realistic flight time (acceleration-aware, not just distance&divide;speed), photo count — and warnings when the mission needs multiple batteries or exceeds the safe per-file waypoint limit.'},
  {title:'Export & fly', target:'#tgroup-export',
   body:'<b>Export WPML</b> writes the .kmz your drone flies. <b>Export by Battery</b> splits a big mission into legs sized to your real usable endurance. <b>Upload to RC</b> pushes straight into a mission slot on a USB-connected RC2.'},
  {title:'Safety nets', target:'#tgroup-file',
   body:'<b>Save/Load</b> stores the whole project (mission, zones, GCPs, settings) as a file. Undo/redo (<b>Ctrl+Z / Ctrl+Y</b>) covers drawing, generating, imports and deletions.'},
  {title:'Before every flight',
   body:'&#128247; In Turn Only mode, set the camera&rsquo;s interval timer to the shown value — the app can&rsquo;t do that for you.<br>&#128267; Check the battery warnings in the stats bar.<br>&#128065; Keep visual line of sight and respect local regulations.<br><br>Good flying! Replay this tour anytime via &#127891; Tutorial.'},
];
var tourIdx = -1;
var tourActive = false;
function startTour(){
  if(drawMode) cancelDraw();
  closeDronePicker();
  tourActive = true;
  tourIdx = -1;
  document.getElementById('tour-overlay').classList.add('visible');
  window.addEventListener('resize', tourReposition);
  document.addEventListener('keydown', tourKeys, true);
  tourStep(1);
}
function tourKeys(e){
  if(!tourActive) return;
  if(e.key==='Escape'){ e.preventDefault(); e.stopPropagation(); endTour(); }
  else if(e.key==='ArrowRight' || e.key==='Enter'){ e.preventDefault(); e.stopPropagation(); tourStep(1); }
  else if(e.key==='ArrowLeft'){ e.preventDefault(); e.stopPropagation(); tourStep(-1); }
}
function tourStep(delta){
  var next = tourIdx + delta;
  if(next >= TOUR_STEPS.length){ endTour(); return; }
  if(next < 0) next = 0;
  tourIdx = next;
  var s = TOUR_STEPS[tourIdx];
  if(s.tab && activeTab !== s.tab) showTab(s.tab);
  document.getElementById('tour-title').innerHTML = s.title;
  document.getElementById('tour-body').innerHTML = s.body;
  document.getElementById('tour-count').textContent = (tourIdx+1)+' / '+TOUR_STEPS.length;
  document.getElementById('tour-back').disabled = tourIdx===0;
  document.getElementById('tour-next').innerHTML = (tourIdx===TOUR_STEPS.length-1) ? 'Finish &#10003;' : 'Next &#8594;';
  // The sidebar scrolls independently and is much taller than the window, so
  // several steps' targets start well below the fold -- bring the target into
  // view before measuring, or the spotlight lands off-screen on a target the
  // user can't see. Verified: #tab-content scrollHeight was 1877px against
  // ~796px visible.
  if(s.target){
    var tEl = document.querySelector(s.target);
    if(tEl && tEl.scrollIntoView){
      try{ tEl.scrollIntoView({block:'center', inline:'nearest'}); }catch(e){ tEl.scrollIntoView(); }
    }
  }
  // Position synchronously, then again shortly after to catch late layout
  // shifts. Deliberately not requestAnimationFrame: rAF never fires while the
  // window isn't compositing, which would leave the spotlight unpositioned.
  tourReposition();
  setTimeout(tourReposition, 90);
}
function tourReposition(){
  if(!tourActive || tourIdx<0) return;
  var s = TOUR_STEPS[tourIdx];
  var sp = document.getElementById('tour-spotlight');
  var card = document.getElementById('tour-card');
  var r = null;
  if(s.target){
    var el = document.querySelector(s.target);
    if(el){
      var b = el.getBoundingClientRect();
      if(b.width>0 && b.height>0) r = b;
    }
  }
  var vw = window.innerWidth, vh = window.innerHeight;
  var cw = card.offsetWidth || 308, ch = card.offsetHeight || 200;
  if(r){
    var pad = (s.pad!=null) ? s.pad : 8;
    sp.classList.remove('hidden-target');
    sp.style.top = (r.top-pad)+'px';
    sp.style.left = (r.left-pad)+'px';
    sp.style.width = (r.width+pad*2)+'px';
    sp.style.height = (r.height+pad*2)+'px';
    // Card: prefer beside the spotlight (right, then left), else below/above.
    var gap = 14, top, left;
    if(r.right + gap + cw < vw - 10){ left = r.right + gap; top = r.top; }
    else if(r.left - gap - cw > 10){ left = r.left - gap - cw; top = r.top; }
    else if(r.bottom + gap + ch < vh - 10){ left = r.left + r.width/2 - cw/2; top = r.bottom + gap; }
    else { left = r.left + r.width/2 - cw/2; top = r.top - gap - ch; }
    card.style.top = Math.max(10, Math.min(vh - ch - 10, top))+'px';
    card.style.left = Math.max(10, Math.min(vw - cw - 10, left))+'px';
  } else {
    sp.classList.add('hidden-target');
    // Park the collapsed spotlight at center so its giant shadow still dims
    // the whole screen for target-less (welcome/closing) steps.
    sp.style.top = (vh/2)+'px'; sp.style.left = (vw/2)+'px';
    sp.style.width = '0px'; sp.style.height = '0px';
    card.style.top = (vh/2 - ch/2)+'px';
    card.style.left = (vw/2 - cw/2)+'px';
  }
}
function endTour(){
  tourActive = false;
  document.getElementById('tour-overlay').classList.remove('visible');
  window.removeEventListener('resize', tourReposition);
  document.removeEventListener('keydown', tourKeys, true);
  if(window.pywebview) pywebview.api.set_app_setting('tutorialSeen', true);
}
var pendingTourOffer = false;
function offerTour(){
  document.getElementById('tour-welcome-overlay').classList.add('visible');
}
function closeTourWelcome(start){
  document.getElementById('tour-welcome-overlay').classList.remove('visible');
  if(window.pywebview) pywebview.api.set_app_setting('tutorialSeen', true);
  if(start) startTour();
}

// ── Init ───────────────────────────────────────────────────────────────────
function init(){
  Promise.all([pywebview.api.get_presets(), pywebview.api.get_app_settings()]).then(function(res){
    PRESETS = res[0];
    var vEl = document.getElementById('app-version');
    if(vEl && PRESETS.version) vEl.textContent = 'v' + PRESETS.version;
    var st = (res[1] && res[1].settings) || {};
    cfg = Object.assign({}, PRESETS.defaults);
    // Drone choice: Python settings file first (survives everything), then
    // localStorage (kept for projects saved before the settings file existed).
    var saved = st.drone || null;
    if(!saved){ try{ saved = localStorage.getItem('dmp_drone'); }catch(e){} }
    var firstRun = !st.tutorialSeen;
    hideSplash();
    if(saved && PRESETS.drones[saved]){
      setDrone(saved);
      if(firstRun) setTimeout(offerTour, 2000);
    } else {
      renderSetup();
      showDronePicker();
      // Don't stack the tour offer on top of the drone picker -- it follows
      // once the picker closes (see closeDronePicker).
      pendingTourOffer = firstRun;
    }
  });
}
window.addEventListener('pywebviewready', init);

// ── Drone picker — asked once at startup, remembered, editable anytime ─────
function showDronePicker(){
  var el = document.getElementById('drone-picker-list');
  el.innerHTML = Object.keys(PRESETS.drones).map(function(k){
    var d = PRESETS.drones[k];
    return '<div class="slot-row"><div class="slot-info"><div>'+d.label+'</div></div>' +
      '<button class="primary" onclick="setDrone(\''+k+'\');closeDronePicker()">Select</button></div>';
  }).join('');
  document.getElementById('drone-picker-overlay').classList.add('visible');
}
function closeDronePicker(){
  document.getElementById('drone-picker-overlay').classList.remove('visible');
  // First launch stacks two onboarding moments (drone pick, then tour offer) --
  // sequenced so they never sit on top of each other.
  if(pendingTourOffer){ pendingTourOffer=false; setTimeout(offerTour, 400); }
}

// ── Tabs ───────────────────────────────────────────────────────────────────
function showTab(name){
  activeTab = name;
  document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('active', t.dataset.tab===name));
  if(name==='setup') renderSetup();
  else if(name==='waypoints') renderWaypointsTab();
  else if(name==='layers') renderLayersTab();
}

// ── Setup tab ──────────────────────────────────────────────────────────────
function droneOptions(){
  return Object.keys(PRESETS.drones).map(k=>{
    var d=PRESETS.drones[k];
    return '<option value="'+k+'"'+(cfg.drone===k?' selected':'')+'>'+d.label+'</option>';
  }).join('');
}
function cameraOptions(){
  return Object.keys(PRESETS.cameras).map(k=>{
    var c=PRESETS.cameras[k];
    return '<option value="'+k+'"'+(cfg.camera===k?' selected':'')+'>'+c.label+'</option>';
  }).join('');
}
function cameraInfoLine(c){
  var diagMm=Math.sqrt(c.sensor_w*c.sensor_w+c.sensor_h*c.sensor_h);
  var fovD=2*Math.atan(diagMm/(2*c.focal))*180/Math.PI;
  var fw=footprintWH(c);
  var gsdCm=(c.altitude*c.sensor_w)/(c.img_w*c.focal)*100;
  return c.sensor_w+'&times;'+c.sensor_h+'mm sensor, '+c.focal+'mm &mdash; ~'+fovD.toFixed(0)+'&deg; diagonal FOV &middot; '+
    'footprint '+fw[0].toFixed(0)+'&times;'+fw[1].toFixed(0)+'m at '+c.altitude+'m &middot; GSD ~'+gsdCm.toFixed(1)+'cm/px';
}
function gimbalOptions(){
  return Object.keys(PRESETS.gimbals).map(k=>{
    var g=PRESETS.gimbals[k];
    return '<option value="'+k+'"'+(cfg.gimbalPreset===k?' selected':'')+'>'+g.label+' ('+g.pitch+'&deg;)</option>';
  }).join('');
}
function setWaypointMode(mode){
  cfg.waypointMode = mode;
  refreshEstimate(); renderSetup();
}
function setGimbalPreset(k){
  cfg.gimbalPreset=k;
  var g=PRESETS.gimbals[k];
  if(g && k!=='custom'){
    cfg.gimbalPitch=g.pitch;
    if(g.overlap){ cfg.forwardOverlap=g.overlap[0]; cfg.sideOverlap=g.overlap[1]; }
    // A preset fully determines whether this is a 3D (nadir+oblique) capture,
    // so switching presets can't leave 3D mapping silently on from a previous
    // choice -- which would make the Gimbal pitch field above a no-op.
    cfg.threeDMapping = !!g.threeD;
    if(g.threeD) cfg.obliqueGimbal = g.pitch;
  }
  refreshEstimate(); renderSetup();
}

function renderSetup(){
  if(activeTab!=='setup') return;
  var drone = PRESETS.drones[cfg.drone] || {};
  var gimbalNote = (PRESETS.gimbals[cfg.gimbalPreset] || {}).note || '';
  var el = document.getElementById('tab-content');
  var currentKind = pendingKind || 'grid'; // which mission-type section to keep open
  var op = function(kind){ return currentKind===kind ? ' open' : ''; };
  var badge = function(kind){ return currentKind===kind ? '<span class="kind-badge">current</span>' : ''; };

  var pendingHtml = '';
  if(pendingKind){
    var kindLabels = {grid:'Grid survey area', corridor:'Corridor route', orbit:'Orbit center'};
    var label = kindLabels[pendingKind] || pendingKind;
    var genLabel = pendingGenerated ? '&#8635; Regenerate mission' : 'Generate mission';
    pendingHtml =
      '<div class="panel-section pending"><h4>'+(pendingGenerated?'&#9989;':'&#9888;')+' '+(pendingGenerated?'Mission':'Pending')+': '+label+'</h4>' +
      '<div id="live-estimate"></div>' +
      '<div style="display:flex;gap:6px;margin-top:9px;">' +
        '<button class="primary" style="flex:1;" onclick="commitPending()">'+genLabel+'</button>' +
        '<button onclick="discardPending()">Discard</button>' +
      '</div>' +
      '<div class="hint">'+(pendingGenerated
        ? 'Adjust parameters below and click Regenerate to update this mission in place.'
        : 'Tune the parameters below, then generate. You can keep adjusting and re-generate as many times as you like.')+'</div>' +
      '</div>';
  }

  el.innerHTML =
    pendingHtml +

    // ── New mission ──
    '<div class="panel-section"><h4>New mission</h4>' +
    '<div class="field"><label>Mission name (export filename prefix)</label><input type="text" value="'+missionName+'" onchange="setMissionName(this.value)"></div>' +
    '<div class="mission-type-grid">' +
      missionTypeBtn('area','Grid Survey',MISSION_ICONS.area,'Sweep an area in parallel rows — mapping &amp; photogrammetry') +
      missionTypeBtn('route','Corridor',MISSION_ICONS.route,'Follow a path — road, river, pipeline, transect') +
      missionTypeBtn('orbit','Orbit',MISSION_ICONS.orbit,'Circle a point of interest, camera locked on it') +
      missionTypeBtn('manual','Manual',MISSION_ICONS.manual,'Place individual waypoints one by one') +
    '</div></div>' +

    // ── Site markup — no-fly holes and survey-control reference points ──
    '<div class="panel-section" id="sec-site-markup"><h4>Site markup</h4>' +
    '<button style="width:100%;" onclick="startDraw(\'exclude\')" title="Draw a hole inside a grid survey area that the flight path skips entirely">&#9888; Draw no-fly zone</button>' +
    '<button style="width:100%;margin-top:6px;" onclick="startDraw(\'gcp\')" title="Click roughly where you plan to place a physical ground marker before flying -- you\'ll refine the exact coordinate here once you\'ve measured it in the field.">&#128204; Place GCP</button>' +
    (exclusionZones.length ?
      '<div class="hint" style="margin-top:6px;">'+exclusionZones.length+' no-fly zone(s) active on the current grid area &mdash; '+
      '<a href="#" onclick="clearExclusionZones();return false;">clear all</a></div>' : '') +
    '<div class="field" style="margin-top:8px;"><label>Ground control points'+help('A GCP only helps georeferencing once its coordinate is precisely measured, not just clicked on a map. Workflow: place one here roughly where you plan to put a physical marker (checkerboard target, painted cross, survey nail) before flying, spread out for good coverage; after the flight, once you\'ve measured that marker\'s real position with something more accurate than this map -- RTK GPS, a total station, a good handheld unit -- come back, type the precise lat/lon/elevation into the fields below, and tick Surveyed. Only surveyed GCPs are worth feeding into Pix4D/Metashape/WebODM.')+
      '<span style="float:right;">'+
      (gcpPoints.length ? '<a href="#" onclick="clearGCPs();return false;">clear all</a>' : '')+'</span></label>' +
      '<div id="gcp-list"></div>' +
      (gcpPoints.length ? '<button style="width:100%;margin-top:6px;" onclick="exportGCPs()">&#11123; Export GCPs (.csv)</button>' : '') +
    '</div>' +
    '</div>' +

    // ── Core flight parameters (always visible — used by every mission type) ──
    '<div class="panel-section" id="sec-flight"><h4>Flight</h4>' +
    '<div class="field-row">' +
      '<div class="field"><label>Altitude (m AGL)</label><input type="number" value="'+cfg.altitude+'" onchange="cfg.altitude=parseFloat(this.value)||10;refreshEstimate()"></div>' +
      '<div class="field"><label>Speed (m/s)</label><input type="number" value="'+cfg.speed+'" onchange="cfg.speed=parseFloat(this.value)||1"></div>' +
    '</div>' +
    '<div class="field"><label>Delay at each waypoint (sec, 0=none)'+help('Only applies to Orbit, Manual, and the Overview lap -- those still take one discrete photo per waypoint, and the aircraft moves on once it considers that action done, which in real-world reports is roughly "shutter fired," not "confirmed written to the card." On a slow card, or shooting RAW/DNG, that can mean a skipped photo the mission never notices; 1-2s is usually enough for JPEG on a fast card, several seconds for RAW on a slow one. Grid/corridor missions don\'t stop per shot at all now (see Camera interval under Advanced), so this has no effect on those.')+'</label><input type="number" min="0" value="'+cfg.delayAtWaypoint+'" onchange="cfg.delayAtWaypoint=parseFloat(this.value)||0"></div>' +
    '</div>' +

    // ── Battery & endurance — drives automatic mission splitting ──
    '<div class="panel-section"><h4>Battery &amp; endurance</h4>' +
    '<div class="field"><label>Battery</label><select onchange="setBattery(this.value)">'+batteryOptions()+'</select></div>' +
    '<details><summary><span>Usable-time assumptions'+help('Rated flight times are windless lab-ideal figures. Real-world usable endurance is commonly 70-80% of rated, and standard practice reserves 20-30% battery for return-to-home/contingency -- default here is 75% x (1-30%) ~= 52% of the rated number.')+'</span><span></span></summary><div class="details-body">' +
      '<div class="field-row">' +
        '<div class="field"><label>Realistic-conditions factor</label><input type="number" step="0.05" min="0.1" max="1" value="'+cfg.realisticFactor+'" onchange="cfg.realisticFactor=parseFloat(this.value)||0.75;refreshEstimate()"></div>' +
        '<div class="field"><label>RTH/safety reserve</label><input type="number" step="0.05" min="0" max="0.6" value="'+cfg.reserveFraction+'" onchange="cfg.reserveFraction=parseFloat(this.value)||0.3;refreshEstimate()"></div>' +
      '</div>' +
    '</div></details>' +
    '<div class="hint" style="margin-top:8px;">Usable per battery: <b>~'+usableBatteryMinutes(cfg).toFixed(0)+' min</b> of the '+cfg.batteryMinutes+' min rated.</div>' +
    '</div>' +

    // ── Capture purpose / gimbal — kept prominent since it's science-driven ──
    '<div class="panel-section"><h4>Capture purpose</h4>' +
    '<div class="field"><label>What are you capturing?'+(gimbalNote?help(gimbalNote):'')+'</label><select onchange="setGimbalPreset(this.value)">'+gimbalOptions()+'</select></div>' +
    '<div class="field" style="margin-top:8px;"><label>Gimbal pitch <span style="float:right;color:var(--text-faint);">-90&deg;=down &middot; 0&deg;=horizon</span></label>' +
      '<input type="number" value="'+cfg.gimbalPitch+'" '+(cfg.threeDMapping?'disabled title="3D mapping is on, so this is unused -- the nadir pass is fixed at -90° and the oblique pass uses the separate Oblique pass gimbal pitch field under Grid survey settings."':'')+' onchange="cfg.gimbalPitch=parseFloat(this.value)||0;cfg.gimbalPreset=\'custom\';refreshEstimate()"></div>' +
    '</div>' +

    // ── Per-mission-type settings, collapsed except the currently relevant one ──
    '<div class="panel-section" id="sec-mission-specific"><h4>Mission-specific settings</h4>' +
    '<details'+op('grid')+' class="'+(currentKind==='grid'?'active-kind':'')+'"><summary>Grid survey'+badge('grid')+'</summary><div class="details-body">' +
      '<div class="field"><label>Waypoint mode'+help('Turn Only (recommended): sparse waypoints at each row\'s start/end only, camera fires on its own interval timer during continuous flight -- avoids the position-hold jitter and RC2 waypoint-count problems a dense mission can hit, but needs the camera manually set to Timer/interval mode before flight (see Camera interval under Advanced). Full: a real stop-and-shoot waypoint at every photo, no manual step needed -- matches how YMapper and Waypoint OS both default, and how a hand-made DJI Fly mission works -- but can cause visible jitter at close spacing (aircraft settling within hover-accuracy tolerance at every stop) and can hit the RC2\'s waypoint-count limit on a larger survey.')+'</label>' +
        '<div class="field-row" style="gap:6px;">' +
          '<button style="flex:1;" class="'+(cfg.waypointMode!=='full'?'active':'')+'" onclick="setWaypointMode(\'turnOnly\')">Turn Only</button>' +
          '<button style="flex:1;" class="'+(cfg.waypointMode==='full'?'active':'')+'" onclick="setWaypointMode(\'full\')">Full</button>' +
        '</div>' +
      '</div>' +
      '<div class="field-row" style="margin-top:8px;">' +
        '<div class="field"><label>Forward overlap %</label><input type="number" value="'+cfg.forwardOverlap+'" onchange="cfg.forwardOverlap=parseFloat(this.value)||0;refreshEstimate()"></div>' +
        '<div class="field"><label>Side overlap %</label><input type="number" value="'+cfg.sideOverlap+'" onchange="cfg.sideOverlap=parseFloat(this.value)||0;refreshEstimate()"></div>' +
      '</div>' +
      '<div class="field"><label>Grid rotation <span id="rot-val" style="color:var(--orange);float:right;">'+cfg.rotationDeg+'&deg;</span></label>' +
        '<input id="rot-slider" type="range" min="0" max="359" value="'+cfg.rotationDeg+'" style="width:100%;accent-color:var(--orange);" ' +
        'oninput="cfg.rotationDeg=parseFloat(this.value);document.getElementById(\'rot-val\').textContent=this.value+\'°\';refreshEstimate()"></div>' +
      '<button style="width:100%;margin-top:2px;" onclick="autoRotate()" title="Align the sweep to the area\'s longest edge, minimizing wasted transit distance">&#8635; Auto-rotate to minimize flight distance</button>' +
      '<div class="field" style="margin-top:6px;"><label>Wind from (&deg;, optional)</label><input id="wind-dir" type="number" min="0" max="359" placeholder="e.g. 270"></div>' +
      '<button style="width:100%;margin-top:2px;" onclick="rotateForWind()" title="Coverage-path research (e.g. Boustrophedon CPP for UAV surveys in wind) finds sweeping parallel to the wind, not across it, covers faster with steadier speed and less battery spent fighting a crosswind every pass. Enter the direction wind is coming FROM above, if you know it.">&#8634; Align to wind</button>' +
      '<div class="field" style="margin-top:8px;"><label>Turn style'+help(cfg.waypointMode==='full'
        ? 'In Full mode every waypoint is a photo stop, so this turn mode applies to all of them -- "Stop at each point" keeps camera position/GSD consistent (the standard choice for mapping) but can cause visible position-hold jitter at close spacing, since the aircraft actively settles within its own hover-accuracy tolerance at every single stop. Switch to Turn Only mode instead of just changing this if that jitter is the problem -- it removes the per-photo stops entirely rather than just softening them.'
        : 'In Turn Only mode, waypoints are only at each row\'s start/end, so this only affects that row-to-row turn, not individual photos -- stopping to reverse direction between rows is standard photogrammetry practice and won\'t cause the position-hold jitter a mid-row stop would. Smooth flythrough banks through the turn instead of stopping, saving a little time but overshooting the row start slightly.')+'</label><select onchange="cfg.turnMode=this.value">' +
        opt('toPointAndStopWithDiscontinuityCurvature',cfg.turnMode,'Stop at each point (precise — recommended for mapping)')+
        opt('toPointAndStopWithContinuityCurvature',cfg.turnMode,'Slow smooth turn, still stops')+
        opt('toPointAndPassWithContinuityCurvature',cfg.turnMode,'Smooth flythrough, never stops')+
      '</select></div>' +
      '<div class="checkbox-row" style="margin-top:8px;'+(cfg.threeDMapping?'opacity:.4;':'')+'"><input type="checkbox" id="cb-xh" '+(cfg.crosshatch?'checked':'')+(cfg.threeDMapping?' disabled':'')+' onchange="cfg.crosshatch=this.checked;refreshEstimate()"><label for="cb-xh">Crosshatch (double grid) for thorough coverage'+help(cfg.threeDMapping?'Superseded by 3D mapping below, which already flies two full passes -- crosshatch is ignored while it\'s on.':'Second pass at 90° to the first. Roughly doubles photo count and flight time but fills gaps a single sweep misses on irregular sites.')+'</label></div>' +
      '<div class="checkbox-row" style="margin-top:8px;"><input type="checkbox" id="cb-3d" '+(cfg.threeDMapping?'checked':'')+' onchange="cfg.threeDMapping=this.checked;if(this.checked)cfg.crosshatch=false;refreshEstimate();renderSetup()"><label for="cb-3d">3D mapping (nadir + oblique double-grid)'+help('Flies the area twice: once straight down, once tilted (rotated 90° from the first pass) -- the method DJI Terra/Pix4D document for full 3D reconstruction, since a pure-nadir pass never images vertical surfaces like walls. Roughly doubles photo count.')+'</label></div>' +
      (cfg.threeDMapping ?
        '<div class="field" style="margin-top:8px;"><label>Oblique pass gimbal pitch</label><input type="number" value="'+cfg.obliqueGimbal+'" onchange="cfg.obliqueGimbal=parseFloat(this.value)||-45;refreshEstimate()"></div>' : '') +
      '<details style="margin-top:8px;"><summary><span>Photo spacing override'+help('Larger numbers = fewer, more spread-out photos. Leave at 0 to derive spacing from overlap % instead.')+'</span><span></span></summary><div class="details-body">' +
        '<div class="field-row">' +
          '<div class="field"><label>Line spacing (m, 0=auto)</label><input type="number" value="'+cfg.sideSpacingOverride+'" onchange="cfg.sideSpacingOverride=parseFloat(this.value)||0;refreshEstimate()"></div>' +
          '<div class="field"><label>Photo spacing (m, 0=auto)</label><input type="number" value="'+cfg.forwardSpacingOverride+'" onchange="cfg.forwardSpacingOverride=parseFloat(this.value)||0;refreshEstimate()"></div>' +
        '</div>' +
      '</div></details>' +
      '<div class="checkbox-row" style="margin-top:8px;"><input type="checkbox" id="cb-ov" '+(cfg.overviewEnabled?'checked':'')+' onchange="cfg.overviewEnabled=this.checked;renderSetup()"><label for="cb-ov">Add a perimeter overview lap'+help('Quick lap around the boundary at a higher altitude, photo at every corner/mid-edge -- whole-site context in addition to the detailed grid.')+'</label></div>' +
      (cfg.overviewEnabled ?
        '<div class="field-row" style="margin-top:8px;">' +
          '<div class="field"><label>Overview altitude (m, 0=auto 1.5&times;)</label><input type="number" value="'+cfg.overviewAltitude+'" onchange="cfg.overviewAltitude=parseFloat(this.value)||0"></div>' +
          '<div class="field"><label>Overview gimbal pitch</label><input type="number" value="'+cfg.overviewGimbal+'" onchange="cfg.overviewGimbal=parseFloat(this.value)||-60"></div>' +
        '</div>' : '') +
    '</div></details>' +

    '<details'+op('corridor')+' class="'+(currentKind==='corridor'?'active-kind':'')+'"><summary>Corridor'+badge('corridor')+'</summary><div class="details-body">' +
      '<div class="field"><label>Waypoint mode'+help('Turn Only (recommended): sparse waypoints at each pass\'s start/end only, camera fires on its own interval timer -- avoids position-hold jitter and RC2 waypoint-count problems, but needs the camera manually set to Timer/interval mode before flight (see Camera interval under Advanced). Full: a real stop-and-shoot waypoint at every photo, no manual step needed, but can cause visible jitter at close spacing and hit the RC2\'s waypoint-count limit on a long route.')+'</label>' +
        '<div class="field-row" style="gap:6px;">' +
          '<button style="flex:1;" class="'+(cfg.waypointMode!=='full'?'active':'')+'" onclick="setWaypointMode(\'turnOnly\')">Turn Only</button>' +
          '<button style="flex:1;" class="'+(cfg.waypointMode==='full'?'active':'')+'" onclick="setWaypointMode(\'full\')">Full</button>' +
        '</div>' +
      '</div>' +
      '<div class="field-row" style="margin-top:8px;">' +
        '<div class="field"><label>Forward overlap %</label><input type="number" value="'+cfg.forwardOverlap+'" onchange="cfg.forwardOverlap=parseFloat(this.value)||0;refreshEstimate()"></div>' +
        '<div class="field"><label>Side overlap %</label><input type="number" value="'+cfg.sideOverlap+'" onchange="cfg.sideOverlap=parseFloat(this.value)||0;refreshEstimate()"></div>' +
      '</div>' +
      '<div class="field"><label>Corridor width (m)</label><input type="number" value="'+cfg.corridorWidth+'" onchange="cfg.corridorWidth=parseFloat(this.value)||0;refreshEstimate()"></div>' +
    '</div></details>' +

    '<details'+op('orbit')+' class="'+(currentKind==='orbit'?'active-kind':'')+'"><summary>Orbit'+badge('orbit')+'</summary><div class="details-body">' +
      '<div class="field-row">' +
        '<div class="field"><label>Orbit radius (m)</label><input type="number" value="'+cfg.orbitRadius+'" onchange="cfg.orbitRadius=parseFloat(this.value)||5;refreshEstimate()"></div>' +
        '<div class="field"><label>Orbit points</label><input type="number" value="'+cfg.orbitPoints+'" onchange="cfg.orbitPoints=parseInt(this.value)||8;refreshEstimate()"></div>' +
      '</div>' +
      '<div class="checkbox-row"><input type="checkbox" id="cb-cw" '+(cfg.orbitClockwise?'checked':'')+' onchange="cfg.orbitClockwise=this.checked"><label for="cb-cw">Orbit clockwise'+help('Gimbal continuously tracks the center point -- no fixed pitch needed here.')+'</label></div>' +
      '<div class="field" style="margin-top:8px;"><label>Turn style'+help('DJI Fly\'s continuity-curvature mode flies a smooth spline through the waypoints -- the only option here that actually looks and flies like a circle rather than a many-sided polygon with stop-and-rotate corners.')+'</label><select onchange="cfg.orbitTurnMode=this.value">' +
        opt('toPointAndPassWithContinuityCurvature',cfg.orbitTurnMode,'Smooth flythrough (recommended — flies an actual circle)')+
        opt('toPointAndStopWithContinuityCurvature',cfg.orbitTurnMode,'Slow smooth turn, still stops at each point')+
        opt('toPointAndStopWithDiscontinuityCurvature',cfg.orbitTurnMode,'Stop at each point (stuttering polygon, not a circle)')+
      '</select></div>' +
      '<div class="field" style="margin-top:8px;"><label>Altitude rings (1=single ring)</label><input type="number" min="1" max="8" value="'+cfg.orbitRings+'" onchange="cfg.orbitRings=parseInt(this.value)||1;refreshEstimate();renderSetup()"></div>' +
      (cfg.orbitRings>1 ?
        '<div class="field-row">' +
          '<div class="field"><label>Lowest ring altitude (m, 0=auto)</label><input type="number" value="'+cfg.orbitMinAltitude+'" onchange="cfg.orbitMinAltitude=parseFloat(this.value)||0;refreshEstimate()"></div>' +
          '<div class="field"><label>Highest ring altitude (m, 0=auto)'+help('Stacked rings at different altitudes around the same center -- recommended for full 3D reconstruction of a tall/complex object (tower, silo, monument), where a single ring only sees it from one elevation angle. Aim for >=30 photos per ring.')+'</label><input type="number" value="'+cfg.orbitMaxAltitude+'" onchange="cfg.orbitMaxAltitude=parseFloat(this.value)||0;refreshEstimate()"></div>' +
        '</div>'
        : '') +
    '</div></details>' +
    '</div>' +

    // ── Aircraft & camera — set once via the startup picker, rarely touched after ──
    '<div class="panel-section"><h4>Aircraft &amp; camera</h4>' +
    '<div class="field"><label>Drone</label><select onchange="setDrone(this.value)">'+droneOptions()+'</select></div>' +
    '<div class="field"><label>Camera</label><select onchange="setCamera(this.value)">'+cameraOptions()+'</select></div>' +
    '<div class="hint">'+cameraInfoLine(cfg)+'</div>' +
    '<details><summary>Advanced (raw WPML / sensor values)<span></span></summary><div class="details-body">' +
      '<div class="field-row">' +
        '<div class="field"><label>droneEnumValue</label><input type="number" value="'+cfg.droneEnumValue+'" onchange="cfg.droneEnumValue=parseInt(this.value)||0"></div>' +
        '<div class="field"><label>droneSubEnumValue</label><input type="number" value="'+cfg.droneSubEnumValue+'" onchange="cfg.droneSubEnumValue=parseInt(this.value)||0"></div>' +
      '</div>' +
      '<div class="field-row">' +
        '<div class="field"><label>Sensor width (mm)</label><input type="number" step="0.01" value="'+cfg.sensor_w+'" onchange="cfg.sensor_w=parseFloat(this.value)||1;refreshEstimate();renderSetup()"></div>' +
        '<div class="field"><label>Sensor height (mm)</label><input type="number" step="0.01" value="'+cfg.sensor_h+'" onchange="cfg.sensor_h=parseFloat(this.value)||1;refreshEstimate();renderSetup()"></div>' +
      '</div>' +
      '<div class="field-row">' +
        '<div class="field"><label>Focal length (mm)</label><input type="number" step="0.01" value="'+cfg.focal+'" onchange="cfg.focal=parseFloat(this.value)||1;refreshEstimate();renderSetup()"></div>' +
        '<div class="field"><label>Image width (px)</label><input type="number" value="'+cfg.img_w+'" onchange="cfg.img_w=parseInt(this.value)||1;refreshEstimate()"></div>' +
      '</div>' +
      '<div class="field"><label>Image height (px)</label><input type="number" value="'+cfg.img_h+'" onchange="cfg.img_h=parseInt(this.value)||1;refreshEstimate()"></div>' +
      '<div class="field" style="margin-top:8px;"><label>Accel/decel (m/s&sup2;)'+help('Used to estimate real flight time and battery-split points -- distance/speed alone assumes the aircraft is instantly at cruise speed and stops instantly, which overstates how fast a tightly-spaced stop-and-rotate grid actually flies. 1.4 m/s&sup2; is a measured average for a small quadcopter (Xu et al., 2021, MDPI Drones journal); lower it for a heavily-loaded aircraft, raise it if yours feels snappier in Sport-like modes.')+'</label><input type="number" step="0.1" min="0.1" value="'+cfg.droneAccel+'" onchange="cfg.droneAccel=parseFloat(this.value)||1.4;refreshEstimate()"></div>' +
      '<div class="field" style="margin-top:8px;"><label>Max waypoints per file'+help('DJI Fly currently caps a single mission file at 200 waypoints on the Mini 5 Pro and other current consumer models, but real-world reports describe the RC2\'s own mission UI destabilizing well before that on mapping-style missions with many closely-packed points -- exactly what a grid survey produces. 90 stays safely under both; lower it further if your RC2 still struggles, raise it (up to 200) if it handles more without issue.')+'</label><input type="number" min="1" max="200" value="'+cfg.maxWaypointsPerFile+'" onchange="cfg.maxWaypointsPerFile=parseInt(this.value)||90;refreshEstimate()"></div>' +
      '<div class="field" style="margin-top:8px;"><label>Camera interval (sec)'+help('Grid/corridor missions no longer stop at every photo (that caused position-hold jitter and RC2 instability -- see the Waypoints-tab warning after generating). Instead the camera\'s own Timer/interval-shooting mode fires the shutter throughout continuous flight, and this app derives cruise speed FROM this fixed interval so photos still land at the right spacing. It cannot set this on the camera for you -- WPML distance/time photo triggers aren\'t reliably supported on consumer DJI Fly (confirmed: documented as enterprise-drone-only, and real-world reports of it simply not working over RC2 KMZ import). You MUST set this manually on the camera before every flight. 2.0s matches HOT\'s drone-flightplan, a production tool used for real humanitarian drone mapping.')+'</label><input type="number" step="0.1" min="0.5" value="'+cfg.cameraInterval+'" onchange="cfg.cameraInterval=parseFloat(this.value)||2.0;refreshEstimate()"></div>' +
    '</div></details></div>' +

    // ── Safety & mission behaviour — sane defaults, rarely touched ──
    '<details style="margin:0;"><summary>&#9881; Safety &amp; mission behaviour<span></span></summary><div class="details-body">' +
      '<div class="field"><label>Fly-to-first-waypoint mode</label><select onchange="cfg.flyToWaylineMode=this.value">' +
        opt('safely',cfg.flyToWaylineMode,'Safely (climb then fly)')+opt('pointToPoint',cfg.flyToWaylineMode,'Point to point')+'</select></div>' +
      '<div class="field"><label>On mission finish</label><select onchange="cfg.finishAction=this.value">' +
        opt('goHome',cfg.finishAction,'Return to home')+opt('autoLand',cfg.finishAction,'Auto land')+
        opt('gotoFirstWaypoint',cfg.finishAction,'Go to first waypoint')+opt('noAction',cfg.finishAction,'No action')+'</select></div>' +
      '<div class="field"><label>If RC signal lost</label><select onchange="cfg.executeRCLostAction=this.value">' +
        opt('goBack',cfg.executeRCLostAction,'Return to home')+opt('landing',cfg.executeRCLostAction,'Land')+opt('hover',cfg.executeRCLostAction,'Hover')+'</select></div>' +
      '<div class="field-row">' +
        '<div class="field"><label>Takeoff safety height (m)</label><input type="number" value="'+cfg.takeOffSecurityHeight+'" onchange="cfg.takeOffSecurityHeight=parseFloat(this.value)||1.2"></div>' +
        '<div class="field"><label>Transitional speed (m/s)</label><input type="number" value="'+cfg.globalTransitionalSpeed+'" onchange="cfg.globalTransitionalSpeed=parseFloat(this.value)||1"></div>' +
      '</div>' +
      '<div class="field"><label>Altitude reference</label><select onchange="cfg.heightMode=this.value">' +
        opt('relativeToStartPoint',cfg.heightMode,'Relative to takeoff point (recommended)')+
        opt('EGM96',cfg.heightMode,'Sea level (EGM96)')+'</select></div>' +
    '</div></details>';
  renderGCPList();
  refreshEstimate();
}
// Matching line-art icons for the four mission types. These used to be three
// geometric text glyphs plus one color emoji (◻ ▮ ◎ 📌), which rendered at
// different weights/sizes and put a full-color pushpin next to flat outlines.
// One consistent 24x24 stroke set instead, sharing the app's accent color.
var MISSION_ICONS = {
  area:'<path d="M3 5h18v14H3z"/><path d="M3 9.7h18M3 14.3h18"/>',
  route:'<path d="M4 19c4 0 3-6 7-6s3-6 9-6"/><circle cx="4" cy="19" r="1.6" fill="currentColor" stroke="none"/><circle cx="20" cy="7" r="1.6" fill="currentColor" stroke="none"/>',
  orbit:'<ellipse cx="12" cy="12" rx="9" ry="5.2"/><circle cx="12" cy="12" r="2.2" fill="currentColor" stroke="none"/>',
  manual:'<circle cx="6" cy="7" r="1.9" fill="currentColor" stroke="none"/><circle cx="17" cy="10" r="1.9" fill="currentColor" stroke="none"/><circle cx="9" cy="18" r="1.9" fill="currentColor" stroke="none"/><path d="M6 7l11 3-8 8" stroke-dasharray="3 2.5"/>',
};
function missionTypeBtn(mode, label, icon, tip){
  // Modes map onto drawMode names except 'area'/'route', which produce grid
  // and corridor missions respectively -- highlight whichever is armed so the
  // sidebar shows what the map clicks are currently doing.
  var on = (drawMode===mode) ? ' active' : '';
  return '<button class="mission-type-btn'+on+'" onclick="startDraw(\''+mode+'\')" title="'+tip+'">' +
    '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round">'+icon+'</svg>' +
    '<span>'+label+'</span></button>';
}
function opt(val,cur,label){ return '<option value="'+val+'"'+(cur===val?' selected':'')+'>'+label+'</option>'; }
// A small hover-only "?" badge for the longer explanatory/citation text that used
// to sit permanently under a field as a paragraph -- native title tooltip, so it
// can't clip or overflow the sidebar the way a custom floating tooltip could in
// this narrow, deeply-nested layout (this app has hit that overflow bug twice).
function help(text){ return '<span class="help-icon" title="'+String(text).replace(/"/g,'&quot;')+'">?</span>'; }
function setDrone(k){
  cfg.drone=k; var d=PRESETS.drones[k];
  cfg.droneEnumValue=d.droneEnumValue; cfg.droneSubEnumValue=d.droneSubEnumValue;
  if(d.batteries && d.batteries.length){ cfg.batteryIdx=0; cfg.batteryMinutes=d.batteries[0].minutes; }
  // Persisted via the Python settings file, not just localStorage -- browser
  // storage is origin-scoped, and this page's origin includes the local
  // server's port, so it silently resets if the port ever changes.
  try{ localStorage.setItem('dmp_drone', k); }catch(e){}
  if(window.pywebview) pywebview.api.set_app_setting('drone', k);
  if(d.defaultCamera) setCamera(d.defaultCamera); else renderSetup();
}
function setCamera(k){
  cfg.camera=k; var c=PRESETS.cameras[k];
  cfg.sensor_w=c.sensor_w; cfg.sensor_h=c.sensor_h; cfg.focal=c.focal; cfg.img_w=c.img_w; cfg.img_h=c.img_h;
  refreshEstimate(); renderSetup();
}
function batteryOptions(){
  var d=PRESETS.drones[cfg.drone];
  if(!d || !d.batteries) return '';
  return d.batteries.map((b,i)=>'<option value="'+i+'"'+((cfg.batteryIdx||0)===i?' selected':'')+'>'+b.label+'</option>').join('');
}
function setBattery(idx){
  var d=PRESETS.drones[cfg.drone];
  cfg.batteryIdx=parseInt(idx);
  cfg.batteryMinutes=d.batteries[cfg.batteryIdx].minutes;
  refreshEstimate(); renderSetup();
}
function usableBatteryMinutes(c){
  var realistic=c.realisticFactor!=null?c.realisticFactor:0.75;
  var reserve=c.reserveFraction!=null?c.reserveFraction:0.30;
  return Math.max(1, (c.batteryMinutes||20) * realistic * (1-reserve));
}
function autoRotate(silent){
  var poly = (pendingKind==='grid') ? pendingGeom : null;
  if(!poly){ if(!silent) alert('Draw or select a grid area first, then Auto-rotate.'); return; }
  pywebview.api.optimal_rotation(poly, cfg).then(function(res){
    if(!res.ok){ if(!silent) alert('Could not compute rotation: '+res.msg); return; }
    cfg.rotationDeg = res.rotation;
    var slider=document.getElementById('rot-slider'), val=document.getElementById('rot-val');
    if(slider) slider.value=res.rotation;
    if(val) val.textContent=res.rotation+'°';
    refreshEstimate();
    if(silent) renderSetup();
  });
}
function rotateForWind(){
  var windEl = document.getElementById('wind-dir');
  var windDeg = parseFloat(windEl ? windEl.value : '');
  if(isNaN(windDeg)){ alert('Enter the direction the wind is coming FROM (0-359°) first.'); return; }
  // Coverage-path research on UAV surveys in wind finds sweeping the long
  // passes parallel to the wind axis (not across it) covers faster with
  // steadier ground speed than fighting a crosswind every pass. The grid's
  // pass direction runs along bearing (90 - rotationDeg), so solving for
  // rotationDeg that puts the pass bearing on the wind axis gives:
  var rotation = Math.round((((90 - windDeg) % 360 + 360) % 360) * 10) / 10;
  cfg.rotationDeg = rotation;
  var slider=document.getElementById('rot-slider'), val=document.getElementById('rot-val');
  if(slider) slider.value=rotation;
  if(val) val.textContent=rotation+'°';
  refreshEstimate();
}

// ── Drawing / mission creation ─────────────────────────────────────────────
function sanitizeMissionName(v){
  var cleaned = (v||'').trim().replace(/[\\/:*?"<>|]/g,'').replace(/\s+/g,'_');
  return cleaned || 'Mission';
}
function setMissionName(v){ missionName = sanitizeMissionName(v); }
function promptMissionName(){
  var name = window.prompt('Mission name (used as the export filename prefix):', missionName);
  if(name!==null && name.trim()!=='') missionName = sanitizeMissionName(name);
}
var drawHintBase = ''; // startDraw's per-mode instructions, kept separate from the live distance suffix appended in redrawTemp/mousemove
function startDraw(mode){
  if(mode!=='exclude' && mode!=='gcp') promptMissionName();
  drawMode = mode; tempPoints = [];
  tempGroup.clearLayers();
  // Double-click finishes the shape while drawing -- the map's own
  // double-click zoom would fight that, so it's parked until cancelDraw.
  map.doubleClickZoom.disable();
  var hint = document.getElementById('draw-hint');
  hint.classList.add('visible');
  document.getElementById('btn-finish').style.display = (mode==='orbit'||mode==='manual'||mode==='gcp') ? 'none' : 'inline-block';
  document.getElementById('btn-cancel').style.display = 'inline-block';
  var snapNote = importedLayers.length ? ' Clicks near an imported line/point snap to it.' : '';
  var keysNote = ' Right-click undoes a point · double-click or Enter finishes · Esc cancels.';
  if(mode==='area') drawHintBase='Click to add area corners (min. 3).'+keysNote+snapNote;
  if(mode==='route') drawHintBase='Click to add route points (min. 2).'+keysNote+snapNote;
  if(mode==='orbit') drawHintBase='Click on the map to place the orbit center. Esc cancels.'+snapNote;
  if(mode==='manual') drawHintBase='Click to add waypoints. Esc or "Cancel" stops.'+snapNote;
  if(mode==='exclude') drawHintBase='Click to add corners of a no-fly hole (min. 3).'+keysNote+snapNote;
  if(mode==='gcp') drawHintBase='Click to drop ground control points. Esc or "Cancel" stops.'+snapNote;
  hint.textContent = drawHintBase;
  if(activeTab==='setup') renderSetup(); // reflect the armed tool on the mission-type buttons
}
function removeLastTempPoint(){
  if(!(drawMode==='area' || drawMode==='route' || drawMode==='exclude')) return;
  if(!tempPoints.length) return;
  tempPoints.pop();
  redrawTemp();
  updateDrawHintDistance(null);
}
// Traced-so-far perimeter/path length, live while placing area/route/exclude
// points -- optionally plus one more live segment out to the cursor so it
// updates continuously between clicks, not just after each one.
function updateDrawHintDistance(cursorLatLng){
  if(!(drawMode==='area' || drawMode==='route' || drawMode==='exclude')) return;
  var hint = document.getElementById('draw-hint');
  if(tempPoints.length===0){ hint.textContent = drawHintBase; return; }
  var dist = 0;
  for(var i=1;i<tempPoints.length;i++){
    dist += haversine(tempPoints[i-1][0],tempPoints[i-1][1], tempPoints[i][0],tempPoints[i][1]);
  }
  var last = tempPoints[tempPoints.length-1];
  if(cursorLatLng) dist += haversine(last[0],last[1], cursorLatLng.lat,cursorLatLng.lng);
  var label = (drawMode==='area' ? 'Perimeter so far' : 'Length so far');
  hint.textContent = drawHintBase + '  —  ' + label + ': ' + (dist/1000).toFixed(2) + ' km';
}
function cancelDraw(){
  var wasDrawing = !!drawMode;
  drawMode = null; tempPoints = [];
  tempGroup.clearLayers();
  snapGroup.clearLayers();
  map.doubleClickZoom.enable();
  map.getContainer().style.cursor='';
  document.getElementById('draw-hint').classList.remove('visible');
  document.getElementById('btn-finish').style.display='none';
  document.getElementById('btn-cancel').style.display='none';
  // Clear the armed-tool highlight. finishDraw calls cancelDraw then showTab
  // ('setup') itself, so this only needs to cover a bare cancel.
  if(wasDrawing && activeTab==='setup') renderSetup();
}

// ── Snap-to-import: pull drawn points onto imported KML/KMZ vertices/edges ──
// Two radii on purpose: a vertex is an exact point worth landing on precisely,
// so it gets a bigger sticky radius; a position along an edge is inferred, so
// it only gets a small soft radius that won't yank the cursor across.
var SNAP_PX_VERTEX = 18;
var SNAP_PX_EDGE = 9;
// A vertex is only sticky if it's far enough from its neighbours to be a real
// corner. Without this, a GPS track or CAD path with a vertex every few metres
// would grab the cursor at every one, making it impossible to trace smoothly.
// Computed once per imported layer, not per mousemove -- it ignores zoom.
var VERTEX_ISOLATION_M = 15;
function computeIsolatedFlags(coords, closed){
  var n = coords.length;
  return coords.map(function(c, i){
    var prev = closed ? coords[(i - 1 + n) % n] : (i > 0 ? coords[i - 1] : null);
    var next = closed ? coords[(i + 1) % n] : (i < n - 1 ? coords[i + 1] : null);
    if(prev && haversine(c[0], c[1], prev[0], prev[1]) < VERTEX_ISOLATION_M) return false;
    if(next && haversine(c[0], c[1], next[0], next[1]) < VERTEX_ISOLATION_M) return false;
    return true;
  });
}
function closestPointOnSegment(p, a, b){
  var dx=b.x-a.x, dy=b.y-a.y, lenSq=dx*dx+dy*dy;
  if(lenSq===0) return a;
  var t=((p.x-a.x)*dx+(p.y-a.y)*dy)/lenSq;
  t=Math.max(0,Math.min(1,t));
  return L.point(a.x+t*dx, a.y+t*dy);
}
function findSnapPoint(latlng){
  if(!importedLayers.length && !tempPoints.length) return null;
  var clickPt = map.latLngToContainerPoint(latlng);
  var bestVertex=null, bestVertexDist=SNAP_PX_VERTEX;
  // Also snap to the user's own in-progress points -- each was a deliberate
  // click, so all get the sticky radius. This makes closing a loop land exactly
  // on the first point rather than merely near it.
  tempPoints.forEach(function(c){
    var p=map.latLngToContainerPoint([c[0],c[1]]);
    var d=p.distanceTo(clickPt);
    if(d<bestVertexDist){ bestVertexDist=d; bestVertex=[c[0],c[1]]; }
  });
  importedLayers.forEach(function(layer){
    if(layer.kind==='point'){
      var p=map.latLngToContainerPoint([layer.lat,layer.lon]);
      var d=p.distanceTo(clickPt);
      if(d<bestVertexDist){ bestVertexDist=d; bestVertex=[layer.lat,layer.lon]; }
      return;
    }
    layer.coords.forEach(function(c, i){
      if(layer.isolated && layer.isolated[i]===false) return;
      var p=map.latLngToContainerPoint([c[0],c[1]]);
      var d=p.distanceTo(clickPt);
      if(d<bestVertexDist){ bestVertexDist=d; bestVertex=[c[0],c[1]]; }
    });
  });
  if(bestVertex) return {point:bestVertex, onVertex:true};
  // No vertex close enough -- try a softer, smaller-radius snap onto the
  // nearest point along an edge instead.
  var bestEdge=null, bestEdgeDist=SNAP_PX_EDGE;
  importedLayers.forEach(function(layer){
    if(layer.kind==='point') return;
    var coords = layer.coords;
    if(coords.length<2) return;
    var edgeCount = layer.kind==='polygon' ? coords.length : coords.length-1;
    for(var i=0;i<edgeCount;i++){
      var a=coords[i], b=coords[(i+1)%coords.length];
      var pa=map.latLngToContainerPoint([a[0],a[1]]);
      var pb=map.latLngToContainerPoint([b[0],b[1]]);
      var proj=closestPointOnSegment(clickPt, pa, pb);
      var d=proj.distanceTo(clickPt);
      if(d<bestEdgeDist){
        bestEdgeDist=d;
        var ll=map.containerPointToLatLng(proj);
        bestEdge=[ll.lat,ll.lng];
      }
    }
  });
  return bestEdge ? {point:bestEdge, onVertex:false} : null;
}
function showSnapIndicator(latlng, onVertex){
  snapGroup.clearLayers();
  if(!latlng) return;
  if(onVertex) L.circleMarker(latlng,{radius:9,color:'#ff9500',weight:2,fillColor:'#ff9500',fillOpacity:.35}).addTo(snapGroup);
  else L.circleMarker(latlng,{radius:6,color:'#ff9500',weight:1.5,fillColor:'#ff9500',fillOpacity:.15,dashArray:'2,2'}).addTo(snapGroup);
}

map.on('mousemove', function(e){
  if(!drawMode){
    if(snapGroup.getLayers().length) snapGroup.clearLayers();
    map.getContainer().style.cursor='';
    return;
  }
  var snap=findSnapPoint(e.latlng);
  showSnapIndicator(snap ? L.latLng(snap.point[0],snap.point[1]) : null, snap && snap.onVertex);
  // A "+" cursor specifically for the soft line-snap case -- a cue that
  // clicking here inserts a point onto the line, distinct from the strong
  // pull of a vertex (which already has its own larger, filled indicator).
  map.getContainer().style.cursor = (snap && !snap.onVertex) ? 'crosshair' : '';
  updateDrawHintDistance(snap ? L.latLng(snap.point[0],snap.point[1]) : e.latlng);
});

map.on('click', function(e){
  if(!drawMode) return;
  var snap=findSnapPoint(e.latlng);
  var lat=snap?snap.point[0]:e.latlng.lat, lon=snap?snap.point[1]:e.latlng.lng;
  if(drawMode==='area' || drawMode==='route' || drawMode==='exclude'){
    tempPoints.push([lat,lon]);
    redrawTemp();
    updateDrawHintDistance(null);
  } else if(drawMode==='orbit'){
    pushHistory();
    pendingKind='orbit'; pendingGeom=[lat,lon]; pendingGenerated=false;
    cancelDraw();
    redrawPendingBoundary();
    showTab('setup');
  } else if(drawMode==='manual'){
    addManualWaypoint(lat,lon);
  } else if(drawMode==='gcp'){
    addGCP(lat,lon);
  }
});

// Right-click while drawing = undo the last placed point (the standard GIS
// digitizing gesture). The browser context menu is suppressed app-wide below,
// so this can't accidentally open it mid-trace.
map.on('contextmenu', function(e){
  if(drawMode) removeLastTempPoint();
});
// Double-click = finish the shape. A double-click also fires two single
// clicks first, which just placed two (nearly) coincident points at the
// same spot -- drop the duplicate tail before finishing so the shape doesn't
// carry a phantom zero-length edge.
map.on('dblclick', function(e){
  if(!(drawMode==='area' || drawMode==='route' || drawMode==='exclude')) return;
  while(tempPoints.length>=2){
    var a=tempPoints[tempPoints.length-2], b=tempPoints[tempPoints.length-1];
    if(haversine(a[0],a[1],b[0],b[1]) < 0.5) tempPoints.pop();
    else break;
  }
  finishDraw();
});
// This is a desktop app window, not a web page -- the browser's own
// right-click menu (Reload, Back...) reads as broken UI here and Reload
// would wipe the whole session's state.
document.addEventListener('contextmenu', function(e){ e.preventDefault(); });
// Drawing keyboard shortcuts. Kept separate from the Ctrl+Z/Y undo listener:
// these are plain keys, only active while a draw tool is armed, and never
// while typing in a field.
document.addEventListener('keydown', function(e){
  if(e.ctrlKey || e.metaKey || e.altKey) return;
  var tag = document.activeElement ? document.activeElement.tagName : '';
  if(tag==='INPUT' || tag==='TEXTAREA' || (document.activeElement && document.activeElement.isContentEditable)) return;
  if(!drawMode) return;
  if(e.key==='Escape'){ e.preventDefault(); cancelDraw(); }
  else if(e.key==='Enter' && (drawMode==='area'||drawMode==='route'||drawMode==='exclude')){ e.preventDefault(); finishDraw(); }
  else if(e.key==='Backspace' || e.key==='Delete'){ e.preventDefault(); removeLastTempPoint(); }
});

function redrawTemp(){
  tempGroup.clearLayers();
  if(tempPoints.length===0) return;
  if(drawMode==='area' || drawMode==='exclude'){
    var col = drawMode==='exclude' ? '#e0342e' : '#e07b00';
    if(tempPoints.length>=3) L.polygon(tempPoints,{color:col,fillOpacity:.15,weight:2}).addTo(tempGroup);
    else L.polyline(tempPoints,{color:col,weight:2,dashArray:'4,4'}).addTo(tempGroup);
  } else {
    L.polyline(tempPoints,{color:'#e07b00',weight:2}).addTo(tempGroup);
  }
  var mcol = drawMode==='exclude' ? '#e0342e' : '#e07b00';
  tempPoints.forEach(function(p){ L.circleMarker(p,{radius:5,color:'#000',weight:1,fillColor:mcol,fillOpacity:1}).addTo(tempGroup); });
}

function finishDraw(){
  pushHistory();
  if(drawMode==='area'){
    if(tempPoints.length<3){ alert('Add at least 3 points to define an area.'); return; }
    pendingKind='grid'; pendingGeom=tempPoints.slice(); pendingGenerated=false;
  } else if(drawMode==='route'){
    if(tempPoints.length<2){ alert('Add at least 2 points to define a route.'); return; }
    // The last point can land on the first just from snapping near a KMZ
    // vertex, so this asks rather than silently reclassifying. KML import is
    // different -- a closed LineString there is a saved fact, not a guess about
    // live intent -- so that path still auto-classifies.
    var pts = tempPoints.slice();
    var closedLoop = pts.length>=4 &&
      haversine(pts[0][0],pts[0][1], pts[pts.length-1][0],pts[pts.length-1][1]) < 2;
    if(closedLoop){
      var wantsArea = confirm('The last point landed back on the first, closing the loop. Did you mean to draw a closed AREA boundary for a grid survey instead of a corridor route?\n\nOK — treat as a grid area\nCancel — keep it as a corridor route');
      pts = pts.slice(0, -1); // drop the coincident closing point either way -- kept, it'd be a zero-length final corridor leg
      if(wantsArea){ pendingKind='grid'; pendingGeom=pts; pendingGenerated=false; }
      else { pendingKind='corridor'; pendingGeom=pts; pendingGenerated=false; }
    } else {
      pendingKind='corridor'; pendingGeom=pts; pendingGenerated=false;
    }
  } else if(drawMode==='exclude'){
    if(tempPoints.length<3){ alert('Add at least 3 points to define a no-fly hole.'); return; }
    exclusionZones.push({coords: tempPoints.slice()});
    redrawExclusionZones();
    refreshEstimate();
  }
  cancelDraw();
  if(pendingKind==='grid' || pendingKind==='corridor') redrawPendingBoundary();
  if(pendingKind==='grid') autoRotate(true); // hand-clicked corners are never a perfect rectangle in screen coords --
                                              // align the sweep to the polygon's own longest edge by default so rows
                                              // come out uniform regardless of how the shape is tilted, instead of
                                              // leaving whatever rotationDeg was left over from a previous mission.
  showTab('setup');
}

// ── No-fly / exclusion zones — holes a grid mission's coverage skips ───────
function redrawExclusionZones(){
  exclusionGroup.clearLayers();
  exclusionZones.forEach(function(z,i){
    L.polygon(z.coords,{color:'#e0342e',weight:2,fillColor:'#e0342e',fillOpacity:.25,dashArray:'6,4'})
      .bindTooltip('No-fly zone '+(i+1))
      .addTo(exclusionGroup);
  });
}
function clearExclusionZones(){
  if(!exclusionZones.length) return;
  if(!confirm('Remove all '+exclusionZones.length+' exclusion zone(s)?')) return;
  pushHistory();
  exclusionZones = [];
  redrawExclusionZones();
  refreshEstimate();
}

// ── Ground control points ───────────────────────────────────────────────────
// Workflow: mark roughly where each physical marker will go before flying;
// afterwards enter its surveyed lat/lon/elevation and tick "Surveyed". Only a
// surveyed GCP is worth feeding to Pix4D/Metashape/WebODM, which need X/Y/Z.
function addGCP(lat,lon){
  pushHistory();
  var label = 'GCP'+(gcpPoints.length+1);
  gcpPoints.push({lat:lat, lon:lon, elevation:0, notes:'', surveyed:false, label:label});
  redrawGCPs();
}
function redrawGCPs(){
  gcpGroup.clearLayers();
  gcpPoints.forEach(function(p,i){
    L.circleMarker([p.lat,p.lon],{radius:7,color:'#000',weight:1,
      fillColor: p.surveyed ? '#3fae4b' : '#2e8fe0',
      fillOpacity: p.surveyed ? 1 : 0.5,
      dashArray: p.surveyed ? null : '3,2'})
      .bindTooltip(p.label + (p.surveyed?' (surveyed)':' (planned)'), {permanent:true, direction:'top', offset:[0,-8], className:'gcp-label'})
      .on('click', function(){ renameGCP(i); })
      .addTo(gcpGroup);
  });
  renderGCPList();
}
function renameGCP(i){
  var name = window.prompt('Label for this GCP:', gcpPoints[i].label);
  if(name!==null && name.trim()!=='') gcpPoints[i].label = name.trim();
  redrawGCPs();
}
function updateGCP(i, field, value){
  var g = gcpPoints[i];
  if(!g) return;
  if(field==='lat' || field==='lon' || field==='elevation') g[field] = parseFloat(value) || 0;
  else if(field==='notes') g.notes = value;
  else if(field==='surveyed') g.surveyed = !!value;
  if(field==='lat' || field==='lon') redrawGCPs();
  else if(field==='surveyed') redrawGCPs();
}
function deleteGCP(i){
  pushHistory();
  gcpPoints.splice(i,1);
  redrawGCPs();
}
function clearGCPs(){
  if(!gcpPoints.length) return;
  if(!confirm('Remove all '+gcpPoints.length+' ground control point(s)?')) return;
  pushHistory();
  gcpPoints = [];
  redrawGCPs();
}
function renderGCPList(){
  var el = document.getElementById('gcp-list');
  if(!el) return;
  if(gcpPoints.length===0){ el.innerHTML='<div class="hint">None placed yet.</div>'; return; }
  el.innerHTML = gcpPoints.map(function(p,i){
    return '<div class="layer-item" style="margin-top:6px;">' +
      '<div class="name" style="justify-content:space-between;">' +
        '<span onclick="renameGCP('+i+')" style="cursor:pointer;" title="Click to rename">'+p.label+
          (p.surveyed?' <span style="color:var(--green);font-size:10px;">&#10003; surveyed</span>':' <span style="color:var(--text-faint);font-size:10px;">planned</span>')+'</span>' +
        '<span onclick="deleteGCP('+i+')" style="cursor:pointer;color:var(--red);font-weight:bold;opacity:.7;">&#10005;</span>' +
      '</div>' +
      '<div class="field-row" style="margin-top:4px;">' +
        '<div class="field"><label style="font-size:9.5px;">Lat</label><input type="number" step="0.000001" value="'+p.lat+'" onchange="updateGCP('+i+',\'lat\',this.value)"></div>' +
        '<div class="field"><label style="font-size:9.5px;">Lon</label><input type="number" step="0.000001" value="'+p.lon+'" onchange="updateGCP('+i+',\'lon\',this.value)"></div>' +
        '<div class="field"><label style="font-size:9.5px;">Elev (m)</label><input type="number" step="0.01" value="'+p.elevation+'" onchange="updateGCP('+i+',\'elevation\',this.value)"></div>' +
      '</div>' +
      '<div class="field" style="margin-top:4px;"><input type="text" placeholder="Notes -- e.g. painted cross, NE fence corner" value="'+(p.notes||'').replace(/"/g,'&quot;')+'" onchange="updateGCP('+i+',\'notes\',this.value)"></div>' +
      '<div class="checkbox-row" style="margin-top:4px;margin-bottom:0;"><input type="checkbox" id="gcp-surv-'+i+'" '+(p.surveyed?'checked':'')+' onchange="updateGCP('+i+',\'surveyed\',this.checked)"><label for="gcp-surv-'+i+'">Surveyed (precise measured coordinate, ready to export)</label></div>' +
    '</div>';
  }).join('');
}
function exportGCPs(){
  if(!gcpPoints.length){ alert('No ground control points placed yet.'); return; }
  var unsurveyed = gcpPoints.filter(function(p){ return !p.surveyed; }).length;
  if(unsurveyed && !confirm(unsurveyed+' of '+gcpPoints.length+' GCP(s) are still marked "planned," not "surveyed" -- their coordinates are just where you clicked on the map, not a precise measurement. Export anyway?')) return;
  pywebview.api.export_gcps(gcpPoints, buildExportFilename(waypoints)+'_gcps.csv').then(function(res){
    setStatus(res.msg); if(!res.ok && res.msg!=='Cancelled') alert(res.msg);
  });
}

function commitPending(){
  if(pendingKind==='grid') generateGridMission(pendingGeom);
  else if(pendingKind==='corridor') generateCorridorMission(pendingGeom);
  else if(pendingKind==='orbit') generateOrbitMission(pendingGeom);
}
function discardPending(){
  pushHistory();
  pendingKind=null; pendingGeom=null; pendingGenerated=false; redrawPendingBoundary(); renderSetup();
}

// A generate call that filtered out points for a marked no-fly zone hands back
// how many it dropped rather than silently keeping them out -- this asks before
// committing to that, and re-fetches an unfiltered result if the user would
// rather ignore the zone(s) for this particular mission.
function confirmExclusionDrop(res, regenerateWithoutExclusions){
  // Resolves with the whole res object (not just .waypoints) so callers can
  // also read estimated_photos -- dense photo count no longer equals
  // waypoint count for grid/corridor, see generate_grid's Python comment.
  if(!res.excluded_count){ return Promise.resolve(res); }
  var msg = res.excluded_count+' waypoint(s) fall inside a marked no-fly zone and were left out.\n\n'+
    'OK — keep them left out (recommended)\nCancel — ignore the no-fly zone(s) for this mission and generate it anyway';
  if(confirm(msg)) return Promise.resolve(res);
  return regenerateWithoutExclusions();
}

function mergeOverviewIfEnabled(polygonForOverview){
  if(!cfg.overviewEnabled || !polygonForOverview || polygonForOverview.length<3){
    return Promise.resolve();
  }
  var exclusions = exclusionZones.map(function(z){ return z.coords; });
  return pywebview.api.generate_overview(polygonForOverview, cfg, exclusions).then(function(res){
    if(!res.ok) return;
    return confirmExclusionDrop(res, function(){
      return pywebview.api.generate_overview(polygonForOverview, cfg, []);
    }).then(function(r){ waypoints = waypoints.concat(r.waypoints); });
  });
}

// Note: these don't clear pendingKind/pendingGeom, so the pending panel stays open
// after generating and can be tuned + regenerated in place.
function generateGridMission(polygon){
  pushHistory();
  showProgress('Generating grid...');
  var exclusions = exclusionZones.map(function(z){ return z.coords; });
  pywebview.api.generate_grid(polygon, cfg, exclusions).then(function(res){
    if(!res.ok){ hideProgress(); alert('Grid generation failed: '+res.msg); return; }
    confirmExclusionDrop(res, function(){
      return pywebview.api.generate_grid(polygon, cfg, []);
    }).then(function(r){
      waypoints = r.waypoints; lastEstimatedPhotos = r.estimated_photos||0;
      pois=[]; selectedWpIdx=null; pendingGenerated=true;
      mergeOverviewIfEnabled(polygon).then(function(){
        hideProgress();
        renderWaypoints(); showTab('waypoints');
        setStatus(waypoints.length+' waypoints (~'+lastEstimatedPhotos+' photos) generated');
      });
    });
  });
}
function generateCorridorMission(line){
  pushHistory();
  showProgress('Generating corridor...');
  var exclusions = exclusionZones.map(function(z){ return z.coords; });
  pywebview.api.generate_corridor(line, cfg, exclusions).then(function(res){
    if(!res.ok){ hideProgress(); alert('Corridor generation failed: '+res.msg); return; }
    confirmExclusionDrop(res, function(){
      return pywebview.api.generate_corridor(line, cfg, []);
    }).then(function(r){
      hideProgress();
      waypoints = r.waypoints; lastEstimatedPhotos = r.estimated_photos||0;
      pois=[]; selectedWpIdx=null; pendingGenerated=true;
      renderWaypoints(); showTab('waypoints');
      setStatus(waypoints.length+' waypoints (~'+lastEstimatedPhotos+' photos) generated');
    });
  });
}
function generateOrbitMission(center){
  pushHistory();
  showProgress('Generating orbit...');
  pywebview.api.generate_orbit(center, cfg).then(function(res){
    hideProgress();
    if(!res.ok){ alert('Orbit generation failed: '+res.msg); return; }
    waypoints = res.waypoints;
    pois = [{name:'Orbit center', lat:center[0], lon:center[1]}];
    selectedWpIdx=null; pendingGenerated=true;
    renderWaypoints(); showTab('waypoints'); setStatus(waypoints.length+' waypoints generated');
  });
}
function isInsideAnyExclusionZone(lat,lon){
  for(var i=0;i<exclusionZones.length;i++){
    if(pointInPolygonJS(lat,lon,exclusionZones[i].coords)) return true;
  }
  return false;
}
function addManualWaypoint(lat,lon){
  pushHistory();
  waypoints.push({lat:lat, lon:lon, alt:cfg.altitude, speed:cfg.speed, gimbal:cfg.gimbalPitch,
    heading_mode:'followWayline', heading_angle:0, photo:true, hover:cfg.delayAtWaypoint||0});
  renderWaypoints();
  if(activeTab==='waypoints') renderWaypointsTab();
  setStatus(waypoints.length+' waypoints');
  // Manual placement is a deliberate click, unlike a generated sweep, so this
  // warns rather than silently dropping or blocking it -- you may genuinely
  // want a waypoint inside a marked zone (e.g. re-checking why it's excluded).
  if(isInsideAnyExclusionZone(lat,lon)){
    alert('Heads up: this waypoint is inside a marked no-fly zone. It was placed anyway since you clicked there deliberately — delete it from the Waypoints tab if that was a mistake.');
  }
}

function clearMission(){
  if(waypoints.length && !confirm('Clear the current mission?')) return;
  pushHistory();
  waypoints=[]; pois=[]; selectedWpIdx=null;
  pendingKind=null; pendingGeom=null; pendingGenerated=false;
  redrawPendingBoundary();
  renderWaypoints();
  if(activeTab==='waypoints') renderWaypointsTab();
  if(activeTab==='setup') renderSetup();
  setStatus('Mission cleared');
}

// ── Waypoint map rendering ────────────────────────────────────────────────
function waypointTooltipHtml(wp,i){
  var heading = wp.heading_mode==='fixed' ? (wp.heading_angle||0)+'&deg; fixed' : (wp.heading_mode||'followWayline');
  return '<b>#'+(i+1)+'</b> &middot; '+wp.alt.toFixed(0)+'m &middot; '+(wp.speed||cfg.speed)+'m/s<br>' +
    'gimbal '+wp.gimbal+'&deg; &middot; heading '+heading+'<br>' +
    (wp.photo?'&#128247; photo':'no photo') + (wp.hover?' &middot; hover '+wp.hover+'s':'');
}
function renderWaypoints(){
  pauseReplay();
  replay.marker=null; replay.elapsed=0; replay.times=[0]; replay._curIdx=0; replay._curAlt=undefined;
  wpGroup.clearLayers(); wpMarkers={};
  if(wpPathLayer){ map.removeLayer(wpPathLayer); wpPathLayer=null; }
  var badge=document.getElementById('tab-wp-count');
  if(badge) badge.textContent = waypoints.length ? String(waypoints.length) : '';
  if(waypoints.length===0){ document.getElementById('wp-stats').innerHTML=''; return; }

  var latlngs = waypoints.map(w=>[w.lat,w.lon]);
  wpPathLayer = L.polyline(latlngs,{color:'#e07b00',weight:2,opacity:.8,dashArray:waypoints.length>60?'4,4':null}).addTo(map);

  waypoints.forEach(function(wp,i){
    var cls='wp-marker'+(i===0?' first':(i===waypoints.length-1?' last':''))+(i===selectedWpIdx?' selected':'');
    var icon=L.divIcon({className:'', html:'<div class="'+cls+'">'+(i+1)+'</div>', iconSize:[22,22], iconAnchor:[11,11]});
    var mk=L.marker([wp.lat,wp.lon],{icon:icon,draggable:true}).addTo(wpGroup);
    mk.bindTooltip(waypointTooltipHtml(wp,i), {className:'wp-tooltip', direction:'top', offset:[0,-12], opacity:1});
    mk.on('dragend', function(){
      var ll=mk.getLatLng(); wp.lat=ll.lat; wp.lon=ll.lng;
      renderWaypoints(); if(activeTab==='waypoints') renderWaypointsTab();
    });
    mk.on('click', function(){ selectWaypoint(i); });
    wpMarkers[i]=mk;
  });
  pois.forEach(function(p){
    L.marker([p.lat,p.lon],{icon:L.divIcon({className:'',html:'<div class="poi-marker"></div>',iconSize:[16,16],iconAnchor:[8,8]})})
      .bindTooltip(p.name).addTo(wpGroup);
  });
  updateStats();
}
function selectWaypoint(i){
  selectedWpIdx=i; renderWaypoints();
  if(activeTab==='waypoints') renderWaypointsTab();
  var row=document.getElementById('wp-row-'+i);
  if(row) row.scrollIntoView({behavior:'smooth',block:'nearest'});
}

// ── Waypoints tab ──────────────────────────────────────────────────────────
function renderWaypointsTab(){
  if(activeTab!=='waypoints') return;
  var el=document.getElementById('tab-content');
  if(waypoints.length===0){
    el.innerHTML='<div class="empty-hint">No waypoints yet.<br>Draw an area/route, set an orbit center, or use Manual Waypoints from the Setup tab.</div>';
    return;
  }
  var rows = waypoints.map(function(wp,i){
    return '<tr id="wp-row-'+i+'" class="'+(i===selectedWpIdx?'selected':'')+'" onclick="selectWaypoint('+i+')">' +
      '<td>'+(i+1)+'</td>' +
      '<td><input type="number" value="'+wp.alt.toFixed(1)+'" onclick="event.stopPropagation()" onchange="waypoints['+i+'].alt=parseFloat(this.value)||0;renderWaypoints()"></td>' +
      '<td><input type="number" value="'+wp.speed+'" onclick="event.stopPropagation()" onchange="waypoints['+i+'].speed=parseFloat(this.value)||1;renderWaypoints()"></td>' +
      '<td><input type="number" value="'+wp.gimbal+'" onclick="event.stopPropagation()" onchange="waypoints['+i+'].gimbal=parseFloat(this.value)||0;renderWaypoints()"></td>' +
      '<td><input type="checkbox" '+(wp.photo?'checked':'')+' onclick="event.stopPropagation()" onchange="waypoints['+i+'].photo=this.checked;renderWaypoints()"></td>' +
      '<td><span class="del-btn" onclick="event.stopPropagation();deleteWaypoint('+i+')">&#10005;</span></td>' +
    '</tr>';
  }).join('');
  var camReminder = '';
  if((pendingKind==='grid' || pendingKind==='corridor') && cfg.waypointMode!=='full'){
    var ci = cfg.cameraInterval||2.0;
    // Read the speed off an actual sparse (non-photo) waypoint -- waypoints[0]
    // could be an appended overview-lap point flying a different speed.
    var sparseWp = waypoints.find(w=>!w.photo && w.speed);
    var rowSpd = sparseWp ? sparseWp.speed : null;
    camReminder = '<div class="hint warn" style="margin-bottom:8px;padding:8px 10px;border:1px solid var(--orange-dim);border-radius:var(--radius-sm);background:#1c1300;">' +
      '&#128247; <b>Before you fly:</b> set the camera to Timer/interval shooting at <b>'+ci.toFixed(1)+'s</b>'+
      (rowSpd?' and confirm cruise speed is ~<b>'+rowSpd.toFixed(1)+' m/s</b>':'')+
      ' &mdash; these waypoints don\'t carry per-shot photo actions on purpose (see Camera interval under Setup &rarr; Advanced for why).</div>';
  }
  el.innerHTML = replayPanelHtml() + camReminder +
    '<div class="row" style="margin-bottom:8px;">' +
      '<button onclick="applyTerrainFollow()" title="Looks up ground elevation under every waypoint (needs internet) and shifts each altitude so real height above ground stays constant over sloped terrain, instead of a flat plane from the first waypoint">&#9968; Apply terrain-following altitude</button>' +
    '</div>' +
    '<table id="wp-table"><thead><tr><th>#</th><th>Alt(m)</th><th>Spd</th><th>Gimbal</th><th>Photo</th><th></th></tr></thead>' +
    '<tbody>'+rows+'</tbody></table>';
  updateReplayUI();
}
function applyTerrainFollow(){
  if(waypoints.length===0) return;
  if(!confirm('This looks up ground elevation for every waypoint online (SRTM data, ~30m resolution) and shifts each waypoint\'s altitude to hold a constant height above the actual ground instead of a flat plane from the first waypoint.\n\nIt overwrites the altitude values directly -- regenerate the mission to go back to flat AGL. Continue?')) return;
  var ref = waypoints[0];
  setStatus('Looking up terrain elevation...');
  pywebview.api.terrain_follow(waypoints, ref.lat, ref.lon).then(function(res){
    if(!res.ok){ alert(res.msg); setStatus(''); return; }
    res.altitudes.forEach(function(a,i){ waypoints[i].alt = a; });
    renderWaypoints(); renderWaypointsTab(); updateStats();
    setStatus(res.warn ? res.warn : 'Terrain-following altitude applied.');
    if(res.warn) alert(res.warn);
  });
}
function deleteWaypoint(i){
  pushHistory();
  waypoints.splice(i,1);
  if(selectedWpIdx===i) selectedWpIdx=null;
  renderWaypoints(); renderWaypointsTab();
}

function updateStats(){
  var dist=0;
  for(var i=1;i<waypoints.length;i++){ dist += haversine(waypoints[i-1].lat,waypoints[i-1].lon,waypoints[i].lat,waypoints[i].lon); }
  // Turn Only waypoints carry photo:false (the camera's interval timer takes
  // the shots), so their dense estimate is added back here. Full mode already
  // has photo:true per shot, where adding it would double-count.
  var isDenseKind = (pendingKind==='grid' || pendingKind==='corridor') && cfg.waypointMode!=='full';
  var photoCount = (isDenseKind ? lastEstimatedPhotos : 0) + waypoints.filter(w=>w.photo).length;
  var flightSec = computeFlightSeconds(waypoints, cfg);
  var mins = Math.floor(flightSec/60), secs = Math.round(flightSec%60);
  var usableSec = usableBatteryMinutes(cfg)*60;
  var battBatches = Math.max(1, Math.ceil(flightSec/usableSec));
  var wpBatches = Math.max(1, Math.ceil(waypoints.length/(cfg.maxWaypointsPerFile||90)));
  var batches = Math.max(battBatches, wpBatches);
  var battWarn = batches>1
    ? '<span style="color:var(--orange2)">&#128267; ~'+batches+' files needed ('+(wpBatches>battBatches?'waypoint-count limit':'battery')+') &mdash; use "Export by Battery" to split automatically</span>' : '';
  // DJI Fly caps a single mission file at 200 waypoints (current Mini 5 Pro
  // limit) and real-world reports describe the RC2's own mission UI getting
  // unstable well before that on mapping-style missions -- see
  // maxWaypointsPerFile's comment in DEFAULT_MISSION_CONFIG (Python).
  var warn = waypoints.length>(cfg.maxWaypointsPerFile||90)
    ? '<span style="color:var(--orange2)">&#9888; '+waypoints.length+' waypoints is over the safe per-file limit ('+(cfg.maxWaypointsPerFile||90)+') &mdash; use "Export by Battery" to split it, or lower overlap %/raise altitude to shrink it</span>' : '';
  // Distance/speed alone would say this mission is faster than it really is
  // whenever waypoints are close enough together (relative to cruise speed and
  // Accel/decel under Advanced) that the aircraft keeps stopping and re-
  // accelerating without ever reaching cruise speed -- flag it rather than
  // let the flight-time number silently be the only sign something's off.
  var naiveSec = dist/(cfg.speed||5);
  var accelWarn = (flightSec > naiveSec*1.3 && waypoints.length>3)
    ? '<span style="color:var(--orange2)">&#9888; Waypoints are closely spaced for the stop-and-rotate turn style &mdash; cruise speed is rarely reached, which is why flight time is well above straight distance&divide;speed. Raise altitude, lower overlap, or switch Turn style to smooth flythrough.</span>' : '';
  document.getElementById('wp-stats').innerHTML =
    '<span><b>'+waypoints.length+'</b> waypoints</span>' +
    '<span><b>'+(dist/1000).toFixed(2)+'</b> km</span>' +
    '<span><b>~'+mins+'m '+secs+'s</b> flight time</span>' +
    '<span><b>'+photoCount+'</b> photos</span>' + battWarn + warn + accelWarn;
}
function haversine(lat1,lon1,lat2,lon2){
  var R=6371000, toRad=d=>d*Math.PI/180;
  var dp=toRad(lat2-lat1), dl=toRad(lon2-lon1);
  var a=Math.sin(dp/2)**2+Math.cos(toRad(lat1))*Math.cos(toRad(lat2))*Math.sin(dl/2)**2;
  return 2*R*Math.asin(Math.sqrt(a));
}
function isStopTurn(mode){
  // Mirrors Python's _is_stop_turn -- every turn mode except smooth flythrough
  // brings the aircraft to a stop at the waypoint.
  return mode !== 'toPointAndPassWithContinuityCurvature';
}
function legTimeSec(distM, cruiseSpeed, accel, mustStop){
  // Mirrors Python's leg_time_sec exactly -- see droneAccel's comment in
  // DEFAULT_MISSION_CONFIG (Python) for why distance/speed alone undercounts
  // flight time, sometimes by several times over, for closely-spaced
  // waypoints with a stop-and-rotate turn mode.
  if(!cruiseSpeed || cruiseSpeed<=0) return 0;
  if(!mustStop) return distM/cruiseSpeed;
  accel = accel || 1.4;
  var dHalf = (cruiseSpeed*cruiseSpeed)/(2*accel);
  if(distM >= 2*dHalf) return 2*(cruiseSpeed/accel) + (distM-2*dHalf)/cruiseSpeed;
  return 2*Math.sqrt(distM/accel);
}
// Single source of truth for leg duration -- the stats bar, live estimate and
// replay clock all go through this, so they can't drift apart.
function legSecondsBetween(a, b, c){
  var accel = c.droneAccel||1.4, defaultTurn = c.turnMode||'toPointAndStopWithDiscontinuityCurvature';
  var speed = a.speed || c.speed || 5;
  var dist = haversine(a.lat,a.lon,b.lat,b.lon);
  var mustStop = isStopTurn(a.turn_mode||defaultTurn) || isStopTurn(b.turn_mode||defaultTurn);
  return legTimeSec(dist, speed, accel, mustStop);
}
// Shared, accel-aware flight-time calculation -- used everywhere flight time
// is estimated instead of separate dist/speed copies (each missing the same
// acceleration physics).
function computeFlightSeconds(wps, c){
  if(!wps || wps.length<2) return (wps&&wps[0]&&wps[0].hover)||0;
  var t = wps[0].hover||0;
  for(var i=1;i<wps.length;i++) t += legSecondsBetween(wps[i-1], wps[i], c) + (wps[i].hover||0);
  return t;
}

// ── Live mission replay (basic) ──────────────────────────────────────────
// Interpolates position/altitude between waypoints using each leg's speed, pausing
// at hover waypoints, so elapsed time roughly matches a real flight.
var replay = {playing:false, elapsed:0, totalTime:0, times:[0], rafId:null, wallStart:0,
              elapsedAtStart:0, speedMult:1, marker:null};

function replayPanelHtml(){
  if(waypoints.length<2) return '';
  return '<div id="replay-panel">' +
    '<div class="row">' +
      '<button id="replay-play" onclick="toggleReplay()">&#9654; Play</button>' +
      '<button onclick="stopReplay()">&#9632; Stop</button>' +
      '<select onchange="replay.speedMult=parseFloat(this.value)">' +
        '<option value="1">1&times;</option><option value="2">2&times;</option>' +
        '<option value="5">5&times;</option><option value="10" selected>10&times;</option>' +
        '<option value="30">30&times;</option>' +
      '</select>' +
    '</div>' +
    '<div class="row"><input type="range" id="replay-slider" min="0" max="1000" value="0" oninput="scrubReplay(this.value)"></div>' +
    '<div id="replay-info"></div>' +
  '</div>';
}

function prepareReplay(){
  // Same accel-aware model as the stats bar, so the replay clock and reported
  // flight time agree. Hover is attached to the leg's start here (its end in
  // computeFlightSeconds) so the marker keeps moving; the totals are identical.
  var times=[0], t=0;
  for(var i=1;i<waypoints.length;i++){
    t += legSecondsBetween(waypoints[i-1], waypoints[i], cfg) + (waypoints[i-1].hover||0);
    times.push(t);
  }
  t += waypoints[waypoints.length-1].hover||0;
  replay.times=times; replay.totalTime=t||1;
}

function ensureReplayMarker(){
  if(replay.marker) return;
  // A quadcopter glyph drawn from scratch -- U+1F501 (the repeat-arrows emoji)
  // reads as a "replay" button, not a drone, which is exactly backwards for a
  // marker whose whole job is to look like the aircraft flying the route.
  var droneSvg = '<svg width="22" height="22" viewBox="0 0 24 24">' +
    '<g stroke="var(--orange)" stroke-width="1.6" stroke-linecap="round">' +
      '<line x1="12" y1="12" x2="4" y2="4"/><line x1="12" y1="12" x2="20" y2="4"/>' +
      '<line x1="12" y1="12" x2="4" y2="20"/><line x1="12" y1="12" x2="20" y2="20"/>' +
    '</g>' +
    '<g fill="var(--orange)">' +
      '<circle cx="4" cy="4" r="3"/><circle cx="20" cy="4" r="3"/>' +
      '<circle cx="4" cy="20" r="3"/><circle cx="20" cy="20" r="3"/>' +
    '</g>' +
    '<rect x="8.5" y="8.5" width="7" height="7" rx="2" fill="#1a1a1a" stroke="var(--orange)" stroke-width="1.4"/>' +
  '</svg>';
  var icon=L.divIcon({className:'', html:'<div class="drone-marker">'+droneSvg+'</div>', iconSize:[26,26], iconAnchor:[13,13]});
  replay.marker=L.marker([waypoints[0].lat,waypoints[0].lon],{icon:icon,zIndexOffset:2000}).addTo(wpGroup);
}

function toggleReplay(){
  if(waypoints.length<2) return;
  if(replay.playing){ pauseReplay(); return; }
  if(replay.elapsed<=0 || replay.times.length!==waypoints.length) prepareReplay();
  if(replay.elapsed>=replay.totalTime) replay.elapsed=0;
  ensureReplayMarker();
  replay.playing=true;
  replay.wallStart=performance.now();
  replay.elapsedAtStart=replay.elapsed;
  replay.rafId=requestAnimationFrame(replayTick);
  updateReplayUI();
}
function pauseReplay(){
  replay.playing=false;
  if(replay.rafId) cancelAnimationFrame(replay.rafId);
  updateReplayUI();
}
function stopReplay(){
  pauseReplay();
  replay.elapsed=0;
  if(waypoints.length>=2){ prepareReplay(); setReplayMarkerAt(0); }
  updateReplayUI();
}
function scrubReplay(sliderVal){
  if(replay.times.length!==waypoints.length) prepareReplay();
  pauseReplay();
  ensureReplayMarker();
  replay.elapsed = (sliderVal/1000) * replay.totalTime;
  setReplayMarkerAt(replay.elapsed);
  updateReplayUI();
}
function replayTick(now){
  if(!replay.playing) return;
  var dt=(now-replay.wallStart)/1000*replay.speedMult;
  replay.elapsed = replay.elapsedAtStart + dt;
  if(replay.elapsed>=replay.totalTime){
    replay.elapsed=replay.totalTime;
    setReplayMarkerAt(replay.elapsed);
    pauseReplay();
    return;
  }
  setReplayMarkerAt(replay.elapsed);
  updateReplayUI();
  replay.rafId=requestAnimationFrame(replayTick);
}
function setReplayMarkerAt(elapsed){
  var times=replay.times;
  var i=0;
  while(i<times.length-1 && times[i+1]<elapsed) i++;
  var wpA=waypoints[i], wpB=waypoints[Math.min(i+1,waypoints.length-1)];
  var segStart=times[i], segEnd=times[Math.min(i+1,times.length-1)];
  var frac = segEnd>segStart ? Math.min(1,(elapsed-segStart)/(segEnd-segStart)) : 0;
  var lat=wpA.lat+(wpB.lat-wpA.lat)*frac;
  var lon=wpA.lon+(wpB.lon-wpA.lon)*frac;
  var alt=wpA.alt+(wpB.alt-wpA.alt)*frac;
  if(replay.marker) replay.marker.setLatLng([lat,lon]);
  replay._curIdx=i; replay._curAlt=alt;
}
function updateReplayUI(){
  var info=document.getElementById('replay-info');
  var btn=document.getElementById('replay-play');
  var slider=document.getElementById('replay-slider');
  if(!info) return;
  if(btn) btn.innerHTML = replay.playing ? '&#9208; Pause' : '&#9654; Play';
  if(slider) slider.value = replay.totalTime>0 ? Math.round((replay.elapsed/replay.totalTime)*1000) : 0;
  var idx=(replay._curIdx||0)+1;
  var alt=replay._curAlt!==undefined ? replay._curAlt.toFixed(0) : (waypoints[0]?waypoints[0].alt.toFixed(0):0);
  info.innerHTML = fmtTime(replay.elapsed)+' / '+fmtTime(replay.totalTime)+
    ' &middot; waypoint '+idx+'/'+waypoints.length+' &middot; alt <b>'+alt+'m</b>';
}
function fmtTime(s){ s=Math.max(0,s); var m=Math.floor(s/60), sec=Math.round(s%60); return m+':'+(sec<10?'0':'')+sec; }

// ── Layers tab (imported KML/KMZ) ────────────────────────────────────────
function importKml(){
  showProgress('Opening file...');
  pywebview.api.import_kml().then(function(res){
    hideProgress();
    if(!res.ok){ if(res.msg!=='Cancelled') alert(res.msg); setStatus(res.msg); return; }
    pushHistory();
    importedLayers = [];
    res.polygons.forEach(p=>importedLayers.push({kind:'polygon', name:p.name, coords:p.coords, isolated:computeIsolatedFlags(p.coords, true)}));
    res.lines.forEach(l=>importedLayers.push({kind:'line', name:l.name, coords:l.coords, isolated:computeIsolatedFlags(l.coords, false)}));
    res.points.forEach(pt=>importedLayers.push({kind:'point', name:pt.name, lat:pt.lat, lon:pt.lon}));
    drawImportedLayers();
    updateImportButton();
    showTab('layers');
    setStatus(res.msg);
  });
}
function clearImportedLayers(){
  if(!importedLayers.length) return;
  if(!confirm('Remove all '+importedLayers.length+' imported layer(s) from the map? You can re-import the file anytime.')) return;
  pushHistory();
  importedLayers = [];
  importedGroup.clearLayers();
  updateImportButton();
  renderLayersTab();
  setStatus('Imported layers cleared.');
}
function updateImportButton(){
  var btn = document.getElementById('btn-import');
  if(!btn) return;
  if(importedLayers.length){
    btn.textContent = '✖ Clear KML/KMZ';
    btn.setAttribute('onclick', 'clearImportedLayers()');
    btn.title = 'Remove the '+importedLayers.length+' imported layer(s) currently on the map';
  } else {
    btn.innerHTML = '&#128193; Import KML/KMZ';
    btn.setAttribute('onclick', 'importKml()');
    btn.title = '';
  }
}
function drawImportedLayers(fitBounds){
  if(fitBounds===undefined) fitBounds=true; // default true for the normal "just imported a file" case; undo/redo passes false so it doesn't yank the view
  importedGroup.clearLayers();
  var bounds=[];
  importedLayers.forEach(function(layer){
    if(layer.kind==='polygon'){
      L.polygon(layer.coords,{color:'#4488ff',weight:2,fillOpacity:.1}).addTo(importedGroup);
      layer.coords.forEach(c=>bounds.push(c));
    } else if(layer.kind==='line'){
      L.polyline(layer.coords,{color:'#4488ff',weight:2}).addTo(importedGroup);
      layer.coords.forEach(c=>bounds.push(c));
    } else if(layer.kind==='point'){
      L.circleMarker([layer.lat,layer.lon],{radius:6,color:'#000',weight:1,fillColor:'#4488ff',fillOpacity:1}).addTo(importedGroup);
      bounds.push([layer.lat,layer.lon]);
    }
  });
  if(fitBounds && bounds.length) map.fitBounds(bounds, {padding:[40,40]});
}
function renderLayersTab(){
  if(activeTab!=='layers') return;
  var el=document.getElementById('tab-content');
  if(importedLayers.length===0){
    el.innerHTML='<div class="empty-hint">No imported layers.<br>Use <b>Import KML/KMZ</b> in the toolbar to bring in areas, routes or points from another tool.</div>';
    return;
  }
  el.innerHTML = '<div id="layers-list">' + importedLayers.map(function(layer,i){
    var kindLabel = layer.kind==='polygon'?'Area':(layer.kind==='line'?'Route':'Point');
    var actions='';
    if(layer.kind==='polygon') actions='<button onclick="useLayerAsGrid('+i+')">Use as grid area</button>';
    else if(layer.kind==='line') actions='<button onclick="useLayerAsCorridor('+i+')">Use as corridor route</button><button onclick="useLayerAsWaypoints('+i+')">Import as waypoints</button>';
    else actions='<button onclick="useLayerAsOrbit('+i+')">Use as orbit center</button>';
    return '<div class="layer-item"><div class="name">'+kindLabel+': '+layer.name+'</div><div class="actions">'+actions+'</div></div>';
  }).join('') + '</div>';
}
function useLayerAsGrid(i){ pushHistory(); pendingKind='grid'; pendingGeom=importedLayers[i].coords.slice(); pendingGenerated=false; autoRotate(true); showTab('setup'); }
function useLayerAsCorridor(i){ pushHistory(); pendingKind='corridor'; pendingGeom=importedLayers[i].coords.slice(); pendingGenerated=false; showTab('setup'); }
function useLayerAsOrbit(i){ pushHistory(); pendingKind='orbit'; pendingGeom=[importedLayers[i].lat, importedLayers[i].lon]; pendingGenerated=false; showTab('setup'); }
function useLayerAsWaypoints(i){
  pushHistory();
  var coords = importedLayers[i].coords;
  waypoints = coords.map(function(c){
    return {lat:c[0], lon:c[1], alt:cfg.altitude, speed:cfg.speed, gimbal:cfg.gimbalPitch,
            heading_mode:'followWayline', heading_angle:0, photo:false, hover:0};
  });
  pois=[]; selectedWpIdx=null;
  renderWaypoints(); showTab('waypoints'); setStatus(waypoints.length+' waypoints imported from route');
}

// ── Export / Save / Load ─────────────────────────────────────────────────
// Filename encodes the mission at a glance: {name}_{flightTime}_{photoCount}p_{drone}
// e.g. Site5_13m26s_130p_mini_5_pro.kmz
function droneSlug(){
  var d=PRESETS.drones[cfg.drone];
  var label = d ? d.label : (cfg.drone||'drone');
  return label.replace(/^DJI\s+/i,'').trim().toLowerCase().replace(/[^a-z0-9]+/g,'_').replace(/^_+|_+$/g,'');
}
function timestampTag(){
  var d = new Date();
  var pad = n => String(n).padStart(2,'0');
  return d.getFullYear()+pad(d.getMonth()+1)+pad(d.getDate())+'-'+pad(d.getHours())+pad(d.getMinutes());
}
function buildExportFilename(wps){
  var flightSec = computeFlightSeconds(wps, cfg);
  var mins=Math.floor(flightSec/60), secs=Math.round(flightSec%60);
  // Exact for a whole-mission Turn Only export; a battery-split batch falls
  // back to counting photo:true (undercounts Turn Only batches) rather than
  // apportioning the dense estimate. Full mode always counts photo:true.
  var isDenseKind = (pendingKind==='grid' || pendingKind==='corridor') && cfg.waypointMode!=='full' && wps===waypoints;
  var photoCount = (isDenseKind ? lastEstimatedPhotos : 0) + wps.filter(w=>w.photo).length;
  return sanitizeMissionName(missionName)+'_'+timestampTag()+'_'+mins+'m'+secs+'s_'+photoCount+'p_'+droneSlug();
}
function exportWpml(){
  if(waypoints.length===0){ alert('No waypoints in the current mission.'); return; }
  showProgress('Exporting...');
  pywebview.api.export_wpml(cfg, waypoints, buildExportFilename(waypoints)+'.kmz').then(function(res){
    hideProgress();
    setStatus(res.msg);
    if(!res.ok && res.msg!=='Cancelled') alert(res.msg);
  });
}
function exportWpmlSplit(){
  if(waypoints.length===0){ alert('No waypoints in the current mission.'); return; }
  showProgress('Splitting by battery and exporting...');
  pywebview.api.export_wpml_split(cfg, waypoints, buildExportFilename(waypoints)+'.kmz').then(function(res){
    hideProgress();
    setStatus(res.msg);
    if(!res.ok && res.msg!=='Cancelled') alert(res.msg);
  });
}
// A schematic preview rather than a map screenshot: Leaflet's raster tiles
// can't be read back into a canvas without permissive CORS headers, and the
// numbered markers are HTML divIcons no canvas capture can rasterise. Still
// shows the route shape and waypoint count, which is what identifies a mission.
function buildPreviewImageDataUrl(wps){
  if(!wps || !wps.length) return null;
  var W=800, H=600, pad=70;
  var canvas=document.createElement('canvas');
  canvas.width=W; canvas.height=H;
  var ctx=canvas.getContext('2d');
  ctx.fillStyle='#eee6d6'; ctx.fillRect(0,0,W,H);
  ctx.strokeStyle='#ddd2ba'; ctx.lineWidth=1;
  for(var gx=0; gx<W; gx+=40){ ctx.beginPath(); ctx.moveTo(gx,0); ctx.lineTo(gx,H); ctx.stroke(); }
  for(var gy=0; gy<H; gy+=40){ ctx.beginPath(); ctx.moveTo(0,gy); ctx.lineTo(W,gy); ctx.stroke(); }

  var lats=wps.map(w=>w.lat), lons=wps.map(w=>w.lon);
  var minLat=Math.min.apply(null,lats), maxLat=Math.max.apply(null,lats);
  var minLon=Math.min.apply(null,lons), maxLon=Math.max.apply(null,lons);
  var cosLat=Math.cos((minLat+maxLat)/2*Math.PI/180);
  var spanX=Math.max((maxLon-minLon)*cosLat, 1e-7), spanY=Math.max(maxLat-minLat, 1e-7);
  var scale=Math.min((W-2*pad)/spanX, (H-2*pad)/spanY);
  function project(lat,lon){
    var x = W/2 + (lon-(minLon+maxLon)/2)*cosLat*scale;
    var y = H/2 - (lat-(minLat+maxLat)/2)*scale;
    return [x,y];
  }

  ctx.strokeStyle='#e07b00'; ctx.lineWidth=3; ctx.lineJoin='round';
  ctx.beginPath();
  wps.forEach(function(w,i){ var p=project(w.lat,w.lon); if(i===0) ctx.moveTo(p[0],p[1]); else ctx.lineTo(p[0],p[1]); });
  ctx.stroke();

  var dense = wps.length > 25;
  wps.forEach(function(w,i){
    var p=project(w.lat,w.lon);
    var color = i===0 ? '#3fae4b' : (i===wps.length-1 ? '#c0392b' : '#e07b00');
    var r = dense ? 4 : 14;
    ctx.beginPath(); ctx.arc(p[0],p[1],r,0,Math.PI*2);
    ctx.fillStyle=color; ctx.fill(); ctx.lineWidth=2; ctx.strokeStyle='#000'; ctx.stroke();
    if(!dense){
      ctx.fillStyle='#000'; ctx.font='bold 12px sans-serif'; ctx.textAlign='center'; ctx.textBaseline='middle';
      ctx.fillText(String(i+1), p[0], p[1]);
    }
  });

  ctx.fillStyle='#000c'; ctx.fillRect(0,H-30,W,30);
  ctx.fillStyle='#fff'; ctx.font='13px sans-serif'; ctx.textAlign='left'; ctx.textBaseline='middle';
  ctx.fillText((missionName||'Mission')+' — '+wps.length+' waypoints', 12, H-15);

  return canvas.toDataURL('image/jpeg', 0.85);
}

// ── Upload to RC — slot picker + live terminal-style log ────────────────────
// The log lines shown here come live from Python via evaluate_js as each step
// happens (device lookup, per-slot reads, delete, copy, verify) — not just a
// summary dumped at the end. Useful for the user to see it isn't just hung, and
// for diagnosing exactly which step failed on a flaky MTP connection.
function appendUploadLog(msg, isErr){
  var log = document.getElementById('picker-log');
  log.classList.add('visible');
  var line = document.createElement('div');
  if(isErr) line.className='err';
  line.textContent = '> ' + msg;
  log.appendChild(line);
  log.scrollTop = log.scrollHeight;
}
function currentMissionBatteries(){
  var flightSec = computeFlightSeconds(waypoints, cfg);
  var usableSec = usableBatteryMinutes(cfg)*60;
  var battBatches = Math.max(1, Math.ceil(flightSec/usableSec));
  var wpBatches = Math.max(1, Math.ceil(waypoints.length/(cfg.maxWaypointsPerFile||90)));
  return {minutes: flightSec/60, batteries: Math.max(battBatches, wpBatches)};
}
function openUploadPicker(){
  if(waypoints.length===0){ alert('No waypoints in the current mission.'); return; }
  document.getElementById('picker-overlay').classList.add('visible');
  document.getElementById('picker-log').innerHTML=''; document.getElementById('picker-log').classList.remove('visible');
  document.getElementById('picker-list').innerHTML='<div class="empty-hint">Scanning the controller...</div>';
  var battInfo = currentMissionBatteries();
  var wpOverLimit = waypoints.length > (cfg.maxWaypointsPerFile||90);
  var warnEl = document.getElementById('picker-battery-warn');
  if(battInfo.batteries>1){
    warnEl.classList.add('visible');
    var reason = wpOverLimit
      ? waypoints.length+' waypoints is over the safe per-file limit for the RC2\'s own mission UI'
      : '~'+Math.round(battInfo.minutes)+' min needs more than one battery';
    warnEl.innerHTML = '&#9888; This mission needs <b>'+battInfo.batteries+
      '</b> separate files ('+reason+'). Uploading it as-is to one slot risks the drone not finishing it, or the RC2 struggling to load it. '+
      'Close this and use <b>Export by Battery</b> instead to split it into '+battInfo.batteries+
      ' separate missions, then upload each one to its own slot.';
  } else {
    warnEl.classList.remove('visible'); warnEl.innerHTML='';
  }
  pywebview.api.list_rc_missions().then(function(res){
    if(!res.ok){
      document.getElementById('picker-list').innerHTML =
        '<div class="empty-hint">Could not read the controller:<br><b style="color:var(--red)">'+res.msg+'</b><br><br>'+
        'ADB does not work on the RC2 (deliberately blocked in firmware) — this uses MTP only. '+
        'If this keeps failing, fall back to the manual method in the README.</div>';
      return;
    }
    renderMissionSlots(res.slots);
  });
}
function closeUploadPicker(){ document.getElementById('picker-overlay').classList.remove('visible'); }
function renderMissionSlots(slots){
  var el = document.getElementById('picker-list');
  if(!slots.length){ el.innerHTML='<div class="empty-hint">No mission slots found.</div>'; return; }
  el.innerHTML = slots.map(function(s){
    var meta = [];
    if(s.modified) meta.push(s.modified);
    if(s.waypoints) meta.push('<b>'+s.waypoints+'</b> waypoints');
    if(s.lat!=null && s.lon!=null) meta.push(s.lat.toFixed(4)+', '+s.lon.toFixed(4));
    var title = s.name || '(unnamed mission)';
    return '<div class="slot-row">' +
      '<div class="slot-info"><div>'+title+'</div>' +
        '<div class="meta">'+(meta.join(' &middot; ')||'no details read')+'</div>' +
        '<div class="uuid">'+s.uuid+'</div></div>' +
      '<button class="primary" onclick="confirmUploadToSlot(\''+s.uuid+'\')">Replace</button>' +
    '</div>';
  }).join('');
}
function confirmUploadToSlot(uuid){
  if(!confirm('Overwrite mission slot '+uuid.slice(0,8)+'... on the controller with the current mission? This cannot be undone.')) return;
  document.getElementById('picker-list').innerHTML='';
  document.getElementById('picker-log').innerHTML=''; document.getElementById('picker-log').classList.add('visible');
  var previewUrl = buildPreviewImageDataUrl(waypoints);
  pywebview.api.upload_to_rc_slot(cfg, waypoints, uuid, 'DJI', previewUrl).then(function(res){
    setStatus(res.msg);
    if(res.ok){
      appendUploadLog('Done. '+res.msg);
      setTimeout(closeUploadPicker, 1500);
    } else {
      appendUploadLog('FAILED: '+res.msg, true);
    }
  });
}
function saveProject(){
  var data = JSON.stringify({cfg:cfg, waypoints:waypoints, pois:pois,
    exclusionZones:exclusionZones, gcpPoints:gcpPoints}, null, 2);
  pywebview.api.save_project(data).then(function(res){ setStatus(res.msg); if(!res.ok && res.msg!=='Cancelled') alert(res.msg); });
}
function loadProject(){
  pywebview.api.load_project().then(function(res){
    if(!res.ok){ if(res.msg!=='Cancelled') alert(res.msg); return; }
    try{
      var data = JSON.parse(res.data);
      pushHistory();
      cfg = Object.assign({}, PRESETS.defaults, data.cfg||{});
      waypoints = data.waypoints||[];
      pois = data.pois||[];
      exclusionZones = data.exclusionZones||[];
      gcpPoints = data.gcpPoints||[];
      selectedWpIdx=null;
      pendingKind=null; pendingGeom=null; pendingGenerated=false;
      redrawExclusionZones(); redrawGCPs(); redrawPendingBoundary();
      renderWaypoints(); renderSetup(); showTab('waypoints'); setStatus(res.msg);
    }catch(e){ alert('Invalid project file: '+e); }
  });
}

// ── Misc UI ────────────────────────────────────────────────────────────────
function setStatus(msg){ document.getElementById('status-bar').textContent=msg; }
function showProgress(msg){ document.getElementById('progress-msg').textContent=msg; document.getElementById('progress-overlay').classList.add('visible'); }
function hideProgress(){ document.getElementById('progress-overlay').classList.remove('visible'); }
</script>
</body>
</html>"""

# ── Entry point ────────────────────────────────────────────────────────────────
# Served over loopback HTTP rather than pywebview's html=, which loads the page
# with a null origin. Geolocation (and other secure-context features) only work
# on https or 127.0.0.1, so html= failed with "Only secure origins are allowed".
# Nothing here is reachable from outside this machine.
APP_SETTINGS_PATH = os.path.join(os.path.expanduser('~'), '.drone_mission_planner.json')

def _load_app_settings():
    try:
        with open(APP_SETTINGS_PATH, 'r', encoding='utf-8') as f:
            s = json.load(f)
            return s if isinstance(s, dict) else {}
    except Exception:
        return {}

def _save_app_settings(s):
    try:
        with open(APP_SETTINGS_PATH, 'w', encoding='utf-8') as f:
            json.dump(s, f, indent=2)
    except Exception:
        pass  # a read-only home dir shouldn't crash the app over a remembered preference

class _AppRequestHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ('/', '/index.html'):
            # favicon.ico etc. -- a proper 404 instead of serving the whole
            # app HTML to every stray request.
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(HTML.encode('utf-8'))
    def log_message(self, format, *args):
        pass  # keep the app's own console output clean

def _start_local_server():
    # A stable port matters: localStorage is origin-scoped and the origin is
    # host:port, so a random port wiped the remembered drone every launch.
    # Falls back to any free port if taken; the settings file covers that case.
    try:
        server = http.server.HTTPServer(('127.0.0.1', 8577), _AppRequestHandler)
    except OSError:
        server = http.server.HTTPServer(('127.0.0.1', 0), _AppRequestHandler)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return port

def _work_area():
    """Usable desktop rectangle (x, y, w, h) -- the screen minus the taskbar.

    Returns None if it can't be determined, in which case the window falls
    back to pywebview's own default placement.
    """
    if sys.platform != 'win32':
        return None
    try:
        import ctypes
        import ctypes.wintypes as wintypes
        # Deliberately does NOT touch the process's DPI awareness: these
        # coordinates and create_window's x/y are read in the same space either
        # way, and declaring awareness would shrink the whole UI on a scaled
        # display.
        rect = wintypes.RECT()
        SPI_GETWORKAREA = 0x0030
        if not ctypes.windll.user32.SystemParametersInfoW(SPI_GETWORKAREA, 0, ctypes.byref(rect), 0):
            return None
        w, h = rect.right - rect.left, rect.bottom - rect.top
        if w <= 0 or h <= 0:
            return None
        return rect.left, rect.top, w, h
    except Exception:
        return None


def main():
    api = Api()
    port = _start_local_server()

    # Open maximised and centred. Without explicit geometry the OS cascades
    # the window down-and-right, so it kept starting offset to the right.
    win_w, win_h, win_x, win_y = 1500, 900, None, None
    area = _work_area()
    if area:
        ax, ay, aw, ah = area
        win_w = max(1000, int(aw * 0.9))
        win_h = max(650, int(ah * 0.9))
        win_x = ax + (aw - win_w) // 2
        win_y = ay + (ah - win_h) // 2

    window = webview.create_window(
        f'Drone Mission Planner v{APP_VERSION}',
        url=f'http://127.0.0.1:{port}/',
        js_api=api,
        width=win_w, height=win_h,
        x=win_x, y=win_y,
        min_size=(1000, 650),
        # Maximised rather than fullscreen=True: this is a planning tool, so
        # the title bar and taskbar need to stay reachable -- fullscreen is
        # kiosk mode and hides both.
        maximized=True,
        background_color='#0d0d0d',
    )
    api.set_window(window)
    # A frozen .exe carries its icon from PyInstaller's --icon and pywebview
    # picks that up automatically; this only covers running the script directly,
    # which would otherwise show Python's own icon.
    icon_path = None
    if not getattr(sys, 'frozen', False):
        candidate = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'icon.ico')
        if os.path.isfile(candidate):
            icon_path = candidate
    webview.start(debug=False, icon=icon_path)

if __name__ == '__main__':
    main()
