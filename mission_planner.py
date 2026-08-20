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
import xml.etree.ElementTree as ET
import webview

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
                        if len(coords) >= 2:
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
# droneEnumValue=68: no official DJI docs for consumer drones (the Cloud API/WPML
# spec only covers enterprise models). YMapper's author found this value works
# for DJI Fly regardless of which consumer drone is connected, so we match it.
#
# Camera specs (sensor mm, focal length, resolution) come from YMapper's own
# preset table, cross-checked against DJI's spec pages for the Mini 4/5 Pro.
# Footprint = altitude * sensorSize / focalLength, same physical model YMapper
# uses. The Mini 5 Pro's 1" sensor is genuinely bigger than the Mini 4 Pro's
# 1/1.3" — not just a label difference, it changes the numbers.

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
                'with Crosshatch for a proper convergent network.',
    },
    '3d_model': {
        'label': '3D model / building / urban scene', 'pitch': -45, 'overlap': (80, 70),
        'note': "Matches DJI Terra's own default oblique tilt (-45°) for 3D reconstruction "
                'missions. Best flown as a nadir + oblique double-grid — enable "3D mapping '
                '(nadir + oblique)" below to generate both passes in one mission automatically.',
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

def _sweep_coverage(rpts, side_spacing, forward_spacing):
    """Boustrophedon (lawnmower) sweep across the polygon's bounding box, keeping
    only in-polygon sample points and snapping each row's last point out to the
    true edge so rows don't stop short. Each sample point becomes a photo
    waypoint, so spacing directly controls how many photos the mission takes."""
    xs = [p[0] for p in rpts]
    ys = [p[1] for p in rpts]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    pts = []
    reverse = False
    y = miny
    while y <= maxy + 1e-9:
        line = []
        x = minx
        while x <= maxx + 1e-9:
            if _point_in_polygon(x, y, rpts):
                line.append((x, y))
            x += forward_spacing
        if line and abs(maxx - line[-1][0]) > 1e-6 and _point_in_polygon(maxx, y, rpts):
            line.append((maxx, y))
        if reverse:
            line.reverse()
        pts.extend(line)
        reverse = not reverse
        y += side_spacing
    return pts

def generate_grid(polygon_latlon, cfg):
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

    pts = _sweep_coverage(rpts, side_spacing, forward_spacing)
    if cfg.get('crosshatch'):
        # A second sweep at 90° catches gaps the first sweep's direction misses,
        # especially on concave/irregular site boundaries — appended as a second
        # pass rather than interleaved, so it always runs after the main grid.
        transposed = [(y, x) for x, y in rpts]
        pts2 = _sweep_coverage(transposed, side_spacing, forward_spacing)
        pts += [(x, y) for y, x in pts2]

    if not pts:
        raise ValueError('No coverage generated — the area may be too small for the current spacing/altitude')

    waypoints = []
    for x, y in pts:
        lx, ly = x * ci - y * si, x * si + y * ci
        lat, lon = from_xy(lx, ly, ref_lat, ref_lon)
        waypoints.append({'lat': lat, 'lon': lon, 'alt': cfg['altitude'], 'speed': cfg['speed'],
                           'gimbal': cfg.get('gimbalPitch', -90), 'heading_mode': 'followWayline',
                           'photo': True, 'hover': cfg.get('delayAtWaypoint', 0)})
    return waypoints

def generate_3d_mapping(polygon_latlon, cfg):
    """Nadir + oblique double-grid for full 3D reconstruction (DJI Terra/Pix4D
    method): a straight-down pass for top surfaces plus a second, 90°-rotated
    pass at an oblique angle so facades actually get imaged too."""
    nadir_cfg = dict(cfg)
    nadir_cfg['gimbalPitch'] = -90
    nadir_cfg['crosshatch'] = False
    nadir_pts = generate_grid(polygon_latlon, nadir_cfg)

    oblique_cfg = dict(cfg)
    oblique_cfg['gimbalPitch'] = cfg.get('obliqueGimbal', -45)
    oblique_cfg['rotationDeg'] = (cfg.get('rotationDeg', 0) + 90) % 360
    oblique_cfg['crosshatch'] = False
    oblique_pts = generate_grid(polygon_latlon, oblique_cfg)

    return nadir_pts + oblique_pts

def generate_corridor(line_latlon, cfg):
    if len(line_latlon) < 2:
        raise ValueError('A corridor route needs at least 2 points')
    ref_lat, ref_lon = line_latlon[0][0], line_latlon[0][1]
    pts_xy = [to_xy(p[0], p[1], ref_lat, ref_lon) for p in line_latlon]

    side_spacing, forward_spacing = coverage_spacing(cfg)

    width = max(0.0, cfg.get('corridorWidth', 0))
    n_passes = 1 if width <= 0 else max(1, math.ceil(width / side_spacing) + 1)
    half = width / 2
    offsets = [0.0] if n_passes == 1 else [-half + i * (width / (n_passes - 1)) for i in range(n_passes)]

    seglens = []
    total_len = 0.0
    for i in range(len(pts_xy) - 1):
        dx = pts_xy[i + 1][0] - pts_xy[i][0]
        dy = pts_xy[i + 1][1] - pts_xy[i][1]
        l = math.hypot(dx, dy)
        seglens.append(l)
        total_len += l
    if total_len <= 0:
        raise ValueError('Route has zero length')

    n_samples = max(2, int(total_len // forward_spacing) + 1)
    samples = []
    dist_accum = 0.0
    seg_i = 0
    for s in range(n_samples + 1):
        target = total_len * s / n_samples
        while seg_i < len(seglens) - 1 and dist_accum + seglens[seg_i] < target:
            dist_accum += seglens[seg_i]
            seg_i += 1
        seg_len = seglens[seg_i] if seglens[seg_i] > 0 else 1e-9
        t = min(max((target - dist_accum) / seg_len, 0), 1)
        x1, y1 = pts_xy[seg_i]
        x2, y2 = pts_xy[seg_i + 1]
        x, y = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
        heading = math.atan2(x2 - x1, y2 - y1)
        samples.append((x, y, heading))

    waypoints = []
    for pass_i, off in enumerate(offsets):
        pass_samples = samples if pass_i % 2 == 0 else list(reversed(samples))
        for x, y, heading in pass_samples:
            perp = heading + math.pi / 2
            ox, oy = x + off * math.sin(perp), y + off * math.cos(perp)
            lat, lon = from_xy(ox, oy, ref_lat, ref_lon)
            waypoints.append({'lat': lat, 'lon': lon, 'alt': cfg['altitude'], 'speed': cfg['speed'],
                               'gimbal': cfg.get('gimbalPitch', -90), 'heading_mode': 'followWayline',
                               'photo': True, 'hover': 0})
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
            waypoints.append({'lat': lat, 'lon': lon, 'alt': altitude, 'speed': cfg.get('speed', 5),
                               'gimbal': round(pitch), 'heading_mode': 'fixed', 'heading_angle': round(hdg),
                               'photo': cfg.get('photo', True), 'hover': cfg.get('delayAtWaypoint', 0),
                               # A circular path made of stop-and-rotate segments (the grid/corridor
                               # default) looks like a stuttering polygon, not an orbit — DJI Fly's
                               # own continuity-curvature turn mode is a centripetal Catmull-Rom spline
                               # through the waypoints, which is what actually flies a smooth circle.
                               'turn_mode': cfg.get('orbitTurnMode', 'toPointAndPassWithContinuityCurvature')})
    return waypoints

def generate_overview(polygon_latlon, cfg):
    """A single higher-altitude lap around the site boundary with a photo at every
    corner plus mid-edge points — a quick 'whole site in context' pass, meant to be
    flown in addition to a detailed grid, not instead of it."""
    if len(polygon_latlon) < 3:
        raise ValueError('An overview lap needs at least 3 points')
    alt = cfg.get('overviewAltitude') or (cfg['altitude'] * 1.5)
    speed = cfg.get('speed', 8)
    waypoints = []
    n = len(polygon_latlon)
    for i in range(n):
        a = polygon_latlon[i]
        b = polygon_latlon[(i + 1) % n]
        waypoints.append({'lat': a[0], 'lon': a[1], 'alt': alt, 'speed': speed,
                           'gimbal': cfg.get('overviewGimbal', -60), 'heading_mode': 'followWayline',
                           'photo': True, 'hover': 0})
        mid_lat, mid_lon = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
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
    pts = generate_grid(polygon_latlon, cfg)
    passes = max(1, round(polygon_area_m2(polygon_latlon) ** 0.5 / side_spacing))
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

def split_mission_by_battery(waypoints, cfg):
    """Greedily group waypoints into flight-time-budgeted batches, each a
    standalone sub-mission: fly it, swap battery, load the next one."""
    if not waypoints:
        return []
    budget = usable_battery_seconds(cfg)
    batches = []
    current = [waypoints[0]]
    elapsed = waypoints[0].get('hover', 0) or 0
    for i in range(1, len(waypoints)):
        prev_wp, wp = waypoints[i - 1], waypoints[i]
        speed = prev_wp.get('speed') or cfg.get('speed') or 5
        leg = haversine_m(prev_wp['lat'], prev_wp['lon'], wp['lat'], wp['lon']) / speed + (wp.get('hover', 0) or 0)
        if elapsed + leg > budget and current:
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

def optimal_rotation_deg(polygon_latlon):
    """Rotating calipers: the minimum-area bounding rectangle of a convex hull is
    always aligned with one of its edges, so test each edge angle and keep the
    smallest. Aligns the grid sweep to minimize wasted transit distance."""
    if len(polygon_latlon) < 3:
        return 0.0
    ref_lat = sum(p[0] for p in polygon_latlon) / len(polygon_latlon)
    ref_lon = sum(p[1] for p in polygon_latlon) / len(polygon_latlon)
    pts = [to_xy(p[0], p[1], ref_lat, ref_lon) for p in polygon_latlon]
    hull = _convex_hull(pts)
    if len(hull) < 3:
        return 0.0

    best_angle, best_area = 0.0, float('inf')
    n = len(hull)
    for i in range(n):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % n]
        edge_angle = math.atan2(y2 - y1, x2 - x1)
        c, s = math.cos(-edge_angle), math.sin(-edge_angle)
        rx = [x * c - y * s for x, y in hull]
        ry = [x * s + y * c for x, y in hull]
        area = (max(rx) - min(rx)) * (max(ry) - min(ry))
        if area < best_area:
            best_area = area
            best_angle = edge_angle

    return round(math.degrees(best_angle) % 180, 1)

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
          <wpml:waypointHeadingAngleEnable>0</wpml:waypointHeadingAngleEnable>
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
# ADB is not a real option on the RC2: independent reverse-engineering (see
# docs.f1y.ing's RC2 research notes) confirms adbd is present but deliberately
# refuses host handshakes as a firmware hardening measure — that's why it shows
# "offline" forever, not "unauthorized" or missing. No driver/cable/settings fix
# changes that. MTP is what DJI actually supports, so that's what this drives,
# via the same Shell.Application COM automation Windows Explorer itself uses to
# browse MTP devices — no extra driver or library needed beyond pywin32.
#
# DJI Fly only ever loads a mission it created itself, so the trick (same one
# DJI-KMZ-Injector's own ADB backend uses) is: a dummy mission already exists on
# the controller as a UUID-named folder containing "<uuid>.kmz". Rather than
# guessing which one to overwrite, list_mission_slots() surfaces all of them —
# with waypoint count and approximate location read from each mission file, since
# DJI Fly's own mission title isn't stored anywhere MTP can reach — and the user
# picks which slot upload_kmz_to_slot() replaces.

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
                'defaults': DEFAULT_MISSION_CONFIG}

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
    def generate_grid(self, polygon, cfg):
        try:
            fn = generate_3d_mapping if cfg.get('threeDMapping') else generate_grid
            return {'ok': True, 'waypoints': fn(polygon, cfg)}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def generate_corridor(self, line, cfg):
        try:
            return {'ok': True, 'waypoints': generate_corridor(line, cfg)}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def generate_orbit(self, center, cfg):
        try:
            return {'ok': True, 'waypoints': generate_orbit(center[0], center[1], cfg)}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def generate_overview(self, polygon, cfg):
        try:
            return {'ok': True, 'waypoints': generate_overview(polygon, cfg)}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def estimate_grid(self, polygon, cfg):
        try:
            return {'ok': True, **estimate_coverage(polygon, cfg)}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def optimal_rotation(self, polygon):
        try:
            return {'ok': True, 'rotation': optimal_rotation_deg(polygon)}
        except Exception as e:
            return {'ok': False, 'msg': str(e)}

    def estimate_battery_split(self, waypoints, cfg):
        try:
            batches = split_mission_by_battery(waypoints, cfg)
            budget = usable_battery_seconds(cfg)

            def batch_time(b):
                t = 0.0
                for i in range(1, len(b)):
                    speed = b[i - 1].get('speed') or cfg.get('speed') or 5
                    t += haversine_m(b[i - 1]['lat'], b[i - 1]['lon'], b[i]['lat'], b[i]['lon']) / speed
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
#titlebar .sub{color:var(--text-faint);font-size:10px;margin-top:2px;letter-spacing:.2px;}
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
.mission-type-grid button{padding:12px 4px;text-align:center;font-size:12px;line-height:1.7;
  background:var(--bg3);border-color:var(--border2);}
.mission-type-grid button:hover{background:var(--bg4);border-color:var(--orange-dim);}

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
#draw-hint{position:absolute;top:12px;left:50%;transform:translateX(-50%);z-index:900;
  background:#0a0a0ae8;border:1px solid var(--orange);border-radius:var(--radius);padding:8px 18px;
  font-size:12px;color:#fff;display:none;pointer-events:none;box-shadow:0 4px 16px #000a;}
#draw-hint.visible{display:block;}

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
</style>
</head>
<body>

<div id="titlebar">
  <div>
    <div class="logo">&#128225; Drone Mission Planner</div>
    <div class="sub">KML/KMZ import &middot; grid / corridor / orbit / manual missions &middot; DJI WPML export</div>
  </div>
  <div class="credit">by <a href="https://github.com/0xpraet0rian" target="_blank">praet0rian (mark0)</a></div>
</div>

<div id="toolbar">
  <div class="tgroup">
    <button class="primary" onclick="importKml()">&#128193; Import KML/KMZ</button>
  </div>
  <div class="tgroup">
    <button id="btn-finish" onclick="finishDraw()" style="display:none;">&#10003; Finish</button>
    <button id="btn-cancel" onclick="cancelDraw()" style="display:none;">&#10005; Cancel</button>
  </div>
  <div class="sep"></div>
  <div class="tgroup">
    <button onclick="clearMission()">&#128465; Clear</button>
    <button class="primary" onclick="exportWpml()">&#128190; Export WPML</button>
    <button onclick="exportWpmlSplit()" title="Split into multiple missions sized to your battery's usable endurance">&#128267; Export by Battery</button>
    <button onclick="openUploadPicker()" title="Pick a mission slot on a connected DJI RC/RC2 to replace, over MTP">&#128225; Upload to RC</button>
  </div>
  <div class="sep"></div>
  <div class="tgroup">
    <button onclick="saveProject()">Save</button>
    <button onclick="loadProject()">Load</button>
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

<div id="main">
  <div id="sidebar">
    <div id="tabs">
      <div class="tab active" data-tab="setup" onclick="showTab('setup')">Setup</div>
      <div class="tab" data-tab="waypoints" onclick="showTab('waypoints')">Waypoints</div>
      <div class="tab" data-tab="layers" onclick="showTab('layers')">Layers</div>
    </div>
    <div id="tab-content"></div>
    <div id="wp-stats"></div>
  </div>
  <div id="content">
    <div id="draw-hint"></div>
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
var drawMode = null;     // 'area'|'route'|'orbit'|'manual'
var tempPoints = [];
var selectedWpIdx = null;
var pendingKind = null;      // 'grid'|'corridor'|'orbit' — the source shape for the current mission
var pendingGeom = null;
var pendingGenerated = false; // true once this pending shape has been generated at least once
var missionName = 'Mission';  // prompted at the start of each mission, used as the export filename prefix

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
// Port of the Python point-in-polygon + sweep (_point_in_polygon/_sweep_coverage)
// so the live estimate matches the real generator exactly instead of guessing off
// the bounding box, which badly overcounts for diagonal or thin polygons.
function pointInPolygonJS(x,y,poly){
  var inside=false, n=poly.length;
  for(var i=0,j=n-1;i<n;j=i++){
    var xi=poly[i][0], yi=poly[i][1], xj=poly[j][0], yj=poly[j][1];
    if(((yi>y)!==(yj>y)) && (x < (xj-xi)*(y-yi)/(yj-yi)+xi)) inside=!inside;
  }
  return inside;
}
function sweepCoverageJS(rpts, sideSpacing, forwardSpacing){
  var xs=rpts.map(p=>p[0]), ys=rpts.map(p=>p[1]);
  var minx=Math.min.apply(null,xs), maxx=Math.max.apply(null,xs);
  var miny=Math.min.apply(null,ys), maxy=Math.max.apply(null,ys);
  var pts=[], reverse=false, count=0, maxIter=200000;
  for(var y=miny; y<=maxy+1e-9 && count<maxIter; y+=sideSpacing){
    var line=[];
    for(var x=minx; x<=maxx+1e-9 && count<maxIter; x+=forwardSpacing, count++){
      if(pointInPolygonJS(x,y,rpts)) line.push([x,y]);
    }
    if(line.length && Math.abs(maxx-line[line.length-1][0])>1e-6 && pointInPolygonJS(maxx,y,rpts)) line.push([maxx,y]);
    if(reverse) line.reverse();
    pts=pts.concat(line);
    reverse=!reverse;
  }
  return pts;
}
function estimateGrid(polygon, c){
  if(!polygon || polygon.length<3) return null;
  var refLat=polygon.reduce((s,p)=>s+p[0],0)/polygon.length;
  var refLon=polygon.reduce((s,p)=>s+p[1],0)/polygon.length;
  var pts=polygon.map(p=>toXY(p[0],p[1],refLat,refLon));
  var rot=(c.rotationDeg||0)*Math.PI/180, cf=Math.cos(-rot), sf=Math.sin(-rot);
  var rpts=pts.map(p=>[p[0]*cf-p[1]*sf, p[0]*sf+p[1]*cf]);
  var sp=coverageSpacing(c);
  var side=sp[0], forward=sp[1];
  var count=sweepCoverageJS(rpts, side, forward).length;
  if(c.crosshatch && !c.threeDMapping){
    var transposed=rpts.map(p=>[p[1],p[0]]);
    count += sweepCoverageJS(transposed, side, forward).length;
  }
  if(c.threeDMapping){
    // Mirrors generate_3d_mapping: a second full pass rotated 90°, oblique gimbal.
    var rot2=((c.rotationDeg||0)+90)*Math.PI/180, cf2=Math.cos(-rot2), sf2=Math.sin(-rot2);
    var rpts2=pts.map(p=>[p[0]*cf2-p[1]*sf2, p[0]*sf2+p[1]*cf2]);
    count += sweepCoverageJS(rpts2, side, forward).length;
  }
  var xs=rpts.map(p=>p[0]), ys=rpts.map(p=>p[1]);
  var passes=Math.max(1,Math.round((Math.max.apply(null,ys)-Math.min.apply(null,ys))/side)+1);
  return {side:side.toFixed(1), forward:forward.toFixed(1), passes:passes, photos:count};
}
function estimateCorridor(line, c){
  if(!line || line.length<2) return null;
  var length=0;
  for(var i=1;i<line.length;i++) length+=haversine(line[i-1][0],line[i-1][1],line[i][0],line[i][1]);
  var sp=coverageSpacing(c);
  var width=Math.max(0,c.corridorWidth||0);
  var passes = width<=0 ? 1 : Math.max(1,Math.ceil(width/sp[0])+1);
  var perPass=Math.max(1,Math.round(length/sp[1])+1);
  return {side:sp[0].toFixed(1), forward:sp[1].toFixed(1), passes:passes, photos:passes*perPass};
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
  if(pendingKind==='grid'){ est=estimateGrid(pendingGeom, cfg); areaM2=polygonAreaM2(pendingGeom); }
  else if(pendingKind==='corridor') est=estimateCorridor(pendingGeom, cfg);
  if(!est){ el.innerHTML=''; return; }
  var shutter=recommendedShutterSpeed(cfg);
  var interval=cfg.speed ? (est.forward/cfg.speed) : 0;
  var distM=est.photos>0 ? est.photos*est.forward : 0; // rough distance estimate for flight time
  var flightSec = distM/(cfg.speed||1);
  var usableSec = usableBatteryMinutes(cfg)*60;
  var batteries = Math.max(1, Math.ceil(flightSec/usableSec));
  var warn = batteries>1
    ? '<div class="hint warn">&#128267; ~'+batteries+' batteries needed at this size &mdash; use "Export by Battery" after generating to split automatically, or lower overlap/raise altitude to shrink it.</div>' : '';
  var extraStats = '<div class="hint">' +
    (areaM2 ? 'Area <b>'+(areaM2>=10000?(areaM2/10000).toFixed(2)+' ha':Math.round(areaM2)+' m&sup2;')+'</b> &middot; ' : '') +
    'Photo interval <b>~'+interval.toFixed(1)+'s</b>' +
    (shutter ? ' &middot; shutter &le; <b>1/'+shutter+'</b> to avoid blur' : '') +
    ' &middot; flight <b>~'+Math.round(flightSec/60)+'m</b>' +
    '</div>';
  el.innerHTML = '<div class="hint">Line spacing <b>'+est.side+'m</b> &middot; photo spacing <b>'+est.forward+'m</b> &middot; '+est.passes+' pass(es)</div>' +
    '<div style="font-size:16px;color:var(--orange);font-weight:700;margin:4px 0;">~'+est.photos+' photos (estimate)</div>' +
    extraStats + warn;
}

var map = L.map('map', {preferCanvas:true}).setView([45.35,22.28], 12);

// ── Base layers (switchable) ────────────────────────────────────────────────
// maxNativeZoom is where the tile provider's real imagery stops; Leaflet upscales
// past that instead of going blank. Esri satellite goes to z23, OSM/CARTO ~z19-20,
// OpenTopoMap z17.
var baseStreets = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  {maxZoom:22, maxNativeZoom:19, attribution:'&copy; OpenStreetMap'});
var baseSatellite = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {maxZoom:23, maxNativeZoom:23, attribution:'Tiles &copy; Esri'});
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
var wpPathLayer = null;
var wpMarkers = {};

var satelliteHybrid = L.layerGroup([baseSatellite, baseSatelliteLabels]);
L.control.layers({
  'Street': baseStreets,
  'Satellite': satelliteHybrid,
  'Satellite (no labels)': baseSatellite,
  'Topographic': baseTopo,
  'Dark': baseDark,
}, {
  'Imported KML/KMZ': importedGroup,
  'Flight path': wpGroup,
}, {position:'topright', collapsed:true}).addTo(map);

// ── Init ───────────────────────────────────────────────────────────────────
function init(){
  pywebview.api.get_presets().then(function(p){
    PRESETS = p;
    cfg = Object.assign({}, p.defaults);
    var saved = null;
    try{ saved = localStorage.getItem('dmp_drone'); }catch(e){}
    if(saved && PRESETS.drones[saved]){
      setDrone(saved);
    } else {
      renderSetup();
      showDronePicker();
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
function closeDronePicker(){ document.getElementById('drone-picker-overlay').classList.remove('visible'); }

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
function setGimbalPreset(k){
  cfg.gimbalPreset=k;
  var g=PRESETS.gimbals[k];
  if(g && k!=='custom'){
    cfg.gimbalPitch=g.pitch;
    if(g.overlap){ cfg.forwardOverlap=g.overlap[0]; cfg.sideOverlap=g.overlap[1]; }
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
      '<button onclick="startDraw(\'area\')">&#9723;<br>Grid Survey</button>' +
      '<button onclick="startDraw(\'route\')">&#9646;<br>Corridor</button>' +
      '<button onclick="startDraw(\'orbit\')">&#9678;<br>Orbit</button>' +
      '<button onclick="startDraw(\'manual\')">&#128204;<br>Manual</button>' +
    '</div></div>' +

    // ── Core flight parameters (always visible — used by every mission type) ──
    '<div class="panel-section"><h4>Flight</h4>' +
    '<div class="field-row">' +
      '<div class="field"><label>Altitude (m AGL)</label><input type="number" value="'+cfg.altitude+'" onchange="cfg.altitude=parseFloat(this.value)||10;refreshEstimate()"></div>' +
      '<div class="field"><label>Speed (m/s)</label><input type="number" value="'+cfg.speed+'" onchange="cfg.speed=parseFloat(this.value)||1"></div>' +
    '</div>' +
    '<div class="field"><label>Delay at each waypoint (sec, 0=none)</label><input type="number" min="0" value="'+cfg.delayAtWaypoint+'" onchange="cfg.delayAtWaypoint=parseFloat(this.value)||0"></div>' +
    '</div>' +

    // ── Battery & endurance — drives automatic mission splitting ──
    '<div class="panel-section"><h4>Battery &amp; endurance</h4>' +
    '<div class="field"><label>Battery</label><select onchange="setBattery(this.value)">'+batteryOptions()+'</select></div>' +
    '<details><summary>Usable-time assumptions<span></span></summary><div class="details-body">' +
      '<div class="field-row">' +
        '<div class="field"><label>Realistic-conditions factor</label><input type="number" step="0.05" min="0.1" max="1" value="'+cfg.realisticFactor+'" onchange="cfg.realisticFactor=parseFloat(this.value)||0.75;refreshEstimate()"></div>' +
        '<div class="field"><label>RTH/safety reserve</label><input type="number" step="0.05" min="0" max="0.6" value="'+cfg.reserveFraction+'" onchange="cfg.reserveFraction=parseFloat(this.value)||0.3;refreshEstimate()"></div>' +
      '</div>' +
      '<div class="hint">Rated flight times are windless lab-ideal figures. Real-world usable endurance is commonly 70-80% of rated, and standard practice reserves 20-30% battery for return-to-home/contingency &mdash; default here is 75% &times; (1-30%) &asymp; 52% of the rated number.</div>' +
    '</div></details>' +
    '<div class="hint" style="margin-top:8px;">Usable per battery: <b>~'+usableBatteryMinutes(cfg).toFixed(0)+' min</b> of the '+cfg.batteryMinutes+' min rated.</div>' +
    '</div>' +

    // ── Capture purpose / gimbal — kept prominent since it's science-driven ──
    '<div class="panel-section"><h4>Capture purpose</h4>' +
    '<div class="field"><label>What are you capturing?</label><select onchange="setGimbalPreset(this.value)">'+gimbalOptions()+'</select></div>' +
    (gimbalNote ? '<div class="hint">'+gimbalNote+'</div>' : '') +
    '<div class="field" style="margin-top:8px;"><label>Gimbal pitch <span style="float:right;color:var(--text-faint);">-90&deg;=down &middot; 0&deg;=horizon</span></label>' +
      '<input type="number" value="'+cfg.gimbalPitch+'" onchange="cfg.gimbalPitch=parseFloat(this.value)||0;cfg.gimbalPreset=\'custom\';refreshEstimate()"></div>' +
    '</div>' +

    // ── Per-mission-type settings, collapsed except the currently relevant one ──
    '<div class="panel-section"><h4>Mission-specific settings</h4>' +
    '<details'+op('grid')+' class="'+(currentKind==='grid'?'active-kind':'')+'"><summary>Grid survey'+badge('grid')+'</summary><div class="details-body">' +
      '<div class="field-row">' +
        '<div class="field"><label>Forward overlap %</label><input type="number" value="'+cfg.forwardOverlap+'" onchange="cfg.forwardOverlap=parseFloat(this.value)||0;refreshEstimate()"></div>' +
        '<div class="field"><label>Side overlap %</label><input type="number" value="'+cfg.sideOverlap+'" onchange="cfg.sideOverlap=parseFloat(this.value)||0;refreshEstimate()"></div>' +
      '</div>' +
      '<div class="field"><label>Grid rotation <span id="rot-val" style="color:var(--orange);float:right;">'+cfg.rotationDeg+'&deg;</span></label>' +
        '<input id="rot-slider" type="range" min="0" max="359" value="'+cfg.rotationDeg+'" style="width:100%;accent-color:var(--orange);" ' +
        'oninput="cfg.rotationDeg=parseFloat(this.value);document.getElementById(\'rot-val\').textContent=this.value+\'°\';refreshEstimate()"></div>' +
      '<button style="width:100%;margin-top:2px;" onclick="autoRotate()" title="Align the sweep to the area\'s longest edge, minimizing wasted transit distance">&#8635; Auto-rotate to minimize flight distance</button>' +
      '<div class="field-row" style="margin-top:6px;">' +
        '<div class="field"><label>Wind from (&deg;, optional)</label><input id="wind-dir" type="number" min="0" max="359" placeholder="e.g. 270"></div>' +
        '<div class="field" style="display:flex;align-items:flex-end;"><button style="width:100%;" onclick="rotateForWind()" title="Fly the long passes into/with the wind rather than across it — steadier ground speed and less battery spent fighting a crosswind on every pass">&#8634; Align to wind</button></div>' +
      '</div>' +
      '<div class="hint">Coverage-path research (e.g. Boustrophedon CPP for UAV surveys in wind) finds sweeping parallel to the wind (not perpendicular) covers faster with steadier speed. Enter the direction wind is coming FROM if you know it.</div>' +
      '<div class="field" style="margin-top:8px;"><label>Turn style</label><select onchange="cfg.turnMode=this.value">' +
        opt('toPointAndStopWithDiscontinuityCurvature',cfg.turnMode,'Stop at each point (precise — recommended for mapping)')+
        opt('toPointAndStopWithContinuityCurvature',cfg.turnMode,'Slow smooth turn, still stops')+
        opt('toPointAndPassWithContinuityCurvature',cfg.turnMode,'Smooth flythrough, never stops')+
      '</select></div>' +
      '<div class="hint">Stopping at each point keeps camera position/GSD consistent for photogrammetry — the standard choice for mapping. Smooth flythrough covers ground faster but can blur shots taken mid-turn.</div>' +
      '<div class="checkbox-row" style="margin-top:8px;"><input type="checkbox" id="cb-xh" '+(cfg.crosshatch?'checked':'')+' onchange="cfg.crosshatch=this.checked;refreshEstimate()"><label for="cb-xh">Crosshatch (double grid) for thorough coverage</label></div>' +
      '<div class="hint">Second pass at 90° to the first. Roughly doubles photo count and flight time but fills gaps a single sweep misses on irregular sites.</div>' +
      '<div class="checkbox-row" style="margin-top:8px;"><input type="checkbox" id="cb-3d" '+(cfg.threeDMapping?'checked':'')+' onchange="cfg.threeDMapping=this.checked;refreshEstimate();renderSetup()"><label for="cb-3d">3D mapping (nadir + oblique double-grid)</label></div>' +
      '<div class="hint">Flies the area twice: once straight down, once tilted (rotated 90° from the first pass) — the method DJI Terra/Pix4D document for full 3D reconstruction, since a pure-nadir pass never images vertical surfaces like walls. Roughly doubles photo count.</div>' +
      (cfg.threeDMapping ?
        '<div class="field" style="margin-top:8px;"><label>Oblique pass gimbal pitch</label><input type="number" value="'+cfg.obliqueGimbal+'" onchange="cfg.obliqueGimbal=parseFloat(this.value)||-45;refreshEstimate()"></div>' : '') +
      '<details style="margin-top:8px;"><summary>Photo spacing override<span></span></summary><div class="details-body">' +
        '<div class="field-row">' +
          '<div class="field"><label>Line spacing (m, 0=auto)</label><input type="number" value="'+cfg.sideSpacingOverride+'" onchange="cfg.sideSpacingOverride=parseFloat(this.value)||0;refreshEstimate()"></div>' +
          '<div class="field"><label>Photo spacing (m, 0=auto)</label><input type="number" value="'+cfg.forwardSpacingOverride+'" onchange="cfg.forwardSpacingOverride=parseFloat(this.value)||0;refreshEstimate()"></div>' +
        '</div>' +
        '<div class="hint">Larger numbers = fewer, more spread-out photos. Leave at 0 to derive spacing from overlap % instead.</div>' +
      '</div></details>' +
      '<div class="checkbox-row" style="margin-top:8px;"><input type="checkbox" id="cb-ov" '+(cfg.overviewEnabled?'checked':'')+' onchange="cfg.overviewEnabled=this.checked;renderSetup()"><label for="cb-ov">Add a perimeter overview lap</label></div>' +
      '<div class="hint">Quick lap around the boundary at a higher altitude, photo at every corner/mid-edge &mdash; whole-site context in addition to the detailed grid.</div>' +
      (cfg.overviewEnabled ?
        '<div class="field-row" style="margin-top:8px;">' +
          '<div class="field"><label>Overview altitude (m, 0=auto 1.5&times;)</label><input type="number" value="'+cfg.overviewAltitude+'" onchange="cfg.overviewAltitude=parseFloat(this.value)||0"></div>' +
          '<div class="field"><label>Overview gimbal pitch</label><input type="number" value="'+cfg.overviewGimbal+'" onchange="cfg.overviewGimbal=parseFloat(this.value)||-60"></div>' +
        '</div>' : '') +
    '</div></details>' +

    '<details'+op('corridor')+' class="'+(currentKind==='corridor'?'active-kind':'')+'"><summary>Corridor'+badge('corridor')+'</summary><div class="details-body">' +
      '<div class="field-row">' +
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
      '<div class="checkbox-row"><input type="checkbox" id="cb-cw" '+(cfg.orbitClockwise?'checked':'')+' onchange="cfg.orbitClockwise=this.checked"><label for="cb-cw">Orbit clockwise</label></div>' +
      '<div class="hint">Gimbal continuously tracks the center point — no fixed pitch needed here.</div>' +
      '<div class="field" style="margin-top:8px;"><label>Turn style</label><select onchange="cfg.orbitTurnMode=this.value">' +
        opt('toPointAndPassWithContinuityCurvature',cfg.orbitTurnMode,'Smooth flythrough (recommended — flies an actual circle)')+
        opt('toPointAndStopWithContinuityCurvature',cfg.orbitTurnMode,'Slow smooth turn, still stops at each point')+
        opt('toPointAndStopWithDiscontinuityCurvature',cfg.orbitTurnMode,'Stop at each point (stuttering polygon, not a circle)')+
      '</select></div>' +
      '<div class="hint">DJI Fly\'s continuity-curvature mode flies a smooth spline through the waypoints — the only option here that actually looks and flies like a circle rather than a many-sided polygon with stop-and-rotate corners.</div>' +
      '<div class="field" style="margin-top:8px;"><label>Altitude rings (1=single ring)</label><input type="number" min="1" max="8" value="'+cfg.orbitRings+'" onchange="cfg.orbitRings=parseInt(this.value)||1;refreshEstimate();renderSetup()"></div>' +
      (cfg.orbitRings>1 ?
        '<div class="field-row">' +
          '<div class="field"><label>Lowest ring altitude (m, 0=auto)</label><input type="number" value="'+cfg.orbitMinAltitude+'" onchange="cfg.orbitMinAltitude=parseFloat(this.value)||0;refreshEstimate()"></div>' +
          '<div class="field"><label>Highest ring altitude (m, 0=auto)</label><input type="number" value="'+cfg.orbitMaxAltitude+'" onchange="cfg.orbitMaxAltitude=parseFloat(this.value)||0;refreshEstimate()"></div>' +
        '</div>' +
        '<div class="hint">Stacked rings at different altitudes around the same center &mdash; recommended for full 3D reconstruction of a tall/complex object (tower, silo, monument), where a single ring only sees it from one elevation angle. Aim for &ge;30 photos per ring.</div>'
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
  refreshEstimate();
}
function opt(val,cur,label){ return '<option value="'+val+'"'+(cur===val?' selected':'')+'>'+label+'</option>'; }
function setDrone(k){
  cfg.drone=k; var d=PRESETS.drones[k];
  cfg.droneEnumValue=d.droneEnumValue; cfg.droneSubEnumValue=d.droneSubEnumValue;
  if(d.batteries && d.batteries.length){ cfg.batteryIdx=0; cfg.batteryMinutes=d.batteries[0].minutes; }
  try{ localStorage.setItem('dmp_drone', k); }catch(e){}
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
function autoRotate(){
  var poly = (pendingKind==='grid') ? pendingGeom : null;
  if(!poly){ alert('Draw or select a grid area first, then Auto-rotate.'); return; }
  pywebview.api.optimal_rotation(poly).then(function(res){
    if(!res.ok){ alert('Could not compute rotation: '+res.msg); return; }
    cfg.rotationDeg = res.rotation;
    var slider=document.getElementById('rot-slider'), val=document.getElementById('rot-val');
    if(slider) slider.value=res.rotation;
    if(val) val.textContent=res.rotation+'°';
    refreshEstimate();
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
function startDraw(mode){
  promptMissionName();
  drawMode = mode; tempPoints = [];
  tempGroup.clearLayers();
  var hint = document.getElementById('draw-hint');
  hint.classList.add('visible');
  document.getElementById('btn-finish').style.display = (mode==='orbit'||mode==='manual') ? 'none' : 'inline-block';
  document.getElementById('btn-cancel').style.display = 'inline-block';
  var snapNote = importedLayers.length ? ' Clicks near an imported line/point snap to it.' : '';
  if(mode==='area') hint.textContent='Click to add area corners (min. 3). Click "Finish" when done.'+snapNote;
  if(mode==='route') hint.textContent='Click to add route points (min. 2). Click "Finish" when done.'+snapNote;
  if(mode==='orbit') hint.textContent='Click on the map to place the orbit center.'+snapNote;
  if(mode==='manual') hint.textContent='Click to add waypoints. Click "Cancel" or switch tools to stop.'+snapNote;
}
function cancelDraw(){
  drawMode = null; tempPoints = [];
  tempGroup.clearLayers();
  snapGroup.clearLayers();
  document.getElementById('draw-hint').classList.remove('visible');
  document.getElementById('btn-finish').style.display='none';
  document.getElementById('btn-cancel').style.display='none';
}

// ── Snap-to-import: pull drawn points onto imported KML/KMZ vertices/edges ──
var SNAP_PX = 14;
function closestPointOnSegment(p, a, b){
  var dx=b.x-a.x, dy=b.y-a.y, lenSq=dx*dx+dy*dy;
  if(lenSq===0) return a;
  var t=((p.x-a.x)*dx+(p.y-a.y)*dy)/lenSq;
  t=Math.max(0,Math.min(1,t));
  return L.point(a.x+t*dx, a.y+t*dy);
}
function findSnapPoint(latlng){
  if(!importedLayers.length) return null;
  var clickPt = map.latLngToContainerPoint(latlng);
  var best=null, bestDist=SNAP_PX;
  importedLayers.forEach(function(layer){
    var coords = layer.kind==='point' ? [[layer.lat,layer.lon]] : layer.coords;
    coords.forEach(function(c){
      var p=map.latLngToContainerPoint([c[0],c[1]]);
      var d=p.distanceTo(clickPt);
      if(d<bestDist){ bestDist=d; best=[c[0],c[1]]; }
    });
    if(layer.kind!=='point' && coords.length>1){
      var edgeCount = layer.kind==='polygon' ? coords.length : coords.length-1;
      for(var i=0;i<edgeCount;i++){
        var a=coords[i], b=coords[(i+1)%coords.length];
        var pa=map.latLngToContainerPoint([a[0],a[1]]);
        var pb=map.latLngToContainerPoint([b[0],b[1]]);
        var proj=closestPointOnSegment(clickPt, pa, pb);
        var d=proj.distanceTo(clickPt);
        if(d<bestDist){
          bestDist=d;
          var ll=map.containerPointToLatLng(proj);
          best=[ll.lat,ll.lng];
        }
      }
    }
  });
  return best;
}
function showSnapIndicator(latlng){
  snapGroup.clearLayers();
  if(latlng) L.circleMarker(latlng,{radius:9,color:'#ff9500',weight:2,fillColor:'#ff9500',fillOpacity:.25}).addTo(snapGroup);
}

map.on('mousemove', function(e){
  if(!drawMode){ if(snapGroup.getLayers().length) snapGroup.clearLayers(); return; }
  var snap=findSnapPoint(e.latlng);
  showSnapIndicator(snap ? L.latLng(snap[0],snap[1]) : null);
});

map.on('click', function(e){
  if(!drawMode) return;
  var snap=findSnapPoint(e.latlng);
  var lat=snap?snap[0]:e.latlng.lat, lon=snap?snap[1]:e.latlng.lng;
  if(drawMode==='area' || drawMode==='route'){
    tempPoints.push([lat,lon]);
    redrawTemp();
  } else if(drawMode==='orbit'){
    pendingKind='orbit'; pendingGeom=[lat,lon]; pendingGenerated=false;
    cancelDraw();
    showTab('setup');
  } else if(drawMode==='manual'){
    addManualWaypoint(lat,lon);
  }
});

function redrawTemp(){
  tempGroup.clearLayers();
  if(tempPoints.length===0) return;
  if(drawMode==='area'){
    if(tempPoints.length>=3) L.polygon(tempPoints,{color:'#e07b00',fillOpacity:.15,weight:2}).addTo(tempGroup);
    else L.polyline(tempPoints,{color:'#e07b00',weight:2,dashArray:'4,4'}).addTo(tempGroup);
  } else {
    L.polyline(tempPoints,{color:'#e07b00',weight:2}).addTo(tempGroup);
  }
  tempPoints.forEach(function(p){ L.circleMarker(p,{radius:5,color:'#000',weight:1,fillColor:'#e07b00',fillOpacity:1}).addTo(tempGroup); });
}

function finishDraw(){
  if(drawMode==='area'){
    if(tempPoints.length<3){ alert('Add at least 3 points to define an area.'); return; }
    pendingKind='grid'; pendingGeom=tempPoints.slice(); pendingGenerated=false;
  } else if(drawMode==='route'){
    if(tempPoints.length<2){ alert('Add at least 2 points to define a route.'); return; }
    pendingKind='corridor'; pendingGeom=tempPoints.slice(); pendingGenerated=false;
  }
  cancelDraw();
  showTab('setup');
}

function commitPending(){
  if(pendingKind==='grid') generateGridMission(pendingGeom);
  else if(pendingKind==='corridor') generateCorridorMission(pendingGeom);
  else if(pendingKind==='orbit') generateOrbitMission(pendingGeom);
}
function discardPending(){
  pendingKind=null; pendingGeom=null; pendingGenerated=false; renderSetup();
}

function mergeOverviewIfEnabled(polygonForOverview){
  if(!cfg.overviewEnabled || !polygonForOverview || polygonForOverview.length<3){
    return Promise.resolve();
  }
  return pywebview.api.generate_overview(polygonForOverview, cfg).then(function(res){
    if(res.ok) waypoints = waypoints.concat(res.waypoints);
  });
}

// Note: these don't clear pendingKind/pendingGeom, so the pending panel stays open
// after generating and can be tuned + regenerated in place.
function generateGridMission(polygon){
  showProgress('Generating grid...');
  pywebview.api.generate_grid(polygon, cfg).then(function(res){
    if(!res.ok){ hideProgress(); alert('Grid generation failed: '+res.msg); return; }
    waypoints = res.waypoints; pois=[]; selectedWpIdx=null; pendingGenerated=true;
    mergeOverviewIfEnabled(polygon).then(function(){
      hideProgress();
      renderWaypoints(); showTab('waypoints'); setStatus(waypoints.length+' waypoints generated');
    });
  });
}
function generateCorridorMission(line){
  showProgress('Generating corridor...');
  pywebview.api.generate_corridor(line, cfg).then(function(res){
    hideProgress();
    if(!res.ok){ alert('Corridor generation failed: '+res.msg); return; }
    waypoints = res.waypoints; pois=[]; selectedWpIdx=null; pendingGenerated=true;
    renderWaypoints(); showTab('waypoints'); setStatus(waypoints.length+' waypoints generated');
  });
}
function generateOrbitMission(center){
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
function addManualWaypoint(lat,lon){
  waypoints.push({lat:lat, lon:lon, alt:cfg.altitude, speed:cfg.speed, gimbal:cfg.gimbalPitch,
    heading_mode:'followWayline', heading_angle:0, photo:true, hover:cfg.delayAtWaypoint||0});
  renderWaypoints();
  if(activeTab==='waypoints') renderWaypointsTab();
  setStatus(waypoints.length+' waypoints');
}

function clearMission(){
  if(waypoints.length && !confirm('Clear the current mission?')) return;
  waypoints=[]; pois=[]; selectedWpIdx=null;
  pendingKind=null; pendingGeom=null; pendingGenerated=false;
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
  el.innerHTML = replayPanelHtml() +
    '<table id="wp-table"><thead><tr><th>#</th><th>Alt(m)</th><th>Spd</th><th>Gimbal</th><th>Photo</th><th></th></tr></thead>' +
    '<tbody>'+rows+'</tbody></table>';
  updateReplayUI();
}
function deleteWaypoint(i){
  waypoints.splice(i,1);
  if(selectedWpIdx===i) selectedWpIdx=null;
  renderWaypoints(); renderWaypointsTab();
}

function updateStats(){
  var dist=0;
  for(var i=1;i<waypoints.length;i++){ dist += haversine(waypoints[i-1].lat,waypoints[i-1].lon,waypoints[i].lat,waypoints[i].lon); }
  var photoCount = waypoints.filter(w=>w.photo).length;
  var avgSpeed = cfg.speed || 5;
  var flightSec = dist/avgSpeed + waypoints.reduce((s,w)=>s+(w.hover||0),0);
  var mins = Math.floor(flightSec/60), secs = Math.round(flightSec%60);
  var usableSec = usableBatteryMinutes(cfg)*60;
  var batteries = Math.max(1, Math.ceil(flightSec/usableSec));
  var battWarn = batteries>1
    ? '<span style="color:var(--orange2)">&#128267; ~'+batteries+' batteries needed &mdash; use "Export by Battery" to split automatically</span>' : '';
  var warn = waypoints.length>500
    ? '<span style="color:var(--orange2)">&#9888; '+waypoints.length+' waypoints is a lot &mdash; consider lowering overlap %, raising altitude, or splitting into multiple missions</span>' : '';
  document.getElementById('wp-stats').innerHTML =
    '<span><b>'+waypoints.length+'</b> waypoints</span>' +
    '<span><b>'+(dist/1000).toFixed(2)+'</b> km</span>' +
    '<span><b>~'+mins+'m '+secs+'s</b> flight time</span>' +
    '<span><b>'+photoCount+'</b> photos</span>' + battWarn + warn;
}
function haversine(lat1,lon1,lat2,lon2){
  var R=6371000, toRad=d=>d*Math.PI/180;
  var dp=toRad(lat2-lat1), dl=toRad(lon2-lon1);
  var a=Math.sin(dp/2)**2+Math.cos(toRad(lat1))*Math.cos(toRad(lat2))*Math.sin(dl/2)**2;
  return 2*R*Math.asin(Math.sqrt(a));
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
  var times=[0], t=0;
  for(var i=1;i<waypoints.length;i++){
    var d=haversine(waypoints[i-1].lat,waypoints[i-1].lon,waypoints[i].lat,waypoints[i].lon);
    var spd=waypoints[i-1].speed||cfg.speed||5;
    t += d/spd + (waypoints[i-1].hover||0);
    times.push(t);
  }
  t += waypoints[waypoints.length-1].hover||0;
  replay.times=times; replay.totalTime=t||1;
}

function ensureReplayMarker(){
  if(replay.marker) return;
  var icon=L.divIcon({className:'', html:'<div class="drone-marker">&#128257;</div>', iconSize:[26,26], iconAnchor:[13,13]});
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
    importedLayers = [];
    res.polygons.forEach(p=>importedLayers.push({kind:'polygon', name:p.name, coords:p.coords}));
    res.lines.forEach(l=>importedLayers.push({kind:'line', name:l.name, coords:l.coords}));
    res.points.forEach(pt=>importedLayers.push({kind:'point', name:pt.name, lat:pt.lat, lon:pt.lon}));
    drawImportedLayers();
    showTab('layers');
    setStatus(res.msg);
  });
}
function drawImportedLayers(){
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
  if(bounds.length) map.fitBounds(bounds, {padding:[40,40]});
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
function useLayerAsGrid(i){ pendingKind='grid'; pendingGeom=importedLayers[i].coords.slice(); pendingGenerated=false; showTab('setup'); }
function useLayerAsCorridor(i){ pendingKind='corridor'; pendingGeom=importedLayers[i].coords.slice(); pendingGenerated=false; showTab('setup'); }
function useLayerAsOrbit(i){ pendingKind='orbit'; pendingGeom=[importedLayers[i].lat, importedLayers[i].lon]; pendingGenerated=false; showTab('setup'); }
function useLayerAsWaypoints(i){
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
  var dist=0;
  for(var i=1;i<wps.length;i++) dist+=haversine(wps[i-1].lat,wps[i-1].lon,wps[i].lat,wps[i].lon);
  var flightSec = dist/(cfg.speed||5) + wps.reduce((s,w)=>s+(w.hover||0),0);
  var mins=Math.floor(flightSec/60), secs=Math.round(flightSec%60);
  var photoCount = wps.filter(w=>w.photo).length;
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
// Draws a schematic route preview (not a real map screenshot — Leaflet's raster
// tiles can't be read back into a canvas without the tile server sending
// permissive CORS headers, and the numbered waypoint markers are HTML divIcons,
// which no canvas-capture approach can rasterize at all). This is deliberately
// self-contained instead: fast, no CORS dependency, and still shows the actual
// route/waypoint count, which is the part that matters for picking a mission
// out in DJI Fly's list.
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
  var dist=0;
  for(var i=1;i<waypoints.length;i++) dist+=haversine(waypoints[i-1].lat,waypoints[i-1].lon,waypoints[i].lat,waypoints[i].lon);
  var flightSec = dist/(cfg.speed||5) + waypoints.reduce((s,w)=>s+(w.hover||0),0);
  var usableSec = usableBatteryMinutes(cfg)*60;
  return {minutes: flightSec/60, batteries: Math.max(1, Math.ceil(flightSec/usableSec))};
}
function openUploadPicker(){
  if(waypoints.length===0){ alert('No waypoints in the current mission.'); return; }
  document.getElementById('picker-overlay').classList.add('visible');
  document.getElementById('picker-log').innerHTML=''; document.getElementById('picker-log').classList.remove('visible');
  document.getElementById('picker-list').innerHTML='<div class="empty-hint">Scanning the controller...</div>';
  var battInfo = currentMissionBatteries();
  var warnEl = document.getElementById('picker-battery-warn');
  if(battInfo.batteries>1){
    warnEl.classList.add('visible');
    warnEl.innerHTML = '&#9888; This mission is ~'+Math.round(battInfo.minutes)+' min &mdash; needs about <b>'+battInfo.batteries+
      '</b> batteries. Uploading it as-is to one slot means the drone can\'t finish it on a single charge. '+
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
  var data = JSON.stringify({cfg:cfg, waypoints:waypoints, pois:pois}, null, 2);
  pywebview.api.save_project(data).then(function(res){ setStatus(res.msg); if(!res.ok && res.msg!=='Cancelled') alert(res.msg); });
}
function loadProject(){
  pywebview.api.load_project().then(function(res){
    if(!res.ok){ if(res.msg!=='Cancelled') alert(res.msg); return; }
    try{
      var data = JSON.parse(res.data);
      cfg = Object.assign({}, PRESETS.defaults, data.cfg||{});
      waypoints = data.waypoints||[];
      pois = data.pois||[];
      selectedWpIdx=null;
      pendingKind=null; pendingGeom=null; pendingGenerated=false;
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

def main():
    api = Api()
    window = webview.create_window(
        'Drone Mission Planner',
        html=HTML,
        js_api=api,
        width=1500, height=900,
        min_size=(1000, 650),
        background_color='#0d0d0d',
    )
    api.set_window(window)
    webview.start(debug=False)

if __name__ == '__main__':
    main()
