# Drone Mission Planner

A desktop tool for planning DJI waypoint missions — grid surveys, corridors, orbits,
manual routes — that actually imports KML/KMZ properly and exports missions DJI Fly
will load without a fight. Inspired by YMapper


---

## What it does

**Import** — KML and KMZ, parsed in a namespace-agnostic way so it doesn't choke on
nested `Folder`s, `MultiGeometry`, or whatever quirks Google Earth/QGIS/DJI's own tools
put in their output. Pulls out polygons, routes, and points, and you can snap new
drawing onto any of them while planning a mission, so a hand-drawn grid lines up exactly
with an imported boundary instead of eyeballing it.

**Four mission types**, all generating real, editable waypoints rather than an opaque
DJI "auto" pattern:

- **Grid survey** — draw or import a polygon, get a rotation-aware lawnmower pattern.
  Coverage is computed with point-in-polygon sampling (not scanline edge intersection,
  which turned out to silently drop rows of coverage at certain polygon shapes — see the
  commit history if curious). Spacing comes from your camera's real sensor/focal geometry
  and your chosen overlap %, or you can just type an exact spacing in meters if the
  overlap-percentage math isn't how you think about it. Optional crosshatch (second pass
  at 90°) for sites where a single sweep direction leaves gaps, and an optional 3D
  mapping mode that flies the area twice — once nadir, once oblique — which is the same
  method DJI Terra and Pix4D document for actually capturing building facades instead of
  just rooftops.
- **Corridor** — draw or import a route, get a multi-pass buffered flight path covering
  a configurable width either side of it. Good for roads, pipelines, rail, anything
  linear.
- **Orbit** — click a center point, get a circular path at a fixed radius with the
  gimbal continuously tracking the center. Supports stacked rings at different altitudes
  for scanning a tall or complex object (tower, silo, monument) from more than one
  elevation angle, which a single ring can't do well.
- **Manual** — click to drop waypoints one at a time, each independently editable.

Every generated mission shows a live estimate — photo count, area, flight time,
recommended shutter speed to avoid motion blur — **before** you commit to it, computed
by literally running the real generator client-side rather than a rough formula that
might disagree with what actually gets built.

**Capture-purpose presets** set the gimbal angle (and matching overlap %) from published
sources instead of a guessed default: flat 2D mapping, elevation/DEM work (a slight
oblique tilt breaks the "doming" distortion pure-nadir flights are known to produce),
3D/building models, facade inspection, roof inspection, corridor documentation. Each one
names where the number came from.

**Battery-aware.** Rated flight times for the Mini 4 Pro, Mini 5 Pro, Air 3/3S, and
Mavic 3/3 Pro are built in, including the extended "Plus" batteries where they exist.
Rated minutes aren't what you actually get, though — real-world usable endurance runs
70–80% of the lab figure, and standard practice reserves 20–30% battery for
return-to-home. Both factors are editable, not hidden, and the app uses them to warn you
when a mission needs more than one battery and to split it into sequential,
correctly-named sub-missions on export if you ask it to.

**Auto-rotation.** One button aligns the grid sweep to the polygon's longest edge
(minimum-bounding-rectangle via rotating calipers, not a heuristic) instead of you
dragging a slider by eye.

**Map layers** — Street, Satellite, Satellite with labels, Topographic, Dark, switchable
from a layer control, with imported layers and the flight path as separate toggleable
overlays.

**Mission replay** — play/pause/scrub through the generated mission on the map with a
moving marker, so you can sanity-check the flight path before you ever fly it.

**Export.** Every export is auto-named `{mission name}_{flight time}_{photo count}p_
{drone}.kmz` — you're prompted for the mission name at the start of each one, and can
rename it anytime in the Setup tab. The KMZ itself (`wpmz/template.kml` +
`wpmz/waylines.wpml`) is structured to match what DJI Fly's own parser actually accepts,
which is stricter and slightly different from the documented enterprise Cloud-API spec
built for DJI Pilot 2.

Projects save/load as plain `.json` so you can come back and keep editing a mission
later, or reuse a site's setup for a repeat survey.

---

## Requirements

- Windows 10 / 11
- [WebView2 Runtime](https://developer.microsoft.com/en-us/microsoft-edge/webview2/)
  (already on Windows 11; free download on Windows 10)

## Installation

**Run from source:**

```bash
pip install pywebview pywin32
python mission_planner.py
```

(`pywin32` is only needed for the "Upload to RC" button — the app runs fine without it,
that one button just won't work.)

**Or build a standalone EXE:** double-click `BUILD_EXE.bat`. `DroneMissionPlanner.exe`
shows up in the same folder afterward — no Python needed to run it from there.

## Usage

1. Import a KML/KMZ, or draw an area/route/orbit center directly on the map.
2. Pick your drone, camera, altitude, overlap, and (for grid missions) what you're
   actually trying to capture — the gimbal angle and overlap follow from that.
3. Check the live estimate, adjust anything, generate. Fine-tune individual waypoints in
   the Waypoints tab if needed — drag markers, edit altitude/speed/gimbal per point.
4. Export. If the mission needs more than one battery, use "Export by Battery" instead
   of the plain export to get it split automatically. Or skip exporting a file at all and
   hit **Upload to RC** to push the current mission straight to a connected controller.
5. If Upload to RC doesn't work for some reason, see the manual method below — it's not a
   simple drag-and-drop, DJI Fly is picky about this.

### Getting a mission onto a DJI RC / RC2

DJI Fly won't pick up an arbitrary KMZ dropped into its file system; it only recognizes
missions it created itself. The trick — same one **Upload to RC** automates — is: create
a throwaway waypoint mission in DJI Fly on the controller first, connect the RC to your
PC, then find that mission's folder and overwrite the file inside it.

**Upload to RC** does this over MTP (the same way Windows Explorer talks to the
controller — no extra driver needed beyond `pywin32`). **ADB does not work for this on
the RC2** — its `adbd` is present but firmware-hardened to refuse real host connections,
which is why `adb devices` shows it stuck "offline" forever no matter what you try with
cables, drivers, or developer-options toggling. That's confirmed by independent
reverse-engineering of the RC2, not a driver problem on your end, so don't waste time
chasing an ADB fix here.

If the automatic upload fails, or you don't have `pywin32` installed, do it by hand:
connect over USB, browse to
`This PC \ DJI RC 2 \ Internal shared storage \ Android \ data \ dji.go.v5 \ files \
waypoint`, find the newest subfolder — it's named with a GUID, and contains a `.kmz` file
sharing that same GUID as its filename — rename your exported mission to match that exact
filename, and overwrite it. Reopen the mission in DJI Fly and it loads your real
waypoints instead of the dummy ones.

Also: don't edit or re-save a mission from inside DJI Fly after importing it — it can
rewrite curved-turn missions in a way that breaks them, which is part of why this app
always exports straight-line, stop-at-waypoint turns rather than curved ones by default.

If a mission ever refuses to import, `Setup → Advanced` exposes the raw
`droneEnumValue`/`droneSubEnumValue` and sensor/focal-length values in case they need
overriding for your specific hardware/firmware combination.

## File structure

```
drone mission planner/
├── mission_planner.py   # the whole app
├── BUILD_EXE.bat         # Windows EXE builder
├── LICENSE
└── README.md
```

## Credits

Built with [pywebview](https://pywebview.flowrl.com/) (native window, embedded
browser), [Leaflet](https://leafletjs.com/) (the map), and tiles from
[OpenStreetMap](https://www.openstreetmap.org/), [Esri](https://www.esri.com/),
[OpenTopoMap](https://opentopomap.org/), and [CARTO](https://carto.com/).

The DJI WPML export structure and the `droneEnumValue=68` value (undocumented for
consumer drones anywhere official) came from studying
[YLabs-FPV/YMapper](https://github.com/YLabs-FPV/YMapper) and
[fcsonline/droneroute](https://github.com/fcsonline/droneroute), both MIT-licensed —
real credit to both for the reverse-engineering legwork that made a working export
possible without DJI's cooperation.

Developed by **[praet0rian (mark0)](https://github.com/0xpraet0rian)**.

## License

GNU General Public License v3.0 — see [LICENSE](LICENSE). Copyright (C) 2026
praet0rian (mark0). This program comes with absolutely no warranty; see the license for
details.
