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

### How grid/corridor missions actually fly (and why)

A grid or corridor mission does **not** put a real waypoint at every photo. Early
versions did, and real flight testing surfaced two problems at once: the aircraft
visibly hunting for position every time it stopped to "take" a photo (normal
position-hold behavior at that density, not a malfunction — but genuinely unpleasant to
watch and bad for image sharpness), and the RC2's own mission UI struggling once a
survey reached the waypoint counts that overlap/altitude settings produce trivially (a
modest survey can easily want thousands). Neither is a bug specific to this app —
independent reports (a Litchi forum thread, unrelated to this project) describe the same
RC2 instability on mapping missions with many closely-packed points, and DJI's own
consumer waypoint documentation only supports discrete stop-and-shoot camera actions, not
a continuous-flight interval trigger — that WPML mechanism exists but is documented as
enterprise-drone-only (M300/M350/M30/M3-series), not available on Mini/Air/Mavic-class
hardware.

What actually works, confirmed independently by [HOT's `drone-flightplan`](https://github.com/hotosm/drone-flightplan)
(a production tool used for real humanitarian drone mapping): put waypoints only at each
row's start and end, fly the row as one continuous straight line, and let the **camera's
own Timer/interval-shooting mode** — set manually on the controller before the flight,
since it can't be written into a WPML file — fire the shutter throughout. Cruise speed is
then *derived from* that fixed interval (`photo spacing ÷ camera interval`) rather than
the other way around, so a photo still lands roughly where it's supposed to. The app
tells you the exact interval and speed to set before every grid/corridor flight, in the
live estimate and again in the Waypoints tab. **This is a manual pre-flight step the app
cannot do for you — skip it and the mission still flies, it just won't take any
photos.** No-fly zones still work correctly with this: a zone cutting through a row
splits it into separate flyable segments instead of drawing a straight line through it.

Orbit and Manual missions are unaffected — their waypoints were always meant to be
individual shots, not a continuous strip, so they keep ordinary per-waypoint photo
actions.

Every generated mission shows a live estimate — photo count, area, flight time,
recommended shutter speed to avoid motion blur — **before** you commit to it, computed
by literally running the real generator client-side rather than a rough formula that
might disagree with what actually gets built.

Flight time accounts for acceleration, not just distance÷speed: the aircraft doesn't
teleport to cruise speed, and a short leg (the hop between two rows, for instance) can
mean it never gets there at all before decelerating again. A distance÷speed estimate
misses that and can undercount real flight time several times over; this one models the
accelerate/cruise/decelerate profile per leg (1.4 m/s² default, editable under Setup →
Aircraft & camera → Advanced, sourced from real acceleration-aware path-planning
research). It also feeds directly into "Export by Battery," so a mission that looked like
it fit on one battery under a flat estimate won't silently turn out not to — and that
same export step now also splits on waypoint count (DJI Fly's own 200-per-file cap on
current consumer drones, kept well clear of by default) whenever a mission needs it,
independent of battery life.

**Capture-purpose presets** set the gimbal angle (and matching overlap %) from published
sources instead of a guessed default: flat 2D mapping, vegetation/crop health, elevation/
DEM work (a slight oblique tilt breaks the "doming" distortion pure-nadir flights are
known to produce), 3D/building models, facade inspection, roof inspection, corridor
documentation. Each one names where the number came from. Picking the 3D-model preset
also turns on nadir + oblique double-grid capture automatically, rather than leaving that
as a separate step — a single oblique pass never images vertical surfaces like walls, so
the preset and the double-grid setting always move together instead of being two things
you can independently forget to match up.

Ground control points (placed under Site markup) pair with the elevation/DEM preset for
survey-grade accuracy in whatever SfM software (Agisoft Metashape, WebODM, Pix4D)
processes the photos afterward — this app doesn't do the 3D reconstruction itself, only
the flight planning and the GCP coordinate export.

**Battery-aware.** Rated flight times for the Mini 4 Pro, Mini 5 Pro, Air 3/3S, and
Mavic 3/3 Pro are built in, including the extended "Plus" batteries where they exist.
Rated minutes aren't what you actually get, though — real-world usable endurance runs
70–80% of the lab figure, and standard practice reserves 20–30% battery for
return-to-home. Both factors are editable, not hidden, and the app uses them to warn you
when a mission needs more than one battery and to split it into sequential,
correctly-named sub-missions on export if you ask it to.

**Auto-rotation.** One button aligns the grid sweep to the polygon's longest edge
(minimum-bounding-rectangle via rotating calipers, not a heuristic) instead of you
dragging a slider by eye. A second button aligns it to the wind instead, if you know it:
enter the direction the wind is coming *from* and the sweep rotates so the long passes
run parallel to that axis rather than across it — coverage-path research on UAV surveys
in wind (boustrophedon patterns specifically) finds that flying with/against the wind
holds a steadier ground speed than a sweep that has to fight a crosswind on every pass.

**Turn style**, per mission type, because "how the drone actually flies between
waypoints" turns out to be its own decision, not just a byproduct of the waypoints
themselves. DJI Fly supports stopping and rotating in place at each waypoint, or a smooth
continuous-curvature turn (a centripetal Catmull-Rom spline through the points) that
never fully stops. Grid/corridor missions default to stop-at-each-point, matching what
Pix4D/UgCS/DJI Terra all recommend for photogrammetry — a spline turn keeps drifting
gimbal position and ground speed through the corner, which is exactly what you don't want
when every photo needs a consistent, known camera position. Orbits default the other way:
a circular path built from stop-and-rotate segments flies a stuttering many-sided polygon
instead of a circle, so orbit missions use the smooth spline mode by default, which is
what actually produces a circular flight path. Both are overridable per mission type.

**No-fly / exclusion zones.** Draw a hole (a building, a hazard, restricted airspace)
and grid, corridor, and overview-lap coverage all skip it instead of flying straight over
it — crosshatch and 3D-mapping included. Nothing gets left out silently: if a zone would
actually remove waypoints from the mission you're generating, you're asked first, with a
count, and can choose to ignore the zone for that mission instead. A manually-placed
waypoint (Manual mode) inside a zone gets a warning instead — you clicked there on
purpose, so it's placed, not blocked. The live grid estimate honors zones exactly too —
it runs the same coverage math client-side, not an approximation, same as the rest of the
estimate. (Orbit missions aren't affected — a fixed-radius circle around a point isn't an
area-coverage sweep, so there's nothing for a zone to filter there.)

**Ground control points.** Drop reference markers at known coordinates directly on the
map — click to rename, click the &times; to remove — and export them as a plain
`Label,Latitude,Longitude` CSV, the format Pix4D/Metashape/WebODM all import directly for
correcting the orthomosaic afterward. They're never written into the flight-path export;
DJI Fly would try to fly to them if they were.

**Terrain-following altitude.** One button (in the Waypoints tab, after generating a
mission) looks up ground elevation under every waypoint from a free SRTM-derived
elevation API and shifts each altitude so real height above ground stays roughly
constant over sloped terrain, instead of a flat plane projected from the first waypoint.
DJI Fly's consumer app doesn't honor WPML's `aboveGroundLevel` height mode — that's a
Pilot 2 / FlightHub 2 / enterprise-drone feature — so this computes the offsets itself
and bakes them into ordinary `relativeToStartPoint` altitudes, the same approach
third-party planners like Litchi and Maven use to get terrain-following on consumer
drones. Needs internet access; SRTM data is ~30m resolution, so it's a real help on
hillsides, not a substitute for caution near sharp terrain features.

**Map layers** — Street, Satellite, Satellite with labels, Topographic, Dark, switchable
from a layer control, with imported layers, the flight path, exclusion zones, and ground
control points all as separate toggleable overlays.

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

The first time you open the app it asks which drone you fly — sets the right camera and
battery defaults from that, remembers it for next time, and it stays editable later under
Setup → Aircraft & camera (moved down near Safety & mission behaviour, since you'll rarely
touch it again after the first run).

1. Import a KML/KMZ, or draw an area/route/orbit center directly on the map.
2. Pick altitude, overlap, and (for grid missions) what you're actually trying to
   capture — the gimbal angle and overlap follow from that.
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
controller — no extra driver needed beyond `pywin32`), and always asks first: it lists
every mission slot on the controller — modified time, waypoint count, and an approximate
location, each read straight from the mission file already sitting there — and you pick
which one gets replaced. It does **not** guess or auto-pick the newest one for you, and if
the current mission needs more than one battery it warns you up front rather than
uploading something the drone can't finish on one charge. Worth knowing: DJI Fly's own
mission title (the name you type when saving on the controller) isn't stored anywhere MTP
can reach, so it can't be shown here — nobody's found where Android/RC2 keeps it, unlike
iOS which has an accessible database for it. A live green-on-black log shows each step as
it happens, so if something goes wrong you can see exactly where.

It also replaces the mission's map-preview thumbnail on the controller (`waypoint/
map_preview/<uuid>/<uuid>.jpg`) with a drawn schematic of the actual route — not a
screenshot of the app's own map, since Leaflet's tiles can't be read back into an image
without the tile server's cooperation, and the waypoint markers are HTML, which no
screenshot approach can capture at all. The replace is verified byte-for-byte (not just
"did a copy command run") and specifically checks for a known MTP failure mode where a
delete that hasn't fully propagated causes the device to silently create a renamed
duplicate instead of overwriting — if that happens you'll get a clear error instead of a
silent no-op.

Even when the file replace is fully verified, though: **DJI Fly's mission-*list* view
appears to cache the thumbnail independent of the file on disk**, and doesn't reliably
notice an external file replace. It does correctly regenerate the thumbnail from the
mission's actual content the moment you open that mission in the waypoint editor, so the
real content is never wrong — it's specifically the *list* thumbnail, before you've
opened the mission, that can lag. If it's still showing the old picture, that's DJI Fly's
own app-level cache, not a failed upload — opening the mission, restarting DJI Fly, or
rebooting the controller forces it to catch up.

**ADB does not work for this on the RC2** — its `adbd` is present but firmware-hardened to
refuse real host connections, which is why `adb devices` shows it stuck "offline" forever
no matter what you try with cables, drivers, or developer-options toggling. That's
confirmed by independent reverse-engineering of the RC2, not a driver problem on your
end, so don't waste time chasing an ADB fix here.

If the upload dialog doesn't work, or you don't have `pywin32` installed, do it by hand:
connect over USB, browse to
`This PC \ DJI RC 2 \ Internal shared storage \ Android \ data \ dji.go.v5 \ files \
waypoint`, pick a mission subfolder — it's named with a GUID, and contains a `.kmz` file
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
