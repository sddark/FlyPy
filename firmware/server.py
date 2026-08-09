# Ground-station portal: Pico W hosts its own AP while disarmed
# (self-contained, no router needed); microdot serves three pages: a status
# landing page (link/arm/IMU health, RC channels at a slow poll), the config
# form, and the waypoint mission editor. WiFi and the server are torn down
# before arming, so they never share CPU with the flight loop.
#
# The earlier bench-test pages (servo/motor live drive, RC channel detail,
# IMU live view) were deliberately removed: they were debugging aids whose
# 200-300 ms polling dominated server load (each request pays a gc.collect),
# and the portal's job is ground station, not real-time telemetry. Bench
# debugging happens over USB/mpremote instead.
#
# Fully self-contained HTML/CSS/JS (no CDN, no external fonts, no map tiles)
# since the AP has no internet -- the mission editor therefore uses manual
# coordinate entry plus a local top-down canvas plot, not a slippy map.
# Colors follow a fixed status palette (good/warning/critical) so link/arm/
# IMU state never reads from hue alone -- every badge pairs its color with
# an icon-equivalent dot *and* a text label.

import gc
import time

import network
from microdot import Microdot, Response

import config as config_module
import logic as logic_module
import mission as mission_module
import pins
from compass import QMC5883, flat_field_angle_deg

_AP_IP = "192.168.4.1"
_MPU6050_ADDRESS = 0x68  # see imu.py -- used here only for a presence check

_CONFIG_GROUPS = (
    ("Attitude & rates", (
        "pid_pitch_p", "pid_pitch_i", "pid_pitch_d", "pid_pitch_ff",
        "pid_yaw_p", "pid_yaw_i", "pid_yaw_d", "pid_yaw_ff",
        "pitch_level_p", "pitch_rate_limit_dps",
        "pitch_angle_max_deg", "rate_yaw_dps", "turn_assist_gain",
    )),
    ("Mixer", (
        "mixer_left_pitch", "mixer_left_yaw",
        "mixer_right_pitch", "mixer_right_yaw",
    )),
    ("Servo endpoints", (
        "servo_left_min_us", "servo_left_max_us",
        "servo_right_min_us", "servo_right_max_us",
    )),
    ("RC channel map", (
        "channel_roll", "channel_pitch", "channel_throttle",
        "channel_yaw", "channel_arm", "channel_mode",
    )),
    ("Arming & failsafe", (
        "arm_max_throttle_us", "failsafe_link_timeout_ms",
    )),
    ("Navigation — guidance", (
        "nav_heading_p", "nav_xtrack_p", "nav_xtrack_limit_deg",
        "nav_alt_p", "nav_wp_radius_m", "nav_loiter_radius_m",
    )),
    ("Navigation — throttle & angles", (
        "nav_cruise_throttle_pct", "nav_min_throttle_pct",
        "nav_max_throttle_pct", "nav_pitch_to_throttle",
        "nav_climb_angle_deg", "nav_dive_angle_deg",
    )),
    ("Navigation — engage gates", (
        "nav_min_sats", "nav_max_h_acc_m", "nav_min_ground_speed_ms",
        "nav_max_safe_distance_m",
    )),
    ("Navigation — automatic landing", (
        "nav_land_heading_deg", "nav_land_approach_length_m",
        "nav_land_approach_alt_m", "nav_land_glide_alt_m",
        "nav_land_glide_pitch_deg",
    )),
    ("WiFi", (
        "wifi_ssid_suffix", "wifi_password",
    )),
)

_STYLE = """
:root {
  color-scheme: light;
  --page: #f9f9f7;
  --surface: #fcfcfb;
  --text-primary: #0b0b0b;
  --text-secondary: #52514e;
  --muted: #898781;
  --border: rgba(11,11,11,0.10);
  --gridline: #e1e0d9;
  --accent: #2a78d6;
  --accent-track: #cde2fb;
  --good: #0ca30c;
  --warning: #fab219;
  --critical: #d03b3b;
}
@media (prefers-color-scheme: dark) {
  :root {
    color-scheme: dark;
    --page: #0d0d0d;
    --surface: #1a1a19;
    --text-primary: #ffffff;
    --text-secondary: #c3c2b7;
    --muted: #898781;
    --border: rgba(255,255,255,0.10);
    --gridline: #2c2c2a;
    --accent: #3987e5;
    --accent-track: #184f95;
    --good: #0ca30c;
    --warning: #fab219;
    --critical: #d03b3b;
  }
}
* { box-sizing: border-box; }
body {
  font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
  background: var(--page);
  color: var(--text-primary);
  max-width: 30em;
  margin: 0 auto;
  padding: 0 1em 2em;
}
header { padding: 1em 0 0.25em; }
h1 { font-size: 1.15em; margin: 0; }
.subtitle { color: var(--text-secondary); font-size: 0.85em; margin: 0.2em 0 0; }
nav {
  display: flex; flex-wrap: wrap; gap: 0.4em;
  margin: 0.75em 0 1.25em;
  border-bottom: 1px solid var(--border);
  padding-bottom: 0.75em;
}
nav a {
  color: var(--text-secondary); text-decoration: none; font-size: 0.9em;
  padding: 0.3em 0.6em; border-radius: 999px;
}
nav a.active { background: var(--accent); color: #fff; }
h2 { font-size: 0.85em; text-transform: uppercase; letter-spacing: 0.04em;
  color: var(--muted); margin: 1.5em 0 0.6em; }
.tiles {
  display: grid; grid-template-columns: repeat(auto-fit, minmax(135px, 1fr));
  gap: 0.6em;
}
.tile {
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 10px; padding: 0.7em 0.8em;
}
.tile-label { font-size: 0.8em; color: var(--text-secondary); }
.tile-value {
  font-size: 1.3em; font-weight: 600; margin: 0.15em 0 0.35em;
  font-variant-numeric: tabular-nums;
}
.badge {
  display: inline-flex; align-items: center; gap: 0.4em;
  font-size: 0.8em; color: var(--text-secondary);
}
.dot { width: 8px; height: 8px; border-radius: 50%; background: var(--muted); flex: none; }
.badge.good .dot { background: var(--good); }
.badge.warning .dot { background: var(--warning); }
.badge.critical .dot { background: var(--critical); }
.meter { margin-bottom: 0.9em; }
.meter-label {
  display: flex; justify-content: space-between; font-size: 0.85em;
  color: var(--text-secondary); margin-bottom: 0.3em;
}
.meter-value { font-variant-numeric: tabular-nums; }
.meter-track {
  position: relative; height: 10px; border-radius: 999px;
  background: var(--accent-track); overflow: hidden;
}
.meter-center {
  position: absolute; left: 50%; top: 0; bottom: 0; width: 1px;
  background: var(--border);
}
.meter-fill {
  position: absolute; top: 0; bottom: 0; background: var(--accent);
  border-radius: 999px;
}
.nav-cards { display: grid; gap: 0.6em; }
.nav-card {
  display: block; background: var(--surface); border: 1px solid var(--border);
  border-radius: 10px; padding: 0.7em 0.9em; text-decoration: none;
  color: var(--text-primary);
}
.nav-card .title { font-weight: 600; }
.nav-card .desc { color: var(--text-secondary); font-size: 0.85em; margin-top: 0.15em; }
label { display: block; margin-top: 0.7em; font-size: 0.9em; }
input[type="text"] {
  width: 100%; padding: 0.4em 0.5em; margin-top: 0.2em;
  background: var(--surface); color: var(--text-primary);
  border: 1px solid var(--border); border-radius: 6px;
}
input[type="checkbox"] { width: auto; margin-right: 0.4em; }
.range-row { display: flex; align-items: center; gap: 0.6em; margin-top: 0.3em; }
.range-row input[type="range"] { flex: 1; accent-color: var(--accent); }
input[type="range"]:disabled { opacity: 0.4; }
.range-value {
  min-width: 3.6em; text-align: right; color: var(--text-secondary);
  font-size: 0.9em; font-variant-numeric: tabular-nums;
}
details {
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 10px; margin-top: 0.6em; padding: 0 0.9em;
}
details[open] { padding-bottom: 0.7em; }
summary { padding: 0.8em 0; font-weight: 600; cursor: pointer; list-style: none; }
summary::-webkit-details-marker { display: none; }
summary::before { content: "+ "; color: var(--accent); }
details[open] summary::before { content: "- "; }
button {
  margin-top: 1.1em; padding: 0.6em 1.1em; border-radius: 8px; border: none;
  background: var(--accent); color: #fff; font-size: 1em;
}
.warn-text { color: var(--critical); font-weight: bold; }
canvas.plot {
  width: 100%; display: block; touch-action: none; margin-top: 0.8em;
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 10px;
}
.fix-bar {
  display: flex; flex-wrap: wrap; gap: 0.5em 1.2em; align-items: center;
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 10px; padding: 0.65em 0.9em; margin-bottom: 0.5em;
}
.fix-bar .label { font-size: 0.75em; text-transform: uppercase;
  letter-spacing: 0.04em; color: var(--muted); display: block; }
.fix-bar .value { font-variant-numeric: tabular-nums; font-weight: 600; }
.chip {
  display: inline-flex; align-items: center; gap: 0.4em;
  font-size: 0.8em; padding: 0.2em 0.6em; border-radius: 999px;
  border: 1px solid var(--border); background: var(--page);
  color: var(--text-secondary); font-variant-numeric: tabular-nums;
}
.chip .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--muted); }
.chip.good .dot { background: var(--good); }
.chip.warning .dot { background: var(--warning); }
.chip.critical .dot { background: var(--critical); }
.btn-quiet {
  margin: 0; padding: 0.35em 0.7em; font-size: 0.9em; border-radius: 7px;
  border: 1px solid var(--border); background: var(--surface);
  color: var(--text-primary);
}
.btn-quiet:disabled, button:disabled { opacity: 0.45; }
.toolbar { display: flex; flex-wrap: wrap; gap: 0.4em; align-items: center;
  margin-top: 0.7em; }
.toolbar button { margin-top: 0; }
.readout { font-size: 0.85em; color: var(--text-secondary);
  font-variant-numeric: tabular-nums; margin-top: 0.5em; min-height: 1.3em; }
.wp-strip { display: flex; flex-direction: column; gap: 0.3em; margin-top: 0.6em; }
.wp-item {
  display: grid; grid-template-columns: 1.6em 1fr auto auto;
  align-items: center; gap: 0.5em; font-size: 0.9em;
  padding: 0.45em 0.6em; border-radius: 8px;
  border: 1px solid var(--border); background: var(--surface);
}
.wp-item.sel { border-color: var(--accent); box-shadow: 0 0 0 1px var(--accent); }
.wp-item .idx {
  width: 1.6em; height: 1.6em; border-radius: 50%; background: var(--accent);
  color: #fff; font-size: 0.75em; font-weight: 700;
  display: flex; align-items: center; justify-content: center;
}
.wp-item.first .idx { background: var(--good); }
.wp-item .coords { color: var(--text-secondary); font-variant-numeric: tabular-nums; }
.wp-item .alt { display: flex; align-items: center; gap: 0.25em; }
.wp-item .alt input, .home-fields input {
  padding: 0.2em 0.35em; font-size: 0.9em;
  background: var(--page); color: var(--text-primary);
  border: 1px solid var(--border); border-radius: 6px;
  font-variant-numeric: tabular-nums;
}
.wp-item .alt input { width: 4.2em; }
.home-fields { display: flex; flex-wrap: wrap; gap: 0.5em; align-items: center;
  margin-bottom: 0.8em; }
.home-fields input { width: 8em; }
.empty-hint { text-align: center; color: var(--muted); font-size: 0.9em;
  padding: 1.2em 0.6em; }
input:focus-visible, button:focus-visible, textarea:focus-visible {
  outline: 2px solid var(--accent); outline-offset: 1px;
}
.rule-row { display: flex; justify-content: space-between; gap: 0.7em;
  padding: 0.35em 0.55em; border-radius: 7px; font-size: 0.88em;
  border: 1px solid var(--border); background: var(--surface);
  margin-bottom: 0.25em; }
.rule-row.bad { border-color: var(--critical); }
.rule-t { font-family: ui-monospace, Menlo, monospace; color: var(--accent); }
.rule-row.bad .rule-t { color: var(--critical); }
.rule-v { color: var(--text-secondary); font-variant-numeric: tabular-nums;
  text-align: right; }
.code, .code-pre, textarea.code {
  font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  font-size: 13px; line-height: 1.65;
}
.ed { position: relative; border: 1px solid var(--border); border-radius: 10px;
  background: var(--page); overflow: hidden; }
.ed.focus { border-color: var(--accent); box-shadow: 0 0 0 1px var(--accent); }
textarea.code { display: block; width: 100%; min-height: 13em; resize: vertical;
  border: none; outline: none; background: transparent; white-space: pre;
  overflow-wrap: normal; overflow-x: auto; position: relative; z-index: 2;
  color: transparent; caret-color: var(--text-primary); padding: 0.75em 0.85em; }
.code-pre { position: absolute; inset: 0; margin: 0; z-index: 1;
  pointer-events: none; white-space: pre; overflow: hidden;
  color: var(--text-primary); padding: 0.75em 0.85em; }
.t-kw { color: #c678dd; font-weight: 600; }
.t-num { color: #d19a66; }
.t-par { color: var(--accent); font-weight: 600; }
.t-pin { color: #e5a03c; font-weight: 600; }
.t-in { color: #56b6c2; }
.t-fn { color: #61afef; }
.t-cmt { color: var(--muted); font-style: italic; }
.t-str { color: #98c379; }
.t-bad { color: var(--critical); text-decoration: underline wavy var(--critical); }
.cols { display: grid; grid-template-columns: 1fr; gap: 0.7em; margin-top: 1.1em; }
@media (min-width: 42em) { .cols { grid-template-columns: 1fr 1fr; } }
.panel { background: var(--surface); border: 1px solid var(--border);
  border-radius: 11px; padding: 0.75em 0.85em; }
.panel > h2 { margin: 0; }
.panel .sub { color: var(--muted); font-size: 0.78em; margin: 0.1em 0 0.5em; }
.filter { width: 100%; padding: 0.35em 0.5em; font-size: 0.85em;
  margin-bottom: 0.45em; background: var(--page); color: var(--text-primary);
  border: 1px solid var(--border); border-radius: 7px; }
.scroll { max-height: 26em; overflow-y: auto; }
details.grp { background: transparent; border: none; border-radius: 0;
  margin: 0; padding: 0; border-top: 1px solid var(--border); }
details.grp summary { padding: 0.4em 0; font-size: 0.7em; text-transform: uppercase;
  letter-spacing: 0.05em; color: var(--muted); font-weight: 700; }
details.grp summary::before { content: "+ "; }
details.grp[open] summary::before { content: "- "; }
details.grp .cnt { color: var(--muted); font-weight: 400; text-transform: none; }
.rws { display: flex; flex-direction: column; gap: 0.08em; padding-bottom: 0.35em; }
.rw { display: grid; grid-template-columns: 1fr auto; gap: 0.5em;
  align-items: baseline; padding: 0.22em 0.4em; border-radius: 6px;
  cursor: pointer; border: 1px solid transparent; font-size: 0.83em; }
.rw:hover { background: var(--page); border-color: var(--border); }
.rw .n { font-family: ui-monospace, Menlo, monospace; }
.rw .n.in { color: #56b6c2; } .rw .n.par { color: var(--accent); }
.rw .n.pin { color: #e5a03c; }
.rw .d { color: var(--muted); text-align: right; white-space: nowrap; }
.rw.locked { cursor: default; opacity: 0.45; }
.rw.locked:hover { background: none; border-color: transparent; }
"""

# The page shell is split into pieces and streamed (see _page_chunks) rather
# than merged into one big string, because RP2040 has no PSRAM: a single
# ~10-15 KB allocation for a fully-merged page can fail with a MemoryError
# under real running conditions (WiFi + asyncio + everything else in
# main.py sharing one small heap) even when gc.mem_free() reports plenty
# free overall -- MicroPython's collector doesn't compact, so "plenty free"
# doesn't mean "one contiguous block that big exists." Streaming small,
# already-existing constants (zero extra allocation to yield a reference)
# plus small per-request fragments sidesteps that entirely. _HEAD/_STYLE/
# _FOOTER are the static, zero-copy pieces; _AFTER_STYLE is the one small
# per-request merge (subtitle + nav highlighting).
_HEAD = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Pico Wing FC</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>"""

_AFTER_STYLE = """</style></head>
<body>
<header><h1>Pico Wing FC</h1><p class="subtitle">{subtitle}</p></header>
<nav>
<a href="/" class="{nav_status}">Status</a>
<a href="/config" class="{nav_config}">Config</a>
<a href="/mission" class="{nav_mission}">Mission</a>
<a href="/logic" class="{nav_logic}">Logic</a>
</nav>
"""

_FOOTER = "</body></html>"

_NAV_PAGES = ("status", "config", "mission", "logic")


_WRITE_CHUNK = 1024


def _chunked(text):
    # Every yielded chunk becomes one socket write, and a single 15 KB write
    # of the mission script was reaching the browser truncated -- which shows
    # up as the whole script failing to parse, so none of its functions exist
    # and any handler referencing one raises "can't find variable".
    #
    # Sliced lazily here rather than pre-split into tuples at import: those
    # tuples are copies, and holding them alongside the originals added ~28 KB
    # of permanent heap on a board where memory is the binding constraint (it
    # pushed `import server` itself into MemoryError). Each slice made here is
    # a ~1 KB temporary that dies with the request.
    if len(text) <= _WRITE_CHUNK:
        yield text
        return
    for start in range(0, len(text), _WRITE_CHUNK):
        yield text[start:start + _WRITE_CHUNK]


def _page_chunks(title, body_chunks, active, subtitle=""):
    # Plain (non-async) generator -- microdot streams any body with
    # __next__, async or not (see lib/microdot/microdot.py body_iter()).
    nav_classes = {"nav_" + name: ("active" if name == active else "") for name in _NAV_PAGES}
    yield _HEAD
    for piece in _chunked(_STYLE):
        yield piece
    yield _AFTER_STYLE.format(
        subtitle=(subtitle or title) + " &middot; build " + _BUILD, **nav_classes
    )
    for chunk in body_chunks:
        for piece in _chunked(chunk):
            yield piece
    yield _FOOTER


def _status_body():
    return (
        '<h2>Live status</h2>'
        '<div class="tiles">'
        '<div class="tile"><div class="tile-label">RC link</div>'
        '<div class="tile-value" id="link-value">--</div>'
        '<div class="badge" id="link-badge"><span class="dot"></span><span id="link-text">checking&hellip;</span></div></div>'
        '<div class="tile"><div class="tile-label">Flight mode</div>'
        '<div class="tile-value" id="mode-value">--</div>'
        '<div class="badge"><span class="dot"></span><span>set via TX switch</span></div></div>'
        '<div class="tile"><div class="tile-label">Arm switch</div>'
        '<div class="tile-value" id="arm-value">--</div>'
        '<div class="badge" id="ready-badge"><span class="dot"></span><span id="ready-text">checking&hellip;</span></div></div>'
        '<div class="tile"><div class="tile-label">IMU (MPU6050)</div>'
        '<div class="tile-value" id="imu-value">--</div>'
        '<div class="badge" id="imu-badge"><span class="dot"></span><span id="imu-text">checking&hellip;</span></div></div>'
        '<div class="tile"><div class="tile-label">GPS (M10)</div>'
        '<div class="tile-value" id="gps-value">--</div>'
        '<div class="badge" id="gps-badge"><span class="dot"></span><span id="gps-text">checking&hellip;</span></div></div>'
        '<div class="tile"><div class="tile-label">Compass</div>'
        '<div class="tile-value" id="compass-value">--</div>'
        '<div class="badge" id="compass-badge"><span class="dot"></span><span id="compass-text">checking&hellip;</span></div></div>'
        '</div>'
        '<h2>Active rules</h2>'
        '<div id="rules"><p class="subtitle">checking&hellip;</p></div>'
        '<h2>Ground station</h2>'
        '<div class="nav-cards">'
        '<a class="nav-card" href="/config"><div class="title">Config</div>'
        '<div class="desc">PIDs, rates, mixer gains, failsafe, WiFi</div></a>'
        '<a class="nav-card" href="/mission"><div class="title">Mission</div>'
        '<div class="desc">Waypoint plan for autonomous navigation</div></a>'
        '<a class="nav-card" href="/logic"><div class="title">Logic</div>'
        '<div class="desc">Rules that adjust parameters and drive pins</div></a>'
        '</div>'
        '<script>'
        'function setMeter(id,value,bipolar){'
        'const track=document.getElementById("m-"+id);'
        'const fill=track.querySelector(".meter-fill");'
        'const label=document.getElementById("v-"+id);'
        'const v=Math.max(bipolar?-1:0,Math.min(1,value));'
        'if(bipolar){if(v>=0){fill.style.left="50%";fill.style.width=(v*50)+"%";}'
        'else{fill.style.left=(50+v*50)+"%";fill.style.width=(-v*50)+"%";}}'
        'else{fill.style.left="0%";fill.style.width=(v*100)+"%";}'
        'label.textContent=v.toFixed(2);}'
        'function badge(el,ok,okText,badText){'
        'el.className="badge "+(ok?"good":"critical");'
        'el.querySelector("span:last-child").textContent=ok?okText:badText;}'
        # Rules evaluate only while armed, so the values shown here are from
        # the most recent armed session -- which is exactly what you want to
        # inspect after a flight, standing next to the aircraft.
        'function renderRules(L){'
        'const host=document.getElementById("rules");'
        'if(!L||!L.rules.length){'
        'host.innerHTML=L&&L.errors.length'
        '?\'<p class="subtitle" style="color:var(--critical)">\'+L.errors[0]+"</p>"'
        ':\'<p class="subtitle">No rules. Add them on the Logic page.</p>\';'
        'return;}'
        'host.innerHTML=L.rules.map(function(r){'
        'const bad=r.error!==null;'
        'const val=bad?r.error:(r.value===null?"not evaluated yet"'
        ':(r.kind==="gpio"?(r.value?"HIGH":"LOW")'
        ':String(Math.round(r.value*1000)/1000)+(r.clamped?" (clamped)":"")));'
        'return \'<div class="rule-row\'+(bad?" bad":"")+\'">\''
        '+\'<span class="rule-t">\'+r.target+"</span>"'
        '+\'<span class="rule-v">\'+val+"</span></div>";'
        '}).join("");}'
        'async function poll(){'
        'try{'
        'const r=await fetch("/status.json");const d=await r.json();'
        'document.getElementById("link-value").textContent=d.link_alive?(d.age_ms+" ms"):"--";'
        'badge(document.getElementById("link-badge"),d.link_alive,"link OK","no link");'
        'document.getElementById("mode-value").textContent=d.mode;'
        'document.getElementById("arm-value").textContent=d.arm?"ON":"off";'
        'const readyBadge=document.getElementById("ready-badge");'
        'readyBadge.className="badge "+(d.arm?"warning":"good");'
        'readyBadge.querySelector("span:last-child").textContent=d.arm?"switch is ON":"disarmed";'
        'badge(document.getElementById("imu-badge"),d.imu_detected,"detected","not wired");'
        'document.getElementById("imu-value").textContent=d.imu_detected?"OK":"--";'
        'const g=d.gps;'
        'document.getElementById("gps-value").textContent=g.fix_ok?(g.num_sv+" sats"):"no fix";'
        'badge(document.getElementById("gps-badge"),g.fix_ok,'
        'g.lat_deg.toFixed(5)+", "+g.lon_deg.toFixed(5),'
        'g.pvt_age_ms===null?"no data from module":(g.num_sv+" sats, acquiring"));'
        'document.getElementById("compass-value").textContent='
        'd.compass.detected?(d.compass.field_angle_deg.toFixed(0)+"°"):"--";'
        'badge(document.getElementById("compass-badge"),d.compass.detected,"detected","not wired");'
        'renderRules(d.logic);'
        '}catch(e){}'
        # 1 s poll: the status page is a health check, not live telemetry.
        'finally{setTimeout(poll,1000);}}'
        'poll();'
        '</script>'
    )


def _meter(key, label):
    bipolar = key != "throttle"
    return (
        '<div class="meter"><div class="meter-label"><span>{label}</span>'
        '<span class="meter-value" id="v-{key}">0.00</span></div>'
        '<div class="meter-track" id="m-{key}">'
        + ('<div class="meter-center"></div>' if bipolar else "")
        + '<div class="meter-fill"></div></div></div>'
    ).format(label=label, key=key)


def _slider_field(name, value, minimum, maximum, step, label=None, oninput=None, disabled=False):
    return (
        '<label>{label}<div class="range-row">'
        '<input type="range" id="in-{name}" name="{name}" min="{minimum}" max="{maximum}" '
        'step="{step}" value="{value}" oninput="{oninput}"{disabled}>'
        '<span class="range-value">{value}</span>'
        '</div></label>'
    ).format(
        label=label if label is not None else name,
        name=name,
        minimum=minimum,
        maximum=maximum,
        step=step,
        value=value,
        oninput=oninput if oninput is not None else "this.nextElementSibling.textContent=this.value",
        disabled=" disabled" if disabled else "",
    )


def _logic_catalogue(free_pins):
    # Built ONCE at import time, not per app creation: this is ~5 KB of JSON
    # plus the intermediate lists, and doing it at runtime -- after WiFi is
    # up and the heap is carved up -- failed outright with MemoryError. Same
    # reasoning as the page bodies below.
    #
    # Inputs come from logic.INPUT_CATALOGUE so the page can never advertise
    # a name the engine doesn't provide; outputs come from config.SCHEMA so
    # it can never miss one.
    import json

    inputs = []
    for group, entries in logic_module.INPUT_CATALOGUE:
        inputs.append([group, [[name, note] for name, _v, note in entries]])
    inputs.append(["Raw RC channels",
                   [["ch%d" % n, "CRSF 172..1811"] for n in range(1, 17)]])
    inputs.append(["Analog pins",
                   [["adc%d" % n, "volts, GP%d" % n]
                    for n in free_pins if n >= 26]])
    inputs.append(["Digital pins (read)",
                   [["gpio%d" % n, "0 or 1"] for n in free_pins if n < 26]])

    groups = [["Flight demands (refused only in failsafe)",
               [[name, "%g to %g" % logic_module.DEMAND_TARGETS[name]]
                for name in ("throttle_demand", "pitch_demand",
                             "yaw_demand", "roll_demand")]]]
    for title, names in _CONFIG_GROUPS:
        rows = []
        for name in names:
            if name in logic_module.EXCLUDED_TARGETS:
                continue
            _d, low, high, _s = config_module.SCHEMA[name]
            rows.append([name, "%g-%g" % (low, high)])
        if rows:
            groups.append([title, rows])
    groups.append(["Digital pins",
                   [["gpio%d" % n, "on / off"] for n in free_pins if n < 26]])
    groups.append(["PWM pins",
                   [["pwm%d" % n, "500-2500 us"] for n in free_pins if n < 26]])
    return json.dumps({"inputs": inputs, "outputs": groups})


def _escape(text):
    # Minimal HTML attribute/text escaping for the string config values --
    # a quote in the SSID suffix or password must not break the form markup
    # (which would make the value uneditable from the web UI).
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def _config_body_chunks(current_config, message=""):
    # The only page whose body varies per request (every parameter's current
    # value is server-rendered into its slider) -- so unlike every other
    # page here, it can't be a precomputed constant. Yielded field-by-field
    # (each a few hundred bytes) rather than joined into one ~8 KB string,
    # since that join was the original MemoryError (see git history / the
    # gc.collect() before_request hook above -- this is the real fix, that
    # was only a mitigation).
    yield "<p>{message}</p><form method=\"post\" action=\"/config\">".format(message=message)
    for title, names in _CONFIG_GROUPS:
        yield "<details><summary>{title}</summary>".format(title=title)
        for name in names:
            value = current_config[name]
            if isinstance(value, str):
                yield (
                    '<label>{name}<input type="text" name="{name}" '
                    'value="{value}"></label>'.format(name=name, value=_escape(value))
                )
            else:
                _, minimum, maximum, step = config_module.SCHEMA[name]
                yield _slider_field(name, value, minimum, maximum, step)
        yield "</details>"
    yield '<button type="submit">Save</button></form>'


# Mission editor: tap-to-place. The canvas is the *input*, not a preview --
# you work in metres from a reference position (the module's own GPS fix
# where available), because that's the frame of reference you actually have
# standing in a field. Latitude/longitude are derived, displayed, and only
# materialize as lat/lon in the POST body; the flight controller stores and
# validates {"waypoints":[{lat,lon,alt_m}], "end_action":.., "alt_frame":..}.
#
# alt_frame is mission-wide rather than per-waypoint (which is what INAV and
# ArduPilot both do) because those stacks need per-leg datums to mix terrain
# following with absolute altitudes, and this airframe carries no terrain
# data to follow. Per-waypoint stays a compatible extension: a waypoint key
# would simply default to the mission's.
#
# The reference position is fetched ONCE on load, plus on an explicit
# button press -- deliberately not polled. Fast polling from portal pages
# is what made the server unusable before the ground-station cut.
_LOGIC_MARKUP = (
    '<p>One line per rule: <code>name = expression</code>. Evaluated 25&times;'
    ' a second while armed; every result is clamped to that parameter\'s'
    ' configured range. Comment a line out with <code>#</code> to disable'
    ' it.</p>'
    '<div class="ed" id="ed">'
    '<pre class="code-pre code" id="hl"></pre>'
    '<textarea class="code" id="src" spellcheck="false" autocapitalize="off"'
    ' autocomplete="off" autocorrect="off"></textarea>'
    '</div>'
    '<div class="toolbar">'
    '<button type="button" id="save-btn">Save</button>'
    '<span class="readout" id="msg" style="margin-top:0"></span>'
    '</div>'
    '<div class="cols">'
    '<div class="panel"><h2>Inputs</h2>'
    '<p class="sub" id="in-sub">readable</p>'
    '<input class="filter" id="in-filter" placeholder="filter inputs...">'
    '<div class="scroll" id="in-list"></div></div>'
    '<div class="panel"><h2>Outputs</h2>'
    '<p class="sub" id="out-sub">assignable</p>'
    '<input class="filter" id="out-filter" placeholder="filter outputs...">'
    '<div class="scroll" id="out-list"></div></div>'
    '</div>'
    # Error handler installed before the main script, so a script that fails
    # to parse still reports itself rather than leaving a dead page.
    '<script>'
    'window.__err=function(m){var el=document.getElementById("msg");'
    'if(el){el.textContent=m;el.style.color="var(--critical)";}};'
    'window.onerror=function(m,s,l){'
    'window.__err("Page script error: "+m+" (line "+l+")");return false;};'
    '</script>'
)

_LOGIC_SCRIPT = r"""<script>
var CAT = null, NAMES = {}, OUTS = {};
var src = document.getElementById('src');
var hl = document.getElementById('hl');
var KW = ['if','else','and','or','not','True','False','None','in','is'];
var FN = ['min','max','abs','round','int','float','bool','pow'];

function esc(s){
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}
function classify(tok){
  if (KW.indexOf(tok) >= 0) return 't-kw';
  if (FN.indexOf(tok) >= 0) return 't-fn';
  if (OUTS[tok]) return OUTS[tok] === 'pin' ? 't-pin' : 't-par';
  if (NAMES[tok]) return NAMES[tok] === 'pin' ? 't-pin' : 't-in';
  return 't-bad';
}
function highlight(line){
  if (line.trim().charAt(0) === '#') {
    return '<span class="t-cmt">' + esc(line) + '</span>';
  }
  // Tokenise the RAW line, escaping only what is emitted between tokens.
  // Escaping the whole line first and tokenising afterwards corrupts the
  // entities: "&lt;" contains "lt", which matches the identifier pattern,
  // so it was wrapped in a span and stopped rendering as "<" -- comparison
  // operators appeared on screen as literal &lt; and &gt;. Identifiers and
  // numbers cannot contain HTML specials, so only the gaps and string
  // literals need escaping.
  var out = '';
  var re = /('[^']*')|(\d+\.?\d*)|([A-Za-z_][A-Za-z0-9_]*)/g;
  var last = 0, m;
  while ((m = re.exec(line)) !== null) {
    out += esc(line.slice(last, m.index));
    var tok = m[0];
    if (tok.charAt(0) === "'") {
      out += '<span class="t-str">' + esc(tok) + '</span>';
    } else if (tok.charCodeAt(0) >= 48 && tok.charCodeAt(0) <= 57) {
      out += '<span class="t-num">' + tok + '</span>';
    } else {
      out += '<span class="' + classify(tok) + '">' + tok + '</span>';
    }
    last = m.index + tok.length;
  }
  return out + esc(line.slice(last));
}
function paint(){
  hl.innerHTML = src.value.split('\n').map(function (l) {
    return highlight(l) || '&nbsp;';
  }).join('\n');
}
function setMsg(text, ok){
  var el = document.getElementById('msg');
  el.textContent = text;
  el.style.color = ok ? 'var(--good)' : 'var(--critical)';
}
function rowHtml(name, cls, detail){
  return '<div class="rw" data-ins="' + name + '">'
    + '<span class="n ' + cls + '">' + name + '</span>'
    + '<span class="d">' + detail + '</span></div>';
}
function renderList(hostId, filterId, groups, cls, subId, noun){
  var q = document.getElementById(filterId).value.trim().toLowerCase();
  var html = '', total = 0, shown = 0;
  groups.forEach(function (g, gi) {
    var rows = '', n = 0;
    g[1].forEach(function (entry) {
      total++;
      if (q && (entry[0] + ' ' + entry[1]).toLowerCase().indexOf(q) < 0) return;
      n++; shown++;
      rows += rowHtml(entry[0],
        /^(gpio|pwm|adc)\d+$/.test(entry[0]) ? 'pin' : cls, entry[1]);
    });
    if (n) {
      html += '<details class="grp"' + (q || gi < 2 ? ' open' : '') + '>'
        + '<summary>' + g[0] + ' <span class="cnt">' + n + '</span></summary>'
        + '<div class="rws">' + rows + '</div></details>';
    }
  });
  document.getElementById(hostId).innerHTML = html
    || '<p class="subtitle">no matches</p>';
  document.getElementById(subId).textContent =
    (q ? shown + ' of ' + total : total + ' ' + noun) + ' — click to insert';
}
function renderAll(){
  // The filter boxes are live from first paint, but the catalogue arrives
  // asynchronously -- typing before it lands must not throw.
  if (!CAT) return;
  renderList('in-list', 'in-filter', CAT.inputs, 'in', 'in-sub', 'readable');
  renderList('out-list', 'out-filter', CAT.outputs, 'par', 'out-sub', 'assignable');
}

src.addEventListener('input', paint);
src.addEventListener('scroll', function () {
  hl.scrollTop = src.scrollTop; hl.scrollLeft = src.scrollLeft;
});
src.addEventListener('focus', function () {
  document.getElementById('ed').classList.add('focus');
});
src.addEventListener('blur', function () {
  document.getElementById('ed').classList.remove('focus');
});
document.querySelector('.cols').addEventListener('click', function (ev) {
  var r = ev.target.closest('[data-ins]');
  if (!r) return;
  var name = r.getAttribute('data-ins'), at = src.selectionStart;
  src.value = src.value.slice(0, at) + name + src.value.slice(src.selectionEnd);
  src.focus();
  src.selectionStart = src.selectionEnd = at + name.length;
  paint();
});
document.getElementById('in-filter').addEventListener('input', renderAll);
document.getElementById('out-filter').addEventListener('input', renderAll);

document.getElementById('save-btn').addEventListener('click', async function () {
  setMsg('saving…', true);
  try {
    var r = await fetch('/logic', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({source: src.value})
    });
    var text = await r.text();
    setMsg(text, r.ok);
  } catch (e) { setMsg('save failed: ' + e, false); }
});

(async function () {
  try {
    var r = await fetch('/logic.json');
    var text = await r.text();
    var d;
    try {
      d = JSON.parse(text);
    } catch (parseError) {
      // Distinguish a truncated response from a server error: the former
      // used to surface later as a confusing null dereference.
      throw new Error('catalogue came back incomplete (' + text.length
        + ' bytes) - reload the page');
    }
    CAT = d.catalogue;
    CAT.inputs.forEach(function (g) {
      g[1].forEach(function (e) {
        NAMES[e[0]] = /^(gpio|pwm|adc)\d+$/.test(e[0]) ? 'pin' : 'in';
      });
    });
    CAT.outputs.forEach(function (g) {
      g[1].forEach(function (e) {
        OUTS[e[0]] = /^(gpio|pwm|adc)\d+$/.test(e[0]) ? 'pin' : 'par';
      });
    });
    src.value = d.source;
    paint();
    renderAll();
    if (d.errors && d.errors.length) setMsg(d.errors[0], false);
    else setMsg(d.rules + ' rule' + (d.rules === 1 ? '' : 's') + ' active', true);
  } catch (e) { setMsg('failed to load: ' + e, false); }
})();
</script>"""

_MISSION_MARKUP = (
    '<div class="fix-bar">'
    '<div><span class="label">Reference position</span>'
    '<span class="value" id="home-value">&mdash;</span></div>'
    '<span class="chip" id="home-chip"><span class="dot"></span>'
    '<span id="home-text">checking&hellip;</span></span>'
    '<button type="button" class="btn-quiet" onclick="useGps()">Use GPS position</button>'
    '<button type="button" class="btn-quiet" onclick="toggleManual()">Set manually</button>'
    '</div>'
    '<div class="home-fields" id="manual-home" hidden>'
    '<label style="margin:0">Latitude<input id="man-lat" inputmode="decimal"></label>'
    '<label style="margin:0">Longitude<input id="man-lon" inputmode="decimal"></label>'
    '<button type="button" class="btn-quiet" onclick="applyManual()">Use this</button>'
    '</div>'
    '<p>Tap the field to drop a waypoint where you want the plane to fly.'
    ' Drag a marker to move it. Everything is measured from the reference'
    ' position at the centre.</p>'
    '<canvas class="plot" id="plot" width="720" height="560"></canvas>'
    '<div class="readout" id="readout">Tap anywhere to place waypoint 1.</div>'
    '<div class="toolbar">'
    '<button type="button" class="btn-quiet" onclick="zoom(0.5)">Zoom in</button>'
    '<button type="button" class="btn-quiet" onclick="zoom(2)">Zoom out</button>'
    '<button type="button" class="btn-quiet" id="undo-btn" onclick="undo()">Undo</button>'
    '<button type="button" class="btn-quiet" id="clear-btn" onclick="clearAll()">Clear</button>'
    '<span class="chip" id="count-chip"><span class="dot"></span>'
    '<span id="count-text">0 of 20</span></span>'
    '<span class="chip" id="heading-chip"><span class="dot"></span>'
    '<span id="heading-text">heading &mdash;</span></span>'
    '</div>'
    '<h2>Waypoints</h2>'
    '<div class="toolbar">'
    '<select id="alt-frame" onchange="altFrameChanged()">'
    '<option value="rel">Altitudes are above the launch point</option>'
    '<option value="amsl">Altitudes are above sea level</option>'
    '</select>'
    '</div>'
    '<p class="sub" id="alt-frame-note"></p>'
    '<div class="wp-strip" id="strip"></div>'
    '<h2>After the last waypoint</h2>'
    '<div class="toolbar">'
    '<select id="end-action" onchange="endActionChanged()">'
    '<option value="loiter">Loiter over the last waypoint</option>'
    '<option value="rth">Return to home, then loiter</option>'
    '<option value="repeat">Fly back to waypoint 1 and repeat</option>'
    '<option value="land">Land at the launch point</option>'
    '</select>'
    '</div>'
    '<p class="sub" id="end-action-note"></p>'
    '</div>'
    '<div class="toolbar">'
    '<button type="button" id="save-btn" onclick="save()">Save mission</button>'
    '<span class="readout" id="msg" style="margin-top:0"></span>'
    '</div>'
    # Installed BEFORE the main script so it survives that script failing to
    # parse at all -- a response truncated mid-stream shows up as a syntax
    # error here rather than as a silently dead page. There is no console on
    # a phone, so errors have to land somewhere visible.
    '<script>'
    'window.__err=function(m){'
    'var el=document.getElementById("msg");'
    'if(el){el.textContent=m;el.style.color="var(--critical)";}'
    'var c=document.getElementById("diag");if(c){c.textContent=m;}};'
    'window.onerror=function(m,src,line){'
    'window.__err("Page script error: "+m+" (line "+line+")");return false;};'
    'window.addEventListener("unhandledrejection",function(e){'
    'window.__err("Page script error: "+((e.reason&&e.reason.message)||e.reason));});'
    '</script>'
)

# Raw string: the JS keeps its own \u escapes rather than letting Python
# decode them.
_MISSION_SCRIPT = r"""<script>
var M_PER_DEG = 111320;
var MAX_WP = 20;
var DEFAULT_ALT = 100;
// Must match mission._ALT_RANGE_BY_FRAME. The datum is picked once for the
// whole plan; "rel" means metres above the altitude captured when you arm,
// which is what you actually mean standing in a field, and is what both
// INAV and ArduPilot default their waypoints to.
var ALT_RANGE = {rel: [0, 500], amsl: [-100, 5000]};
function altFrame() {
  var el = document.getElementById('alt-frame');
  return el ? el.value : 'rel';
}
function altRange() { return ALT_RANGE[altFrame()] || ALT_RANGE.rel; }
var state = {wps: [], span: 400, sel: -1, drag: -1, home: null, source: 'none'};
// Which way the aircraft is pointing, for the centre chevron. Compass when
// the QMC5883 answers (valid standing still), otherwise GPS course over
// ground -- which is only meaningful once actually moving.
var heading = {deg: null, source: 'none'};
var HEADING_POLL_MS = 2000;

var canvas = document.getElementById('plot');
var ctx = canvas.getContext('2d');
var strip = document.getElementById('strip');

// Mid-grey fallbacks, deliberately visible against BOTH the light and dark
// surfaces: if a custom property fails to resolve, assigning '' to
// ctx.strokeStyle is silently IGNORED by canvas and the context keeps its
// default black -- which on the dark theme's near-black surface renders the
// whole plot invisible and looks exactly like "the canvas stopped working".
var COLOR_FALLBACK = {
  '--gridline': '#888880', '--accent': '#3987e5', '--good': '#0ca30c',
  '--muted': '#898781', '--text-primary': '#888880', '--text-secondary': '#898781'
};
// The range rings are NOT drawn with --gridline. That token is sized for
// hairline CSS borders against the page, and on canvas it measures 1.24:1
// against the dark theme's surface and 1.29:1 against the light one -- below
// the ~1.5:1 where a thin line stops being visible on a phone at all, which
// reads as "the plot is just a black rectangle". Mid-grey at partial alpha
// composites over either surface at usable contrast while staying
// subordinate to the route itself.
var GRID_STROKE = 'rgba(137,135,129,0.55)';
function cssVar(n) {
  var v = '';
  try {
    v = getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  } catch (e) { /* fall through to the fallback */ }
  return v || COLOR_FALLBACK[n] || '#888880';
}
function mpp() { return state.span / canvas.width; }
function toPx(w) {
  return {x: canvas.width / 2 + w.e / mpp(), y: canvas.height / 2 - w.n / mpp()};
}
function toM(x, y) {
  return {e: (x - canvas.width / 2) * mpp(), n: (canvas.height / 2 - y) * mpp()};
}
function toLatLon(w) {
  var h = state.home;
  return {
    lat: h.lat + w.n / M_PER_DEG,
    lon: h.lon + w.e / (M_PER_DEG * Math.cos(h.lat * Math.PI / 180)),
    alt_m: w.alt
  };
}
function fromLatLon(w) {
  var h = state.home;
  return {
    n: (w.lat - h.lat) * M_PER_DEG,
    e: (w.lon - h.lon) * M_PER_DEG * Math.cos(h.lat * Math.PI / 180),
    alt: w.alt_m
  };
}
function eventPos(ev) {
  var r = canvas.getBoundingClientRect();
  return {
    x: (ev.clientX - r.left) * (canvas.width / r.width),
    y: (ev.clientY - r.top) * (canvas.height / r.height)
  };
}
function hitRadius() {
  // The canvas is 720 px internally but renders ~350 px wide on a phone,
  // so a fixed radius in canvas units shrinks to about 11 CSS px on screen
  // -- far too small for a fingertip, and a miss ADDS a waypoint instead of
  // selecting one. Scale the target to a ~24 CSS px radius on any display.
  var box = canvas.getBoundingClientRect();
  var scale = box.width ? canvas.width / box.width : 1;
  return Math.max(22, 24 * scale);
}
function hit(p) {
  var limit = hitRadius();
  var best = -1;
  var bestDistance = limit;
  // Nearest wins, not last-drawn: with a touch-sized radius several markers
  // can be inside it at once, and the closest is the one meant.
  for (var i = 0; i < state.wps.length; i++) {
    var q = toPx(state.wps[i]);
    var d = Math.hypot(q.x - p.x, q.y - p.y);
    if (d <= bestDistance) {
      bestDistance = d;
      best = i;
    }
  }
  return best;
}

function draw() {
  var w = canvas.width, h = canvas.height, i;
  ctx.clearRect(0, 0, w, h);
  var step = state.span <= 200 ? 25 : state.span <= 400 ? 50 : 100;
  ctx.strokeStyle = GRID_STROKE;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (var d = step; d < state.span; d += step) {
    var r = d / mpp();
    ctx.moveTo(w / 2 + r, h / 2);
    ctx.arc(w / 2, h / 2, r, 0, 2 * Math.PI);
  }
  ctx.moveTo(w / 2, 0); ctx.lineTo(w / 2, h);
  ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2);
  ctx.stroke();

  ctx.fillStyle = cssVar('--muted');
  ctx.font = '11px system-ui';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'bottom';
  ctx.fillText('N', w / 2 + 5, 16);
  for (var g = step; g < state.span; g += step) {
    ctx.fillText(g + ' m', w / 2 + g / mpp() + 4, h / 2 - 3);
  }

  if (state.wps.length > 1) {
    ctx.strokeStyle = cssVar('--accent');
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (i = 0; i < state.wps.length; i++) {
      var q = toPx(state.wps[i]);
      i ? ctx.lineTo(q.x, q.y) : ctx.moveTo(q.x, q.y);
    }
    ctx.stroke();
  }
  // Outside the polyline guard above: a one-waypoint plan draws no legs but
  // still has an ending, and "return home" from that single point is a real
  // leg the pilot should see.
  drawEndActionLeg();

  drawHeadingMarker(w / 2, h / 2);

  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (i = 0; i < state.wps.length; i++) {
    var p = toPx(state.wps[i]);
    if (i === state.sel) {
      ctx.strokeStyle = cssVar('--accent');
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 17, 0, 2 * Math.PI);
      ctx.stroke();
    }
    ctx.fillStyle = i === 0 ? cssVar('--good') : cssVar('--accent');
    ctx.beginPath();
    ctx.arc(p.x, p.y, 12, 0, 2 * Math.PI);
    ctx.fill();
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 12px system-ui';
    ctx.fillText(String(i + 1), p.x, p.y);
  }
}

// The continuation after the final waypoint, dashed so it reads as an
// ending rather than another leg: repeat closes the loop back to waypoint
// 1, rth heads for the reference position at the centre.
function drawEndActionLeg() {
  var action = document.getElementById('end-action').value;
  if (action !== 'repeat' && action !== 'rth' && action !== 'land') return;
  // Repeat closes a loop, so it needs two points to close between; rth and
  // land only need somewhere to leave from. None can draw from an empty plan.
  if (!state.wps.length || (action === 'repeat' && state.wps.length < 2)) return;
  var from = toPx(state.wps[state.wps.length - 1]);
  var to = action === 'repeat' ? toPx(state.wps[0])
                               : {x: canvas.width / 2, y: canvas.height / 2};
  ctx.strokeStyle = cssVar('--accent');
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 6]);
  ctx.beginPath();
  ctx.moveTo(from.x, from.y);
  ctx.lineTo(to.x, to.y);
  ctx.stroke();
  ctx.setLineDash([]);
}

// Switching the datum REINTERPRETS the numbers, it does not convert them:
// the editor knows the reference latitude/longitude but never its altitude,
// so it cannot turn 100 m over the field into an MSL figure. Say so, because
// silently leaving "100" in place while changing what it means is exactly
// the ambiguity this selector exists to remove.
function altFrameChanged() {
  var rel = altFrame() === 'rel';
  document.getElementById('alt-frame-note').textContent = rel
    ? 'Each waypoint’s height above wherever the plane is armed.'
    : 'Absolute altitudes. Changing this does not convert the numbers below'
      + ' — check every waypoint.';
  // Re-clamp into the new range before re-rendering, so a value the new
  // datum cannot hold is corrected here rather than rejected on save.
  var r = altRange();
  for (var i = 0; i < state.wps.length; i++) {
    state.wps[i].alt = Math.max(r[0], Math.min(r[1], state.wps[i].alt));
  }
  refresh();
}

function endActionChanged() {
  var action = document.getElementById('end-action').value;
  if (action === 'repeat' && state.wps.length < 2) {
    setMsg('Repeat needs at least 2 waypoints.', false);
  } else {
    setMsg('', true);
  }
  // The approach direction is a config-page parameter, not part of the
  // plan, and it is the one setting that has to change with the day's
  // wind -- so say so here, where landing is actually being chosen.
  document.getElementById('end-action-note').textContent = action === 'land'
    ? 'Flies an approach into the launch point and glides in with the motor'
      + ' off. Set “nav_land_heading_deg” on the Config page to the'
      + ' direction you want it landing — into wind.'
    : '';
  draw();
}

function drawHeadingMarker(cx, cy) {
  // North is up on this plot, so a heading of t points along the screen
  // vector (sin t, -cos t). Chevron drawn in a local (right, forward) frame
  // and rotated onto that: forward = (sin t, -cos t), right = (cos t, sin t).
  if (heading.deg === null) {
    // No heading source yet -- a plain dot, so the marker never implies a
    // direction it doesn't actually know.
    ctx.fillStyle = cssVar('--text-secondary');
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, 2 * Math.PI);
    ctx.fill();
    return;
  }
  var t = heading.deg * Math.PI / 180;
  var cosT = Math.cos(t);
  var sinT = Math.sin(t);
  var shape = [[0, 17], [-11, -11], [0, -5], [11, -11]];
  ctx.beginPath();
  for (var i = 0; i < shape.length; i++) {
    var px = shape[i][0];
    var py = shape[i][1];
    var sx = cx + px * cosT + py * sinT;
    var sy = cy + px * sinT - py * cosT;
    i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy);
  }
  ctx.closePath();
  ctx.fillStyle = cssVar('--text-primary');
  ctx.fill();
}

function totalDistance() {
  var d = 0;
  for (var i = 1; i < state.wps.length; i++) {
    d += Math.hypot(state.wps[i].e - state.wps[i - 1].e,
                    state.wps[i].n - state.wps[i - 1].n);
  }
  return d;
}

function renderStrip() {
  var ready = state.home !== null;
  if (state.wps.length === 0) {
    strip.innerHTML = '<div class="empty-hint">No waypoints yet.</div>';
  } else {
    var html = '';
    for (var i = 0; i < state.wps.length; i++) {
      var wp = state.wps[i];
      var coords = ready
        ? (function (ll) { return ll.lat.toFixed(5) + ', ' + ll.lon.toFixed(5); })(toLatLon(wp))
        : Math.round(wp.n) + ' m N, ' + Math.round(wp.e) + ' m E';
      // Listeners are delegated (see below) rather than inline: an inline
      // handler resolves names against the element first, so onclick="remove(i)"
      // on a <button> silently calls Element.prototype.remove() -- it deletes
      // the button from the page and leaves the waypoint in the list.
      html += '<div class="wp-item' + (i === state.sel ? ' sel' : '')
        + (i === 0 ? ' first' : '') + '" data-i="' + i + '">'
        + '<span class="idx">' + (i + 1) + '</span>'
        + '<span class="coords">' + coords + '</span>'
        + '<span class="alt"><input type="number" value="' + wp.alt
        + '" min="' + altRange()[0] + '" max="' + altRange()[1]
        + '" data-alt="' + i + '"> m</span>'
        + '<button type="button" class="btn-quiet" data-remove="' + i
        + '" aria-label="Delete waypoint ' + (i + 1) + '">&times;</button>'
        + '</div>';
    }
    strip.innerHTML = html;
  }
  var n = state.wps.length;
  var chip = document.getElementById('count-chip');
  chip.className = 'chip ' + (n === 0 ? '' : n >= MAX_WP ? 'warning' : 'good');
  document.getElementById('count-text').textContent = n + ' of ' + MAX_WP
    + (n > 1 ? '  \u00b7  ' + Math.round(totalDistance()) + ' m track' : '');
  document.getElementById('undo-btn').disabled = n === 0;
  document.getElementById('clear-btn').disabled = n === 0;
  document.getElementById('save-btn').disabled = n === 0 || !ready;
}

function refresh() { draw(); renderStrip(); }

// Selection only changes which row is outlined, so it retags the existing
// rows instead of going through renderStrip(). That matters more than the
// saved work: renderStrip() reassigns strip.innerHTML, which destroys and
// rebuilds every row -- including an altitude field the user is currently
// typing into. Doing that on tap dropped the on-screen keyboard and made
// the page jump as focus was lost.
function applySelection() {
  var rows = strip.querySelectorAll('[data-i]');
  for (var r = 0; r < rows.length; r++) {
    var idx = parseInt(rows[r].getAttribute('data-i'), 10);
    rows[r].className = 'wp-item' + (idx === state.sel ? ' sel' : '')
      + (idx === 0 ? ' first' : '');
  }
}
function selectWaypoint(i) {
  state.sel = i;
  applySelection();
  draw();
}
function setAlt(i, v, field) {
  var r = altRange();
  var alt = Math.max(r[0], Math.min(r[1], parseFloat(v) || 0));
  state.wps[i].alt = alt;
  // Reflect clamping in place rather than re-rendering the strip, for the
  // same reason as above -- altitude doesn't affect the plot or the row
  // layout, so nothing else needs redrawing.
  if (field && parseFloat(field.value) !== alt) field.value = alt;
}
function removeWaypoint(i) {
  state.wps.splice(i, 1);
  if (state.sel >= state.wps.length) state.sel = state.wps.length - 1;
  refresh();
}

strip.addEventListener('click', function (ev) {
  var del = ev.target.closest('[data-remove]');
  if (del) {
    removeWaypoint(parseInt(del.getAttribute('data-remove'), 10));
    return;
  }
  // Taps that land in the altitude field belong to the field, not the row.
  if (ev.target.closest('input')) return;
  var row = ev.target.closest('[data-i]');
  if (row) selectWaypoint(parseInt(row.getAttribute('data-i'), 10));
});
strip.addEventListener('change', function (ev) {
  var field = ev.target.closest('[data-alt]');
  if (field) {
    setAlt(parseInt(field.getAttribute('data-alt'), 10), field.value, field);
  }
});
function undo() { state.wps.pop(); state.sel = state.wps.length - 1; refresh(); }
function clearAll() { state.wps = []; state.sel = -1; refresh(); }
function zoom(f) {
  state.span = Math.max(100, Math.min(4000, state.span * f));
  refresh();
}
function fitView() {
  var far = 0;
  for (var i = 0; i < state.wps.length; i++) {
    far = Math.max(far, Math.abs(state.wps[i].n), Math.abs(state.wps[i].e));
  }
  if (far > 0) state.span = Math.max(100, Math.min(4000, far * 2.6));
}

canvas.addEventListener('pointerdown', function (ev) {
  var p = eventPos(ev);
  var i = hit(p);
  if (i >= 0) {
    state.drag = i;
    state.sel = i;
    canvas.setPointerCapture(ev.pointerId);
  } else {
    if (state.wps.length >= MAX_WP) {
      setMsg('Twenty waypoints is the limit the flight controller stores.', false);
      return;
    }
    var m = toM(p.x, p.y);
    var alt = state.wps.length ? state.wps[state.wps.length - 1].alt : DEFAULT_ALT;
    state.wps.push({n: m.n, e: m.e, alt: alt});
    state.sel = state.wps.length - 1;
    setMsg('', true);
  }
  refresh();
});

canvas.addEventListener('pointermove', function (ev) {
  var p = eventPos(ev);
  if (state.drag >= 0) {
    var d = toM(p.x, p.y);
    state.wps[state.drag].n = d.n;
    state.wps[state.drag].e = d.e;
    refresh();
  }
  var m = toM(p.x, p.y);
  var bearing = (Math.atan2(m.e, m.n) * 180 / Math.PI + 360) % 360;
  document.getElementById('readout').textContent =
    Math.round(Math.hypot(m.n, m.e)) + ' m out, bearing ' + Math.round(bearing) + '\u00b0';
});

canvas.addEventListener('pointerup', function () { state.drag = -1; });
canvas.addEventListener('pointerleave', function () {
  document.getElementById('readout').textContent = state.wps.length
    ? 'Tap to add, drag a marker to move it.'
    : 'Tap anywhere to place waypoint 1.';
});

function setMsg(text, ok) {
  var el = document.getElementById('msg');
  el.textContent = text;
  el.style.color = ok ? 'var(--good)' : 'var(--critical)';
}

function setHome(lat, lon, source, label, level) {
  state.home = {lat: lat, lon: lon};
  state.source = source;
  document.getElementById('home-value').textContent =
    lat.toFixed(5) + ', ' + lon.toFixed(5);
  document.getElementById('home-chip').className = 'chip ' + level;
  document.getElementById('home-text').textContent = label;
  refresh();
}

function toggleManual() {
  var box = document.getElementById('manual-home');
  box.hidden = !box.hidden;
  if (!box.hidden && state.home) {
    document.getElementById('man-lat').value = state.home.lat.toFixed(5);
    document.getElementById('man-lon').value = state.home.lon.toFixed(5);
  }
}

function applyManual() {
  var lat = parseFloat(document.getElementById('man-lat').value);
  var lon = parseFloat(document.getElementById('man-lon').value);
  if (!isFinite(lat) || lat < -90 || lat > 90) {
    setMsg('Latitude must be a number between -90 and 90.', false);
    return;
  }
  if (!isFinite(lon) || lon < -180 || lon > 180) {
    setMsg('Longitude must be a number between -180 and 180.', false);
    return;
  }
  setHome(lat, lon, 'manual', 'set manually', 'warning');
  document.getElementById('manual-home').hidden = true;
  setMsg('', true);
}

function applyHeading(d) {
  // Compass first: it reads correctly standing still, which is exactly when
  // you're planning. GPS course only means anything once moving.
  if (d.compass && d.compass.detected) {
    heading.deg = d.compass.field_angle_deg;
    heading.source = 'compass';
  } else if (d.gps && d.gps.fix_ok
             && d.gps.ground_speed_ms > 1.0) {
    heading.deg = d.gps.course_deg;
    heading.source = 'GPS course';
  } else {
    heading.deg = null;
    heading.source = 'none';
  }
  var chip = document.getElementById('heading-chip');
  chip.className = 'chip ' + (heading.deg === null ? '' : 'good');
  document.getElementById('heading-text').textContent =
    heading.deg === null
      ? 'heading \u2014 no compass'
      : Math.round(heading.deg) + '\u00b0 (' + heading.source + ')';
}

// Deliberately slow, and paused when the page isn't visible: this keeps the
// chevron honest without returning to the fast polling that made the server
// unusable before. One request every 2 s is a fraction of what the removed
// debug pages did at 200-300 ms.
async function pollHeading() {
  if (document.visibilityState === 'visible') {
    try {
      var r = await fetch('/status.json');
      applyHeading(await r.json());
      draw();
    } catch (e) { /* keep the last known heading */ }
  }
  setTimeout(pollHeading, HEADING_POLL_MS);
}

// Home is fetched on demand only -- never on the poll, since silently
// moving the reference would drag the whole plan with it.
async function useGps() {
  try {
    var r = await fetch('/status.json');
    var d = await r.json();
    applyHeading(d);
    if (d.gps && d.gps.fix_ok) {
      setHome(d.gps.lat_deg, d.gps.lon_deg,
              'gps', d.gps.num_sv + ' sats, 3D fix', 'good');
      setMsg('', true);
    } else {
      var sats = d.gps ? d.gps.num_sv : 0;
      document.getElementById('home-chip').className = 'chip critical';
      document.getElementById('home-text').textContent =
        d.gps && d.gps.pvt_age_ms === null
          ? 'no data from module' : sats + ' sats, acquiring';
      if (!state.home) {
        setMsg('No GPS fix yet. Wait for one, or set the reference position '
               + 'manually to plan now.', false);
      }
    }
  } catch (e) {
    document.getElementById('home-chip').className = 'chip critical';
    document.getElementById('home-text').textContent = 'status unavailable';
  }
}

async function loadMission() {
  var stored = {waypoints: []};
  try {
    var r = await fetch('/mission.json');
    stored = await r.json();
  } catch (e) {
    setMsg('Could not load the stored mission.', false);
    return;
  }
  await useGps();
  // Without a fix, fall back to the stored plan's own first waypoint so an
  // existing mission stays viewable and editable indoors.
  if (!state.home && stored.waypoints.length) {
    setHome(stored.waypoints[0].lat, stored.waypoints[0].lon,
            'mission', 'from stored mission', 'warning');
  }
  if (state.home) {
    state.wps = stored.waypoints.map(fromLatLon);
    fitView();
  }
  if (stored.end_action) {
    document.getElementById('end-action').value = stored.end_action;
  }
  // Set the datum BEFORE refresh() so the altitude inputs render with the
  // right bounds first time.
  if (stored.alt_frame) {
    document.getElementById('alt-frame').value = stored.alt_frame;
  }
  altFrameChanged();
  // Both notes are rendered by their change handlers, so they have to be
  // run once on load or a stored plan shows its selector without the
  // explanation that goes with it.
  endActionChanged();
  if (stored.waypoints.length) {
    setMsg(stored.waypoints.length + ' waypoints loaded.', true);
  }
  refresh();
  setTimeout(pollHeading, HEADING_POLL_MS);
}

async function save() {
  var endAction = document.getElementById('end-action').value;
  if (endAction === 'repeat' && state.wps.length < 2) {
    setMsg('Repeat needs at least 2 waypoints.', false);
    return;
  }
  var mission = {
    waypoints: state.wps.map(toLatLon),
    end_action: endAction,
    alt_frame: altFrame()
  };
  try {
    var r = await fetch('/mission', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(mission)
    });
    var text = await r.text();
    setMsg(r.ok ? ('Saved. ' + mission.waypoints.length
      + ' waypoints written to the flight controller.') : text, r.ok);
  } catch (e) {
    setMsg('Save failed: ' + e, false);
  }
}

// Reaching this line proves the whole script parsed and ran -- if the page
// looks dead and this never clears, the response was truncated.
setMsg('', true);
loadMission();
</script>"""


# These bodies are fully static (placeholders filled in by client-side JS
# after a fetch, never by server-side per-request data) -- computed once
# here, at import time, before any runtime fragmentation exists, and reused
# (zero-copy reference, no re-allocation) on every request thereafter.
# The mission page ships as two constants rather than one ~9 KB string so
# neither the import-time build nor the response needs a single large
# contiguous block. _config_body_chunks is the sole per-request exception.
_STATUS_BODY = _status_body()
_MISSION_CHUNKS = (_MISSION_MARKUP, _MISSION_SCRIPT)
_LOGIC_CHUNKS = (_LOGIC_MARKUP, _LOGIC_SCRIPT)
# Built on FIRST REQUEST, not at import. _logic_catalogue() ends in a
# json.dumps() of the whole input/output table, which needs one contiguous
# block of about 5 KB -- and asking for that while server.py is still being
# imported is a boot-time allocation, made before main() has claimed the
# UART ring buffers and with the heap at its most fragmented. It failed:
#
#   File "server.py", line 573, in _logic_catalogue
#   MemoryError: memory allocation failed, allocating 5216 bytes
#
# and took the whole firmware down before the portal ever came up, which
# presents as a dead board rather than as a broken page. Nothing needs this
# until someone opens the Logic page, by which time the heap has settled and
# the flight loop is not running. Cached after the first build, so the cost
# is paid once per boot at worst -- and never at all on a board whose owner
# never opens that page.
_LOGIC_CATALOGUE = None


def _logic_catalogue_json():
    global _LOGIC_CATALOGUE

    if _LOGIC_CATALOGUE is None:
        _LOGIC_CATALOGUE = _logic_catalogue(pins.FREE_PINS)
    return _LOGIC_CATALOGUE


def _json_string(text):
    # Minimal JSON string encoder: the logic source is user text with
    # newlines and quotes in it, and it is assembled into a response by hand
    # to avoid building the whole (large) catalogue dict per request.
    out = ['"']
    for character in text:
        if character == '"':
            out.append('\\"')
        elif character == "\\":
            out.append("\\\\")
        elif character == "\n":
            out.append("\\n")
        elif character == "\r":
            out.append("\\r")
        elif character == "\t":
            out.append("\\t")
        elif character < " ":
            out.append("\\u%04x" % ord(character))
        else:
            out.append(character)
    out.append('"')
    return "".join(out)


def _json_string_list(items):
    return "[" + ",".join(_json_string(str(item)) for item in items) + "]"


def _json_response(payload):
    # microdot serializes a returned dict with json.dumps and writes it in a
    # single call. That is fine for a few hundred bytes and NOT fine past a
    # couple of KB -- a large write arrives truncated, and truncated JSON
    # fails to parse on the client in ways that surface far from the cause.
    # status.json reaches ~2.5 KB with twenty rules and mission.json ~1 KB
    # with twenty waypoints, so every JSON endpoint goes out chunked.
    import json

    return Response(body=_chunked(json.dumps(payload)),
                    headers={"Content-Type": "application/json"})

# Shown in every page's subtitle. Derived from the page content itself, so it
# changes whenever the markup, styles or scripts do without anyone having to
# remember to bump it -- the point is to answer "is the browser showing the
# version currently on the board, or a cached one?" at a glance.
_BUILD = str((len(_STYLE) + len(_STATUS_BODY) + len(_MISSION_MARKUP)
              + len(_MISSION_SCRIPT)) % 100000)


_imu_detected_cache = [None, 0]  # [result, last check ticks_ms]
_IMU_DETECT_CACHE_MS = 3000


def _imu_detected():
    # A full i2c.scan() probes 112 addresses and blocks the event loop for
    # tens of ms; the status page polls every 400 ms, so cache the answer
    # and re-scan at most every few seconds -- plug/unplug detection doesn't
    # need to be instant.
    now_ms = time.ticks_ms()
    cached, checked_ms = _imu_detected_cache
    if cached is not None and time.ticks_diff(now_ms, checked_ms) < _IMU_DETECT_CACHE_MS:
        return cached
    try:
        import pins
        from machine import I2C, Pin

        i2c = I2C(pins.IMU_I2C_ID, sda=Pin(pins.IMU_SDA_PIN), scl=Pin(pins.IMU_SCL_PIN))
        result = _MPU6050_ADDRESS in i2c.scan()
    except OSError:
        result = False
    _imu_detected_cache[0] = result
    _imu_detected_cache[1] = now_ms
    return result


_AP_START_TIMEOUT_MS = 5000
_AP_POLL_MS = 50


def _activate_access_point(ap, ssid, password, feed=None):
    ap.active(False)
    ap.config(ssid=ssid, password=password)
    ap.active(True)
    # Bounded wait instead of `while not ap.active(): pass` -- if the AP
    # never comes up, an untimed spin is an unrecoverable silent hang.
    #
    # feed: the caller's watchdog, because this is the one blocking stretch
    # in the firmware that can outlast the watchdog timeout (5 s bound
    # against a 4 s window). It is a plain callback rather than a WDT
    # instance so this module keeps knowing nothing about machine.WDT.
    for _ in range(_AP_START_TIMEOUT_MS // _AP_POLL_MS):
        if feed is not None:
            feed()
        if ap.active():
            return
        time.sleep_ms(_AP_POLL_MS)
    raise OSError("AP failed to start")


def start_access_point(config, feed=None):
    ap = network.WLAN(network.AP_IF)
    ssid = "pico-wing-" + config["wifi_ssid_suffix"]
    try:
        _activate_access_point(ap, ssid, config["wifi_password"], feed)
    except (OSError, ValueError, RuntimeError) as error:
        # Persisted credentials the driver rejects (or an AP that never came
        # up) must not brick the portal -- config.validate() screens new
        # saves, but an old/hand-edited config.json can still hold anything.
        # Retry once with the schema defaults, which are known-good.
        ssid = "pico-wing-" + config_module.SCHEMA["wifi_ssid_suffix"][0]
        print("AP start failed (", error, "); retrying with default credentials")
        try:
            _activate_access_point(
                ap, ssid, config_module.SCHEMA["wifi_password"][0], feed
            )
        except (OSError, ValueError, RuntimeError) as retry_error:
            # Even the defaults failed: the CYW43 radio itself is wedged. A
            # full reboot reinitializes it via the cold-boot path (the one
            # that always works); crashing here would leave a dead board
            # with no AP and no recovery short of a power cycle.
            print("AP start failed twice (", retry_error, "); rebooting")
            import machine

            machine.reset()
    # WiFi power save adds multi-second latency to every request; disable it
    # while the portal is up (WiFi is torn down entirely when armed anyway).
    ap.config(pm=network.WLAN.PM_NONE)
    ap.ifconfig((_AP_IP, "255.255.255.0", _AP_IP, _AP_IP))
    print("config portal: http://" + _AP_IP + " (AP " + ssid + ")")
    return ap


def stop_access_point(ap):
    ap.active(False)


def create_app(current_config, save_callback, rc, gps, logic_engine, free_pins):
    app = Microdot()
    Response.default_content_type = "text/html"

    # Lazy compass probe, kept across polls; a mid-session bus error drops
    # the instance so the next poll re-probes (replug recovers on its own).
    compass_sensor = None

    def read_compass_angle():
        nonlocal compass_sensor
        if compass_sensor is None:
            try:
                compass_sensor = QMC5883()
            except OSError:
                return None
        try:
            x, y, _ = compass_sensor.read()
        except OSError:
            compass_sensor = None
            return None
        return flat_field_angle_deg(x, y)

    @app.after_request
    async def no_caching(request, response):
        # The portal is reflashed constantly during development and every
        # page is live device state, so a cached copy is always wrong -- and
        # worse, it makes a fix look like it did not work because the browser
        # is still running the old script. Nothing here is worth caching.
        response.headers["Cache-Control"] = "no-store, must-revalidate"
        return response

    @app.before_request
    async def collect_garbage_before_each_request(request):
        # RP2040 has no PSRAM; building a ~10-15 KB page from many small
        # concatenated fragments (config's slider groups especially) can hit
        # a MemoryError under real running conditions (WiFi + asyncio + the
        # rest of main.py all sharing the same small heap) even when total
        # free memory looks sufficient -- fragmentation, not just size. A
        # collect right before each request's likely-to-allocate-heavily
        # handler runs is the standard MicroPython mitigation.
        gc.collect()

    @app.get("/")
    async def status_page(request):
        ssid = "pico-wing-" + current_config()["wifi_ssid_suffix"]
        return _page_chunks("Status", [_STATUS_BODY], "status", subtitle=ssid)

    @app.get("/status.json")
    async def status_json(request):
        now_ms = time.ticks_ms()
        angle = read_compass_angle()
        return _json_response({
            "link_alive": rc.link_alive,
            "age_ms": time.ticks_diff(now_ms, rc.last_frame_ms),
            "mode": rc.mode,
            "arm": rc.arm_switch_on,
            "channels": rc.channels,
            "imu_detected": _imu_detected(),
            "gps": {
                "fix_ok": gps.fix_ok,
                "fix_type": gps.fix_type,
                "num_sv": gps.num_sv,
                "lat_deg": gps.lat_deg,
                "lon_deg": gps.lon_deg,
                "alt_m": gps.alt_m,
                "ground_speed_ms": gps.ground_speed_ms,
                "course_deg": gps.course_deg,
                "h_acc_m": gps.h_acc_m,
                "pvt_age_ms": (
                    None if gps.last_pvt_ms is None
                    else time.ticks_diff(now_ms, gps.last_pvt_ms)
                ),
            },
            "compass": {
                "detected": angle is not None,
                "field_angle_deg": angle if angle is not None else 0.0,
            },
            "logic": {
                "rules": logic_engine.summary(),
                "errors": logic_engine.errors,
            },
        })

    @app.get("/config")
    async def config_page(request):
        return _page_chunks(
            "Config", _config_body_chunks(current_config()), "config"
        )

    @app.post("/config")
    async def save_config(request):
        # Merge onto the live config: a partial/malformed POST must leave
        # untouched parameters at their current values, not silently
        # factory-reset them (validate's default base is the schema defaults).
        updated = config_module.validate(request.form, current_config())
        save_callback(updated)
        return _page_chunks(
            "Config", _config_body_chunks(current_config(), "Saved."), "config"
        )

    @app.get("/logic")
    async def logic_page(request):
        return _page_chunks("Logic", _LOGIC_CHUNKS, "logic")

    @app.get("/logic.json")
    async def logic_json(request):
        # Streamed, not built as one string. The catalogue alone is ~5 KB and
        # the whole response went out in a single socket write, which is
        # exactly the size that arrives truncated -- the browser then failed
        # to parse it, left the page's catalogue null, and any keystroke in a
        # filter box raised "null is not an object". Same 1 KB write ceiling
        # as the pages.
        def body():
            yield '{"source":'
            for piece in _chunked(_json_string(logic_engine.source)):
                yield piece
            yield ',"rules":%d,"errors":' % len(logic_engine.rules)
            yield _json_string_list(logic_engine.errors)
            yield ',"catalogue":'
            for piece in _chunked(_logic_catalogue_json()):
                yield piece
            yield "}"

        return Response(body=body(),
                        headers={"Content-Type": "application/json"})

    @app.post("/logic")
    async def save_logic(request):
        # Accepted only if every rule compiles AND survives a timed trial run
        # against representative inputs -- a rule that raises or runs long is
        # caught here, on the ground, not in the air.
        try:
            body = request.json
            source = body["source"]
        except (ValueError, KeyError, TypeError):
            return "request body must be JSON with a 'source' string", 400
        if not isinstance(source, str):
            return "'source' must be a string", 400
        if not source.strip():
            logic_engine.clear()
            logic_module.save("")
            return "Saved. No rules — logic is off."
        rules, errors = logic_engine.load_source(source)
        if errors:
            return str(errors[0]), 400
        problems = logic_module.smoke_test(
            rules, logic_module.sample_namespace(free_pins)
        )
        if problems:
            return str(problems[0]), 400
        logic_module.save(source)
        logic_engine.activate(source, rules)
        return "Saved. %d rule%s active." % (
            len(rules), "" if len(rules) == 1 else "s")

    @app.get("/mission")
    async def mission_page(request):
        return _page_chunks("Mission", _MISSION_CHUNKS, "mission")

    @app.get("/mission.json")
    async def mission_json(request):
        return _json_response(mission_module.load())

    @app.post("/mission")
    async def save_mission(request):
        # Strict all-or-nothing: a rejected waypoint rejects the whole save
        # (see mission.py) and the reason goes back to the editor verbatim.
        try:
            body = request.json
        except ValueError:
            return "request body is not valid JSON", 400
        try:
            mission_module.save(body)
        except ValueError as error:
            return str(error), 400
        return "ok"

    return app
