#!/usr/bin/env python3
"""Headless browser viewer and measurement tool for ROS occupancy-grid maps."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import gzip
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import struct
from typing import Any
from urllib.parse import urlparse
import zlib

import yaml


DEFAULT_MAP = (
    "/home/hsb/hsb_analysis/lidar_validation/slam_toolbox/"
    "corrected_icp_odom/oct1_corrected_icp_odom.yaml"
)
DEFAULT_SCANS = (
    "/home/hsb/hsb_analysis/lidar_validation/web_viewer/oct1_scans.json"
)
DEFAULT_ROOM_MEASUREMENTS = str(
    Path(__file__).resolve().parents[1]
    / "src"
    / "horseshitbot"
    / "config"
    / "room_measurements_oct1.yaml"
)


def load_room_measurements(path: Path) -> dict[str, Any]:
    """Load and validate the physical room measurement YAML file."""
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Room measurements YAML does not exist: {path}")
    payload = yaml.safe_load(path.read_text()) or {}
    for required in ("room_metadata", "discrepancies", "walls", "widths", "measurements"):
        if required not in payload:
            raise ValueError(f"Room measurements YAML is missing: {required}")
    return payload


def rigid_transform_point(
    x: float,
    y: float,
    tx: float,
    ty: float,
    yaw_rad: float,
) -> tuple[float, float]:
    """Apply a 2D rigid transform (rotation + translation only, unit scale)."""
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return (
        tx + cosine * x - sine * y,
        ty + sine * x + cosine * y,
    )


def comparison_statistics(rows: list[dict[str, Any]], key: str) -> dict[str, float | int | None]:
    """Compute signed/absolute/percentage errors plus MAE and RMSE for identified rows."""
    abs_errors: list[float] = []
    sq_errors: list[float] = []
    for row in rows:
        real_val = row.get("real_distance_m")
        est_val = row.get(key)
        if real_val is None or est_val is None:
            continue
        diff = float(est_val) - float(real_val)
        abs_errors.append(abs(diff))
        sq_errors.append(diff * diff)
    if not abs_errors:
        return {"count": 0, "mae_m": None, "rmse_m": None}
    return {
        "count": len(abs_errors),
        "mae_m": sum(abs_errors) / len(abs_errors),
        "rmse_m": math.sqrt(sum(sq_errors) / len(sq_errors)),
    }


def _read_token(handle: Any) -> bytes:
    token = bytearray()
    while True:
        byte = handle.read(1)
        if not byte:
            raise ValueError("Unexpected end of PGM header")
        if byte == b"#":
            handle.readline()
            continue
        if not byte.isspace():
            token.extend(byte)
            break
    while True:
        byte = handle.read(1)
        if not byte or byte.isspace():
            return bytes(token)
        token.extend(byte)


def read_pgm(path: Path) -> tuple[int, int, int, bytes]:
    """Read P5 or P2 PGM data and normalize samples to unsigned 8-bit."""
    with path.open("rb") as handle:
        magic = _read_token(handle)
        width = int(_read_token(handle))
        height = int(_read_token(handle))
        maximum = int(_read_token(handle))
        if width <= 0 or height <= 0 or not 0 < maximum <= 65535:
            raise ValueError(f"Invalid PGM geometry in {path}")
        sample_count = width * height
        if magic == b"P5":
            bytes_per_sample = 1 if maximum < 256 else 2
            payload = handle.read(sample_count * bytes_per_sample)
            if len(payload) != sample_count * bytes_per_sample:
                raise ValueError(f"Truncated PGM pixel data in {path}")
            if bytes_per_sample == 1:
                values = payload
            else:
                values = bytes(
                    round(value * 255 / maximum)
                    for value in struct.unpack(f">{sample_count}H", payload)
                )
        elif magic == b"P2":
            values_list = [int(_read_token(handle)) for _ in range(sample_count)]
            values = bytes(round(value * 255 / maximum) for value in values_list)
        else:
            raise ValueError(f"Unsupported PGM format {magic!r} in {path}")
        if maximum != 255 and magic == b"P5" and maximum < 256:
            values = bytes(round(value * 255 / maximum) for value in values)
        return width, height, maximum, values


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + kind
        + payload
        + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
    )


def grayscale_png(width: int, height: int, pixels: bytes) -> bytes:
    """Encode 8-bit grayscale pixels as a standards-compliant PNG."""
    rows = b"".join(
        b"\x00" + pixels[row * width : (row + 1) * width]
        for row in range(height)
    )
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(
            b"IHDR",
            struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0),
        )
        + _png_chunk(b"IDAT", zlib.compress(rows, level=9))
        + _png_chunk(b"IEND", b"")
    )


@dataclass(frozen=True)
class MapModel:
    yaml_path: Path
    image_path: Path
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    mode: str
    negate: int
    occupied_thresh: float
    free_thresh: float
    pixels: bytes
    png: bytes

    @classmethod
    def load(cls, yaml_path: Path) -> "MapModel":
        yaml_path = yaml_path.expanduser().resolve()
        if not yaml_path.is_file():
            raise FileNotFoundError(f"Map YAML does not exist: {yaml_path}")
        metadata = yaml.safe_load(yaml_path.read_text()) or {}
        required = ("image", "resolution", "origin")
        missing = [key for key in required if key not in metadata]
        if missing:
            raise ValueError(f"Map YAML is missing: {', '.join(missing)}")
        image_path = Path(str(metadata["image"])).expanduser()
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        image_path = image_path.resolve()
        if not image_path.is_file():
            raise FileNotFoundError(f"Map image does not exist: {image_path}")
        origin = metadata["origin"]
        if not isinstance(origin, list) or len(origin) < 3:
            raise ValueError("Map origin must be [x, y, yaw]")
        resolution = float(metadata["resolution"])
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError("Map resolution must be a positive finite value")
        width, height, _, pixels = read_pgm(image_path)
        return cls(
            yaml_path=yaml_path,
            image_path=image_path,
            width=width,
            height=height,
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]),
            mode=str(metadata.get("mode", "trinary")),
            negate=int(metadata.get("negate", 0)),
            occupied_thresh=float(metadata.get("occupied_thresh", 0.65)),
            free_thresh=float(metadata.get("free_thresh", 0.25)),
            pixels=pixels,
            png=grayscale_png(width, height, pixels),
        )

    def image_to_map(self, image_x: float, image_y: float) -> tuple[float, float]:
        """Convert top-left image coordinates to ROS map coordinates."""
        local_x = image_x * self.resolution
        local_y = (self.height - image_y) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )

    def map_to_image(self, map_x: float, map_y: float) -> tuple[float, float]:
        """Convert ROS map coordinates to top-left image coordinates."""
        delta_x = map_x - self.origin_x
        delta_y = map_y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        return local_x / self.resolution, self.height - local_y / self.resolution

    def metadata(self) -> dict[str, Any]:
        corners = [
            self.image_to_map(0, 0),
            self.image_to_map(self.width, 0),
            self.image_to_map(0, self.height),
            self.image_to_map(self.width, self.height),
        ]
        return {
            "name": self.yaml_path.stem,
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": [self.origin_x, self.origin_y, self.origin_yaw],
            "mode": self.mode,
            "negate": self.negate,
            "occupied_thresh": self.occupied_thresh,
            "free_thresh": self.free_thresh,
            "world_bounds": {
                "min_x": min(point[0] for point in corners),
                "max_x": max(point[0] for point in corners),
                "min_y": min(point[1] for point in corners),
                "max_y": max(point[1] for point in corners),
            },
            "source": str(self.yaml_path),
        }


def measurement_error(map_distance: float, real_distance: float | None) -> dict[str, float | None]:
    if real_distance is None:
        return {"difference": None, "percentage_error": None}
    difference = map_distance - real_distance
    percentage = difference / real_distance * 100.0 if real_distance else None
    return {"difference": difference, "percentage_error": percentage}


HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HorseShitBot October 1 LiDAR Viewer</title>
<style>
:root{color-scheme:dark;--bg:#111827;--panel:#1f2937;--line:#374151;--text:#f3f4f6;--muted:#9ca3af;--accent:#22d3ee;--warn:#f59e0b;--bad:#f87171}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px system-ui,sans-serif;height:100vh;overflow:hidden}
header{height:58px;display:flex;align-items:center;gap:18px;padding:8px 16px;background:#0b1220;border-bottom:1px solid var(--line)}
h1{font-size:17px;margin:0}.meta{color:var(--muted);min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.tabs{display:flex;gap:5px;margin-left:auto}
button,input,select{font:inherit}
button{background:#334155;color:white;border:1px solid #475569;border-radius:6px;padding:7px 10px;cursor:pointer}
button:hover,button.active{background:#0e7490;border-color:var(--accent)}button.danger:hover{background:#991b1b}
#layout{display:grid;grid-template-columns:minmax(0,1fr) 420px;height:calc(100vh - 58px)}
#viewport{position:relative;overflow:hidden;background:#030712;min-width:0}
#mapCanvas{width:100%;height:100%;display:block;cursor:crosshair}
#status{position:absolute;left:10px;bottom:10px;padding:7px 10px;background:#111827dd;border:1px solid var(--line);border-radius:6px;font-variant-numeric:tabular-nums}
#help{position:absolute;right:10px;top:10px;color:var(--muted);background:#111827dd;padding:6px 9px;border-radius:5px}
aside{background:var(--panel);border-left:1px solid var(--line);display:flex;flex-direction:column;min-width:0}
.controls{padding:12px;border-bottom:1px solid var(--line);display:flex;flex-wrap:wrap;gap:7px}
.controls label{color:var(--muted);display:flex;align-items:center;gap:5px}.controls select{max-width:190px;background:#111827;color:white;border:1px solid #4b5563;border-radius:5px;padding:6px}
.timeline{padding:9px 12px;border-bottom:1px solid var(--line);display:grid;grid-template-columns:1fr auto;gap:6px}.timeline input{width:100%;grid-column:1/-1}.timeline span{color:var(--muted);font-variant-numeric:tabular-nums}
.summary{padding:10px 12px;color:var(--muted);border-bottom:1px solid var(--line)}
.warning{padding:9px 12px;color:#fecaca;background:#7f1d1d66;border-bottom:1px solid #991b1b}.good{color:#bbf7d0}
.tableWrap{overflow:auto;flex:1}table{width:100%;border-collapse:collapse;font-size:12px}
th,td{padding:7px 6px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}
th{position:sticky;top:0;background:#273449;z-index:1;color:#cbd5e1}
td input{width:100%;min-width:64px;background:#111827;color:white;border:1px solid #4b5563;border-radius:4px;padding:4px}
.coords{white-space:nowrap;color:var(--muted);font-variant-numeric:tabular-nums}.number{white-space:nowrap;font-variant-numeric:tabular-nums}
.empty{padding:24px;color:var(--muted);text-align:center}.pending{color:var(--warn)}
.wallList{padding:8px 12px;max-height:160px;overflow:auto;border-bottom:1px solid var(--line);font-size:12px}.wallList div{padding:3px 0;color:#cbd5e1}
#measurementsIntro{display:none;padding:40px;max-width:780px;margin:auto;color:#cbd5e1;line-height:1.6}#measurementsIntro h2{color:white}
@media(max-width:900px){#layout{grid-template-columns:1fr;grid-template-rows:58vh 1fr}aside{border-left:0;border-top:1px solid var(--line);overflow:auto}.tableWrap{min-height:160px;overflow:visible}}
</style>
</head>
<body>
<header><h1>HorseShitBot · October 1 LiDAR</h1><span id="mapMeta" class="meta">Loading actual recording data…</span>
  <nav class="tabs"><button data-tab="scan" class="active">LiDAR Scan</button><button data-tab="map">Room Map</button><button data-tab="comparison">Real Room Comparison</button><button data-tab="measurements">Measurements</button></nav>
</header>
<div id="layout">
  <main id="viewport">
    <canvas id="mapCanvas"></canvas>
    <section id="measurementsIntro"><h2>Physical comparison measurements</h2><p>Measurements from the single-scan and reconstructed-map views remain separate and are labelled by source below. Enter the corresponding physical distance to calculate signed error and percentage error.</p><p>Single-scan distances describe LiDAR range geometry directly. Map distances also include scan registration and SLAM drift, so they are not evidence of range calibration by themselves.</p></section>
    <div id="help">Measure: click A then B · Pan: drag · Wheel: zoom</div>
    <div id="status">Loading…</div>
  </main>
  <aside>
    <div class="controls">
      <button id="measureBtn" class="active">Measure</button><button id="panBtn">Pan</button>
      <button id="zoomIn">＋</button><button id="zoomOut">−</button><button id="fitBtn">Fit</button>
      <span id="playbackControls"><button id="prevBtn">◀</button><button id="playBtn">Play</button><button id="pauseBtn">Pause</button><button id="nextBtn">▶</button></span>
      <label id="speedLabel">Speed <select id="speedSelect"><option value=".25">0.25×</option><option value=".5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option></select></label>
      <label id="accumulationLabel">View <select id="accumulationSelect"><option value="1">Single scan</option><option value="5">Last 5 scans</option><option value="20">Last 20 scans</option><option value="stationary">Selected stationary interval</option></select></label>
      <label id="stationaryLabel" hidden>Interval <select id="stationarySelect"></select></label>
      <label id="recommendedLabel">Recommended Scans for Measurement <select id="recommendedSelect"><option value="">Choose…</option></select></label>
      <label id="validatedGeometryLabel"><input id="validatedGeometryToggle" type="checkbox"> Show Validated Room Geometry</label>
      <button id="rawBtn">Raw</button><button id="correctedBtn" class="active">Corrected</button>
      <span id="comparisonControls" style="display:none;flex-wrap:wrap;gap:6px;width:100%">
        <label><input id="compValidatedToggle" type="checkbox" checked> Show Validated Room Geometry</label>
        <label><input id="overlayToggle" type="checkbox" checked> Rigid Overlay (Scale=1.0)</label>
        <label>dX <input id="ovTx" type="number" step="0.05" value="-2.62" style="width:62px"></label>
        <label>dY <input id="ovTy" type="number" step="0.05" value="-4.15" style="width:62px"></label>
        <label>Yaw° <input id="ovYaw" type="number" step="1" value="-11" style="width:56px"></label>
        <label>Target <select id="compTargetSelect"><option value="W1">Upper width W1 (3.117 m)</option><option value="W2">Middle width W2 (3.272 m)</option><option value="W3">Lower width W3 (3.137 m)</option><option value="L1">Overall length L1 (10.600 m)</option></select></label>
        <label>Stationary Interval <select id="compIntervalSelect"><option value="all">All 9 stationary intervals (summary)</option></select></label>
        <button id="exportFloorPlanBtn">Floor Plan PNG</button>
        <button id="exportCompPngBtn">Comparison PNG</button>
        <button id="exportCompCsvBtn">Comparison CSV</button>
        <button id="exportReportMdBtn">Report MD</button>
      </span>
      <button id="csvBtn">Export CSV</button><button id="pngBtn">Export PNG</button>
      <button id="clearBtn" class="danger">Clear</button>
    </div>
    <div id="timeline" class="timeline"><input id="timelineSlider" type="range" min="0" max="0" value="0"><span id="scanClock">—</span><span id="scanNumber">—</span></div>
    <div id="warning" class="warning">Loading geometry diagnosis…</div>
    <div id="summary" class="summary">Click two map points to add a measurement.</div>
    <div id="wallList" class="wallList"></div>
    <div id="comparisonPanel" class="tableWrap" style="display:none">
      <div id="discrepancyBanner" class="warning" style="font-size:12px"></div>
      <table id="accuracyTable"><thead><tr><th>Reference</th><th>Real distance</th><th>LiDAR scan</th><th>SLAM map</th><th>Error</th></tr></thead><tbody></tbody></table>
      <div id="accuracyStats" class="summary"></div>
      <div id="stationaryDiagnosticBox" class="wallList" style="max-height:none"></div>
      <div id="unassignedNotes" class="wallList"></div>
    </div>
    <div id="measureTableWrap" class="tableWrap"><div id="empty" class="empty">No measurements yet.</div>
      <table id="measureTable" hidden><thead><tr><th>Name / source</th><th>Measured</th><th>Real</th><th>Error</th><th></th></tr></thead><tbody></tbody></table>
    </div>
  </aside>
</div>
<script>
"use strict";
const canvas=document.getElementById("mapCanvas"),ctx=canvas.getContext("2d"),viewport=document.getElementById("viewport");
const image=new Image();let meta=null,data=null,room=null,tab="scan",scanKind="corrected",scanIndex=0,zoom=1,offsetX=0,offsetY=0,mode="measure",pending=null,drag=null,measurements=[],nextId=1,playTimer=null,selectedWalls=[],selectedPhysicalWall="R1",selectedMapWalls=[],comparisonRows=[];
const colors=["#f43f5e","#22d3ee","#a3e635","#f59e0b","#c084fc","#fb7185","#2dd4bf","#60a5fa"];
function imageToMap(x,y){const lx=x*meta.resolution,ly=(meta.height-y)*meta.resolution,c=Math.cos(meta.origin[2]),s=Math.sin(meta.origin[2]);return{x:meta.origin[0]+c*lx-s*ly,y:meta.origin[1]+s*lx+c*ly}}
function mapToImage(x,y){const dx=x-meta.origin[0],dy=y-meta.origin[1],c=Math.cos(meta.origin[2]),s=Math.sin(meta.origin[2]);return{x:(c*dx+s*dy)/meta.resolution,y:meta.height-(-s*dx+c*dy)/meta.resolution}}
function worldToScreen(p){if(tab==="map"){const q=mapToImage(p.x,p.y);return{x:offsetX+q.x*zoom,y:offsetY+q.y*zoom}}return{x:offsetX+p.x*zoom,y:offsetY-p.y*zoom}}
function screenToWorld(x,y){if(tab==="map"){const q={x:(x-offsetX)/zoom,y:(y-offsetY)/zoom};return imageToMap(q.x,q.y)}return{x:(x-offsetX)/zoom,y:(offsetY-y)/zoom}}
function distance(a,b){return Math.hypot(a.world.x-b.world.x,a.world.y-b.world.y)}
function chooseGrid(){const meterPixels=tab==="map"?zoom/meta.resolution:zoom,target=85/meterPixels,p=Math.pow(10,Math.floor(Math.log10(Math.max(target,1e-6))));return[1,2,5,10].map(v=>v*p).find(v=>v>=target)||10*p}
function resize(){const d=devicePixelRatio||1,w=viewport.clientWidth,h=viewport.clientHeight;canvas.width=Math.round(w*d);canvas.height=Math.round(h*d);canvas.style.width=w+"px";canvas.style.height=h+"px";ctx.setTransform(d,0,0,d,0,0);render()}
function scanPoints(scan){return scanKind==="raw"?scan.corrected_points.map(p=>[p[0],-p[1]]):scan.corrected_points}
function fit(){if(!meta||!data)return;const w=viewport.clientWidth,h=viewport.clientHeight;if(tab==="map"){zoom=Math.min((w-50)/meta.width,(h-50)/meta.height);offsetX=(w-meta.width*zoom)/2;offsetY=(h-meta.height*zoom)/2}else if(tab==="comparison"){zoom=Math.min((w/2-90)/5.2,(h-90)/17.5);offsetX=0;offsetY=0}else{const pts=scanPoints(currentScan()),extent=Math.max(2,...pts.flatMap(p=>[Math.abs(p[0]),Math.abs(p[1])]))*1.08;zoom=Math.min((w-70)/(2*extent),(h-70)/(2*extent));offsetX=w/2;offsetY=h/2}render()}
function drawGrid(){
  if(tab==="comparison")return;
  const step=chooseGrid(),w=viewport.clientWidth,h=viewport.clientHeight,a=screenToWorld(0,h),b=screenToWorld(w,0),minX=Math.min(a.x,b.x),maxX=Math.max(a.x,b.x),minY=Math.min(a.y,b.y),maxY=Math.max(a.y,b.y);
  ctx.save();ctx.lineWidth=1;ctx.strokeStyle="#0ea5e944";ctx.fillStyle="#bfdbfe";ctx.font="11px system-ui";
  for(let x=Math.ceil(minX/step)*step;x<=maxX;x+=step){const p=worldToScreen({x,y:0});ctx.beginPath();ctx.moveTo(p.x,0);ctx.lineTo(p.x,h);ctx.stroke();if(p.x>5&&p.x<w-35)ctx.fillText(`${x.toFixed(step<1?1:0)}m`,p.x+3,h-7)}
  for(let y=Math.ceil(minY/step)*step;y<=maxY;y+=step){const p=worldToScreen({x:0,y});ctx.beginPath();ctx.moveTo(0,p.y);ctx.lineTo(w,p.y);ctx.stroke()}
  ctx.restore();
}
function render(){
  if(!meta||!data||!image.complete||tab==="measurements")return;const w=viewport.clientWidth,h=viewport.clientHeight;ctx.clearRect(0,0,w,h);ctx.imageSmoothingEnabled=false;
  if(tab==="comparison"){renderComparison(w,h);drawScaleBar(ctx,w,h);return}
  if(tab==="map")renderMap();else renderScan();drawGrid();
  measurements.filter(m=>m.source===sourceName()).forEach((m,i)=>drawMeasurement(ctx,m,colors[i%colors.length],1));
  if(pending){const p=worldToScreen(pending.world);ctx.fillStyle="#fbbf24";ctx.beginPath();ctx.arc(p.x,p.y,6,0,Math.PI*2);ctx.fill()}
  drawScaleBar(ctx,w,h);
}
function physToScreen(x,y,w,h){const cx=w*0.25+offsetX,cy=h*0.52+offsetY;return{x:cx+(x-1.6)*zoom,y:cy-(y-5.3)*zoom}}
function screenToPhys(sx,sy,w,h){const cx=w*0.25+offsetX,cy=h*0.52+offsetY;return{x:(sx-cx)/zoom+1.6,y:5.3-(sy-cy)/zoom}}
function mapWorldToCompScreen(x,y,w,h){const cx=w*0.74+offsetX,cy=h*0.52+offsetY;return{x:cx+(x+0.4)*zoom,y:cy-(y+1.9)*zoom}}
function compScreenToMapWorld(sx,sy,w,h){const cx=w*0.74+offsetX,cy=h*0.52+offsetY;return{x:(sx-cx)/zoom-0.4,y:-1.9-(sy-cy)/zoom}}
function renderComparison(w,h){
  if(!room)return;const half=w/2;
  ctx.save();ctx.strokeStyle="#334155";ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(half,0);ctx.lineTo(half,h);ctx.stroke();
  ctx.fillStyle="#e2e8f0";ctx.font="bold 13px system-ui";
  ctx.fillText("LEFT: Physical Reference Floor Plan (Provisional m, Equal Axes)",16,24);
  ctx.fillText("RIGHT: Reconstructed SLAM Map + Rigid Overlay (Scale=1.0)",half+16,24);
  // Metric 1m grid on left & right
  ctx.strokeStyle="#0ea5e928";ctx.lineWidth=1;
  for(let gx=-1;gx<=5;gx++){const a=physToScreen(gx,-1,w,h),b=physToScreen(gx,11.5,w,h);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke()}
  for(let gy=0;gy<=11;gy++){const a=physToScreen(-1,gy,w,h),b=physToScreen(5,gy,w,h);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke()}
  // Draw physical walls
  room.walls.forEach(wall=>{
    const a=physToScreen(wall.x_start_m,wall.y_start_m,w,h),b=physToScreen(wall.x_end_m,wall.y_end_m,w,h),isGap=wall.status==="unresolved_discrepancy",sel=selectedPhysicalWall===wall.id;
    ctx.save();if(isGap)ctx.setLineDash([6,4]);
    ctx.strokeStyle=sel?"#ffffff":(isGap?"#f59e0b":(wall.side==="left"?"#38bdf8":"#4ade80"));
    ctx.lineWidth=sel?5:3;ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();
    const mx=(a.x+b.x)/2+(wall.side==="left"?-68:8),my=(a.y+b.y)/2;
    ctx.fillStyle=ctx.strokeStyle;ctx.font="11px system-ui";ctx.fillText(`${wall.id}: ${wall.length_m.toFixed(3)} m`,mx,my);ctx.restore();
  });
  room.widths.forEach(item=>{
    const a=physToScreen(0,item.y_display_m,w,h),b=physToScreen(item.measured_m,item.y_display_m,w,h),active=document.getElementById("compTargetSelect").value===item.id;
    ctx.save();ctx.strokeStyle=active?"#fbbf24":"#f43f5e";ctx.fillStyle=ctx.strokeStyle;ctx.lineWidth=active?2.5:1.5;ctx.setLineDash([4,3]);
    ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();ctx.setLineDash([]);
    ctx.font="bold 11px system-ui";ctx.fillText(`${item.id} = ${item.measured_m.toFixed(3)} m`,(a.x+b.x)/2-38,a.y-6);ctx.restore();
  });
  // Draw SLAM occupancy grid on right
  const tl=mapWorldToCompScreen(meta.origin[0],meta.origin[1]+meta.height*meta.resolution,w,h);
  ctx.drawImage(image,tl.x,tl.y,meta.width*meta.resolution*zoom,meta.height*meta.resolution*zoom);
  (data.map_walls||[]).forEach((mw,i)=>{
    const a=mapWorldToCompScreen(mw.start[0],mw.start[1],w,h),b=mapWorldToCompScreen(mw.end[0],mw.end[1],w,h),sel=selectedMapWalls.includes(i);
    ctx.save();ctx.strokeStyle=sel?"#ffffff":"#f97316";ctx.lineWidth=sel?5:2.4;
    ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();
    ctx.fillStyle=sel?"#ffffff":"#fdba74";ctx.font="11px system-ui";ctx.fillText(`${mw.id} (${mw.length_m.toFixed(2)}m)`,(a.x+b.x)/2+4,(a.y+b.y)/2);ctx.restore();
  });
  if(document.getElementById("overlayToggle").checked){
    const tx=+document.getElementById("ovTx").value||0,ty=+document.getElementById("ovTy").value||0,yaw=(+document.getElementById("ovYaw").value||0)*Math.PI/180,c=Math.cos(yaw),s=Math.sin(yaw);
    room.walls.forEach(wall=>{
      const x0=tx+c*wall.x_start_m-s*wall.y_start_m,y0=ty+s*wall.x_start_m+c*wall.y_start_m,x1=tx+c*wall.x_end_m-s*wall.y_end_m,y1=ty+s*wall.x_end_m+c*wall.y_end_m;
      const a=mapWorldToCompScreen(x0,y0,w,h),b=mapWorldToCompScreen(x1,y1,w,h),isGap=wall.status==="unresolved_discrepancy";
      ctx.save();ctx.strokeStyle=isGap?"#f59e0b":"#22d3ee";ctx.lineWidth=2.2;if(isGap)ctx.setLineDash([5,4]);
      ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();ctx.restore();
    });
  }
  if(document.getElementById("compValidatedToggle")?.checked){
    const cand=steppedCandidateForScan(currentScan().source_index);
    if(cand){
      const sObj=data.scans[cand.scan_index],boxW=Math.min(340,half-28),boxH=230,bx=14,by=h-boxH-52;
      ctx.save();ctx.fillStyle="#0b1220ea";ctx.strokeStyle="#22d3ee";ctx.lineWidth=1.5;ctx.fillRect(bx,by,boxW,boxH);ctx.strokeRect(bx,by,boxW,boxH);
      ctx.fillStyle="#e2e8f0";ctx.font="bold 11px system-ui";
      ctx.fillText(`2. Actual LiDAR Scan #${cand.scan_index+1} (+${cand.elapsed_s.toFixed(1)}s) — Unreshaped Points`,bx+8,by+16);
      const sc=24,cx=bx+boxW*0.5,cy=by+boxH*0.54,toInset=p=>({x:cx+p.x*sc,y:cy-p.y*sc});
      ctx.fillStyle="#94a3b8";for(const pt of sObj.corrected_points){const q=toInset({x:pt[0],y:pt[1]});if(q.x>bx+4&&q.x<bx+boxW-4&&q.y>by+22&&q.y<by+boxH-4)ctx.fillRect(q.x-1,q.y-1,2,2)}
      const o=toInset({x:0,y:0});ctx.fillStyle="#ef4444";ctx.beginPath();ctx.arc(o.x,o.y,4,0,Math.PI*2);ctx.fill();
      drawValidatedScanOverlay(cand,toInset);ctx.restore();
    }
  }
  ctx.restore();
}
function renderMap(){ctx.drawImage(image,offsetX,offsetY,meta.width*zoom,meta.height*zoom);const poses=data.trajectory.poses;ctx.save();ctx.setLineDash([6,5]);ctx.strokeStyle="#f97316";ctx.lineWidth=2;ctx.beginPath();poses.forEach((p,i)=>{const q=worldToScreen({x:p[1],y:p[2]});i?ctx.lineTo(q.x,q.y):ctx.moveTo(q.x,q.y)});ctx.stroke();ctx.restore()}
function currentScan(){return data.scans[scanIndex]}
function transformToCurrent(point,source,current){const cs=Math.cos(source.pose[2]),ss=Math.sin(source.pose[2]),gx=source.pose[0]+cs*point[0]-ss*point[1],gy=source.pose[1]+ss*point[0]+cs*point[1],dx=gx-current.pose[0],dy=gy-current.pose[1],cc=Math.cos(current.pose[2]),sc=Math.sin(current.pose[2]);return[cc*dx+sc*dy,-sc*dx+cc*dy]}
function requestedScans(){const value=document.getElementById("accumulationSelect").value,current=currentScan();if(value==="1"||scanKind==="raw")return{scans:[current],reliable:value==="1",reason:scanKind==="raw"&&value!=="1"?"Accumulation disabled: ICP alignment was estimated from corrected scans.":""};let start;if(value==="stationary"){const interval=data.stationary_intervals[+document.getElementById("stationarySelect").value];if(!interval)return{scans:[current],reliable:false,reason:"No qualified stationary interval selected."};start=interval.start_index;const end=interval.end_index;if(scanIndex<start||scanIndex>end)scanIndex=Math.floor((start+end)/2);return{scans:data.scans.slice(start,end+1),reliable:true,reason:`Qualified stationary interval: ${end-start+1} aligned scans.`}}const count=+value;start=Math.max(0,scanIndex-count+1);const scans=data.scans.slice(start,scanIndex+1),policy=data.accumulation_policy;for(let i=1;i<scans.length;i++){const a=scans[i-1],b=scans[i],dt=Math.hypot(b.pose[0]-a.pose[0],b.pose[1]-a.pose[1]),da=Math.abs(((b.pose[2]-a.pose[2]+Math.PI)%(2*Math.PI))-Math.PI);if(dt>policy.translation_step_limit_m||da>policy.heading_step_limit_deg*Math.PI/180||b.registration_inlier<policy.minimum_inlier_ratio||b.registration_rmse_m>policy.maximum_rmse_m)return{scans:[current],reliable:false,reason:`Requested ${count}-scan window includes motion or weak registration; showing only scan ${scanIndex+1}.`}}return{scans,reliable:true,reason:`${scans.length} scans aligned using gated diagnostic ICP poses.`}}
function steppedCandidateForScan(idx){return(data.stepped_scan_candidates||[]).find(c=>c.scan_index===idx)||(data.stepped_scan_candidates||[])[0]||null}
function drawValidatedScanOverlay(cand,projectFn){
  if(!cand)return;
  const lwStart={x:cand.left_wall_start[0],y:cand.left_wall_start[1]},lwEnd={x:cand.left_wall_end[0],y:cand.left_wall_end[1]};
  const aL=projectFn(lwStart),bL=projectFn(lwEnd);
  ctx.save();ctx.strokeStyle="#38bdf8";ctx.lineWidth=5;ctx.beginPath();ctx.moveTo(aL.x,aL.y);ctx.lineTo(bL.x,bL.y);ctx.stroke();
  ctx.fillStyle="#0b1220ee";ctx.fillRect((aL.x+bL.x)/2-88,(aL.y+bL.y)/2-22,176,18);
  ctx.fillStyle="#38bdf8";ctx.font="bold 11px system-ui";ctx.fillText(`LiDAR ${cand.left_wall_id} → Physical L1 (${cand.left_wall_length_m.toFixed(2)}m)`,(aL.x+bL.x)/2-84,(aL.y+bL.y)/2-9);
  const dx=lwEnd.x-lwStart.x,dy=lwEnd.y-lwStart.y,len=Math.hypot(dx,dy)||1,ux=dx/len,uy=dy/len,cx=(lwStart.x+lwEnd.x)/2,cy=(lwStart.y+lwEnd.y)/2;
  let nx=-uy,ny=ux;if((-cx*nx-cy*ny)<0){nx=-nx;ny=-ny}
  const ms=data.physical_comparison?.multiscan_statistics?.sections||{};
  const specs=[
    {id:"W1",phys:"Physical R1/W1 (3.117 m)",seg:cand.upper_candidate,col:"#4ade80",stat:ms.W1},
    {id:"W2",phys:"Physical R2/W2 (3.272 m)",seg:cand.middle_candidate,col:"#fbbf24",stat:ms.W2},
    {id:"W3",phys:"Physical R3/W3 (3.137 m)",seg:cand.lower_candidate,col:"#c084fc",stat:ms.W3},
  ];
  specs.forEach(sp=>{
    const seg=sp.seg;if(!seg)return;
    const s0={x:seg.start[0],y:seg.start[1]},s1={x:seg.end[0],y:seg.end[1]},cb={x:(s0.x+s1.x)/2,y:(s0.y+s1.y)/2};
    const proj=((cb.x-cx)*nx+(cb.y-cy)*ny),foot={x:cb.x-proj*nx,y:cb.y-proj*ny};
    const p0=projectFn(s0),p1=projectFn(s1),pc=projectFn(cb),pf=projectFn(foot);
    ctx.strokeStyle=sp.col;ctx.lineWidth=5;ctx.beginPath();ctx.moveTo(p0.x,p0.y);ctx.lineTo(p1.x,p1.y);ctx.stroke();
    ctx.setLineDash([5,4]);ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(pf.x,pf.y);ctx.lineTo(pc.x,pc.y);ctx.stroke();ctx.setLineDash([]);
    const mx=(pf.x+pc.x)/2,my=(pf.y+pc.y)/2,txt=`LiDAR ${seg.wall_id}: ${seg.perpendicular_width_m.toFixed(4)}m | ${sp.phys}${sp.stat?` | multi-scan ${sp.stat.mean_m.toFixed(3)}±${sp.stat.std_m.toFixed(3)}m`:""}`;
    ctx.font="bold 11px system-ui";const tw=ctx.measureText(txt).width;
    ctx.fillStyle="#0b1220ee";ctx.fillRect(mx-tw/2-4,my-15,tw+8,18);
    ctx.fillStyle=sp.col;ctx.fillText(txt,mx-tw/2,my-2);
  });
  ctx.restore();
}
function renderScan(){const current=currentScan(),group=requestedScans();group.scans.forEach((scan,index)=>{const pts=scanPoints(scan),alpha=.2+.8*(index+1)/group.scans.length;ctx.fillStyle=scanKind==="raw"?`rgba(245,158,11,${alpha})`:`rgba(103,232,249,${alpha})`;for(const point of pts){const q=scan===current?point:transformToCurrent(point,scan,current),p=worldToScreen({x:q[0],y:q[1]});ctx.fillRect(p.x-1.3,p.y-1.3,2.6,2.6)}});const o=worldToScreen({x:0,y:0});ctx.fillStyle="#ef4444";ctx.beginPath();ctx.arc(o.x,o.y,7,0,Math.PI*2);ctx.fill();ctx.strokeStyle="#ef4444";ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(o.x,o.y);ctx.lineTo(o.x+30,o.y);ctx.stroke();if(scanKind==="corrected")current.walls.forEach((wall,i)=>{const a=worldToScreen({x:wall.start[0],y:wall.start[1]}),b=worldToScreen({x:wall.end[0],y:wall.end[1]});ctx.strokeStyle=selectedWalls.includes(i)?"#fff":colors[i%colors.length];ctx.lineWidth=selectedWalls.includes(i)?6:3;ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke()});if(scanKind==="corrected"&&document.getElementById("validatedGeometryToggle")?.checked){drawValidatedScanOverlay(steppedCandidateForScan(current.source_index),worldToScreen)}document.getElementById("warning").textContent=group.reason||"Single scan: actual ranges only; no odometry or SLAM."}
function drawMeasurement(context,m,color,mult){
  const a=worldToScreen(m.a.world),b=worldToScreen(m.b.world);context.save();context.strokeStyle=color;context.fillStyle=color;context.lineWidth=3*mult;
  context.beginPath();context.moveTo(a.x,a.y);context.lineTo(b.x,b.y);context.stroke();
  [a,b].forEach(p=>{context.beginPath();context.arc(p.x,p.y,5*mult,0,Math.PI*2);context.fill()});
  const label=`${m.name}: ${distance(m.a,m.b).toFixed(2)} m`,mx=(a.x+b.x)/2,my=(a.y+b.y)/2;
  context.font=`${12*mult}px system-ui`;const tw=context.measureText(label).width;context.fillStyle="#111827dd";context.fillRect(mx-tw/2-4*mult,my-18*mult,tw+8*mult,17*mult);context.fillStyle=color;context.fillText(label,mx-tw/2,my-5*mult);context.restore()
}
function drawScaleBar(context,w,h){const ppm=tab==="map"?zoom/meta.resolution:zoom,target=120/ppm,p=Math.pow(10,Math.floor(Math.log10(target))),meters=[1,2,5,10].map(v=>v*p).find(v=>v>=target)||10*p,px=meters*ppm,x=18,y=h-45;context.strokeStyle="#fff";context.fillStyle="#fff";context.lineWidth=3;context.beginPath();context.moveTo(x,y);context.lineTo(x+px,y);context.moveTo(x,y-5);context.lineTo(x,y+5);context.moveTo(x+px,y-5);context.lineTo(x+px,y+5);context.stroke();context.font="12px system-ui";context.fillText(`${meters.toPrecision(2)} m`,x,y-9)}
function eventPoint(e){const r=canvas.getBoundingClientRect();return{x:e.clientX-r.left,y:e.clientY-r.top}}
function sourceName(){return tab==="scan"?`scan-${currentScan().source_index}-${scanKind}`:"occupancy-map"}
function pointSegmentDistance(p,a,b){const dx=b.x-a.x,dy=b.y-a.y,t=Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy)));return Math.hypot(p.x-a.x-t*dx,p.y-a.y-t*dy)}
function hitWall(screen){if(tab!=="scan"||scanKind!=="corrected")return-1;let best=-1,distance=9;currentScan().walls.forEach((wall,i)=>{const a=worldToScreen({x:wall.start[0],y:wall.start[1]}),b=worldToScreen({x:wall.end[0],y:wall.end[1]}),d=pointSegmentDistance(screen,a,b);if(d<distance){distance=d;best=i}});return best}
function wallAngle(w){return Math.atan2(w.end[1]-w.start[1],w.end[0]-w.start[0])}
function wallCenter(w){return{x:(w.start[0]+w.end[0])/2,y:(w.start[1]+w.end[1])/2}}
function selectWall(index){if(selectedWalls.includes(index))selectedWalls=selectedWalls.filter(value=>value!==index);else{selectedWalls.push(index);if(selectedWalls.length>2)selectedWalls.shift()}const s=currentScan(),details=selectedWalls.map(i=>{const w=s.walls[i];return`W${i+1}: distance ${w.sensor_distance_m.toFixed(3)} m · length ${w.length_m.toFixed(3)} m · orientation ${(wallAngle(w)*180/Math.PI).toFixed(1)}° · RMSE ${w.rmse_m.toFixed(3)} m · +${s.elapsed_s.toFixed(3)} s`});if(selectedWalls.length===2){const [ia,ib]=selectedWalls,wa=s.walls[ia],wb=s.walls[ib],aa=wallAngle(wa),ab=wallAngle(wb),delta=Math.abs(Math.atan2(Math.sin(aa-ab),Math.cos(aa-ab))),parallel=Math.min(delta,Math.PI-delta)<5*Math.PI/180,ca=wallCenter(wa),cb=wallCenter(wb),axis={x:Math.cos(aa),y:Math.sin(aa)},aProj=[wa.start[0]*axis.x+wa.start[1]*axis.y,wa.end[0]*axis.x+wa.end[1]*axis.y].sort((x,y)=>x-y),bProj=[wb.start[0]*axis.x+wb.start[1]*axis.y,wb.end[0]*axis.x+wb.end[1]*axis.y].sort((x,y)=>x-y),overlap=Math.min(aProj[1],bProj[1])-Math.max(aProj[0],bProj[0]);let value,label,a,b;if(parallel&&overlap>0.2){const normal={x:-axis.y,y:axis.x};value=Math.abs((cb.x-ca.x)*normal.x+(cb.y-ca.y)*normal.y);label="perpendicular wall separation";a={world:ca};b={world:{x:ca.x+normal.x*value,y:ca.y+normal.y*value}}}else{const pairs=[[wa.start,wb.start],[wa.start,wb.end],[wa.end,wb.start],[wa.end,wb.end]].map(pair=>({pair,d:Math.hypot(pair[0][0]-pair[1][0],pair[0][1]-pair[1][1])})).sort((x,y)=>x.d-y.d);value=pairs[0].d;label="nearest-endpoint distance (walls not parallel/overlapping)";a={world:{x:pairs[0].pair[0][0],y:pairs[0].pair[0][1]}};b={world:{x:pairs[0].pair[1][0],y:pairs[0].pair[1][1]}}}measurements.push({id:nextId,name:`W${ia+1} ↔ W${ib+1}`,source:`scan-${s.source_index}-walls`,scan_stamp_ns:s.stamp_ns,wall_ids:[`W${ia+1}`,`W${ib+1}`],method:label,a,b,real:null});nextId++;details.push(`${label}: ${value.toFixed(3)} m`);updateTable()}document.getElementById("wallList").innerHTML=details.length?details.map(x=>`<div>${x}</div>`).join(""):"Click a colored wall segment for details.";render()}
function hitPhysicalWall(screen,w,h){if(!room||screen.x>w/2)return null;let best=null,dist=10;room.walls.forEach(wall=>{const a=physToScreen(wall.x_start_m,wall.y_start_m,w,h),b=physToScreen(wall.x_end_m,wall.y_end_m,w,h),d=pointSegmentDistance(screen,a,b);if(d<dist){dist=d;best=wall.id}});return best}
function hitMapWall(screen,w,h){if(screen.x<=w/2)return-1;let best=-1,dist=10;(data.map_walls||[]).forEach((mw,i)=>{const a=mapWorldToCompScreen(mw.start[0],mw.start[1],w,h),b=mapWorldToCompScreen(mw.end[0],mw.end[1],w,h),d=pointSegmentDistance(screen,a,b);if(d<dist){dist=d;best=i}});return best}
function applyMapWallSelection(){
  const target=document.getElementById("compTargetSelect").value,row=comparisonRows.find(r=>r.reference_id===target);
  if(!row)return;
  if(target==="L1"&&selectedMapWalls.length===1){
    const mw=data.map_walls[selectedMapWalls[0]];
    row.slam_map_distance_m=mw.length_m;
    row.slam_map_source=`${mw.id} length (${mw.length_m.toFixed(3)} m, RMSE ${mw.rmse_m.toFixed(3)} m)`;
    updateComparisonTable();return;
  }
  if(selectedMapWalls.length===2){
    const [ia,ib]=selectedMapWalls,wa=data.map_walls[ia],wb=data.map_walls[ib],aa=wallAngle(wa),ab=wallAngle(wb),delta=Math.abs(Math.atan2(Math.sin(aa-ab),Math.cos(aa-ab))),parallel=Math.min(delta,Math.PI-delta)<7*Math.PI/180,ca=wallCenter(wa),cb=wallCenter(wb),axis={x:Math.cos(aa),y:Math.sin(aa)},normal={x:-axis.y,y:axis.x};
    const val=parallel?Math.abs((cb.x-ca.x)*normal.x+(cb.y-ca.y)*normal.y):Math.hypot(ca.x-cb.x,ca.y-cb.y);
    row.slam_map_distance_m=+val.toFixed(4);
    row.slam_map_source=`${wa.id}↔${wb.id} (${parallel?"perp":"center"} ${val.toFixed(3)} m)`;
    updateComparisonTable();
  }
}
function updateComparisonTable(){
  if(!room)return;
  const disc=room.discrepancies[0];
  document.getElementById("discrepancyBanner").textContent=`DISCREPANCY FLAGGED: Left wall L1 = ${disc.left_wall_length_m.toFixed(3)} m, whereas R1+R2+R3 = 4.750 + 4.400 + 1.172 = ${disc.right_wall_sum_m.toFixed(3)} m (Δ = ${disc.difference_m.toFixed(3)} m). Floor-plan polygon is not closed without this unresolved 0.278 m longitudinal interval.`;
  const tbody=document.querySelector("#accuracyTable tbody");tbody.textContent="";
  const scanErrs=[],mapErrs=[];
  comparisonRows.forEach(row=>{
    const real=row.real_distance_m,scanVal=row.lidar_scan_distance_m,mapVal=row.slam_map_distance_m;
    const sDiff=scanVal==null?null:scanVal-real,sAbs=sDiff==null?null:Math.abs(sDiff),sPct=sDiff==null?null:sDiff/real*100;
    const mDiff=mapVal==null?null:mapVal-real,mAbs=mDiff==null?null:Math.abs(mDiff),mPct=mDiff==null?null:mDiff/real*100;
    if(sAbs!=null)scanErrs.push(sDiff);if(mAbs!=null)mapErrs.push(mDiff);
    const errParts=[];
    if(sDiff!=null)errParts.push(`Scan: ${signed(sDiff,4)} m (|e|=${sAbs.toFixed(4)} m, ${signed(sPct,2)}%)`);
    if(mDiff!=null)errParts.push(`Map: ${signed(mDiff,4)} m (|e|=${mAbs.toFixed(4)} m, ${signed(mPct,2)}%)`);
    const tr=document.createElement("tr");
    tr.innerHTML=`<td><strong>${escapeHtml(row.reference)}</strong><div class="coords">${escapeHtml(row.reference_id)}</div></td><td class="number">${real.toFixed(3)} m</td><td class="number">${scanVal==null?"To measure / unconfirmed":`${scanVal.toFixed(4)} m`}<div class="coords">${escapeHtml(row.lidar_scan_source||"")}</div></td><td class="number">${mapVal==null?"To measure":`${mapVal.toFixed(4)} m`}<div class="coords">${escapeHtml(row.slam_map_source||"Select corresponding map walls")}</div></td><td class="number">${errParts.length?errParts.join("<br>"):"Identify features"}</td>`;
    tbody.appendChild(tr);
  });
  const fmtStats=(label,arr)=>{if(!arr.length)return`${label}: MAE — · RMSE — (0 matched)`;const mae=arr.reduce((a,b)=>a+Math.abs(b),0)/arr.length,rmse=Math.sqrt(arr.reduce((a,b)=>a+b*b,0)/arr.length);return`${label} (${arr.length}): MAE ${mae.toFixed(4)} m · RMSE ${rmse.toFixed(4)} m`};
  const msInfo=data.physical_comparison?.multiscan_statistics;
  const stDiag=data.physical_comparison?.stationary_diagnostics;
  const biasNote=msInfo?.selection_bias_finding?`<br><span class="coords">${escapeHtml(msInfo.selection_bias_finding)}</span>`:"";
  document.getElementById("accuracyStats").innerHTML=`${fmtStats("Target-free multi-scan LiDAR",scanErrs)}<br>${fmtStats("SLAM occupancy map",mapErrs)}<br><span class="coords">Diagnostic SLAM longitudinal scale ratio (M1 9.104 m / L1 10.600 m): 0.859 (not applied to overlay)</span>${biasNote}`;
  const ivSel=document.getElementById("compIntervalSelect")?.value||"all";
  let diagHtml=`<strong>Stationary-Interval & Uncertainty Diagnostic</strong>`;
  if(stDiag){
    if(ivSel==="all"){
      diagHtml+=`<table style="margin-top:6px"><thead><tr><th>Width</th><th>Physical</th><th>LiDAR Planar Mean</th><th>Diff (cm / %)</th><th>Scans (k intervals)</th><th>Repeatability</th><th>LiDAR 1σ Uncertainty</th><th>Confidence</th></tr></thead><tbody>`;
      ["W1","W2","W3"].forEach(k=>{
        const b=stDiag.between_intervals[k];if(!b)return;
        diagHtml+=`<tr><td><strong>${k}</strong></td><td class="number">${b.real_distance_m.toFixed(3)} m</td><td class="number">${b.planar_interval_mean_m.toFixed(4)} m</td><td class="number">${signed(b.planar_diff_cm,2)} cm (${signed(b.planar_diff_pct,2)}%)</td><td class="number">${b.total_stationary_scans} (k=${b.independent_intervals_k})</td><td class="number">within ±${(b.within_interval_repeatability_sd_m*1000).toFixed(1)} mm<br>between ±${(b.between_interval_sd_m*1000).toFixed(1)} mm</td><td class="number">planar ±${(b.lidar_planar_1sigma_m*1000).toFixed(1)} mm<br>w/ seg ±${(b.lidar_with_segmentation_1sigma_m*1000).toFixed(1)} mm<br><span class="coords">phys: unestablished</span></td><td>${escapeHtml(b.correspondence_confidence)}</td></tr>`;
      });
      diagHtml+=`</tbody></table>`;
    }else{
      const ivObj=stDiag.per_interval.find(x=>String(x.interval_id)===String(ivSel));
      if(ivObj){
        diagHtml+=`<div class="coords" style="margin:4px 0">Stationary Interval #${ivObj.interval_id}: scans ${escapeHtml(ivObj.scan_range)} (${ivObj.scan_count} scans, ${escapeHtml(ivObj.elapsed_range_s)})</div><table><thead><tr><th>Width</th><th>Physical</th><th>LiDAR Mean</th><th>Diff (cm / %)</th><th>Scans</th><th>Repeatability SD</th><th>Fit RMSE / Angle</th><th>Confidence</th></tr></thead><tbody>`;
        [["W1",3.117],["W2",3.272],["W3",3.137]].forEach(([k,realM])=>{
          const c=ivObj[k];
          if(!c){diagHtml+=`<tr><td><strong>${k}</strong></td><td class="number">${realM.toFixed(3)} m</td><td colspan="6" class="coords">Not simultaneously unobstructed in this stationary pose</td></tr>`;return}
          diagHtml+=`<tr><td><strong>${k}</strong></td><td class="number">${realM.toFixed(3)} m</td><td class="number">${c.mean_m.toFixed(4)} m</td><td class="number">${signed(c.diff_cm,2)} cm (${signed(c.diff_pct,2)}%)</td><td class="number">${c.n_scans}/${ivObj.scan_count}</td><td class="number">±${(c.std_m*1000).toFixed(1)} mm</td><td class="number">±${(c.fit_rmse_m*1000).toFixed(1)} mm / ${c.angle_diff_deg.toFixed(2)}°</td><td>${escapeHtml(c.confidence)}</td></tr>`;
        });
        diagHtml+=`</tbody></table>`;
      }
    }
  }
  document.getElementById("stationaryDiagnosticBox").innerHTML=diagHtml;
  document.getElementById("unassignedNotes").innerHTML=`<strong>Unassigned / Excluded Sketch Annotations</strong>`+(room.unassigned_or_uncertain_measurements||[]).map(u=>`<div>• <code>${escapeHtml(String(u.raw_value))}</code> (${escapeHtml(u.notation)}): ${escapeHtml(u.status)} — ${escapeHtml(u.notes)}</div>`).join("");
}
canvas.addEventListener("pointerdown",e=>{const p=eventPoint(e),w=viewport.clientWidth,h=viewport.clientHeight;if(mode==="pan"||e.button===1){drag={x:p.x,y:p.y,ox:offsetX,oy:offsetY};canvas.setPointerCapture(e.pointerId);return}if(e.button!==0||tab==="measurements")return;if(tab==="comparison"){const pw=hitPhysicalWall(p,w,h);if(pw){selectedPhysicalWall=pw;const wallObj=room.walls.find(x=>x.id===pw);if(wallObj?.width_id)document.getElementById("compTargetSelect").value=wallObj.width_id;else if(pw==="L1")document.getElementById("compTargetSelect").value="L1";render();return}const mw=hitMapWall(p,w,h);if(mw>=0){if(selectedMapWalls.includes(mw))selectedMapWalls=selectedMapWalls.filter(x=>x!==mw);else{selectedMapWalls.push(mw);if(selectedMapWalls.length>2)selectedMapWalls.shift()}applyMapWallSelection();render();return}return}const wall=hitWall(p);if(wall>=0){selectWall(wall);return}const point={world:screenToWorld(p.x,p.y)};if(!pending){pending=point;document.getElementById("summary").innerHTML='<span class="pending">Point A selected. Click point B.</span>'}else{measurements.push({id:nextId,name:`Measurement ${nextId}`,source:sourceName(),scan_stamp_ns:tab==="scan"?currentScan().stamp_ns:null,wall_ids:[],method:"point-to-point",a:pending,b:point,real:null});nextId++;pending=null;updateTable()}render()});
canvas.addEventListener("pointermove",e=>{const p=eventPoint(e),w=viewport.clientWidth,h=viewport.clientHeight;if(drag){offsetX=drag.ox+p.x-drag.x;offsetY=drag.oy+p.y-drag.y;render()}if(meta&&tab!=="measurements"){if(tab==="comparison"){if(p.x<=w/2){const q=screenToPhys(p.x,p.y,w,h);document.getElementById("status").textContent=`Physical room: x ${q.x.toFixed(3)} m, y ${q.y.toFixed(3)} m · selected ${selectedPhysicalWall}`}else{const q=compScreenToMapWorld(p.x,p.y,w,h);document.getElementById("status").textContent=`SLAM map: x ${q.x.toFixed(3)} m, y ${q.y.toFixed(3)} m · selected ${(selectedMapWalls.map(i=>data.map_walls[i].id).join(",")||"none")}`}}else{const m=screenToWorld(p.x,p.y);document.getElementById("status").textContent=`${tab==="map"?"Map":"LiDAR"}: x ${m.x.toFixed(3)} m, y ${m.y.toFixed(3)} m`}}});
canvas.addEventListener("pointerup",()=>drag=null);canvas.addEventListener("pointercancel",()=>drag=null);
canvas.addEventListener("wheel",e=>{e.preventDefault();const p=eventPoint(e),before=screenToWorld(p.x,p.y),factor=Math.exp(-e.deltaY*.0015);zoom=Math.max(.1,Math.min(1500,zoom*factor));const after=worldToScreen(before);offsetX+=p.x-after.x;offsetY+=p.y-after.y;render()},{passive:false});
function setMode(value){mode=value;document.getElementById("measureBtn").classList.toggle("active",value==="measure");document.getElementById("panBtn").classList.toggle("active",value==="pan");canvas.style.cursor=value==="pan"?"grab":"crosshair"}
function zoomAt(factor){const x=viewport.clientWidth/2,y=viewport.clientHeight/2,b=screenToWorld(x,y);zoom=Math.max(.1,Math.min(1500,zoom*factor));const after=worldToScreen(b);offsetX+=x-after.x;offsetY+=y-after.y;render()}
function updateTable(){const table=document.getElementById("measureTable"),body=table.querySelector("tbody"),empty=document.getElementById("empty");body.textContent="";table.hidden=!measurements.length;empty.hidden=!!measurements.length;measurements.forEach((m,i)=>{const d=distance(m.a,m.b),err=m.real==null?null:d-m.real,pct=err==null||!m.real?null:err/m.real*100,tr=document.createElement("tr");tr.innerHTML=`<td><input data-name="${m.id}" value="${escapeHtml(m.name)}"><div class="coords">${escapeHtml(m.source)}${m.wall_ids?.length?" · "+m.wall_ids.join(", "):""}<br>${escapeHtml(m.method||"point-to-point")}<br>A (${m.a.world.x.toFixed(3)}, ${m.a.world.y.toFixed(3)})<br>B (${m.b.world.x.toFixed(3)}, ${m.b.world.y.toFixed(3)})</div></td><td class="number">${d.toFixed(3)} m</td><td><input data-real="${m.id}" type="number" min="0" step="0.001" placeholder="meters" value="${m.real??""}"></td><td class="number">${err==null?"—":`absolute ${Math.abs(err).toFixed(3)} m<br>signed ${signed(err,3)} m<br>${pct==null?"—":signed(pct,2)+"%"}`}</td><td><button data-delete="${m.id}">×</button></td>`;body.appendChild(tr)});
  body.querySelectorAll("[data-name]").forEach(el=>el.oninput=()=>{const m=measurements.find(x=>x.id===+el.dataset.name);m.name=el.value;render()});
  body.querySelectorAll("[data-real]").forEach(el=>el.oninput=()=>{const m=measurements.find(x=>x.id===+el.dataset.real);m.real=el.value===""?null:+el.value;updateTable();render()});
  body.querySelectorAll("[data-delete]").forEach(el=>el.onclick=()=>{measurements=measurements.filter(x=>x.id!==+el.dataset.delete);updateTable();render()});
  document.getElementById("summary").textContent=measurements.length?`${measurements.length} measurement${measurements.length===1?"":"s"} · scan and SLAM-map results are kept separately`:"Click two points to add a measurement.";
}
function escapeHtml(v){return v.replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]))}
function signed(v,n){return`${v>=0?"+":""}${v.toFixed(n)}`}
function csvCell(v){return`"${String(v??"").replaceAll('"','""')}"`}
function exportCSV(){const rows=[["name","source","scan_stamp_ns","wall_ids","method","point_a_x_m","point_a_y_m","point_b_x_m","point_b_y_m","measured_distance_m","real_distance_m","absolute_error_m","signed_difference_m","percentage_error"]];measurements.forEach(m=>{const d=distance(m.a,m.b),e=m.real==null?null:d-m.real,p=e==null||!m.real?null:e/m.real*100;rows.push([m.name,m.source,m.scan_stamp_ns,m.wall_ids?.join("|"),m.method,m.a.world.x,m.a.world.y,m.b.world.x,m.b.world.y,d,m.real,e==null?null:Math.abs(e),e,p])});download(new Blob([rows.map(r=>r.map(csvCell).join(",")).join("\n")+"\n"],{type:"text/csv"}),"oct1_lidar_measurements.csv")}
function exportFloorPlanPNG(){
  if(!room)return;const out=document.createElement("canvas");out.width=720;out.height=1080;const c=out.getContext("2d");c.fillStyle="#0b1220";c.fillRect(0,0,out.width,out.height);
  const s=82,ox=220,oy=980,toP=(x,y)=>({x:ox+x*s,y:oy-y*s});
  c.strokeStyle="#1e293b";c.lineWidth=1;for(let x=-1;x<=5;x++){const a=toP(x,-0.5),b=toP(x,11);c.beginPath();c.moveTo(a.x,a.y);c.lineTo(b.x,b.y);c.stroke()}for(let y=0;y<=11;y++){const a=toP(-1,y),b=toP(5,y);c.beginPath();c.moveTo(a.x,a.y);c.lineTo(b.x,b.y);c.stroke()}
  c.fillStyle="#f8fafc";c.font="bold 16px system-ui";c.fillText("Physical Reference Floor Plan (Provisional m, Equal Metric Scale)",24,32);
  c.fillStyle="#fbbf24";c.font="13px system-ui";c.fillText("Discrepancy Flagged: L1 = 10.600 m vs R1+R2+R3 = 10.322 m (Δ = 0.278 m unresolved)",24,54);
  room.walls.forEach(w=>{const a=toP(w.x_start_m,w.y_start_m),b=toP(w.x_end_m,w.y_end_m),gap=w.status==="unresolved_discrepancy";c.save();if(gap)c.setLineDash([7,5]);c.strokeStyle=gap?"#f59e0b":(w.side==="left"?"#38bdf8":"#4ade80");c.lineWidth=4;c.beginPath();c.moveTo(a.x,a.y);c.lineTo(b.x,b.y);c.stroke();c.fillStyle=c.strokeStyle;c.font="bold 13px system-ui";c.fillText(`${w.id}: ${w.length_m.toFixed(3)} m`,(a.x+b.x)/2+(w.side==="left"?-120:12),(a.y+b.y)/2);c.restore()});
  room.widths.forEach(w=>{const a=toP(0,w.y_display_m),b=toP(w.measured_m,w.y_display_m);c.save();c.strokeStyle="#f43f5e";c.fillStyle="#f43f5e";c.lineWidth=2;c.setLineDash([5,4]);c.beginPath();c.moveTo(a.x,a.y);c.lineTo(b.x,b.y);c.stroke();c.setLineDash([]);c.font="bold 13px system-ui";c.fillText(`${w.id} = ${w.measured_m.toFixed(3)} m`,(a.x+b.x)/2-45,a.y-8);c.restore()});
  out.toBlob(blob=>download(blob,"physical_reference_floor_plan.png"),"image/png");
}
function exportComparisonCSV(){
  const rows=[["reference","reference_id","real_distance_m","lidar_scan_distance_m","lidar_scan_source","slam_map_distance_m","slam_map_source","scan_signed_error_m","scan_abs_error_m","scan_pct_error","map_signed_error_m","map_abs_error_m","map_pct_error"]];
  comparisonRows.forEach(r=>{const sd=r.lidar_scan_distance_m==null?null:r.lidar_scan_distance_m-r.real_distance_m,md=r.slam_map_distance_m==null?null:r.slam_map_distance_m-r.real_distance_m;rows.push([r.reference,r.reference_id,r.real_distance_m,r.lidar_scan_distance_m,r.lidar_scan_source,r.slam_map_distance_m,r.slam_map_source,sd,sd==null?null:Math.abs(sd),sd==null?null:sd/r.real_distance_m*100,md,md==null?null:Math.abs(md),md==null?null:md/r.real_distance_m*100])});
  download(new Blob([rows.map(r=>r.map(csvCell).join(",")).join("\n")+"\n"],{type:"text/csv"}),"physical_vs_lidar_comparison.csv");
}
function exportValidationMarkdown(){
  const lines=["# Physical Room vs. LiDAR & SLAM Validation Report (October 1, 2026)","","- **Discrepancy flagged:** `L1 = 10.600 m` vs `R1+R2+R3 = 10.322 m` (`Δ = 0.278 m`).","","| Reference | Real distance | LiDAR scan | SLAM map | Error |","|---|---:|---:|---:|---|"];
  comparisonRows.forEach(r=>{const sd=r.lidar_scan_distance_m==null?null:r.lidar_scan_distance_m-r.real_distance_m,md=r.slam_map_distance_m==null?null:r.slam_map_distance_m-r.real_distance_m,errs=[];if(sd!=null)errs.push(`Scan: ${signed(sd,4)} m (${signed(sd/r.real_distance_m*100,2)}%)`);if(md!=null)errs.push(`Map: ${signed(md,4)} m (${signed(md/r.real_distance_m*100,2)}%)`);lines.push(`| ${r.reference} | ${r.real_distance_m.toFixed(3)} m | ${r.lidar_scan_distance_m==null?"Unconfirmed":r.lidar_scan_distance_m.toFixed(4)+" m"} | ${r.slam_map_distance_m==null?"To measure":r.slam_map_distance_m.toFixed(4)+" m"} | ${errs.join("; ")||"Identify features"} |`)});
  download(new Blob([lines.join("\n")+"\n"],{type:"text/markdown"}),"PHYSICAL_ROOM_VALIDATION_REPORT.md");
}
function exportPNG(){if(tab==="measurements")return;canvas.toBlob(blob=>download(blob,tab==="scan"?"oct1_lidar_scan_annotated.png":(tab==="comparison"?"reference_vs_lidar_comparison.png":"oct1_room_map_annotated.png")),"image/png")}
function download(blob,name){const a=document.createElement("a");a.href=URL.createObjectURL(blob);a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)}
document.getElementById("measureBtn").onclick=()=>setMode("measure");document.getElementById("panBtn").onclick=()=>setMode("pan");document.getElementById("zoomIn").onclick=()=>zoomAt(1.3);document.getElementById("zoomOut").onclick=()=>zoomAt(1/1.3);document.getElementById("fitBtn").onclick=fit;document.getElementById("clearBtn").onclick=()=>{measurements=[];selectedWalls=[];selectedMapWalls=[];pending=null;nextId=1;updateTable();render()};document.getElementById("csvBtn").onclick=exportCSV;document.getElementById("pngBtn").onclick=exportPNG;
document.getElementById("exportFloorPlanBtn").onclick=exportFloorPlanPNG;document.getElementById("exportCompPngBtn").onclick=exportPNG;document.getElementById("exportCompCsvBtn").onclick=exportComparisonCSV;document.getElementById("exportReportMdBtn").onclick=exportValidationMarkdown;
["overlayToggle","ovTx","ovTy","ovYaw","compTargetSelect","validatedGeometryToggle","compValidatedToggle"].forEach(id=>document.getElementById(id).oninput=()=>render());
document.getElementById("compIntervalSelect").onchange=e=>{const v=e.target.value;if(v!=="all"){const iv=data.physical_comparison?.stationary_diagnostics?.per_interval?.find(x=>String(x.interval_id)===String(v));if(iv)scanIndex=iv.mid_index}updateComparisonTable();render()};
function updateScanInfo(){const s=currentScan();selectedWalls=[];document.getElementById("mapMeta").textContent=`Actual October 1 scan ${s.source_index+1}/${data.scan_count_unique} · +${s.elapsed_s.toFixed(3)} s · ${s.valid_count}/${s.bin_count} valid bins`;document.getElementById("scanClock").textContent=`Recording +${s.elapsed_s.toFixed(3)} s · stamp ${s.stamp_ns}`;document.getElementById("scanNumber").textContent=`Scan ${s.source_index+1} / ${data.scan_count_unique}`;document.getElementById("timelineSlider").value=scanIndex;document.getElementById("wallList").innerHTML=scanKind==="corrected"?`<strong>Detected straight segments — click colored lines</strong>${s.walls.map((w,i)=>`<div style="color:${colors[i%colors.length]}">W${i+1}: ${w.sensor_distance_m.toFixed(3)} m from sensor · ${(wallAngle(w)*180/Math.PI).toFixed(1)}° · RMSE ${w.rmse_m.toFixed(3)} m</div>`).join("")}`:"Raw view mirrors Y; wall fits are shown only for corrected geometry.";document.getElementById("warning").className="warning good";render()}
function goToScan(index,refit=false){scanIndex=Math.max(0,Math.min(data.scans.length-1,index));pending=null;updateScanInfo();if(refit)fit()}
function pause(){if(playTimer){clearTimeout(playTimer);playTimer=null}document.getElementById("playBtn").classList.remove("active")}
function scheduleNext(){if(!playTimer)return;if(scanIndex>=data.scans.length-1){pause();return}const current=currentScan(),next=data.scans[scanIndex+1],delay=Math.max(1,(next.elapsed_s-current.elapsed_s)*1000/+document.getElementById("speedSelect").value);playTimer=setTimeout(()=>{goToScan(scanIndex+1);scheduleNext()},delay)}
function play(){if(playTimer)return;if(scanIndex>=data.scans.length-1)goToScan(0);document.getElementById("playBtn").classList.add("active");playTimer=setTimeout(()=>{goToScan(scanIndex+1);scheduleNext()},Math.max(1,(data.scans[Math.min(scanIndex+1,data.scans.length-1)].elapsed_s-currentScan().elapsed_s)*1000/+document.getElementById("speedSelect").value))}
function setTab(next){tab=next;pending=null;if(tab!=="scan")pause();document.querySelectorAll("[data-tab]").forEach(b=>b.classList.toggle("active",b.dataset.tab===tab));const measurement=tab==="measurements",scan=tab==="scan",comp=tab==="comparison";canvas.style.display=measurement?"none":"block";document.getElementById("measurementsIntro").style.display=measurement?"block":"none";document.getElementById("help").style.display=measurement?"none":"block";document.getElementById("status").style.display=measurement?"none":"block";["playbackControls","speedLabel","accumulationLabel","recommendedLabel","validatedGeometryLabel","rawBtn","correctedBtn","timeline","wallList"].forEach(id=>document.getElementById(id).style.display=scan?"":"none");document.getElementById("comparisonControls").style.display=comp?"flex":"none";document.getElementById("comparisonPanel").style.display=comp?"block":"none";document.getElementById("measureTableWrap").style.display=comp?"none":"block";document.getElementById("stationaryLabel").style.display=scan&&document.getElementById("accumulationSelect").value==="stationary"?"flex":"none";if(scan)updateScanInfo();else if(tab==="map"){document.getElementById("mapMeta").textContent=`${meta.name} · ${meta.width}×${meta.height} · ${meta.resolution} m/pixel · origin [${meta.origin.map(v=>v.toFixed(2)).join(", ")}]`;document.getElementById("warning").className="warning";document.getElementById("warning").textContent="UNRELIABLE GEOMETRY: the original PGM contains the diagonal/overlapping walls shown here. The orange ICP trajectory is diagnostic and accumulated without loop closure."}else if(comp){document.getElementById("mapMeta").textContent=`Physical room comparison · L1=10.600 m · W1=3.117 m, W2=3.272 m, W3=3.137 m · Δlength=0.278 m flagged`;document.getElementById("warning").className="warning";document.getElementById("warning").textContent="WARNING: SLAM map has duplicated diagonal walls; click a physical wall on the LEFT and two opposing map walls (M1..M18) on the RIGHT to assign a SLAM width correspondence. Rigid overlay uses fixed scale = 1.0.";updateComparisonTable()}else{document.getElementById("mapMeta").textContent="Physical comparison · scan and SLAM-map measurements remain labelled separately";document.getElementById("warning").className="warning";document.getElementById("warning").textContent="Do not interpret SLAM-map error as LiDAR range error; ICP and SLAM drift are included."}if(!measurement)fit();updateTable()}
document.querySelectorAll("[data-tab]").forEach(b=>b.onclick=()=>setTab(b.dataset.tab));document.getElementById("rawBtn").onclick=()=>{scanKind="raw";selectedWalls=[];document.getElementById("rawBtn").classList.add("active");document.getElementById("correctedBtn").classList.remove("active");updateScanInfo();fit()};document.getElementById("correctedBtn").onclick=()=>{scanKind="corrected";document.getElementById("correctedBtn").classList.add("active");document.getElementById("rawBtn").classList.remove("active");updateScanInfo();fit()};document.getElementById("playBtn").onclick=play;document.getElementById("pauseBtn").onclick=pause;document.getElementById("prevBtn").onclick=()=>{pause();goToScan(scanIndex-1)};document.getElementById("nextBtn").onclick=()=>{pause();goToScan(scanIndex+1)};document.getElementById("timelineSlider").oninput=e=>{pause();goToScan(+e.target.value)};document.getElementById("accumulationSelect").onchange=e=>{const stationary=e.target.value==="stationary";document.getElementById("stationaryLabel").hidden=!stationary;document.getElementById("stationaryLabel").style.display=stationary?"flex":"none";if(stationary&&data.stationary_intervals.length){const interval=data.stationary_intervals[+document.getElementById("stationarySelect").value];goToScan(Math.floor((interval.start_index+interval.end_index)/2),true)}else render()};document.getElementById("stationarySelect").onchange=e=>{const interval=data.stationary_intervals[+e.target.value];goToScan(Math.floor((interval.start_index+interval.end_index)/2),true)};document.getElementById("recommendedSelect").onchange=e=>{if(e.target.value!==""){pause();goToScan(+e.target.value,true)}};
Promise.all([fetch("/metadata.json").then(r=>r.ok?r.json():Promise.reject(Error(r.statusText))),fetch("/scans.json").then(r=>r.ok?r.json():Promise.reject(Error(r.statusText))),fetch("/room_measurements.json").then(r=>r.ok?r.json():Promise.reject(Error(r.statusText))),new Promise((resolve,reject)=>{image.onload=resolve;image.onerror=reject;image.src="/map.png"})]).then(([m,d,rm])=>{meta=m;data=d;room=rm;comparisonRows=(d.physical_comparison?.default_rows||[]).map(r=>({...r}));document.getElementById("timelineSlider").max=d.scans.length-1;const intervals=document.getElementById("stationarySelect"),compIv=document.getElementById("compIntervalSelect");d.stationary_intervals.forEach((s,i)=>{const lbl=`#${i+1}: scans ${s.start_index+1}–${s.end_index+1} (${s.scan_count}, ${s.start_elapsed_s.toFixed(1)}–${s.end_elapsed_s.toFixed(1)} s)`;intervals.add(new Option(lbl,i));compIv.add(new Option(lbl,i+1))});const recommended=document.getElementById("recommendedSelect");d.recommended_scans.forEach(s=>recommended.add(new Option(`Scan ${s.scan_index+1} · +${s.elapsed_s.toFixed(1)} s — ${s.reason}`,s.scan_index)));(d.stepped_scan_candidates||[]).slice(0,3).forEach(s=>recommended.add(new Option(`Stepped W1/W2/W3 Scan ${s.scan_index+1} · +${s.elapsed_s.toFixed(1)} s`,s.scan_index)));document.getElementById("status").textContent="Ready · equal metric axes";resize();setTab("scan")}).catch(e=>document.getElementById("status").textContent=`Load failed: ${e}`);
new ResizeObserver(resize).observe(viewport);
</script>
</body></html>
"""


def create_server(
    model: MapModel,
    bind: str,
    port: int,
    scan_data: bytes = b'{"scans":[],"trajectory":{"poses":[]}}',
    room_measurements: dict[str, Any] | None = None,
) -> ThreadingHTTPServer:
    compressed_scan_data = gzip.compress(scan_data, compresslevel=6)
    room_payload = json.dumps(
        room_measurements
        or load_room_measurements(Path(DEFAULT_ROOM_MEASUREMENTS)),
        separators=(",", ":"),
    ).encode()

    class Handler(BaseHTTPRequestHandler):
        server_version = "HorseShitBotMapViewer/1.0"

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", HTML.encode())
            elif path == "/map.png":
                self._send(200, "image/png", model.png, cache="public, max-age=3600")
            elif path == "/metadata.json":
                payload = json.dumps(model.metadata(), separators=(",", ":")).encode()
                self._send(200, "application/json", payload)
            elif path == "/room_measurements.json":
                self._send(200, "application/json", room_payload)
            elif path == "/scans.json":
                if "gzip" in self.headers.get("Accept-Encoding", ""):
                    self._send(
                        200,
                        "application/json",
                        compressed_scan_data,
                        content_encoding="gzip",
                    )
                else:
                    self._send(200, "application/json", scan_data)
            elif path == "/health":
                self._send(200, "application/json", b'{"status":"ok"}')
            else:
                self._send(404, "text/plain; charset=utf-8", b"Not found\n")

        def _send(
            self,
            status: int,
            content_type: str,
            payload: bytes,
            cache: str = "no-store",
            content_encoding: str | None = None,
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", cache)
            if content_encoding:
                self.send_header("Content-Encoding", content_encoding)
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self' 'unsafe-inline'")
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, format_string: str, *args: Any) -> None:
            print(f"{self.address_string()} - {format_string % args}")

    return ThreadingHTTPServer((bind, port), Handler)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default=DEFAULT_MAP, help="ROS map_server YAML file")
    parser.add_argument(
        "--scans",
        default=DEFAULT_SCANS,
        help="Representative October 1 scan JSON from extract_oct1_lidar_web_data.py",
    )
    parser.add_argument(
        "--room-measurements",
        default=DEFAULT_ROOM_MEASUREMENTS,
        help="Physical room measurements YAML file",
    )
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = MapModel.load(Path(args.map))
    scans_path = Path(args.scans).expanduser().resolve()
    if not scans_path.is_file():
        raise FileNotFoundError(
            f"Scan dataset does not exist: {scans_path}. "
            "Run scripts/extract_oct1_lidar_web_data.py in a sourced ROS 2 shell."
        )
    scan_data = scans_path.read_bytes()
    parsed_scans = json.loads(scan_data)
    if not parsed_scans.get("scans"):
        raise ValueError(f"Scan dataset contains no representative scans: {scans_path}")
    room_measurements = load_room_measurements(Path(args.room_measurements))
    server = create_server(model, args.bind, args.port, scan_data, room_measurements)
    print(
        f"Loaded {model.width}x{model.height} map at {model.resolution:g} m/pixel "
        f"from {model.yaml_path}"
    )
    print(
        f"Loaded {len(parsed_scans['scans'])} representative scans from {scans_path}"
    )
    print(f"Open http://localhost:{server.server_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping map viewer.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
