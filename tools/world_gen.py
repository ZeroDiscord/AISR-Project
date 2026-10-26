"""Generate the three tomb worlds, their textures and ground-truth maps.

    python tools/world_gen.py            # all worlds
    python tools/world_gen.py treasury   # one world

Outputs
    worlds/<name>.wbt               Webots world
    worlds/textures/*.png           procedural textures
    ground_truth/<name>.npz         ground-truth grids (see ground_truth.py)
    ground_truth/<name>.png         preview of the true map with the start pose
"""

import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

import _paths
from layouts import Box, Cyl, LAYOUTS, WALL_H, WALL_T, get_layout
import ground_truth
import textures
from tomb import config

ROOT = _paths.ROOT
WORLDS = os.path.join(ROOT, "worlds")
GT_DIR = os.path.join(ROOT, "ground_truth")

# Full URLs rather than webots:// so the worlds load on any R2025a install:
# some installs ship without local PROTO files, and Webots caches downloads.
PROTO_BASE = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/"
EXTERNPROTOS = [PROTO_BASE + p for p in (
    "objects/backgrounds/protos/TexturedBackground.proto",
    "objects/backgrounds/protos/TexturedBackgroundLight.proto",
    "appearances/protos/Sand.proto",
    "robots/adept/pioneer3/protos/Pioneer3dx.proto",
    "devices/sick/protos/SickLms291.proto",
)]


# ------------------------------------------------------------- appearances
def textured(png, su=1.0, sv=1.0):
    return (f'PBRAppearance {{ baseColorMap ImageTexture {{ url [ "textures/{png}" ] }} '
            f'roughness 0.95 metalness 0 '
            f'textureTransform TextureTransform {{ scale {su:.3f} {sv:.3f} }} }}')


def plain(r, g, b, rough, metal=0.0):
    return f'PBRAppearance {{ baseColor {r} {g} {b} roughness {rough} metalness {metal} }}'


LOOKS = {
    "obsidian": plain(0.07, 0.07, 0.09, 0.3, 0.3),
    # not fully metallic: a pure metal only mirrors the dark tomb sky and looks black
    "gold": plain(1.0, 0.72, 0.22, 0.35, 0.6),
    "clay": plain(0.6, 0.3, 0.15, 0.75),
    "painted": plain(0.16, 0.3, 0.6, 0.5),
    "wood": plain(0.42, 0.25, 0.12, 0.8),
    "handle": plain(0.3, 0.18, 0.08, 0.9),
    "flame": ('PBRAppearance { baseColor 1 0.6 0.2 emissiveColor 1 0.45 0.1 '
              'emissiveIntensity 6 roughness 1 metalness 0 }'),
}


def appearance(look, ob=None):
    if look in LOOKS:
        return LOOKS[look]
    if look == "door":
        return textured("door.png")
    png = "hieroglyphs.png" if look == "glyphs" else "sandstone.png"
    if isinstance(ob, Box):
        return textured(png, max(ob.sx, ob.sy) / ob.h, 1.0)
    if isinstance(ob, Cyl):
        return textured(png, 2 * math.pi * ob.r / ob.h, 1.0)
    return textured(png)


def shape(geom, look, at=(0, 0, 0)):
    return (f'Pose {{ translation {at[0]:.3f} {at[1]:.3f} {at[2]:.3f} children [ '
            f'Shape {{ appearance {appearance(look)} geometry {geom} }} ] }}')


# -------------------------------------------------------------- decoration
# Decor is placed relative to the obstacle centre (z from -h/2 to +h/2) and is
# kept above the lidar plane or inside the footprint, so it never changes
# what the lidar sees.
def decor(ob):
    h2 = ob.h / 2
    k = ob.kind
    if k == "pillar":
        return [shape(f"Cylinder {{ radius {ob.r * 1.25:.3f} height 0.12 }}", "stone", (0, 0, h2 - 0.06)),
                shape(f"Cylinder {{ radius {ob.r * 1.02:.3f} height 0.08 }}", "gold", (0, 0, 0.4))]
    if k == "urn":
        return [shape(f"Cylinder {{ radius {ob.r * 0.55:.3f} height 0.14 }}", "clay", (0, 0, h2 + 0.07)),
                shape(f"Cylinder {{ radius {ob.r * 0.7:.3f} height 0.03 }}", "gold", (0, 0, h2 + 0.15))]
    if k == "statue":
        return [shape("Sphere { radius 0.2 subdivision 3 }", ob.look, (0, 0, h2 + 0.2)),
                shape("Box { size 0.3 0.5 0.28 }", "gold", (-0.05, 0, h2 + 0.26)),
                shape("Box { size 0.62 0.62 0.06 }", "gold", (0, 0, h2 - 0.6))]
    if k == "sarcophagus":
        lid = "gold" if ob.look != "gold" else "painted"
        return [shape(f"Box {{ size {ob.sx + 0.04:.3f} {ob.sy + 0.04:.3f} 0.08 }}", lid, (0, 0, h2 + 0.04)),
                shape("Sphere { radius 0.22 subdivision 3 }", "gold", (ob.sx / 2 - 0.3, 0, h2 + 0.1))]
    if k == "chest":
        bands = [shape(f"Box {{ size 0.06 {ob.sy + 0.02:.3f} 0.25 }}", "gold", (dx, 0, h2 - 0.125))
                 for dx in (-ob.sx / 3, ob.sx / 3)]
        return bands + [shape(f"Box {{ size {ob.sx + 0.03:.3f} {ob.sy + 0.03:.3f} 0.05 }}", "wood",
                              (0, 0, h2 + 0.025))]
    if k == "coins":
        heap = [shape(f"Cone {{ bottomRadius {ob.r * 0.95:.3f} height 0.3 }}", "gold", (0, 0, h2 + 0.15))]
        rng = np.random.default_rng(int(abs(ob.x * 100 + ob.y * 10)))
        for _ in range(6):
            a, d = rng.uniform(0, 2 * math.pi), rng.uniform(0.1, ob.r * 0.9)
            heap.append(shape("Cylinder { radius 0.05 height 0.012 }", "gold",
                              (d * math.cos(a), d * math.sin(a), h2 + 0.006)))
        return heap
    if k == "altar":
        return [shape("Cylinder { radius 0.08 height 0.2 }", "gold", (0, 0, h2 + 0.1)),
                shape("Sphere { radius 0.1 subdivision 3 }", "gold", (0, 0, h2 + 0.28))]
    if k == "mask":
        return ['Transform { translation 0 0 %.3f scale 1 0.8 1.3 children [ Shape { appearance %s '
                'geometry Sphere { radius 0.16 subdivision 3 } } ] }' % (h2 + 0.2, LOOKS["gold"])]
    return []


def solid(ob, idx):
    if isinstance(ob, Box):
        geom = f"Box {{ size {ob.sx:.3f} {ob.sy:.3f} {ob.h:.3f} }}"
        rot = ob.yaw
    else:
        geom = f"Cylinder {{ radius {ob.r:.3f} height {ob.h:.3f} subdivision 24 }}"
        rot = 0.0
    children = [f"Shape {{ appearance {appearance(ob.look, ob)} geometry DEF OB{idx} {geom} }}"]
    children += decor(ob)
    return (f"Solid {{\n  translation {ob.x:.3f} {ob.y:.3f} {ob.h / 2:.3f}\n"
            f"  rotation 0 0 1 {rot:.4f}\n"
            f"  children [\n    " + "\n    ".join(children) + "\n  ]\n"
            f'  name "{ob.kind}_{idx}"\n  boundingObject USE OB{idx}\n  locked TRUE\n}}')


def torch(t, idx):
    c, s = math.cos(t.facing), math.sin(t.facing)
    x, y = t.x + 0.08 * c, t.y + 0.08 * s
    flame = (x + 0.04 * c, y + 0.04 * s, 1.22)
    return "\n".join([
        f'Pose {{ translation {x:.3f} {y:.3f} 1.0 rotation {-s:.4f} {c:.4f} 0 0.35 children [',
        f'  Shape {{ appearance {LOOKS["handle"]} geometry Cylinder {{ radius 0.03 height 0.3 }} }}',
        f'  {shape("Sphere { radius 0.07 subdivision 2 }", "flame", (0, 0, 0.2))}',
        "] }",
        f"PointLight {{ attenuation 0 0 1 color 1 0.62 0.3 intensity 2.2 "
        f"location {flame[0]:.3f} {flame[1]:.3f} {flame[2]:.3f} radius 7 castShadows FALSE }}",
    ])


def robot(layout):
    x, y, yaw = layout.start
    return f"""DEF SCARAB Pioneer3dx {{
  translation {x:.3f} {y:.3f} 0.0975
  rotation 0 0 1 {yaw:.4f}
  name "Scarab"
  controller "scarab"
  controllerArgs [ "--world={layout.name}" "--strategy=utility" ]
  supervisor TRUE
  extensionSlot [
    SickLms291 {{
      translation {config.LIDAR_OFFSET_X} 0 0.21
    }}
    GPS {{
      name "gps"
    }}
    InertialUnit {{
      name "imu"
    }}
    Display {{
      name "map"
      width 256
      height 256
    }}
    Transform {{
      translation -0.12 0 0.16
      scale 1.4 1 0.45
      children [
        Shape {{ appearance {LOOKS["gold"]} geometry Sphere {{ radius 0.11 subdivision 3 }} }}
      ]
    }}
    Pose {{
      translation -0.12 0 0.21
      children [
        Shape {{ appearance {LOOKS["obsidian"]} geometry Box {{ size 0.3 0.012 0.012 }} }}
      ]
    }}
  ]
}}"""


def world_text(layout):
    w, h = layout.inner_w, layout.inner_h
    fw, fh = w + 2 * WALL_T + 1.0, h + 2 * WALL_T + 1.0
    view = max(w, h) * 1.75
    pitch = 1.05
    parts = ["#VRML_SIM R2025a utf8", ""]
    parts += [f'EXTERNPROTO "{p}"' for p in EXTERNPROTOS]
    parts.append(f"""
WorldInfo {{
  info [
    "{layout.story}"
    "Project P12 - Occupancy-Grid Mapping and Frontier Exploration"
    "Generated by tools/world_gen.py - edit tools/layouts.py, not this file."
  ]
  title "{layout.title}"
  basicTimeStep 32
}}
Viewpoint {{
  orientation 0 1 0 {pitch}
  position {-view * math.cos(pitch):.3f} 0 {view * math.sin(pitch):.3f}
}}
TexturedBackground {{
  texture "dusk"
  luminosity 0.6
  skybox FALSE
  skyColor [ 0.05 0.03 0.02 ]
}}
TexturedBackgroundLight {{
  texture "dusk"
  luminosity 0.45
  castShadows FALSE
}}
Solid {{
  translation 0 0 -0.05
  children [
    Shape {{
      appearance Sand {{ textureTransform TextureTransform {{ scale {fw / 2:.2f} {fh / 2:.2f} }} }}
      geometry DEF FLOOR Box {{ size {fw:.2f} {fh:.2f} 0.1 }}
    }}
  ]
  name "floor"
  boundingObject USE FLOOR
  locked TRUE
}}""")
    parts += [solid(ob, i) for i, ob in enumerate(layout.obstacles)]
    parts += [torch(t, i) for i, t in enumerate(layout.torches)]
    parts.append(robot(layout))
    return "\n".join(parts) + "\n"


# ------------------------------------------------------------ previews
PAPYRUS = (232, 212, 165)
INK = (55, 35, 15)
STONE = (30, 24, 20)


def gt_preview(gt, path, scale=3):
    occ, reach = gt["occ"], gt["reachable"]
    img = np.empty(occ.shape + (3,), dtype=np.uint8)
    img[:] = STONE
    img[reach] = PAPYRUS
    img[occ] = INK
    rows = np.where(occ.any(axis=1) | reach.any(axis=1))[0]
    cols = np.where(occ.any(axis=0) | reach.any(axis=0))[0]
    img = img[rows[0]:rows[-1] + 1, cols[0]:cols[-1] + 1][::-1]
    pil = Image.fromarray(img).resize((img.shape[1] * scale, img.shape[0] * scale), Image.NEAREST)
    d = ImageDraw.Draw(pil)
    for i, (x, y, yaw) in enumerate(gt["starts"]):
        px = ((x - config.MAP_ORIGIN[0]) / config.MAP_RES - cols[0]) * scale
        py = (rows[-1] - (y - config.MAP_ORIGIN[1]) / config.MAP_RES) * scale
        r = 0.26 / config.MAP_RES * scale
        d.ellipse([px - r, py - r, px + r, py + r], fill=(222, 175, 60))
        d.line([px, py, px + 2 * r * math.cos(yaw), py - 2 * r * math.sin(yaw)], fill=(150, 50, 30), width=4)
        d.text((px - 3, py - 6), str(i), fill=(55, 35, 15))
    pil.save(path)


def main(names):
    os.makedirs(WORLDS, exist_ok=True)
    os.makedirs(GT_DIR, exist_ok=True)
    textures.write_all(os.path.join(WORLDS, "textures"))
    for name in names:
        layout = get_layout(name)
        with open(os.path.join(WORLDS, f"{name}.wbt"), "w", encoding="utf-8") as f:
            f.write(world_text(layout))
        gt = ground_truth.build(layout)
        np.savez_compressed(os.path.join(GT_DIR, f"{name}.npz"), **gt)
        gt_preview(gt, os.path.join(GT_DIR, f"{name}.png"))
        print(f"{name:12s} obstacles={len(layout.obstacles):3d} torches={len(layout.torches):2d} "
              f"explorable={gt['reachable'].sum() * config.MAP_RES ** 2:6.1f} m^2")


if __name__ == "__main__":
    main(sys.argv[1:] or list(LAYOUTS))
