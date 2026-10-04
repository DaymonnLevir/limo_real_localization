#!/usr/bin/env python3

"""Generate precise tree ground-truth CSV/SVG from a Gazebo world file."""

import argparse
import csv
import math
import os
import re
import sys
import xml.etree.ElementTree as ET


def parse_tree_id(name):
    m = re.search(r"(\d+)$", name or "")
    if not m:
        return None
    try:
        return int(m.group(1))
    except ValueError:
        return None


def parse_model_name(uri):
    text = (uri or "").strip()
    if not text.startswith("model://"):
        return ""
    rest = text[len("model://") :]
    if not rest:
        return ""
    return rest.split("/", 1)[0].strip()


def parse_scale(scale_text):
    vals = (scale_text or "").strip().split()
    if len(vals) >= 3:
        try:
            return float(vals[0]), float(vals[1]), float(vals[2])
        except Exception:
            pass
    return 1.0, 1.0, 1.0


def resolve_mesh_path(model_root, default_model_name, mesh_uri):
    uri = (mesh_uri or "").strip()
    if not uri:
        return ""

    if uri.startswith("file://"):
        return os.path.abspath(uri[len("file://") :])

    if uri.startswith("model://"):
        rest = uri[len("model://") :]
        if "/" not in rest:
            return ""
        model_name, rel_path = rest.split("/", 1)
        return os.path.abspath(os.path.join(model_root, model_name, rel_path))

    return os.path.abspath(os.path.join(model_root, default_model_name, uri))


def estimate_model_offset_xy(model_root, model_name, ground_band_z):
    """
    Estimate local XY offset (model origin -> trunk center) from OBJ mesh vertices.

    The offset is estimated by averaging vertices close to the lowest Z slice
    (ground contact ring), then later rotated by include yaw in world file.
    """
    model_dir = os.path.join(model_root, model_name)
    model_sdf = os.path.join(model_dir, "model.sdf")
    if not os.path.exists(model_sdf):
        return 0.0, 0.0, "no_model_sdf"

    try:
        root = ET.parse(model_sdf).getroot()
    except Exception:
        return 0.0, 0.0, "invalid_model_sdf"

    mesh_uri = ""
    scale_xyz = (1.0, 1.0, 1.0)
    for mesh in root.findall(".//mesh"):
        uri_text = mesh.findtext("uri")
        if not (uri_text and uri_text.strip()):
            continue
        mesh_uri = uri_text.strip()
        scale_xyz = parse_scale(mesh.findtext("scale"))
        break

    if not mesh_uri:
        return 0.0, 0.0, "no_mesh_uri"

    mesh_path = resolve_mesh_path(model_root, model_name, mesh_uri)
    if not mesh_path or (not os.path.exists(mesh_path)):
        return 0.0, 0.0, "mesh_not_found"
    if not mesh_path.lower().endswith(".obj"):
        return 0.0, 0.0, "mesh_not_obj"

    sx, sy, sz = scale_xyz
    verts = []
    try:
        with open(mesh_path, "r", errors="ignore") as f:
            for line in f:
                if not line.startswith("v "):
                    continue
                parts = line.split()
                if len(parts) < 4:
                    continue
                x = float(parts[1]) * sx
                y = float(parts[2]) * sy
                z = float(parts[3]) * sz
                verts.append((x, y, z))
    except Exception:
        return 0.0, 0.0, "mesh_read_failed"

    if not verts:
        return 0.0, 0.0, "mesh_has_no_vertices"

    min_z = min(v[2] for v in verts)
    band_max_z = min_z + max(float(ground_band_z), 1e-3)
    base = [(x, y) for (x, y, z) in verts if z <= band_max_z]
    if not base:
        base = [(x, y) for (x, y, _z) in verts]

    ox = sum(p[0] for p in base) / float(len(base))
    oy = sum(p[1] for p in base) / float(len(base))
    return ox, oy, "ok"


def parse_world_trees(world_path, uri_filter, apply_mesh_offset, model_root, ground_band_z):
    tree = ET.parse(world_path)
    root = tree.getroot()

    world = root.find("world")
    if world is None:
        raise RuntimeError("No <world> element found in {}".format(world_path))

    model_offset_cache = {}
    trees = []

    entries = []
    for inc in world.findall("include"):
        entries.append(
            {
                "name": (inc.findtext("name") or "").strip(),
                "pose_txt": (inc.findtext("pose") or "").strip(),
                "uri": (inc.findtext("uri") or "").strip(),
            }
        )

    for model in world.findall("model"):
        uri = ""
        for mesh in model.findall(".//mesh"):
            uri_txt = (mesh.findtext("uri") or "").strip()
            if uri_txt:
                uri = uri_txt
                break
        entries.append(
            {
                "name": (model.get("name") or "").strip(),
                "pose_txt": (model.findtext("pose") or "").strip(),
                "uri": uri,
            }
        )

    for entry in entries:
        uri = entry["uri"]
        if uri_filter not in uri:
            continue

        name = entry["name"]
        pose_txt = entry["pose_txt"]
        if not pose_txt:
            continue

        parts = pose_txt.split()
        if len(parts) < 6:
            continue

        try:
            x, y, z, roll, pitch, yaw = [float(v) for v in parts[:6]]
        except ValueError:
            continue

        raw_x = x
        raw_y = y
        offset_x = 0.0
        offset_y = 0.0
        offset_status = "disabled"

        if apply_mesh_offset:
            model_name = parse_model_name(uri)
            if model_name:
                if model_name not in model_offset_cache:
                    model_offset_cache[model_name] = estimate_model_offset_xy(model_root, model_name, ground_band_z)
                offset_x, offset_y, offset_status = model_offset_cache[model_name]
                x = raw_x + math.cos(yaw) * offset_x - math.sin(yaw) * offset_y
                y = raw_y + math.sin(yaw) * offset_x + math.cos(yaw) * offset_y
            else:
                offset_status = "invalid_model_uri"

        trees.append(
            {
                "tree_id": parse_tree_id(name),
                "name": name,
                "x": x,
                "y": y,
                "raw_x": raw_x,
                "raw_y": raw_y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "uri": uri,
                "offset_x_local": offset_x,
                "offset_y_local": offset_y,
                "offset_status": offset_status,
            }
        )

    trees.sort(key=lambda t: (t["tree_id"] is None, t["tree_id"] if t["tree_id"] is not None else 999999, t["name"]))
    return trees


def ensure_parent_dir(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.exists(parent):
        os.makedirs(parent)


def write_csv(trees, csv_path):
    ensure_parent_dir(csv_path)
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "tree_id",
                "name",
                "x",
                "y",
                "z",
                "roll",
                "pitch",
                "yaw",
                "uri",
                "raw_x",
                "raw_y",
                "offset_x_local",
                "offset_y_local",
                "offset_status",
            ]
        )
        for t in trees:
            writer.writerow(
                [
                    "" if t["tree_id"] is None else t["tree_id"],
                    t["name"],
                    "{:.6f}".format(t["x"]),
                    "{:.6f}".format(t["y"]),
                    "{:.6f}".format(t["z"]),
                    "{:.6f}".format(t["roll"]),
                    "{:.6f}".format(t["pitch"]),
                    "{:.6f}".format(t["yaw"]),
                    t["uri"],
                    "{:.6f}".format(t.get("raw_x", t["x"])),
                    "{:.6f}".format(t.get("raw_y", t["y"])),
                    "{:.6f}".format(t.get("offset_x_local", 0.0)),
                    "{:.6f}".format(t.get("offset_y_local", 0.0)),
                    t.get("offset_status", ""),
                ]
            )


def nice_step(span):
    if span <= 0:
        return 1.0
    base = 10 ** int(math.floor(math.log10(span)))
    candidates = [1, 2, 5, 10]
    for c in candidates:
        step = c * base
        if span / step <= 10:
            return float(step)
    return float(10 * base)


def tick_values(vmin, vmax, step):
    if step <= 0:
        return []
    start = math.floor(vmin / step) * step
    vals = []
    x = start
    while x <= vmax + 1e-9:
        if x >= vmin - 1e-9:
            vals.append(round(x, 6))
        x += step
    return vals


def write_svg(trees, svg_path, title, x_min, x_max, y_min, y_max):
    ensure_parent_dir(svg_path)

    width = 1400.0
    height = 980.0
    margin_left = 90.0
    margin_right = 25.0
    margin_top = 65.0
    margin_bottom = 75.0
    inner_w = width - margin_left - margin_right
    inner_h = height - margin_top - margin_bottom

    def sx(x):
        return margin_left + ((x - x_min) / (x_max - x_min)) * inner_w

    def sy(y):
        return margin_top + (1.0 - (y - y_min) / (y_max - y_min)) * inner_h

    out = []
    out.append('<?xml version="1.0" encoding="UTF-8"?>')
    out.append(
        '<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">'.format(
            w=int(width), h=int(height)
        )
    )
    out.append('<rect x="0" y="0" width="{w}" height="{h}" fill="#f9fbfd"/>'.format(w=int(width), h=int(height)))
    out.append(
        '<text x="{x}" y="36" font-size="24" font-weight="bold" fill="#1f1f1f">{}</text>'.format(
            title.replace("&", "&amp;"),
            x=margin_left,
        )
    )
    out.append(
        '<text x="{x}" y="56" font-size="14" fill="#4c5560">Total trees: {}</text>'.format(
            len(trees), x=margin_left
        )
    )

    # Border
    out.append(
        '<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="none" stroke="#b8c2cc" stroke-width="1.4"/>'.format(
            x=margin_left, y=margin_top, w=inner_w, h=inner_h
        )
    )

    # Grid
    tick_x = nice_step(x_max - x_min)
    tick_y = nice_step(y_max - y_min)
    ticks_x = tick_values(x_min, x_max, tick_x)
    ticks_y = tick_values(y_min, y_max, tick_y)

    for xv in ticks_x:
        px = sx(xv)
        out.append(
            '<line x1="{x}" y1="{y1}" x2="{x}" y2="{y2}" stroke="#e0e7ee" stroke-width="1"/>'.format(
                x=px, y1=margin_top, y2=margin_top + inner_h
            )
        )
        out.append(
            '<text x="{x}" y="{y}" font-size="12" fill="#66717c" text-anchor="middle">{}</text>'.format(
                "{:.1f}".format(xv), x=px, y=margin_top + inner_h + 22
            )
        )

    for yv in ticks_y:
        py = sy(yv)
        out.append(
            '<line x1="{x1}" y1="{y}" x2="{x2}" y2="{y}" stroke="#e0e7ee" stroke-width="1"/>'.format(
                x1=margin_left, x2=margin_left + inner_w, y=py
            )
        )
        out.append(
            '<text x="{x}" y="{y}" font-size="12" fill="#66717c" text-anchor="end">{}</text>'.format(
                "{:.1f}".format(yv), x=margin_left - 8, y=py + 4
            )
        )

    # Axes lines (x=0, y=0)
    if x_min <= 0.0 <= x_max:
        x0 = sx(0.0)
        out.append(
            '<line x1="{x}" y1="{y1}" x2="{x}" y2="{y2}" stroke="#707b86" stroke-width="1.3"/>'.format(
                x=x0, y1=margin_top, y2=margin_top + inner_h
            )
        )
    if y_min <= 0.0 <= y_max:
        y0 = sy(0.0)
        out.append(
            '<line x1="{x1}" y1="{y}" x2="{x2}" y2="{y}" stroke="#707b86" stroke-width="1.3"/>'.format(
                x1=margin_left, x2=margin_left + inner_w, y=y0
            )
        )

    # Points + labels
    for t in trees:
        px = sx(t["x"])
        py = sy(t["y"])
        tree_id = "?" if t["tree_id"] is None else str(t["tree_id"])
        out.append(
            '<circle cx="{x}" cy="{y}" r="6.2" fill="#2b8a3e" stroke="#0f4c20" stroke-width="1"/>'.format(
                x=px, y=py
            )
        )
        label = "{} ({:.2f},{:.2f})".format(tree_id, t["x"], t["y"]).replace("&", "&amp;")
        out.append(
            '<text x="{x}" y="{y}" font-size="11.5" fill="#153b1e">{}</text>'.format(
                label, x=px + 8, y=py - 8
            )
        )

    out.append(
        '<text x="{x}" y="{y}" font-size="14" fill="#3f4952">X (m)</text>'.format(
            x=margin_left + inner_w * 0.5 - 16, y=height - 20
        )
    )
    out.append(
        '<text x="{x}" y="{y}" font-size="14" fill="#3f4952">Y (m)</text>'.format(
            x=22, y=margin_top + inner_h * 0.5
        )
    )
    out.append("</svg>")

    with open(svg_path, "w") as f:
        f.write("\n".join(out) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Ground-truth tree position plotter for Gazebo world files.")
    parser.add_argument("--world", required=True, help="Path to .world file")
    parser.add_argument("--uri-filter", default="tree", help="Include URI substring filter")
    parser.add_argument("--csv-out", required=True, help="Output CSV path")
    parser.add_argument("--svg-out", required=True, help="Output SVG path")
    parser.add_argument("--title", default="World Ground Truth - Tree Positions", help="SVG title")
    parser.add_argument(
        "--model-root",
        default="",
        help="Path to Gazebo models root (default: <world_dir>/../models)",
    )
    parser.add_argument(
        "--disable-mesh-offset",
        action="store_true",
        help="Disable mesh-origin offset correction and use raw <include><pose> coordinates",
    )
    parser.add_argument(
        "--ground-band-z",
        type=float,
        default=0.15,
        help="Z slice thickness (m) above min mesh Z used to estimate trunk base center",
    )
    parser.add_argument("--x-min", type=float, default=-10.0)
    parser.add_argument("--x-max", type=float, default=10.0)
    parser.add_argument("--y-min", type=float, default=-8.0)
    parser.add_argument("--y-max", type=float, default=7.0)
    args = parser.parse_args()

    if args.x_max <= args.x_min or args.y_max <= args.y_min:
        print("Invalid axis limits.", file=sys.stderr)
        return 2

    model_root = args.model_root
    if not model_root:
        model_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(args.world)), "..", "models"))
    apply_mesh_offset = (not args.disable_mesh_offset)

    trees = parse_world_trees(
        args.world,
        args.uri_filter,
        apply_mesh_offset=apply_mesh_offset,
        model_root=model_root,
        ground_band_z=args.ground_band_z,
    )
    if not trees:
        print("No trees found with uri filter '{}' in {}".format(args.uri_filter, args.world), file=sys.stderr)
        return 1

    write_csv(trees, args.csv_out)
    write_svg(trees, args.svg_out, args.title, args.x_min, args.x_max, args.y_min, args.y_max)

    status_counts = {}
    for t in trees:
        key = t.get("offset_status", "")
        status_counts[key] = status_counts.get(key, 0) + 1

    print("Trees parsed: {}".format(len(trees)))
    print("Mesh offset correction: {}".format("enabled" if apply_mesh_offset else "disabled"))
    print("Model root: {}".format(model_root))
    if status_counts:
        print("Offset status counts: {}".format(status_counts))
    print("CSV: {}".format(os.path.abspath(args.csv_out)))
    print("SVG: {}".format(os.path.abspath(args.svg_out)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
