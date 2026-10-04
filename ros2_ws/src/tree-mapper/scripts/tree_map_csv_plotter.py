#!/usr/bin/env python3

"""Offline SVG plotter for object_map_fuser CSV exports.

Input CSV format (object_map_fuser.py):
  map_id,x,y,z,hits,std_xy,confirmed,age_sec,last_seen_sec,source_ids
"""

import argparse
import csv
import math
import os
import sys


def parse_bool(raw):
    text = str(raw).strip().lower()
    return text in ("1", "true", "yes", "y")


def parse_float(raw, default=0.0):
    try:
        return float(str(raw).strip())
    except Exception:
        return default


def parse_int(raw, default=0):
    try:
        return int(str(raw).strip())
    except Exception:
        return default


def esc(text):
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def nice_step(span):
    if span <= 0.0:
        return 1.0
    target = span / 8.0
    magnitude = 10 ** math.floor(math.log10(target))
    residual = target / magnitude
    if residual < 1.5:
        base = 1.0
    elif residual < 3.0:
        base = 2.0
    elif residual < 7.0:
        base = 5.0
    else:
        base = 10.0
    return base * magnitude


def load_rows(csv_path):
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = parse_float(row.get("x"))
            y = parse_float(row.get("y"))
            rows.append(
                {
                    "map_id": parse_int(row.get("map_id"), -1),
                    "x": x,
                    "y": y,
                    "z": parse_float(row.get("z")),
                    "diameter_m": parse_float(row.get("diameter_m"), 0.0),
                    "hits": parse_int(row.get("hits")),
                    "std_xy": parse_float(row.get("std_xy")),
                    "std_diameter": parse_float(row.get("std_diameter"), 0.0),
                    "circle_fit_score": parse_float(row.get("circle_fit_score"), 0.0),
                    "geometric_score": parse_float(row.get("geometric_score"), 0.0),
                    "confirmed": parse_bool(row.get("confirmed")),
                }
            )
    return rows


def write_svg(rows, out_path, width, height, label_mode):
    margin_left = 66
    margin_right = 20
    margin_top = 34
    margin_bottom = 56
    inner_w = max(100, width - margin_left - margin_right)
    inner_h = max(100, height - margin_top - margin_bottom)

    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]

    x_min = min(xs)
    x_max = max(xs)
    y_min = min(ys)
    y_max = max(ys)

    if x_max - x_min < 1e-6:
        x_min -= 1.0
        x_max += 1.0
    if y_max - y_min < 1e-6:
        y_min -= 1.0
        y_max += 1.0

    x_pad = max((x_max - x_min) * 0.08, 0.5)
    y_pad = max((y_max - y_min) * 0.08, 0.5)
    x_min -= x_pad
    x_max += x_pad
    y_min -= y_pad
    y_max += y_pad

    def map_x(x):
        return margin_left + ((x - x_min) / (x_max - x_min)) * inner_w

    def map_y(y):
        return margin_top + (1.0 - (y - y_min) / (y_max - y_min)) * inner_h

    tick_x = nice_step(x_max - x_min)
    tick_y = nice_step(y_max - y_min)

    def tick_values(vmin, vmax, step):
        start = math.ceil(vmin / step) * step
        end = math.floor(vmax / step) * step
        vals = []
        cur = start
        lim = 0
        while cur <= end + 1e-9 and lim < 1000:
            vals.append(cur)
            cur += step
            lim += 1
        return vals

    x_ticks = tick_values(x_min, x_max, tick_x)
    y_ticks = tick_values(y_min, y_max, tick_y)

    confirmed_rows = [r for r in rows if r["confirmed"]]
    candidate_rows = [r for r in rows if not r["confirmed"]]

    lines = []
    lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    lines.append(
        '<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">'.format(
            w=width, h=height
        )
    )
    lines.append('<rect x="0" y="0" width="{w}" height="{h}" fill="#ffffff"/>'.format(w=width, h=height))

    # Grid and axis labels
    for xv in x_ticks:
        xpx = map_x(xv)
        lines.append(
            '<line x1="{x:.2f}" y1="{y1}" x2="{x:.2f}" y2="{y2}" stroke="#ececec" stroke-width="1"/>'.format(
                x=xpx, y1=margin_top, y2=margin_top + inner_h
            )
        )
        lines.append(
            '<text x="{x:.2f}" y="{y}" text-anchor="middle" font-size="11" fill="#606060">{t}</text>'.format(
                x=xpx, y=height - 22, t=("{:.2f}".format(xv)).rstrip("0").rstrip(".")
            )
        )
    for yv in y_ticks:
        ypx = map_y(yv)
        lines.append(
            '<line x1="{x1}" y1="{y:.2f}" x2="{x2}" y2="{y:.2f}" stroke="#ececec" stroke-width="1"/>'.format(
                x1=margin_left, x2=margin_left + inner_w, y=ypx
            )
        )
        lines.append(
            '<text x="{x}" y="{y:.2f}" text-anchor="end" dominant-baseline="middle" font-size="11" fill="#606060">{t}</text>'.format(
                x=margin_left - 8, y=ypx, t=("{:.2f}".format(yv)).rstrip("0").rstrip(".")
            )
        )

    # Axes at x=0 and y=0, if visible
    if x_min <= 0.0 <= x_max:
        x0 = map_x(0.0)
        lines.append(
            '<line x1="{x:.2f}" y1="{y1}" x2="{x:.2f}" y2="{y2}" stroke="#9a9a9a" stroke-width="1.4"/>'.format(
                x=x0, y1=margin_top, y2=margin_top + inner_h
            )
        )
    if y_min <= 0.0 <= y_max:
        y0 = map_y(0.0)
        lines.append(
            '<line x1="{x1}" y1="{y:.2f}" x2="{x2}" y2="{y:.2f}" stroke="#9a9a9a" stroke-width="1.4"/>'.format(
                x1=margin_left, x2=margin_left + inner_w, y=y0
            )
        )

    lines.append(
        '<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="none" stroke="#555555" stroke-width="1.5"/>'.format(
            x=margin_left, y=margin_top, w=inner_w, h=inner_h
        )
    )

    def draw_points(data_rows, fill, stroke):
        for r in data_rows:
            cx = map_x(r["x"])
            cy = map_y(r["y"])
            # Keep marker size readable while still reflecting diameter estimate.
            radius = 4.0 + min(10.0, max(0.0, r["diameter_m"] * 10.0))
            lines.append(
                '<circle cx="{x:.2f}" cy="{y:.2f}" r="{r:.2f}" fill="{f}" fill-opacity="0.62" stroke="{s}" stroke-width="1.2"/>'.format(
                    x=cx, y=cy, r=radius, f=fill, s=stroke
                )
            )
            if label_mode != "none":
                if label_mode == "id":
                    label = str(r["map_id"])
                elif label_mode == "hits":
                    label = str(r["hits"])
                elif label_mode == "diameter":
                    label = "{:.2f}m".format(r["diameter_m"])
                elif label_mode == "id_diameter":
                    label = "{} / {:.2f}m".format(r["map_id"], r["diameter_m"])
                elif label_mode == "id_hits_diameter":
                    label = "{}/{} / {:.2f}m".format(r["map_id"], r["hits"], r["diameter_m"])
                else:
                    label = "{}/{}".format(r["map_id"], r["hits"])
                lines.append(
                    '<text x="{x:.2f}" y="{y:.2f}" font-size="11" fill="#202020" dominant-baseline="middle">{t}</text>'.format(
                        x=cx + radius + 2.0, y=cy - radius - 2.0, t=esc(label)
                    )
                )

    # Draw candidates first so confirmed nodes stay on top.
    draw_points(candidate_rows, fill="#f4a340", stroke="#b57612")
    draw_points(confirmed_rows, fill="#2ca25f", stroke="#176b3f")

    # Title and legend
    lines.append(
        '<text x="{x}" y="{y}" font-size="16" font-weight="bold" fill="#1f1f1f">Tree Map CSV Plot</text>'.format(
            x=margin_left, y=22
        )
    )
    lines.append(
        '<text x="{x}" y="{y}" font-size="12" fill="#404040">Confirmed: {c} | Candidates: {k} | Total: {t}</text>'.format(
            x=margin_left, y=38, c=len(confirmed_rows), k=len(candidate_rows), t=len(rows)
        )
    )
    lx = width - 215
    ly = 18
    lines.append('<rect x="{x}" y="{y}" width="196" height="42" fill="#ffffff" fill-opacity="0.9" stroke="#cccccc"/>'.format(x=lx, y=ly))
    lines.append('<circle cx="{x}" cy="{y}" r="6" fill="#2ca25f" fill-opacity="0.62" stroke="#176b3f" stroke-width="1.2"/>'.format(x=lx + 16, y=ly + 15))
    lines.append('<text x="{x}" y="{y}" font-size="12" fill="#222222" dominant-baseline="middle">confirmed</text>'.format(x=lx + 30, y=ly + 15))
    lines.append('<circle cx="{x}" cy="{y}" r="6" fill="#f4a340" fill-opacity="0.62" stroke="#b57612" stroke-width="1.2"/>'.format(x=lx + 16, y=ly + 31))
    lines.append('<text x="{x}" y="{y}" font-size="12" fill="#222222" dominant-baseline="middle">candidate</text>'.format(x=lx + 30, y=ly + 31))

    lines.append(
        '<text x="{x}" y="{y}" text-anchor="middle" font-size="13" fill="#303030">X (m)</text>'.format(
            x=margin_left + inner_w / 2.0, y=height - 6
        )
    )
    lines.append(
        '<g transform="translate(16,{cy}) rotate(-90)"><text text-anchor="middle" font-size="13" fill="#303030">Y (m)</text></g>'.format(
            cy=margin_top + inner_h / 2.0
        )
    )
    lines.append("</svg>")

    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Plot tree_map_fuser CSV as SVG.")
    parser.add_argument(
        "--csv",
        dest="csv_path",
        required=True,
        help="Path to tree_map_final.csv",
    )
    parser.add_argument(
        "--out",
        dest="out_path",
        default="",
        help="Output SVG path (default: <csv_basename>_plot.svg)",
    )
    parser.add_argument("--width", type=int, default=1200)
    parser.add_argument("--height", type=int, default=900)
    parser.add_argument(
        "--label",
        choices=("none", "id", "hits", "id_hits", "diameter", "id_diameter", "id_hits_diameter"),
        default="id_diameter",
        help="Point label mode",
    )
    args = parser.parse_args()

    rows = load_rows(args.csv_path)
    if not rows:
        print("No rows found in CSV: {}".format(args.csv_path), file=sys.stderr)
        return 1

    out_path = args.out_path
    if not out_path:
        base, _ = os.path.splitext(args.csv_path)
        out_path = base + "_plot.svg"

    write_svg(rows, out_path, max(400, args.width), max(300, args.height), args.label)
    print("CSV rows: {}".format(len(rows)))
    print("Saved plot: {}".format(out_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
