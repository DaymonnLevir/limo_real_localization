#!/usr/bin/env python3

"""Overlay plot: world ground-truth trees vs tree_map_fuser CSV."""

import argparse
import csv
import os
import sys


def load_truth(csv_path):
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rows.append(
                    {
                        "id": int(row["tree_id"]),
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                    }
                )
            except Exception:
                continue
    return rows


def load_map(csv_path):
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rows.append(
                    {
                        "id": int(row["map_id"]),
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "confirmed": int(row.get("confirmed", "1")) != 0,
                    }
                )
            except Exception:
                continue
    return rows


def ensure_parent(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.exists(parent):
        os.makedirs(parent)


def write_svg(truth_rows, map_rows, out_path, title, x_min, x_max, y_min, y_max):
    w = 1400.0
    h = 980.0
    ml = 90.0
    mr = 25.0
    mt = 65.0
    mb = 75.0
    iw = w - ml - mr
    ih = h - mt - mb

    def sx(x):
        return ml + ((x - x_min) / (x_max - x_min)) * iw

    def sy(y):
        return mt + (1.0 - ((y - y_min) / (y_max - y_min))) * ih

    out = []
    out.append('<?xml version="1.0" encoding="UTF-8"?>')
    out.append('<svg xmlns="http://www.w3.org/2000/svg" width="{0}" height="{1}" viewBox="0 0 {0} {1}">'.format(int(w), int(h)))
    out.append('<rect x="0" y="0" width="{0}" height="{1}" fill="#f9fbfd"/>'.format(int(w), int(h)))
    out.append('<text x="{0}" y="36" font-size="24" font-weight="bold" fill="#1f1f1f">{1}</text>'.format(ml, title))
    out.append(
        '<text x="{0}" y="56" font-size="14" fill="#4c5560">ground_truth={1} | tree_map={2}</text>'.format(
            ml, len(truth_rows), len(map_rows)
        )
    )
    out.append('<rect x="{0}" y="{1}" width="{2}" height="{3}" fill="none" stroke="#b8c2cc" stroke-width="1.4"/>'.format(ml, mt, iw, ih))

    # 1m grid
    x = x_min
    while x <= x_max + 1e-9:
        px = sx(x)
        out.append('<line x1="{0}" y1="{1}" x2="{0}" y2="{2}" stroke="#e0e7ee" stroke-width="1"/>'.format(px, mt, mt + ih))
        out.append('<text x="{0}" y="{1}" font-size="12" fill="#66717c" text-anchor="middle">{2:.1f}</text>'.format(px, mt + ih + 22, x))
        x += 1.0

    y = y_min
    while y <= y_max + 1e-9:
        py = sy(y)
        out.append('<line x1="{0}" y1="{2}" x2="{1}" y2="{2}" stroke="#e0e7ee" stroke-width="1"/>'.format(ml, ml + iw, py))
        out.append('<text x="{0}" y="{1}" font-size="12" fill="#66717c" text-anchor="end">{2:.1f}</text>'.format(ml - 8, py + 4, y))
        y += 1.0

    if x_min <= 0.0 <= x_max:
        x0 = sx(0.0)
        out.append('<line x1="{0}" y1="{1}" x2="{0}" y2="{2}" stroke="#707b86" stroke-width="1.3"/>'.format(x0, mt, mt + ih))
    if y_min <= 0.0 <= y_max:
        y0 = sy(0.0)
        out.append('<line x1="{0}" y1="{2}" x2="{1}" y2="{2}" stroke="#707b86" stroke-width="1.3"/>'.format(ml, ml + iw, y0))

    for row in truth_rows:
        px = sx(row["x"])
        py = sy(row["y"])
        out.append('<circle cx="{0}" cy="{1}" r="5.5" fill="#2b8a3e" stroke="#0f4c20" stroke-width="1"/>'.format(px, py))
        out.append('<text x="{0}" y="{1}" font-size="11" fill="#153b1e">T{2}</text>'.format(px + 7, py - 7, row["id"]))

    for row in map_rows:
        px = sx(row["x"])
        py = sy(row["y"])
        color = "#1256d6" if row["confirmed"] else "#e67e22"
        out.append('<line x1="{0}" y1="{1}" x2="{2}" y2="{3}" stroke="{4}" stroke-width="1.6"/>'.format(px - 5, py - 5, px + 5, py + 5, color))
        out.append('<line x1="{0}" y1="{1}" x2="{2}" y2="{3}" stroke="{4}" stroke-width="1.6"/>'.format(px - 5, py + 5, px + 5, py - 5, color))
        out.append('<text x="{0}" y="{1}" font-size="11" fill="{2}">M{3}</text>'.format(px + 7, py + 11, color, row["id"]))

    lx = ml + iw - 280
    ly = mt + 18
    out.append('<rect x="{0}" y="{1}" width="255" height="68" fill="#ffffff" stroke="#c7d0da"/>'.format(lx, ly))
    out.append('<circle cx="{0}" cy="{1}" r="5.5" fill="#2b8a3e" stroke="#0f4c20" stroke-width="1"/>'.format(lx + 16, ly + 20))
    out.append('<text x="{0}" y="{1}" font-size="12" fill="#153b1e">Ground truth tree (T#)</text>'.format(lx + 30, ly + 24))
    out.append('<line x1="{0}" y1="{1}" x2="{2}" y2="{3}" stroke="#1256d6" stroke-width="1.6"/>'.format(lx + 11, ly + 44, lx + 21, ly + 54))
    out.append('<line x1="{0}" y1="{1}" x2="{2}" y2="{3}" stroke="#1256d6" stroke-width="1.6"/>'.format(lx + 11, ly + 54, lx + 21, ly + 44))
    out.append('<text x="{0}" y="{1}" font-size="12" fill="#1256d6">Tree map estimate (M#)</text>'.format(lx + 30, ly + 52))

    out.append('<text x="{0}" y="{1}" font-size="14" fill="#3f4952">X (m)</text>'.format(ml + iw * 0.5 - 16, h - 20))
    out.append('<text x="22" y="{0}" font-size="14" fill="#3f4952">Y (m)</text>'.format(mt + ih * 0.5))
    out.append("</svg>")

    ensure_parent(out_path)
    with open(out_path, "w") as f:
        f.write("\n".join(out) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Compare world tree ground-truth with tree_map_fuser CSV.")
    parser.add_argument("--truth-csv", required=True, help="CSV from world_tree_ground_truth_plotter.py")
    parser.add_argument("--map-csv", required=True, help="tree_map_final.csv path")
    parser.add_argument("--svg-out", required=True, help="Output overlay SVG")
    parser.add_argument("--title", default="Ground Truth vs Tree Map", help="SVG title")
    parser.add_argument("--x-min", type=float, default=-10.0)
    parser.add_argument("--x-max", type=float, default=10.0)
    parser.add_argument("--y-min", type=float, default=-8.0)
    parser.add_argument("--y-max", type=float, default=7.0)
    args = parser.parse_args()

    if args.x_max <= args.x_min or args.y_max <= args.y_min:
        print("Invalid axis limits", file=sys.stderr)
        return 2

    truth_rows = load_truth(args.truth_csv)
    if not truth_rows:
        print("No rows in truth CSV: {}".format(args.truth_csv), file=sys.stderr)
        return 1

    map_rows = load_map(args.map_csv) if os.path.exists(args.map_csv) else []
    write_svg(truth_rows, map_rows, args.svg_out, args.title, args.x_min, args.x_max, args.y_min, args.y_max)

    print("Truth rows: {}".format(len(truth_rows)))
    print("Map rows: {}".format(len(map_rows)))
    print("SVG: {}".format(os.path.abspath(args.svg_out)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
