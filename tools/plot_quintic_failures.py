#!/usr/bin/env python3

import argparse
import csv
import html
import math
from pathlib import Path


AXES = ("X", "Y", "Z", "A", "B", "C")
COLORS = {
    "prepared": "#555",
    "quintic": "#1677ff",
    "deviation": "#18a558",
    "jerk": "#d62728",
    "limit": "#8c8c8c",
}


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def group_by(rows, key):
    result = {}
    for row in rows:
        result.setdefault(int(row[key]), []).append(row)
    return result


def points(values, x_value, y_value, left, top, width, height, x_range, y_range):
    x_min, x_max = x_range
    y_min, y_max = y_range
    x_scale = width / max(x_max - x_min, 1e-30)
    y_scale = height / max(y_max - y_min, 1e-30)
    return " ".join(
        f"{left + (x_value(value) - x_min) * x_scale:.3f},"
        f"{top + height - (y_value(value) - y_min) * y_scale:.3f}"
        for value in values
    )


def padded_range(values, minimum_span=1e-12):
    low = min(values)
    high = max(values)
    span = max(high - low, minimum_span)
    padding = span * 0.08
    return low - padding, high + padding


def geometry_svg(samples):
    width = 520
    height = 310
    left = 54
    top = 24
    plot_width = 440
    plot_height = 240
    x_values = [
        float(row[column])
        for row in samples
        for column in ("quintic_x", "prepared_x")
    ]
    y_values = [
        float(row[column])
        for row in samples
        for column in ("quintic_y", "prepared_y")
    ]
    x_range = padded_range(x_values)
    y_range = padded_range(y_values)
    prepared = points(
        samples, lambda row: float(row["prepared_x"]),
        lambda row: float(row["prepared_y"]), left, top, plot_width, plot_height,
        x_range, y_range)
    quintic = points(
        samples, lambda row: float(row["quintic_x"]),
        lambda row: float(row["quintic_y"]), left, top, plot_width, plot_height,
        x_range, y_range)
    return f"""
<svg viewBox="0 0 {width} {height}" role="img" aria-label="XY geometry">
  <rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" class="frame"/>
  <polyline points="{prepared}" fill="none" stroke="{COLORS['prepared']}" stroke-width="4"/>
  <polyline points="{quintic}" fill="none" stroke="{COLORS['quintic']}" stroke-width="1.7"/>
  <text x="{left}" y="295">X: {x_range[0]:.9g} to {x_range[1]:.9g}</text>
  <text x="{left + 250}" y="295">Y: {y_range[0]:.9g} to {y_range[1]:.9g}</text>
  <text x="10" y="16" class="chart-title">XY geometry</text>
</svg>"""


def line_chart(samples, value, title, color, limit=None):
    width = 520
    height = 250
    left = 54
    top = 24
    plot_width = 440
    plot_height = 170
    values = [value(row) for row in samples]
    upper = max(max(values) * 1.08, limit * 1.08 if limit is not None else 0.0, 1e-30)
    curve = points(
        samples, lambda row: float(row["parameter"]), value, left, top,
        plot_width, plot_height, (0.0, 1.0), (0.0, upper))
    limit_line = ""
    if limit is not None:
        y = top + plot_height - limit / upper * plot_height
        limit_line = (
            f'<line x1="{left}" y1="{y:.3f}" x2="{left + plot_width}" '
            f'y2="{y:.3f}" stroke="{COLORS["limit"]}" stroke-dasharray="6 4"/>'
        )
    return f"""
<svg viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">
  <rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" class="frame"/>
  {limit_line}
  <polyline points="{curve}" fill="none" stroke="{color}" stroke-width="1.8"/>
  <text x="10" y="16" class="chart-title">{html.escape(title)}</text>
  <text x="{left}" y="222">0</text>
  <text x="{left + plot_width - 8}" y="222">1</text>
  <text x="{left + 190}" y="240">normalized time</text>
  <text x="{left + 5}" y="{top + 14}">max {max(values):.9g}</text>
</svg>"""


def jerk_ratio_function(coefficient_rows, path_jerk):
    coefficients = {
        row["axis"]: [float(row[f"seconds_c{index}"]) for index in range(6)]
        for row in coefficient_rows
    }

    def jerk_ratio(sample):
        time = float(sample["time"])
        squared = 0.0
        for axis in AXES:
            values = coefficients[axis]
            jerk = (60.0 * values[5] * time + 24.0 * values[4]) * time \
                + 6.0 * values[3]
            squared += jerk * jerk
        return math.sqrt(squared) / path_jerk

    return jerk_ratio


def source_summary(rows):
    descriptions = [
        f'{row["source"]}:{row["line"]}  {row["text"]}'
        for row in rows
    ]
    if len(descriptions) <= 8:
        shown = descriptions
    else:
        shown = descriptions[:4] + [f"... {len(descriptions) - 8} more ..."] \
            + descriptions[-4:]
    return "<br>".join(html.escape(value) for value in shown)


def generate(prefix, output, path_jerk):
    failures = read_rows(Path(str(prefix) + "_failures.csv"))
    coefficients = group_by(read_rows(Path(str(prefix) + "_coefficients.csv")), "failure")
    sources = group_by(read_rows(Path(str(prefix) + "_sources.csv")), "failure")
    samples = group_by(read_rows(Path(str(prefix) + "_samples.csv")), "failure")
    cards = []
    for failure in failures:
        failure_index = int(failure["failure"])
        failure_samples = samples[failure_index]
        deviation_chart = line_chart(
            failure_samples, lambda row: float(row["deviation"]),
            "quintic-to-prepared deviation", COLORS["deviation"])
        jerk_chart = line_chart(
            failure_samples,
            jerk_ratio_function(coefficients[failure_index], path_jerk),
            f"path jerk ratio (limit {path_jerk:.9g})", COLORS["jerk"], 1.0)
        cards.append(f"""
<section class="card" id="failure-{failure_index}">
  <h2>Failure {failure_index}: {html.escape(failure["prepared_kind"])}</h2>
  <p class="metrics">
    timing piece {failure["timing_piece"]}; prepared piece {failure["prepared_piece"]};
    knot {failure["knot_interval"]}; duration {float(failure["duration"]) * 1e6:.9g} us;
    certified ratio {float(failure["certified_ratio"]):.12g};
    sampled ratio {float(failure["sampled_ratio"]):.12g}
  </p>
  <details><summary>Source entities ({failure["source_count"]})</summary>
    <p class="sources">{source_summary(sources.get(failure_index, []))}</p>
  </details>
  <div class="charts">
    {geometry_svg(failure_samples)}
    {deviation_chart}
    {jerk_chart}
  </div>
</section>""")
    document = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Unresolved endpoint-PVA quintics</title>
<style>
body {{ margin: 0; padding: 24px; font: 14px/1.45 "Segoe UI", sans-serif; color: #202124;
       background: #f4f6f8; }}
h1 {{ margin: 0 0 8px; }}
.intro {{ max-width: 1100px; }}
.card {{ margin: 22px 0; padding: 20px; background: white; border: 1px solid #d9dde3;
         border-radius: 8px; box-shadow: 0 1px 4px #00000012; }}
.card h2 {{ margin: 0 0 4px; }}
.metrics {{ color: #444; }}
.sources {{ font-family: Consolas, monospace; font-size: 12px; }}
.charts {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(420px, 1fr)); gap: 8px; }}
svg {{ width: 100%; min-width: 0; }}
svg text {{ font: 11px "Segoe UI", sans-serif; fill: #333; }}
svg .chart-title {{ font-size: 13px; font-weight: 600; }}
svg .frame {{ fill: #fafafa; stroke: #c8ccd2; }}
code {{ font-family: Consolas, monospace; }}
</style>
</head>
<body>
<h1>Unresolved endpoint-PVA quintics</h1>
<p class="intro">All {len(failures)} unresolved intervals from <code>{html.escape(str(prefix))}</code>.
The thick gray XY trace is the prepared geometry evaluated under the original scalar time law;
the thin blue trace is the endpoint-PVA quintic. Jerk is evaluated directly from the exported
local-time power coefficients.</p>
{''.join(cards)}
</body>
</html>
"""
    output.write_text(document, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix", type=Path, help="CSV prefix used by --quintic-failures")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--path-jerk", type=float, default=101.0)
    arguments = parser.parse_args()
    output = arguments.output or Path(str(arguments.prefix) + "_report.html")
    generate(arguments.prefix, output, arguments.path_jerk)
    print(output)


if __name__ == "__main__":
    main()
