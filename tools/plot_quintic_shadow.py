#!/usr/bin/env python3

import argparse
import csv
import html
import math
from pathlib import Path


AXES = ("X", "Y", "Z", "A", "B", "C")
COLORS = ("#1677ff", "#d62728", "#18a558", "#9467bd", "#ff7f0e", "#17becf")


def polynomial(coefficients, parameter):
    result = 0.0
    for coefficient in reversed(coefficients):
        result = result * parameter + coefficient
    return result


def derivative(coefficients, parameter, order):
    values = list(coefficients)
    for _ in range(order):
        values = [index * values[index] for index in range(1, len(values))]
    return polynomial(values, parameter)


def selected_spans(path, first_span, span_count):
    selected = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            span = int(row["shadow_span"])
            if span < first_span:
                continue
            if span >= first_span + span_count:
                break
            selected.setdefault(span, {})[row["axis"]] = row
    return selected


def input_sources(path):
    result = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            result[int(row["input"])] = row
    return result


def sample_shadow(spans, samples_per_span):
    samples = []
    for span_index, rows in spans.items():
        if set(rows) != set(AXES):
            raise RuntimeError(f"shadow span {span_index} does not contain all six axes")
        reference = rows["X"]
        duration = float(reference["duration"])
        for sample_index in range(samples_per_span + 1):
            if samples and sample_index == 0:
                continue
            parameter = sample_index / samples_per_span
            sample = {
                "span": span_index,
                "parameter": parameter,
                "time": float(reference["global_time_from"]) + duration * parameter,
            }
            jerk_squared = 0.0
            for axis in AXES:
                row = rows[axis]
                coefficients = [
                    float(row[f"normalized_c{index}"]) for index in range(6)
                ]
                sample[axis] = float(row["origin"]) + polynomial(
                    coefficients, parameter)
                sample[f"{axis}_velocity"] = derivative(
                    coefficients, parameter, 1) / duration
                sample[f"{axis}_acceleration"] = derivative(
                    coefficients, parameter, 2) / (duration * duration)
                jerk = derivative(
                    coefficients, parameter, 3) / (duration * duration * duration)
                jerk_squared += jerk * jerk
            sample["path_jerk"] = math.sqrt(jerk_squared)
            samples.append(sample)
    return samples


def padded_range(values):
    low = min(values)
    high = max(values)
    span = max(high - low, 1e-12)
    return low - span * 0.06, high + span * 0.06


def polyline(samples, x_key, y_key, bounds, size):
    x_low, x_high, y_low, y_high = bounds
    left, top, width, height = size
    return " ".join(
        f"{left + (sample[x_key] - x_low) / (x_high - x_low) * width:.3f},"
        f"{top + height - (sample[y_key] - y_low) / (y_high - y_low) * height:.3f}"
        for sample in samples
    )


def chart(samples, x_key, series, title, x_label):
    width = 760
    height = 300
    size = (66, 28, 660, 220)
    x_range = padded_range([sample[x_key] for sample in samples])
    y_range = padded_range([
        sample[key] for key, _ in series for sample in samples
    ])
    paths = []
    for index, (key, label) in enumerate(series):
        coordinates = polyline(
            samples, x_key, key, (*x_range, *y_range), size)
        paths.append(
            f'<polyline points="{coordinates}" fill="none" '
            f'stroke="{COLORS[index]}" stroke-width="1.6"/>'
        )
        paths.append(
            f'<text x="{80 + index * 90}" y="278" '
            f'fill="{COLORS[index]}">{html.escape(label)}</text>'
        )
    return f"""
<svg viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">
  <rect x="{size[0]}" y="{size[1]}" width="{size[2]}" height="{size[3]}" class="frame"/>
  {''.join(paths)}
  <text x="10" y="18" class="chart-title">{html.escape(title)}</text>
  <text x="{size[0]}" y="266">{x_range[0]:.9g}</text>
  <text x="{size[0] + size[2] - 70}" y="266">{x_range[1]:.9g}</text>
  <text x="320" y="294">{html.escape(x_label)}</text>
  <text x="{size[0] + 5}" y="{size[1] + 14}">{y_range[1]:.9g}</text>
  <text x="{size[0] + 5}" y="{size[1] + size[3] - 5}">{y_range[0]:.9g}</text>
</svg>"""


def source_summary(spans, sources):
    referenced = set()
    for rows in spans.values():
        reference = rows["X"]
        referenced.update(range(
            int(reference["first_source_input"]),
            int(reference["last_source_input"]) + 1))
    descriptions = []
    for input_index in sorted(referenced):
        source = sources.get(input_index)
        if source:
            descriptions.append(
                f'{source["source"]}:{source["line"]}  {source["text"]}')
    if len(descriptions) > 20:
        descriptions = descriptions[:10] + [
            f"... {len(descriptions) - 20} more ..."
        ] + descriptions[-10:]
    return "<br>".join(html.escape(value) for value in descriptions)


def generate(prefix, output, first_span, span_count, samples_per_span):
    spans = selected_spans(
        Path(str(prefix) + "_shadow_spans.csv"), first_span, span_count)
    if not spans:
        raise RuntimeError("the requested shadow span range is empty")
    samples = sample_shadow(spans, samples_per_span)
    sources = input_sources(Path(str(prefix) + "_shadow_inputs.csv"))
    linear_axes = [
        (axis, axis) for axis in AXES
        if max(sample[axis] for sample in samples)
            - min(sample[axis] for sample in samples) > 1e-12
    ]
    if not linear_axes:
        linear_axes = [("X", "X")]
    geometry = chart(samples, "X", [("Y", "Y")], "XY path", "X")
    positions = chart(
        samples, "time", linear_axes, "axis position", "global time (s)")
    jerk = chart(
        samples, "time", [("path_jerk", "|axis jerk|")],
        "axis-space jerk magnitude", "global time (s)")
    first = min(spans)
    last = max(spans)
    document = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Quintic shadow spans {first}-{last}</title>
<style>
body {{ margin: 0; padding: 24px; font: 14px/1.45 "Segoe UI", sans-serif;
       color: #202124; background: #f4f6f8; }}
.card {{ margin: 18px 0; padding: 18px; background: white; border: 1px solid #d9dde3;
         border-radius: 8px; }}
.charts {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(600px, 1fr)); }}
svg {{ width: 100%; min-width: 0; }}
svg text {{ font: 11px "Segoe UI", sans-serif; fill: #333; }}
.chart-title {{ font-size: 13px; font-weight: 600; }}
.frame {{ fill: #fafafa; stroke: #c8ccd2; }}
.sources {{ font: 12px Consolas, monospace; }}
</style>
</head>
<body>
<h1>Quintic shadow spans {first}-{last}</h1>
<p>{len(spans)} spans, {len(samples)} evaluated points, global time
{samples[0]["time"]:.12g} to {samples[-1]["time"]:.12g} seconds.
Position is evaluated directly as <code>origin + sum(c[i] * u^i)</code>
from the exported double coefficients.</p>
<section class="card charts">{geometry}{positions}{jerk}</section>
<section class="card"><h2>Source entities</h2>
<p class="sources">{source_summary(spans, sources)}</p></section>
</body>
</html>
"""
    output.write_text(document, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix", type=Path, help="CSV prefix used by --quintic-failures")
    parser.add_argument("--first-span", type=int, default=0)
    parser.add_argument("--span-count", type=int, default=100)
    parser.add_argument("--samples-per-span", type=int, default=16)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    if arguments.first_span < 0 or arguments.span_count <= 0:
        parser.error("span range must be positive")
    if arguments.samples_per_span <= 0:
        parser.error("--samples-per-span must be positive")
    output = arguments.output or Path(
        f"{arguments.prefix}_shadow_{arguments.first_span}_"
        f"{arguments.span_count}.html")
    generate(arguments.prefix, output, arguments.first_span,
             arguments.span_count, arguments.samples_per_span)
    print(output)


if __name__ == "__main__":
    main()
