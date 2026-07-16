use clarabel::algebra::CscMatrix;
use clarabel::solver::{
    DefaultSettingsBuilder, DefaultSolver, IPSolver, NonnegativeConeT, SecondOrderConeT,
    SolverStatus, SupportedConeT, ZeroConeT,
};
use std::cmp::Ordering;
use std::env;
use std::error::Error;
use std::fs;
use std::io::{BufWriter, Write};
use std::path::Path;

const AXES: usize = 6;

#[derive(Debug, Clone)]
struct Segment {
    piece: usize,
    input: usize,
    length: f64,
    velocity_limit: f64,
    tangent: [f64; AXES],
    curvature: [f64; AXES],
    curvature_derivative: [f64; AXES],
}

#[derive(Debug)]
struct PlannerPiece {
    input: usize,
    length: f64,
    velocity_limit: f64,
    acceleration_limit: f64,
    jerk_limit: f64,
    entry_velocity: f64,
    entry_acceleration: f64,
    exit_velocity: f64,
    exit_acceleration: f64,
    duration: f64,
}

#[derive(Debug)]
struct Model {
    path_acceleration: f64,
    axis_acceleration: [f64; AXES],
    path_jerk: f64,
    axis_jerk: [f64; AXES],
    planner_duration: f64,
    pieces: Vec<PlannerPiece>,
    segments: Vec<Segment>,
}

fn parse_f64(value: &str, context: &str) -> Result<f64, Box<dyn Error>> {
    let parsed = value
        .parse::<f64>()
        .map_err(|error| format!("invalid {context} value {value:?}: {error}"))?;
    if parsed.is_nan() {
        return Err(format!("{context} must not be NaN").into());
    }
    Ok(parsed)
}

fn read_model(path: &Path) -> Result<Model, Box<dyn Error>> {
    let text = fs::read_to_string(path)?;
    let mut lines = text.lines();
    if lines.next() != Some("ngc_clarabel_trajectory_oracle_v3") {
        return Err("unsupported or missing oracle model header".into());
    }

    let path_line = lines.next().ok_or("missing path_acceleration")?;
    let path_fields: Vec<_> = path_line.split_whitespace().collect();
    if path_fields.len() != 2 || path_fields[0] != "path_acceleration" {
        return Err("invalid path_acceleration row".into());
    }
    let path_acceleration = parse_f64(path_fields[1], "path_acceleration")?;

    let axis_line = lines.next().ok_or("missing axis_acceleration")?;
    let axis_fields: Vec<_> = axis_line.split_whitespace().collect();
    if axis_fields.len() != AXES + 1 || axis_fields[0] != "axis_acceleration" {
        return Err("invalid axis_acceleration row".into());
    }
    let mut axis_acceleration = [0.0; AXES];
    for axis in 0..AXES {
        axis_acceleration[axis] = parse_f64(axis_fields[axis + 1], "axis acceleration")?;
    }

    let jerk_line = lines.next().ok_or("missing path_jerk")?;
    let jerk_fields: Vec<_> = jerk_line.split_whitespace().collect();
    if jerk_fields.len() != 2 || jerk_fields[0] != "path_jerk" {
        return Err("invalid path_jerk row".into());
    }
    let path_jerk = parse_f64(jerk_fields[1], "path_jerk")?;

    let axis_jerk_line = lines.next().ok_or("missing axis_jerk")?;
    let axis_jerk_fields: Vec<_> = axis_jerk_line.split_whitespace().collect();
    if axis_jerk_fields.len() != AXES + 1 || axis_jerk_fields[0] != "axis_jerk" {
        return Err("invalid axis_jerk row".into());
    }
    let mut axis_jerk = [0.0; AXES];
    for axis in 0..AXES {
        axis_jerk[axis] = parse_f64(axis_jerk_fields[axis + 1], "axis jerk")?;
    }

    let duration_line = lines.next().ok_or("missing planner_duration")?;
    let duration_fields: Vec<_> = duration_line.split_whitespace().collect();
    if duration_fields.len() != 2 || duration_fields[0] != "planner_duration" {
        return Err("invalid planner_duration row".into());
    }
    let planner_duration = parse_f64(duration_fields[1], "planner_duration")?;

    let piece_count_line = lines.next().ok_or("missing piece_count")?;
    let piece_count_fields: Vec<_> = piece_count_line.split_whitespace().collect();
    if piece_count_fields.len() != 2 || piece_count_fields[0] != "piece_count" {
        return Err("invalid piece_count row".into());
    }
    let expected_pieces = piece_count_fields[1].parse::<usize>()?;
    let mut pieces = Vec::with_capacity(expected_pieces);
    for expected_index in 0..expected_pieces {
        let line = lines.next().ok_or("missing piece row")?;
        let fields: Vec<_> = line.split_whitespace().collect();
        if fields.len() != 12 || fields[0] != "piece" {
            return Err(format!("invalid piece row {expected_index}").into());
        }
        let index = fields[1].parse::<usize>()?;
        if index != expected_index {
            return Err(format!("expected piece {expected_index}, found {index}").into());
        }
        pieces.push(PlannerPiece {
            input: fields[2].parse::<usize>()?,
            length: parse_f64(fields[3], "piece length")?,
            velocity_limit: parse_f64(fields[4], "piece velocity limit")?,
            acceleration_limit: parse_f64(fields[5], "piece acceleration limit")?,
            jerk_limit: parse_f64(fields[6], "piece jerk limit")?,
            entry_velocity: parse_f64(fields[7], "piece entry velocity")?,
            entry_acceleration: parse_f64(fields[8], "piece entry acceleration")?,
            exit_velocity: parse_f64(fields[9], "piece exit velocity")?,
            exit_acceleration: parse_f64(fields[10], "piece exit acceleration")?,
            duration: parse_f64(fields[11], "piece duration")?,
        });
    }

    let count_line = lines.next().ok_or("missing segment_count")?;
    let count_fields: Vec<_> = count_line.split_whitespace().collect();
    if count_fields.len() != 2 || count_fields[0] != "segment_count" {
        return Err("invalid segment_count row".into());
    }
    let expected_segments = count_fields[1].parse::<usize>()?;

    let mut segments = Vec::with_capacity(expected_segments);
    for (line_index, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let fields: Vec<_> = line.split_whitespace().collect();
        if fields.len() != 23 || fields[0] != "segment" {
            return Err(format!("invalid segment row at model line {}", line_index + 8).into());
        }
        let piece = fields[1].parse::<usize>()?;
        let input = fields[2].parse::<usize>()?;
        let length = parse_f64(fields[3], "segment length")?;
        let velocity_limit = parse_f64(fields[4], "segment velocity limit")?;
        let mut tangent = [0.0; AXES];
        let mut curvature = [0.0; AXES];
        let mut curvature_derivative = [0.0; AXES];
        for axis in 0..AXES {
            tangent[axis] = parse_f64(fields[5 + axis], "tangent")?;
            curvature[axis] = parse_f64(fields[11 + axis], "curvature")?;
            curvature_derivative[axis] = parse_f64(fields[17 + axis], "curvature derivative")?;
        }
        if !length.is_finite() || length <= 0.0 {
            return Err(format!("segment {} has invalid length {}", segments.len(), length).into());
        }
        if !velocity_limit.is_finite() || velocity_limit <= 0.0 {
            return Err(format!(
                "segment {} has invalid velocity limit {}",
                segments.len(),
                velocity_limit
            )
            .into());
        }
        segments.push(Segment {
            piece,
            input,
            length,
            velocity_limit,
            tangent,
            curvature,
            curvature_derivative,
        });
    }
    if segments.len() != expected_segments || segments.is_empty() {
        return Err(format!(
            "model declares {expected_segments} segments but contains {}",
            segments.len()
        )
        .into());
    }
    if !path_acceleration.is_finite() || path_acceleration <= 0.0 {
        return Err("path_acceleration must be finite and positive".into());
    }
    if !path_jerk.is_finite() || path_jerk <= 0.0 {
        return Err("path_jerk must be finite and positive".into());
    }
    if !planner_duration.is_finite() || planner_duration <= 0.0 {
        return Err("planner_duration must be finite and positive".into());
    }
    for (axis, limit) in axis_acceleration.iter().enumerate() {
        if *limit <= 0.0 {
            return Err(format!("axis acceleration {axis} must be positive").into());
        }
    }
    for (axis, limit) in axis_jerk.iter().enumerate() {
        if *limit <= 0.0 {
            return Err(format!("axis jerk {axis} must be positive").into());
        }
    }
    Ok(Model {
        path_acceleration,
        axis_acceleration,
        path_jerk,
        axis_jerk,
        planner_duration,
        pieces,
        segments,
    })
}

#[derive(Clone, Copy)]
struct Triplet {
    row: usize,
    column: usize,
    value: f64,
}

struct ConstraintBuilder {
    columns: usize,
    rows: usize,
    triplets: Vec<Triplet>,
    rhs: Vec<f64>,
    cones: Vec<SupportedConeT<f64>>,
}

type ConicData = (CscMatrix<f64>, Vec<f64>, Vec<SupportedConeT<f64>>);

impl ConstraintBuilder {
    fn new(columns: usize) -> Self {
        Self {
            columns,
            rows: 0,
            triplets: Vec::new(),
            rhs: Vec::new(),
            cones: Vec::new(),
        }
    }

    fn add_block(&mut self, rows: &[Vec<(usize, f64)>], rhs: &[f64], cone: SupportedConeT<f64>) {
        assert_eq!(rows.len(), rhs.len());
        let first = self.rows;
        for (local_row, entries) in rows.iter().enumerate() {
            for &(column, value) in entries {
                if value != 0.0 {
                    self.triplets.push(Triplet {
                        row: first + local_row,
                        column,
                        value,
                    });
                }
            }
        }
        self.rhs.extend_from_slice(rhs);
        self.rows += rows.len();
        self.cones.push(cone);
    }

    fn finish(mut self) -> Result<ConicData, Box<dyn Error>> {
        self.triplets.sort_by(|left, right| {
            left.column
                .cmp(&right.column)
                .then_with(|| left.row.cmp(&right.row))
        });
        let mut combined: Vec<Triplet> = Vec::with_capacity(self.triplets.len());
        for item in self.triplets {
            if let Some(last) = combined.last_mut()
                && last.column == item.column
                && last.row == item.row
            {
                last.value += item.value;
            } else {
                combined.push(item);
            }
        }
        combined.retain(|item| item.value != 0.0);

        let mut colptr = vec![0usize; self.columns + 1];
        for item in &combined {
            colptr[item.column + 1] += 1;
        }
        for column in 0..self.columns {
            colptr[column + 1] += colptr[column];
        }
        let rowval = combined.iter().map(|item| item.row).collect();
        let nzval = combined.iter().map(|item| item.value).collect();
        let matrix = CscMatrix::new(self.rows, self.columns, colptr, rowval, nzval);
        matrix
            .check_format()
            .map_err(|error| format!("invalid generated sparse matrix: {error:?}"))?;
        Ok((matrix, self.rhs, self.cones))
    }
}

fn x_index(station: usize) -> usize {
    station
}

fn v_index(stations: usize, station: usize) -> usize {
    stations + station
}

fn time_index(stations: usize, segment: usize) -> usize {
    2 * stations + segment
}

fn coarsen_model(mut model: Model, factor: usize) -> Result<Model, Box<dyn Error>> {
    if factor == 1 {
        return Ok(model);
    }
    if factor == 0 {
        return Err("coarsening factor must be positive".into());
    }
    let source = std::mem::take(&mut model.segments);
    let mut segments = Vec::with_capacity(source.len().div_ceil(factor));
    let mut first = 0;
    while first < source.len() {
        let piece = source[first].piece;
        let mut piece_end = first + 1;
        while piece_end < source.len() && source[piece_end].piece == piece {
            piece_end += 1;
        }
        if !(piece_end - first).is_multiple_of(factor) {
            return Err(format!(
                "piece {piece} has {} intervals, not divisible by coarsening factor {factor}",
                piece_end - first
            )
            .into());
        }
        for group_first in (first..piece_end).step_by(factor) {
            let group = &source[group_first..group_first + factor];
            let length = group.iter().map(|segment| segment.length).sum::<f64>();
            let mut tangent = [0.0; AXES];
            let mut curvature = [0.0; AXES];
            let mut curvature_derivative = [0.0; AXES];
            for segment in group {
                let weight = segment.length / length;
                for axis in 0..AXES {
                    tangent[axis] += weight * segment.tangent[axis];
                    curvature[axis] += weight * segment.curvature[axis];
                    curvature_derivative[axis] += weight * segment.curvature_derivative[axis];
                }
            }
            segments.push(Segment {
                piece,
                input: group[0].input,
                length,
                velocity_limit: group
                    .iter()
                    .map(|segment| segment.velocity_limit)
                    .fold(f64::INFINITY, f64::min),
                tangent,
                curvature,
                curvature_derivative,
            });
        }
        first = piece_end;
    }
    model.segments = segments;
    Ok(model)
}

fn refine_model(mut model: Model, factor: usize) -> Result<Model, Box<dyn Error>> {
    if factor == 0 {
        return Err("refinement factor must be positive".into());
    }
    if factor == 1 {
        return Ok(model);
    }
    let source = std::mem::take(&mut model.segments);
    let mut segments = Vec::with_capacity(source.len() * factor);
    for segment in source {
        for _ in 0..factor {
            let mut refined = segment.clone();
            refined.length /= factor as f64;
            segments.push(refined);
        }
    }
    model.segments = segments;
    Ok(model)
}

fn jerk_slack_index(stations: usize, segments: usize) -> usize {
    2 * stations + segments
}

fn scalar_acceleration(model: &Model, squared_velocity: &[f64], segment: usize) -> f64 {
    (squared_velocity[segment + 1] - squared_velocity[segment])
        / (2.0 * model.segments[segment].length)
}

fn segment_time_from_squared_speed(model: &Model, squared_velocity: &[f64], segment: usize) -> f64 {
    let from = squared_velocity[segment].max(0.0).sqrt();
    let to = squared_velocity[segment + 1].max(0.0).sqrt();
    2.0 * model.segments[segment].length / (from + to).max(1e-10)
}

fn discrete_jerk(model: &Model, squared_velocity: &[f64], station: usize) -> [f64; AXES] {
    let segments = model.segments.len();
    let (scalar_jerk, scalar_acceleration, velocity, left_geometry, right_geometry) =
        if station == 0 {
            let acceleration = scalar_acceleration(model, squared_velocity, 0);
            let time = segment_time_from_squared_speed(model, squared_velocity, 0);
            (acceleration / (0.5 * time), 0.5 * acceleration, 0.0, 0, 0)
        } else if station == segments {
            let last = segments - 1;
            let acceleration = scalar_acceleration(model, squared_velocity, last);
            let time = segment_time_from_squared_speed(model, squared_velocity, last);
            (
                -acceleration / (0.5 * time),
                0.5 * acceleration,
                0.0,
                last,
                last,
            )
        } else {
            let left_acceleration = scalar_acceleration(model, squared_velocity, station - 1);
            let right_acceleration = scalar_acceleration(model, squared_velocity, station);
            let elapsed = 0.5
                * (segment_time_from_squared_speed(model, squared_velocity, station - 1)
                    + segment_time_from_squared_speed(model, squared_velocity, station));
            (
                (right_acceleration - left_acceleration) / elapsed.max(1e-10),
                0.5 * (left_acceleration + right_acceleration),
                squared_velocity[station].max(0.0).sqrt(),
                station - 1,
                station,
            )
        };
    let left = &model.segments[left_geometry];
    let right = &model.segments[right_geometry];
    let mut result = [0.0; AXES];
    for axis in 0..AXES {
        let tangent = 0.5 * (left.tangent[axis] + right.tangent[axis]);
        let curvature = 0.5 * (left.curvature[axis] + right.curvature[axis]);
        let curvature_derivative =
            0.5 * (left.curvature_derivative[axis] + right.curvature_derivative[axis]);
        result[axis] = tangent * scalar_jerk
            + 3.0 * curvature * velocity * scalar_acceleration
            + curvature_derivative * velocity * velocity * velocity;
    }
    result
}

fn jerk_linearization(
    model: &Model,
    reference: &mut [f64],
    station: usize,
) -> ([f64; AXES], Vec<(usize, [f64; AXES])>) {
    let value = discrete_jerk(model, reference, station);
    let first = station.saturating_sub(1);
    let last = (station + 1).min(reference.len() - 1);
    let mut derivatives = Vec::with_capacity(last - first + 1);
    for variable in first..=last {
        let scale = reference[variable].abs().max(1.0);
        let step = 1e-5 * scale;
        let original = reference[variable];
        reference[variable] = original + step;
        let upper_value = discrete_jerk(model, reference, station);
        let mut derivative = [0.0; AXES];
        if original >= step {
            reference[variable] = original - step;
            let lower_value = discrete_jerk(model, reference, station);
            for axis in 0..AXES {
                derivative[axis] = (upper_value[axis] - lower_value[axis]) / (2.0 * step);
            }
        } else {
            for axis in 0..AXES {
                derivative[axis] = (upper_value[axis] - value[axis]) / step;
            }
        }
        reference[variable] = original;
        derivatives.push((variable, derivative));
    }
    (value, derivatives)
}

struct JerkSubproblem<'a> {
    reference: &'a [f64],
    trust_fraction: f64,
}

#[derive(Clone)]
struct OracleSolution {
    duration: f64,
    squared_velocity: Vec<f64>,
    velocity: Vec<f64>,
    segment_time: Vec<f64>,
    solve_time: f64,
    iterations: u32,
    jerk_slack: f64,
}

#[derive(Clone, Copy)]
struct JerkViolation {
    maximum_ratio: f64,
    station: usize,
    path_ratio: f64,
    axis_ratio: f64,
    axis: usize,
}

fn maximum_jerk_violation(model: &Model, squared_velocity: &[f64]) -> JerkViolation {
    let mut worst = JerkViolation {
        maximum_ratio: 0.0,
        station: 0,
        path_ratio: 0.0,
        axis_ratio: 0.0,
        axis: 0,
    };
    for station in 0..squared_velocity.len() {
        let jerk = discrete_jerk(model, squared_velocity, station);
        let path_ratio =
            jerk.iter().map(|value| value * value).sum::<f64>().sqrt() / model.path_jerk;
        let (axis, axis_ratio) = jerk
            .iter()
            .enumerate()
            .filter(|(axis, _)| model.axis_jerk[*axis].is_finite())
            .map(|(axis, value)| (axis, value.abs() / model.axis_jerk[axis]))
            .max_by(|left, right| left.1.partial_cmp(&right.1).unwrap_or(Ordering::Equal))
            .unwrap_or((0, 0.0));
        let maximum_ratio = path_ratio.max(axis_ratio);
        if maximum_ratio > worst.maximum_ratio {
            worst = JerkViolation {
                maximum_ratio,
                station,
                path_ratio,
                axis_ratio,
                axis,
            };
        }
    }
    worst
}

fn solve(model: &Model, jerk: Option<&JerkSubproblem>) -> Result<OracleSolution, Box<dyn Error>> {
    let segments = model.segments.len();
    let stations = segments + 1;
    let variables = 2 * stations + segments + usize::from(jerk.is_some());
    let mut station_cap = vec![f64::INFINITY; stations];
    for (index, segment) in model.segments.iter().enumerate() {
        station_cap[index] = station_cap[index].min(segment.velocity_limit);
        station_cap[index + 1] = station_cap[index + 1].min(segment.velocity_limit);
    }

    let mut builder = ConstraintBuilder::new(variables);

    let equality_rows = vec![
        vec![(x_index(0), 1.0)],
        vec![(x_index(stations - 1), 1.0)],
        vec![(v_index(stations, 0), 1.0)],
        vec![(v_index(stations, stations - 1), 1.0)],
    ];
    builder.add_block(&equality_rows, &[0.0, 0.0, 0.0, 0.0], ZeroConeT(4));

    let mut linear_rows = Vec::new();
    let mut linear_rhs = Vec::new();
    for (station, cap) in station_cap.iter().copied().enumerate() {
        linear_rows.push(vec![(x_index(station), -1.0)]);
        linear_rhs.push(0.0);
        linear_rows.push(vec![(v_index(stations, station), -1.0)]);
        linear_rhs.push(0.0);
        linear_rows.push(vec![(x_index(station), 1.0)]);
        linear_rhs.push(cap * cap);
    }
    for segment in 0..segments {
        linear_rows.push(vec![(time_index(stations, segment), -1.0)]);
        linear_rhs.push(0.0);
    }
    if let Some(jerk) = jerk {
        let slack = jerk_slack_index(stations, segments);
        linear_rows.push(vec![(slack, -1.0)]);
        linear_rhs.push(0.0);
        for station in 0..stations {
            let radius = jerk.trust_fraction * station_cap[station] * station_cap[station];
            let lower = (jerk.reference[station] - radius).max(0.0);
            let upper =
                (jerk.reference[station] + radius).min(station_cap[station] * station_cap[station]);
            linear_rows.push(vec![(x_index(station), -1.0)]);
            linear_rhs.push(-lower);
            linear_rows.push(vec![(x_index(station), 1.0)]);
            linear_rhs.push(upper);
        }
    }

    for (segment_index, segment) in model.segments.iter().enumerate() {
        let inverse_twice_length = 1.0 / (2.0 * segment.length);
        for axis in 0..AXES {
            let limit = model.axis_acceleration[axis];
            if !limit.is_finite() {
                continue;
            }
            let left =
                -segment.tangent[axis] * inverse_twice_length + 0.5 * segment.curvature[axis];
            let right =
                segment.tangent[axis] * inverse_twice_length + 0.5 * segment.curvature[axis];
            linear_rows.push(vec![
                (x_index(segment_index), left),
                (x_index(segment_index + 1), right),
            ]);
            linear_rhs.push(limit);
            linear_rows.push(vec![
                (x_index(segment_index), -left),
                (x_index(segment_index + 1), -right),
            ]);
            linear_rhs.push(limit);
        }
    }
    let linear_count = linear_rows.len();
    builder.add_block(&linear_rows, &linear_rhs, NonnegativeConeT(linear_count));

    if let Some(jerk) = jerk {
        let slack = jerk_slack_index(stations, segments);
        let mut perturbed_reference = jerk.reference.to_vec();
        for station in 0..stations {
            let (value, derivatives) = jerk_linearization(model, &mut perturbed_reference, station);
            let mut constant = value;
            for (variable, derivative) in &derivatives {
                for axis in 0..AXES {
                    constant[axis] -= derivative[axis] * jerk.reference[*variable];
                }
            }

            let mut rows = vec![Vec::new(); AXES + 1];
            let mut rhs = vec![0.0; AXES + 1];
            rows[0].push((slack, -1.0));
            rhs[0] = 1.0;
            for axis in 0..AXES {
                rhs[axis + 1] = constant[axis] / model.path_jerk;
                for (variable, derivative) in &derivatives {
                    rows[axis + 1].push((*variable, -derivative[axis] / model.path_jerk));
                }
            }
            builder.add_block(&rows, &rhs, SecondOrderConeT(AXES + 1));

            let mut axis_rows = Vec::new();
            let mut axis_rhs = Vec::new();
            for axis in 0..AXES {
                let limit = model.axis_jerk[axis];
                if !limit.is_finite() {
                    continue;
                }
                let positive = derivatives
                    .iter()
                    .map(|(variable, derivative)| (*variable, derivative[axis] / limit))
                    .chain(std::iter::once((slack, -1.0)))
                    .collect();
                axis_rows.push(positive);
                axis_rhs.push(1.0 - constant[axis] / limit);
                let negative = derivatives
                    .iter()
                    .map(|(variable, derivative)| (*variable, -derivative[axis] / limit))
                    .chain(std::iter::once((slack, -1.0)))
                    .collect();
                axis_rows.push(negative);
                axis_rhs.push(1.0 + constant[axis] / limit);
            }
            if !axis_rows.is_empty() {
                let count = axis_rows.len();
                builder.add_block(&axis_rows, &axis_rhs, NonnegativeConeT(count));
            }
        }
    }

    // Rotated cone v^2 <= x, represented as ||(2v, x-1)|| <= x+1.
    for station in 0..stations {
        let x = x_index(station);
        let velocity = v_index(stations, station);
        builder.add_block(
            &[vec![(x, -1.0)], vec![(velocity, -2.0)], vec![(x, -1.0)]],
            &[1.0, 0.0, -1.0],
            SecondOrderConeT(3),
        );
    }

    // Exact constant-path-acceleration segment time:
    // dt = 2 ds / (v_i + v_{i+1}).  The rotated cone enforces
    // dt*(v_i+v_{i+1}) >= 2 ds.
    for (segment_index, segment) in model.segments.iter().enumerate() {
        let time = time_index(stations, segment_index);
        let from_velocity = v_index(stations, segment_index);
        let to_velocity = v_index(stations, segment_index + 1);
        builder.add_block(
            &[
                vec![(time, -1.0), (from_velocity, -1.0), (to_velocity, -1.0)],
                vec![],
                vec![(time, -1.0), (from_velocity, 1.0), (to_velocity, 1.0)],
            ],
            &[0.0, 2.0 * (2.0 * segment.length).sqrt(), 0.0],
            SecondOrderConeT(3),
        );
    }

    // Midpoint axis acceleration is affine in adjacent squared speeds:
    // a = q'(s)*(x_{i+1}-x_i)/(2 ds) + q''(s)*(x_i+x_{i+1})/2.
    for (segment_index, segment) in model.segments.iter().enumerate() {
        let inverse_twice_length = 1.0 / (2.0 * segment.length);
        let mut rows = vec![Vec::new(); AXES + 1];
        let mut rhs = vec![0.0; AXES + 1];
        rhs[0] = model.path_acceleration;
        for axis in 0..AXES {
            let left =
                -segment.tangent[axis] * inverse_twice_length + 0.5 * segment.curvature[axis];
            let right =
                segment.tangent[axis] * inverse_twice_length + 0.5 * segment.curvature[axis];
            // Slack b-Ax must contain the acceleration vector.
            rows[axis + 1].push((x_index(segment_index), -left));
            rows[axis + 1].push((x_index(segment_index + 1), -right));
        }
        builder.add_block(&rows, &rhs, SecondOrderConeT(AXES + 1));
    }

    let (a, b, cones) = builder.finish()?;
    let p = CscMatrix::<f64>::spalloc((variables, variables), 0);
    let mut q = vec![0.0; variables];
    for segment in 0..segments {
        q[time_index(stations, segment)] = 1.0;
    }
    if jerk.is_some() {
        q[jerk_slack_index(stations, segments)] = model.planner_duration * 1000.0;
    }
    let settings = DefaultSettingsBuilder::default()
        .verbose(false)
        .tol_gap_abs(1e-9)
        .tol_gap_rel(1e-9)
        .tol_feas(1e-9)
        .max_iter(300)
        .build()?;
    let mut solver = DefaultSolver::new(&p, &q, &a, &b, &cones, settings)?;
    solver.solve();
    if !matches!(
        solver.solution.status,
        SolverStatus::Solved | SolverStatus::AlmostSolved
    ) {
        return Err(format!(
            "Clarabel failed: status={:?}, iterations={}, solve_time={} s",
            solver.solution.status, solver.solution.iterations, solver.solution.solve_time
        )
        .into());
    }
    let squared_velocity = solver.solution.x[0..stations].to_vec();
    let velocity = solver.solution.x[stations..2 * stations].to_vec();
    let segment_time = solver.solution.x[2 * stations..].to_vec();
    let duration = segment_time.iter().sum();
    let jerk_slack = if jerk.is_some() {
        solver.solution.x[jerk_slack_index(stations, segments)]
    } else {
        0.0
    };
    Ok(OracleSolution {
        duration,
        squared_velocity,
        velocity,
        segment_time,
        solve_time: solver.solution.solve_time,
        iterations: solver.solution.iterations,
        jerk_slack,
    })
}

fn solve_jerk_aware(model: &Model) -> Result<(OracleSolution, JerkViolation, u32), Box<dyn Error>> {
    let acceleration_solution = solve(model, None)?;
    let initial_violation = maximum_jerk_violation(model, &acceleration_solution.squared_velocity);
    println!(
        "jerk SCP seed: duration={:.9} s maximum discrete jerk ratio={:.6}",
        acceleration_solution.duration, initial_violation.maximum_ratio
    );

    let mut reference = acceleration_solution.squared_velocity.clone();
    let mut trust_fraction = 0.15;
    let mut previous_ratio = initial_violation.maximum_ratio;
    let mut best: Option<(OracleSolution, JerkViolation)> = None;
    let mut total_solve_time = acceleration_solution.solve_time;
    let mut total_iterations = acceleration_solution.iterations;
    let mut completed_scp_iterations = 0;
    const MAX_SCP_ITERATIONS: u32 = 12;
    for iteration in 1..=MAX_SCP_ITERATIONS {
        let subproblem = JerkSubproblem {
            reference: &reference,
            trust_fraction,
        };
        completed_scp_iterations = iteration;
        let candidate = match solve(model, Some(&subproblem)) {
            Ok(candidate) => candidate,
            Err(error) => {
                trust_fraction *= 0.5;
                println!(
                    "jerk SCP iteration {iteration}: subproblem failed ({error}); shrinking trust to {trust_fraction:.5}"
                );
                if trust_fraction < 1e-4 {
                    break;
                }
                continue;
            }
        };
        total_solve_time += candidate.solve_time;
        total_iterations += candidate.iterations;
        let violation = maximum_jerk_violation(model, &candidate.squared_velocity);
        let maximum_change = candidate
            .squared_velocity
            .iter()
            .zip(&reference)
            .map(|(value, old)| (value - old).abs() / old.abs().max(1.0))
            .fold(0.0, f64::max);
        println!(
            "jerk SCP iteration {iteration}: duration={:.9} s linearized_slack={:.3e} actual_ratio={:.6} trust={:.5} change={:.3e}",
            candidate.duration,
            candidate.jerk_slack,
            violation.maximum_ratio,
            trust_fraction,
            maximum_change
        );

        if violation.maximum_ratio <= 1.001 {
            let replace = best
                .as_ref()
                .map(|(solution, _)| candidate.duration < solution.duration)
                .unwrap_or(true);
            if replace {
                best = Some((candidate.clone(), violation));
            } else if maximum_change <= 1e-4 {
                break;
            }
        }

        if violation.maximum_ratio > previous_ratio * 1.1 {
            trust_fraction *= 0.5;
            if trust_fraction < 1e-4 {
                break;
            }
            continue;
        }
        previous_ratio = violation.maximum_ratio;
        reference = candidate.squared_velocity;
        trust_fraction = (trust_fraction * 1.15).min(0.25);
    }
    let Some((mut solution, violation)) = best else {
        return Err(format!(
            "jerk-aware Clarabel SCP did not produce a discretely jerk-feasible profile; final maximum ratio={previous_ratio:.6}"
        )
        .into());
    };
    solution.solve_time = total_solve_time;
    solution.iterations = total_iterations;
    Ok((solution, violation, completed_scp_iterations))
}

fn acceleration(model: &Model, solution: &OracleSolution, segment_index: usize) -> [f64; AXES] {
    let segment = &model.segments[segment_index];
    let left = solution.squared_velocity[segment_index];
    let right = solution.squared_velocity[segment_index + 1];
    let scalar_acceleration = (right - left) / (2.0 * segment.length);
    let squared_speed = 0.5 * (left + right);
    let mut result = [0.0; AXES];
    for (axis, value) in result.iter_mut().enumerate() {
        *value =
            segment.tangent[axis] * scalar_acceleration + segment.curvature[axis] * squared_speed;
    }
    result
}

fn write_solution(
    path: &Path,
    model: &Model,
    solution: &OracleSolution,
) -> Result<(), Box<dyn Error>> {
    let file = fs::File::create(path)?;
    let mut output = BufWriter::new(file);
    writeln!(
        output,
        "segment,piece,input,length,velocity_from,velocity_to,scalar_acceleration,duration,acceleration_norm"
    )?;
    for (index, segment) in model.segments.iter().enumerate() {
        let scalar_acceleration = (solution.squared_velocity[index + 1]
            - solution.squared_velocity[index])
            / (2.0 * segment.length);
        let axis_acceleration = acceleration(model, solution, index);
        let norm = axis_acceleration
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        writeln!(
            output,
            "{index},{},{},{},{},{},{},{},{}",
            segment.piece,
            segment.input,
            segment.length,
            solution.velocity[index],
            solution.velocity[index + 1],
            scalar_acceleration,
            solution.segment_time[index],
            norm
        )?;
    }
    output.flush()?;
    Ok(())
}

fn oracle_station_acceleration(model: &Model, solution: &OracleSolution, station: usize) -> f64 {
    if station == 0 || station + 1 == solution.squared_velocity.len() {
        return 0.0;
    }
    0.5 * (scalar_acceleration(model, &solution.squared_velocity, station - 1)
        + scalar_acceleration(model, &solution.squared_velocity, station))
}

fn report_profile_gaps(model: &Model, solution: &OracleSolution) {
    if model.pieces.is_empty() {
        return;
    }
    struct Gap {
        piece: usize,
        input: usize,
        planner_duration: f64,
        oracle_duration: f64,
        length: f64,
        velocity_limit: f64,
        acceleration_limit: f64,
        jerk_limit: f64,
        entry_velocity_gap: f64,
        exit_velocity_gap: f64,
        entry_acceleration_gap: f64,
        exit_acceleration_gap: f64,
    }
    let mut first_segment = vec![usize::MAX; model.pieces.len()];
    let mut past_last_segment = vec![0; model.pieces.len()];
    for (segment, geometry) in model.segments.iter().enumerate() {
        first_segment[geometry.piece] = first_segment[geometry.piece].min(segment);
        past_last_segment[geometry.piece] = segment + 1;
    }
    let mut gaps = Vec::with_capacity(model.pieces.len());
    for (piece_index, piece) in model.pieces.iter().enumerate() {
        let first = first_segment[piece_index];
        let last = past_last_segment[piece_index];
        if first == usize::MAX || last <= first {
            continue;
        }
        let oracle_duration = solution.segment_time[first..last].iter().sum::<f64>();
        gaps.push(Gap {
            piece: piece_index,
            input: piece.input,
            planner_duration: piece.duration,
            oracle_duration,
            length: piece.length,
            velocity_limit: piece.velocity_limit,
            acceleration_limit: piece.acceleration_limit,
            jerk_limit: piece.jerk_limit,
            entry_velocity_gap: solution.velocity[first] - piece.entry_velocity,
            exit_velocity_gap: solution.velocity[last] - piece.exit_velocity,
            entry_acceleration_gap: oracle_station_acceleration(model, solution, first)
                - piece.entry_acceleration,
            exit_acceleration_gap: oracle_station_acceleration(model, solution, last)
                - piece.exit_acceleration,
        });
    }
    gaps.sort_by(|left, right| {
        let left_gap = left.planner_duration - left.oracle_duration;
        let right_gap = right.planner_duration - right.oracle_duration;
        right_gap.partial_cmp(&left_gap).unwrap_or(Ordering::Equal)
    });
    println!("largest planner-versus-oracle piece duration gaps:");
    for gap in gaps.iter().take(12) {
        println!(
            "  piece {} input {}: dt={:+.6} s planner={:.6} oracle={:.6} length={:.5} limits=[v={:.5} a={:.5} j={:.5}] dv=[{:+.5},{:+.5}] da=[{:+.5},{:+.5}]",
            gap.piece,
            gap.input,
            gap.planner_duration - gap.oracle_duration,
            gap.planner_duration,
            gap.oracle_duration,
            gap.length,
            gap.velocity_limit,
            gap.acceleration_limit,
            gap.jerk_limit,
            gap.entry_velocity_gap,
            gap.exit_velocity_gap,
            gap.entry_acceleration_gap,
            gap.exit_acceleration_gap
        );
    }
}

fn report(model: &Model, solution: &OracleSolution, jerk: Option<(JerkViolation, u32)>) {
    let (worst_segment, worst_norm) = model
        .segments
        .iter()
        .enumerate()
        .map(|(index, _)| {
            let values = acceleration(model, solution, index);
            let norm = values.iter().map(|value| value * value).sum::<f64>().sqrt();
            (index, norm)
        })
        .max_by(|left, right| left.1.partial_cmp(&right.1).unwrap_or(Ordering::Equal))
        .unwrap();
    let maximum_velocity = solution.velocity.iter().copied().fold(0.0, f64::max);
    let gap = model.planner_duration - solution.duration;
    println!("Clarabel status: solved");
    println!(
        "model: {} segments, {} stations, {}",
        model.segments.len(),
        model.segments.len() + 1,
        if jerk.is_some() {
            "jerk-aware finite-difference collocation"
        } else {
            "acceleration-only midpoint discretization"
        }
    );
    println!(
        "current jerk-limited planner duration: {:.9} s",
        model.planner_duration
    );
    println!(
        "Clarabel {} duration:  {:.9} s",
        if jerk.is_some() {
            "jerk-aware SCP"
        } else {
            "acceleration-only"
        },
        solution.duration
    );
    println!(
        "available duration gap:              {:.9} s ({:.3}%)",
        gap,
        100.0 * gap / model.planner_duration
    );
    println!(
        "maximum oracle station velocity:     {:.9}",
        maximum_velocity
    );
    println!(
        "maximum midpoint acceleration norm:  {:.9} / {:.9} at segment {} (piece {}, input {})",
        worst_norm,
        model.path_acceleration,
        worst_segment,
        model.segments[worst_segment].piece,
        model.segments[worst_segment].input
    );
    println!(
        "Clarabel solve: {} iterations, {:.6} s",
        solution.iterations, solution.solve_time
    );
    if let Some((violation, iterations)) = jerk {
        println!(
            "maximum discrete jerk ratio:       {:.9} at station {} (path {:.9}, axis {} {:.9})",
            violation.maximum_ratio,
            violation.station,
            violation.path_ratio,
            violation.axis,
            violation.axis_ratio
        );
        println!("sequential convex iterations:     {iterations}");
        println!(
            "NOTE: jerk constraints are finite-difference collocation constraints solved by sequential convexification. This is a locally converged reference, not a certified global optimum or executable PlanChunk trajectory."
        );
        report_profile_gaps(model, solution);
    } else {
        println!(
            "NOTE: this oracle enforces velocity caps and acceleration but not dynamic jerk; its duration is an optimistic comparison, not an executable trajectory."
        );
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<_> = env::args_os().collect();
    if arguments.len() < 2 || arguments.len() > 6 {
        eprintln!(
            "usage: ngc-clarabel-trajectory-oracle <model.txt> [solution.csv] [--jerk-aware] [--coarsen=N] [--refine=N]"
        );
        std::process::exit(2);
    }
    let mut jerk_aware = false;
    let mut coarsening = 1;
    let mut refinement = 1;
    let mut output_path = None;
    for argument in &arguments[2..] {
        if argument == "--jerk-aware" {
            jerk_aware = true;
        } else if let Some(value) = argument.to_string_lossy().strip_prefix("--coarsen=") {
            coarsening = value.parse::<usize>()?;
        } else if let Some(value) = argument.to_string_lossy().strip_prefix("--refine=") {
            refinement = value.parse::<usize>()?;
        } else if output_path.is_none() {
            output_path = Some(argument);
        } else {
            return Err("only one solution CSV path may be supplied".into());
        }
    }
    if coarsening != 1 && refinement != 1 {
        return Err("coarsening and refinement cannot be requested together".into());
    }
    let model = refine_model(
        coarsen_model(read_model(Path::new(&arguments[1]))?, coarsening)?,
        refinement,
    )?;
    let (solution, jerk_report) = if jerk_aware {
        let (solution, violation, iterations) = solve_jerk_aware(&model)?;
        (solution, Some((violation, iterations)))
    } else {
        (solve(&model, None)?, None)
    };
    report(&model, &solution, jerk_report);
    if let Some(path) = output_path {
        write_solution(Path::new(path), &model, &solution)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_rest_to_rest_matches_analytic_acceleration_limited_time() {
        let segments = (0..10)
            .map(|_| Segment {
                piece: 0,
                input: 0,
                length: 0.1,
                velocity_limit: 2.0,
                tangent: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                curvature: [0.0; AXES],
                curvature_derivative: [0.0; AXES],
            })
            .collect();
        let model = Model {
            path_acceleration: 20.0,
            axis_acceleration: [f64::INFINITY; AXES],
            path_jerk: 100.0,
            axis_jerk: [f64::INFINITY; AXES],
            planner_duration: 0.7,
            pieces: Vec::new(),
            segments,
        };
        let solution = solve(&model, None).expect("straight-line SOCP should solve");
        assert!(
            (solution.duration - 0.6).abs() < 1e-6,
            "{}",
            solution.duration
        );
        assert!(
            solution.velocity[0].abs() < 1e-6,
            "{}",
            solution.velocity[0]
        );
        assert!(
            solution.velocity[10].abs() < 1e-6,
            "{}",
            solution.velocity[10]
        );
        assert!((solution.velocity[1] - 2.0).abs() < 1e-6);
        assert!((solution.velocity[9] - 2.0).abs() < 1e-6);
    }

    #[test]
    fn straight_jerk_aware_scp_converges_to_a_discretely_feasible_profile() {
        let segments = (0..40)
            .map(|_| Segment {
                piece: 0,
                input: 0,
                length: 0.025,
                velocity_limit: 2.0,
                tangent: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                curvature: [0.0; AXES],
                curvature_derivative: [0.0; AXES],
            })
            .collect();
        let model = Model {
            path_acceleration: 20.0,
            axis_acceleration: [f64::INFINITY; AXES],
            path_jerk: 100.0,
            axis_jerk: [f64::INFINITY; AXES],
            planner_duration: 1.0,
            pieces: Vec::new(),
            segments,
        };
        let acceleration = solve(&model, None).expect("acceleration seed should solve");
        let (jerk_aware, violation, _) =
            solve_jerk_aware(&model).expect("jerk-aware SCP should converge");
        assert!(
            violation.maximum_ratio <= 1.001,
            "{}",
            violation.maximum_ratio
        );
        assert!(jerk_aware.duration > acceleration.duration);
    }
}
