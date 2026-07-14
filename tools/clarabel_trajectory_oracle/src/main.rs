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

#[derive(Debug)]
struct Segment {
    piece: usize,
    input: usize,
    length: f64,
    velocity_limit: f64,
    tangent: [f64; AXES],
    curvature: [f64; AXES],
}

#[derive(Debug)]
struct Model {
    path_acceleration: f64,
    axis_acceleration: [f64; AXES],
    planner_duration: f64,
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
    if lines.next() != Some("ngc_clarabel_acceleration_oracle_v1") {
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

    let duration_line = lines.next().ok_or("missing planner_duration")?;
    let duration_fields: Vec<_> = duration_line.split_whitespace().collect();
    if duration_fields.len() != 2 || duration_fields[0] != "planner_duration" {
        return Err("invalid planner_duration row".into());
    }
    let planner_duration = parse_f64(duration_fields[1], "planner_duration")?;

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
        if fields.len() != 17 || fields[0] != "segment" {
            return Err(format!("invalid segment row at model line {}", line_index + 6).into());
        }
        let piece = fields[1].parse::<usize>()?;
        let input = fields[2].parse::<usize>()?;
        let length = parse_f64(fields[3], "segment length")?;
        let velocity_limit = parse_f64(fields[4], "segment velocity limit")?;
        let mut tangent = [0.0; AXES];
        let mut curvature = [0.0; AXES];
        for axis in 0..AXES {
            tangent[axis] = parse_f64(fields[5 + axis], "tangent")?;
            curvature[axis] = parse_f64(fields[11 + axis], "curvature")?;
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
    if !planner_duration.is_finite() || planner_duration <= 0.0 {
        return Err("planner_duration must be finite and positive".into());
    }
    for (axis, limit) in axis_acceleration.iter().enumerate() {
        if *limit <= 0.0 {
            return Err(format!("axis acceleration {axis} must be positive").into());
        }
    }
    Ok(Model {
        path_acceleration,
        axis_acceleration,
        planner_duration,
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

struct OracleSolution {
    duration: f64,
    squared_velocity: Vec<f64>,
    velocity: Vec<f64>,
    segment_time: Vec<f64>,
    solve_time: f64,
    iterations: u32,
}

fn solve(model: &Model) -> Result<OracleSolution, Box<dyn Error>> {
    let segments = model.segments.len();
    let stations = segments + 1;
    let variables = 2 * stations + segments;
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
    Ok(OracleSolution {
        duration,
        squared_velocity,
        velocity,
        segment_time,
        solve_time: solver.solution.solve_time,
        iterations: solver.solution.iterations,
    })
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

fn report(model: &Model, solution: &OracleSolution) {
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
        "model: {} segments, {} stations, acceleration-only midpoint discretization",
        model.segments.len(),
        model.segments.len() + 1
    );
    println!(
        "current jerk-limited planner duration: {:.9} s",
        model.planner_duration
    );
    println!(
        "Clarabel acceleration-only duration:  {:.9} s",
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
    println!(
        "NOTE: this oracle enforces velocity caps and acceleration but not dynamic jerk; its duration is an optimistic comparison, not an executable trajectory."
    );
}

fn main() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<_> = env::args_os().collect();
    if arguments.len() < 2 || arguments.len() > 3 {
        eprintln!("usage: ngc-clarabel-trajectory-oracle <model.txt> [solution.csv]");
        std::process::exit(2);
    }
    let model = read_model(Path::new(&arguments[1]))?;
    let solution = solve(&model)?;
    report(&model, &solution);
    if let Some(path) = arguments.get(2) {
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
            })
            .collect();
        let model = Model {
            path_acceleration: 20.0,
            axis_acceleration: [f64::INFINITY; AXES],
            planner_duration: 0.7,
            segments,
        };
        let solution = solve(&model).expect("straight-line SOCP should solve");
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
}
