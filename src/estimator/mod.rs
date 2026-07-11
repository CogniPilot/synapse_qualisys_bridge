use std::collections::BTreeMap;

use synapse_fbs::topic::{ExternalOdometryFlags, ExternalOdometryStatus};

#[path = "generated/MocapExternalOdometryEstimator_solve.rs"]
mod mocap_external_odometry_estimator_solve;

use mocap_external_odometry_estimator_solve as generated;

const ATTITUDE_OFFSET: usize = 0;
const LINEAR_VELOCITY_OFFSET: usize = 4;
const POSITION_OFFSET: usize = 7;
const ANGULAR_VELOCITY_OFFSET: usize = 10;
const COVARIANCE_OFFSET: usize = 13;
const TANGENT_LEN: usize = 12;
const MEASUREMENT_LEN: usize = 6;
const MAX_PREDICTION_STEP_S: f64 = 0.01;
const MIN_VARIANCE: f64 = 1.0e-12;

type Vector12 = [f64; TANGENT_LEN];
type Vector6 = [f64; MEASUREMENT_LEN];
/// Covariance over the 12D tangent state, ordered as attitude (rad),
/// linear velocity (m/s), position (m), and angular velocity (rad/s) blocks.
pub type CovarianceMatrix = [[f64; TANGENT_LEN]; TANGENT_LEN];
type Matrix12 = CovarianceMatrix;
type Matrix6 = [[f64; MEASUREMENT_LEN]; MEASUREMENT_LEN];

#[derive(Debug, Clone, Copy)]
pub struct MocapExternalOdometryEstimatorConfig {
    pub max_gap_us: u64,
    pub min_dt_us: u64,
    pub position_stddev_m: f64,
    pub attitude_stddev_rad: f64,
    pub initial_linear_velocity_stddev_m_s: f64,
    pub initial_angular_velocity_stddev_rad_s: f64,
    pub attitude_process_variance: f64,
    pub velocity_process_variance: f64,
    pub position_process_variance: f64,
    pub angular_velocity_process_variance: f64,
    pub position_gate_m: f64,
    pub attitude_gate_rad: f64,
}

impl Default for MocapExternalOdometryEstimatorConfig {
    fn default() -> Self {
        Self {
            max_gap_us: 100_000,
            min_dt_us: 1_000,
            position_stddev_m: 0.002,
            attitude_stddev_rad: 0.01,
            initial_linear_velocity_stddev_m_s: 5.0,
            initial_angular_velocity_stddev_rad_s: 5.0,
            attitude_process_variance: 1.0e-6,
            velocity_process_variance: 1.0e-4,
            position_process_variance: 1.0e-8,
            angular_velocity_process_variance: 1.0e-4,
            position_gate_m: 1.0,
            attitude_gate_rad: 0.75,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MocapMeasurement {
    pub timestamp_us: u64,
    pub position_enu_m: [f64; 3],
    pub attitude_wxyz: [f64; 4],
    pub tracking_valid: bool,
    /// QTM delivered a finite pose, but reported transport quality loss.
    pub quality_degraded: bool,
}

#[derive(Debug, Clone)]
pub struct ExternalOdometryEstimate {
    pub timestamp_us: u64,
    pub position_enu_m: [f32; 3],
    pub attitude_wxyz: [f32; 4],
    pub linear_velocity_enu_m_s: [f32; 3],
    pub angular_velocity_flu_rad_s: [f32; 3],
    pub status: ExternalOdometryStatus,
    pub flags: ExternalOdometryFlags,
    pub covariance: CovarianceMatrix,
    /// Wraps when the filter re-initializes or jumps the state after the
    /// first acquisition.
    pub reset_counter: u8,
}

impl ExternalOdometryEstimate {
    fn lost(timestamp_us: u64) -> Self {
        Self {
            timestamp_us,
            position_enu_m: [0.0; 3],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            linear_velocity_enu_m_s: [0.0; 3],
            angular_velocity_flu_rad_s: [0.0; 3],
            status: ExternalOdometryStatus::Lost,
            flags: ExternalOdometryFlags::Lost,
            covariance: zero_matrix12(),
            reset_counter: 0,
        }
    }
}

#[derive(Debug)]
pub struct ExternalOdometryEstimators {
    config: MocapExternalOdometryEstimatorConfig,
    estimators: BTreeMap<i32, MocapExternalOdometryEstimator>,
}

impl ExternalOdometryEstimators {
    pub fn new(config: MocapExternalOdometryEstimatorConfig) -> Self {
        Self {
            config,
            estimators: BTreeMap::new(),
        }
    }

    pub fn update(
        &mut self,
        body_id: i32,
        measurement: MocapMeasurement,
    ) -> ExternalOdometryEstimate {
        self.estimators
            .entry(body_id)
            .or_insert_with(|| MocapExternalOdometryEstimator::new(self.config))
            .update(measurement)
    }
}

#[derive(Debug)]
struct MocapExternalOdometryEstimator {
    config: MocapExternalOdometryEstimatorConfig,
    y: generated::State,
    p: generated::Parameters,
    dydt: generated::Derivative,
    initialized: bool,
    timestamp_us: u64,
    last_mocap_timestamp_us: u64,
    frames_since_last_mocap_count: u8,
    twist_valid: bool,
    reset_counter: u8,
}

impl MocapExternalOdometryEstimator {
    fn new(config: MocapExternalOdometryEstimatorConfig) -> Self {
        debug_assert_eq!(
            generated::Y_LEN,
            COVARIANCE_OFFSET + TANGENT_LEN * TANGENT_LEN
        );
        debug_assert_eq!(generated::DERIVATIVE_LEN, generated::Y_LEN);

        let mut p = [0.0; generated::P_LEN];
        p[0] = config.attitude_process_variance;
        p[1] = config.velocity_process_variance;
        p[2] = config.position_process_variance;
        p[3] = config.angular_velocity_process_variance;

        Self {
            config,
            y: [0.0; generated::Y_LEN],
            p,
            dydt: [0.0; generated::DERIVATIVE_LEN],
            initialized: false,
            timestamp_us: 0,
            last_mocap_timestamp_us: 0,
            frames_since_last_mocap_count: 0,
            twist_valid: false,
            reset_counter: 0,
        }
    }

    fn update(&mut self, measurement: MocapMeasurement) -> ExternalOdometryEstimate {
        if !measurement.tracking_valid {
            return self.update_without_measurement(measurement);
        }

        if !self.initialized {
            self.initialize(measurement);
            return self.estimate(
                measurement.timestamp_us,
                measurement_status(&measurement),
                false,
                false,
            );
        }

        // Compare against the last accepted measurement, not the last prediction.  During a
        // tracking outage `timestamp_us` continues advancing, while this timestamp deliberately
        // remains at the last observed pose.
        let dt_us = measurement
            .timestamp_us
            .saturating_sub(self.last_mocap_timestamp_us);
        if dt_us > self.config.max_gap_us {
            self.initialize(measurement);
            return self.estimate(
                measurement.timestamp_us,
                measurement_status(&measurement),
                false,
                false,
            );
        }

        if dt_us >= self.config.min_dt_us {
            self.predict_to(measurement.timestamp_us);
        }

        let residual = self.measurement_residual(&measurement);
        if norm3([residual[0], residual[1], residual[2]]) > self.config.attitude_gate_rad
            || norm3([residual[3], residual[4], residual[5]]) > self.config.position_gate_m
        {
            // Rejected samples never advance last_mocap_timestamp_us, so
            // disagreement outlasting the dropout budget re-initializes via
            // the max-gap check above instead of rejecting forever.
            self.frames_since_last_mocap_count =
                self.frames_since_last_mocap_count.saturating_add(1);
            return self.estimate(
                measurement.timestamp_us,
                ExternalOdometryStatus::OutlierRejected,
                true,
                true,
            );
        }

        if self.correct(residual).is_some() {
            self.last_mocap_timestamp_us = measurement.timestamp_us;
            self.frames_since_last_mocap_count = 0;
            self.twist_valid = true;
            self.estimate(
                measurement.timestamp_us,
                measurement_status(&measurement),
                false,
                false,
            )
        } else {
            self.frames_since_last_mocap_count =
                self.frames_since_last_mocap_count.saturating_add(1);
            self.estimate(
                measurement.timestamp_us,
                ExternalOdometryStatus::Degraded,
                true,
                false,
            )
        }
    }

    fn update_without_measurement(
        &mut self,
        measurement: MocapMeasurement,
    ) -> ExternalOdometryEstimate {
        if !self.initialized {
            return ExternalOdometryEstimate::lost(measurement.timestamp_us);
        }

        self.frames_since_last_mocap_count = self.frames_since_last_mocap_count.saturating_add(1);
        let extrapolation_us = measurement
            .timestamp_us
            .saturating_sub(self.last_mocap_timestamp_us);
        if extrapolation_us <= self.config.max_gap_us {
            self.predict_to(measurement.timestamp_us);
            self.estimate(
                measurement.timestamp_us,
                ExternalOdometryStatus::ExtrapolatedShort,
                true,
                false,
            )
        } else {
            // Propagate at most to the loss horizon once.  Continuing to integrate an old twist
            // after QTM has declared the body lost makes a stale estimate appear to be live.
            let loss_horizon = self
                .last_mocap_timestamp_us
                .saturating_add(self.config.max_gap_us);
            self.predict_to(loss_horizon);
            self.y[LINEAR_VELOCITY_OFFSET..LINEAR_VELOCITY_OFFSET + 3].fill(0.0);
            self.y[ANGULAR_VELOCITY_OFFSET..ANGULAR_VELOCITY_OFFSET + 3].fill(0.0);
            self.timestamp_us = measurement.timestamp_us.max(self.timestamp_us);
            self.twist_valid = false;
            self.estimate(
                measurement.timestamp_us,
                ExternalOdometryStatus::Lost,
                false,
                false,
            )
        }
    }

    fn initialize(&mut self, measurement: MocapMeasurement) {
        if self.initialized {
            self.reset_counter = self.reset_counter.wrapping_add(1);
        }
        self.y.fill(0.0);
        self.y[ATTITUDE_OFFSET..ATTITUDE_OFFSET + 4]
            .copy_from_slice(&normalized_quat(measurement.attitude_wxyz));
        self.y[POSITION_OFFSET..POSITION_OFFSET + 3].copy_from_slice(&measurement.position_enu_m);

        let mut covariance = zero_matrix12();
        for i in 0..3 {
            covariance[i][i] = self.config.attitude_stddev_rad.powi(2);
            covariance[3 + i][3 + i] = self.config.initial_linear_velocity_stddev_m_s.powi(2);
            covariance[6 + i][6 + i] = self.config.position_stddev_m.powi(2);
            covariance[9 + i][9 + i] = self.config.initial_angular_velocity_stddev_rad_s.powi(2);
        }
        self.write_covariance(&covariance);

        self.initialized = true;
        self.timestamp_us = measurement.timestamp_us;
        self.last_mocap_timestamp_us = measurement.timestamp_us;
        self.frames_since_last_mocap_count = 0;
        self.twist_valid = false;
    }

    fn predict_to(&mut self, timestamp_us: u64) {
        if !self.initialized || timestamp_us <= self.timestamp_us {
            self.timestamp_us = timestamp_us.max(self.timestamp_us);
            return;
        }

        let dt_s = timestamp_us.saturating_sub(self.timestamp_us) as f64 * 1.0e-6;
        let steps = (dt_s / MAX_PREDICTION_STEP_S).ceil().max(1.0) as usize;
        let step_s = dt_s / steps as f64;
        for _ in 0..steps {
            generated::derivative_rhs_into(0.0, &self.y, &self.p, &mut self.dydt);
            for i in 0..generated::DERIVATIVE_LEN {
                self.y[i] += self.dydt[i] * step_s;
            }
            self.add_process_noise(step_s);
            self.normalize_attitude();
            self.symmetrize_covariance();
        }
        self.timestamp_us = timestamp_us;
    }

    fn add_process_noise(&mut self, dt_s: f64) {
        let variances = [
            self.config.attitude_process_variance,
            self.config.velocity_process_variance,
            self.config.position_process_variance,
            self.config.angular_velocity_process_variance,
        ];
        for axis in 0..3 {
            self.y[covariance_index(axis, axis)] += variances[0] * dt_s;
            self.y[covariance_index(3 + axis, 3 + axis)] += variances[1] * dt_s;
            self.y[covariance_index(6 + axis, 6 + axis)] += variances[2] * dt_s;
            self.y[covariance_index(9 + axis, 9 + axis)] += variances[3] * dt_s;
        }
    }

    fn measurement_residual(&self, measurement: &MocapMeasurement) -> Vector6 {
        let predicted_attitude = self.attitude();
        let measured_attitude = normalized_quat(measurement.attitude_wxyz);
        let attitude_residual = quat_log(quat_multiply(
            quat_conjugate(predicted_attitude),
            measured_attitude,
        ));

        let predicted_position = self.position();
        [
            attitude_residual[0],
            attitude_residual[1],
            attitude_residual[2],
            measurement.position_enu_m[0] - predicted_position[0],
            measurement.position_enu_m[1] - predicted_position[1],
            measurement.position_enu_m[2] - predicted_position[2],
        ]
    }

    fn correct(&mut self, residual: Vector6) -> Option<()> {
        let covariance = self.covariance();
        let observed = [0_usize, 1, 2, 6, 7, 8];
        let mut innovation_covariance = zero_matrix6();
        for i in 0..3 {
            for j in 0..MEASUREMENT_LEN {
                innovation_covariance[i][j] = covariance[observed[i]][observed[j]];
                innovation_covariance[3 + i][j] = covariance[observed[3 + i]][observed[j]];
            }
            innovation_covariance[i][i] += self.config.attitude_stddev_rad.powi(2);
            innovation_covariance[3 + i][3 + i] += self.config.position_stddev_m.powi(2);
        }

        let innovation_covariance_inv = inverse6(innovation_covariance)?;
        let mut gain = [[0.0; MEASUREMENT_LEN]; TANGENT_LEN];
        for row in 0..TANGENT_LEN {
            for measurement_col in 0..MEASUREMENT_LEN {
                for observed_row in 0..MEASUREMENT_LEN {
                    gain[row][measurement_col] += covariance[row][observed[observed_row]]
                        * innovation_covariance_inv[observed_row][measurement_col];
                }
            }
        }

        let mut correction = [0.0; TANGENT_LEN];
        for row in 0..TANGENT_LEN {
            for measurement_col in 0..MEASUREMENT_LEN {
                correction[row] += gain[row][measurement_col] * residual[measurement_col];
            }
        }

        self.inject(correction);

        let mut a = identity_matrix12();
        for row in 0..TANGENT_LEN {
            for measurement_col in 0..MEASUREMENT_LEN {
                a[row][observed[measurement_col]] -= gain[row][measurement_col];
            }
        }
        let mut measurement_noise = zero_matrix6();
        for i in 0..3 {
            measurement_noise[i][i] = self.config.attitude_stddev_rad.powi(2);
            measurement_noise[3 + i][3 + i] = self.config.position_stddev_m.powi(2);
        }
        let updated_covariance = joseph_update(&a, &covariance, &gain, &measurement_noise);
        self.write_covariance(&stabilized_covariance(updated_covariance));
        Some(())
    }

    fn inject(&mut self, correction: Vector12) {
        let attitude_delta = quat_exp([correction[0], correction[1], correction[2]]);
        let attitude = quat_multiply(self.attitude(), attitude_delta);
        self.y[ATTITUDE_OFFSET..ATTITUDE_OFFSET + 4].copy_from_slice(&normalized_quat(attitude));

        for i in 0..3 {
            self.y[LINEAR_VELOCITY_OFFSET + i] += correction[3 + i];
            self.y[POSITION_OFFSET + i] += correction[6 + i];
            self.y[ANGULAR_VELOCITY_OFFSET + i] += correction[9 + i];
        }
    }

    fn estimate(
        &self,
        timestamp_us: u64,
        status: ExternalOdometryStatus,
        extrapolated: bool,
        outlier_rejected: bool,
    ) -> ExternalOdometryEstimate {
        let pose_valid = self.initialized && !matches!(status, ExternalOdometryStatus::Lost);
        let twist_valid = pose_valid && self.twist_valid;
        let mut flags = ExternalOdometryFlags::empty();
        if pose_valid {
            flags |= ExternalOdometryFlags::PositionValid;
            flags |= ExternalOdometryFlags::AttitudeValid;
        }
        if twist_valid {
            flags |= ExternalOdometryFlags::LinearVelocityValid;
            flags |= ExternalOdometryFlags::AngularVelocityValid;
        }
        if extrapolated {
            flags |= ExternalOdometryFlags::Extrapolated;
        }
        if outlier_rejected {
            flags |= ExternalOdometryFlags::OutlierRejected;
        }
        if matches!(status, ExternalOdometryStatus::Lost) {
            flags |= ExternalOdometryFlags::Lost;
        }
        if matches!(status, ExternalOdometryStatus::Degraded) {
            flags |= ExternalOdometryFlags::Degraded;
        }

        ExternalOdometryEstimate {
            timestamp_us,
            position_enu_m: f32_array(self.position()),
            attitude_wxyz: f32_array4(self.attitude()),
            linear_velocity_enu_m_s: f32_array(self.linear_velocity()),
            angular_velocity_flu_rad_s: f32_array(self.angular_velocity()),
            status,
            flags,
            covariance: self.covariance(),
            reset_counter: self.reset_counter,
        }
    }

    fn normalize_attitude(&mut self) {
        let attitude = normalized_quat(self.attitude());
        self.y[ATTITUDE_OFFSET..ATTITUDE_OFFSET + 4].copy_from_slice(&attitude);
    }

    fn symmetrize_covariance(&mut self) {
        let covariance = stabilized_covariance(self.covariance());
        self.write_covariance(&covariance);
    }

    fn attitude(&self) -> [f64; 4] {
        [
            self.y[ATTITUDE_OFFSET],
            self.y[ATTITUDE_OFFSET + 1],
            self.y[ATTITUDE_OFFSET + 2],
            self.y[ATTITUDE_OFFSET + 3],
        ]
    }

    fn linear_velocity(&self) -> [f64; 3] {
        [
            self.y[LINEAR_VELOCITY_OFFSET],
            self.y[LINEAR_VELOCITY_OFFSET + 1],
            self.y[LINEAR_VELOCITY_OFFSET + 2],
        ]
    }

    fn position(&self) -> [f64; 3] {
        [
            self.y[POSITION_OFFSET],
            self.y[POSITION_OFFSET + 1],
            self.y[POSITION_OFFSET + 2],
        ]
    }

    fn angular_velocity(&self) -> [f64; 3] {
        [
            self.y[ANGULAR_VELOCITY_OFFSET],
            self.y[ANGULAR_VELOCITY_OFFSET + 1],
            self.y[ANGULAR_VELOCITY_OFFSET + 2],
        ]
    }

    fn covariance(&self) -> Matrix12 {
        let mut covariance = zero_matrix12();
        for row in 0..TANGENT_LEN {
            for col in 0..TANGENT_LEN {
                covariance[row][col] = self.y[covariance_index(row, col)];
            }
        }
        covariance
    }

    fn write_covariance(&mut self, covariance: &Matrix12) {
        for row in 0..TANGENT_LEN {
            for col in 0..TANGENT_LEN {
                self.y[covariance_index(row, col)] = covariance[row][col];
            }
        }
    }
}

fn measurement_status(measurement: &MocapMeasurement) -> ExternalOdometryStatus {
    if measurement.quality_degraded {
        ExternalOdometryStatus::Degraded
    } else {
        ExternalOdometryStatus::Filtered
    }
}

fn covariance_index(row: usize, col: usize) -> usize {
    COVARIANCE_OFFSET + col * TANGENT_LEN + row
}

fn zero_matrix12() -> Matrix12 {
    [[0.0; TANGENT_LEN]; TANGENT_LEN]
}

fn zero_matrix6() -> Matrix6 {
    [[0.0; MEASUREMENT_LEN]; MEASUREMENT_LEN]
}

fn identity_matrix12() -> Matrix12 {
    let mut identity = zero_matrix12();
    for (i, row) in identity.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    identity
}

fn stabilized_covariance(covariance: Matrix12) -> Matrix12 {
    let mut stabilized = covariance;
    for row in 0..TANGENT_LEN {
        for col in row + 1..TANGENT_LEN {
            let value = 0.5 * (stabilized[row][col] + stabilized[col][row]);
            stabilized[row][col] = value;
            stabilized[col][row] = value;
        }
        if !stabilized[row][row].is_finite() || stabilized[row][row] < MIN_VARIANCE {
            stabilized[row][row] = MIN_VARIANCE;
        }
    }
    stabilized
}

fn inverse6(mut matrix: Matrix6) -> Option<Matrix6> {
    let mut inverse = zero_matrix6();
    for (i, row) in inverse.iter_mut().enumerate() {
        row[i] = 1.0;
    }

    for pivot in 0..MEASUREMENT_LEN {
        let mut best = pivot;
        let mut best_abs = matrix[pivot][pivot].abs();
        for (row, values) in matrix.iter().enumerate().skip(pivot + 1) {
            let candidate = values[pivot].abs();
            if candidate > best_abs {
                best = row;
                best_abs = candidate;
            }
        }
        if best_abs < 1.0e-18 {
            return None;
        }
        if best != pivot {
            matrix.swap(best, pivot);
            inverse.swap(best, pivot);
        }

        let diag = matrix[pivot][pivot];
        for col in 0..MEASUREMENT_LEN {
            matrix[pivot][col] /= diag;
            inverse[pivot][col] /= diag;
        }
        for row in 0..MEASUREMENT_LEN {
            if row == pivot {
                continue;
            }
            let factor = matrix[row][pivot];
            for col in 0..MEASUREMENT_LEN {
                matrix[row][col] -= factor * matrix[pivot][col];
                inverse[row][col] -= factor * inverse[pivot][col];
            }
        }
    }
    Some(inverse)
}

fn joseph_update(
    a: &Matrix12,
    covariance: &Matrix12,
    gain: &[[f64; MEASUREMENT_LEN]; TANGENT_LEN],
    measurement_noise: &Matrix6,
) -> Matrix12 {
    let mut ap = zero_matrix12();
    for row in 0..TANGENT_LEN {
        for col in 0..TANGENT_LEN {
            for (inner, covariance_row) in covariance.iter().enumerate() {
                ap[row][col] += a[row][inner] * covariance_row[col];
            }
        }
    }

    let mut updated = zero_matrix12();
    for row in 0..TANGENT_LEN {
        for col in 0..TANGENT_LEN {
            for inner in 0..TANGENT_LEN {
                updated[row][col] += ap[row][inner] * a[col][inner];
            }
        }
    }

    let mut kr = [[0.0; MEASUREMENT_LEN]; TANGENT_LEN];
    for row in 0..TANGENT_LEN {
        for col in 0..MEASUREMENT_LEN {
            for (inner, noise_row) in measurement_noise.iter().enumerate() {
                kr[row][col] += gain[row][inner] * noise_row[col];
            }
        }
    }

    for row in 0..TANGENT_LEN {
        for col in 0..TANGENT_LEN {
            for inner in 0..MEASUREMENT_LEN {
                updated[row][col] += kr[row][inner] * gain[col][inner];
            }
        }
    }
    updated
}

fn normalized_quat(value: [f64; 4]) -> [f64; 4] {
    let norm =
        (value[0] * value[0] + value[1] * value[1] + value[2] * value[2] + value[3] * value[3])
            .sqrt();
    if norm <= f64::EPSILON || !norm.is_finite() {
        [1.0, 0.0, 0.0, 0.0]
    } else {
        [
            value[0] / norm,
            value[1] / norm,
            value[2] / norm,
            value[3] / norm,
        ]
    }
}

fn quat_conjugate(value: [f64; 4]) -> [f64; 4] {
    [value[0], -value[1], -value[2], -value[3]]
}

fn quat_multiply(lhs: [f64; 4], rhs: [f64; 4]) -> [f64; 4] {
    [
        lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3],
        lhs[0] * rhs[1] + lhs[1] * rhs[0] + lhs[2] * rhs[3] - lhs[3] * rhs[2],
        lhs[0] * rhs[2] - lhs[1] * rhs[3] + lhs[2] * rhs[0] + lhs[3] * rhs[1],
        lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0],
    ]
}

fn quat_exp(rotation_vector: [f64; 3]) -> [f64; 4] {
    let theta = (rotation_vector[0] * rotation_vector[0]
        + rotation_vector[1] * rotation_vector[1]
        + rotation_vector[2] * rotation_vector[2])
        .sqrt();
    if theta <= 1.0e-12 {
        return normalized_quat([
            1.0,
            0.5 * rotation_vector[0],
            0.5 * rotation_vector[1],
            0.5 * rotation_vector[2],
        ]);
    }

    let half_theta = 0.5 * theta;
    let scale = half_theta.sin() / theta;
    [
        half_theta.cos(),
        rotation_vector[0] * scale,
        rotation_vector[1] * scale,
        rotation_vector[2] * scale,
    ]
}

fn quat_log(value: [f64; 4]) -> [f64; 3] {
    let mut value = normalized_quat(value);
    if value[0] < 0.0 {
        value = [-value[0], -value[1], -value[2], -value[3]];
    }

    let vector_norm = (value[1] * value[1] + value[2] * value[2] + value[3] * value[3]).sqrt();
    if vector_norm <= 1.0e-12 {
        return [2.0 * value[1], 2.0 * value[2], 2.0 * value[3]];
    }

    let angle = 2.0 * vector_norm.atan2(value[0]);
    let scale = angle / vector_norm;
    [value[1] * scale, value[2] * scale, value[3] * scale]
}

fn norm3(value: [f64; 3]) -> f64 {
    (value[0] * value[0] + value[1] * value[1] + value[2] * value[2]).sqrt()
}

fn f32_array(value: [f64; 3]) -> [f32; 3] {
    [value[0] as f32, value[1] as f32, value[2] as f32]
}

fn f32_array4(value: [f64; 4]) -> [f32; 4] {
    [
        value[0] as f32,
        value[1] as f32,
        value[2] as f32,
        value[3] as f32,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generated_prediction_advances_position_from_velocity() {
        let mut estimator =
            MocapExternalOdometryEstimator::new(MocapExternalOdometryEstimatorConfig::default());
        estimator.initialize(MocapMeasurement {
            timestamp_us: 0,
            position_enu_m: [0.0, 0.0, 0.0],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            tracking_valid: true,
            quality_degraded: false,
        });
        estimator.y[LINEAR_VELOCITY_OFFSET] = 2.0;

        estimator.predict_to(500_000);

        assert!((estimator.position()[0] - 1.0).abs() < 1.0e-9);
        let covariance = estimator.covariance();
        assert!(
            covariance[3][6] > 0.0,
            "velocity-position covariance = {}",
            covariance[3][6]
        );
    }

    #[test]
    fn second_pose_measurement_makes_velocity_observable() {
        let mut estimator =
            MocapExternalOdometryEstimator::new(MocapExternalOdometryEstimatorConfig::default());
        let first = estimator.update(MocapMeasurement {
            timestamp_us: 1_000_000,
            position_enu_m: [0.0, 0.0, 0.0],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            tracking_valid: true,
            quality_degraded: false,
        });
        assert!(
            !first
                .flags
                .contains(ExternalOdometryFlags::LinearVelocityValid)
        );

        let second = estimator.update(MocapMeasurement {
            timestamp_us: 1_100_000,
            position_enu_m: [0.1, 0.0, 0.0],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            tracking_valid: true,
            quality_degraded: false,
        });

        assert!(
            second
                .flags
                .contains(ExternalOdometryFlags::LinearVelocityValid)
        );
        assert!(
            (second.linear_velocity_enu_m_s[0] - 1.0).abs() < 0.15,
            "estimated velocity x = {}",
            second.linear_velocity_enu_m_s[0]
        );
    }

    #[test]
    fn short_dropout_extrapolates_without_reinitializing() {
        let mut estimator =
            MocapExternalOdometryEstimator::new(MocapExternalOdometryEstimatorConfig::default());
        estimator.update(MocapMeasurement {
            timestamp_us: 1_000_000,
            position_enu_m: [0.0, 0.0, 0.0],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            tracking_valid: true,
            quality_degraded: false,
        });
        estimator.update(MocapMeasurement {
            timestamp_us: 1_100_000,
            position_enu_m: [0.1, 0.0, 0.0],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            tracking_valid: true,
            quality_degraded: false,
        });

        let dropout = estimator.update(MocapMeasurement {
            timestamp_us: 1_150_000,
            position_enu_m: [0.0, 0.0, 0.0],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            tracking_valid: false,
            quality_degraded: false,
        });

        assert_eq!(dropout.status, ExternalOdometryStatus::ExtrapolatedShort);
        assert!(dropout.flags.contains(ExternalOdometryFlags::Extrapolated));
        assert!(
            dropout.position_enu_m[0] > 0.1,
            "extrapolated x = {}",
            dropout.position_enu_m[0]
        );
    }

    #[test]
    fn lost_pose_freezes_at_the_extrapolation_horizon_and_reacquires() {
        let mut estimator =
            MocapExternalOdometryEstimator::new(MocapExternalOdometryEstimatorConfig::default());
        for (timestamp_us, x) in [(1_000_000, 0.0), (1_050_000, 0.05)] {
            estimator.update(MocapMeasurement {
                timestamp_us,
                position_enu_m: [x, 0.0, 0.0],
                attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
                tracking_valid: true,
                quality_degraded: false,
            });
        }
        let lost = estimator.update(MocapMeasurement {
            timestamp_us: 1_250_000,
            position_enu_m: [0.0, 0.0, 0.0],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            tracking_valid: false,
            quality_degraded: false,
        });
        let still_lost = estimator.update(MocapMeasurement {
            timestamp_us: 2_000_000,
            position_enu_m: [0.0, 0.0, 0.0],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            tracking_valid: false,
            quality_degraded: false,
        });
        assert_eq!(lost.status, ExternalOdometryStatus::Lost);
        assert_eq!(lost.position_enu_m, still_lost.position_enu_m);
        assert_eq!(still_lost.linear_velocity_enu_m_s, [0.0; 3]);

        let reacquired = estimator.update(MocapMeasurement {
            timestamp_us: 2_050_000,
            position_enu_m: [5.0, 0.0, 0.0],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            tracking_valid: true,
            quality_degraded: false,
        });
        assert_eq!(reacquired.status, ExternalOdometryStatus::Filtered);
        assert!((reacquired.position_enu_m[0] - 5.0).abs() < 1.0e-6);
    }

    #[test]
    fn persistent_outliers_reacquire_after_max_gap() {
        let config = MocapExternalOdometryEstimatorConfig::default();
        let mut estimator = MocapExternalOdometryEstimator::new(config);
        estimator.update(MocapMeasurement {
            timestamp_us: 1_000_000,
            position_enu_m: [0.0, 0.0, 0.0],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            tracking_valid: true,
            quality_degraded: false,
        });

        // A sustained 10 m jump with valid tracking: rejected while inside
        // the dropout budget, then re-acquired instead of locking out.
        let mut timestamp_us = 1_000_000;
        let mut last = None;
        while timestamp_us < 1_000_000 + config.max_gap_us + 8_000 {
            timestamp_us += 4_000;
            last = Some(estimator.update(MocapMeasurement {
                timestamp_us,
                position_enu_m: [10.0, 0.0, 0.0],
                attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
                tracking_valid: true,
                quality_degraded: false,
            }));
        }

        let last = last.expect("at least one update");
        assert_eq!(last.status, ExternalOdometryStatus::Filtered);
        assert!(
            (last.position_enu_m[0] - 10.0).abs() < 1.0e-3,
            "reacquired x = {}",
            last.position_enu_m[0]
        );
    }

    #[test]
    fn quaternion_log_uses_shortest_path() {
        let yaw = 90.0_f64.to_radians();
        let residual = quat_log(quat_exp([0.0, 0.0, yaw]));

        assert!(residual[0].abs() < 1.0e-12);
        assert!(residual[1].abs() < 1.0e-12);
        assert!((residual[2] - yaw).abs() < 1.0e-12);
    }
}
