#!/usr/bin/env bash
set -euo pipefail

# Reproduce the selected alpha-750 model against visual DROID on V1_03.
# The model path is fixed: direct R/v/p confidence plus learned global alpha.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python}"
DATA_ROOT="${DATA_ROOT:?Set DATA_ROOT to the prepared V1_03 cam0 directory}"
IMU_PRIOR="${IMU_PRIOR:?Set IMU_PRIOR to the rectified IMU prior CSV}"
GT_CSV="${GT_CSV:?Set GT_CSV to the V1_03 ground-truth CSV}"
ORIGINAL_WEIGHTS="${ORIGINAL_WEIGHTS:?Set ORIGINAL_WEIGHTS to droid.pth}"
LEARNED_WEIGHTS="${LEARNED_WEIGHTS:?Set LEARNED_WEIGHTS to the selected alpha-750 checkpoint}"
OUT_DIR="${OUT_DIR:?Set OUT_DIR to a new result directory}"
EVAL_PREFLIGHT_ONLY="${EVAL_PREFLIGHT_ONLY:-0}"

IMAGEDIR="${DATA_ROOT}/data"
FRAMES_CSV="${DATA_ROOT}/frames.csv"
IMU_SENSOR_YAML="${DATA_ROOT}/imu0_sensor.yaml"
ASSET_DIR="$(dirname "$IMU_PRIOR")"
CALIB="${ASSET_DIR}/calib_rectified.txt"
CAMERA_SENSOR_YAML="${ASSET_DIR}/cam0_sensor_rectified.yaml"
EXPECTED_SHA256="63bdbb5965c683962ad382a9e7477a04b130ce58ec2e0752637babf8669eacfa"

for path in "$IMAGEDIR" "$FRAMES_CSV" "$IMU_SENSOR_YAML" "$IMU_PRIOR" \
            "$CALIB" "$CAMERA_SENSOR_YAML" "$GT_CSV" \
            "$ORIGINAL_WEIGHTS" "$LEARNED_WEIGHTS"; do
  [[ -e "$path" ]] || { echo "Missing required path: $path" >&2; exit 2; }
done
actual_sha256="$(sha256sum "$LEARNED_WEIGHTS" | cut -d ' ' -f 1)"
[[ "$actual_sha256" == "$EXPECTED_SHA256" ]] || {
  echo "Selected alpha-750 checkpoint SHA-256 mismatch: $actual_sha256" >&2
  exit 2
}
[[ "$EVAL_PREFLIGHT_ONLY" == 0 || "$EVAL_PREFLIGHT_ONLY" == 1 ]] || {
  echo "EVAL_PREFLIGHT_ONLY must be 0 or 1" >&2; exit 2
}
if [[ "$EVAL_PREFLIGHT_ONLY" == 1 ]]; then
  echo "[PREFLIGHT] selected alpha-750 checkpoint and EuRoC inputs verified"
  exit 0
fi
[[ ! -e "$OUT_DIR" ]] || { echo "OUT_DIR already exists: $OUT_DIR" >&2; exit 2; }

cd "$ROOT"
export LD_LIBRARY_PATH="${CONDA_PREFIX:+${CONDA_PREFIX}/lib:}${LD_LIBRARY_PATH:-}"
mkdir -p "$OUT_DIR"
cat > "$OUT_DIR/run_manifest.txt" <<EOF
model=selected_alpha750
learned_weights=${LEARNED_WEIGHTS}
learned_sha256=${actual_sha256}
tracking_stride=2
trajectory_fill_stride=1
motion_filter_threshold=2.4
imu_full_max_dt=0.5
local_schedule=visual:3,full15dof:2
global_schedule=visual:7,full15dof:12
learned_cross_modal_calibration=1
joint_visual_imu_confidence=0
EOF

echo "[1/3] Original visual DROID"
"$PYTHON" demo.py \
  --imagedir "$IMAGEDIR" --calib "$CALIB" \
  --weights "$ORIGINAL_WEIGHTS" \
  --stride 2 --euroc_protocol_tracking_stride 2 \
  --trajectory_fill_stride 1 --euroc_original_protocol \
  --filter_thresh 2.4 --buffer 512 --disable_vis \
  --reconstruction_path "$OUT_DIR/original_droid_recon.pth"

echo "[2/3] Selected learned R/v/p confidence + alpha 15DoF model"
"$PYTHON" demo.py \
  --imagedir "$IMAGEDIR" --calib "$CALIB" \
  --weights "$LEARNED_WEIGHTS" \
  --stride 2 --euroc_protocol_tracking_stride 2 \
  --trajectory_fill_stride 1 --euroc_original_protocol \
  --filter_thresh 2.4 --buffer 512 --disable_vis \
  --reconstruction_path "$OUT_DIR/selected_alpha750_recon.pth" \
  --imu_prior "$IMU_PRIOR" --imu_frame_manifest "$FRAMES_CSV" \
  --cam_sensor_yaml "$CAMERA_SENSOR_YAML" \
  --imu_sensor_yaml "$IMU_SENSOR_YAML" \
  --use_two_stage_imu_ba \
  --imu_two_stage_visual_steps 3 --imu_two_stage_full_steps 2 \
  --imu_two_stage_global_visual_pre_steps 7 \
  --imu_two_stage_global_steps 12 \
  --no_vi_init_bootstrap \
  --use_imu_noise_model --use_imu_bias_random_walk \
  --use_imu_full_covariance_weighting \
  --use_learned_imu_precision \
  --use_learned_imu_cross_modal_calibration \
  --imu_full_max_dt 0.5 \
  --imu_ba_debug \
  --imu_ba_debug_path "$OUT_DIR/selected_alpha750_imu_debug.csv"

echo "[3/3] Dense whole-trajectory Sim(3) ATE"
common_eval_args=(
  --trajectory-source dense
  --frames-csv "$FRAMES_CSV"
  --gt-csv "$GT_CSV"
  --gt-frame body
  --camera-sensor-yaml "$CAMERA_SENSOR_YAML"
  --run "original_droid=$OUT_DIR/original_droid_recon.pth"
  --run "selected_alpha750=$OUT_DIR/selected_alpha750_recon.pth,$OUT_DIR/selected_alpha750_imu_debug.csv"
)
"$PYTHON" tools/evaluate_euroc_recon.py \
  "${common_eval_args[@]}" \
  --output-csv "$OUT_DIR/summary_full.csv" \
  --output-md "$OUT_DIR/summary_full.md"
"$PYTHON" tools/evaluate_euroc_recon.py \
  "${common_eval_args[@]}" --common-frame-ids \
  --output-csv "$OUT_DIR/summary_common.csv" \
  --output-md "$OUT_DIR/summary_common.md"
echo "[DONE] $OUT_DIR/summary_full.md"
