#!/usr/bin/env bash
set -euo pipefail

# Reproduce the alpha-calibration stage of the selected model.  The input is
# the released 1k direct R/v/p confidence checkpoint.  Only three alpha
# parameters are trainable; the selected candidate is the 750-step snapshot.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python}"
TRAIN_DATA_ROOT="${TRAIN_DATA_ROOT:?Set TRAIN_DATA_ROOT to raw EuRoC sequences}"
MANIFEST_ROOT="${MANIFEST_ROOT:?Set MANIFEST_ROOT to the MotionFilter manifests}"
BASE_CONFIDENCE_WEIGHTS="${BASE_CONFIDENCE_WEIGHTS:?Set BASE_CONFIDENCE_WEIGHTS to the released 1k direct R/v/p checkpoint}"
RUN_NAME="final_alpha_calibration_mh03_mh04"

[[ -f "$BASE_CONFIDENCE_WEIGHTS" ]] || {
  echo "Missing base confidence checkpoint: $BASE_CONFIDENCE_WEIGHTS" >&2; exit 2
}
for seq in MH_03_medium MH_04_difficult; do
  [[ -f "$TRAIN_DATA_ROOT/$seq/mav0/imu0/data.csv" ]] || {
    echo "Missing training sequence: $seq" >&2; exit 2
  }
  [[ -f "$MANIFEST_ROOT/$seq.csv" ]] || {
    echo "Missing MotionFilter manifest: $seq" >&2; exit 2
  }
done
cd "$ROOT"
[[ ! -e "checkpoints/${RUN_NAME}_000750.pth" ]] || {
  echo "Selected candidate already exists: checkpoints/${RUN_NAME}_000750.pth" >&2
  exit 2
}
export LD_LIBRARY_PATH="${CONDA_PREFIX:+${CONDA_PREFIX}/lib:}${LD_LIBRARY_PATH:-}"

"$PYTHON" train.py \
  --name "$RUN_NAME" --ckpt "$BASE_CONFIDENCE_WEIGHTS" \
  --datasets euroc --datapath "$TRAIN_DATA_ROOT" \
  --euroc_sequence_include MH_03_medium MH_04_difficult \
  --euroc_original_protocol --euroc_camera_frame_stride 2 \
  --euroc_keyframe_manifest_root "$MANIFEST_ROOT" \
  --imu_train_frame_strides 1 --balance_sequence_sampling \
  --gpus 1 --num_workers 0 \
  --iters 5 --imu_train_pose_only_steps 3 \
  --imu_train_stage_schedule visual:3,full:2 \
  --steps 1000 --save_freq 250 --save_final \
  --lr 0.005 --log_freq 25 \
  --freeze_non_imu --freeze_global_imu_bias \
  --freeze_existing_imu_precision \
  --use_imu_ba --use_full_imu_ba \
  --use_learned_imu_precision \
  --use_learned_imu_cross_modal_calibration \
  --use_imu_noise_model --use_imu_full_covariance_weighting \
  --use_imu_bias_random_walk \
  --imu_train_accel_bias_random_walk 0.003 \
  --imu_train_gyro_bias_random_walk 1.9393e-5 \
  --imu_require --imu_gravity 0 0 -9.81 \
  --w1 0 --w2 0 --w3 0 --w_imu 0 --w_imu_bias 0 \
  --w_imu_precision_augmentation_calibration 0 \
  --w_imu_confidence_post_ba_pose 1.0 \
  --disable_flow_loss

echo "[DONE] selected candidate: checkpoints/${RUN_NAME}_000750.pth"
