#!/usr/bin/env bash
set -euo pipefail

# Two-stage training recipe for the selected model's direct R/v/p base head.
# The published alpha-750 checkpoint was initialized from the 1k snapshot of
# phase B.  Original DROID visual weights and global IMU bias stay frozen.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python}"
TRAIN_DATA_ROOT="${TRAIN_DATA_ROOT:?Set TRAIN_DATA_ROOT to raw EuRoC sequences}"
MANIFEST_ROOT="${MANIFEST_ROOT:?Set MANIFEST_ROOT to the MotionFilter manifests}"
ORIGINAL_WEIGHTS="${ORIGINAL_WEIGHTS:?Set ORIGINAL_WEIGHTS to droid.pth}"
PRETRAIN_NAME="final_base_confidence_pretrain500"
JOINT_NAME="final_base_confidence_joint2000"

[[ -f "$ORIGINAL_WEIGHTS" ]] || { echo "Missing DROID weights: $ORIGINAL_WEIGHTS" >&2; exit 2; }
for seq in MH_03_medium MH_04_difficult; do
  [[ -f "$TRAIN_DATA_ROOT/$seq/mav0/imu0/data.csv" ]] || {
    echo "Missing training sequence: $seq" >&2; exit 2
  }
  [[ -f "$MANIFEST_ROOT/$seq.csv" ]] || {
    echo "Missing MotionFilter manifest: $seq" >&2; exit 2
  }
done
cd "$ROOT"
[[ ! -e "checkpoints/${PRETRAIN_NAME}_final_000500.pth" ]] || {
  echo "Base pretrain checkpoint already exists" >&2; exit 2
}
[[ ! -e "checkpoints/${JOINT_NAME}_001000.pth" ]] || {
  echo "Base joint checkpoint already exists" >&2; exit 2
}
export LD_LIBRARY_PATH="${CONDA_PREFIX:+${CONDA_PREFIX}/lib:}${LD_LIBRARY_PATH:-}"

common_args=(
  --datasets euroc --datapath "$TRAIN_DATA_ROOT"
  --euroc_sequence_include MH_03_medium MH_04_difficult
  --euroc_original_protocol --euroc_camera_frame_stride 2
  --euroc_keyframe_manifest_root "$MANIFEST_ROOT"
  --imu_train_frame_strides 1 --balance_sequence_sampling
  --gpus 1 --num_workers 0
  --iters 7 --imu_train_pose_only_steps 3
  --imu_train_stage_schedule visual:3,full:4
  --freeze_non_imu --freeze_global_imu_bias
  --use_imu_ba --use_full_imu_ba --use_learned_imu_precision
  --use_imu_noise_model --use_imu_full_covariance_weighting
  --use_imu_bias_random_walk
  --imu_train_accel_bias_random_walk 0.003
  --imu_train_gyro_bias_random_walk 1.9393e-5
  --imu_require --imu_gravity 0 0 -9.81
  --w1 0 --w2 0 --w3 0 --w_imu 0 --w_imu_bias 0
  --w_imu_precision_augmentation_calibration 1.0
  --use_imu_confidence_augmentation --disable_flow_loss
  --save_final
)

"$PYTHON" train.py \
  --name "$PRETRAIN_NAME" --ckpt "$ORIGINAL_WEIGHTS" \
  --steps 500 --lr 0.0001 --log_freq 50 --save_freq 500 \
  --w_imu_confidence_post_ba_pose 0.0 \
  "${common_args[@]}"

PRETRAIN_CKPT="checkpoints/${PRETRAIN_NAME}_final_000500.pth"
[[ -f "$PRETRAIN_CKPT" ]] || { echo "Missing phase-A checkpoint: $PRETRAIN_CKPT" >&2; exit 2; }
"$PYTHON" train.py \
  --name "$JOINT_NAME" --ckpt "$PRETRAIN_CKPT" \
  --steps 2000 --lr 0.0001 --log_freq 50 --save_freq 1000 \
  --w_imu_confidence_post_ba_pose 1.0 \
  --imu_confidence_post_ba_multi_horizon \
  "${common_args[@]}"

echo "[DONE] base candidate for alpha training: checkpoints/${JOINT_NAME}_001000.pth"
