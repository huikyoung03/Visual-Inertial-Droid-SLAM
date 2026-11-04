#!/usr/bin/env bash
set -euo pipefail

# Launch the browser-based phone capture server for the selected alpha-750 model.
# Prerequisites:
#   1. conda environment droid_env built by scripts/install_droid_env.sh
#   2. phone venv built by scripts/install_phone_server.sh
#   3. selected checkpoint placed at checkpoints/<filename>
SERVER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SERVER_DIR}/.." && pwd)"
if [[ -f "${PROJECT_ROOT}/demo.py" ]]; then
  DEFAULT_DROID_SLAM_DIR="${PROJECT_ROOT}"
else
  DEFAULT_DROID_SLAM_DIR="${PROJECT_ROOT}/DROID-SLAM"
fi
DROID_SLAM_DIR="${DROID_SLAM_DIR:-${DEFAULT_DROID_SLAM_DIR}}"
if [[ -x "${PROJECT_ROOT}/.venv-phone/bin/python" ]]; then
  DEFAULT_PHONE_PYTHON="${PROJECT_ROOT}/.venv-phone/bin/python"
else
  DEFAULT_PHONE_PYTHON="${DROID_SLAM_DIR}/.venv-phone/bin/python"
fi
PHONE_PYTHON="${PHONE_PYTHON:-${DEFAULT_PHONE_PYTHON}}"
CHECKPOINT="${LEARNED_WEIGHTS:-${DROID_SLAM_DIR}/checkpoints/learned_cross_modal_alpha_localmatched_mh03_mh04_1k_000750.pth}"

if [[ -z "${DROID_PYTHON:-}" ]]; then
  command -v conda >/dev/null || {
    echo "Set DROID_PYTHON or install Conda and create droid_env first." >&2
    exit 2
  }
  DROID_PYTHON="$(conda run -n droid_env python -c 'import sys; print(sys.executable)')"
fi

[[ -x "${PHONE_PYTHON}" ]] || { echo "Phone server Python missing: ${PHONE_PYTHON}" >&2; exit 2; }
[[ -x "${DROID_PYTHON}" ]] || { echo "DROID Python missing: ${DROID_PYTHON}" >&2; exit 2; }
[[ -f "${CHECKPOINT}" ]] || { echo "Checkpoint missing: ${CHECKPOINT}" >&2; exit 2; }
[[ -f "${DROID_SLAM_DIR}/demo.py" ]] || { echo "DROID-SLAM source missing: ${DROID_SLAM_DIR}" >&2; exit 2; }
[[ -f "${DROID_SLAM_DIR}/droid.pth" ]] || { echo "Original DROID weights missing: ${DROID_SLAM_DIR}/droid.pth" >&2; exit 2; }

export DROID_SLAM_DIR
export DROID_PYTHON
export LEARNED_WEIGHTS="${CHECKPOINT}"
export FORCE_DEVICE_PROFILE_ID="galaxy_s20_rear_wide_640x480_alpha750"
export DEFAULT_CAPTURE_PURPOSE="slam"
export AUTO_SLAM_MODE="imu_15dof"
export ALLOW_PROVISIONAL_IMU="1"
export PYTHONUNBUFFERED=1
# Ensure the web-server process can launch the Conda-linked CUDA extensions.
DROID_CONDA_PREFIX="$("${DROID_PYTHON}" -c 'import sys; print(sys.prefix)')"
export LD_LIBRARY_PATH="${DROID_CONDA_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

cd "${SERVER_DIR}"
echo "[SERVER] http://0.0.0.0:8000  profile=${FORCE_DEVICE_PROFILE_ID}"
exec "${PHONE_PYTHON}" -m uvicorn main:app --host 0.0.0.0 --port 8000
