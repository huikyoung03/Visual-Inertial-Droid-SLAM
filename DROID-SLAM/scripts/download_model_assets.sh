#!/usr/bin/env bash
set -euo pipefail

# Downloads the two non-Git model artifacts. Override ALPHA_URL for a fork or
# a republished GitHub Release.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ALPHA_NAME="learned_cross_modal_alpha_localmatched_mh03_mh04_1k_000750.pth"
ALPHA_SHA256="63bdbb5965c683962ad382a9e7477a04b130ce58ec2e0752637babf8669eacfa"
ALPHA_URL="${ALPHA_URL:-https://github.com/huikyoung03/SLAM/releases/download/final-alpha750-2026-11-03/${ALPHA_NAME}}"

mkdir -p "${REPO_ROOT}/checkpoints"

if [[ ! -f "${REPO_ROOT}/droid.pth" ]]; then
  echo "[1/2] Downloading upstream visual DROID weights"
  (cd "${REPO_ROOT}" && bash tools/download_model.sh)
else
  echo "[1/2] droid.pth already exists; keeping it"
fi

if [[ ! -f "${REPO_ROOT}/checkpoints/${ALPHA_NAME}" ]]; then
  echo "[2/2] Downloading selected alpha-750 checkpoint"
  curl --fail --location --retry 3 --output "${REPO_ROOT}/checkpoints/${ALPHA_NAME}" "${ALPHA_URL}"
else
  echo "[2/2] selected alpha-750 checkpoint already exists; keeping it"
fi

actual="$(sha256sum "${REPO_ROOT}/checkpoints/${ALPHA_NAME}" | awk '{print $1}')"
[[ "${actual}" == "${ALPHA_SHA256}" ]] || {
  echo "Selected checkpoint checksum mismatch: ${actual}" >&2
  exit 2
}
echo "[DONE] Model assets verified"
