#!/usr/bin/env bash
set -euo pipefail

# Creates the CUDA environment required by the selected DROID-SLAM model.
# Usage: bash scripts/install_droid_env.sh [conda-environment-name]
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_NAME="${1:-droid_env}"
MAX_JOBS="${MAX_JOBS:-4}"

command -v conda >/dev/null || {
  echo "Conda or Miniconda is required. Install Miniconda, reopen the shell, then retry." >&2
  exit 2
}
command -v nvidia-smi >/dev/null || {
  echo "NVIDIA driver/GPU was not detected. Install a current NVIDIA driver first." >&2
  exit 2
}

echo "[1/6] Creating conda environment: ${ENV_NAME}"
if conda env list | awk '{print $1}' | grep -Fxq "${ENV_NAME}"; then
  echo "Conda environment already exists: ${ENV_NAME}" >&2
  echo "Remove it explicitly with: conda env remove -n ${ENV_NAME}" >&2
  exit 2
fi
conda env create -f "${REPO_ROOT}/environment.yml" -n "${ENV_NAME}"

echo "[2/6] Installing PyTorch 2.4.1 CUDA 11.8 wheels"
conda run -n "${ENV_NAME}" python -m pip install --upgrade pip setuptools wheel
conda run -n "${ENV_NAME}" python -m pip install \
  --index-url https://download.pytorch.org/whl/cu118 \
  torch==2.4.1+cu118 torchvision==0.19.1+cu118 torchaudio==2.4.1+cu118

echo "[3/6] Installing Python runtime packages"
conda run -n "${ENV_NAME}" python -m pip install -r "${REPO_ROOT}/requirements.txt"

echo "[4/6] Initializing CUDA extension submodules"
git -C "${REPO_ROOT}" submodule update --init --recursive

echo "[5/6] Building CUDA extensions (this can take several minutes)"
BUILD_CMD="cd '${REPO_ROOT}' && export CUDA_HOME=\"\${CONDA_PREFIX}\" && export PATH=\"\${CUDA_HOME}/bin:\${PATH}\" && export MAX_JOBS='${MAX_JOBS}' && python -m pip install -v ./thirdparty/lietorch && python -m pip install -v ./thirdparty/pytorch_scatter && python -m pip install -v -e ."
conda run -n "${ENV_NAME}" bash -lc "${BUILD_CMD}"

echo "[6/6] Verifying CUDA, PyTorch, and custom extensions"
conda run -n "${ENV_NAME}" python "${REPO_ROOT}/scripts/verify_droid_env.py"

echo "[DONE] Activate with: conda activate ${ENV_NAME}"
