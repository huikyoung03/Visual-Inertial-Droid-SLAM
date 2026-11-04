#!/usr/bin/env python3
"""Fail-fast verification for the selected CUDA DROID-SLAM environment."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "droid_slam"))

import torch


def main() -> None:
    if not torch.cuda.is_available():
        raise SystemExit("CUDA is unavailable to PyTorch. Check driver/PyTorch installation.")

    import droid_backends  # noqa: F401
    import lietorch  # noqa: F401
    from torch_scatter import scatter_sum

    sample = torch.tensor([1.0, 2.0], device="cuda")
    index = torch.tensor([0, 0], device="cuda")
    result = scatter_sum(sample, index)
    if float(result[0].cpu()) != 3.0:
        raise SystemExit("torch-scatter CUDA smoke test failed")

    print(f"Python: {sys.version.split()[0]}")
    print(f"PyTorch: {torch.__version__}")
    print(f"PyTorch CUDA: {torch.version.cuda}")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print("Custom CUDA extensions: OK")


if __name__ == "__main__":
    main()
