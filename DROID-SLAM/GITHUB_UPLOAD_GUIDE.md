# GitHub upload guide: selected alpha-750 model

The source release represents the **selected alpha-750 15DoF model** with
V1_03 dense Sim(3) ATE `0.018312 m`. The prior `0.018467 m` confidence-only
poster model is documented as its training base, not as the final result.

## Include

Keep all already tracked upstream DROID source, CUDA code, build files,
submodules, and license/attribution files. Add the modified core and these
new source files that the selected model imports:

```text
README.md
.gitignore
demo.py
train.py
droid_slam/                          core modules already tracked upstream
droid_slam/geom/imu_factor.py
droid_slam/imu_contract.py
droid_slam/imu_prior.py
droid_slam/imu_residual.py
droid_slam/imu_runtime.py
droid_slam/vi_initialization.py
droid_slam/data_readers/euroc.py
droid_slam/data_readers/euroc_protocol.py
droid_slam/data_readers/imu_preintegration.py
droid_slam/data_readers/sampling.py
droid_slam/data_readers/tartan_v2.py
src/                                changed CUDA/extension files
tools/evaluate_euroc_recon.py
tools/prepare_euroc_original_protocol_assets.py
tools/export_euroc_original_protocol_motion_filter_manifests.sh
tools/imu_preintegrate.py
tools/run_base_confidence_train.sh
tools/run_final_alpha750_train.sh
tools/run_final_alpha750_evaluation.sh
configs/final_alpha750.env.example
models/FINAL_CHECKPOINT.md
results/FINAL_MODEL_SELECTION_2026_11_02.md
results/REFERENCE_MODELS_2026_11_02.md
docs/FINAL_ALPHA750_ARCHITECTURE.md
GITHUB_UPLOAD_GUIDE.md
```

The three public training/evaluation scripts have fixed camera stride, BA schedules, and model
components. Experimental joint gates, manual confidence settings, alternate
input rates, and audit paths are absent from the final evaluation command.
The research modules still contain inactive compatibility branches; removing
them would require a separate regression refactor of checkpoint loading and
CUDA BA, so this release restricts the documented and executable path.

## Exclude

Do not commit raw datasets, `logs/`, `outputs/`, `checkpoints/`, `.pth` or
`.ply` files, temporary environment files, unused experiment scripts,
earlier presentation diagrams, or chronological ablation reports. `.gitignore`
excludes the untracked instances. Review staged files before uploading.

## Checkpoint asset

Upload `checkpoints/learned_cross_modal_alpha_localmatched_mh03_mh04_1k_000750.pth`
as a versioned GitHub Release asset. Its SHA-256 is
`63bdbb5965c683962ad382a9e7477a04b130ce58ec2e0752637babf8669eacfa`.
After publishing the asset, replace the placeholder in
`models/FINAL_CHECKPOINT.md` with its exact download link. The upstream
`droid.pth` is obtained from DROID-SLAM; the previous direct confidence
checkpoint is linked from `models/FINAL_CHECKPOINT.md` for training
reproduction.

## Stage and review

```bash
git add README.md .gitignore GITHUB_UPLOAD_GUIDE.md demo.py train.py \
  droid_slam/depth_video.py droid_slam/droid.py droid_slam/droid_async.py \
  droid_slam/droid_backend.py droid_slam/droid_frontend.py \
  droid_slam/droid_net.py droid_slam/factor_graph.py \
  droid_slam/logger.py droid_slam/motion_filter.py \
  droid_slam/modules/extractor.py droid_slam/geom/ba.py \
  droid_slam/geom/losses.py droid_slam/geom/imu_factor.py \
  droid_slam/data_readers/base.py droid_slam/data_readers/factory.py \
  droid_slam/data_readers/euroc.py \
  droid_slam/data_readers/euroc_protocol.py \
  droid_slam/data_readers/imu_preintegration.py \
  droid_slam/data_readers/sampling.py \
  droid_slam/data_readers/tartan_v2.py \
  droid_slam/imu_contract.py droid_slam/imu_prior.py \
  droid_slam/imu_residual.py droid_slam/imu_runtime.py \
  droid_slam/vi_initialization.py \
  evaluation_scripts/test_euroc.py src/droid.cpp src/droid_kernels.cu \
  tools/evaluate_euroc_recon.py \
  tools/prepare_euroc_original_protocol_assets.py \
  tools/export_euroc_original_protocol_motion_filter_manifests.sh \
  tools/imu_preintegrate.py tools/run_base_confidence_train.sh \
  tools/run_final_alpha750_train.sh \
  tools/run_final_alpha750_evaluation.sh \
  configs/final_alpha750.env.example models/FINAL_CHECKPOINT.md \
  results/FINAL_MODEL_SELECTION_2026_11_02.md \
  results/REFERENCE_MODELS_2026_11_02.md \
  docs/FINAL_ALPHA750_ARCHITECTURE.md

git diff --cached --check
git diff --cached --stat
git status --short
```

This guide does not run `git add`, commit, push, or publish a Release. Check
the staged diff and the Release asset URL before doing those steps.
