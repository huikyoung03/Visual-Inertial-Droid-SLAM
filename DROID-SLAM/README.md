# DROID-SLAM with learned IMU confidence and cross-modal calibration

This repository extends [DROID-SLAM](https://arxiv.org/abs/2108.10869) with a
15DoF visual–inertial factor graph. The selected research checkpoint is
`learned_cross_modal_alpha_localmatched_mh03_mh04_1k_000750.pth`.

The pretrained DROID visual network remains frozen. A small head reads its
ConvGRU state and seven IMU-preintegration features to predict separate
rotation, velocity, and position confidence values for each temporal factor.
Three learned global calibration values, `alpha=[alpha_R, alpha_v, alpha_p]`,
set the relative information scale of those IMU components. The visual and
IMU factors then update the same pose variables in 15DoF bundle adjustment.

## Selected result and scope

All values below are **dense whole-trajectory Sim(3) ATE RMSE** in metres,
using the original EuRoC cam0 protocol, tracking stride 2, trajectory-fill
stride 1, and the same GT timestamps. Lower is better.

| Sequence | Original visual DROID | Selected learned model | Change |
| --- | ---: | ---: | ---: |
| V1_03 | 0.018560 | **0.018312** | 1.34% lower |
| V2_03 | **0.013864** | 0.014157 | 2.11% higher |
| MH05 | 0.042880 | **0.042225** | 1.53% lower |

V1_03 was used repeatedly to choose model variants, so it is not a wholly
untouched model-selection test. These are single-run observations, not a claim
of statistically significant or universal improvement. Sim(3) alignment
removes global scale error; metric-scale accuracy must be reported separately.
See [the selection record](results/FINAL_MODEL_SELECTION_2026_11_02.md).

## Active model path

```text
rectified cam0 frames + calibration -> DROID MotionFilter -> visual factor graph
                                           |
                                           v
                                 FNet/CNet -> ConvGRU
                                           |          + IMU quality 7D
                                           |                    |
                                           +-> confidence head <-+
                                               c_R, c_v, c_p

gyro + accel + timestamps + calibration -> IMU preintegration
                                           -> R/v/p and bias residuals

visual reprojection factors + covariance information * confidence * learned alpha
                                           -> local and global 15DoF BA
                                           -> dense pose trajectory
```

The active IMU information multiplier is `alpha_k * c_k` for
`k in {R, v, p}`. The confidence head outputs values in `(0, 1)` per interval;
the learned alpha values are global checkpoint parameters. There is no
manually chosen global scalar IMU lambda. Bias random-walk rows use the sensor
noise model and are not gated by `c_R/c_v/c_p`. The selected checkpoint has
`alpha=[1.2150, 1.2126, 1.9887]`. Visual factor weights are produced by the
frozen DROID head; the optional joint visual/IMU gate is not active.

The fixed runtime schedule is:

```text
selected keyframe -> visual local DBA x3 -> VI initialization when observable
                  -> visual+IMU full 15DoF local DBA x2
end of sequence   -> visual global BA x7 -> visual+IMU full 15DoF global BA x12
                  -> fill poses on all original camera timestamps
```

Initial warmup has its own visual iterations. Full local 15DoF updates begin
only after VI initialization succeeds. An IMU interval longer than 0.5 s
does not contribute an IMU factor; the image and visual factors remain.
See [the code-level architecture](docs/FINAL_ALPHA750_ARCHITECTURE.md).

## Installation

```bash
git clone --recursive <YOUR_REPOSITORY_URL>
cd DROID-SLAM
conda env create -f environment_novis.yaml
conda activate droid_env
pip install -r requirements.txt
pip install thirdparty/lietorch
pip install thirdparty/pytorch_scatter
pip install -e .
```

Use a CUDA/PyTorch build compatible with the host GPU and driver. Keep the
upstream `droid.pth` and the selected learned checkpoint outside ordinary Git
source history. Their origin and checksum are listed in
[models/FINAL_CHECKPOINT.md](models/FINAL_CHECKPOINT.md).

## Reproduce the V1_03 evaluation

The prepared camera directory must contain `data/`, `frames.csv`, and
`imu0_sensor.yaml`. The rectified IMU prior must be generated with the same
camera–IMU calibration and placed beside `calib_rectified.txt` and
`cam0_sensor_rectified.yaml`. The preparation tools are in `tools/`.

```bash
cp configs/final_alpha750.env.example /tmp/final_alpha750.env
# Edit every absolute path in /tmp/final_alpha750.env.
source /tmp/final_alpha750.env
EVAL_PREFLIGHT_ONLY=1 bash tools/run_final_alpha750_evaluation.sh
bash tools/run_final_alpha750_evaluation.sh
```

The script checks the exact selected checkpoint SHA-256, runs original visual
DROID and the selected model on the same input, and writes dense full and
common-frame Sim(3) reports into `OUT_DIR`. It has no switches for fixed
confidence, manual lambda, joint visual gates, rotation-flow adapters, or
other experimental paths.

## Reproduce the training stages

Training uses EuRoC `MH_03_medium` and `MH_04_difficult`, their frozen
original-protocol MotionFilter manifests, and camera stride 2. `V1_03` is not
used for gradient-based training.

1. `bash tools/run_base_confidence_train.sh` trains the direct R/v/p head:
   500-step quality pretraining followed by 2,000-step post-BA fine-tuning.
   The selected base is its **1,000-step phase-B snapshot**.
2. Set `BASE_CONFIDENCE_WEIGHTS` to that base checkpoint, then run
   `bash tools/run_final_alpha750_train.sh`. It freezes the visual network,
   confidence head, and global IMU biases, trains only three alpha parameters
   for 1,000 steps, and saves the **750-step selected snapshot**.

The released checkpoints remain the reference artifacts; a fresh run may
differ because training is stochastic. The selected alpha training used a
seven-frame local `visual:3, full:2` unroll. The global backend is evaluated
on full sequences and is not unrolled during this training stage.

## Main source files

| File | Role |
| --- | --- |
| `demo.py` | Image/IMU-prior input, rectification, inference, dense trajectory output |
| `droid_slam/motion_filter.py` | Visual frame selection and composed IMU intervals |
| `droid_slam/droid_net.py` | Frozen DROID visual network, direct R/v/p head, learned alpha |
| `droid_slam/geom/imu_factor.py` | IMU quality features and preintegration helpers |
| `droid_slam/factor_graph.py` | Temporal graph edges and confidence transfer to IMU factors |
| `droid_slam/geom/ba.py` | Joint 15DoF residual/Jacobian weighting |
| `droid_slam/droid_frontend.py` | Local visual then full 15DoF updates |
| `droid_slam/droid_backend.py` | Global graph refinement |
| `train.py` | Confidence and alpha optimization |

The final scripts expose only the selected execution path. Older experiments
remain in the research workspace but are excluded from the curated GitHub
upload. See [the upload guide](GITHUB_UPLOAD_GUIDE.md) for the file list.

## Attribution

This work builds on the original DROID-SLAM implementation by Zachary Teed
and Jia Deng. Retain the upstream and third-party license notices when
redistributing the repository.
