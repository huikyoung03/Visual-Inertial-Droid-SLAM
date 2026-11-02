# Selected alpha-750 model: active code path

This document describes the selected checkpoint
`learned_cross_modal_alpha_localmatched_mh03_mh04_1k_000750.pth` with
`tools/run_final_alpha750_evaluation.sh`. It is the direct R/v/p confidence
model plus three learned cross-modal information coefficients. The older
confidence-only poster checkpoint is the initialization for alpha training.

## Data and frame selection

1. `demo.py:image_stream` reads rectified EuRoC cam0 images, frame IDs and
   timestamps from `frames.csv`, and the rectified camera intrinsics. The
   input is resized to 320×512; tracking takes every second 20 Hz camera
   frame. The resulting candidates are approximately 10 Hz.
2. `demo.py` loads the rectified IMU prior CSV and associates a preintegration
   row with each candidate frame. The prior was built from the raw gyro and
   accelerometer stream using calibration, synchronization, bias Jacobians,
   and sensor noise. At inference, `Droid.track` receives this prior rather
   than directly receiving individual raw IMU samples.
3. `MotionFilter.track` estimates approximate visual motion against the last
   accepted frame. The threshold is 2.4. Accepted frames enter `DepthVideo`;
   skipped-camera IMU intervals are composed to the next accepted frame.
   Frontend redundancy removal may later remove a selected keyframe.

Relevant code: `demo.py:image_stream`, `demo.py:main`,
`droid_slam/droid.py:Droid.track`, and
`droid_slam/motion_filter.py:MotionFilter.track`.

## Visual network and direct IMU confidence

`DroidNet.fnet` produces image features. `DroidNet.cnet` produces 256 context
channels split into 128 hidden and 128 input channels. The update module
encodes pairwise correlation and projected motion, then updates the 128-channel
hidden state with its ConvGRU. Separate visual heads output correspondence
correction `delta` and visual BA weight `weight`.

The selected confidence path reads the updated ConvGRU hidden state through a
1×1 convolution `128 -> 32` followed by spatial averaging. In parallel, a
7D IMU descriptor passes through a `7 -> 16 -> 16` quality MLP. The two
features are concatenated and a `48 -> 32 -> 3` fusion head predicts logits.
The inference code maps logits with `exp(-softplus(z))` to
`c_R, c_v, c_p` in `(0,1)`. Historical checkpoint loading may pad the MLP
input from 7D to 10D with zero columns; optional discrepancy branches are
disabled for this selected run, so its effective input remains the 7D quality
descriptor.

The seven IMU features are:

```text
log(1 + dt / 0.05)
log(1 + ||delta_R|| / dt)
log(1 + ||delta_v|| / dt)
log(1 + ||delta_p|| / dt²)
log(info_R / reference_info_R)
log(info_v / reference_info_v)
log(info_p / reference_info_p)
```

These are measured-interval descriptors, not ground-truth quality labels.
Direct temporal visual edges supply the ConvGRU-plus-quality prediction. For
an IMU interval without an active direct temporal visual edge, the graph uses
a quality-only fallback from the same head.

Relevant code: `droid_slam/droid_net.py:UpdateModule.forward`,
`droid_slam/droid_net.py:UpdateModule.imu_precision_weights`,
`droid_slam/geom/imu_factor.py:temporal_imu_quality_features`, and
`droid_slam/factor_graph.py:FactorGraph._edge_confidence_to_frame_confidence`.

## Factor graph objective and state

For each keyframe, the 15DoF IMU state contains pose (6), velocity (3),
accelerometer bias (3), and gyroscope bias (3). DROID inverse depth remains
an additional visual BA state. The graph has visual reprojection factors
between correlated keyframes and IMU preintegration factors between temporal
neighbors. They contribute to the same BA normal equations.

Conceptually, the active objective is:

```text
E = E_visual(T, depth; delta, weight)
    + sum_t sum_{k in {R,v,p}} alpha_k * c_{k,t} * ||r_{k,t}||²_{information_k}
    + E_bias_random_walk
```

In code the square root of each `alpha_k * c_{k,t}` multiplier scales the IMU
residual and its Jacobian. The information term comes from the calibrated,
propagated IMU covariance with project-specific normalization. Bias
random-walk rows use the noise model independently of `c_R/c_v/c_p`.

The selected checkpoint stores:

```text
alpha_R = 1.2150
alpha_v = 1.2126
alpha_p = 1.9887
```

These three parameters were learned once on training clips. They are constant
through an inference run, whereas `c_{k,t}` changes between intervals. No
manually set global scalar IMU lambda or joint visual-weight gate is active.
An interval longer than 0.5 s contributes no IMU factor; visual processing
continues.

Relevant code: `droid_slam/droid_net.py:DroidNet.get_imu_cross_modal_calibration`,
`droid_slam/factor_graph.py:FactorGraph`, and `droid_slam/geom/ba.py`.

## Runtime order

| Stage | Calls | Active objective |
| --- | ---: | --- |
| Frontend warmup | separate initialization iterations | visual |
| Each new accepted keyframe | 3 | visual local DBA |
| VI initialization | when observable | scale, gravity, velocity and bias seed |
| Local full stage after alignment | 2 | visual + IMU, 15DoF |
| Termination global prepass | 7 | visual-only BA |
| Termination global refinement | 12 | visual + IMU, 15DoF |
| PoseTrajectoryFiller | once | restore non-keyframe camera poses |

The `3+2` sequence is conditional: no local 15DoF update runs before VI
alignment. The global visual prepass does not discard visual factors from the
subsequent full 15DoF solve. Full-rate trajectory filling produces one pose
per original camera timestamp even though tracking used stride 2.

Relevant code: `droid_slam/droid_frontend.py:DroidFrontend._update`,
`droid_slam/droid.py:Droid.terminate`, and `demo.py:main`.

## Training and evaluation provenance

- The base direct confidence head was trained on MH03/MH04 MotionFilter
  manifests with seven-frame clips. Its selected snapshot is phase-B step
  1,000. The original DROID visual network and global bias network parameters
  were frozen. Local velocity and bias states still move inside each BA solve.
- Alpha calibration started from that selected base checkpoint. It froze the
  visual network, confidence head, and global bias parameters; only the three
  alpha values were trainable. The local training unroll was visual 3 + full
  15DoF 2. The selected snapshot is step 750 of a 1,000-step run.
- The selected alpha training recipe predates the explicit training-side
  0.5-second IMU interval mask. Runtime always uses its 0.5-second factor
  limit. This is a known train/runtime difference; do not claim they match
  on that condition.
- The V1_03 dense Sim(3) ATE is 0.018312 m versus 0.018560 m for the visual
  baseline. V2_03 worsens and MH05 improves. These are single-run results.

The public scripts fix the selected training/evaluation recipes and leave
experiment switches out of the release command surface. The underlying
research modules retain inactive compatibility branches for historical
checkpoints; they are not exercised by the final entry point.
