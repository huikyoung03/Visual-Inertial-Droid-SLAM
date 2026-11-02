# Selected model checkpoint

The selected learned checkpoint is distributed as a versioned GitHub Release
asset, not as an ordinary source file. Upload the local file below before
publishing the repository instructions. No release URL has been assigned yet.

| Field | Value |
| --- | --- |
| Filename | `learned_cross_modal_alpha_localmatched_mh03_mh04_1k_000750.pth` |
| Size | 16,717,669 bytes |
| SHA-256 | `63bdbb5965c683962ad382a9e7477a04b130ce58ec2e0752637babf8669eacfa` |
| Model | Frozen DROID visual backbone + direct R/v/p confidence + learned R/v/p alpha |
| Alpha `[R,v,p]` | `[1.2150, 1.2126, 1.9887]` |
| Release URL | [GitHub Release asset](https://github.com/huikyoung03/SLAM/releases/download/final-alpha750-2026-11-03/learned_cross_modal_alpha_localmatched_mh03_mh04_1k_000750.pth) |

The base confidence checkpoint for reproducing alpha training is
`learned_attenuation_rvp_quality_pretrain_joint2000_001000.pth` (SHA-256
`0b01773923e4585c05299a347567edf2c9e6347175360f65bf03de3947682574`).
It is available as a
[previous release asset](https://github.com/huikyoung03/SLAM/releases/download/final-direct-rvp-2026-10-27/learned_attenuation_rvp_quality_pretrain_joint2000_001000.pth).
The original visual `droid.pth` is an upstream dependency and is not bundled
here.

`tools/run_final_alpha750_evaluation.sh` checks the selected checkpoint hash
before using it. A newly retrained checkpoint will have a different hash and
must be evaluated as a distinct model, not substituted under this result.
