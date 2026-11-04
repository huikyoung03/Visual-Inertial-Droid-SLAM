# Visual–Inertial DROID-SLAM

This repository is organized as two deployable components:

```text
SLAM/
├── DROID-SLAM/   # selected learned 15DoF visual–inertial DROID-SLAM model
└── SLAM/         # browser/phone camera + IMU capture server and PLY viewer
```

## DROID-SLAM model

`DROID-SLAM/` contains the selected alpha-750 model: frozen DROID visual
backbone, learned per-factor rotation/velocity/position IMU confidence, and
learned global R/v/p cross-modal calibration. The selected EuRoC V1_03 result
is dense Sim(3) ATE **0.018312 m** versus visual DROID **0.018560 m**.

```bash
cd DROID-SLAM
bash scripts/install_droid_env.sh
conda activate droid_env
bash scripts/download_model_assets.sh
python scripts/verify_droid_env.py
```

Read [DROID-SLAM/README.md](DROID-SLAM/README.md) for the model structure,
training and evaluation protocol. See
[DROID-SLAM/docs/INSTALL_AND_DEPLOY.md](DROID-SLAM/docs/INSTALL_AND_DEPLOY.md)
for NVIDIA/CUDA, WSL2/Linux, custom-extension build, EuRoC, and troubleshooting
details.

## Phone capture server

The `SLAM/` directory is the mobile capture server. It receives native phone
camera/IMU timestamps, creates IMU priors, invokes the sibling `DROID-SLAM/`
model, exports a filtered metric PLY, and hosts a touch viewer.

```bash
cd DROID-SLAM
bash scripts/install_phone_server.sh
cd ../SLAM
bash start_alpha750_server.sh
```

Open `http://<HOST_LAN_IP>:8000/` on a phone connected to the same Wi-Fi or
hotspot. The bundled Galaxy S20 rear-wide profile is a controlled, provisional
calibration; do not reuse it for another device without recalibration.

## Model assets

Large model files are GitHub Release assets, not ordinary Git files:

- upstream `droid.pth`
- `learned_cross_modal_alpha_localmatched_mh03_mh04_1k_000750.pth`

The selected learned checkpoint SHA-256 is
`63bdbb5965c683962ad382a9e7477a04b130ce58ec2e0752637babf8669eacfa`.
