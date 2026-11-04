from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse

from pathlib import Path
from queue import Queue, Empty

import asyncio
import csv
import json
import math
import os
import subprocess
import sys
import threading
import time
import zipfile


app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# =========================================================
# 경로 설정
# =========================================================
# Repository layout: DROID-SLAM/phone_server/main.py
BASE_DIR = Path(__file__).resolve().parent

# Repository root. The server may also be launched from any working directory.
PROJECT_ROOT = BASE_DIR.parent

# The default supports both released layouts:
#   DROID-SLAM/phone_server/             (standalone repository)
#   SLAM/{DROID-SLAM,SLAM}/              (combined GitHub repository)
# Override for a shared or external checkout with DROID_SLAM_DIR=/path/to/DROID-SLAM.
_default_droid_slam_dir = (
    PROJECT_ROOT
    if (PROJECT_ROOT / "demo.py").is_file()
    else PROJECT_ROOT / "DROID-SLAM"
)
DROID_SLAM_DIR = Path(
    os.getenv("DROID_SLAM_DIR", _default_droid_slam_dir)
).resolve()

# DROID-SLAM은 별도의 CUDA/PyTorch 환경에서 실행한다. 웹 수집 서버의 venv와
# 섞지 않기 위해 추론과 PLY export에는 이 interpreter만 사용한다.
DROID_PYTHON = Path(
    os.getenv(
        "DROID_PYTHON",
        PROJECT_ROOT / ".conda" / "envs" / "droid_env" / "bin" / "python",
    )
).resolve()
DROID_DEMO_SCRIPT = (DROID_SLAM_DIR / "demo.py").resolve()
PLY_EXPORT_SCRIPT = (DROID_SLAM_DIR / "export_reconstruction_ply.py").resolve()
DROID_RUN_TIMEOUT_SEC = int(os.getenv("DROID_RUN_TIMEOUT_SEC", "1800"))

# 실제 기기별/lens별 calibration은 이 폴더의 JSON profile로 관리한다.
# profile에 calibrated 상태가 명시되기 전에는 SLAM 실행을 거부하고 수집만 허용한다.
DEVICE_PROFILE_DIR = Path(
    os.getenv("DEVICE_PROFILE_DIR", BASE_DIR / "device_profiles")
).resolve()
DEVICE_PROFILE_DIR.mkdir(parents=True, exist_ok=True)
DEFAULT_DEVICE_PROFILE_ID = os.getenv(
    "DEFAULT_DEVICE_PROFILE_ID", "galaxy_s20_rear_wide_640x480"
)
# A native client may have an old, hard-coded profile id. This override is
# intentionally server-side and should be used only for a controlled session.
FORCE_DEVICE_PROFILE_ID = os.getenv("FORCE_DEVICE_PROFILE_ID", "").strip()
ALLOW_PROVISIONAL_IMU = os.getenv("ALLOW_PROVISIONAL_IMU", "0") == "1"

# A static-noise capture intentionally stores only native-timestamp IMU data.
# It must never enqueue preintegration or an accidental visual SLAM job on stop.
CAPTURE_PURPOSE_SLAM = "slam"
CAPTURE_PURPOSE_IMU_NOISE_STATIC = "imu_noise_static"
CAPTURE_PURPOSE_IMU_CAMERA_TARGET = "imu_camera_target"
VALID_CAPTURE_PURPOSES = {
    CAPTURE_PURPOSE_SLAM,
    CAPTURE_PURPOSE_IMU_NOISE_STATIC,
    CAPTURE_PURPOSE_IMU_CAMERA_TARGET,
}
DEFAULT_CAPTURE_PURPOSE = str(
    os.getenv("DEFAULT_CAPTURE_PURPOSE", CAPTURE_PURPOSE_SLAM)
).strip().lower()
if DEFAULT_CAPTURE_PURPOSE not in VALID_CAPTURE_PURPOSES:
    raise RuntimeError(
        "DEFAULT_CAPTURE_PURPOSE must be one of: "
        + ", ".join(sorted(VALID_CAPTURE_PURPOSES))
    )

# 상위폴더/DROID-SLAM/tools/imu_preintegrate.py
IMU_PREINTEGRATE_SCRIPT = Path(
    os.getenv(
        "IMU_PREINTEGRATE_SCRIPT",
        DROID_SLAM_DIR / "tools" / "imu_preintegrate.py"
    )
).resolve()

BASE_UPLOAD_DIR = Path(
    os.getenv("UPLOAD_DIR", BASE_DIR / "uploads")
).resolve()
BASE_UPLOAD_DIR.mkdir(parents=True, exist_ok=True)

IMU_PREINTEGRATE_TIMEOUT_SEC = int(
    os.getenv("IMU_PREINTEGRATE_TIMEOUT_SEC", "120")
)

# 1이면 이미지가 들어올 때마다 imu_prior.csv 갱신 요청
# 0이면 stop 때만 최종 imu_prior.csv 생성
ENABLE_IMU_PREINTEGRATION_LIVE = (
    os.getenv("ENABLE_IMU_PREINTEGRATION_LIVE", "1") == "1"
)

# 사전적분 요청 큐
# maxsize=1: 너무 많이 밀리면 오래된 요청 버리고 최신 요청만 유지
preintegration_queue = Queue(maxsize=1)

# imu_prior.csv를 동시에 여러 번 쓰지 않도록 잠금
preintegration_lock = threading.Lock()
slam_job_lock = threading.Lock()


# =========================================================
# 기본 페이지
# =========================================================
@app.get("/")
def root():
    index_path = BASE_DIR / "static" / "index.html"

    if index_path.exists():
        return FileResponse(index_path)

    return {
        "ok": True,
        "message": "server working, but static/index.html not found",
        "base_dir": str(BASE_DIR),
        "droid_slam_dir": str(DROID_SLAM_DIR),
        "imu_preintegrate_script": str(IMU_PREINTEGRATE_SCRIPT),
        "imu_script_exists": IMU_PREINTEGRATE_SCRIPT.exists(),
        "websocket": "/ws/stream",
        "sessions": "/sessions",
    }


@app.get("/routes")
def routes():
    return [route.path for route in app.routes]


@app.get("/device-profiles")
def device_profiles():
    return {
        "ok": True,
        "default_profile_id": DEFAULT_DEVICE_PROFILE_ID,
        "profiles": list_device_profiles(),
    }


@app.get("/device-profiles/{profile_id}")
def device_profile(profile_id: str):
    try:
        profile = load_device_profile(profile_id)
    except ValueError as exc:
        return JSONResponse(status_code=404, content={"ok": False, "message": str(exc)})
    return {"ok": True, "profile": profile_public_view(profile)}


# =========================================================
# 공통 유틸
# =========================================================
def sec_to_ns(timestamp_sec: float) -> int:
    return int(timestamp_sec * 1_000_000_000)


def normalize_timestamp_sec(value) -> float:
    """
    초 / 밀리초 / 나노초 timestamp를 모두 초 단위로 정규화.
    웹 Date.now()/1000, Android System.currentTimeMillis()/1000 둘 다 초 단위.
    """
    try:
        ts = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("timestamp must be numeric") from exc

    if not math.isfinite(ts) or ts <= 0:
        raise ValueError("timestamp must be positive and finite")

    # 나노초 수준
    if ts > 1_000_000_000_000_000:
        return ts / 1_000_000_000.0

    # 밀리초 수준
    if ts > 10_000_000_000:
        return ts / 1000.0

    # 초 수준
    return ts


def safe_session_id(session_id: str) -> str:
    allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"
    cleaned = "".join(ch for ch in str(session_id) if ch in allowed)

    if cleaned == "":
        cleaned = "session_default"

    return cleaned


def safe_profile_id(profile_id: str) -> str:
    """Return an identifier that is safe to use as a profile filename."""
    return safe_session_id(profile_id)


def read_json(path: Path, default=None):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return default


def write_json(path: Path, data: dict):
    """Atomically persist small session/profile status documents."""
    temp_path = path.with_suffix(path.suffix + ".tmp")
    temp_path.write_text(
        json.dumps(data, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    os.replace(temp_path, path)


def finite_vector(value, size: int):
    """Return a finite vector, accepting a row-major 3x3 rotation matrix.

    Device-profile JSON may represent ``R_imu_to_camera`` either as a flat
    nine-value row-major vector or as three rows.  The DROID CLI always takes
    the flat form, so normalising here prevents a calibrated 3x3 estimate from
    being silently discarded before it reaches preintegration/runtime.
    """
    if not isinstance(value, (list, tuple)):
        return None

    if size == 9 and len(value) == 3 and all(
        isinstance(row, (list, tuple)) and len(row) == 3 for row in value
    ):
        value = [component for row in value for component in row]

    if len(value) != size:
        return None
    try:
        vector = [float(item) for item in value]
    except (TypeError, ValueError):
        return None
    return vector if all(math.isfinite(item) for item in vector) else None


def rotation_matrix_error(matrix: list) -> str | None:
    """Validate an IMU-to-camera rotation before it can enter 15DoF BA."""
    if len(matrix) != 9:
        return "must contain nine row-major values"

    rows = [matrix[0:3], matrix[3:6], matrix[6:9]]
    for row_index, row in enumerate(rows):
        norm = math.sqrt(sum(value * value for value in row))
        if abs(norm - 1.0) > 2e-2:
            return f"row {row_index} is not unit length (norm={norm:.6f})"

    for first in range(3):
        for second in range(first + 1, 3):
            dot = sum(rows[first][k] * rows[second][k] for k in range(3))
            if abs(dot) > 2e-2:
                return f"rows {first}/{second} are not orthogonal (dot={dot:.6f})"

    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if abs(determinant - 1.0) > 3e-2:
        return f"determinant must be +1 (det={determinant:.6f})"
    return None


def parse_primary_timestamp(payload: dict, message_kind: str):
    """Return the common camera/IMU time base used for frame matching.

    ``timestamp_ns`` is the production path: Camera2 and SensorEvent must
    provide values on the same Android monotonic clock. When a native client
    explicitly declares that source, its existing ``sensor_timestamp_ns`` is
    accepted as the same primary field. The browser collector keeps its
    explicit epoch-second ``timestamp`` fallback for smoke tests.
    Server arrival time is intentionally never used as a substitute.
    """
    declared_source = str(payload.get("timestamp_source") or "").strip()
    timestamp_ns = payload.get("timestamp_ns")
    if timestamp_ns in (None, "") and message_kind == "frame_meta":
        timestamp_ns = payload.get("capture_timestamp_ns")
    if timestamp_ns in (None, "") and declared_source in {
        "device_monotonic_ns",
        "android_sensor_nanoseconds",
    }:
        timestamp_ns = payload.get("sensor_timestamp_ns")
    if timestamp_ns not in (None, ""):
        try:
            timestamp_ns = int(float(timestamp_ns))
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"{message_kind}.timestamp_ns must be an integer") from exc
        if timestamp_ns <= 0:
            raise ValueError(f"{message_kind}.timestamp_ns must be positive")
        source = declared_source or "device_monotonic_ns"
        return timestamp_ns / 1_000_000_000.0, timestamp_ns, source

    raw_timestamp = payload.get("timestamp")
    if raw_timestamp in (None, ""):
        raise ValueError(
            f"{message_kind} requires a capture timestamp; server arrival time is not valid"
        )
    try:
        timestamp_sec = normalize_timestamp_sec(raw_timestamp)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{message_kind}.timestamp must be numeric") from exc
    if not math.isfinite(timestamp_sec) or timestamp_sec <= 0.0:
        raise ValueError(f"{message_kind}.timestamp must be positive and finite")
    source = declared_source or "client_epoch_seconds"
    return timestamp_sec, sec_to_ns(timestamp_sec), source


def set_session_timestamp_source(session_dir: Path, source: str):
    """Persist the first-order time-base contract once per capture session."""
    source = str(source).strip() or "unknown"
    metadata = load_session_metadata(session_dir)
    alignment = metadata.get("timestamp_alignment") or {}
    existing = alignment.get("primary_source")
    if existing and existing != source:
        raise ValueError(
            "camera and IMU must use one timestamp source; "
            f"session has {existing!r}, received {source!r}"
        )
    if existing:
        return
    update_session_metadata(session_dir, {
        "timestamp_alignment": {
            "primary_source": source,
            "primary_contract": (
                "frames.csv.timestamp_ns and imu.csv.timestamp_ns share this "
                "capture-time base; IMU calibration offset is applied later "
                "inside imu_preintegrate.py"
            ),
        },
    })


def get_profile_path(profile_id: str) -> Path:
    return DEVICE_PROFILE_DIR / f"{safe_profile_id(profile_id)}.json"


def load_device_profile(profile_id: str) -> dict:
    profile_id = safe_profile_id(profile_id)
    profile_path = get_profile_path(profile_id)
    profile = read_json(profile_path)
    if not isinstance(profile, dict):
        raise ValueError(f"device profile not found or invalid: {profile_id}")
    if safe_profile_id(profile.get("id", profile_id)) != profile_id:
        raise ValueError(f"device profile id does not match filename: {profile_id}")
    profile["id"] = profile_id
    profile["_path"] = str(profile_path)
    return profile


def profile_validation_errors(profile: dict, require_imu: bool = False):
    """Validate values that DROID needs; never substitute fake calibration."""
    errors = []
    capture = profile.get("capture") or {}
    camera = profile.get("camera") or {}
    imu = profile.get("imu") or {}

    if require_imu and profile.get("status") != "calibrated":
        if profile.get("status") == "provisional" and ALLOW_PROVISIONAL_IMU:
            # The separate profile itself records the provenance. The explicit
            # environment opt-in prevents a provisional estimate from silently
            # becoming the normal runtime path.
            pass
        elif profile.get("status") == "provisional":
            errors.append(
                "provisional IMU profile requires ALLOW_PROVISIONAL_IMU=1"
            )
        else:
            errors.append("IMU mode requires a fully calibrated profile")

    width = capture.get("width")
    height = capture.get("height")
    if not isinstance(width, int) or width <= 0 or not isinstance(height, int) or height <= 0:
        errors.append("capture.width and capture.height must be positive integers")

    intrinsics = finite_vector(camera.get("intrinsics"), 4)
    if intrinsics is None or intrinsics[0] <= 0 or intrinsics[1] <= 0:
        errors.append("camera.intrinsics must contain calibrated fx, fy, cx, cy")

    distortion = camera.get("distortion", [])
    if distortion is not None and not isinstance(distortion, list):
        errors.append("camera.distortion must be a numeric list")
    elif distortion is not None:
        try:
            distortion_is_valid = all(math.isfinite(float(value)) for value in distortion)
        except (TypeError, ValueError):
            distortion_is_valid = False
        if not distortion_is_valid:
            errors.append("camera.distortion contains a non-finite value")

    calibration_size = camera.get("calibration_resolution")
    if calibration_size != [width, height]:
        errors.append("camera.calibration_resolution must match capture width and height")

    if require_imu:
        if finite_vector(imu.get("R_imu_to_camera"), 9) is None:
            errors.append("imu.R_imu_to_camera must contain a calibrated 3x3 rotation")
        else:
            rotation_error = rotation_matrix_error(finite_vector(imu.get("R_imu_to_camera"), 9))
            if rotation_error is not None:
                errors.append(f"imu.R_imu_to_camera is invalid: {rotation_error}")
        if finite_vector(imu.get("t_imu_to_camera_m"), 3) is None:
            errors.append("imu.t_imu_to_camera_m must contain a calibrated translation in metres")
        for field_name in ("gyro_bias_rad_s", "accel_bias_m_s2"):
            if finite_vector(imu.get(field_name), 3) is None:
                errors.append(f"imu.{field_name} must contain three calibrated values")
        for field_name in (
            "gyro_noise_density",
            "gyro_random_walk",
            "accel_noise_density",
            "accel_random_walk",
        ):
            try:
                value = float(imu.get(field_name))
            except (TypeError, ValueError):
                value = float("nan")
            if not math.isfinite(value) or value <= 0:
                errors.append(f"imu.{field_name} must be a positive calibrated value")
        try:
            time_offset = float(imu.get("camera_imu_time_offset_sec"))
        except (TypeError, ValueError):
            time_offset = float("nan")
        if not math.isfinite(time_offset):
            errors.append("imu.camera_imu_time_offset_sec must be calibrated")
        expected_offset_convention = "imu_aligned_time = imu_raw_time + offset"
        if imu.get("camera_imu_time_offset_convention") != expected_offset_convention:
            errors.append(
                "imu.camera_imu_time_offset_convention must be "
                f"{expected_offset_convention!r}"
            )
        accel_scale = imu.get("accel_scale")
        if accel_scale is not None:
            scale = finite_vector(accel_scale, 3)
            if scale is None or any(value <= 0.0 for value in scale):
                errors.append("imu.accel_scale must contain three positive calibrated values")

    return errors


def profile_public_view(profile: dict) -> dict:
    """Expose only configuration and readiness information to the phone UI."""
    return {
        "id": profile["id"],
        "name": profile.get("name", profile["id"]),
        "status": profile.get("status", "unknown"),
        "capture": profile.get("capture", {}),
        "description": profile.get("description", ""),
        "visual_ready": not profile_validation_errors(profile, require_imu=False),
        "imu_15dof_ready": not profile_validation_errors(profile, require_imu=True),
        "visual_errors": profile_validation_errors(profile, require_imu=False),
        "imu_15dof_errors": profile_validation_errors(profile, require_imu=True),
    }


def list_device_profiles():
    profiles = []
    for profile_path in sorted(DEVICE_PROFILE_DIR.glob("*.json")):
        profile = read_json(profile_path)
        if not isinstance(profile, dict):
            continue
        try:
            profile["id"] = safe_profile_id(profile.get("id", profile_path.stem))
            profiles.append(profile_public_view(profile))
        except (TypeError, ValueError):
            continue
    return profiles


def load_session_metadata(session_dir: Path) -> dict:
    data = read_json(session_dir / "meta.json", default={})
    return data if isinstance(data, dict) else {}


def update_session_metadata(session_dir: Path, updates: dict):
    metadata = load_session_metadata(session_dir)
    metadata.update(updates)
    write_json(session_dir / "meta.json", metadata)
    return metadata


def parse_capture_purpose(value) -> str:
    """Return a capture purpose that controls post-capture work only."""
    purpose = str(value or DEFAULT_CAPTURE_PURPOSE).strip().lower()
    if purpose not in VALID_CAPTURE_PURPOSES:
        allowed = ", ".join(sorted(VALID_CAPTURE_PURPOSES))
        raise ValueError(f"unsupported capture_purpose={purpose!r}; use one of: {allowed}")
    return purpose


def session_capture_purpose(session_dir: Path) -> str:
    """Read a session's immutable capture intent, defaulting old sessions to SLAM."""
    metadata = load_session_metadata(session_dir)
    return parse_capture_purpose(metadata.get("capture_purpose"))


def write_droid_calibration(calib_path: Path, profile: dict):
    camera = profile.get("camera") or {}
    intrinsics = finite_vector(camera.get("intrinsics"), 4)
    if intrinsics is None:
        raise ValueError("cannot write DROID calibration without valid intrinsics")

    distortion = camera.get("distortion") or []
    values = intrinsics + [float(value) for value in distortion]
    calib_path.write_text(
        " ".join(f"{value:.12g}" for value in values) + "\n",
        encoding="utf-8",
    )


def configure_session_profile(session_dir: Path, profile_id: str) -> dict:
    """Attach a selected device profile and materialize its camera calibration."""
    profile = load_device_profile(profile_id)
    visual_errors = profile_validation_errors(profile, require_imu=False)
    calib_path = session_dir / "calib.txt"

    if not visual_errors:
        write_droid_calibration(calib_path, profile)
    elif calib_path.exists():
        # A stale placeholder must never be mistaken for a real phone calibration.
        calib_path.unlink()

    snapshot = dict(profile)
    snapshot.pop("_path", None)
    write_json(session_dir / "device_profile.json", snapshot)
    update_session_metadata(session_dir, {
        "device_profile_id": profile["id"],
        "device_profile_path": str(get_profile_path(profile["id"])),
        "calibration_file": "calib.txt" if not visual_errors else None,
        "calibration_ready": not visual_errors,
        "imu_15dof_ready": not profile_validation_errors(profile, require_imu=True),
    })
    return profile_public_view(profile)


def get_session_profile(session_dir: Path) -> dict:
    metadata = load_session_metadata(session_dir)
    profile_id = metadata.get("device_profile_id", DEFAULT_DEVICE_PROFILE_ID)
    return load_device_profile(profile_id)


def get_session_dir(session_id: str) -> Path:
    session_id = safe_session_id(session_id)

    session_dir = BASE_UPLOAD_DIR / session_id
    images_dir = session_dir / "images"

    session_dir.mkdir(parents=True, exist_ok=True)
    images_dir.mkdir(parents=True, exist_ok=True)

    init_session_files(session_dir)

    return session_dir


def init_session_files(session_dir: Path):
    frames_csv = session_dir / "frames.csv"
    imu_csv = session_dir / "imu.csv"
    times_txt = session_dir / "times.txt"
    calib_txt = session_dir / "calib.txt"
    meta_json = session_dir / "meta.json"

    if not frames_csv.exists():
        with open(frames_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "frame_id",
                "timestamp_sec",
                "timestamp_ns",
                "filename",
                "width",
                "height",
                "format",
                "sensor_timestamp_ns",
            ])

    if not imu_csv.exists():
        with open(imu_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp_sec",
                "timestamp_ns",
                "gx",
                "gy",
                "gz",
                "ax",
                "ay",
                "az",
                "sensor_timestamp_ns",
                "gyro_timestamp_ns",
            ])

    if not times_txt.exists():
        times_txt.write_text("", encoding="utf-8")

    if not meta_json.exists():
        meta = {
            "target_slam": "DROID-SLAM",
            "communication": "single websocket",
            "websocket_endpoint": "/ws/stream",
            "image_format": "jpg_or_webp",
            "image_dir": "images",
            "frame_file": "frames.csv",
            "imu_file": "imu.csv",
            "sync_file": "synced.json",
            "imu_prior_file": "imu_prior.csv",
            "imu_preintegrate_script": str(IMU_PREINTEGRATE_SCRIPT),
            "droid_slam_dir": str(DROID_SLAM_DIR),
            "calibration_file": None,
            "calibration_ready": False,
            "device_profile_id": None,
            "timestamp_alignment": {
                "primary_source": None,
                "primary_contract": (
                    "Camera and IMU must use one capture-time clock. "
                    "IMU-to-camera offset is applied only during preintegration."
                ),
            },
            "live_preintegration": ENABLE_IMU_PREINTEGRATION_LIVE,
            "protocol": {
                "start": "세션 시작",
                "frame_meta": "이미지 메타데이터 전송",
                "binary": "이미지 바이너리 전송",
                "imu": "IMU 데이터 전송",
                "stop": "마지막 데이터까지 synced.json 생성 후 imu_prior.csv 최종 생성",
            },
            "note": (
                "SLAM/main.py에서 ../DROID-SLAM/tools/imu_preintegrate.py를 실행해서 "
                "세션 폴더 안에 imu_prior.csv를 생성한다. 실제 DROID 실행 전에는 "
                "device profile의 보정값으로 calib.txt가 생성되어야 한다."
            ),
        }

        meta_json.write_text(
            json.dumps(meta, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )


def get_next_frame_id(session_dir: Path) -> int:
    frames_csv = session_dir / "frames.csv"

    if not frames_csv.exists():
        return 0

    with open(frames_csv, "r", newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))

    return max(0, len(rows) - 1)


def extract_imu_values(payload: dict):
    """
    웹/앱에서 들어오는 IMU 형식이 조금 달라도 안전하게 읽기.
    """
    accel = payload.get("accel") or payload.get("linear_accel") or {}
    accel_g = (
        payload.get("accel_g")
        or payload.get("accelerationIncludingGravity")
        or {}
    )
    gyro = payload.get("gyro") or {}

    ax = float(payload.get("acc_x", accel.get("x", accel_g.get("x", 0.0))))
    ay = float(payload.get("acc_y", accel.get("y", accel_g.get("y", 0.0))))
    az = float(payload.get("acc_z", accel.get("z", accel_g.get("z", 0.0))))

    gx = float(payload.get("gyro_x", gyro.get("alpha", gyro.get("x", 0.0))))
    gy = float(payload.get("gyro_y", gyro.get("beta", gyro.get("y", 0.0))))
    gz = float(payload.get("gyro_z", gyro.get("gamma", gyro.get("z", 0.0))))

    return gx, gy, gz, ax, ay, az


# =========================================================
# 프레임-IMU 동기화 json 생성
# =========================================================
def build_synced_json(session_dir: Path):
    frames_path = session_dir / "frames.csv"
    imu_path = session_dir / "imu.csv"
    synced_path = session_dir / "synced.json"

    if not frames_path.exists():
        raise FileNotFoundError("frames.csv not found")

    if not imu_path.exists():
        raise FileNotFoundError("imu.csv not found")

    frames = []

    with open(frames_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for row in reader:
            frames.append({
                "frame_id": int(row["frame_id"]),
                "timestamp_sec": float(row["timestamp_sec"]),
                "timestamp_ns": int(float(row["timestamp_ns"])),
                "filename": row["filename"],
                "width": int(row["width"]),
                "height": int(row["height"]),
                "format": row.get("format", "jpg"),
            })

    imu_samples = []

    with open(imu_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for row in reader:
            imu_samples.append({
                "timestamp_sec": float(row["timestamp_sec"]),
                "timestamp_ns": int(float(row["timestamp_ns"])),
                "gx": float(row["gx"]),
                "gy": float(row["gy"]),
                "gz": float(row["gz"]),
                "ax": float(row["ax"]),
                "ay": float(row["ay"]),
                "az": float(row["az"]),
            })

    frames.sort(key=lambda x: x["timestamp_ns"])
    imu_samples.sort(key=lambda x: x["timestamp_ns"])

    synced = []

    for i, frame in enumerate(frames):
        curr_ts = frame["timestamp_ns"]

        if i == 0:
            prev_ts = None
            imu_window = []
        else:
            prev_ts = frames[i - 1]["timestamp_ns"]
            imu_window = [
                imu
                for imu in imu_samples
                if prev_ts < imu["timestamp_ns"] <= curr_ts
            ]

        synced.append({
            "frame_id": frame["frame_id"],
            "timestamp_sec": frame["timestamp_sec"],
            "timestamp_ns": frame["timestamp_ns"],
            "image": f"images/{frame['filename']}",
            "width": frame["width"],
            "height": frame["height"],
            "format": frame["format"],
            "imu_range": {
                "start_timestamp_ns": prev_ts,
                "end_timestamp_ns": curr_ts,
            },
            "imu_count": len(imu_window),
            "imu": imu_window,
        })

    synced_path.write_text(
        json.dumps(synced, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    avg_imu_per_frame = 0.0

    if len(synced) > 0:
        avg_imu_per_frame = (
            sum(item["imu_count"] for item in synced)
            / len(synced)
        )

    return {
        "synced_path": str(synced_path),
        "frame_count": len(frames),
        "imu_count": len(imu_samples),
        "avg_imu_per_frame": avg_imu_per_frame,
    }


# =========================================================
# DROID-SLAM/tools/imu_preintegrate.py 실행
# =========================================================
def validate_generated_imu_prior(frames_csv: Path, imu_prior_csv: Path) -> dict:
    """Verify a generated prior uses the exact camera-frame manifest contract."""

    droid_module_dir = str(DROID_SLAM_DIR / "droid_slam")
    if droid_module_dir not in sys.path:
        sys.path.insert(0, droid_module_dir)

    from imu_contract import load_frame_manifest, validate_frame_prior_mapping

    frames = load_frame_manifest(frames_csv)
    priors = {}
    with open(imu_prior_csv, "r", newline="", encoding="utf-8") as stream:
        for row_number, row in enumerate(csv.DictReader(stream), start=2):
            try:
                frame_index = int(float(row["frame_index"]))
                frame_id = int(float(row["frame_id"]))
                timestamp_ns = int(float(row["timestamp_ns"]))
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                raise ValueError(
                    f"imu_prior.csv row {row_number} has invalid frame identifiers"
                ) from exc

            if not math.isfinite(float(timestamp_ns)):
                raise ValueError(
                    f"imu_prior.csv row {row_number} has non-finite timestamp_ns"
                )
            if frame_index in priors:
                raise ValueError(
                    f"imu_prior.csv has duplicate frame_index={frame_index}"
                )

            priors[frame_index] = {
                "frame_id": frame_id,
                "timestamp_ns": timestamp_ns,
                "filename": str(row.get("filename", "")).strip().replace("\\", "/"),
            }

    validate_frame_prior_mapping(frames, priors)
    return {
        "frames": len(frames),
        "priors": len(priors),
    }


def profile_preintegration_args(session_dir: Path):
    """Build only explicit, calibrated IMU arguments for imu_preintegrate.py."""
    try:
        profile = get_session_profile(session_dir)
    except ValueError:
        return [], None

    imu = profile.get("imu") or {}
    # Bias/scale corrections are useful for visual-session diagnostics as well.
    # The geometry/time transform is withheld until the profile is fully
    # calibrated so a provisional R/time pair can never enter a 15DoF prior.
    full_imu_calibration = not profile_validation_errors(profile, require_imu=True)
    args = []
    always_safe_vectors = (
        ("gyro_bias_rad_s", 3, "--gyro_bias"),
        ("accel_bias_m_s2", 3, "--acc_bias"),
        # Six-position calibration estimates a multiplicative correction after
        # accelerometer bias removal. It belongs in preintegration, not in the
        # DROID command, because runtime BA consumes the corrected prior.
        ("accel_scale", 3, "--accel_scale"),
    )
    for field_name, size, option in always_safe_vectors:
        vector = finite_vector(imu.get(field_name), size)
        if vector is not None:
            args.extend([option, *[f"{value:.12g}" for value in vector]])

    if full_imu_calibration:
        for field_name, size, option in (
            ("R_imu_to_camera", 9, "--imu_to_cam_rotation"),
            ("t_imu_to_camera_m", 3, "--imu_to_cam_translation"),
        ):
            vector = finite_vector(imu.get(field_name), size)
            if vector is not None:
                args.extend([option, *[f"{value:.12g}" for value in vector]])

    option_scalars = (
        ("gyro_noise_density", "--gyro_noise_density"),
        ("gyro_random_walk", "--gyro_random_walk"),
        ("accel_noise_density", "--accel_noise_density"),
        ("accel_random_walk", "--accel_random_walk"),
    )
    if full_imu_calibration:
        for field_name, option in option_scalars:
            try:
                value = float(imu.get(field_name))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value) and value > 0:
                args.extend([option, f"{value:.12g}"])

        try:
            time_offset = float(imu.get("camera_imu_time_offset_sec"))
        except (TypeError, ValueError):
            time_offset = float("nan")
        if math.isfinite(time_offset):
            args.extend(["--imu_time_offset_sec", f"{time_offset:.12g}"])

    return args, profile["id"]


def run_imu_preintegration(session_dir: Path, reason: str):
    """
    실제 실행 명령:

    python ../DROID-SLAM/tools/imu_preintegrate.py
        --session_dir uploads/세션명
        --frames uploads/세션명/frames.csv
        --imu uploads/세션명/imu.csv
        --output uploads/세션명/imu_prior.csv

    결과:
    uploads/세션명/imu_prior.csv
    """
    frames_csv = session_dir / "frames.csv"
    imu_csv = session_dir / "imu.csv"
    output_csv = session_dir / "imu_prior.csv"
    pending_csv = session_dir / "imu_prior.pending.csv"
    status_json = session_dir / "imu_preintegrate_status.json"
    log_txt = session_dir / "imu_preintegrate.log"

    started_at = time.time()

    def write_status(data: dict):
        status_json.write_text(
            json.dumps(data, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        return data

    if not IMU_PREINTEGRATE_SCRIPT.exists():
        return write_status({
            "ok": False,
            "status": "script_not_found",
            "reason": reason,
            "message": f"imu_preintegrate.py not found: {IMU_PREINTEGRATE_SCRIPT}",
            "script": str(IMU_PREINTEGRATE_SCRIPT),
            "droid_slam_dir": str(DROID_SLAM_DIR),
            "output": str(output_csv),
        })

    if not frames_csv.exists():
        return write_status({
            "ok": False,
            "status": "frames_not_found",
            "reason": reason,
            "message": "frames.csv not found",
            "output": str(output_csv),
        })

    if not imu_csv.exists():
        return write_status({
            "ok": False,
            "status": "imu_not_found",
            "reason": reason,
            "message": "imu.csv not found",
            "output": str(output_csv),
        })

    profile_args, profile_id = profile_preintegration_args(session_dir)
    command = [
        sys.executable,
        str(IMU_PREINTEGRATE_SCRIPT),
        "--session_dir", str(session_dir),
        "--frames", str(frames_csv),
        "--imu", str(imu_csv),
        "--output", str(pending_csv),
    ] + profile_args

    try:
        contract = None
        contract_error = None
        with preintegration_lock:
            with open(log_txt, "a", encoding="utf-8") as log_file:
                log_file.write("\n" + "=" * 80 + "\n")
                log_file.write("[IMU PREINTEGRATION START]\n")
                log_file.write(f"reason={reason}\n")
                log_file.write(f"started_at={started_at}\n")
                log_file.write(f"cwd={DROID_SLAM_DIR}\n")
                log_file.write("command=" + " ".join(command) + "\n")
                log_file.write("=" * 80 + "\n")
                log_file.flush()

                result = subprocess.run(
                    command,
                    cwd=str(DROID_SLAM_DIR),
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=IMU_PREINTEGRATE_TIMEOUT_SEC,
                )
            if result.returncode == 0 and pending_csv.exists():
                try:
                    contract = validate_generated_imu_prior(frames_csv, pending_csv)
                    os.replace(pending_csv, output_csv)
                except Exception as exc:
                    contract_error = str(exc)

        finished_at = time.time()

        if result.returncode == 0 and contract is not None and output_csv.exists():
            return write_status({
                "ok": True,
                "status": "done",
                "reason": reason,
                "message": "imu_preintegrate.py completed",
                "script": str(IMU_PREINTEGRATE_SCRIPT),
                "droid_slam_dir": str(DROID_SLAM_DIR),
                "output": str(output_csv),
                "output_exists": output_csv.exists(),
                "prior_contract": contract,
                "device_profile_id": profile_id,
                "log": str(log_txt),
                "returncode": result.returncode,
                "started_at": started_at,
                "finished_at": finished_at,
                "elapsed_sec": round(finished_at - started_at, 3),
                "command": command,
            })

        if result.returncode == 0 and contract_error is not None:
            return write_status({
                "ok": False,
                "status": "contract_invalid",
                "reason": reason,
                "message": (
                    "imu_prior.csv was generated but does not match frames.csv: "
                    f"{contract_error}"
                ),
                "script": str(IMU_PREINTEGRATE_SCRIPT),
                "droid_slam_dir": str(DROID_SLAM_DIR),
                "output": str(output_csv),
                "output_exists": output_csv.exists(),
                "pending_output": str(pending_csv),
                "log": str(log_txt),
                "returncode": result.returncode,
                "started_at": started_at,
                "finished_at": finished_at,
                "elapsed_sec": round(finished_at - started_at, 3),
                "command": command,
            })

        if result.returncode == 0 and not pending_csv.exists():
            return write_status({
                "ok": False,
                "status": "output_missing",
                "reason": reason,
                "message": "imu_preintegrate.py returned 0 but the pending imu prior was not created",
                "script": str(IMU_PREINTEGRATE_SCRIPT),
                "droid_slam_dir": str(DROID_SLAM_DIR),
                "output": str(output_csv),
                "output_exists": False,
                "log": str(log_txt),
                "returncode": result.returncode,
                "started_at": started_at,
                "finished_at": finished_at,
                "elapsed_sec": round(finished_at - started_at, 3),
                "command": command,
            })

        return write_status({
            "ok": False,
            "status": "failed",
            "reason": reason,
            "message": f"imu_preintegrate.py failed with returncode={result.returncode}",
            "script": str(IMU_PREINTEGRATE_SCRIPT),
            "droid_slam_dir": str(DROID_SLAM_DIR),
            "output": str(output_csv),
            "output_exists": output_csv.exists(),
            "log": str(log_txt),
            "returncode": result.returncode,
            "started_at": started_at,
            "finished_at": finished_at,
            "elapsed_sec": round(finished_at - started_at, 3),
            "command": command,
        })

    except subprocess.TimeoutExpired:
        finished_at = time.time()

        return write_status({
            "ok": False,
            "status": "timeout",
            "reason": reason,
            "message": f"imu_preintegrate.py timeout after {IMU_PREINTEGRATE_TIMEOUT_SEC}s",
            "script": str(IMU_PREINTEGRATE_SCRIPT),
            "output": str(output_csv),
            "output_exists": output_csv.exists(),
            "log": str(log_txt),
            "started_at": started_at,
            "finished_at": finished_at,
            "elapsed_sec": round(finished_at - started_at, 3),
            "command": command,
        })

    except Exception as e:
        finished_at = time.time()

        return write_status({
            "ok": False,
            "status": "exception",
            "reason": reason,
            "message": str(e),
            "script": str(IMU_PREINTEGRATE_SCRIPT),
            "output": str(output_csv),
            "output_exists": output_csv.exists(),
            "log": str(log_txt),
            "started_at": started_at,
            "finished_at": finished_at,
            "elapsed_sec": round(finished_at - started_at, 3),
            "command": command,
        })


def clear_preintegration_queue():
    while True:
        try:
            preintegration_queue.get_nowait()
            preintegration_queue.task_done()
        except Empty:
            break
        except Exception:
            break


def request_live_preintegration(session_dir: Path, frame_id: int) -> bool:
    """
    이미지가 저장될 때마다 최신 frame/imu 기준 사전적분 요청.
    큐가 이미 차 있으면 오래된 요청은 버리고 최신 요청만 남김.
    """
    if not ENABLE_IMU_PREINTEGRATION_LIVE:
        return False

    # Allan/static-noise captures need raw IMU only.  Do not spend GPU/CPU on
    # a continually rewritten prior for frames that are irrelevant to this
    # calibration task.
    if session_capture_purpose(session_dir) != CAPTURE_PURPOSE_SLAM:
        return False

    item = {
        "session_dir": str(session_dir),
        "frame_id": frame_id,
        "reason": f"live_frame_{frame_id}",
        "requested_at": time.time(),
    }

    if preintegration_queue.full():
        try:
            preintegration_queue.get_nowait()
            preintegration_queue.task_done()
        except Empty:
            pass
        except Exception:
            pass

    try:
        preintegration_queue.put_nowait(item)
        return True
    except Exception:
        return False


def preintegration_worker():
    """
    백그라운드 worker.
    프레임이 들어올 때마다 현재까지의 frames.csv + imu.csv 기준으로 imu_prior.csv 갱신.
    """
    while True:
        item = preintegration_queue.get()

        try:
            session_dir = Path(item["session_dir"])
            reason = item.get("reason", "live")

            result = run_imu_preintegration(
                session_dir=session_dir,
                reason=reason,
            )

            print(
                f"[IMU PREINTEGRATION LIVE] "
                f"reason={reason}, "
                f"status={result.get('status')}, "
                f"ok={result.get('ok')}, "
                f"output={result.get('output')}",
                flush=True,
            )

        except Exception as e:
            print(f"[IMU PREINTEGRATION LIVE ERROR] {e}", flush=True)

        finally:
            try:
                preintegration_queue.task_done()
            except Exception:
                pass


threading.Thread(target=preintegration_worker, daemon=True).start()


# =========================================================
# DROID-SLAM + PLY 실행 job
# =========================================================
def session_frame_geometry(session_dir: Path):
    frames_path = session_dir / "frames.csv"
    if not frames_path.exists():
        return None
    try:
        with open(frames_path, "r", newline="", encoding="utf-8") as stream:
            row = next(csv.DictReader(stream), None)
        if row is None:
            return None
        return int(row["width"]), int(row["height"])
    except (OSError, KeyError, TypeError, ValueError):
        return None


def session_row_count(csv_path: Path) -> int:
    if not csv_path.exists():
        return 0
    try:
        with open(csv_path, "r", newline="", encoding="utf-8") as stream:
            return max(0, sum(1 for _ in stream) - 1)
    except OSError:
        return 0


def resolve_droid_artifact(value: str, label: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = DROID_SLAM_DIR / path
    path = path.resolve()
    if DROID_SLAM_DIR != path and DROID_SLAM_DIR not in path.parents:
        raise ValueError(f"{label} must be inside DROID_SLAM_DIR")
    if not path.is_file():
        raise ValueError(f"{label} not found: {path}")
    return path


def get_runtime_settings(profile: dict, mode: str) -> dict:
    runtime = (profile.get("runtime") or {}).get(mode)
    if not isinstance(runtime, dict):
        raise ValueError(f"profile does not define runtime settings for mode={mode}")

    settings = {}
    for key, minimum in (
        ("stride", 1),
        ("buffer", 16),
        ("warmup", 1),
    ):
        try:
            value = int(runtime[key])
        except (KeyError, TypeError, ValueError):
            raise ValueError(f"runtime.{mode}.{key} must be an integer")
        if value < minimum:
            raise ValueError(f"runtime.{mode}.{key} must be >= {minimum}")
        settings[key] = value

    for key in ("filter_thresh", "keyframe_thresh"):
        try:
            value = float(runtime[key])
        except (KeyError, TypeError, ValueError):
            raise ValueError(f"runtime.{mode}.{key} must be a number")
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"runtime.{mode}.{key} must be positive")
        settings[key] = value

    settings["weights"] = str(resolve_droid_artifact(runtime.get("weights", ""), "weights"))
    return settings


def append_profile_runtime_args(command: list, profile: dict):
    """Use the exact calibration convention used to build ``imu_prior.csv``.

    The 15DoF runtime validates the prior's R/t *and* its noise and bias
    random-walk provenance.  Passing just R/t/bias here made a calibrated
    phone prior appear inconsistent at runtime because ``demo.py`` then
    resolved the four noise values to its defaults.  Keep this list aligned
    with :func:`profile_preintegration_args`.
    """
    imu = profile.get("imu") or {}
    for field_name, size, option in (
        ("R_imu_to_camera", 9, "--imu_to_cam_rotation"),
        ("t_imu_to_camera_m", 3, "--imu_to_cam_translation"),
        ("gyro_bias_rad_s", 3, "--gyro_bias"),
        ("accel_bias_m_s2", 3, "--acc_bias"),
    ):
        vector = finite_vector(imu.get(field_name), size)
        if vector is None:
            raise ValueError(f"missing calibrated imu.{field_name}")
        command.extend([option, *[f"{value:.12g}" for value in vector]])

    for field_name, option in (
        ("gyro_noise_density", "--gyro_noise_density"),
        ("gyro_random_walk", "--gyro_random_walk"),
        ("accel_noise_density", "--accel_noise_density"),
        ("accel_random_walk", "--accel_random_walk"),
    ):
        try:
            value = float(imu.get(field_name))
        except (TypeError, ValueError):
            value = float("nan")
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"missing calibrated imu.{field_name}")
        command.extend([option, f"{value:.12g}"])


def build_droid_command(session_dir: Path, profile: dict, mode: str):
    if mode not in {"visual_only", "imu_15dof"}:
        raise ValueError("mode must be visual_only or imu_15dof")
    if not DROID_PYTHON.is_file():
        raise ValueError(f"DROID_PYTHON not found: {DROID_PYTHON}")
    if not DROID_DEMO_SCRIPT.is_file():
        raise ValueError(f"DROID demo.py not found: {DROID_DEMO_SCRIPT}")

    errors = profile_validation_errors(profile, require_imu=(mode == "imu_15dof"))
    if errors:
        raise ValueError("; ".join(errors))

    expected = profile.get("capture") or {}
    geometry = session_frame_geometry(session_dir)
    if geometry != (expected.get("width"), expected.get("height")):
        raise ValueError(
            "captured frame geometry does not match the selected device profile: "
            f"captured={geometry}, profile={(expected.get('width'), expected.get('height'))}"
        )

    settings = get_runtime_settings(profile, mode)
    calib_path = session_dir / "calib.txt"
    if not calib_path.is_file():
        raise ValueError("calib.txt is missing; attach a calibrated device profile first")

    reconstruction_path = session_dir / f"reconstruction_{mode}.pth"
    command = [
        str(DROID_PYTHON),
        str(DROID_DEMO_SCRIPT),
        "--imagedir", str(session_dir / "images"),
        "--calib", str(calib_path),
        "--weights", settings["weights"],
        "--stride", str(settings["stride"]),
        # The phone profile controls tracking cadence explicitly.  The current
        # deployment profile records 10 Hz and tracks every input (stride 1);
        # dense export remains on every available camera timestamp.
        "--trajectory_fill_stride", "1",
        "--buffer", str(settings["buffer"]),
        "--filter_thresh", str(settings["filter_thresh"]),
        "--warmup", str(settings["warmup"]),
        "--keyframe_thresh", str(settings["keyframe_thresh"]),
        "--disable_vis",
        "--reconstruction_path", str(reconstruction_path),
    ]

    if mode == "imu_15dof":
        imu_runtime = (profile.get("runtime") or {}).get("imu_15dof") or {}
        use_cross_modal_alpha = imu_runtime.get(
            "use_learned_imu_cross_modal_calibration", False
        )
        if not isinstance(use_cross_modal_alpha, bool):
            raise ValueError(
                "runtime.imu_15dof.use_learned_imu_cross_modal_calibration "
                "must be a boolean"
            )
        command.extend([
            "--imu_prior", str(session_dir / "imu_prior.csv"),
            "--imu_frame_manifest", str(session_dir / "frames.csv"),
            "--use_two_stage_imu_ba",
            "--imu_two_stage_visual_steps", "3",
            "--imu_two_stage_full_steps", "2",
            "--imu_two_stage_global_visual_pre_steps", "7",
            "--imu_two_stage_global_steps", "12",
            "--no_vi_init_bootstrap",
            "--use_imu_noise_model",
            "--use_imu_full_covariance_weighting",
            "--use_imu_bias_random_walk",
            "--use_learned_imu_precision",
            "--imu_ba_debug",
            "--imu_ba_debug_path", str(session_dir / "imu_ba_debug.csv"),
        ])
        if use_cross_modal_alpha:
            command.append("--use_learned_imu_cross_modal_calibration")
        for profile_key, option in (
            ("imu_full_max_dt_sec", "--imu_full_max_dt"),
            ("vi_init_max_dt_sec", "--vi_init_max_dt"),
        ):
            if profile_key in imu_runtime:
                try:
                    max_dt = float(imu_runtime[profile_key])
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"runtime.imu_15dof.{profile_key} must be a number") from exc
                if not math.isfinite(max_dt) or max_dt < 0.0:
                    raise ValueError(f"runtime.imu_15dof.{profile_key} must be finite and >= 0")
                command.extend([option, f"{max_dt:.12g}"])
        append_profile_runtime_args(command, profile)

    return command, reconstruction_path


def build_ply_command(reconstruction_path: Path, session_dir: Path, profile: dict, mode: str):
    ply = profile.get("ply") or {}
    try:
        threshold = float(ply.get("filter_threshold"))
        count = int(ply.get("filter_count"))
    except (TypeError, ValueError):
        raise ValueError("profile.ply requires numeric filter_threshold and filter_count")
    if not math.isfinite(threshold) or threshold < 0 or count < 0:
        raise ValueError("invalid PLY filter settings")
    align_gravity_up = ply.get("align_gravity_up", False)
    if not isinstance(align_gravity_up, bool):
        raise ValueError("profile.ply.align_gravity_up must be a boolean")
    if align_gravity_up and mode != "imu_15dof":
        raise ValueError("profile.ply.align_gravity_up requires imu_15dof metric reconstruction")
    if not PLY_EXPORT_SCRIPT.is_file():
        raise ValueError(f"PLY export script not found: {PLY_EXPORT_SCRIPT}")

    suffix = f"t{threshold:.3f}_c{count}"
    output_path = session_dir / f"reconstruction_{mode}_{suffix}.ply"
    command = [
        str(DROID_PYTHON),
        str(PLY_EXPORT_SCRIPT),
        str(reconstruction_path),
        "--output", str(output_path),
        "--filter_threshold", str(threshold),
        "--filter_count", str(count),
    ]
    if align_gravity_up:
        command.append("--align-gravity-up")
    return command, output_path


def read_slam_job(session_dir: Path):
    state = read_json(session_dir / "slam_job.json", default={})
    return state if isinstance(state, dict) else {}


def write_slam_job(session_dir: Path, state: dict):
    write_json(session_dir / "slam_job.json", state)


def run_slam_job(session_dir: Path, profile: dict, mode: str):
    """Run preintegration, DROID, then PLY serially and leave an auditable job state."""
    state = read_slam_job(session_dir)
    state.update({
        "status": "running",
        "mode": mode,
        "device_profile_id": profile["id"],
        "started_at": time.time(),
        "finished_at": None,
        "error": None,
    })
    write_slam_job(session_dir, state)

    try:
        sync_result = build_synced_json(session_dir)
        preintegration = run_imu_preintegration(session_dir, f"slam_{mode}")
        if not preintegration.get("ok"):
            raise RuntimeError(f"IMU preintegration failed: {preintegration.get('message')}")

        droid_command, reconstruction_path = build_droid_command(session_dir, profile, mode)
        droid_log = session_dir / f"droid_{mode}.log"
        with open(droid_log, "w", encoding="utf-8") as stream:
            stream.write("command=" + " ".join(droid_command) + "\n\n")
            stream.flush()
            droid_result = subprocess.run(
                droid_command,
                cwd=str(DROID_SLAM_DIR),
                stdout=stream,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=DROID_RUN_TIMEOUT_SEC,
            )
        if droid_result.returncode != 0:
            raise RuntimeError(f"DROID failed with returncode={droid_result.returncode}; see {droid_log}")
        if not reconstruction_path.is_file():
            raise RuntimeError("DROID returned success but reconstruction .pth was not created")

        ply_command, ply_path = build_ply_command(reconstruction_path, session_dir, profile, mode)
        ply_log = session_dir / f"ply_{mode}.log"
        with open(ply_log, "w", encoding="utf-8") as stream:
            stream.write("command=" + " ".join(ply_command) + "\n\n")
            stream.flush()
            ply_result = subprocess.run(
                ply_command,
                cwd=str(DROID_SLAM_DIR),
                stdout=stream,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=DROID_RUN_TIMEOUT_SEC,
            )
        if ply_result.returncode != 0 or not ply_path.is_file():
            raise RuntimeError(f"PLY export failed; see {ply_log}")

        state.update({
            "status": "done",
            "sync": sync_result,
            "preintegration": preintegration,
            "droid_command": droid_command,
            "droid_log": str(droid_log),
            "reconstruction": str(reconstruction_path),
            "ply_command": ply_command,
            "ply_log": str(ply_log),
            "ply": str(ply_path),
        })
    except subprocess.TimeoutExpired:
        state.update({"status": "failed", "error": f"job timeout after {DROID_RUN_TIMEOUT_SEC}s"})
    except Exception as exc:
        state.update({"status": "failed", "error": str(exc)})
    finally:
        state["finished_at"] = time.time()
        write_slam_job(session_dir, state)
        update_session_metadata(session_dir, {
            "last_slam_job": {
                "status": state.get("status"),
                "mode": mode,
                "finished_at": state["finished_at"],
                "reconstruction": state.get("reconstruction"),
                "ply": state.get("ply"),
            }
        })


def start_slam_job(session_dir: Path, mode: str):
    profile = get_session_profile(session_dir)
    frame_count = session_row_count(session_dir / "frames.csv")
    if frame_count < 20:
        raise ValueError("at least 20 frames are required before running DROID-SLAM")
    if mode == "imu_15dof" and session_row_count(session_dir / "imu.csv") < 100:
        raise ValueError("at least 100 IMU samples are required for 15DoF mode")

    # Validate before responding so an uncalibrated S20 template cannot enqueue a fake run.
    build_droid_command(session_dir, profile, mode)

    with slam_job_lock:
        current = read_slam_job(session_dir)
        if current.get("status") in {"queued", "running"}:
            raise ValueError("a SLAM job is already running for this session")
        queued = {
            "status": "queued",
            "mode": mode,
            "device_profile_id": profile["id"],
            "queued_at": time.time(),
        }
        write_slam_job(session_dir, queued)

    threading.Thread(
        target=run_slam_job,
        args=(session_dir, profile, mode),
        daemon=True,
    ).start()
    return queued


# =========================================================
# WebSocket 통합 수신
# =========================================================
@app.websocket("/ws/stream")
async def websocket_stream(websocket: WebSocket):
    await websocket.accept()

    print("[WEBSOCKET CONNECTED] 클라이언트 연결됨", flush=True)

    pending_frame_meta = None

    imu_received_count = 0
    frame_received_count = 0
    current_session_id = None
    # One capture session must not mix wall-clock, browser and device-monotonic
    # timestamps. The first accepted data message fixes the time base.
    session_timestamp_source = None

    try:
        while True:
            message = await websocket.receive()

            # -------------------------------------------------
            # JSON 메시지 수신
            # -------------------------------------------------
            if message.get("text") is not None:
                try:
                    payload = json.loads(message["text"])
                except json.JSONDecodeError:
                    await websocket.send_text(json.dumps({
                        "ok": False,
                        "type": "error",
                        "message": "invalid json message",
                    }, ensure_ascii=False))
                    continue

                msg_type = payload.get("type")

                # ---------------------------------------------
                # 세션 시작
                # ---------------------------------------------
                if msg_type == "start":
                    session_id = payload.get("session_id")
                    requested_profile_id = payload.get(
                        "profile_id", DEFAULT_DEVICE_PROFILE_ID
                    )
                    profile_id = FORCE_DEVICE_PROFILE_ID or requested_profile_id

                    if not session_id:
                        await websocket.send_text(json.dumps({
                            "ok": False,
                            "type": "error",
                            "message": "session_id missing",
                        }, ensure_ascii=False))
                        continue

                    session_id = safe_session_id(session_id)
                    current_session_id = session_id
                    session_dir = get_session_dir(session_id)

                    try:
                        capture_purpose = parse_capture_purpose(
                            payload.get("capture_purpose")
                        )
                    except ValueError as exc:
                        await websocket.send_text(json.dumps({
                            "ok": False,
                            "type": "error",
                            "message": str(exc),
                        }, ensure_ascii=False))
                        continue

                    try:
                        profile_view = configure_session_profile(session_dir, profile_id)
                    except ValueError as exc:
                        await websocket.send_text(json.dumps({
                            "ok": False,
                            "type": "error",
                            "message": str(exc),
                        }, ensure_ascii=False))
                        continue

                    update_session_metadata(session_dir, {
                        "capture_purpose": capture_purpose,
                        "capture_purpose_note": (
                            "Raw native IMU stationary recording for Allan/noise "
                            "estimation; no preintegration or SLAM is run on stop."
                            if capture_purpose == CAPTURE_PURPOSE_IMU_NOISE_STATIC
                            else (
                                "Checkerboard target capture for camera-IMU "
                                "translation calibration; no preintegration or SLAM is run on stop."
                                if capture_purpose == CAPTURE_PURPOSE_IMU_CAMERA_TARGET
                                else "Regular camera/IMU capture for DROID-SLAM."
                            )
                        ),
                    })

                    imu_received_count = 0
                    frame_received_count = 0
                    pending_frame_meta = None
                    session_timestamp_source = None

                    declared_timestamp_source = str(
                        payload.get("timestamp_source") or ""
                    ).strip()
                    if declared_timestamp_source:
                        try:
                            set_session_timestamp_source(
                                session_dir,
                                declared_timestamp_source,
                            )
                            session_timestamp_source = declared_timestamp_source
                        except ValueError as exc:
                            await websocket.send_text(json.dumps({
                                "ok": False,
                                "type": "error",
                                "message": str(exc),
                            }, ensure_ascii=False))
                            continue

                    print(
                        f"[SESSION START] session_id={session_id}, "
                        f"purpose={capture_purpose}, path={session_dir}",
                        flush=True,
                    )

                    await websocket.send_text(json.dumps({
                        "ok": True,
                        "type": "started",
                        "session_id": session_id,
                        "session_dir": str(session_dir),
                        "device_profile": profile_view,
                        "capture_purpose": capture_purpose,
                        "live_preintegration": ENABLE_IMU_PREINTEGRATION_LIVE,
                        "imu_preintegrate_script": str(IMU_PREINTEGRATE_SCRIPT),
                        "imu_script_exists": IMU_PREINTEGRATE_SCRIPT.exists(),
                    }, ensure_ascii=False))

                # ---------------------------------------------
                # 프레임 메타데이터 수신
                # ---------------------------------------------
                elif msg_type == "frame_meta":
                    session_id = payload.get("session_id") or current_session_id

                    if not session_id:
                        await websocket.send_text(json.dumps({
                            "ok": False,
                            "type": "error",
                            "message": "session_id missing in frame_meta",
                        }, ensure_ascii=False))
                        continue

                    session_id = safe_session_id(session_id)
                    session_dir = get_session_dir(session_id)
                    try:
                        timestamp_sec, timestamp_ns, timestamp_source = parse_primary_timestamp(
                            payload,
                            "frame_meta",
                        )
                        if (
                            session_timestamp_source is not None
                            and timestamp_source != session_timestamp_source
                        ):
                            raise ValueError(
                                "camera and IMU timestamp_source differ: "
                                f"{session_timestamp_source!r} vs {timestamp_source!r}"
                            )
                        if session_timestamp_source is None:
                            set_session_timestamp_source(session_dir, timestamp_source)
                            session_timestamp_source = timestamp_source
                    except ValueError as exc:
                        await websocket.send_text(json.dumps({
                            "ok": False,
                            "type": "error",
                            "message": str(exc),
                        }, ensure_ascii=False))
                        continue

                    payload["session_id"] = session_id
                    # Freeze the capture time before asynchronous JPEG transfer.
                    payload["_capture_timestamp_sec"] = timestamp_sec
                    payload["_capture_timestamp_ns"] = timestamp_ns
                    payload["_timestamp_source"] = timestamp_source
                    pending_frame_meta = payload

                # ---------------------------------------------
                # IMU 수신
                # ---------------------------------------------
                elif msg_type == "imu":
                    session_id = payload.get("session_id") or current_session_id

                    if not session_id:
                        await websocket.send_text(json.dumps({
                            "ok": False,
                            "type": "error",
                            "message": "session_id missing in imu",
                        }, ensure_ascii=False))
                        continue

                    session_id = safe_session_id(session_id)
                    session_dir = get_session_dir(session_id)

                    try:
                        timestamp_sec, timestamp_ns, timestamp_source = parse_primary_timestamp(
                            payload,
                            "imu",
                        )
                        if (
                            session_timestamp_source is not None
                            and timestamp_source != session_timestamp_source
                        ):
                            raise ValueError(
                                "camera and IMU timestamp_source differ: "
                                f"{session_timestamp_source!r} vs {timestamp_source!r}"
                            )
                        if session_timestamp_source is None:
                            set_session_timestamp_source(session_dir, timestamp_source)
                            session_timestamp_source = timestamp_source
                    except ValueError as exc:
                        await websocket.send_text(json.dumps({
                            "ok": False,
                            "type": "error",
                            "message": str(exc),
                        }, ensure_ascii=False))
                        continue

                    sensor_timestamp_ns = int(
                        payload.get("sensor_timestamp_ns") or 0
                    )
                    gyro_timestamp_ns = int(
                        payload.get("gyro_timestamp_ns") or 0
                    )

                    gx, gy, gz, ax, ay, az = extract_imu_values(payload)

                    with open(
                        session_dir / "imu.csv",
                        "a",
                        newline="",
                        encoding="utf-8",
                    ) as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            f"{timestamp_sec:.9f}",
                            timestamp_ns,
                            gx,
                            gy,
                            gz,
                            ax,
                            ay,
                            az,
                            sensor_timestamp_ns,
                            gyro_timestamp_ns,
                        ])

                    imu_received_count += 1

                    if imu_received_count % 200 == 0:
                        print(
                            f"[IMU RECEIVED] "
                            f"count={imu_received_count}, "
                            f"time={timestamp_sec:.6f}, "
                            f"gyro=({gx:.4f}, {gy:.4f}, {gz:.4f}), "
                            f"accel=({ax:.4f}, {ay:.4f}, {az:.4f})",
                            flush=True,
                        )

                        await websocket.send_text(json.dumps({
                            "ok": True,
                            "type": "imu_saved",
                            "imu_received_count": imu_received_count,
                            "timestamp_ns": timestamp_ns,
                        }, ensure_ascii=False))

                # ---------------------------------------------
                # 수집 종료
                # stop이 오면 바로 끊지 않고:
                # 1. 현재까지 저장된 마지막 frame/imu 기준 synced.json 생성
                # 2. live queue 비움
                # 3. 최종 imu_preintegrate.py 실행 완료
                # 4. stopped 응답
                # ---------------------------------------------
                elif msg_type == "stop":
                    session_id = payload.get("session_id") or current_session_id

                    if not session_id:
                        await websocket.send_text(json.dumps({
                            "ok": False,
                            "type": "error",
                            "message": "session_id missing in stop",
                        }, ensure_ascii=False))
                        continue

                    session_id = safe_session_id(session_id)
                    session_dir = get_session_dir(session_id)
                    capture_purpose = session_capture_purpose(session_dir)

                    print(
                        f"[SESSION STOP REQUEST] session_id={session_id}, "
                        f"purpose={capture_purpose}",
                        flush=True,
                    )

                    if capture_purpose in {
                        CAPTURE_PURPOSE_IMU_NOISE_STATIC,
                        CAPTURE_PURPOSE_IMU_CAMERA_TARGET,
                    }:
                        # Both calibration-only capture modes bypass DROID. A
                        # static run needs raw IMU only, whereas a target run
                        # preserves checkerboard frames plus raw IMU for the
                        # offline metric translation estimator.
                        clear_preintegration_queue()
                        frame_count = session_row_count(session_dir / "frames.csv")
                        imu_count = session_row_count(session_dir / "imu.csv")
                        is_static_capture = (
                            capture_purpose == CAPTURE_PURPOSE_IMU_NOISE_STATIC
                        )
                        static_result = {
                            "ok": imu_count >= 100,
                            "status": (
                                "skipped_static_imu_noise_capture"
                                if is_static_capture
                                else "skipped_imu_camera_target_capture"
                            ),
                            "message": (
                                "Raw IMU static capture saved; run "
                                "estimate_phone_imu_allan.py after verifying duration."
                                if is_static_capture
                                else "Checkerboard/IMU target capture saved; run the "
                                "camera-IMU translation calibration tool."
                            ),
                            "output": str(
                                session_dir / ("imu.csv" if is_static_capture else "frames.csv")
                            ),
                            "output_exists": (
                                (session_dir / "imu.csv").is_file()
                                if is_static_capture
                                else (session_dir / "frames.csv").is_file()
                            ),
                        }
                        update_session_metadata(session_dir, {
                            "capture_completed_at": time.time(),
                            "capture_counts": {
                                "frames": frame_count,
                                "imu": imu_count,
                            },
                            "post_capture": (
                                "Allan/noise analysis pending"
                                if is_static_capture
                                else "Checkerboard target and camera-IMU translation analysis pending"
                            ),
                        })
                        print(
                            f"[CALIBRATION CAPTURE DONE] session_id={session_id}, "
                            f"purpose={capture_purpose}, "
                            f"frames={frame_count}, imu={imu_count}",
                            flush=True,
                        )
                        await websocket.send_text(json.dumps({
                            "ok": static_result["ok"],
                            "type": "stopped",
                            "message": (
                                "정지 IMU 수집 완료; 사전적분과 SLAM은 의도적으로 건너뜀"
                                if is_static_capture
                                else "checkerboard/IMU 수집 완료; 사전적분과 SLAM은 의도적으로 건너뜀"
                            ),
                            "session_id": session_id,
                            "session_dir": str(session_dir),
                            "capture_purpose": capture_purpose,
                            "droid_ready": False,
                            "droid_command": None,
                            "droid_error": "calibration-only capture does not run DROID-SLAM",
                            "preintegration": static_result,
                            "frame_count": frame_count,
                            "imu_count": imu_count,
                            "avg_imu_per_frame": (
                                float(imu_count) / frame_count if frame_count else None
                            ),
                        }, ensure_ascii=False))
                        continue

                    await websocket.send_text(json.dumps({
                        "ok": True,
                        "type": "preintegration_started",
                        "message": "마지막 frame/imu 기준 synced.json 생성 및 최종 imu_prior.csv 생성 시작",
                        "session_id": session_id,
                    }, ensure_ascii=False))

                    # 현재까지 저장된 마지막 frame/imu 기준 synced.json 생성
                    sync_result = build_synced_json(session_dir)

                    # stop 최종 계산 전에 밀린 live 요청 제거
                    clear_preintegration_queue()

                    # 최종 사전적분은 반드시 끝날 때까지 기다림
                    final_preintegration_result = await asyncio.to_thread(
                        run_imu_preintegration,
                        session_dir,
                        "stop_final",
                    )

                    try:
                        profile = get_session_profile(session_dir)
                        preview_command, _ = build_droid_command(
                            session_dir,
                            profile,
                            "visual_only",
                        )
                        droid_ready = True
                        droid_command = preview_command
                        droid_error = None
                    except ValueError as exc:
                        droid_ready = False
                        droid_command = None
                        droid_error = str(exc)

                    # AUTO_START_EXISTING_SLAM_JOB: enqueue the existing SLAM+PLY job.
                    # "visual_only" works with camera intrinsics only; "imu_15dof"
                    # uses the existing strict calibrated-device-profile gate.
                    # Set AUTO_SLAM_MODE=imu_15dof ONLY after team leader approves
                    # full 15DoF calibration and runtime parameters.
                    auto_mode = os.getenv("AUTO_SLAM_MODE", "visual_only").strip()
                    if final_preintegration_result.get("ok") and auto_mode in {"visual_only", "imu_15dof"}:
                        try:
                            job = start_slam_job(session_dir, auto_mode)
                            print(f"[AUTO SLAM] session={session_id}, queued={job}", flush=True)
                        except Exception as auto_exc:
                            write_slam_job(session_dir, {
                                "status": "failed", "mode": auto_mode,
                                "error": f"Cannot queue SLAM: {auto_exc}",
                                "finished_at": time.time(),
                            })
                            print(f"[AUTO SLAM ERROR] {auto_exc}", flush=True)
                    elif not final_preintegration_result.get("ok"):
                        write_slam_job(session_dir, {
                            "status": "failed", "mode": auto_mode,
                            "error": "Final IMU preintegration failed; SLAM not started",
                            "finished_at": time.time(),
                        })
                    else:
                        print(f"[AUTO SLAM] disabled: invalid AUTO_SLAM_MODE={auto_mode!r}", flush=True)

                    print(
                        f"[SESSION STOP DONE] "
                        f"session_id={session_id}, "
                        f"frames={sync_result['frame_count']}, "
                        f"imu={sync_result['imu_count']}, "
                        f"avg_imu_per_frame={sync_result['avg_imu_per_frame']:.2f}, "
                        f"preintegration={final_preintegration_result.get('status')}",
                        flush=True,
                    )

                    await websocket.send_text(json.dumps({
                        "ok": True,
                        "type": "stopped",
                        "message": "DROID-SLAM용 데이터 생성 및 최종 imu_prior.csv 생성 완료",
                        "session_id": session_id,
                        "session_dir": str(session_dir),
                        "capture_purpose": capture_purpose,
                        "droid_ready": droid_ready,
                        "droid_command": droid_command,
                        "droid_error": droid_error,
                        "imu_runtime_note": (
                            "15DoF mode is gated on calibrated camera intrinsics, camera-IMU "
                            "extrinsics, time offset, biases, and noise values."
                        ),
                        "preintegration": final_preintegration_result,
                        **sync_result,
                    }, ensure_ascii=False))

                else:
                    await websocket.send_text(json.dumps({
                        "ok": False,
                        "type": "error",
                        "message": f"unknown message type: {msg_type}",
                    }, ensure_ascii=False))

            # -------------------------------------------------
            # 이미지 binary bytes 수신
            # 반드시 직전에 frame_meta가 와야 함
            # -------------------------------------------------
            elif message.get("bytes") is not None:
                image_bytes = message["bytes"]

                if pending_frame_meta is None:
                    await websocket.send_text(json.dumps({
                        "ok": False,
                        "type": "error",
                        "message": "image binary received but frame_meta missing",
                    }, ensure_ascii=False))
                    continue

                session_id = pending_frame_meta.get("session_id") or current_session_id

                if not session_id:
                    await websocket.send_text(json.dumps({
                        "ok": False,
                        "type": "error",
                        "message": "session_id missing in pending frame_meta",
                    }, ensure_ascii=False))

                    pending_frame_meta = None
                    continue

                session_id = safe_session_id(session_id)
                session_dir = get_session_dir(session_id)
                images_dir = session_dir / "images"

                if session_capture_purpose(session_dir) == CAPTURE_PURPOSE_IMU_NOISE_STATIC:
                    # An unchanged client may keep transmitting frames.  The
                    # timestamp was already checked at frame_meta; discard the
                    # image payload here so a 20-minute noise capture does not
                    # fill storage or accidentally become a visual SLAM input.
                    pending_frame_meta = None
                    continue

                frame_id = get_next_frame_id(session_dir)

                image_format = str(
                    pending_frame_meta.get("format", "jpg")
                ).lower()

                if image_format in ["jpg", "jpeg"]:
                    ext = "jpg"
                    save_format = "jpg"
                elif image_format == "webp":
                    ext = "webp"
                    save_format = "webp"
                else:
                    ext = "jpg"
                    save_format = "jpg"

                filename = f"{frame_id:06d}.{ext}"
                image_path = images_dir / filename

                with open(image_path, "wb") as f:
                    f.write(image_bytes)

                timestamp_sec = pending_frame_meta["_capture_timestamp_sec"]
                timestamp_ns = pending_frame_meta["_capture_timestamp_ns"]
                timestamp_source = pending_frame_meta["_timestamp_source"]
                sensor_timestamp_ns = int(
                    pending_frame_meta.get("sensor_timestamp_ns") or 0
                )

                width = int(pending_frame_meta.get("width", 640))
                height = int(pending_frame_meta.get("height", 480))
                size_bytes = int(
                    pending_frame_meta.get("size_bytes", len(image_bytes))
                )

                server_saved_sec = time.time()
                clock_latency_ms = (server_saved_sec - timestamp_sec) * 1000.0

                client_send_perf_ms = pending_frame_meta.get("client_send_perf_ms")
                capture_start_perf_ms = pending_frame_meta.get("capture_start_perf_ms")

                with open(
                    session_dir / "frames.csv",
                    "a",
                    newline="",
                    encoding="utf-8",
                ) as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        frame_id,
                        f"{timestamp_sec:.9f}",
                        timestamp_ns,
                        filename,
                        width,
                        height,
                        save_format,
                        sensor_timestamp_ns,
                    ])

                with open(
                    session_dir / "times.txt",
                    "a",
                    encoding="utf-8",
                ) as f:
                    f.write(f"{timestamp_sec:.9f}\n")

                frame_received_count += 1

                # 이미지가 저장될 때마다 바로 사전적분 요청
                live_preintegration_queued = request_live_preintegration(
                    session_dir=session_dir,
                    frame_id=frame_id,
                )

                print(
                    f"[FRAME RECEIVED] "
                    f"count={frame_received_count}, "
                    f"frame_id={frame_id}, "
                    f"file={filename}, "
                    f"bytes={len(image_bytes)}, "
                    f"size={width}x{height}, "
                    f"format={save_format}, "
                    f"time={timestamp_sec:.6f}, "
                    f"preintegration_queued={live_preintegration_queued}",
                    flush=True,
                )

                await websocket.send_text(json.dumps({
                    "ok": True,
                    "type": "frame_saved",
                    "session_id": session_id,
                    "frame_received_count": frame_received_count,
                    "frame_id": frame_id,
                    "filename": filename,
                    "format": save_format,
                    "timestamp_ns": timestamp_ns,
                    "timestamp_source": timestamp_source,
                    "size_bytes": size_bytes,
                    "clock_latency_ms": round(clock_latency_ms, 2),
                    "client_send_perf_ms": client_send_perf_ms,
                    "capture_start_perf_ms": capture_start_perf_ms,
                    "live_preintegration_queued": live_preintegration_queued,
                }, ensure_ascii=False))

                pending_frame_meta = None

    except WebSocketDisconnect:
        print("[WEBSOCKET DISCONNECTED] 클라이언트 연결 종료", flush=True)

    except RuntimeError as exc:
        # Starlette may raise RuntimeError instead of WebSocketDisconnect when
        # a browser closes immediately after the final server message.
        if "disconnect" in str(exc).lower() or "cannot call \"receive\"" in str(exc).lower():
            print("[WEBSOCKET DISCONNECTED] 클라이언트 연결 종료", flush=True)
        else:
            print(f"[WEBSOCKET ERROR] {exc}", flush=True)

    except Exception as e:
        print(f"[WEBSOCKET ERROR] {e}", flush=True)

        try:
            await websocket.send_text(json.dumps({
                "ok": False,
                "type": "error",
                "message": str(e),
            }, ensure_ascii=False))
        except Exception:
            pass


# =========================================================
# 세션 확인용
# =========================================================
@app.get("/session/{session_id}/summary")
def session_summary(session_id: str):
    session_id = safe_session_id(session_id)
    session_dir = get_session_dir(session_id)

    frames_path = session_dir / "frames.csv"
    imu_path = session_dir / "imu.csv"
    synced_path = session_dir / "synced.json"
    imu_prior_path = session_dir / "imu_prior.csv"
    status_path = session_dir / "imu_preintegrate_status.json"

    frame_count = 0
    imu_count = 0
    synced_count = 0
    imu_prior_count = 0

    if frames_path.exists():
        with open(frames_path, "r", newline="", encoding="utf-8") as f:
            frame_count = max(0, len(list(csv.reader(f))) - 1)

    if imu_path.exists():
        with open(imu_path, "r", newline="", encoding="utf-8") as f:
            imu_count = max(0, len(list(csv.reader(f))) - 1)

    if synced_path.exists():
        try:
            synced = json.loads(synced_path.read_text(encoding="utf-8"))
            synced_count = len(synced)
        except Exception:
            synced_count = 0

    if imu_prior_path.exists():
        with open(imu_prior_path, "r", newline="", encoding="utf-8") as f:
            imu_prior_count = max(0, len(list(csv.reader(f))) - 1)

    preintegration_status = None

    if status_path.exists():
        try:
            preintegration_status = json.loads(
                status_path.read_text(encoding="utf-8")
            )
        except Exception:
            preintegration_status = {
                "ok": False,
                "status": "status_json_read_failed",
            }

    try:
        device_profile = profile_public_view(get_session_profile(session_dir))
    except ValueError as exc:
        device_profile = {"status": "missing", "error": str(exc)}

    return {
        "ok": True,
        "session_id": session_id,
        "session_dir": str(session_dir),
        "frame_count": frame_count,
        "imu_count": imu_count,
        "synced_count": synced_count,
        "imu_prior_count": imu_prior_count,
        "imu_prior_exists": imu_prior_path.exists(),
        "preintegration_status": preintegration_status,
        "imu_preintegrate_script": str(IMU_PREINTEGRATE_SCRIPT),
        "imu_script_exists": IMU_PREINTEGRATE_SCRIPT.exists(),
        "device_profile": device_profile,
        "slam_job": read_slam_job(session_dir),
    }


# =========================================================
# 저장 세션 목록 확인용
# =========================================================
@app.get("/sessions")
def list_sessions():
    sessions = []

    if not BASE_UPLOAD_DIR.exists():
        return {
            "ok": True,
            "sessions": [],
        }

    for session_dir in sorted(BASE_UPLOAD_DIR.iterdir(), reverse=True):
        if not session_dir.is_dir():
            continue

        frames_path = session_dir / "frames.csv"
        imu_path = session_dir / "imu.csv"
        synced_path = session_dir / "synced.json"
        imu_prior_path = session_dir / "imu_prior.csv"
        status_path = session_dir / "imu_preintegrate_status.json"

        frame_count = 0
        imu_count = 0
        synced_count = 0
        imu_prior_count = 0

        if frames_path.exists():
            with open(frames_path, "r", newline="", encoding="utf-8") as f:
                frame_count = max(0, len(list(csv.reader(f))) - 1)

        if imu_path.exists():
            with open(imu_path, "r", newline="", encoding="utf-8") as f:
                imu_count = max(0, len(list(csv.reader(f))) - 1)

        if synced_path.exists():
            try:
                synced = json.loads(synced_path.read_text(encoding="utf-8"))
                synced_count = len(synced)
            except Exception:
                synced_count = 0

        if imu_prior_path.exists():
            with open(imu_prior_path, "r", newline="", encoding="utf-8") as f:
                imu_prior_count = max(0, len(list(csv.reader(f))) - 1)

        preintegration_status = "not_run"

        if status_path.exists():
            try:
                status_json = json.loads(status_path.read_text(encoding="utf-8"))
                preintegration_status = status_json.get("status", "unknown")
            except Exception:
                preintegration_status = "status_read_failed"

        sessions.append({
            "session_id": session_dir.name,
            "frame_count": frame_count,
            "imu_count": imu_count,
            "synced_count": synced_count,
            "imu_prior_count": imu_prior_count,
            "imu_prior_exists": imu_prior_path.exists(),
            "preintegration_status": preintegration_status,
            "summary_url": f"/session/{session_dir.name}/summary",
            "download_url": f"/session/{session_dir.name}/download",
            "preintegrate_url": f"/session/{session_dir.name}/preintegrate",
            "run_url": f"/session/{session_dir.name}/run",
            "slam_job": read_slam_job(session_dir),
        })

    return {
        "ok": True,
        "sessions": sessions,
    }


# =========================================================
# 저장 세션 ZIP 다운로드
# =========================================================
@app.get("/session/{session_id}/download")
def download_session(session_id: str):
    session_id = safe_session_id(session_id)
    session_dir = BASE_UPLOAD_DIR / session_id

    if not session_dir.exists() or not session_dir.is_dir():
        return JSONResponse(
            status_code=404,
            content={
                "ok": False,
                "message": f"session not found: {session_id}",
            },
        )

    zip_path = session_dir / f"{session_id}.zip"

    if zip_path.exists():
        zip_path.unlink()

    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zip_file:
        for file_path in session_dir.rglob("*"):
            if file_path == zip_path:
                continue

            if file_path.is_file():
                arcname = file_path.relative_to(session_dir.parent)
                zip_file.write(file_path, arcname)

    return FileResponse(
        zip_path,
        media_type="application/zip",
        filename=f"{session_id}.zip",
    )


# =========================================================
# 수동 IMU 사전적분 실행 API
# =========================================================
@app.post("/session/{session_id}/preintegrate")
async def preintegrate_api(session_id: str):
    session_id = safe_session_id(session_id)
    session_dir = BASE_UPLOAD_DIR / session_id

    if not session_dir.exists() or not session_dir.is_dir():
        return JSONResponse(
            status_code=404,
            content={
                "ok": False,
                "message": f"session not found: {session_id}",
            },
        )

    sync_result = build_synced_json(session_dir)

    clear_preintegration_queue()

    preintegration_result = await asyncio.to_thread(
        run_imu_preintegration,
        session_dir,
        "manual_api",
    )

    return {
        "ok": True,
        "session_id": session_id,
        "session_dir": str(session_dir),
        "preintegration": preintegration_result,
        **sync_result,
    }


# =========================================================
# 세션별 DROID-SLAM + filtered PLY 실행 API
# =========================================================
@app.post("/session/{session_id}/run")
async def run_session_slam(session_id: str, payload: dict = None):
    session_id = safe_session_id(session_id)
    session_dir = BASE_UPLOAD_DIR / session_id
    payload = payload or {}

    if not session_dir.exists() or not session_dir.is_dir():
        return JSONResponse(
            status_code=404,
            content={"ok": False, "message": f"session not found: {session_id}"},
        )

    mode = payload.get("mode", "visual_only")
    if mode not in {"visual_only", "imu_15dof"}:
        return JSONResponse(
            status_code=400,
            content={"ok": False, "message": "mode must be visual_only or imu_15dof"},
        )

    try:
        job = start_slam_job(session_dir, mode)
    except ValueError as exc:
        return JSONResponse(status_code=409, content={"ok": False, "message": str(exc)})

    return JSONResponse(
        status_code=202,
        content={
            "ok": True,
            "message": "SLAM job queued: final IMU preintegration, DROID reconstruction, filtered PLY export",
            "session_id": session_id,
            "status_url": f"/session/{session_id}/summary",
            "job": job,
        },
    )


# =========================================================
# 예전 프론트 호환용 IMU 사전적분 API
# =========================================================
@app.post("/preintegrate-imu")
async def preintegrate_imu_legacy(payload: dict):
    session_id = payload.get("session_id")

    if not session_id:
        return JSONResponse(
            status_code=400,
            content={
                "ok": False,
                "message": "session_id missing",
            },
        )

    session_id = safe_session_id(session_id)
    session_dir = BASE_UPLOAD_DIR / session_id

    if not session_dir.exists() or not session_dir.is_dir():
        return JSONResponse(
            status_code=404,
            content={
                "ok": False,
                "message": f"session not found: {session_id}",
            },
        )

    sync_result = build_synced_json(session_dir)
    clear_preintegration_queue()

    preintegration_result = await asyncio.to_thread(
        run_imu_preintegration,
        session_dir,
        "legacy_api",
    )

    return {
        "ok": bool(preintegration_result.get("ok")),
        "session_id": session_id,
        "session_dir": str(session_dir),
        "preintegration": preintegration_result,
        "imu_prior_path": preintegration_result.get("output"),
        "log_path": preintegration_result.get("log"),
        **sync_result,
    }


# =========================================================
# 기존 HTTP trigger 비활성화 안내
# =========================================================
@app.post("/trigger-slam")
async def trigger_slam_legacy():
    return JSONResponse(
        status_code=400,
        content={
            "ok": False,
            "message": (
                "이 버전에서는 /trigger-slam을 사용하지 않습니다. "
                "/ws/stream의 stop 메시지 또는 /session/{session_id}/preintegrate를 사용하세요."
            ),
        },
    )


# Viewer for existing slam_job.json and exported PLY.
try:
    # Package import when launched as `python -m uvicorn phone_server.main:app`.
    from .existing_slam_viewer import router as existing_slam_viewer_router
except ImportError:
    # Direct import when the combined repository launches from `SLAM/`.
    from existing_slam_viewer import router as existing_slam_viewer_router
app.include_router(existing_slam_viewer_router)
