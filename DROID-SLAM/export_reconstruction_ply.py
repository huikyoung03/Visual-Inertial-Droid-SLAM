import sys
sys.path.append("droid_slam")

import argparse
import numpy as np
import torch
import open3d as o3d
import droid_backends

from lietorch import SE3
from cuda_timer import CudaTimer


def _scalar(blob, key, default=0.0):
    value = blob.get(key, default)
    if torch.is_tensor(value):
        value = value.detach().cpu().reshape(-1)
        if value.numel() == 0:
            return float(default)
        value = value[0].item()
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _metricize_if_aligned(reconstruction_blob, poses, disps, enabled):
    """Convert a VI-aligned, normalized reconstruction back to metres."""

    aligned = _scalar(reconstruction_blob, "imu_alignment_ready", 0.0) > 0.5
    unit_scale = _scalar(
        reconstruction_blob,
        "pose_translation_unit_scale",
        _scalar(reconstruction_blob, "imu_unit_scale", 0.0),
    )
    if not enabled or not aligned or not torch.isfinite(torch.tensor(unit_scale)) or unit_scale <= 1e-12:
        return poses, disps, False

    # Pose translation has DROID-unit/metre scale U. Inverse depth has the
    # reciprocal unit, so p <- p/U and disparity <- disparity*U.
    poses = poses.clone()
    poses[:, :3] /= unit_scale
    disps = disps * unit_scale
    return poses, disps, True


def _gravity_upright_rotation(reconstruction_blob):
    """Return an SO(3) rotation that maps measured gravity to negative Z.

    This changes only the exported visualization frame. It preserves every
    inter-point distance and leaves the saved SLAM reconstruction untouched.
    """
    gravity = reconstruction_blob.get("imu_gravity_metric")
    if gravity is None:
        gravity = reconstruction_blob.get("imu_gravity")
        scale = _scalar(reconstruction_blob, "pose_translation_unit_scale", 0.0)
        if gravity is not None and scale > 1e-12:
            gravity = torch.as_tensor(gravity) / scale
    gravity = torch.as_tensor(gravity if gravity is not None else []).detach().cpu().reshape(-1).numpy()
    if gravity.size < 3 or not np.all(np.isfinite(gravity[:3])):
        raise ValueError("gravity-up PLY export requires a finite IMU gravity vector")
    source = gravity[:3] / max(float(np.linalg.norm(gravity[:3])), 1e-12)
    target = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    cross = np.cross(source, target)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if sine <= 1e-12:
        if cosine > 0.0:
            return np.eye(3, dtype=np.float64)
        # 180 degrees around X maps +Z to -Z.
        return np.diag([1.0, -1.0, -1.0])
    skew = np.array([
        [0.0, -cross[2], cross[1]],
        [cross[2], 0.0, -cross[0]],
        [-cross[1], cross[0], 0.0],
    ])
    return np.eye(3) + skew + skew @ skew * ((1.0 - cosine) / (sine * sine))


def _common_frame_indices(reconstruction_blob, other_filename):
    """Return one index per frame ID shared with another reconstruction."""

    if "tstamps" not in reconstruction_blob:
        raise KeyError("reconstruction is missing tstamps required for common-frame export")

    other_blob = torch.load(other_filename, map_location="cpu")
    if "tstamps" not in other_blob:
        raise KeyError(
            f"comparison reconstruction is missing tstamps: {other_filename}"
        )

    other_ids = {
        int(round(float(timestamp)))
        for timestamp in other_blob["tstamps"].detach().cpu().reshape(-1)
    }
    indices = []
    seen = set()
    for index, timestamp in enumerate(
        reconstruction_blob["tstamps"].detach().cpu().reshape(-1)
    ):
        frame_id = int(round(float(timestamp)))
        if frame_id in other_ids and frame_id not in seen:
            indices.append(index)
            seen.add(frame_id)

    if not indices:
        raise ValueError(
            "reconstructions have no shared frame IDs; cannot export a common PLY"
        )
    return indices, len(seen)


def export_ply(
    filename,
    output,
    filter_thresh=0.0,
    filter_count=0,
    metricize_imu=True,
    common_with=None,
    color_by_time=False,
    filter_in_raw_units=False,
    align_gravity_up=False,
):
    reconstruction_blob = torch.load(filename, map_location="cpu")

    indices = None
    if common_with:
        indices, common_count = _common_frame_indices(reconstruction_blob, common_with)
        print(f"[PLY] retaining {common_count} frame IDs shared with {common_with}")

    images = reconstruction_blob["images"]
    disps = reconstruction_blob["disps"]
    poses = reconstruction_blob["poses"]
    intrinsics = reconstruction_blob["intrinsics"]
    if indices is not None:
        images = images[indices]
        disps = disps[indices]
        poses = poses[indices]
        intrinsics = intrinsics[indices]

    images = images.cuda()[..., ::2, ::2]
    disps = disps.cuda()[..., ::2, ::2]
    poses = poses.cuda()
    intrinsics = 4 * intrinsics.cuda()
    poses, disps, metricized = _metricize_if_aligned(
        reconstruction_blob,
        poses,
        disps,
        metricize_imu,
    )
    if metricized:
        print("[PLY] exported in metres using IMU visual-inertial alignment")

    disps = disps.contiguous()
    index = torch.arange(len(images), device="cuda")
    effective_filter_thresh = filter_thresh
    if metricized and filter_in_raw_units:
        # depth_filter compares 1/disparity, i.e. depth. Metricization scales
        # depth by 1/U, so the same raw-unit tolerance becomes threshold/U.
        unit_scale = _scalar(
            reconstruction_blob,
            "pose_translation_unit_scale",
            _scalar(reconstruction_blob, "imu_unit_scale", 0.0),
        )
        effective_filter_thresh = filter_thresh / unit_scale
        print(
            "[PLY] depth-filter threshold in raw DROID units: "
            f"{filter_thresh:g} (metric equivalent {effective_filter_thresh:g} m)"
        )
    thresh = effective_filter_thresh * torch.ones_like(disps.mean(dim=[1, 2]))

    with CudaTimer("iproj"):
        points = droid_backends.iproj(SE3(poses).inv().data, disps, intrinsics[0])
        colors = images[:, [2, 1, 0]].permute(0, 2, 3, 1) / 255.0
        if color_by_time:
            phase = torch.linspace(0, 1, len(images), device=images.device)
            palette = torch.stack(
                (phase, 1 - torch.abs(2 * phase - 1), 1 - phase), dim=-1
            )
            colors = palette[:, None, None, :].expand_as(colors)

    with CudaTimer("filter"):
        counts = droid_backends.depth_filter(poses, disps, intrinsics[0], index, thresh)
        mask = (counts >= filter_count) & (disps > 0.25 * disps.mean())

    points_np = points[mask].detach().cpu().numpy()
    colors_np = colors[mask].detach().cpu().numpy()

    if align_gravity_up:
        if not metricized:
            raise ValueError("--align-gravity-up requires a successful metric IMU alignment")
        rotation = _gravity_upright_rotation(reconstruction_blob)
        points_np = points_np @ rotation.T
        print("[PLY] rotated visualization frame so IMU gravity maps to negative Z")

    print("points:", points_np.shape)
    print("colors:", colors_np.shape)

    if points_np.shape[0] == 0:
        print("[ERROR] no points to save")
        return

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points_np)
    point_cloud.colors = o3d.utility.Vector3dVector(colors_np)

    ok = o3d.io.write_point_cloud(output, point_cloud)
    print("saved:", output)
    print("write ok:", ok)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filename")
    parser.add_argument("--output", required=True)
    parser.add_argument("--filter_threshold", type=float, default=0.0)
    parser.add_argument("--filter_count", type=int, default=0)
    parser.add_argument(
        "--filter-in-raw-units", action="store_true",
        help="interpret filter_threshold in reconstruction DROID units even when exporting a metric PLY",
    )
    parser.add_argument(
        "--color-by-time", action="store_true",
        help="diagnostic colors: blue=early, green=middle, red=late keyframes",
    )
    parser.add_argument(
        "--common-with",
        default=None,
        help="export only frame IDs shared with this reconstruction .pth file",
    )
    parser.add_argument(
        "--raw-units",
        dest="metricize_imu",
        action="store_false",
        help="do not convert a VI-aligned reconstruction from DROID units to metres",
    )
    parser.add_argument(
        "--align-gravity-up",
        action="store_true",
        help="rotate only the exported metric PLY so IMU gravity points along negative Z",
    )
    parser.set_defaults(metricize_imu=True)
    args = parser.parse_args()

    export_ply(
        args.filename,
        args.output,
        args.filter_threshold,
        args.filter_count,
        args.metricize_imu,
        args.common_with,
        args.color_by_time,
        args.filter_in_raw_units,
        args.align_gravity_up,
    )
