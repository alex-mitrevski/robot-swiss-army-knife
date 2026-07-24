import numpy as np
from . import constants as C

def _z_from_depth_map(mask, depth):
    rows, cols = np.where(mask > 0)
    if len(rows) == 0:
        return None
    Z = float(np.median(depth[rows, cols]))
    return Z if Z > 0.0 else None


def _pca_axes(xyz, mask, min_pts=30):
    rows, cols = np.where(mask > 0)
    if len(rows) == 0:
        return None, None
    pts = xyz[rows, cols]
    valid = pts[:, 2] > 0
    pts = pts[valid]
    if len(pts) < min_pts:
        return None, None
    centroid = pts.mean(axis=0)
    centered = pts - centroid
    _, _, Vt = np.linalg.svd(centered, full_matrices=False)
    Vt[2] = np.cross(Vt[0], Vt[1])
    return centroid, Vt


def _z_from_pointcloud(xyz, mask):
    """Median Z over the masked pixels of an organised cloud frame."""
    rows, cols = np.where(mask > 0)
    if len(rows) == 0:
        return None
    z = xyz[rows, cols, 2]
    valid = z > 0
    if valid.sum() < 5:
        return None
    return float(np.median(z[valid]))


def _xyz_from_depth_map(depth, fx, fy, cx, cy):
    H, W = depth.shape
    uu, vv = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    Z = depth.astype(float)
    return np.stack([(uu - cx) * Z / fx, (vv - cy) * Z / fy, Z], axis=-1)


def _correct_centroid_partial_occ(x1, y1, x2, y2, clean_box, ref_W, ref_H,
                                   img_W=None, img_H=None):
    cx1, cy1, cx2, cy2 = clean_box
    shrink_left  = x1  - cx1
    shrink_right = cx2 - x2
    shrink_top   = y1  - cy1
    shrink_bot   = cy2 - y2

    if (img_W is not None) and (x1 <= 0 or x2 >= img_W - 1):
        u = (x1 + x2) / 2.0
    elif shrink_left >= shrink_right:
        u = x2 - ref_W / 2.0
    else:
        u = x1 + ref_W / 2.0

    if (img_H is not None) and (y1 <= 0 or y2 >= img_H - 1):
        v = (y1 + y2) / 2.0
    elif shrink_top >= shrink_bot:
        v = y2 - ref_H / 2.0
    else:
        v = y1 + ref_H / 2.0

    return u, v


def _select_z(z_pc, z_da, scale_state):
    '''Performs Depth Anything EMA scaling and returns the best depth estimate.'''
    if z_pc is not None and z_da is not None and z_da > 0:
        r = float(np.clip(z_pc / z_da, *C.DA_SCALE_CLIP))
        scale_state["ratio"] = (C.DA_SCALE_EMA_ALPHA * r +
                                (1.0 - C.DA_SCALE_EMA_ALPHA) * scale_state["ratio"])
        scale_state["calibrated"] = True

    if z_da is not None:
        if scale_state["calibrated"]:
            return z_da * scale_state["ratio"], "depth_anything_scaled"
        return z_da, "depth_anything"
    if z_pc is not None:
        return z_pc, "pointcloud"
    return None, None
