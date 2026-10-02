"""render.py -- small z-buffer renderer (orthographic and oblique) for checking parts."""
import numpy as np
from PIL import Image, ImageDraw


def _project(v, view):
    """Return (u, w, depth) for vertices; smaller depth = closer to the viewer."""
    if view == "top":        # looking down -z
        return v[:, 0], v[:, 1], -v[:, 2]
    if view == "front":      # looking along +y
        return v[:, 0], v[:, 2], v[:, 1]
    if view == "side":       # looking along -x
        return v[:, 1], v[:, 2], -v[:, 0]
    if view == "iso":        # oblique 3/4 view from (+x, -y, +z)
        az, el = np.radians(-35.0), np.radians(30.0)
        ca, sa = np.cos(az), np.sin(az)
        x = v[:, 0] * ca - v[:, 1] * sa
        y = v[:, 0] * sa + v[:, 1] * ca
        z = v[:, 2]
        ce, se = np.cos(el), np.sin(el)
        return x, z * ce + y * se, y * ce - z * se      # viewer above, on the -y side
    raise ValueError(view)


def render(meshes, view="iso", size=900, margin=0.06, colors=None, bg=(255, 255, 255)):
    """meshes: list of (vertices Nx3, faces Mx3). Returns a PIL image."""
    allv = np.vstack([m[0] for m in meshes])
    u, w, _ = _project(allv, view)
    u0, u1, w0, w1 = u.min(), u.max(), w.min(), w.max()
    span = max(u1 - u0, w1 - w0) * (1 + 2 * margin)
    cu, cw = (u0 + u1) / 2, (w0 + w1) / 2
    s = size / span
    depth = np.full((size, size), np.inf)
    img = np.zeros((size, size, 3)) + np.array(bg, float)
    light = np.array([0.35, -0.45, 0.82])
    light /= np.linalg.norm(light)
    for k, (V, F) in enumerate(meshes):
        col = np.array(colors[k] if colors else (90, 130, 200), float)
        pu, pw, pd = _project(V, view)
        X = (pu - cu) * s + size / 2
        Y = size / 2 - (pw - cw) * s
        tri = V[F]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        lam = 0.25 + 0.75 * np.abs(n @ light)
        for t in range(len(F)):
            i0, i1, i2 = F[t]
            xs = np.array([X[i0], X[i1], X[i2]])
            ys = np.array([Y[i0], Y[i1], Y[i2]])
            ds = np.array([pd[i0], pd[i1], pd[i2]])
            xa, xb = int(max(np.floor(xs.min()), 0)), int(min(np.ceil(xs.max()), size - 1))
            ya, yb = int(max(np.floor(ys.min()), 0)), int(min(np.ceil(ys.max()), size - 1))
            if xa > xb or ya > yb:
                continue
            gx, gy = np.meshgrid(np.arange(xa, xb + 1) + 0.5, np.arange(ya, yb + 1) + 0.5)
            d = (xs[1] - xs[0]) * (ys[2] - ys[0]) - (xs[2] - xs[0]) * (ys[1] - ys[0])
            if abs(d) < 1e-9:
                continue
            a = ((xs[1] - gx) * (ys[2] - gy) - (xs[2] - gx) * (ys[1] - gy)) / d
            b = ((xs[2] - gx) * (ys[0] - gy) - (xs[0] - gx) * (ys[2] - gy)) / d
            c = 1 - a - b
            inside = (a >= -1e-6) & (b >= -1e-6) & (c >= -1e-6)
            if not inside.any():
                continue
            dep = a * ds[0] + b * ds[1] + c * ds[2]
            sub = depth[ya:yb + 1, xa:xb + 1]
            upd = inside & (dep < sub)
            sub[upd] = dep[upd]
            img[ya:yb + 1, xa:xb + 1][upd] = col * lam[t]
    return Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)), (cu, cw, s, size)


def label(im, text, xy=(10, 10), color=(20, 20, 20)):
    ImageDraw.Draw(im).text(xy, text, fill=color)
    return im
