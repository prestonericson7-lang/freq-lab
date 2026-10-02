// tracker.cpp -- see tracker.h
#include "tracker.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static const double PI_ = 3.14159265358979323846;

int otsu256(const uint32_t* hist) {
    double total = 0, sum = 0;
    for (int i = 0; i < 256; i++) { total += hist[i]; sum += (double)i * hist[i]; }
    if (total <= 0) return 128;
    double wB = 0, sumB = 0, best = -1;
    int thr = 128;
    for (int t = 0; t < 256; t++) {
        wB += hist[t];
        if (wB <= 0) continue;
        double wF = total - wB;
        if (wF <= 0) break;
        sumB += (double)t * hist[t];
        double mB = sumB / wB, mF = (sum - sumB) / wF;
        double between = wB * wF * (mB - mF) * (mB - mF);
        if (between > best) { best = between; thr = t; }
    }
    return thr;
}

int detectYOffset(const uint8_t* buf, int w, int h) {
    double s[2] = {0, 0}, s2[2] = {0, 0};
    int n = 0;
    long npix = (long)w * h;
    long stride = npix / 4000 + 1;
    for (long p = 0; p < npix; p += stride) {
        for (int k = 0; k < 2; k++) { double v = buf[p * 2 + k]; s[k] += v; s2[k] += v * v; }
        n++;
    }
    double v0 = s2[0] / n - (s[0] / n) * (s[0] / n);
    double v1 = s2[1] / n - (s[1] / n) * (s[1] / n);
    return v1 > v0 ? 1 : 0;
}

Tracker::Tracker()
    : rx(0), ry(0), rr(0), qh(0), nref(0), rotor_area0(0), acquisitions(0),
      axis_angle(0), half_len(0), half_w(0), px(0), py(0), pr(1e9),
      rot_unwrap(0), ref_unwrap(0), rot_last(0), ref_last(0), have_rot(false), have_ref(false) {
    qx[0] = qx[1] = qy[0] = qy[1] = 0;
}

// ---------------------------------------------------------------- weighted moments
// Pixels inside the region contribute weight 1 when clearly darker than the
// Otsu threshold, 0 when clearly lighter, and a linear ramp across +/-band.
struct Acc {
    double w, sx, sy, sxx, syy, sxy;
};

static void finish(const Acc& a, double ox, double oy, int thr, Blob& b) {
    b.ok = a.w > 4.0;
    b.thr = thr;
    b.area = a.w;
    if (!b.ok) return;
    double mx = a.sx / a.w, my = a.sy / a.w;
    double m20 = a.sxx / a.w - mx * mx, m02 = a.syy / a.w - my * my, m11 = a.sxy / a.w - mx * my;
    b.cx = ox + mx;
    b.cy = oy + my;
    b.angle = 0.5 * atan2(2.0 * m11, m20 - m02);
    double c = 0.5 * (m20 + m02), d = sqrt(0.25 * (m20 - m02) * (m20 - m02) + m11 * m11);
    b.major = sqrt(fmax(c + d, 0.0));
    b.minor = sqrt(fmax(c - d, 0.0));
}

// the stadium-shaped rotor region: within half_w of the segment +/-half_len along axis_angle
static inline bool inStadium(double dx, double dy, double ca, double sa, double hl, double hw) {
    double u = dx * ca + dy * sa, v = -dx * sa + dy * ca;
    double au = fabs(u);
    if (au <= hl) return fabs(v) <= hw;
    double du = au - hl;
    return du * du + v * v <= hw * hw;
}

Blob Tracker::measureRotor(const Lum& f, double cx, double cy) const {
    const double pm2 = (pr - 3.0) * (pr - 3.0);
    // the rotor region is a stadium oriented along the last measured angle
    Blob b;
    memset(&b, 0, sizeof(b));
    double ca = cos(axis_angle), sa = sin(axis_angle);
    double hl = half_len, hw = half_w;
    double ext = hl + hw;
    int x0 = (int)fmax(0, floor(cx - ext)), x1 = (int)fmin(f.w - 1, ceil(cx + ext));
    int y0 = (int)fmax(0, floor(cy - ext)), y1 = (int)fmin(f.h - 1, ceil(cy + ext));
    uint32_t hist[256];
    memset(hist, 0, sizeof(hist));
    for (int y = y0; y <= y1; y++) {
        double dy = y - cy, ey = y - py;
        for (int x = x0; x <= x1; x++) {
            if (!inStadium(x - cx, dy, ca, sa, hl, hw)) continue;
            double ex = x - px;
            if (ex * ex + ey * ey > pm2) continue;
            hist[f.at(x, y)]++;
        }
    }
    int thr = otsu256(hist);
    // band from the two class means
    double nd = 0, sd = 0, nl = 0, sl = 0;
    for (int i = 0; i < 256; i++) {
        if (i <= thr) { nd += hist[i]; sd += (double)i * hist[i]; } else { nl += hist[i]; sl += (double)i * hist[i]; }
    }
    double dark = nd > 0 ? sd / nd : thr - 20, light = nl > 0 ? sl / nl : thr + 20;
    double band = fmax(3.0, (light - dark) / 6.0);
    double lo = thr - band, hi = thr + band;
    Acc a = {0, 0, 0, 0, 0, 0};
    for (int y = y0; y <= y1; y++) {
        double dy = y - cy, ey = y - py;
        for (int x = x0; x <= x1; x++) {
            double dx = x - cx;
            if (!inStadium(dx, dy, ca, sa, hl, hw)) continue;
            double ex = x - px;
            if (ex * ex + ey * ey > pm2) continue;
            int L = f.at(x, y);
            double w;
            if (L <= lo) w = 1.0;
            else if (L >= hi) continue;
            else w = (hi - L) / (hi - lo);
            a.w += w; a.sx += w * dx; a.sy += w * dy;
            a.sxx += w * dx * dx; a.syy += w * dy * dy; a.sxy += w * dx * dy;
        }
    }
    finish(a, cx, cy, thr, b);
    return b;
}

Blob Tracker::measureSquare(const Lum& f, double cx, double cy, double h) const {
    const double pm2 = (pr - 3.0) * (pr - 3.0);
    Blob b;
    memset(&b, 0, sizeof(b));
    int x0 = (int)fmax(0, floor(cx - h)), x1 = (int)fmin(f.w - 1, ceil(cx + h));
    int y0 = (int)fmax(0, floor(cy - h)), y1 = (int)fmin(f.h - 1, ceil(cy + h));
    uint32_t hist[256];
    memset(hist, 0, sizeof(hist));
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            double ex = x - px, ey = y - py;
            if (ex * ex + ey * ey > pm2) continue;
            hist[f.at(x, y)]++;
        }
    int thr = otsu256(hist);
    double nd = 0, sd = 0, nl = 0, sl = 0;
    for (int i = 0; i < 256; i++) {
        if (i <= thr) { nd += hist[i]; sd += (double)i * hist[i]; } else { nl += hist[i]; sl += (double)i * hist[i]; }
    }
    double dark = nd > 0 ? sd / nd : thr - 20, light = nl > 0 ? sl / nl : thr + 20;
    double band = fmax(3.0, (light - dark) / 6.0);
    double lo = thr - band, hi = thr + band;
    Acc a = {0, 0, 0, 0, 0, 0};
    for (int y = y0; y <= y1; y++) {
        double dy = y - cy, ey = y - py;
        for (int x = x0; x <= x1; x++) {
            double ex = x - px;
            if (ex * ex + ey * ey > pm2) continue;
            int L = f.at(x, y);
            double w;
            if (L <= lo) w = 1.0;
            else if (L >= hi) continue;
            else w = (hi - L) / (hi - lo);
            double dx = x - cx;
            a.w += w; a.sx += w * dx; a.sy += w * dy;
            a.sxx += w * dx * dx; a.syy += w * dy * dy; a.sxy += w * dx * dy;
        }
    }
    finish(a, cx, cy, thr, b);
    return b;
}

// ---------------------------------------------------------------- acquisition
struct Comp {
    long n;
    double sx, sy, sxx, syy, sxy;
    bool border;
};

bool Tracker::acquire(const Lum& f) {
    const int S = 4;
    int dw = f.w / S, dh = f.h / S;
    int nc = dw * dh;
    uint8_t* d = (uint8_t*)malloc(nc);
    int32_t* lab = (int32_t*)malloc(nc * sizeof(int32_t));
    int32_t* queue = (int32_t*)malloc(nc * sizeof(int32_t));
    const int MAXC = 512;
    Comp* comps = (Comp*)malloc(MAXC * sizeof(Comp));
    bool ok = false;
    if (!d || !lab || !queue || !comps) goto done;
    {
        uint32_t hist[256];
        memset(hist, 0, sizeof(hist));
        for (int j = 0; j < dh; j++)
            for (int i = 0; i < dw; i++) {
                int s = 0;
                for (int yy = 0; yy < S; yy++)
                    for (int xx = 0; xx < S; xx++) s += f.at(i * S + xx, j * S + yy);
                d[j * dw + i] = (uint8_t)(s / (S * S));
                hist[d[j * dw + i]]++;
            }
        int thr = otsu256(hist);
        for (int k = 0; k < nc; k++) lab[k] = -1;
        int ncomp = 0;
        for (int start = 0; start < nc; start++) {
            if (lab[start] != -1 || d[start] > thr) continue;
            if (ncomp >= MAXC) break;
            Comp c = {0, 0, 0, 0, 0, 0, false};
            int qh_ = 0, qt = 0;
            queue[qt++] = start;
            lab[start] = ncomp;
            while (qh_ < qt) {
                int k = queue[qh_++];
                int i = k % dw, j = k / dw;
                c.n++; c.sx += i; c.sy += j; c.sxx += (double)i * i; c.syy += (double)j * j; c.sxy += (double)i * j;
                if (i == 0 || j == 0 || i == dw - 1 || j == dh - 1) c.border = true;
                for (int dj = -1; dj <= 1; dj++)
                    for (int di = -1; di <= 1; di++) {
                        int ii = i + di, jj = j + dj;
                        if (ii < 0 || jj < 0 || ii >= dw || jj >= dh) continue;
                        int kk = jj * dw + ii;
                        if (lab[kk] != -1 || d[kk] > thr) continue;
                        lab[kk] = ncomp;
                        queue[qt++] = kk;
                    }
            }
            comps[ncomp++] = c;
        }
        // the lit paper: the largest bright component, holes filled (its enclosed dark blobs)
        {
            for (int k = 0; k < nc; k++) queue[k] = -1;          // reused as the bright-label map
            int32_t* st = (int32_t*)malloc(nc * sizeof(int32_t)); // DFS stack
            long bestB = 0; double bsx = 0, bsy = 0; bool bborder = false;
            for (int start = 0; st && start < nc; start++) {
                if (queue[start] != -1 || d[start] <= thr) continue;
                long n = 0; double sx = 0, sy = 0; bool border = false;
                int sp = 0;
                st[sp++] = start; queue[start] = start;
                while (sp > 0) {
                    int k = st[--sp];
                    int i = k % dw, j = k / dw;
                    n++; sx += i; sy += j;
                    if (i == 0 || j == 0 || i == dw - 1 || j == dh - 1) border = true;
                    const int di4[4] = {1, -1, 0, 0}, dj4[4] = {0, 0, 1, -1};
                    for (int m = 0; m < 4; m++) {
                        int ii = i + di4[m], jj = j + dj4[m];
                        if (ii < 0 || jj < 0 || ii >= dw || jj >= dh) continue;
                        int kk = jj * dw + ii;
                        if (queue[kk] != -1 || d[kk] <= thr) continue;
                        queue[kk] = start;
                        st[sp++] = kk;
                    }
                }
                if (n > bestB) { bestB = n; bsx = sx; bsy = sy; bborder = border; }
            }
            free(st);
            // add the dark components that do not touch the border (the holes in the paper)
            double fsx = bsx, fsy = bsy; long fn = bestB;
            for (int k = 0; k < ncomp; k++) if (!comps[k].border) { fn += comps[k].n; fsx += comps[k].sx; fsy += comps[k].sy; }
            if (bestB > 0 && !bborder) {
                px = (fsx / fn + 0.5) * S - 0.5;
                py = (fsy / fn + 0.5) * S - 0.5;
                pr = sqrt((double)fn / PI_) * S;
            } else {
                px = f.w / 2.0; py = f.h / 2.0; pr = 1e9;            // paper fills the frame: no mask
            }
        }
        // rotor: the largest elongated component away from the border
        int best = -1;
        double bestN = 0, bmaj = 0, bmin = 0, bang = 0, bcx = 0, bcy = 0;
        for (int k = 0; k < ncomp; k++) {
            Comp& c = comps[k];
            if (c.border || c.n < 40) continue;
            double mx = c.sx / c.n, my = c.sy / c.n;
            double m20 = c.sxx / c.n - mx * mx, m02 = c.syy / c.n - my * my, m11 = c.sxy / c.n - mx * my;
            double cc = 0.5 * (m20 + m02), dd = sqrt(0.25 * (m20 - m02) * (m20 - m02) + m11 * m11);
            double maj = sqrt(fmax(cc + dd, 1e-9)), mnr = sqrt(fmax(cc - dd, 1e-9));
            if (maj / mnr < 2.0) continue;
            if (c.n > bestN) {
                bestN = c.n; best = k; bmaj = maj; bmin = mnr;
                bang = 0.5 * atan2(2 * m11, m20 - m02); bcx = mx; bcy = my;
            }
        }
        if (best < 0) goto done;
        rx = (bcx + 0.5) * S - 0.5;
        ry = (bcy + 0.5) * S - 0.5;
        axis_angle = bang;
        // a dumbbell's disc centres sit at ~1 sigma along the axis; a disc of radius R has sigma R/2 across
        half_len = 1.0 * bmaj * S;
        half_w = 2.0 * bmin * S * 1.2 + 5.0;
        rr = half_len + half_w;
        // reference squares: the two largest compact components clear of the rotor
        int r1 = -1, r2 = -1;
        double n1 = 0, n2 = 0;
        double ca = cos(bang), sa = sin(bang);
        for (int k = 0; k < ncomp; k++) {
            if (k == best) continue;
            Comp& c = comps[k];
            if (c.border || c.n < 3 || c.n > 0.35 * bestN) continue;
            double mx = c.sx / c.n, my = c.sy / c.n;
            double m20 = c.sxx / c.n - mx * mx, m02 = c.syy / c.n - my * my, m11 = c.sxy / c.n - mx * my;
            double cc = 0.5 * (m20 + m02), dd = sqrt(0.25 * (m20 - m02) * (m20 - m02) + m11 * m11);
            double maj = sqrt(fmax(cc + dd, 1e-9)), mnr = sqrt(fmax(cc - dd, 1e-9));
            if (maj / mnr > 2.0) continue;
            double px = (mx + 0.5) * S - 0.5 - rx, py = (my + 0.5) * S - 0.5 - ry;
            // the camera sits 26 mm off the axis, so the rotor (nearer) appears shifted ~3 mm toward -x
            // relative to the paper: keep the exclusion margin small or the -x square gets rejected
            if (inStadium(px, py, ca, sa, half_len, half_w + 1.0 * S)) continue;
            if (c.n > n1) { n2 = n1; r2 = r1; n1 = c.n; r1 = k; }
            else if (c.n > n2) { n2 = c.n; r2 = k; }
        }
        nref = 0;
        if (r1 >= 0 && r2 >= 0) {
            double side = sqrt(n1) * S;
            qh = 0.5 * side + 0.35 * side + 4.0;            // square half-size plus a margin
            int ks[2] = {r1, r2};
            // order the pair left-to-right so the reference angle is continuous
            double x0 = comps[r1].sx / comps[r1].n, x1 = comps[r2].sx / comps[r2].n;
            if (x1 < x0) { ks[0] = r2; ks[1] = r1; }
            for (int m = 0; m < 2; m++) {
                qx[m] = (comps[ks[m]].sx / comps[ks[m]].n + 0.5) * S - 0.5;
                qy[m] = (comps[ks[m]].sy / comps[ks[m]].n + 0.5) * S - 0.5;
            }
            nref = 2;
        }
        Blob b = measureRotor(f, rx, ry);
        if (!b.ok) goto done;
        rotor_area0 = b.area;
        acquisitions++;
        ok = true;
    }
done:
    free(d);
    free(lab);
    free(queue);
    free(comps);
    return ok;
}

// ---------------------------------------------------------------- per frame
TrackOut Tracker::update(const Lum& f) {
    TrackOut o;
    memset(&o, 0, sizeof(o));
    if (rotor_area0 <= 0 && !acquire(f)) return o;
    Blob b = measureRotor(f, rx, ry);
    if (!b.ok || b.area < 0.5 * rotor_area0 || b.area > 1.8 * rotor_area0) {
        if (!acquire(f)) return o;
        b = measureRotor(f, rx, ry);
        if (!b.ok) return o;
    }
    // unwrap the rotor angle (defined mod pi)
    if (!have_rot) { rot_unwrap = b.angle; rot_last = b.angle; have_rot = true; }
    double d = b.angle - rot_last;
    while (d > PI_ / 2) d -= PI_;
    while (d <= -PI_ / 2) d += PI_;
    rot_unwrap += d;
    rot_last = b.angle;
    // keep the region on the rotor: follow its axis and (slowly) its centre
    axis_angle = b.angle;
    rx += 0.2 * (b.cx - rx);
    ry += 0.2 * (b.cy - ry);
    o.rotor_ok = true;
    o.rotor_angle = rot_unwrap;
    o.cx = b.cx; o.cy = b.cy; o.area = b.area; o.thr = b.thr;
    o.angle = rot_unwrap;
    if (nref == 2) {
        Blob q0 = measureSquare(f, qx[0], qy[0], qh), q1 = measureSquare(f, qx[1], qy[1], qh);
        if (q0.ok && q1.ok) {
            double a = atan2(q1.cy - q0.cy, q1.cx - q0.cx);
            if (!have_ref) { ref_unwrap = a; ref_last = a; have_ref = true; }
            double e = a - ref_last;
            while (e > PI_) e -= 2 * PI_;
            while (e <= -PI_) e += 2 * PI_;
            ref_unwrap += e;
            ref_last = a;
            qx[0] += 0.2 * (q0.cx - qx[0]); qy[0] += 0.2 * (q0.cy - qy[0]);
            qx[1] += 0.2 * (q1.cx - qx[1]); qy[1] += 0.2 * (q1.cy - qy[1]);
            o.ref_ok = true;
            o.ref_angle = ref_unwrap;
            o.angle = rot_unwrap - ref_unwrap;
        }
    }
    return o;
}
