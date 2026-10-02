// tracker.h -- finds the torsion pendulum rotor in a camera frame and measures its angle.
//
// Pure C++ (no Arduino calls) so the same code runs on the ESP32-S3 and in the
// PC test harness (sim/test_tracker.cpp).
//
// Scene: backlit paper disc, dark rotor (a dumbbell) seen from above, and two
// small dark reference squares printed on the paper. The rotor's orientation is
// taken from the second moments of its dark area, with each pixel weighted by
// how dark it is near the threshold (a soft edge), which gives sub-pixel
// precision. The reference squares give a second angle; rotor minus reference
// cancels any movement of the camera itself.
#pragma once
#include <stdint.h>

struct Lum {                       // luminance access into a camera frame buffer
    const uint8_t* buf;
    int w, h;
    int step;                      // bytes per pixel: 2 for YUV422, 1 for GRAYSCALE
    int yoff;                      // byte offset of Y within a pixel (0 or 1)
    inline int at(int x, int y) const { return buf[(y * w + x) * step + yoff]; }
};

struct Blob {
    double area;                   // weighted dark pixels
    double cx, cy;                 // centroid (px)
    double angle;                  // orientation of the major axis, rad (mod pi)
    double major, minor;           // RMS extents along/across the major axis (px)
    int thr;                       // threshold used
    bool ok;
};

struct TrackOut {
    bool rotor_ok;
    bool ref_ok;
    double rotor_angle;            // unwrapped, rad
    double ref_angle;              // unwrapped, rad
    double angle;                  // rotor - ref if ref_ok, else rotor (rad)
    double cx, cy, area;
    int thr;
};

class Tracker {
public:
    Tracker();
    // Find rotor and reference marks from scratch (slow-ish: whole frame at 1/4 res).
    bool acquire(const Lum& f);
    // Measure one frame (fast: only inside the ROIs). Re-acquires if the rotor is lost.
    TrackOut update(const Lum& f);
    // Region data for previews / logging.
    double rx, ry, rr;             // rotor ROI centre and radius (px)
    double qx[2], qy[2], qh;       // reference ROI centres and half-size (px)
    int nref;                      // 0 or 2
    double rotor_area0;            // area at acquisition (lost-track test)
    int acquisitions;
    double axis_angle;             // rotor region: stadium along this angle (rad, image frame)
    double half_len, half_w;       //   half-length of its core segment and its half-width (px)
    double px, py, pr;             // the lit paper disc: only pixels inside it are measured

private:
    double rot_unwrap, ref_unwrap; // unwrapped angles
    double rot_last, ref_last;     // last raw (mod pi / mod 2pi) values
    bool have_rot, have_ref;
    Blob measureRotor(const Lum& f, double cx, double cy) const;
    Blob measureSquare(const Lum& f, double cx, double cy, double h) const;
};

// Otsu threshold of a 256-bin histogram.
int otsu256(const uint32_t* hist);
// Byte offset of Y in a YUV422 frame: the byte lane with the larger variance.
int detectYOffset(const uint8_t* buf, int w, int h);
