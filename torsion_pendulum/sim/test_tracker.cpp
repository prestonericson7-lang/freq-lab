// test_tracker.cpp -- run the firmware's tracker on frames.bin (from make_frames.py)
//   g++ -O2 -I../firmware/torsion_cam test_tracker.cpp ../firmware/torsion_cam/tracker.cpp -o test_tracker
//   ./test_tracker frames.bin 640 480 > tracked.csv
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <chrono>
#include "tracker.h"

int main(int argc, char** argv) {
    if (argc < 4) { fprintf(stderr, "usage: %s frames.bin w h\n", argv[0]); return 1; }
    int w = atoi(argv[2]), h = atoi(argv[3]);
    FILE* f = fopen(argv[1], "rb");
    if (!f) { perror("frames"); return 1; }
    std::vector<uint8_t> buf((size_t)w * h * 2);
    Tracker tr;
    int k = 0, yoff = -1;
    double us_total = 0;
    printf("frame,rotor_ok,ref_ok,angle,rotor_angle,ref_angle,cx,cy,area,thr,acq\n");
    while (fread(buf.data(), 1, buf.size(), f) == buf.size()) {
        if (yoff < 0) yoff = detectYOffset(buf.data(), w, h);
        Lum lum = {buf.data(), w, h, 2, yoff};
        auto t0 = std::chrono::steady_clock::now();
        TrackOut o = tr.update(lum);
        auto t1 = std::chrono::steady_clock::now();
        us_total += std::chrono::duration<double, std::micro>(t1 - t0).count();
        printf("%d,%d,%d,%.9f,%.9f,%.9f,%.3f,%.3f,%.1f,%d,%d\n", k, o.rotor_ok, o.ref_ok, o.angle,
               o.rotor_angle, o.ref_angle, o.cx, o.cy, o.area, o.thr, tr.acquisitions);
        k++;
    }
    fprintf(stderr, "frames %d  yoff %d  mean %.0f us/frame (PC)  nref %d  rotor half_len %.1f half_w %.1f  ref box %.1f\n",
            k, yoff, us_total / (k ? k : 1), tr.nref, tr.half_len, tr.half_w, tr.qh);
    return 0;
}
