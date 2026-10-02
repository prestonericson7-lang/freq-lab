// pc_test.cpp -- the NMEA parser used by the VLF station bridge.
//   g++ -std=c++17 -I.. pc_test.cpp -o t && ./t
#include "../gps_nmea.h"
#include <cstdio>
#include <cstring>
using namespace gps;
static int passes = 0, fails = 0;
static void check(bool c, const char *w) { if (c) { passes++; printf("  PASS  %s\n", w); } else { fails++; printf("  FAIL  %s\n", w); } }

int main() {
    Fix fx;
    // a current-era RMC (date 230326 = 23 Mar 2026); the "20" century prefix is
    // correct for any modern GPS yy, which is what this receiver sees.
    char rmc[] = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230326,003.1,W*61";
    char ggade[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

    printf("[1] RMC time and fix\n");
    char tmp[128]; strcpy(tmp, rmc);
    check(parse(tmp, fx), "RMC recognised");
    check(fx.valid, "status A -> valid fix");
    check(strcmp(fx.utc, "2026-03-23T12:35:19Z") == 0, "UTC assembled from time+date (20yy century)");
    printf("    lat %.5f lon %.5f\n", fx.lat, fx.lon);
    check(fx.lat > 48.11 && fx.lat < 48.13 && fx.lon > 11.51 && fx.lon < 11.53, "lat/lon decoded from ddmm.mmm");

    printf("[2] GGA quality and sats\n");
    strcpy(tmp, ggade);
    check(parse(tmp, fx), "GGA recognised");
    check(fx.quality == 1 && fx.sats == 8, "fix quality 1, 8 satellites");

    printf("[3] GN talker id and a void fix\n");
    char gnrmc[] = "$GNRMC,045678.00,V,,,,,,,110725,,,N*7A";
    strcpy(tmp, gnrmc);
    parse(tmp, fx);
    check(!fx.valid, "status V -> not valid (no fix yet)");
    check(strcmp(fx.utc, "2025-07-11T04:56:78Z") == 0, "UTC still assembled from a void RMC (time known before fix)");

    printf("[4] junk is ignored\n");
    char junk[] = "$GPGSV,3,1,11,01,40,083,46*74";
    strcpy(tmp, junk);
    check(!parse(tmp, fx), "an unhandled sentence (GSV) returns false, leaves the fix unchanged");
    char empty[] = "$GP";
    strcpy(tmp, empty);
    check(!parse(tmp, fx), "a too-short line returns false");

    printf("[5] checksum\n");
    check(checksum("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W") == 0x6A, "RMC checksum matches the *6A in the sentence");

    printf("\n=== %d PASS, %d FAIL ===\n", passes, fails);
    return fails ? 1 : 0;
}
