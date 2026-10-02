// gps_nmea.h -- minimal NMEA parser (RMC + GGA) for UTC time, fix status and
// position, free of Arduino so a PC tests it (test/pc_test.cpp). Same parsing
// the freq-lab .ino builds have used (elf_logger etc.): $..RMC for time/date,
// $..GGA for fix quality and satellites. 9600 baud NMEA, any u-blox NEO-6/7/8.
#pragma once
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

namespace gps {

struct Fix {
    bool   valid = false;              // RMC status 'A'
    int    quality = 0;                // GGA fix quality (0 none, 1 GPS, 2 DGPS)
    int    sats = 0;
    char   utc[24] = "";               // "20yy-mm-ddThh:mm:ssZ"
    double lat = 0, lon = 0;
};

// split a comma-separated sentence body into fields; returns the count
inline int split(char *s, char **f, int maxf) {
    int n = 0;
    char *p = s;
    while (n < maxf && p) {
        f[n++] = p;
        p = strchr(p, ',');
        if (p) *p++ = 0;
    }
    return n;
}

// feed one complete NUL-terminated sentence (with or without the leading '$' and
// without the CR/LF). Updates `fix`. Returns true if it was a recognised sentence.
inline bool parse(char *line, Fix &fix) {
    char *s = (line[0] == '$') ? line + 1 : line;
    if (strlen(s) < 6) return false;
    // talker id is 2 chars (GP/GN/GL...), sentence type is the next 3
    const char *type = s + 2;
    char buf[100];
    strncpy(buf, s, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    char *f[20];
    int n = split(buf, f, 20);

    if (!strncmp(type, "RMC", 3) && n >= 10) {
        fix.valid = (f[2][0] == 'A');
        if (strlen(f[1]) >= 6 && strlen(f[9]) >= 6) {
            // f[1] = hhmmss(.sss), f[9] = ddmmyy
            snprintf(fix.utc, sizeof(fix.utc), "20%c%c-%c%c-%c%cT%c%c:%c%c:%c%cZ",
                     f[9][4], f[9][5], f[9][2], f[9][3], f[9][0], f[9][1],
                     f[1][0], f[1][1], f[1][2], f[1][3], f[1][4], f[1][5]);
        }
        if (fix.valid && strlen(f[3]) > 3 && strlen(f[5]) > 4) {
            double la = atof(f[3]), lo = atof(f[5]);
            fix.lat = (int)(la / 100) + (la - 100 * (int)(la / 100)) / 60.0;
            if (f[4][0] == 'S') fix.lat = -fix.lat;
            fix.lon = (int)(lo / 100) + (lo - 100 * (int)(lo / 100)) / 60.0;
            if (f[6][0] == 'W') fix.lon = -fix.lon;
        }
        return true;
    }
    if (!strncmp(type, "GGA", 3) && n >= 8) {
        fix.quality = atoi(f[6]);
        fix.sats = atoi(f[7]);
        return true;
    }
    return false;
}

// checksum of an NMEA sentence body (between '$' and '*'); for optional verification
inline unsigned char checksum(const char *body) {
    unsigned char c = 0;
    for (const char *p = (body[0] == '$') ? body + 1 : body; *p && *p != '*'; ++p) c ^= (unsigned char)*p;
    return c;
}

}  // namespace gps
