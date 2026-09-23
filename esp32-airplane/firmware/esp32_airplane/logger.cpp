// logger.cpp — CSV formatter for the real-time logger (checklist I3).
// Pure C (stdio only): host-unit-tested, no Arduino dependency.
#include "logger.h"
#include <stdio.h>

const char* logger_csv_header(void) {
    return
        "t_ms,mode,armed,rc_ok,phase,fs_evt,env,wind,"
        "rc_p,rc_r,rc_t,rc_y,raw_p,raw_r,raw_t,raw_y,"
        "ax,ay,az,gx,gy,gz,baro_pa,baro_c,"
        "roll,pitch,yaw,fix,sv,hdop,gps_v,gps_alt,gps_trk,vest,"
        "phi_cmd,l_us,r_us,thr_out,overruns\n";
}

size_t logger_format(char* buf, size_t cap, const LogSample& s) {
    if (!buf || cap < 8) return 0;

    int n = snprintf(buf, cap,
        // t, mode, armed, rc_ok, phase, fs_evt, env, wind
        "%lu,%u,%u,%u,%u,%u,%u,%u,"
        // rc_p rc_r rc_t rc_y | raw_p raw_r raw_t raw_y
        "%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,"
        // accel g | gyro rad/s
        "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,"
        // baro
        "%.1f,%.2f,"
        // attitude deg
        "%.2f,%.2f,%.2f,"
        // fix sv hdop gps_v gps_alt gps_trk vest
        "%u,%u,%.2f,%.2f,%.1f,%.1f,%.2f,"
        // phi_cmd l_us r_us thr_out overruns
        "%.4f,%u,%u,%.3f,%lu\n",
        (unsigned long)s.t_ms,
        (unsigned)s.mode, (unsigned)s.armed, (unsigned)s.rc_ok,
        (unsigned)s.phase, (unsigned)s.fs_evt, (unsigned)s.env,
        (unsigned)s.wind,
        (double)s.rc_p, (double)s.rc_r, (double)s.rc_t, (double)s.rc_y,
        (unsigned)s.raw_p, (unsigned)s.raw_r,
        (unsigned)s.raw_t, (unsigned)s.raw_y,
        (double)s.ax, (double)s.ay, (double)s.az,
        (double)s.gx, (double)s.gy, (double)s.gz,
        (double)s.baro_pa, (double)s.baro_c,
        (double)s.roll, (double)s.pitch, (double)s.yaw,
        (unsigned)s.fix, (unsigned)s.sv,
        (double)s.hdop, (double)s.gps_v, (double)s.gps_alt,
        (double)s.gps_trk, (double)s.vest,
        (double)s.phi_cmd,
        (unsigned)s.l_us, (unsigned)s.r_us,
        (double)s.thr_out,
        (unsigned long)s.overruns);

    if (n <= 0 || (size_t)n >= cap) { buf[0] = 0; return 0; }
    return (size_t)n;
}
