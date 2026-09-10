"""Reference model + synthetic drive for the dead-reckoning coast core (Phase 0).

Usage:
  python coast_ref.py gen  scenario.csv   # write a synthetic input log (IMU 100 Hz, KSXT 10 Hz)
  python coast_ref.py run  scenario.csv   # run the Python reference model, print shadow reports
The host C test (coast_host_test.c) reads the same CSV and must print the same reports.

CSV rows:  t_us,I,yaw,roll,pitch           (TM171 code 35, raw values)
           t_us,W,steer_deg
           t_us,G,lat,lon,alt,posq,hdgq,hdg_raw,track,v_mps,dual_roll
"""
import math, sys, random

DEG = math.pi / 180.0
A = 6378137.0
E2 = 0.00669437999014

CFG = dict(wheelbase_m=2.6, antenna_fwd_m=1.2, antenna_height_m=2.8,
           imu_yaw_sign=1.0, imu_roll_sign=1.0, dual_heading_offset_deg=999.0,
           shadow_window_s=20.0, min_speed_mps=0.5)


def wrap180(a):
    a = math.fmod(a + 180.0, 360.0)
    if a < 0: a += 360.0
    return a - 180.0


def wrap360(a):
    a = math.fmod(a, 360.0)
    if a < 0: a += 360.0
    return a


def radii(lat):
    s = math.sin(lat * DEG); d = 1 - E2 * s * s
    return A * (1 - E2) / (d * math.sqrt(d)), A / math.sqrt(d)


# ----------------------------------------------------------------------------- synthetic drive
def gen(path, seed=1):
    """Rear-axle bicycle truth. Straight 10 s at 2.5 m/s, then a R=25 m right-hand curve for 18 s while
    slowing to 1.8 m/s, on a 12 deg right-down side slope with 2 deg downhill crab from t=12 s, then
    straight again. Antenna 1.2 m ahead, 2.8 m up. KSXT heading is reported 90 deg off the vehicle
    heading (left/right antennas) to exercise the quadrant learning. IMU yaw has a 37 deg offset and
    slow drift."""
    rnd = random.Random(seed)
    lat0, lon0 = 47.30000, 11.20000
    rm, rn = radii(lat0)
    L = CFG['wheelbase_m']; a = CFG['antenna_fwd_m']; h = CFG['antenna_height_m']
    dt = 0.01
    n = e = 0.0; psi = 20.0; t = 0.0
    rows = []
    prev_ant = None
    yaw_off = 37.0
    for k in range(int(60 / dt) + 1):
        t = k * dt
        # speed profile
        if t < 10: v = 2.5
        elif t < 28: v = 2.5 - 0.7 * min(1.0, (t - 10) / 8.0)
        else: v = 1.8
        # steer: curve between 10 and 28 s
        delta = math.degrees(math.atan(L / 25.0)) if 10 <= t < 28 else 0.0
        # slope & crab from 12 s to 30 s
        roll = 12.0 if 12 <= t < 30 else 0.0
        beta = 2.0 if 12 <= t < 30 else 0.0
        psi_dot = math.degrees(v * math.tan(delta * DEG) / L)
        psi = wrap360(psi + psi_dot * dt)
        chi = (psi + beta) * DEG
        n += v * math.cos(chi) * dt; e += v * math.sin(chi) * dt
        # antenna
        s, c = math.sin(psi * DEG), math.cos(psi * DEG)
        hr = h * math.sin(roll * DEG)
        ae = e + a * s + hr * c; an = n + a * c - hr * s
        t_us = int(round(t * 1e6))
        yaw = wrap360(psi - yaw_off + 0.005 * t + rnd.gauss(0, 0.02))
        rows.append((t_us, 'I', f"{yaw:.4f}", f"{roll + rnd.gauss(0, 0.05):.4f}", f"{rnd.gauss(0, 0.05):.4f}"))
        if k % 4 == 0:
            rows.append((t_us, 'W', f"{delta + rnd.gauss(0, 0.1):.3f}"))
        if k % 10 == 0:
            if prev_ant is not None:
                de, dn = ae - prev_ant[0], an - prev_ant[1]
                track = wrap360(math.degrees(math.atan2(de, dn)))
                vant = math.hypot(de, dn) / 0.1
            else:
                track, vant = psi, v
            prev_ant = (ae, an)
            lat = lat0 + (an + rnd.gauss(0, 0.01)) / rm / DEG
            lon = lon0 + (ae + rnd.gauss(0, 0.01)) / (rn * math.cos(lat0 * DEG)) / DEG
            hdg_raw = wrap360(psi - 90.0 + rnd.gauss(0, 0.1))
            rows.append((t_us, 'G', f"{lat:.9f}", f"{lon:.9f}", "600.0", "3", "3",
                         f"{hdg_raw:.2f}", f"{track:.2f}", f"{vant:.3f}", f"{roll:.2f}"))
    with open(path, 'w') as f:
        for r in rows:
            f.write(','.join(str(x) for x in r) + '\n')
    print(f"wrote {len(rows)} rows to {path}")


# ----------------------------------------------------------------------------- reference model
class Coast:
    def __init__(self, cfg):
        self.cfg = dict(cfg)
        self.delta = 0.0; self.delta_valid = False; self.delta_t = 0
        self.votes = [0, 0, 0, 0]; self.quad = 0; self.quad_valid = False
        if cfg['dual_heading_offset_deg'] != 999.0:
            self.quad = int(round(cfg['dual_heading_offset_deg'])); self.quad_valid = True
        self.yaw = self.roll = self.pitch = 0.0; self.yaw_rate = 0.0; self.imu_t = 0; self.imu_valid = False
        self.last = None
        self.active = False
        self.reports = []

    def veh(self, hdg_raw):
        return wrap360(hdg_raw + self.quad)

    def imu(self, t_us, yaw, roll, pitch):
        c = self.cfg
        if self.imu_valid:
            dt = (t_us - self.imu_t) * 1e-6
            if 0 < dt < 0.5:
                rate = c['imu_yaw_sign'] * wrap180(yaw - self.yaw) / dt
                alpha = dt / (dt + 0.05)
                self.yaw_rate += (rate - self.yaw_rate) * alpha
                if self.active:
                    self.psi = wrap360(c['imu_yaw_sign'] * yaw + self.delta_frozen)
                    chi = (self.psi + self.beta) * DEG
                    self.n += self.v * math.cos(chi) * dt
                    self.e += self.v * math.sin(chi) * dt
                    self.dist += self.v * dt
            elif dt >= 0.5 and self.active:
                self.active = False
        self.yaw, self.roll, self.pitch = yaw, roll, pitch
        self.imu_t = t_us; self.imu_valid = True

    def predict(self):
        c = self.cfg
        s, co = math.sin(self.psi * DEG), math.cos(self.psi * DEG)
        hr = c['antenna_height_m'] * math.sin(c['imu_roll_sign'] * self.roll * DEG)
        e = self.e + c['antenna_fwd_m'] * s + hr * co
        n = self.n + c['antenna_fwd_m'] * co - hr * s
        return (self.lat0 + n / self.rm / DEG, self.lon0 + e / (self.rn * math.cos(self.lat0 * DEG)) / DEG)

    def gnss(self, t_ms, lat, lon, alt, posq, hdgq, hdg_raw, track, v, dual_roll):
        c = self.cfg
        if posq != 3:
            if self.active and self.last is not None and (t_ms - self.last[0]) > 1000:
                self.active = False
            return
        # quadrant
        if c['dual_heading_offset_deg'] == 999.0 and hdgq == 3 and v >= 1.0:
            q = int(round(wrap180(track - hdg_raw) / 90.0)) % 4
            self.votes[q] += 1
            best = max(range(4), key=lambda i: (self.votes[i], -i))
            total = sum(self.votes)
            if self.votes[best] >= 20 and self.votes[best] * 10 >= total * 8:
                off = best * 90
                if not self.quad_valid or off != self.quad: self.delta_valid = False
                self.quad = off; self.quad_valid = True
        # delta
        if self.quad_valid and self.imu_valid and hdgq == 3:
            raw = wrap180(self.veh(hdg_raw) - c['imu_yaw_sign'] * self.yaw)
            if not self.delta_valid:
                self.delta = raw; self.delta_valid = True
            else:
                dt = (t_ms - self.delta_t) * 1e-3
                if dt <= 0 or dt > 2.0: dt = 0.1
                alpha = dt / (dt + 2.0)
                self.delta = wrap180(self.delta + wrap180(raw - self.delta) * alpha)
            self.delta_t = t_ms
        if self.active:
            plat, plon = self.predict()
            dn = (lat - plat) * DEG * self.rm
            de = (lon - plon) * DEG * self.rn * math.cos(self.lat0 * DEG)
            s, co = math.sin(self.psi * DEG), math.cos(self.psi * DEG)
            self.along = de * s + dn * co; self.cross = de * co - dn * s
            self.max_along = max(self.max_along, abs(self.along)); self.max_cross = max(self.max_cross, abs(self.cross))
            self.count += 1
            if (t_ms - self.t_start) * 1e-3 >= c['shadow_window_s']:
                self.reports.append(dict(duration_ms=t_ms - self.t_start, dist=self.dist, along=self.along,
                                         cross=self.cross, max_along=self.max_along, max_cross=self.max_cross,
                                         delta=self.delta_frozen, offset=self.quad, beta=self.beta,
                                         v_start=self.v, v_end=v, fixes=self.count))
                self.active = False
        self.last = (t_ms,)
        if (not self.active and self.delta_valid and self.quad_valid and self.imu_valid and v >= c['min_speed_mps']):
            self.delta_frozen = self.delta
            self.psi = wrap360(c['imu_yaw_sign'] * self.yaw + self.delta_frozen)
            self.v = v
            beta_total = wrap180(track - self.veh(hdg_raw))
            beta_kin = math.degrees(math.atan2(c['antenna_fwd_m'] * self.yaw_rate * DEG, v))
            self.beta = wrap180(beta_total - beta_kin)
            self.lat0, self.lon0 = lat, lon
            self.rm, self.rn = radii(lat)
            s, co = math.sin(self.psi * DEG), math.cos(self.psi * DEG)
            hr = c['antenna_height_m'] * math.sin(c['imu_roll_sign'] * self.roll * DEG)
            self.e = -c['antenna_fwd_m'] * s - hr * co
            self.n = -c['antenna_fwd_m'] * co + hr * s
            self.dist = 0.0; self.t_start = t_ms
            self.along = self.cross = self.max_along = self.max_cross = 0.0; self.count = 0
            self.active = True


def run(path):
    m = Coast(CFG)
    with open(path) as f:
        for line in f:
            p = line.strip().split(',')
            if len(p) < 2: continue
            t_us = int(p[0])
            if p[1] == 'I':
                m.imu(t_us, float(p[2]), float(p[3]), float(p[4]))
            elif p[1] == 'W':
                pass
            elif p[1] == 'G':
                m.gnss(t_us // 1000, float(p[2]), float(p[3]), float(p[4]), int(p[5]), int(p[6]),
                       float(p[7]), float(p[8]), float(p[9]), float(p[10]))
    for r in m.reports:
        print("REPORT,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%.2f,%.3f,%.3f,%d" % (
            r['duration_ms'], r['dist'], r['along'], r['cross'], r['max_along'], r['max_cross'],
            r['delta'], r['offset'], r['beta'], r['v_start'], r['v_end'], r['fixes']))


if __name__ == '__main__':
    if sys.argv[1] == 'gen': gen(sys.argv[2])
    elif sys.argv[1] == 'run': run(sys.argv[2])
