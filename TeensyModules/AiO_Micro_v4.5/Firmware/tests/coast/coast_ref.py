"""Reference model + synthetic drive for the dead-reckoning coast core (Phase 0 + Phase 2).

Usage:
  python coast_ref.py gen  scenario.csv          # write a synthetic input log (IMU 100 Hz, WAS 25 Hz, KSXT 10 Hz)
  python coast_ref.py run  scenario.csv [flags]  # run the Python reference model, print shadow reports
     flags: --no-observer  --no-crab            # A/B the Phase 2 models
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
           shadow_window_s=20.0, min_speed_mps=0.5, was_sign=0.0,
           speed_observer=True, crab_model=True)

TURN_MIN_WAS = 5.0; TURN_MIN_RATE = 2.0; LEARN_MIN_V = 1.0; CRAB_MIN_ROLL = 3.0; K_LIMIT = 30.0
CRAB_MAX_ROLL_RATE = 3.0; SLIP_TAU = 1.0; RES_TAU = 5.0
LEFF_TAU = 10.0; K_TAU = 10.0; OBS_TAU = 0.25; OBS_MAX_ACC = 1.5; OBS_MAX_RATIO = 1.25
WAS_MIN_VOTES = 20; LEFF_MIN_N = 10; K_MIN_N = 50


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
    """Rear-axle bicycle truth, 70 s. Two identical R=25 m right-hand curves (10-28 s and 40-58 s) on a
    12 deg right-down side slope with 2 deg downhill crab, slowing 2.5 -> 1.8 m/s through each; straight
    and flat in between. The WAS is wired with inverted sign (negative = right) to exercise sign learning.
    Antenna 1.2 m ahead, 2.8 m up. KSXT heading reported 90 deg off the vehicle heading (left/right
    antennas). IMU yaw has a 37 deg offset and slow drift. Roll/crab ramp in over 1 s."""
    rnd = random.Random(seed)
    lat0, lon0 = 47.30000, 11.20000
    rm, rn = radii(lat0)
    L = CFG['wheelbase_m']; a = CFG['antenna_fwd_m']; h = CFG['antenna_height_m']
    dt = 0.01
    n = e = 0.0; psi = 20.0
    rows = []
    prev_roll = 0.0
    yaw_off = 37.0

    def curve_phase(t):
        for t0 in (10.0, 40.0):
            if t0 <= t < t0 + 18.0:
                return (t - t0) / 18.0, min(1.0, t - t0), min(1.0, t0 + 18.0 - t)
        return None

    for k in range(int(70 / dt) + 1):
        t = k * dt
        cp = curve_phase(t)
        if cp:
            frac, ramp_in, ramp_out = cp
            ramp = min(ramp_in, ramp_out)
            v = 2.5 - 0.7 * min(1.0, frac * 18.0 / 8.0)
            delta = math.degrees(math.atan(L / 25.0))
            roll = 12.0 * ramp
            beta = 2.0 * ramp
        else:
            # straight: accelerate back to 2.5 after a curve
            v = 2.5 if t < 10 else min(2.5, 1.8 + 0.35 * (t - (28.0 if t < 40 else 58.0)))
            delta = 0.0; roll = 0.0; beta = 0.0
        psi_dot = math.degrees(v * math.tan(delta * DEG) / L)
        psi = wrap360(psi + psi_dot * dt)
        chi = (psi + beta) * DEG
        n += v * math.cos(chi) * dt; e += v * math.sin(chi) * dt
        s, c = math.sin(psi * DEG), math.cos(psi * DEG)
        hr = h * math.sin(roll * DEG)
        ae = e + a * s + hr * c; an = n + a * c - hr * s
        # true antenna velocity (for a Doppler-like KSXT track/speed): axle velocity + rotation of the
        # (a, hr) offset + sideways swing of the antenna as the roll changes
        roll_rate = (roll - prev_roll) / dt if k > 0 else 0.0
        prev_roll = roll
        w = psi_dot * DEG
        hr_dot = h * math.cos(roll * DEG) * roll_rate * DEG
        ve = v * math.sin(chi) + w * (a * c - hr * s) + hr_dot * c
        vn = v * math.cos(chi) + w * (-a * s - hr * c) - hr_dot * s
        t_us = int(round(t * 1e6))
        yaw = wrap360(psi - yaw_off + 0.005 * t + rnd.gauss(0, 0.02))
        rows.append((t_us, 'I', f"{yaw:.4f}", f"{roll + rnd.gauss(0, 0.05):.4f}", f"{rnd.gauss(0, 0.05):.4f}"))
        if k % 4 == 0:
            rows.append((t_us, 'W', f"{-delta + rnd.gauss(0, 0.1):.3f}"))   # inverted WAS sign
        if k % 10 == 0:
            track = wrap360(math.degrees(math.atan2(ve, vn)) + rnd.gauss(0, 0.3))
            vant = max(0.0, math.hypot(ve, vn) + rnd.gauss(0, 0.02))
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
        c = self.cfg
        self.delta = 0.0; self.delta_valid = False; self.delta_t = 0
        self.votes = [0, 0, 0, 0]; self.quad = 0; self.quad_valid = False
        if c['dual_heading_offset_deg'] != 999.0:
            self.quad = int(round(c['dual_heading_offset_deg'])); self.quad_valid = True
        self.was_pos = self.was_neg = 0; self.was_sign = 0.0; self.was_sign_valid = False
        if c['was_sign'] > 0: self.was_sign, self.was_sign_valid = 1.0, True
        elif c['was_sign'] < 0: self.was_sign, self.was_sign_valid = -1.0, True
        self.leff = 0.0; self.leff_valid = False; self.leff_n = 0; self.leff_t = 0
        self.k = 0.0; self.k_valid = False; self.k_n = 0; self.k_t = 0
        self.slip = 0.0; self.slip_valid = False; self.slip_t = 0
        self.yaw = self.roll = self.pitch = 0.0; self.yaw_rate = 0.0; self.roll_rate = 0.0
        self.imu_t = 0; self.imu_valid = False
        self.was = 0.0; self.was_valid = False
        self.last = None
        self.active = False
        self.reports = []

    def veh(self, hdg_raw):
        return wrap360(hdg_raw + self.quad)

    def imu_roll(self):
        return self.cfg['imu_roll_sign'] * self.roll

    def turning(self):
        if not (self.was_valid and self.was_sign_valid): return None
        d = self.was_sign * self.was
        if abs(d) > TURN_MIN_WAS and abs(self.yaw_rate) > TURN_MIN_RATE: return d
        return None

    def imu(self, t_us, yaw, roll, pitch):
        c = self.cfg
        if self.imu_valid:
            dt = (t_us - self.imu_t) * 1e-6
            if 0 < dt < 0.5:
                rate = c['imu_yaw_sign'] * wrap180(yaw - self.yaw) / dt
                alpha = dt / (dt + 0.05)
                self.yaw_rate += (rate - self.yaw_rate) * alpha
                rrate = c['imu_roll_sign'] * (roll - self.roll) / dt
                self.roll_rate += (rrate - self.roll_rate) * alpha
                if self.active:
                    self.psi = wrap360(c['imu_yaw_sign'] * yaw + self.delta_frozen)
                    rollc = c['imu_roll_sign'] * roll
                    if c['crab_model'] and self.k_valid:
                        self.beta_res -= self.beta_res * (dt / (RES_TAU + dt))
                        self.beta = self.k * math.sin(rollc * DEG) + self.beta_res
                    else:
                        self.beta = self.beta_res
                    d = self.turning()
                    if c['speed_observer'] and self.leff_valid and d is not None:
                        v_obs = abs(self.yaw_rate * DEG * self.leff / math.tan(d * DEG))
                        a_obs = dt / (dt + OBS_TAU)
                        dv = (v_obs - self.v) * a_obs
                        lim = OBS_MAX_ACC * dt
                        dv = max(-lim, min(lim, dv))
                        self.v += dv
                        self.v = max(0.0, min(OBS_MAX_RATIO * self.v_start, self.v))
                        self.obs_n += 1
                    self.int_n += 1
                    chi = (self.psi + self.beta) * DEG
                    self.n += self.v * math.cos(chi) * dt
                    self.e += self.v * math.sin(chi) * dt
                    self.dist += self.v * dt
            elif dt >= 0.5 and self.active:
                self.active = False
        self.yaw, self.roll, self.pitch = yaw, roll, pitch
        self.imu_t = t_us; self.imu_valid = True

    def was_in(self, deg):
        self.was = deg; self.was_valid = True

    def predict(self):
        c = self.cfg
        s, co = math.sin(self.psi * DEG), math.cos(self.psi * DEG)
        hr = c['antenna_height_m'] * math.sin(self.imu_roll() * DEG)
        e = self.e + c['antenna_fwd_m'] * s + hr * co
        n = self.n + c['antenna_fwd_m'] * co - hr * s
        return (self.lat0 + n / self.rm / DEG, self.lon0 + e / (self.rn * math.cos(self.lat0 * DEG)) / DEG)

    def antenna_lateral(self):
        c = self.cfg
        rollc = self.imu_roll() * DEG
        return (c['antenna_fwd_m'] * self.yaw_rate * DEG
                + c['antenna_height_m'] * math.cos(rollc) * self.roll_rate * DEG)

    def axle_speed(self, v_ant):
        c = self.cfg
        hr = c['antenna_height_m'] * math.sin(self.imu_roll() * DEG)
        lat = self.antenna_lateral()
        along2 = v_ant * v_ant - lat * lat
        along = math.sqrt(along2) if along2 > 0 else 0.0
        v = along + hr * self.yaw_rate * DEG
        return v if v > 0 else 0.0

    def slip_crab(self, hdg_raw, track, v):
        c = self.cfg
        beta_total = wrap180(track - self.veh(hdg_raw))
        hr = c['antenna_height_m'] * math.sin(self.imu_roll() * DEG)
        along = self.axle_speed(v) - hr * self.yaw_rate * DEG
        beta_kin = math.degrees(math.atan2(self.antenna_lateral(), along))
        return wrap180(beta_total - beta_kin)

    def gnss(self, t_ms, lat, lon, alt, posq, hdgq, hdg_raw, track, v, dual_roll):
        c = self.cfg
        if posq != 3:
            if self.active and self.last is not None and (t_ms - self.last[0]) > 1000:
                self.active = False
            return
        # quadrant
        if c['dual_heading_offset_deg'] == 999.0 and hdgq == 3 and v >= LEARN_MIN_V:
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
        # WAS sign + L_eff
        if (self.imu_valid and self.was_valid and v >= LEARN_MIN_V and
                abs(self.was) > TURN_MIN_WAS and abs(self.yaw_rate) > TURN_MIN_RATE):
            if c['was_sign'] == 0.0:
                same = (self.was > 0) == (self.yaw_rate > 0)
                if same: self.was_pos += 1
                else: self.was_neg += 1
                hi = max(self.was_pos, self.was_neg); total = self.was_pos + self.was_neg
                if hi >= WAS_MIN_VOTES and hi * 10 >= total * 8:
                    s = 1.0 if self.was_pos >= self.was_neg else -1.0
                    if not self.was_sign_valid or s != self.was_sign:
                        self.leff_valid = False; self.leff_n = 0
                    self.was_sign = s; self.was_sign_valid = True
            if self.was_sign_valid:
                d = self.was_sign * self.was
                l_obs = self.axle_speed(v) * math.tan(d * DEG) / (self.yaw_rate * DEG)
                L = c['wheelbase_m']
                if 0.3 * L < l_obs < 3.0 * L:
                    if self.leff_n == 0:
                        self.leff = l_obs
                    else:
                        dt = (t_ms - self.leff_t) * 1e-3
                        if dt <= 0 or dt > 2.0: dt = 0.1
                        self.leff += (l_obs - self.leff) * (dt / (dt + LEFF_TAU))
                    self.leff = max(0.5 * L, min(2.0 * L, self.leff))
                    self.leff_t = t_ms; self.leff_n += 1
                    if self.leff_n >= LEFF_MIN_N: self.leff_valid = True
        # low-passed slip crab
        if self.quad_valid and self.imu_valid and hdgq == 3 and v >= LEARN_MIN_V:
            raw = self.slip_crab(hdg_raw, track, v)
            if not self.slip_valid:
                self.slip = raw; self.slip_valid = True
            else:
                dt = (t_ms - self.slip_t) * 1e-3
                if dt <= 0 or dt > 2.0: dt = 0.1
                self.slip += wrap180(raw - self.slip) * (dt / (dt + SLIP_TAU))
            self.slip_t = t_ms
        # crab gain
        if self.slip_valid and hdgq == 3 and v >= LEARN_MIN_V:
            rollc = self.imu_roll()
            if abs(rollc) > CRAB_MIN_ROLL and abs(self.roll_rate) <= CRAB_MAX_ROLL_RATE:
                k_obs = self.slip / math.sin(rollc * DEG)
                k_obs = max(-K_LIMIT, min(K_LIMIT, k_obs))
                if self.k_n == 0:
                    self.k = k_obs
                else:
                    dt = (t_ms - self.k_t) * 1e-3
                    if dt <= 0 or dt > 2.0: dt = 0.1
                    self.k += (k_obs - self.k) * (dt / (dt + K_TAU))
                self.k_t = t_ms; self.k_n += 1
                if self.k_n >= K_MIN_N: self.k_valid = True
        if self.active:
            plat, plon = self.predict()
            dn = (lat - plat) * DEG * self.rm
            de = (lon - plon) * DEG * self.rn * math.cos(self.lat0 * DEG)
            s, co = math.sin(self.psi * DEG), math.cos(self.psi * DEG)
            self.along = de * s + dn * co; self.cross = de * co - dn * s
            self.max_along = max(self.max_along, abs(self.along)); self.max_cross = max(self.max_cross, abs(self.cross))
            self.count += 1
            if (t_ms - self.t_start) * 1e-3 >= c['shadow_window_s']:
                self.reports.append(dict(
                    duration_ms=t_ms - self.t_start, dist=self.dist, along=self.along, cross=self.cross,
                    max_along=self.max_along, max_cross=self.max_cross, delta=self.delta_frozen, offset=self.quad,
                    beta=self.beta_entry, v_start=self.v_start, v_end=v, fixes=self.count,
                    leff=self.leff if self.leff_valid else 0.0, k=self.k if self.k_valid else 0.0,
                    was_sign=self.was_sign if self.was_sign_valid else 0.0,
                    obs_frac=(self.obs_n / self.int_n) if self.int_n else 0.0))
                self.active = False
        self.last = (t_ms,)
        if (not self.active and self.delta_valid and self.quad_valid and self.imu_valid and v >= c['min_speed_mps']):
            self.delta_frozen = self.delta
            self.psi = wrap360(c['imu_yaw_sign'] * self.yaw + self.delta_frozen)
            self.v = self.axle_speed(v); self.v_start = self.v
            self.beta_entry = self.slip if self.slip_valid else self.slip_crab(hdg_raw, track, v)
            if c['crab_model'] and self.k_valid:
                self.beta_res = self.beta_entry - self.k * math.sin(self.imu_roll() * DEG)
            else:
                self.beta_res = self.beta_entry
            self.beta = self.beta_entry
            self.lat0, self.lon0 = lat, lon
            self.rm, self.rn = radii(lat)
            s, co = math.sin(self.psi * DEG), math.cos(self.psi * DEG)
            hr = c['antenna_height_m'] * math.sin(self.imu_roll() * DEG)
            self.e = -c['antenna_fwd_m'] * s - hr * co
            self.n = -c['antenna_fwd_m'] * co + hr * s
            self.dist = 0.0; self.t_start = t_ms
            self.along = self.cross = self.max_along = self.max_cross = 0.0; self.count = 0
            self.int_n = self.obs_n = 0
            self.active = True


def run(path, cfg=None):
    m = Coast(cfg or CFG)
    with open(path) as f:
        for line in f:
            p = line.strip().split(',')
            if len(p) < 2: continue
            t_us = int(p[0])
            if p[1] == 'I':
                m.imu(t_us, float(p[2]), float(p[3]), float(p[4]))
            elif p[1] == 'W':
                m.was_in(float(p[2]))
            elif p[1] == 'G':
                m.gnss(t_us // 1000, float(p[2]), float(p[3]), float(p[4]), int(p[5]), int(p[6]),
                       float(p[7]), float(p[8]), float(p[9]), float(p[10]))
    for r in m.reports:
        print("REPORT,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%.2f,%.3f,%.3f,%d,%.3f,%.3f,%.0f,%.3f" % (
            r['duration_ms'], r['dist'], r['along'], r['cross'], r['max_along'], r['max_cross'],
            r['delta'], r['offset'], r['beta'], r['v_start'], r['v_end'], r['fixes'],
            r['leff'], r['k'], r['was_sign'], r['obs_frac']))


if __name__ == '__main__':
    if sys.argv[1] == 'gen':
        gen(sys.argv[2])
    elif sys.argv[1] == 'run':
        cfg = dict(CFG)
        if '--no-observer' in sys.argv: cfg['speed_observer'] = False
        if '--no-crab' in sys.argv: cfg['crab_model'] = False
        run(sys.argv[2], cfg)
