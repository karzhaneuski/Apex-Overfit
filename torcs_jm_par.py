#!/usr/bin/env python3
"""
torcs_jm_par.py — Sensor-reactive TORCS driver.

A deliberately simple, robust controller: every command is computed fresh each
tick from the current sensors, with no track memory, no learned map, and no
corner-phase state machine. The car reads how far it can see straight ahead and
sets its target speed from that ("if the road is long and open, go fast; if a
wall is close, it's a corner — slow down"). Steering is a PD law on heading and
lateral drift; braking is proportional to overspeed with trail-off in corners.

Communication layer (ServerState / DriverAction / Client) is the standard
snakeoil3-derived SCR client.
"""

from __future__ import annotations

import getopt
import math
import os
import socket
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

PI: float      = math.pi
DATA_SIZE: int = 2 ** 17
VERSION: str   = "sensor-reactive-1"

# Track sensor beam geometry (must match the TORCS handshake init string).
SENSOR_ANGLES_DEG: List[float] = [
    -45, -19, -12, -7, -4, -2.5, -1.7, -1, -0.5,
      0,
     0.5,  1,  1.7, 2.5,  4,   7,   12, 19, 45,
]

_OPHELP = (
    "Options:\n"
    "  -H <host>   TORCS server hostname.          [localhost]\n"
    "  -p <port>   TORCS server UDP port.          [3001]\n"
    "  -i <id>     Bot identifier.                 [SCR]\n"
    "  -m <#>      Max simulation steps (50/sec).  [100000]\n"
    "  -e <#>      Max learning episodes.          [1]\n"
    "  -t <name>   Track name tag.                 [unknown]\n"
    "  -s <#>      Stage: 0=warm-up .. 3=unknown.  [3]\n"
    "  -d          Print telemetry each step.\n"
    "  -h          Show help.\n"
    "  -v          Print version.\n"
)
_USAGE = f"Usage: {sys.argv[0]} [options]\n{_OPHELP}"


# ===========================================================================
#  Tuning configuration
# ===========================================================================

@dataclass
class DriveConfig:
    """All tunable constants in one place. Tuned for car1-ow1 (rev limiter
    18700, 7 gears)."""

    # ── Speed target from forward visibility ────────────────────────────────
    # target = base + sight * sight_gain, then reduced by how hard we're turning.
    speed_base:      float = 75.0     # km/h floor even with a corner right ahead
    sight_gain:      float = 2.4      # km/h of target per metre of clear sight
    speed_ceiling:   float = 330.0    # never ask for more than this
    steer_speed_pen: float = 38.0     # km/h cut from target per unit |steer|
    open_road_angle: float = 0.05     # rad — below this & long sight = full send
    open_road_sight: float = 95.0    # m  — sight beyond this on a straight = max
    open_road_speed: float = 305.0    # km/h target when the road is clearly open

    # ── Throttle PD controller (on the speed error) ─────────────────────────
    throttle_kp:     float = 0.032
    throttle_kd:     float = 0.11
    creep_speed:     float = 14.0     # below this, just floor the throttle
    creep_throttle:  float = 1.15

    # ── Braking ─────────────────────────────────────────────────────────────
    brake_base_speed: float = 60.0    # safe speed floor for braking calc
    brake_sight_gain: float = 1.9     # added safe speed per metre of sight
    brake_strength:   float = 30.0    # divisor: overspeed/this = brake force
    trail_angle:      float = 0.8     # rad — above this, ease brakes (trail braking)
    trail_drop:       float = 5.5    # how fast trail-braking releases with angle
    trail_max_drop:   float = 0.8     # max fraction of brake removed in a corner

    # ── Anti-slide balance tap ──────────────────────────────────────────────
    slide_speed_y:   float = 13.0     # km/h lateral speed that triggers a dab
    slide_min_speed: float = 60.0
    slide_brake:     float = 0.15

    # ── Steering PD ─────────────────────────────────────────────────────────
    steer_gain:      float = 40.0     # proportional heading gain
    center_gain:     float = 0.8     # pull back toward track centre
    drift_gain:      float = 0.0003    # damp lateral velocity (speedY)
    steer_speed_soft: float = 180.0   # above this speed, soften steering

    # ── Gearbox (car1-ow1: limiter 18700, 7 gears) ──────────────────────────
    shift_up_rpm:    float = 18200.0
    shift_dn_rpm:    float = 7800.0
    gear_max:        int   = 7

    # ── Traction control ────────────────────────────────────────────────────
    tcs_enable:      bool  = False
    tcs_slip:        float = 2.0      # rear-vs-front spin difference that trips it
    tcs_cut:         float = 0.10     # throttle removed per trip

    track_sensor_angles: List[float] = field(
        default_factory=lambda: list(SENSOR_ANGLES_DEG)
    )
def clip(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def bargraph(x: float, mn: float, mx: float, w: int, c: str = "X") -> str:
    if not w:
        return ""
    x = clip(x, mn, mx)
    tx = mx - mn
    if tx <= 0:
        return "backwards"
    upw = tx / float(w)
    if upw <= 0:
        return "what?"
    negpu = negnonpu = pospu = posnonpu = 0.0
    if mn < 0:
        if x < 0:
            negpu    = -x + min(0.0, mx)
            negnonpu = -mn + x
        else:
            negnonpu = -mn + min(0.0, mx)
    if mx > 0:
        if x > 0:
            pospu    = x - max(0.0, mn)
            posnonpu = mx - x
        else:
            posnonpu = mx - max(0.0, mn)
    return "[{}]".format(
        int(negnonpu / upw) * "-" +
        int(negpu    / upw) * c   +
        int(pospu    / upw) * c   +
        int(posnonpu / upw) * "_"
    )


def destringify(s):
    if not s:
        return s
    if isinstance(s, str):
        try:
            return float(s)
        except ValueError:
            return s
    if isinstance(s, list):
        return destringify(s[0]) if len(s) < 2 else [destringify(i) for i in s]
    return s


# ===========================================================================
#  Server communication
# ===========================================================================
class ServerState:
    def __init__(self) -> None:
        self.servstr = ""
        self.d: Dict = {}

    def parse_server_str(self, s: str) -> None:
        self.servstr = s.strip()[:-1]
        for tok in self.servstr.strip().lstrip("(").rstrip(")").split(")("):
            p = tok.split(" ")
            self.d[p[0]] = destringify(p[1:])

    def __repr__(self) -> str:
        return self.fancyout()

    def fancyout(self) -> str:
        ORDER = [
            "stucktimer","fuel","distRaced","distFromStart","opponents",
            "wheelSpinVel","z","speedZ","speedY","speedX",
            "targetSpeed","rpm","skid","slip","track","trackPos","angle",
        ]
        out = ""
        for k in ORDER:
            val = self.d.get(k)
            if val is None:
                continue
            if isinstance(val, list):
                if k == "track":
                    r = ["%.1f" % x for x in val]
                    strout = " ".join(r[:9]) + "_" + r[9] + "_" + " ".join(r[10:])
                elif k == "opponents":
                    ch = []
                    for o in val:
                        if   o>190: ch.append("_")
                        elif o> 90: ch.append(".")
                        elif o> 39: ch.append(chr(int(o/2)+97-19))
                        elif o> 13: ch.append(chr(int(o)+65-13))
                        elif o>  3: ch.append(chr(int(o)+48-3))
                        else:       ch.append("?")
                    j = "".join(ch)
                    strout = f" -> {j[:18]} {j[18:]} <-"
                else:
                    strout = ", ".join(str(i) for i in val)
            else:
                if k=="speedX":
                    c="R" if val<0 else "X"
                    strout="%6.1f %s"%(val,bargraph(val,-30,300,50,c))
                elif k=="trackPos":
                    c=">" if val<0 else "<"
                    strout="%6.3f %s"%(val,bargraph(-val,-1,1,50,c))
                elif k=="angle":
                    deg=int(val*180/PI)
                    strout="%5.2f %3d deg"%(val,deg)
                elif k=="rpm":
                    g=self.d.get("gear",0)
                    gl="R" if g<0 else "%1d"%g
                    strout=bargraph(val,0,10000,50,gl)
                elif k=="stucktimer":
                    strout="Not stuck!" if not val else "%3d %s"%(val,bargraph(val,0,300,50,"'"))
                elif k=="fuel":
                    strout="%6.0f %s"%(val,bargraph(val,0,100,50,"f"))
                else:
                    strout=str(val)
            out += f"{k}: {strout}\n"
        return out


class DriverAction:
    def __init__(self) -> None:
        self.d: Dict = {
            "accel":  0.2,
            "brake":  0.0,
            "clutch": 0.0,
            "gear":   1,
            "steer":  0.0,
            "focus":  [-90,-45,0,45,90],
            "meta":   0,
        }

    def clip_to_limits(self) -> None:
        self.d["steer"]  = clip(self.d["steer"],  -1.0, 1.0)
        self.d["brake"]  = clip(self.d["brake"],   0.0, 1.0)
        self.d["accel"]  = clip(self.d["accel"],   0.0, 1.0)
        self.d["clutch"] = clip(self.d["clutch"],  0.0, 1.0)
        if self.d["gear"] not in range(-1,7):
            self.d["gear"] = 0
        if self.d["meta"] not in (0,1):
            self.d["meta"] = 0
        f = self.d["focus"]
        if not isinstance(f,list) or min(f)<-180 or max(f)>180:
            self.d["focus"] = 0

    def __repr__(self) -> str:
        self.clip_to_limits()
        parts = []
        for k,v in self.d.items():
            if isinstance(v,list):
                parts.append(f"({k} {' '.join(str(x) for x in v)})")
            else:
                parts.append(f"({k} {v:.3f})")
        return "".join(parts)

    def fancyout(self) -> str:
        out = ""
        od = {k:v for k,v in self.d.items() if k not in ("gear","meta","focus")}
        for k in sorted(od):
            v = od[k]
            if k in ("clutch","brake","accel"):
                strout = "%6.3f %s"%(v,bargraph(v,0,1,50,k[0].upper()))
            elif k=="steer":
                strout = "%6.3f %s"%(v,bargraph(-v,-1,1,50,"S"))
            else:
                strout = str(v)
            out += f"{k}: {strout}\n"
        return out


class Client:
    def __init__(
        self,
        H=None, p=None, i=None, e=None,
        t=None, s=None, d=None,
        vision: bool = False,
        params: Optional[DriveConfig] = None,
    ) -> None:
        self.vision = vision
        self.host = "localhost"; self.port = 3001
        self.sid  = "SCR"; self.maxEpisodes = 1
        self.trackname = "unknown"; self.stage = 2
        self.debug = False; self.maxSteps = 100_000
        self.params = params or DriveConfig()
        self._parse_cli()
        if H is not None: self.host        = H
        if p is not None: self.port        = p
        if i is not None: self.sid         = i
        if e is not None: self.maxEpisodes = e
        if t is not None: self.trackname   = t
        if s is not None: self.stage       = s
        if d is not None: self.debug       = d
        self.S = ServerState(); self.R = DriverAction()
        self.so: Optional[socket.socket] = None
        self._setup_connection()

    def _setup_connection(self) -> None:
        try:
            self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as exc:
            print(f"Cannot create socket: {exc}"); sys.exit(-1)
        self.so.settimeout(1)
        angles   = " ".join(str(a) for a in self.params.track_sensor_angles)
        init_msg = f"{self.sid}(init {angles})".encode()
        n_fail   = 5
        while True:
            try:
                self.so.sendto(init_msg, (self.host, self.port))
            except socket.error as exc:
                print(f"Cannot send handshake: {exc}"); sys.exit(-1)
            try:
                data, _ = self.so.recvfrom(DATA_SIZE)
                sock    = data.decode("utf-8")
            except socket.error:
                print(f"Waiting for server on port {self.port} … {max(n_fail,0)}")
                if n_fail < 0:
                    self._relaunch_torcs(); n_fail = 5
                n_fail -= 1; continue
            if "***identified***" in sock:
                print(f"Connected on port {self.port}."); break

    def _relaunch_torcs(self) -> None:
        print("Re-launching TORCS …")
        os.system("pkill torcs"); time.sleep(1.0)
        flags = "-nofuel -nodamage -nolaptime"
        if self.vision: flags += " -vision"
        os.system(f"torcs {flags} &"); time.sleep(1.0)
        os.system("sh autostart.sh")

    def _parse_cli(self) -> None:
        try:
            opts, args = getopt.getopt(sys.argv[1:],
                "H:p:i:m:e:t:s:dhv",
                ["host=","port=","id=","steps=","episodes=",
                 "track=","stage=","debug","help","version"])
        except getopt.error as exc:
            print(f"getopt error: {exc}\n{_USAGE}"); sys.exit(-1)
        try:
            for opt, val in opts:
                if   opt in ("-h","--help"):     print(_USAGE);             sys.exit(0)
                elif opt in ("-v","--version"):  print(f"{sys.argv[0]} {VERSION}"); sys.exit(0)
                elif opt in ("-d","--debug"):    self.debug       = True
                elif opt in ("-H","--host"):     self.host        = val
                elif opt in ("-i","--id"):       self.sid         = val
                elif opt in ("-t","--track"):    self.trackname   = val
                elif opt in ("-s","--stage"):    self.stage       = int(val)
                elif opt in ("-p","--port"):     self.port        = int(val)
                elif opt in ("-e","--episodes"): self.maxEpisodes = int(val)
                elif opt in ("-m","--steps"):    self.maxSteps    = int(val)
        except ValueError as exc:
            print(f"Bad parameter: {exc}\n{_USAGE}"); sys.exit(-1)

    def get_servers_input(self) -> None:
        if not self.so: return
        misses = 0
        while True:
            try:
                data, _ = self.so.recvfrom(DATA_SIZE)
                sock    = data.decode("utf-8")
            except socket.error:
                misses += 1
                print(".", end=" ", flush=True)
                if misses > 30:           # give up rather than spin forever
                    print(f"\nNo data on port {self.port}; shutting down.")
                    self.shutdown(); return
                continue
            if "***identified***" in sock:
                print(f"Connected on port {self.port}."); continue
            elif "***shutdown***"  in sock:
                print(f"Shutdown on port {self.port}."); self.shutdown(); return
            elif "***restart***"   in sock:
                print(f"Restart on port {self.port}."); self.shutdown(); return
            elif not sock:
                continue
            else:
                self.S.parse_server_str(sock)
                if self.debug:
                    sys.stderr.write("\x1b[2J\x1b[H"); print(self.S)
                return

    def respond_to_server(self) -> None:
        if not self.so: return
        try:
            self.so.sendto(repr(self.R).encode(), (self.host, self.port))
        except socket.error as exc:
            print(f"Send error: {exc}"); sys.exit(-1)
        if self.debug: print(self.R.fancyout())

    def shutdown(self) -> None:
        if not self.so: return
        print(f"Shutting down port {self.port}.")
        self.so.close(); self.so = None


# ===========================================================================
#  SENSOR-REACTIVE DRIVER
# ===========================================================================

class SensorDriver:
    """
    Computes steering / throttle / brake / gear each tick straight from sensors.
    No memory, no map — the forward range beams ARE the look-ahead: a long clear
    reading means a straight (go fast), a short one means a corner is close
    (slow down). This is what keeps it simple and stable.
    """

    def __init__(self, cfg: Optional[DriveConfig] = None) -> None:
        self.cfg = cfg or DriveConfig()
        self._prev_speed_err = 0.0
        self._prev_angle = 0.0
        self._steer_for_speed = 0.0
        self._apex_target = 0.0
        # telemetry logger (TORCS_LOG=1 -> auto file, or TORCS_LOG=path)
        env = os.environ.get("TORCS_LOG", "")
        self._log = None
        self._n = 0
        if env not in ("", "0", "false", "False"):
            fn = (env if env not in ("1", "true", "True")
                  else f"telemetry_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
            try:
                self._log = open(fn, "w")
                print(f"[log] telemetry -> {fn}", flush=True)
            except Exception as e:
                print(f"[!] log disabled: {e}")

    # ── forward visibility: how far the road is clear straight ahead ─────────
    @staticmethod
    def _sight(track: list) -> float:
        # The centre beams (indices 7..11 ≈ straight ahead) report distance to
        # the track edge in that direction. The largest of them is how far we can
        # see down the road — small when a corner wall is close.
        if not isinstance(track, list) or len(track) < 19:
            return 200.0
        return max(track[7], track[8], track[9], track[10], track[11])

    def _road_direction(self, track: list) -> float:
        """
        Estimate where the road is going from track sensors.
        Returns angle in radians.
        Negative/positive sign depends on TORCS sensor side.
        """
        if not isinstance(track, list) or len(track) < 19:
            return 0.0

        # Use forward beams, not extreme 45-degree side beams.
        # Indices 1..17 = -19 deg .. +19 deg.
        num = 0.0
        den = 0.0

        for i in range(1, 18):
            d = float(track[i])
            if d <= 0.0:
                continue

            # Cap distance so one huge beam does not dominate too much.
            d = min(d, 120.0)

            # Longer beam = road probably bends that way.
            w = d * d
            a = math.radians(SENSOR_ANGLES_DEG[i])

            num += a * w
            den += w

        if den <= 0.0:
            return 0.0

        return num / den

    def _steering(self, S: dict) -> float:
        c = self.cfg

        angle    = float(S.get("angle", 0.0) or 0.0)
        trackpos = float(S.get("trackPos", 0.0) or 0.0)
        speed    = float(S.get("speedX", 0.0) or 0.0)
        speed_y  = float(S.get("speedY", 0.0) or 0.0)

        soften = 1.0 + max(0.0, speed - c.steer_speed_soft) / 100.0

        # Original steering
        p = angle * (c.steer_gain / soften) / PI
        track = S.get("track")

        # Default: center line
        target_pos = 0.0

        if isinstance(track, list) and len(track) >= 19:
            left_open  = max(track[4], track[5], track[6], track[7], track[8])
            right_open = max(track[10], track[11], track[12], track[13], track[14])
            front = max(track[7], track[8], track[9], track[10], track[11])

            diff = right_open - left_open

            # Detect corner direction.
            # If it aims wrong side, invert the sign below.
            corner_dir = 0.0
            if abs(diff) > 14.0 and front < 140.0:
                corner_dir = 1.0 if diff > 0.0 else -1.0

            # Aim to inner side instead of center.
            # 0.55 = aggressive inner line, but not fully on edge.
            target_pos = -corner_dir * 0.55

            # Smooth target so it does not jump left/right.
            self._apex_target = 0.78 * self._apex_target + 0.42 * target_pos
        else:
            self._apex_target *= 0.80

        # Instead of pulling to center, pull to desired inner-side position.
        lateral_error = trackpos - self._apex_target
        centre = lateral_error * c.center_gain

        drift = speed_y * c.drift_gain

        base_steer = p - centre - drift

        # Save base steering for speed calculation.
        # We do NOT want the short turn-in boost to reduce target speed.
        self._steer_for_speed = base_steer

        # New: short boost only when angle is changing quickly.
        angle_rate = angle - self._prev_angle
        self._prev_angle = angle

        angle_rate = clip(angle_rate, -0.045, 0.045)

        if speed > 45.0:
            boost = 65.0

            track = S.get("track")

            if isinstance(track, list) and len(track) >= 19:
                # center/forward beams
                front = max(track[7], track[8], track[9], track[10], track[11])

                # left/right forward visibility
                left_open  = max(track[4], track[5], track[6], track[7], track[8])
                right_open = max(track[10], track[11], track[12], track[13], track[14])

                diff = abs(right_open - left_open)

                # More boost when the road shape is clearly asymmetric
                # and the forward visibility is not very long.
                corner_pressure = 0.0
                corner_pressure += clip(diff / 90.0, 0.0, 1.0)
                corner_pressure += clip((115.0 - front) / 80.0, 0.0, 1.0)

                corner_pressure = clip(corner_pressure, 0.0, 1.0)

                # Adaptive boost for all sharp/late corners, not one sector.
                boost *= 1.0 + 0.42 * corner_pressure

            turn_in = angle_rate * boost / PI
        else:
            turn_in = 0.0

        return clip(base_steer + turn_in, -1.0, 1.0)

    def _target_speed(self, S: dict, steer: float) -> float:
        c = self.cfg
        sight = self._sight(S.get("track"))
        angle = float(S.get("angle", 0.0) or 0.0)
        # Wide-open road (nearly straight + long sight): commit to top speed.
        if abs(angle) < c.open_road_angle and sight > c.open_road_sight:
            return c.open_road_speed
        tgt = c.speed_base + sight * c.sight_gain
        steer_for_speed = abs(getattr(self, "_steer_for_speed", steer))
        tgt -= steer_for_speed * c.steer_speed_pen
        return clip(tgt, 0.0, c.speed_ceiling)

    def _throttle(self, S: dict, target: float, braking: bool) -> float:
        c = self.cfg
        speed = float(S.get("speedX", 0.0) or 0.0)
        if speed < c.creep_speed:
            return c.creep_throttle
        if braking:
            return 0.0
        err = target - speed
        d_err = err - self._prev_speed_err
        self._prev_speed_err = err
        out = err * c.throttle_kp + d_err * c.throttle_kd
        return clip(out, 0.0, 1.0)

    def _brake(self, S: dict) -> float:
        c = self.cfg
        speed = float(S.get("speedX", 0.0) or 0.0)
        angle = float(S.get("angle", 0.0) or 0.0)
        speed_y = float(S.get("speedY", 0.0) or 0.0)
        sight = self._sight(S.get("track"))
        safe = c.brake_base_speed + sight * c.brake_sight_gain
        if speed > safe:
            force = (speed - safe) / c.brake_strength
            # Trail braking: bleed off the brake as the car turns in, so braking
            # and cornering don't fight for grip.
            if abs(angle) > c.trail_angle:
                damp = 1.0 - min(c.trail_max_drop, abs(angle) * c.trail_drop)
                force *= damp
            return clip(force, 0.0, 1.0)
        # Small stabilising dab if the car starts to slide sideways at speed.
        if abs(speed_y) > c.slide_speed_y and speed > c.slide_min_speed:
            return c.slide_brake
        return 0.0

    def _gear(self, S: dict) -> int:
        c = self.cfg
        gear = int(S.get("gear", 1) or 1)
        rpm  = float(S.get("rpm", 0.0) or 0.0)
        if gear <= 0:
            return 1
        if rpm > c.shift_up_rpm:
            return min(gear + 1, c.gear_max)
        if rpm < c.shift_dn_rpm and gear > 1:
            return gear - 1
        return gear

    def _tcs(self, S: dict, accel: float) -> float:
        c = self.cfg
        if not c.tcs_enable:
            return accel
        wsv = S.get("wheelSpinVel")
        if not isinstance(wsv, list) or len(wsv) < 4:
            return accel
        # rear wheels (2,3) spinning faster than front (0,1) => wheelspin
        if (wsv[2] + wsv[3]) - (wsv[0] + wsv[1]) > c.tcs_slip:
            accel -= c.tcs_cut
        return clip(accel, 0.0, 1.0)

    def drive(self, client: "Client") -> None:
        S, R = client.S.d, client.R.d
        steer = self._steering(S)
        R["steer"] = steer
        brake = self._brake(S)
        R["brake"] = brake
        target = self._target_speed(S, steer)
        accel = self._throttle(S, target, braking=(brake > 0.0))
        R["accel"] = self._tcs(S, accel)
        R["gear"]  = self._gear(S)
        R["clutch"] = 0.0
        R["meta"]  = 0
        self._log_tick(S, R, target)

    def _log_tick(self, S: dict, R: dict, target: float) -> None:
        if self._log is None:
            return
        try:
            import json
            def num(x, d=0.0):
                try: return float(x)
                except (TypeError, ValueError): return d
            rec = {
                "t": self._n,
                "lapTime": num(S.get("curLapTime")),
                "dist": round(num(S.get("distFromStart")), 1),
                "speedX": round(num(S.get("speedX")), 2),
                "speedY": round(num(S.get("speedY")), 2),
                "target": round(target, 1),
                "sight": round(self._sight(S.get("track")), 1),
                "rpm": round(num(S.get("rpm")), 0),
                "gear": int(num(S.get("gear"))),
                "trackPos": round(num(S.get("trackPos")), 4),
                "angle": round(num(S.get("angle")), 4),
                "damage": round(num(S.get("damage")), 0),
                "steer": round(num(R.get("steer")), 4),
                "accel": round(num(R.get("accel")), 3),
                "brake": round(num(R.get("brake")), 3),
            }
            self._log.write(json.dumps(rec, separators=(",", ":")) + "\n")
            self._n += 1
            if self._n % 25 == 0:
                self._log.flush()
        except Exception:
            pass

    def close_log(self) -> None:
        if self._log:
            try:
                self._log.flush(); self._log.close()
                print(f"[log] wrote {self._n} ticks", flush=True)
            except Exception:
                pass


# ===========================================================================
#  Main
# ===========================================================================

def main() -> None:
    cfg = DriveConfig()
    client = Client(params=cfg)
    driver = SensorDriver(cfg)
    for _ in range(client.maxSteps, 0, -1):
        client.get_servers_input()
        if client.so is None:
            break
        driver.drive(client)
        client.respond_to_server()
    driver.close_log()
    client.shutdown()


if __name__ == "__main__":
    main()